/*
 * Rockchip VPU codec driver
 *
 * Copyright (C) 2014 Google, Inc.
 *	Tomasz Figa <tfiga@chromium.org>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "rockchip_vpu_common.h"

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_platform.h>

#include <asm/dma-iommu.h>

/**
 * struct rockchip_vpu_codec_ops - codec mode specific operations
 *
 * @init:	Prepare for streaming. Called from VB2 .start_streaming()
 *		when streaming from both queues is being enabled.
 * @exit:	Clean-up after streaming. Called from VB2 .stop_streaming()
 *		when streaming from first of both enabled queues is being
 *		disabled.
 * @run:	Start single {en,de)coding run. Called from non-atomic context
 *		to indicate that a pair of buffers is ready and the hardware
 *		should be programmed and started.
 * @done:	Read back processing results and additional data from hardware.
 * @reset:	Reset the hardware in case of a timeout.
 */
struct rockchip_vpu_codec_ops {
	int (*init)(struct rockchip_vpu_ctx *);
	void (*exit)(struct rockchip_vpu_ctx *);

	int (*irq)(int, struct rockchip_vpu_dev *);
	void (*run)(struct rockchip_vpu_ctx *);
	void (*done)(struct rockchip_vpu_ctx *, enum vb2_buffer_state);
	void (*reset)(struct rockchip_vpu_ctx *);
};

/*
 * Hardware control routines.
 */

void rockchip_vpu_power_on(struct rockchip_vpu_dev *vpu)
{
	vpu_debug_enter();

	/* TODO: Clock gating. */

	pm_runtime_get_sync(vpu->dev);

	vpu_debug_leave();
}

static void rockchip_vpu_power_off(struct rockchip_vpu_dev *vpu)
{
	vpu_debug_enter();

	pm_runtime_mark_last_busy(vpu->dev);
	pm_runtime_put_autosuspend(vpu->dev);

	/* TODO: Clock gating. */

	vpu_debug_leave();
}

/*
 * Interrupt handlers.
 */

static irqreturn_t vdpu_irq(int irq, void *dev_id)
{
	struct rockchip_vpu_dev *vpu = dev_id;
	struct rockchip_vpu_ctx *ctx = vpu->current_ctx;

	if (!ctx->hw.codec_ops->irq(irq, vpu)) {
		rockchip_vpu_power_off(vpu);
		cancel_delayed_work(&vpu->watchdog_work);

		ctx->hw.codec_ops->done(ctx, VB2_BUF_STATE_DONE);
	}

	return IRQ_HANDLED;
}

static void rockchip_vpu_watchdog(struct work_struct *work)
{
	struct rockchip_vpu_dev *vpu = container_of(to_delayed_work(work),
					struct rockchip_vpu_dev, watchdog_work);
	struct rockchip_vpu_ctx *ctx = vpu->current_ctx;
	unsigned long flags;

	spin_lock_irqsave(&vpu->irqlock, flags);

	ctx->hw.codec_ops->reset(ctx);

	spin_unlock_irqrestore(&vpu->irqlock, flags);

	vpu_err("frame processing timed out!\n");

	rockchip_vpu_power_off(vpu);
	ctx->hw.codec_ops->done(ctx, VB2_BUF_STATE_ERROR);
}

/*
 * Initialization/clean-up.
 */

#if defined(CONFIG_ROCKCHIP_IOMMU)
static int rockchip_vpu_iommu_init(struct rockchip_vpu_dev *vpu)
{
	int ret;

	vpu->mapping = arm_iommu_create_mapping(&platform_bus_type,
						0x10000000, SZ_2G);
	if (IS_ERR(vpu->mapping)) {
		ret = PTR_ERR(vpu->mapping);
		return ret;
	}

	vpu->dev->dma_parms = devm_kzalloc(vpu->dev,
				sizeof(*vpu->dev->dma_parms), GFP_KERNEL);
	if (!vpu->dev->dma_parms)
		goto err_release_mapping;

	dma_set_max_seg_size(vpu->dev, 0xffffffffu);

	ret = arm_iommu_attach_device(vpu->dev, vpu->mapping);
	if (ret)
		goto err_release_mapping;

	return 0;

err_release_mapping:
	arm_iommu_release_mapping(vpu->mapping);

	return ret;
}

static void rockchip_vpu_iommu_cleanup(struct rockchip_vpu_dev *vpu)
{
	arm_iommu_detach_device(vpu->dev);
	arm_iommu_release_mapping(vpu->mapping);
}
#else
static inline int rockchip_vpu_iommu_init(struct rockchip_vpu_dev *vpu)
{
	return 0;
}

static inline void rockchip_vpu_iommu_cleanup(struct rockchip_vpu_dev *vpu) { }
#endif

int rockchip_vpu_hw_probe(struct rockchip_vpu_dev *vpu)
{
	struct resource *res;
	int irq_dec;
	int ret;

	pr_info("probe device %s\n", dev_name(vpu->dev));

	INIT_DELAYED_WORK(&vpu->watchdog_work, rockchip_vpu_watchdog);

	vpu->aclk = devm_clk_get(vpu->dev, "aclk");
	if (IS_ERR(vpu->aclk)) {
		dev_err(vpu->dev, "failed to get aclk\n");
		return PTR_ERR(vpu->aclk);
	}

	vpu->hclk = devm_clk_get(vpu->dev, "hclk");
	if (IS_ERR(vpu->hclk)) {
		dev_err(vpu->dev, "failed to get hclk\n");
		return PTR_ERR(vpu->hclk);
	}

	/*
	 * Bump ACLK to max. possible freq. (400 MHz) to improve performance.
	 */
	clk_set_rate(vpu->aclk, 400*1000*1000);

	res = platform_get_resource(vpu->pdev, IORESOURCE_MEM, 0);
	vpu->base = devm_ioremap_resource(vpu->dev, res);
	if (IS_ERR(vpu->base))
		return PTR_ERR(vpu->base);

	clk_prepare_enable(vpu->aclk);
	clk_prepare_enable(vpu->hclk);

	vpu->dec_base = vpu->base + vpu->variant->dec_offset;

	ret = dma_set_coherent_mask(vpu->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(vpu->dev, "could not set DMA coherent mask\n");
		goto err_power;
	}

	ret = rockchip_vpu_iommu_init(vpu);
	if (ret)
		goto err_power;

	irq_dec = platform_get_irq_byname(vpu->pdev, "vdpu");
	if (irq_dec <= 0) {
		dev_err(vpu->dev, "could not get vdpu IRQ\n");
		ret = -ENXIO;
		goto err_iommu;
	}

	ret = devm_request_threaded_irq(vpu->dev, irq_dec, NULL, vdpu_irq,
					IRQF_ONESHOT, dev_name(vpu->dev), vpu);
	if (ret) {
		dev_err(vpu->dev, "could not request vdpu IRQ\n");
		goto err_iommu;
	}

	pm_runtime_set_autosuspend_delay(vpu->dev, 100);
	pm_runtime_use_autosuspend(vpu->dev);
	pm_runtime_enable(vpu->dev);

	return 0;

err_iommu:
	rockchip_vpu_iommu_cleanup(vpu);
err_power:
	clk_disable_unprepare(vpu->hclk);
	clk_disable_unprepare(vpu->aclk);

	return ret;
}

void rockchip_vpu_hw_remove(struct rockchip_vpu_dev *vpu)
{
	rockchip_vpu_iommu_cleanup(vpu);

	pm_runtime_disable(vpu->dev);

	clk_disable_unprepare(vpu->hclk);
	clk_disable_unprepare(vpu->aclk);
}

static const struct rockchip_vpu_codec_ops mode_ops[] = {
	[RK3288_VPU_CODEC_VP8D] = {
		.init = rockchip_vpu_vp8d_init,
		.exit = rockchip_vpu_vp8d_exit,
		.irq = rockchip_vdpu_irq,
		.run = rockchip_vpu_vp8d_run,
		.done = rockchip_vpu_run_done,
		.reset = rockchip_vpu_dec_reset,
	},
	[RK3229_VPU_CODEC_VP8D] = {
		.init = rockchip_vpu_vp8d_init,
		.exit = rockchip_vpu_vp8d_exit,
		.irq = rockchip_vdpu_irq,
		.run = rockchip_vpu_vp8d_run,
		.done = rockchip_vpu_run_done,
		.reset = rockchip_vpu_dec_reset,
	},
};

void rockchip_vpu_run(struct rockchip_vpu_ctx *ctx)
{
	ctx->hw.codec_ops->run(ctx);
}

int rockchip_vpu_init(struct rockchip_vpu_ctx *ctx)
{
	enum rockchip_vpu_codec_mode codec_mode;

	codec_mode = ctx->vpu_src_fmt->codec_mode; /* Decoder */

	ctx->hw.codec_ops = &mode_ops[codec_mode];

	return ctx->hw.codec_ops->init(ctx);
}

void rockchip_vpu_deinit(struct rockchip_vpu_ctx *ctx)
{
	ctx->hw.codec_ops->exit(ctx);
}
