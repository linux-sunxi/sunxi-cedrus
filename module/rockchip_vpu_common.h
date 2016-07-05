/*
 * Rockchip VPU codec driver
 *
 * Copyright (C) 2014 Google, Inc.
 *	Tomasz Figa <tfiga@chromium.org>
 *
 * Based on s5p-mfc driver by Samsung Electronics Co., Ltd.
 *
 * Copyright (C) 2011 Samsung Electronics Co., Ltd.
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

#ifndef ROCKCHIP_VPU_COMMON_H_
#define ROCKCHIP_VPU_COMMON_H_

/* Enable debugging by default for now. */
#define DEBUG

#include <linux/platform_device.h>
#include <linux/videodev2.h>
#include <linux/wait.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>

#include "rockchip_vpu_hw.h"

#define ROCKCHIP_VPU_NAME		"rockchip-vpu"
#define ROCKCHIP_VPU_DEC_NAME		"rockchip-vpu-dec"

#define V4L2_CID_CUSTOM_BASE		(V4L2_CID_USER_BASE | 0x1000)

#define DST_QUEUE_OFF_BASE		(TASK_SIZE / 2)

#define ROCKCHIP_VPU_MAX_CTRLS		32

#define MB_DIM				16
#define MB_WIDTH(x_size)		DIV_ROUND_UP(x_size, MB_DIM)
#define MB_HEIGHT(y_size)		DIV_ROUND_UP(y_size, MB_DIM)

struct rockchip_vpu_ctx;
struct rockchip_vpu_codec_ops;

/**
 * struct rockchip_vpu_variant - information about VPU hardware variant
 *
 * @hw_id:		Top 16 bits (product ID) of hardware ID register.
 * @vpu_type:		Vpu type.
 * @name:		Vpu name.
 * @dec_offset:		Offset from VPU base to decoder registers.
 * @dec_reg_num:	Number of registers of decoder block.
 */
struct rockchip_vpu_variant {
	enum rockchip_vpu_type vpu_type;
	char *name;
	unsigned dec_offset;
	unsigned dec_reg_num;
};

/**
 * enum rockchip_vpu_codec_mode - codec operating mode.
 * @RK_VPU_CODEC_NONE:	No operating mode. Used for RAW video formats.
 * @RK3288_VPU_CODEC_VP8D:	Rk3288 VP8 decoder.
 * @RK3229_VPU_CODEC_VP8D:	Rk3229 VP8 decoder.
 */
enum rockchip_vpu_codec_mode {
	RK_VPU_CODEC_NONE = -1,
	RK3288_VPU_CODEC_VP8D,
	RK3229_VPU_CODEC_VP8D,
};

/**
 * enum rockchip_vpu_plane - indices of planes inside a VB2 buffer.
 * @PLANE_Y:		Plane containing luminance data (also denoted as Y).
 * @PLANE_CB_CR:	Plane containing interleaved chrominance data (also
 *			denoted as CbCr).
 * @PLANE_CB:		Plane containing CB part of chrominance data.
 * @PLANE_CR:		Plane containing CR part of chrominance data.
 */
enum rockchip_vpu_plane {
	PLANE_Y		= 0,
	PLANE_CB_CR	= 1,
	PLANE_CB	= 1,
	PLANE_CR	= 2,
};

/**
 * struct rockchip_vpu_buf - Private data related to each VB2 buffer.
 * @vb:			Pointer to related VB2 buffer.
 * @list:		List head for queuing in buffer queue.
 * @flags:		Buffer state. See enum rockchip_vpu_buf_flags.
 */
struct rockchip_vpu_buf {
	struct vb2_v4l2_buffer b;
	struct list_head list;
	/* Mode-specific data. */
};

/**
 * enum rockchip_vpu_state - bitwise flags indicating hardware state.
 * @VPU_RUNNING:	The hardware has been programmed for operation
 *			and is running at the moment.
 * @VPU_SUSPENDED:	System is entering sleep state and no more runs
 *			should be executed on hardware.
 */
enum rockchip_vpu_state {
	VPU_RUNNING	= BIT(0),
	VPU_SUSPENDED	= BIT(1),
};

/**
 * struct rockchip_vpu_dev - driver data
 * @v4l2_dev:		V4L2 device to register video devices for.
 * @vfd_dec:		Video device for decoder.
 * @pdev:		Pointer to VPU platform device.
 * @dev:		Pointer to device for convenient logging using
 *			dev_ macros.
 * @alloc_ctx:		VB2 allocator context
 *			(for allocations without kernel mapping).
 * @alloc_ctx_vm:	VB2 allocator context
 *			(for allocations with kernel mapping).
 * @aclk:		Handle of ACLK clock.
 * @hclk:		Handle of HCLK clock.
 * @base:		Mapped address of VPU registers.
 * @dec_base:		Mapped address of VPU decoder register for convenience.
 * @mapping:		DMA IOMMU mapping.
 * @vpu_mutex:		Mutex to synchronize V4L2 calls.
 * @irqlock:		Spinlock to synchronize access to data structures
 *			shared with interrupt handlers.
 * @state:		Device state.
 * @ready_ctxs:		List of contexts ready to run.
 * @variant:		Hardware variant-specific parameters.
 * @current_ctx:	Context being currently processed by hardware.
 * @run_wq:		Wait queue to wait for run completion.
 * @watchdog_work:	Delayed work for hardware timeout handling.
 */
struct rockchip_vpu_dev {
	struct v4l2_device v4l2_dev;
	struct video_device *vfd_dec;
	struct platform_device *pdev;
	struct device *dev;
	void *alloc_ctx;
	void *alloc_ctx_vm;
	struct clk *aclk;
	struct clk *hclk;
	void __iomem *base;
	void __iomem *dec_base;
	struct dma_iommu_mapping *mapping;

	struct mutex vpu_mutex; /* video_device lock */
	spinlock_t irqlock;
	unsigned long state;
	struct list_head ready_ctxs;
	const struct rockchip_vpu_variant *variant;
	struct rockchip_vpu_ctx *current_ctx;
	wait_queue_head_t run_wq;
	struct delayed_work watchdog_work;
};

/**
 * struct rockchip_vpu_run_ops - per context operations on run data.
 * @prepare_run:	Called when the context was selected for running
 *			to prepare operating mode specific data.
 * @run_done:		Called when hardware completed the run to collect
 *			operating mode specific data from hardware and
 *			finalize the processing.
 */
struct rockchip_vpu_run_ops {
	void (*prepare_run)(struct rockchip_vpu_ctx *);
	void (*run_done)(struct rockchip_vpu_ctx *, enum vb2_buffer_state);
};

/**
 * struct rockchip_vpu_vp8d_run - per-run data specific to VP8
 * decoding.
 * @frame_hdr: Pointer to a buffer containing per-run frame data which
 *			is needed by setting vpu register.
 */
struct rockchip_vpu_vp8d_run {
	const struct v4l2_ctrl_vp8_frame_hdr *frame_hdr;
};

/**
 * struct rockchip_vpu_run - per-run data for hardware code.
 * @src:		Source buffer to be processed.
 * @dst:		Destination buffer to be processed.
 * @priv_src:		Hardware private source buffer.
 * @priv_dst:		Hardware private destination buffer.
 */
struct rockchip_vpu_run {
	/* Generic for more than one operating mode. */
	struct rockchip_vpu_buf *src;
	struct rockchip_vpu_buf *dst;

	struct rockchip_vpu_aux_buf priv_src;
	struct rockchip_vpu_aux_buf priv_dst;

	/* Specific for particular operating modes. */
	union {
		struct rockchip_vpu_vp8d_run vp8d;
		/* Other modes will need different data. */
	};
};

/**
 * struct rockchip_vpu_ctx - Context (instance) private data.
 *
 * @dev:		VPU driver data to which the context belongs.
 * @fh:			V4L2 file handler.
 *
 * @vpu_src_fmt:	Descriptor of active source format.
 * @src_fmt:		V4L2 pixel format of active source format.
 * @vpu_dst_fmt:	Descriptor of active destination format.
 * @dst_fmt:		V4L2 pixel format of active destination format.
 *
 * @vq_src:		Videobuf2 source queue.
 * @src_queue:		Internal source buffer queue.
 * @src_crop:		Configured source crop rectangle (encoder-only).
 * @vq_dst:		Videobuf2 destination queue
 * @dst_queue:		Internal destination buffer queue.
 * @dst_bufs:		Private buffers wrapping VB2 buffers (destination).
 *
 * @ctrls:		Array containing pointer to registered controls.
 * @ctrl_handler:	Control handler used to register controls.
 * @num_ctrls:		Number of registered controls.
 *
 * @list:		List head for queue of ready contexts.
 *
 * @run:		Structure containing data about currently scheduled
 *			processing run.
 * @run_ops:		Set of operations related to currently scheduled run.
 * @hw:			Structure containing hardware-related context.
 */
struct rockchip_vpu_ctx {
	struct rockchip_vpu_dev *dev;
	struct v4l2_fh fh;

	/* Format info */
	struct rockchip_vpu_fmt *vpu_src_fmt;
	struct v4l2_pix_format_mplane src_fmt;
	struct rockchip_vpu_fmt *vpu_dst_fmt;
	struct v4l2_pix_format_mplane dst_fmt;

	/* VB2 queue data */
	struct vb2_queue vq_src;
	struct list_head src_queue;
	struct v4l2_rect src_crop;
	struct vb2_queue vq_dst;
	struct list_head dst_queue;
	struct vb2_buffer *dst_bufs[VIDEO_MAX_FRAME];

	/* Controls */
	struct v4l2_ctrl *ctrls[ROCKCHIP_VPU_MAX_CTRLS];
	struct v4l2_ctrl_handler ctrl_handler;
	unsigned num_ctrls;

	/* Various runtime data */
	struct list_head list;

	struct rockchip_vpu_run run;
	const struct rockchip_vpu_run_ops *run_ops;
	struct rockchip_vpu_hw_ctx hw;
};

/**
 * struct rockchip_vpu_fmt - information about supported video formats.
 * @name:	Human readable name of the format.
 * @fourcc:	FourCC code of the format. See V4L2_PIX_FMT_*.
 * @vpu_type:	Vpu_type;
 * @codec_mode:	Codec mode related to this format. See
 *		enum rockchip_vpu_codec_mode.
 * @num_planes:	Number of planes used by this format.
 * @depth:	Depth of each plane in bits per pixel.
 */
struct rockchip_vpu_fmt {
	char *name;
	u32 fourcc;
	enum rockchip_vpu_type vpu_type;
	enum rockchip_vpu_codec_mode codec_mode;
	int num_planes;
	u8 depth[VIDEO_MAX_PLANES];
};

/**
 * struct rockchip_vpu_control - information about controls to be registered.
 * @id:			Control ID.
 * @type:		Type of the control.
 * @name:		Human readable name of the control.
 * @minimum:		Minimum value of the control.
 * @maximum:		Maximum value of the control.
 * @step:		Control value increase step.
 * @menu_skip_mask:	Mask of invalid menu positions.
 * @default_value:	Initial value of the control.
 * @max_reqs:		Maximum number of configration request.
 * @dims:		Size of each dimension of compound control.
 * @elem_size:		Size of individual element of compound control.
 * @is_volatile:	Control is volatile.
 * @is_read_only:	Control is read-only.
 * @can_store:		Control uses configuration stores.
 *
 * See also struct v4l2_ctrl_config.
 */
struct rockchip_vpu_control {
	u32 id;

	enum v4l2_ctrl_type type;
	const char *name;
	s32 minimum;
	s32 maximum;
	s32 step;
	u32 menu_skip_mask;
	s32 default_value;
	s32 max_reqs;
	u32 dims[V4L2_CTRL_MAX_DIMS];
	u32 elem_size;

	bool is_volatile:1;
	bool is_read_only:1;
	bool can_store:1;
};

/* Logging helpers */

/**
 * debug - Module parameter to control level of debugging messages.
 *
 * Level of debugging messages can be controlled by bits of module parameter
 * called "debug". Meaning of particular bits is as follows:
 *
 * bit 0 - global information: mode, size, init, release
 * bit 1 - each run start/result information
 * bit 2 - contents of small controls from userspace
 * bit 3 - contents of big controls from userspace
 * bit 4 - detail fmt, ctrl, buffer q/dq information
 * bit 5 - detail function enter/leave trace information
 * bit 6 - register write/read information
 */
extern int debug;

#define vpu_debug(level, fmt, args...)				\
	do {							\
		if (debug & BIT(level))				\
			pr_err("%s:%d: " fmt,	                \
			       __func__, __LINE__, ##args);	\
	} while (0)

#define vpu_debug_enter()	vpu_debug(5, "enter\n")
#define vpu_debug_leave()	vpu_debug(5, "leave\n")

#define vpu_err(fmt, args...)					\
	pr_err("%s:%d: " fmt, __func__, __LINE__, ##args)

static inline char *fmt2str(u32 fmt, char *str)
{
	char a = fmt & 0xFF;
	char b = (fmt >> 8) & 0xFF;
	char c = (fmt >> 16) & 0xFF;
	char d = (fmt >> 24) & 0xFF;

	sprintf(str, "%c%c%c%c", a, b, c, d);

	return str;
}

/* Structure access helpers. */
static inline struct rockchip_vpu_ctx *fh_to_ctx(struct v4l2_fh *fh)
{
	return container_of(fh, struct rockchip_vpu_ctx, fh);
}

static inline struct rockchip_vpu_ctx *ctrl_to_ctx(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler,
			    struct rockchip_vpu_ctx, ctrl_handler);
}

static inline struct rockchip_vpu_buf *vb_to_buf(struct vb2_buffer *vb)
{
	return container_of(to_vb2_v4l2_buffer(vb), struct rockchip_vpu_buf, b);
}

static inline bool rockchip_vpu_ctx_is_encoder(struct rockchip_vpu_ctx *ctx)
{
	return ctx->vpu_dst_fmt->codec_mode != RK_VPU_CODEC_NONE;
}

int rockchip_vpu_ctrls_setup(struct rockchip_vpu_ctx *ctx,
			     const struct v4l2_ctrl_ops *ctrl_ops,
			     struct rockchip_vpu_control *controls,
			     unsigned num_ctrls,
			     const char* const* (*get_menu)(u32));
void rockchip_vpu_ctrls_delete(struct rockchip_vpu_ctx *ctx);

void rockchip_vpu_try_context(struct rockchip_vpu_dev *dev,
			      struct rockchip_vpu_ctx *ctx);

void rockchip_vpu_run_done(struct rockchip_vpu_ctx *ctx,
			   enum vb2_buffer_state result);

int rockchip_vpu_aux_buf_alloc(struct rockchip_vpu_dev *vpu,
			       struct rockchip_vpu_aux_buf *buf, size_t size);
void rockchip_vpu_aux_buf_free(struct rockchip_vpu_dev *vpu,
			       struct rockchip_vpu_aux_buf *buf);

static inline void vdpu_write_relaxed(struct rockchip_vpu_dev *vpu,
				      u32 val, u32 reg)
{
	vpu_debug(6, "MARK: set reg[%03d]: %08x\n", reg / 4, val);
	writel_relaxed(val, vpu->dec_base + reg);
}

static inline void vdpu_write(struct rockchip_vpu_dev *vpu, u32 val, u32 reg)
{
	vpu_debug(6, "MARK: set reg[%03d]: %08x\n", reg / 4, val);
	writel(val, vpu->dec_base + reg);
}

static inline u32 vdpu_read(struct rockchip_vpu_dev *vpu, u32 reg)
{
	u32 val = readl(vpu->dec_base + reg);

	vpu_debug(6, "MARK: get reg[%03d]: %08x\n", reg / 4, val);
	return val;
}

int rockchip_vpu_write(const char *file, void *buf, size_t size);

#endif /* ROCKCHIP_VPU_COMMON_H_ */
