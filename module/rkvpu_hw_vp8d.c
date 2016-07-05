/*
 * Rockchip VPU codec vp8 decode driver
 *
 * Copyright (C) 2014 Rockchip Electronics Co., Ltd.
 *	ZhiChao Yu <zhichao.yu@rock-chips.com>
 *
 * Copyright (C) 2014 Google, Inc.
 *      Tomasz Figa <tfiga@chromium.org>
 *
 * Copyright (C) 2015 Rockchip Electronics Co., Ltd.
 *      Alpha Lin <alpha.lin@rock-chips.com>
 *
 * Copyright (C) 2016 Rockchip Electronics Co., Ltd.
 *      Jung Zhao <jung.zhao@rock-chips.com>
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

#include "rockchip_vpu_hw.h"
#include "rockchip_vp8d_regs.h"
#include "rockchip_vpu_common.h"
#ifdef NO_BOILERPLATE_CLEANUP

#define RK_MAX_REGS_NUMS	256
#define DEC_8190_ALIGN_MASK	0x07U

static u32 *rockchip_regs_map;
static u32 (*rockchip_regs_table)[3];
static u32 rk_regs_value[RK_MAX_REGS_NUMS];

#define RK_GET_REG_BASE(x) \
	(rockchip_regs_table[rockchip_regs_map[(x)]][0])

#define RK_GET_REG_BITS_MASK(x) \
	(rockchip_regs_table[rockchip_regs_map[(x)]][1])

#define RK_GET_REG_BITS_OFFSET(x) \
	(rockchip_regs_table[rockchip_regs_map[(x)]][2])

/*
 * probs table with packed
 */
struct vp8_prob_tbl_packed {
	u8 prob_mb_skip_false;
	u8 prob_intra;
	u8 prob_ref_last;
	u8 prob_ref_golden;
	u8 prob_segment[3];
	u8 packed0;

	u8 prob_luma_16x16_pred_mode[4];
	u8 prob_chroma_pred_mode[3];
	u8 packed1;

	/* mv prob */
	u8 prob_mv_context[2][19];
	u8 packed2[2];

	/* coeff probs */
	u8 prob_coeffs[4][8][3][11];
	u8 packed3[96];
};

/*
 * filter taps taken to 7-bit precision,
 * reference RFC6386#Page-16, filters[8][6]
 */
static const u32 vp8d_mc_filter[8][6] = {
	{ 0, 0, 128, 0, 0, 0 },
	{ 0, -6, 123, 12, -1, 0 },
	{ 2, -11, 108, 36, -8, 1 },
	{ 0, -9, 93, 50, -6, 0 },
	{ 3, -16, 77, 77, -16, 3 },
	{ 0, -6, 50, 93, -9, 0 },
	{ 1, -8, 36, 108, -11, 2 },
	{ 0, -1, 12, 123, -6, 0 }
};

/* dump hw params for debug */
#ifdef DEBUG
static void rockchip_vp8d_dump_hdr(struct rockchip_vpu_ctx *ctx)
{
	const struct v4l2_ctrl_vp8_frame_hdr *hdr = ctx->run.vp8d.frame_hdr;
	int dct_total_len = 0;
	int i;

	vpu_debug(4, "Frame tag: key_frame=0x%02x, version=0x%02x\n",
		  !hdr->key_frame, hdr->version);

	vpu_debug(4, "Picture size: w=%d, h=%d\n", hdr->width, hdr->height);

	/* stream addresses */
	vpu_debug(4, "Addresses: segmap=0x%x, probs=0x%x\n",
		  ctx->hw.vp8d.segment_map.dma,
		  ctx->hw.vp8d.prob_tbl.dma);

	/* reference frame info */
	vpu_debug(4, "Ref frame: last=%d, golden=%d, alt=%d\n",
		  hdr->last_frame, hdr->golden_frame, hdr->alt_frame);

	/* bool decoder info */
	vpu_debug(4, "Bool decoder: range=0x%x, value=0x%x, count=0x%x\n",
		  hdr->bool_dec_range, hdr->bool_dec_value,
		  hdr->bool_dec_count);

	/* control partition info */
	vpu_debug(4, "Control Part: offset=0x%x, size=0x%x\n",
		  hdr->first_part_offset, hdr->first_part_size);
	vpu_debug(2, "Macroblock Data: bits_offset=0x%x\n",
		  hdr->macroblock_bit_offset);

	/* dct partition info */
	for (i = 0; i < hdr->num_dct_parts; i++) {
		dct_total_len += hdr->dct_part_sizes[i];
		vpu_debug(4, "Dct Part%d Size: 0x%x\n",
			  i, hdr->dct_part_sizes[i]);
	}

	dct_total_len += (hdr->num_dct_parts - 1) * 3;
	vpu_debug(4, "Dct Part Total Length: 0x%x\n", dct_total_len);
}
#else
static inline void rockchip_vp8d_dump_hdr(struct rockchip_vpu_ctx *ctx) { }
#endif

static void vp8d_write_regs_value(u32 index, u32 value, char *name)
{
	vpu_debug(6, "rk_regs_value[ %s:%03d ]=%08x\n", name, index, value);
	rk_regs_value[index] = value;
}
static void rockchip_vp8d_prob_update(struct rockchip_vpu_ctx *ctx)
{
	const struct v4l2_ctrl_vp8_frame_hdr *hdr = ctx->run.vp8d.frame_hdr;
	const struct v4l2_vp8_entropy_hdr *entropy_hdr = &hdr->entropy_hdr;
	u32 i, j, k;
	u8 *dst;

	/* first probs */
	dst = ctx->hw.vp8d.prob_tbl.cpu;

	dst[0] = hdr->prob_skip_false;
	dst[1] = hdr->prob_intra;
	dst[2] = hdr->prob_last;
	dst[3] = hdr->prob_gf;
	dst[4] = hdr->sgmnt_hdr.segment_probs[0];
	dst[5] = hdr->sgmnt_hdr.segment_probs[1];
	dst[6] = hdr->sgmnt_hdr.segment_probs[2];
	dst[7] = 0;

	dst += 8;
	dst[0] = entropy_hdr->y_mode_probs[0];
	dst[1] = entropy_hdr->y_mode_probs[1];
	dst[2] = entropy_hdr->y_mode_probs[2];
	dst[3] = entropy_hdr->y_mode_probs[3];
	dst[4] = entropy_hdr->uv_mode_probs[0];
	dst[5] = entropy_hdr->uv_mode_probs[1];
	dst[6] = entropy_hdr->uv_mode_probs[2];
	dst[7] = 0; /*unused */

	/* mv probs */
	dst += 8;
	dst[0] = entropy_hdr->mv_probs[0][0]; /* is short */
	dst[1] = entropy_hdr->mv_probs[1][0];
	dst[2] = entropy_hdr->mv_probs[0][1]; /* sign */
	dst[3] = entropy_hdr->mv_probs[1][1];
	dst[4] = entropy_hdr->mv_probs[0][8 + 9];
	dst[5] = entropy_hdr->mv_probs[0][9 + 9];
	dst[6] = entropy_hdr->mv_probs[1][8 + 9];
	dst[7] = entropy_hdr->mv_probs[1][9 + 9];
	dst += 8;
	for (i = 0; i < 2; ++i) {
		for (j = 0; j < 8; j += 4) {
			dst[0] = entropy_hdr->mv_probs[i][j + 9 + 0];
			dst[1] = entropy_hdr->mv_probs[i][j + 9 + 1];
			dst[2] = entropy_hdr->mv_probs[i][j + 9 + 2];
			dst[3] = entropy_hdr->mv_probs[i][j + 9 + 3];
			dst += 4;
		}
	}
	for (i = 0; i < 2; ++i) {
		dst[0] = entropy_hdr->mv_probs[i][0 + 2];
		dst[1] = entropy_hdr->mv_probs[i][1 + 2];
		dst[2] = entropy_hdr->mv_probs[i][2 + 2];
		dst[3] = entropy_hdr->mv_probs[i][3 + 2];
		dst[4] = entropy_hdr->mv_probs[i][4 + 2];
		dst[5] = entropy_hdr->mv_probs[i][5 + 2];
		dst[6] = entropy_hdr->mv_probs[i][6 + 2];
		dst[7] = 0; /*unused */
		dst += 8;
	}

	/* coeff probs (header part) */
	dst = ctx->hw.vp8d.prob_tbl.cpu;
	dst += (8 * 7);
	for (i = 0; i < 4; ++i) {
		for (j = 0; j < 8; ++j) {
			for (k = 0; k < 3; ++k) {
				dst[0] = entropy_hdr->coeff_probs[i][j][k][0];
				dst[1] = entropy_hdr->coeff_probs[i][j][k][1];
				dst[2] = entropy_hdr->coeff_probs[i][j][k][2];
				dst[3] = entropy_hdr->coeff_probs[i][j][k][3];
				dst += 4;
			}
		}
	}

	/* coeff probs (footer part) */
	dst = ctx->hw.vp8d.prob_tbl.cpu;
	dst += (8 * 55);
	for (i = 0; i < 4; ++i) {
		for (j = 0; j < 8; ++j) {
			for (k = 0; k < 3; ++k) {
				dst[0] = entropy_hdr->coeff_probs[i][j][k][4];
				dst[1] = entropy_hdr->coeff_probs[i][j][k][5];
				dst[2] = entropy_hdr->coeff_probs[i][j][k][6];
				dst[3] = entropy_hdr->coeff_probs[i][j][k][7];
				dst[4] = entropy_hdr->coeff_probs[i][j][k][8];
				dst[5] = entropy_hdr->coeff_probs[i][j][k][9];
				dst[6] = entropy_hdr->coeff_probs[i][j][k][10];
				dst[7] = 0; /*unused */
				dst += 8;
			}
		}
	}
}

/*
 * set loop filters
 */
static void rockchip_vp8d_cfg_lf(struct rockchip_vpu_ctx *ctx)
{
	const struct v4l2_ctrl_vp8_frame_hdr *hdr = ctx->run.vp8d.frame_hdr;
	__s8 reg_value;
	int i;

	if (!(hdr->sgmnt_hdr.flags & V4L2_VP8_SEGMNT_HDR_FLAG_ENABLED)) {
		vp8d_write_regs_value(VDPU_REG_REF_PIC_LF_LEVEL_0,
				      hdr->lf_hdr.level,
				      "VDPU_REG_REF_PIC_LF_LEVEL_0");
	} else if (hdr->sgmnt_hdr.segment_feature_mode) {
		/* absolute mode */
		for (i = 0; i < 4; i++) {
			vp8d_write_regs_value(VDPU_REG_REF_PIC_LF_LEVEL_0 + i,
					      hdr->sgmnt_hdr.lf_update[i],
					      "VDPU_REG_REF_PIC_LF_LEVEL_ARRAY"
					      );
		}
	} else {
		/* delta mode */
		for (i = 0; i < 4; i++) {
			vp8d_write_regs_value(VDPU_REG_REF_PIC_LF_LEVEL_0 + i,
					      clamp(hdr->lf_hdr.level +
						    hdr->sgmnt_hdr.lf_update[i],
						    0, 63),
					      "VDPU_REG_REF_PIC_LF_LEVEL_ARRAY"
					      );
		}
	}

	vp8d_write_regs_value(VDPU_REG_REF_PIC_FILT_SHARPNESS,
			      (hdr->lf_hdr.sharpness_level),
			      "VDPU_REG_REF_PIC_FILT_SHARPNESS");
	if (hdr->lf_hdr.type)
		vp8d_write_regs_value(VDPU_REG_REF_PIC_FILT_TYPE_E, 1,
				      "VDPU_REG_REF_PIC_FILT_TYPE_E");

	if (hdr->lf_hdr.flags & V4L2_VP8_LF_HDR_ADJ_ENABLE) {
		for (i = 0; i < 4; i++) {
			reg_value = hdr->lf_hdr.mb_mode_delta_magnitude[i];
			vp8d_write_regs_value(VDPU_REG_FILT_MB_ADJ_0 + i,
					      reg_value,
					      "VDPU_REG_FILT_MB_ADJ_ARRAY");
			reg_value = hdr->lf_hdr.ref_frm_delta_magnitude[i];
			vp8d_write_regs_value(VDPU_REG_REF_PIC_ADJ_0 + i,
					      reg_value,
					      "VDPU_REG_REF_PIC_ADJ_ARRAY");
		}
	}
}

/*
 * set quantization parameters
 */
static void rockchip_vp8d_cfg_qp(struct rockchip_vpu_ctx *ctx)
{
	const struct v4l2_ctrl_vp8_frame_hdr *hdr = ctx->run.vp8d.frame_hdr;
	__s8 reg_value;
	int i;

	if (!(hdr->sgmnt_hdr.flags & V4L2_VP8_SEGMNT_HDR_FLAG_ENABLED)) {
		vp8d_write_regs_value(VDPU_REG_REF_PIC_QUANT_0,
				      hdr->quant_hdr.y_ac_qi,
				      "VDPU_REG_REF_PIC_QUANT_0");
	} else if (hdr->sgmnt_hdr.segment_feature_mode) {
		/* absolute mode */
		for (i = 0; i < 4; i++) {
			vp8d_write_regs_value(VDPU_REG_REF_PIC_QUANT_0 + i,
					      hdr->sgmnt_hdr.quant_update[i],
					      "VDPU_REG_REF_PIC_QUANT_ARRAY");
		}
	} else {
		/* delta mode */
		for (i = 0; i < 4; i++) {
			reg_value = hdr->sgmnt_hdr.quant_update[i];
			vp8d_write_regs_value(VDPU_REG_REF_PIC_QUANT_0 + i,
					      clamp(hdr->quant_hdr.y_ac_qi +
						    reg_value,
						    0, 127),
					      "VDPU_REG_REF_PIC_QUANT_ARRAY");
		}
	}

	vp8d_write_regs_value(VDPU_REG_REF_PIC_QUANT_DELTA_0,
			      hdr->quant_hdr.y_dc_delta,
			      "VDPU_REG_REF_PIC_QUANT_DELTA_0");
	vp8d_write_regs_value(VDPU_REG_REF_PIC_QUANT_DELTA_1,
			      hdr->quant_hdr.y2_dc_delta,
			      "VDPU_REG_REF_PIC_QUANT_DELTA_1");
	vp8d_write_regs_value(VDPU_REG_REF_PIC_QUANT_DELTA_2,
			      hdr->quant_hdr.y2_ac_delta,
			      "VDPU_REG_REF_PIC_QUANT_DELTA_2");
	vp8d_write_regs_value(VDPU_REG_REF_PIC_QUANT_DELTA_3,
			      hdr->quant_hdr.uv_dc_delta,
			      "VDPU_REG_REF_PIC_QUANT_DELTA_3");
	vp8d_write_regs_value(VDPU_REG_REF_PIC_QUANT_DELTA_4,
			      hdr->quant_hdr.uv_ac_delta,
			      "VDPU_REG_REF_PIC_QUANT_DELTA_4");
}

/*
 * set control partition and dct partition regs
 *
 * VP8 frame stream data layout:
 *
 *	                     first_part_size          parttion_sizes[0]
 *                              ^                     ^
 * src_dma                      |                     |
 * ^                   +--------+------+        +-----+-----+
 * |                   | control part  |        |           |
 * +--------+----------------+------------------+-----------+-----+-----------+
 * | tag 3B | extra 7B | hdr | mb_data | dct sz | dct part0 | ... | dct partn |
 * +--------+-----------------------------------+-----------+-----+-----------+
 *                     |     |         |        |                             |
 *                     |     v         +----+---+                             v
 *                     |     mb_start       |                       src_dma_end
 *                     v                    v
 *             first_part_offset         dct size part
 *                                      (num_dct-1)*3B
 * Note:
 *   1. only key frame has extra 7 bytes
 *   2. all offsets are base on src_dma
 *   3. number of dct parts is 1, 2, 4 or 8
 *   4. the addresses set to vpu must be 64bits alignment
 */
static void rockchip_vp8d_cfg_parts(struct rockchip_vpu_ctx *ctx)
{
	const struct v4l2_ctrl_vp8_frame_hdr *hdr = ctx->run.vp8d.frame_hdr;
	u32 dct_part_total_len = 0;
	u32 dct_size_part_size = 0;
	u32 dct_part_offset = 0;
	u32 mb_offset_bytes = 0;
	u32 mb_offset_bits = 0;
	u32 mb_start_bits = 0;
	dma_addr_t src_dma;
	u32 mb_size = 0;
	u32 count = 0;
	u32 i;

	src_dma = vb2_dma_contig_plane_dma_addr(&ctx->run.src->b.vb2_buf, 0);

	/*
	 * Calculate control partition mb data info
	 * @macroblock_bit_offset:	bits offset of mb data from first
	 *				part start pos
	 * @mb_offset_bits:		bits offset of mb data from src_dma
	 *				base addr
	 * @mb_offset_byte:		bytes offset of mb data from src_dma
	 *				base addr
	 * @mb_start_bits:		bits offset of mb data from mb data
	 *				64bits alignment addr
	 */
	mb_offset_bits = hdr->first_part_offset * 8
			 + hdr->macroblock_bit_offset + 8;
	mb_offset_bytes = mb_offset_bits / 8;
	mb_start_bits = mb_offset_bits
			- (mb_offset_bytes & (~DEC_8190_ALIGN_MASK)) * 8;
	mb_size = hdr->first_part_size
		  - (mb_offset_bytes - hdr->first_part_offset)
		  + (mb_offset_bytes & DEC_8190_ALIGN_MASK);

	/* mb data aligned base addr */
	vp8d_write_regs_value(VDPU_REG_VP8_ADDR_CTRL_PART,
			      (mb_offset_bytes & (~DEC_8190_ALIGN_MASK))
			      + src_dma,
			      "VDPU_REG_VP8_ADDR_CTRL_PART");

	/* mb data start bits */
	vp8d_write_regs_value(VDPU_REG_DEC_CTRL2_STRM1_START_BIT,
			      mb_start_bits,
			      "VDPU_REG_DEC_CTRL2_STRM1_START_BIT");

	/* mb aligned data length */
	vp8d_write_regs_value(VDPU_REG_DEC_CTRL6_STREAM1_LEN,
			      mb_size,
			      "VDPU_REG_DEC_CTRL6_STREAM1_LEN");

	/*
	 * Calculate dct partition info
	 * @dct_size_part_size: Containing sizes of dct part, every dct part
	 *			has 3 bytes to store its size, except the last
	 *			dct part
	 * @dct_part_offset:	bytes offset of dct parts from src_dma base addr
	 * @dct_part_total_len: total size of all dct parts
	 */
	dct_size_part_size = (hdr->num_dct_parts - 1) * 3;
	dct_part_offset = hdr->first_part_offset + hdr->first_part_size;
	for (i = 0; i < hdr->num_dct_parts; i++)
		dct_part_total_len += hdr->dct_part_sizes[i];
	dct_part_total_len += dct_size_part_size;
	dct_part_total_len += (dct_part_offset & DEC_8190_ALIGN_MASK);

	/* number of dct partitions */
	vp8d_write_regs_value(VDPU_REG_DEC_CTRL6_COEFFS_PART_AM,
			      (hdr->num_dct_parts - 1),
			      "VDPU_REG_DEC_CTRL6_COEFFS_PART_AM");

	/* dct partition length */
	vp8d_write_regs_value(VDPU_REG_DEC_CTRL3_STREAM_LEN,
			      dct_part_total_len,
			      "VDPU_REG_DEC_CTRL3_STREAM_LEN");
	/* dct partitions base address */
	for (i = 0; i < hdr->num_dct_parts; i++) {
		u32 byte_offset = dct_part_offset + dct_size_part_size + count;
		u32 base_addr = byte_offset + src_dma;

		vp8d_write_regs_value(VDPU_REG_ADDR_STR + i,
				      base_addr & (~DEC_8190_ALIGN_MASK),
				      "VDPU_REG_ADDR_STR_ARRAY");

		vp8d_write_regs_value(VDPU_REG_DEC_CTRL2_STRM_START_BIT + i,
				      ((byte_offset & DEC_8190_ALIGN_MASK) * 8),
				      "VDPU_REG_DEC_CTRL2_STRM_START_BIT_ARRAY"
				     );

		count += hdr->dct_part_sizes[i];
	}
}

/*
 * prediction filter taps
 * normal 6-tap filters
 */
static void rockchip_vp8d_cfg_tap(struct rockchip_vpu_ctx *ctx)
{
	const struct v4l2_ctrl_vp8_frame_hdr *hdr = ctx->run.vp8d.frame_hdr;
	int i, j, index;

	if ((hdr->version & 0x03) != 0)
		return; /* Tap filter not used. */

	for (i = 0; i < 8; i++) {
		for (j = 0; j < 6; j++) {
			index = VDPU_REG_PRED_FLT_NONE_0 + i * 6 + j;
			if (RK_GET_REG_BASE(index) != 0) {
				vp8d_write_regs_value(index,
						      vp8d_mc_filter[i][j],
						      "VDPU_REG_PRED_FLT_ARRAY"
						      );
			}
		}
	}
}

/* set reference frame */
static void rockchip_vp8d_cfg_ref(struct rockchip_vpu_ctx *ctx)
{
	struct vb2_buffer *buf;
	const struct v4l2_ctrl_vp8_frame_hdr *hdr = ctx->run.vp8d.frame_hdr;
	dma_addr_t dma_address;

	/* set last frame address */
	if (hdr->last_frame >= ctx->vq_dst.num_buffers)
		buf = &ctx->run.dst->b.vb2_buf;
	else
		buf = ctx->dst_bufs[hdr->last_frame];

	if (!hdr->key_frame) {
		dma_address =
			vb2_dma_contig_plane_dma_addr(&ctx->run.dst->b.vb2_buf,
						      0);
		vp8d_write_regs_value(VDPU_REG_VP8_ADDR_REF0,
				      dma_address,
				      "VDPU_REG_VP8_ADDR_REF0");
	} else {
		vp8d_write_regs_value(VDPU_REG_VP8_ADDR_REF0,
				      vb2_dma_contig_plane_dma_addr(buf, 0),
				      "VDPU_REG_VP8_ADDR_REF0");
	}

	/* set golden reference frame buffer address */
	if (hdr->golden_frame >= ctx->vq_dst.num_buffers)
		buf = &ctx->run.dst->b.vb2_buf;
	else
		buf = ctx->dst_bufs[hdr->golden_frame];

	vp8d_write_regs_value(VDPU_REG_VP8_ADDR_REF2_5_0,
			      vb2_dma_contig_plane_dma_addr(buf, 0),
			      "VDPU_REG_VP8_ADDR_REF2_5_0");

	if (hdr->sign_bias_golden)
		vp8d_write_regs_value(VDPU_REG_VP8_GREF_SIGN_BIAS_0, 1,
				      "VDPU_REG_VP8_GREF_SIGN_BIAS_0");

	/* set alternate reference frame buffer address */
	if (hdr->alt_frame >= ctx->vq_dst.num_buffers)
		buf = &ctx->run.dst->b.vb2_buf;
	else
		buf = ctx->dst_bufs[hdr->alt_frame];

	vp8d_write_regs_value(VDPU_REG_VP8_ADDR_REF2_5_1,
			      vb2_dma_contig_plane_dma_addr(buf, 0),
			      "VDPU_REG_VP8_ADDR_REF2_5_1");
	if (hdr->sign_bias_alternate)
		vp8d_write_regs_value(VDPU_REG_VP8_AREF_SIGN_BIAS_1, 1,
				      "VDPU_REG_VP8_AREF_SIGN_BIAS_1");
}

static void rockchip_vp8d_cfg_buffers(struct rockchip_vpu_ctx *ctx)
{
	const struct v4l2_ctrl_vp8_frame_hdr *hdr = ctx->run.vp8d.frame_hdr;
	dma_addr_t dma_address;

	/* set probability table buffer address */
	vp8d_write_regs_value(VDPU_REG_ADDR_QTABLE,
			      ctx->hw.vp8d.prob_tbl.dma,
			      "VDPU_REG_ADDR_QTABLE");

	/* set segment map address */
	vp8d_write_regs_value(VDPU_REG_FWD_PIC1_SEGMENT_BASE,
			      ctx->hw.vp8d.segment_map.dma,
			      "VDPU_REG_FWD_PIC1_SEGMENT_BASE");

	if (hdr->sgmnt_hdr.flags & V4L2_VP8_SEGMNT_HDR_FLAG_ENABLED) {
		vp8d_write_regs_value(VDPU_REG_FWD_PIC1_SEGMENT_E, 1,
				      "VDPU_REG_FWD_PIC1_SEGMENT_E");
		if (hdr->sgmnt_hdr.flags & V4L2_VP8_SEGMNT_HDR_FLAG_UPDATE_MAP)
			vp8d_write_regs_value(VDPU_REG_FWD_PIC1_SEGMENT_UPD_E,
					      1,
					      "VDPU_REG_FWD_PIC1_SEGMENT_UPD_E");
	}

	dma_address = vb2_dma_contig_plane_dma_addr(&ctx->run.dst->b.vb2_buf,
						    0);
	/* set output frame buffer address */
	vp8d_write_regs_value(VDPU_REG_ADDR_DST, dma_address,
			      "VDPU_REG_ADDR_DST");
}
#endif

int rockchip_vpu_vp8d_init(struct rockchip_vpu_ctx *ctx)
{
#ifdef NO_BOILERPLATE_CLEANUP
	struct rockchip_vpu_dev *vpu = ctx->dev;
	unsigned int mb_width, mb_height;
	size_t segment_map_size;
	int ret;

	vpu_debug_enter();
	if (vpu->variant->vpu_type == RK3229_VPU) {
		rockchip_regs_table = rk3229_vp8d_regs_table;
		rockchip_regs_map = rk3229_regs_map;
	} else if (vpu->variant->vpu_type == RK3288_VPU) {
		rockchip_regs_table = rk3288_vp8d_regs_table;
		rockchip_regs_map = rk3288_regs_map;
	} else {
		vpu_err("unknown platform\n");
		return -EPERM;
	}
	/* segment map table size calculation */
	mb_width = MB_WIDTH(ctx->dst_fmt.width);
	mb_height = MB_HEIGHT(ctx->dst_fmt.height);
	segment_map_size = round_up(DIV_ROUND_UP(mb_width * mb_height, 4), 64);

	/*
	 * In context init the dma buffer for segment map must be allocated.
	 * And the data in segment map buffer must be set to all zero.
	 */
	ret = rockchip_vpu_aux_buf_alloc(vpu, &ctx->hw.vp8d.segment_map,
					 segment_map_size);
	if (ret) {
		vpu_err("allocate segment map mem failed\n");
		return ret;
	}
	memset(ctx->hw.vp8d.segment_map.cpu, 0, ctx->hw.vp8d.segment_map.size);

	/*
	 * Allocate probability table buffer,
	 * total 1208 bytes, 4K page is far enough.
	 */
	ret = rockchip_vpu_aux_buf_alloc(vpu, &ctx->hw.vp8d.prob_tbl,
					 sizeof(struct vp8_prob_tbl_packed));
	if (ret) {
		vpu_err("allocate prob table mem failed\n");
		goto prob_table_failed;
	}

	vpu_debug_leave();
	return 0;

prob_table_failed:
	rockchip_vpu_aux_buf_free(vpu, &ctx->hw.vp8d.segment_map);

	vpu_debug_leave();
	return ret;
#else
	return 0;
#endif
}

void rockchip_vpu_vp8d_exit(struct rockchip_vpu_ctx *ctx)
{
#ifdef NO_BOILERPLATE_CLEANUP
	struct rockchip_vpu_dev *vpu = ctx->dev;

	vpu_debug_enter();

	rockchip_vpu_aux_buf_free(vpu, &ctx->hw.vp8d.segment_map);
	rockchip_vpu_aux_buf_free(vpu, &ctx->hw.vp8d.prob_tbl);

	vpu_debug_leave();
#endif
}

void rockchip_vpu_vp8d_run(struct rockchip_vpu_ctx *ctx)
{
#ifdef NO_BOILERPLATE_CLEANUP
	const struct v4l2_ctrl_vp8_frame_hdr *hdr = ctx->run.vp8d.frame_hdr;
	struct rockchip_vpu_dev *vpu = ctx->dev;
	size_t height = ctx->dst_fmt.height;
	size_t width = ctx->dst_fmt.width;
	u32 mb_width, mb_height;
	u32 reg;
	u32 cur_reg;
	u32 reg_base;
	int i;

	vpu_debug_enter();

	memset(rk_regs_value, 0, sizeof(rk_regs_value));

	rockchip_vp8d_dump_hdr(ctx);

	/* reset segment_map buffer in keyframe */
	if (!hdr->key_frame && ctx->hw.vp8d.segment_map.cpu)
		memset(ctx->hw.vp8d.segment_map.cpu, 0,
		       ctx->hw.vp8d.segment_map.size);

	rockchip_vp8d_prob_update(ctx);

	rockchip_vpu_power_on(vpu);

	for (i = 0; i < vpu->variant->dec_reg_num; i++)
		vdpu_write_relaxed(vpu, 0, i * 4);

	vp8d_write_regs_value(VDPU_REG_CONFIG_DEC_TIMEOUT_E, 1,
			      "VDPU_REG_CONFIG_DEC_TIMEOUT_E");
	vp8d_write_regs_value(VDPU_REG_CONFIG_DEC_CLK_GATE_E, 1,
			      "VDPU_REG_CONFIG_DEC_CLK_GATE_E");

	if (hdr->key_frame)
		vp8d_write_regs_value(VDPU_REG_DEC_CTRL0_PIC_INTER_E, 1,
				      "VDPU_REG_DEC_CTRL0_PIC_INTER_E");

	vp8d_write_regs_value(VDPU_REG_CONFIG_DEC_STRENDIAN_E, 1,
			      "VDPU_REG_CONFIG_DEC_STRENDIAN_E");
	vp8d_write_regs_value(VDPU_REG_CONFIG_DEC_INSWAP32_E, 1,
			      "VDPU_REG_CONFIG_DEC_INSWAP32_E");
	vp8d_write_regs_value(VDPU_REG_CONFIG_DEC_STRSWAP32_E, 1,
			      "VDPU_REG_CONFIG_DEC_STRSWAP32_E");
	vp8d_write_regs_value(VDPU_REG_CONFIG_DEC_OUTSWAP32_E, 1,
			      "VDPU_REG_CONFIG_DEC_OUTSWAP32_E");
	vp8d_write_regs_value(VDPU_REG_CONFIG_DEC_IN_ENDIAN, 1,
			      "VDPU_REG_CONFIG_DEC_IN_ENDIAN");
	vp8d_write_regs_value(VDPU_REG_CONFIG_DEC_OUT_ENDIAN, 1,
			      "VDPU_REG_CONFIG_DEC_OUT_ENDIAN");

	vp8d_write_regs_value(VDPU_REG_CONFIG_DEC_MAX_BURST, 16,
			      "VDPU_REG_CONFIG_DEC_MAX_BURST");

	vp8d_write_regs_value(VDPU_REG_DEC_CTRL0_DEC_MODE, 10,
			      "VDPU_REG_DEC_CTRL0_DEC_MODE");

	if (!(hdr->flags & V4L2_VP8_FRAME_HDR_FLAG_MB_NO_SKIP_COEFF))
		vp8d_write_regs_value(VDPU_REG_DEC_CTRL0_SKIP_MODE, 1,
				      "VDPU_REG_DEC_CTRL0_SKIP_MODE");
	if (hdr->lf_hdr.level == 0)
		vp8d_write_regs_value(VDPU_REG_DEC_CTRL0_FILTERING_DIS, 1,
				      "VDPU_REG_DEC_CTRL0_FILTERING_DIS");

	/* frame dimensions */
	mb_width = MB_WIDTH(width);
	mb_height = MB_HEIGHT(height);
	vp8d_write_regs_value(VDPU_REG_DEC_PIC_MB_WIDTH, mb_width,
			      "VDPU_REG_DEC_PIC_MB_WIDTH");
	vp8d_write_regs_value(VDPU_REG_DEC_PIC_MB_HEIGHT_P, mb_height,
			      "VDPU_REG_DEC_PIC_MB_HEIGHT_P");
	vp8d_write_regs_value(VDPU_REG_DEC_CTRL1_PIC_MB_W_EXT, (mb_width >> 9),
			      "VDPU_REG_DEC_CTRL1_PIC_MB_W_EXT");
	vp8d_write_regs_value(VDPU_REG_DEC_CTRL1_PIC_MB_H_EXT, (mb_height >> 8),
			      "VDPU_REG_DEC_CTRL1_PIC_MB_H_EXT");

	/* bool decode info */
	vp8d_write_regs_value(VDPU_REG_DEC_CTRL2_BOOLEAN_RANGE,
			      hdr->bool_dec_range,
			      "VDPU_REG_DEC_CTRL2_BOOLEAN_RANGE");
	vp8d_write_regs_value(VDPU_REG_DEC_CTRL2_BOOLEAN_VALUE,
			      hdr->bool_dec_value,
			      "VDPU_REG_DEC_CTRL2_BOOLEAN_VALUE");

	if (hdr->version != 3)
		vp8d_write_regs_value(VDPU_REG_DEC_CTRL4_VC1_HEIGHT_EXT, 1,
				      "VDPU_REG_DEC_CTRL4_VC1_HEIGHT_EXT");
	if (hdr->version & 0x3)
		vp8d_write_regs_value(VDPU_REG_DEC_CTRL4_BILIN_MC_E, 1,
				      "VDPU_REG_DEC_CTRL4_BILIN_MC_E");

	rockchip_vp8d_cfg_lf(ctx);
	rockchip_vp8d_cfg_qp(ctx);
	rockchip_vp8d_cfg_parts(ctx);
	rockchip_vp8d_cfg_tap(ctx);
	rockchip_vp8d_cfg_ref(ctx);
	rockchip_vp8d_cfg_buffers(ctx);

	reg = (rk_regs_value[0]
	       & RK_GET_REG_BITS_MASK(0)) << RK_GET_REG_BITS_OFFSET(0);
	reg_base = RK_GET_REG_BASE(0);

	for (i = 1; i <= VDPU_REG_BEFORE_ENABLE; i++) {
		cur_reg = (rk_regs_value[i]
			   & RK_GET_REG_BITS_MASK(i))
				<< RK_GET_REG_BITS_OFFSET(i);

		if ((reg_base != 0)
		    && (reg_base != RK_GET_REG_BASE(i)
			|| i == VDPU_REG_BEFORE_ENABLE)) {
			reg |= vdpu_read(vpu, reg_base);
			vdpu_write_relaxed(vpu, reg, reg_base);
			reg = cur_reg;
		} else
			reg |= cur_reg;

		reg_base = RK_GET_REG_BASE(i);
	}
	schedule_delayed_work(&vpu->watchdog_work, msecs_to_jiffies(2000));

	reg = vdpu_read(vpu, RK_GET_REG_BASE(VDPU_REG_INTERRUPT_DEC_E));
	reg &= ~(RK_GET_REG_BITS_MASK(VDPU_REG_INTERRUPT_DEC_E)
		 << RK_GET_REG_BITS_OFFSET(VDPU_REG_INTERRUPT_DEC_E));
	reg |= (((1) & RK_GET_REG_BITS_MASK(VDPU_REG_INTERRUPT_DEC_E))
		<< RK_GET_REG_BITS_OFFSET(VDPU_REG_INTERRUPT_DEC_E));
	vdpu_write_relaxed(vpu, reg, RK_GET_REG_BASE(VDPU_REG_INTERRUPT_DEC_E));

	vpu_debug_leave();
#endif
}


int rockchip_vdpu_irq(int irq, struct rockchip_vpu_dev *vpu)
{
#ifdef NO_BOILERPLATE_CLEANUP
	u32 mask;
	u32 status = vdpu_read(vpu,
			       RK_GET_REG_BASE(VDPU_REG_INTERRUPT_DEC_IRQ));

	vdpu_write(vpu, 0, RK_GET_REG_BASE(VDPU_REG_INTERRUPT_DEC_IRQ));

	vpu_debug(3, "vdpu_irq status: %08x\n", status);

	mask = RK_GET_REG_BITS_MASK(VDPU_REG_INTERRUPT_DEC_IRQ);
	mask = mask << RK_GET_REG_BITS_OFFSET(VDPU_REG_INTERRUPT_DEC_IRQ);
	if (status & mask) {
		vdpu_write(vpu, 0,
			   RK_GET_REG_BASE(VDPU_REG_CONFIG_DEC_MAX_BURST));
		return 0;
	}
#endif
	return -1;
}

/*
 * Initialization/clean-up.
 */

void rockchip_vpu_dec_reset(struct rockchip_vpu_ctx *ctx)
{
#ifdef NO_BOILERPLATE_CLEANUP
	struct rockchip_vpu_dev *vpu = ctx->dev;
	u32 mask;

	mask = RK_GET_REG_BITS_MASK(VDPU_REG_INTERRUPT_DEC_IRQ_DIS);
	mask = mask << RK_GET_REG_BITS_OFFSET(VDPU_REG_INTERRUPT_DEC_IRQ_DIS);
	vdpu_write(vpu, mask, RK_GET_REG_BASE(VDPU_REG_INTERRUPT_DEC_IRQ_DIS));
	vdpu_write(vpu, 0, RK_GET_REG_BASE(VDPU_REG_CONFIG_DEC_TIMEOUT_E));
#endif
}
