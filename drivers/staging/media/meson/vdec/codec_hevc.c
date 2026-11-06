// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 Maxime Jourdan <mjourdan@baylibre.com>
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 */

#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "codec_hevc.h"
#include "dos_regs.h"
#include "esparser.h"
#include "hevc_regs.h"
#include "vdec_hevc.h"
#include "vdec_helpers.h"
#include "codec_hevc_common.h"

/* HEVC reg mapping */
#define HEVC_DEC_STATUS_REG	HEVC_ASSIST_SCRATCH_0
	#define HEVC_ACTION_DONE	0xff
#define HEVC_RPM_BUFFER		HEVC_ASSIST_SCRATCH_1
#define HEVC_SHORT_TERM_RPS	HEVC_ASSIST_SCRATCH_2
#define HEVC_VPS_BUFFER		HEVC_ASSIST_SCRATCH_3
#define HEVC_SPS_BUFFER		HEVC_ASSIST_SCRATCH_4
#define HEVC_PPS_BUFFER		HEVC_ASSIST_SCRATCH_5
#define HEVC_SAO_UP		HEVC_ASSIST_SCRATCH_6
#define HEVC_STREAM_SWAP_BUFFER HEVC_ASSIST_SCRATCH_7
#define H265_MMU_MAP_BUFFER	HEVC_ASSIST_SCRATCH_7
#define HEVC_STREAM_SWAP_BUFFER2 HEVC_ASSIST_SCRATCH_8
#define HEVC_sao_mem_unit	HEVC_ASSIST_SCRATCH_9
#define HEVC_SAO_ABV		HEVC_ASSIST_SCRATCH_A
#define HEVC_sao_vb_size	HEVC_ASSIST_SCRATCH_B
#define HEVC_SAO_VB		HEVC_ASSIST_SCRATCH_C
#define HEVC_SCALELUT		HEVC_ASSIST_SCRATCH_D
#define HEVC_WAIT_FLAG		HEVC_ASSIST_SCRATCH_E
#define RPM_CMD_REG		HEVC_ASSIST_SCRATCH_F
#define LMEM_DUMP_ADR		HEVC_ASSIST_SCRATCH_F
#define DEBUG_REG1		HEVC_ASSIST_SCRATCH_G
#define HEVC_DECODE_MODE2	HEVC_ASSIST_SCRATCH_H
#define NAL_SEARCH_CTL		HEVC_ASSIST_SCRATCH_I
	#define NAL_SEARCH_AUTO		0x2
	#define NAL_SEARCH_MULTI		0x4
	#define NAL_SEARCH_LEGACY		0xc
#define HEVC_DECODE_MODE	HEVC_ASSIST_SCRATCH_J
	#define DECODE_MODE_SINGLE			0
	#define DECODE_MODE_MULTI_FRAMEBASE	1
	#define DECODE_MODE_MBOX0			(0x80 << 24)
#define DECODE_STOP_POS		HEVC_ASSIST_SCRATCH_K
#define HEVC_AUX_ADR		HEVC_ASSIST_SCRATCH_L
#define HEVC_AUX_DATA_SIZE	HEVC_ASSIST_SCRATCH_M
#define HEVC_DECODE_SIZE	HEVC_ASSIST_SCRATCH_N
#define HEVC_MPC_E		0xcc20

#define AMRISC_MAIN_REQ		 0x04

/* HEVC Constants */
#define MAX_REF_PIC_NUM		24
#define MAX_REF_ACTIVE		16
#define MAX_TILE_COL_NUM	10
#define MAX_TILE_ROW_NUM	20
#define MAX_SLICE_NUM		800
#define INVALID_POC		0x80000000
#define HEVC_WATCHDOG_POLL_MS	50
#define HEVC_WATCHDOG_PROGRESS_MS	500
#define HEVC_WATCHDOG_STALL_CHECKS	2

/* HEVC Workspace layout */
#define MPRED_MV_BUF_SIZE 0x120000

#define IPP_SIZE	0x4000
#define SAO_ABV_SIZE	0x30000
#define SAO_VB_SIZE	0x30000
#define SH_TM_RPS_SIZE	0x800
#define VPS_SIZE	0x800
#define SPS_SIZE	0x800
#define PPS_SIZE	0x2000
#define SAO_UP_SIZE	0x2800
#define SWAP_BUF_SIZE	0x800
#define SWAP_BUF2_SIZE	0x800
#define SCALELUT_SIZE	0x8000
#define DBLK_PARA_SIZE	0x20000
#define DBLK_DATA_SIZE	0x80000
#define DBLK_DATA2_SIZE	0x80000
#define MMU_VBH_SIZE	0x5000
#define MPRED_ABV_SIZE	0x8000
#define MPRED_MV_SIZE	(MPRED_MV_BUF_SIZE * MAX_REF_PIC_NUM)
#define RPM_BUF_SIZE	0x100
#define LMEM_SIZE	0xA00

#define IPP_OFFSET       0x00
#define SAO_ABV_OFFSET   (IPP_OFFSET + IPP_SIZE)
#define SAO_VB_OFFSET    (SAO_ABV_OFFSET + SAO_ABV_SIZE)
#define SH_TM_RPS_OFFSET (SAO_VB_OFFSET + SAO_VB_SIZE)
#define VPS_OFFSET       (SH_TM_RPS_OFFSET + SH_TM_RPS_SIZE)
#define SPS_OFFSET       (VPS_OFFSET + VPS_SIZE)
#define PPS_OFFSET       (SPS_OFFSET + SPS_SIZE)
#define SAO_UP_OFFSET    (PPS_OFFSET + PPS_SIZE)
#define SWAP_BUF_OFFSET  (SAO_UP_OFFSET + SAO_UP_SIZE)
#define SWAP_BUF2_OFFSET (SWAP_BUF_OFFSET + SWAP_BUF_SIZE)
#define SCALELUT_OFFSET  (SWAP_BUF2_OFFSET + SWAP_BUF2_SIZE)
#define DBLK_PARA_OFFSET (SCALELUT_OFFSET + SCALELUT_SIZE)
#define DBLK_DATA_OFFSET (DBLK_PARA_OFFSET + DBLK_PARA_SIZE)
#define DBLK_DATA2_OFFSET (DBLK_DATA_OFFSET + DBLK_DATA_SIZE)
#define MMU_VBH_OFFSET   (DBLK_DATA2_OFFSET + DBLK_DATA2_SIZE)
#define MPRED_ABV_OFFSET (MMU_VBH_OFFSET + MMU_VBH_SIZE)
#define MPRED_MV_OFFSET  (MPRED_ABV_OFFSET + MPRED_ABV_SIZE)
#define RPM_OFFSET       (MPRED_MV_OFFSET + MPRED_MV_SIZE)
#define LMEM_OFFSET      (RPM_OFFSET + RPM_BUF_SIZE)

/* ISR decode status */
#define HEVC_DEC_IDLE                        0x0
#define HEVC_NAL_UNIT_VPS                    0x1
#define HEVC_NAL_UNIT_SPS                    0x2
#define HEVC_NAL_UNIT_PPS                    0x3
#define HEVC_NAL_UNIT_CODED_SLICE_SEGMENT    0x4
#define HEVC_CODED_SLICE_SEGMENT_DAT         0x5
#define HEVC_SLICE_DECODING                  0x6
#define HEVC_NAL_UNIT_SEI                    0x7
#define HEVC_SLICE_SEGMENT_DONE              0x8
#define HEVC_NAL_SEARCH_DONE                 0x9
#define HEVC_DECPIC_DATA_DONE                0xa
#define HEVC_DECPIC_DATA_ERROR               0xb
#define HEVC_SEI_DAT                         0xc
#define HEVC_SEI_DAT_DONE                    0xd
#define HEVC_DECODE_BUFEMPTY                 0x20
#define HEVC_SEARCH_BUFEMPTY                 0x22
#define HEVC_DECODE_BUFEMPTY2                0x24

/* RPM misc_flag0 */
#define PCM_LOOP_FILTER_DISABLED_FLAG_BIT		0
#define PCM_ENABLE_FLAG_BIT				1
#define LOOP_FILER_ACROSS_TILES_ENABLED_FLAG_BIT	2
#define PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT	3
#define DEBLOCKING_FILTER_OVERRIDE_ENABLED_FLAG_BIT	4
#define PPS_DEBLOCKING_FILTER_DISABLED_FLAG_BIT		5
#define DEBLOCKING_FILTER_OVERRIDE_FLAG_BIT		6
#define SLICE_DEBLOCKING_FILTER_DISABLED_FLAG_BIT	7
#define SLICE_SAO_LUMA_FLAG_BIT				8
#define SLICE_SAO_CHROMA_FLAG_BIT			9
#define SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT 10

/* Constants for HEVC_MPRED_CTRL1 */
#define AMVP_MAX_NUM_CANDS_MEM	3
#define AMVP_MAX_NUM_CANDS	2
#define NUM_CHROMA_MODE		5
#define DM_CHROMA_IDX		36

/* Buffer sizes */
#define SIZE_WORKSPACE ALIGN(LMEM_OFFSET + LMEM_SIZE, 64 * SZ_1K)
#define SIZE_AUX (SZ_1K * 16)
#define SIZE_FW_SWAP (SZ_1K * 16)
#define FW_SWAP_OFFSET (SZ_1K * 12)
#define SIZE_FRAME_MMU (0x1200 * 4)
#define RPM_SIZE 0x80
#define RPS_USED_BIT_LEGACY 14
#define RPS_USED_BIT_MULTI 13
#define HEVC_DPB_BUFFER_MARGIN 2

/* Multi firmware adds four tile-width words to the legacy RPM layout. */
#define RPM_LEGACY_TILE_HEIGHT_OFFSET 46
#define RPM_MULTI_TILE_HEIGHT_OFFSET 50
#define RPM_TILE_HEIGHT_WORDS 8
#define RPM_LEGACY_POST_TILE_OFFSET 54
#define RPM_MULTI_POST_TILE_OFFSET 58
#define RPM_POST_TILE_WORDS 60
#define RPM_MULTI_DPB_OFFSET 118

/* Data received from the HW in this form, do not rearrange */
union rpm_param {
	struct {
		u16 data[RPM_SIZE];
	} l;
	struct {
		u16 CUR_RPS[MAX_REF_ACTIVE];
		u16 num_ref_idx_l0_active;
		u16 num_ref_idx_l1_active;
		u16 slice_type;
		u16 slice_temporal_mvp_enable_flag;
		u16 dependent_slice_segment_flag;
		u16 slice_segment_address;
		u16 num_title_rows_minus1;
		u16 pic_width_in_luma_samples;
		u16 pic_height_in_luma_samples;
		u16 log2_min_coding_block_size_minus3;
		u16 log2_diff_max_min_coding_block_size;
		u16 log2_max_pic_order_cnt_lsb_minus4;
		u16 POClsb;
		u16 collocated_from_l0_flag;
		u16 collocated_ref_idx;
		u16 log2_parallel_merge_level;
		u16 five_minus_max_num_merge_cand;
		u16 sps_num_reorder_pics_0;
		u16 modification_flag;
		u16 tiles_flags;
		u16 num_tile_columns_minus1;
		u16 num_tile_rows_minus1;
		u16 tile_width[8];
		u16 tile_height[8];
		u16 misc_flag0;
		u16 pps_beta_offset_div2;
		u16 pps_tc_offset_div2;
		u16 slice_beta_offset_div2;
		u16 slice_tc_offset_div2;
		u16 pps_cb_qp_offset;
		u16 pps_cr_qp_offset;
		u16 first_slice_segment_in_pic_flag;
		u16 m_temporalId;
		u16 m_nalUnitType;
		u16 vui_num_units_in_tick_hi;
		u16 vui_num_units_in_tick_lo;
		u16 vui_time_scale_hi;
		u16 vui_time_scale_lo;
		u16 bit_depth;
		u16 profile_etc;
		u16 sei_frame_field_info;
		u16 video_signal_type;
		u16 modification_list[0x20];
		u16 conformance_window_flag;
		u16 conf_win_left_offset;
		u16 conf_win_right_offset;
		u16 conf_win_top_offset;
		u16 conf_win_bottom_offset;
		u16 chroma_format_idc;
		u16 color_description;
		u16 aspect_ratio_idc;
		u16 sar_width;
		u16 sar_height;
	} p;
};

enum nal_unit_type {
	NAL_UNIT_CODED_SLICE_BLA	= 16,
	NAL_UNIT_CODED_SLICE_BLANT	= 17,
	NAL_UNIT_CODED_SLICE_BLA_N_LP	= 18,
	NAL_UNIT_CODED_SLICE_IDR	= 19,
	NAL_UNIT_CODED_SLICE_IDR_N_LP	= 20,
};

enum slice_type {
	B_SLICE = 0,
	P_SLICE = 1,
	I_SLICE = 2,
};

/* A frame being decoded */
struct hevc_frame {
	struct list_head list;
	struct vb2_v4l2_buffer *vbuf;
	u32 offset;
	struct amvdec_timestamp_info timestamp;
	bool timestamp_valid;
	u32 poc;

	int referenced;
	int show;
	bool requeued;
	u32 num_reorder_pic;

	u32 cur_slice_idx;
	u32 cur_slice_type;

	/* 2 lists (L0/L1) ; 800 slices ; 16 refs */
	u32 ref_poc_list[2][MAX_SLICE_NUM][MAX_REF_ACTIVE];
	u32 ref_num[2];
};

struct codec_hevc {
	/* Protect the data structure */
	struct mutex lock;
	struct delayed_work watchdog_work;
	struct amvdec_session *sess;

	/* Common part of the HEVC decoder */
	struct codec_hevc_common common;

	/* Buffer for the HEVC Workspace */
	void      *workspace_vaddr;
	dma_addr_t workspace_paddr;

	/* AUX buffer */
	void      *aux_vaddr;
	dma_addr_t aux_paddr;
	void      *fw_swap_vaddr;
	dma_addr_t fw_swap_paddr;

	/* Contains many information parsed from the bitstream */
	union rpm_param rpm_param;

	/* Information computed from the RPM */
	u32 lcu_size; // Largest Coding Unit
	u32 lcu_x_num;
	u32 lcu_y_num;
	u32 lcu_total;

	/* Current Frame being handled */
	struct hevc_frame *cur_frame;
	u32 curr_poc;
	/* Collocated Reference Picture */
	struct hevc_frame *col_frame;
	u32 col_poc;

	/* All ref frames used by the HW at a given time */
	struct list_head ref_frames_list;
	u32 frames_num;

	/* Coded resolution reported by the hardware */
	u32 width, height;
	/* Resolution minus the conformance window offsets */
	u32 dst_width, dst_height;
	u32 dpb_size;
	u16 max_dec_pic_buffering;

	u32 prev_tid0_poc;
	u32 slice_segment_addr;
	u32 slice_addr;
	u32 ldc_flag;
	u32 input_size;
	u32 input_parser_size;
	struct vb2_v4l2_buffer *input_vbuf;
	u8 start_decoding_flag;
	u8 rps_set_id;
	u8 rps_used_bit;
	bool input_has_frame;
	bool input_pending;
	bool waiting_for_input;
	bool resume_pending;
	bool nal_search_done;
	bool reset_buffers;
	bool watchdog_armed;
	bool watchdog_progress_valid;
	unsigned long watchdog_progress_at;
	u32 watchdog_last_lcu;
	u32 watchdog_last_shift;
	unsigned int watchdog_stall_checks;

	/* Whether we detected the bitstream as 10-bit */
	int is_10bit;
};

static void codec_hevc_watchdog_disarm(struct codec_hevc *hevc)
{
	WRITE_ONCE(hevc->watchdog_armed, false);
	cancel_delayed_work(&hevc->watchdog_work);
}

static void codec_hevc_watchdog_cancel_sync(struct codec_hevc *hevc)
{
	WRITE_ONCE(hevc->watchdog_armed, false);

	/* A timeout yields from this work item through the scheduler stop path. */
	if (current_work() == &hevc->watchdog_work.work)
		return;
	cancel_delayed_work_sync(&hevc->watchdog_work);
}

static void codec_hevc_watchdog_note_progress(struct codec_hevc *hevc)
{
	WRITE_ONCE(hevc->watchdog_progress_valid, false);
	WRITE_ONCE(hevc->watchdog_progress_at, jiffies);
	WRITE_ONCE(hevc->watchdog_stall_checks,
		   HEVC_WATCHDOG_STALL_CHECKS);
}

static void codec_hevc_watchdog_arm(struct codec_hevc *hevc)
{
	WRITE_ONCE(hevc->watchdog_armed, true);
	hevc->watchdog_last_lcu = 0;
	hevc->watchdog_last_shift = 0;
	codec_hevc_watchdog_note_progress(hevc);
	mod_delayed_work(system_wq, &hevc->watchdog_work,
			 msecs_to_jiffies(HEVC_WATCHDOG_POLL_MS));
}

static void codec_hevc_watchdog_work(struct work_struct *work)
{
	struct codec_hevc *hevc =
		container_of(to_delayed_work(work), struct codec_hevc,
			     watchdog_work);
	struct amvdec_session *sess = hevc->sess;
	struct amvdec_core *core = sess->core;
	bool abort = false;
	bool has_frame = false;
	bool rearm = false;
	u32 cpsr = 0;
	u32 decode_size = 0;
	u32 lcu = 0;
	u32 mpc = 0;
	u32 mpsr = 0;
	u32 shift = 0;
	u32 slice = 0;
	u32 status = 0;
	u32 wait = 0;

	mutex_lock(&hevc->lock);
	if (READ_ONCE(hevc->watchdog_armed) && hevc->input_pending &&
	    amvdec_session_is_current(sess)) {
		if (!time_after(jiffies,
				READ_ONCE(hevc->watchdog_progress_at) +
				msecs_to_jiffies(HEVC_WATCHDOG_PROGRESS_MS))) {
			rearm = true;
		} else {
			status = amvdec_read_dos(core, HEVC_DEC_STATUS_REG);
			lcu = amvdec_read_dos(core, HEVC_PARSER_LCU_START) &
				0xffffff;
			shift = amvdec_read_dos(core, HEVC_SHIFT_BYTE_COUNT);
		}
		if (!rearm &&
		    (!READ_ONCE(hevc->watchdog_progress_valid) ||
		     lcu != hevc->watchdog_last_lcu ||
		     shift != hevc->watchdog_last_shift)) {
			WRITE_ONCE(hevc->watchdog_progress_valid, true);
			WRITE_ONCE(hevc->watchdog_progress_at, jiffies);
			hevc->watchdog_last_lcu = lcu;
			hevc->watchdog_last_shift = shift;
			WRITE_ONCE(hevc->watchdog_stall_checks,
				   HEVC_WATCHDOG_STALL_CHECKS);
			rearm = true;
		} else if (!rearm &&
			   READ_ONCE(hevc->watchdog_stall_checks) > 1) {
			WRITE_ONCE(hevc->watchdog_stall_checks,
				   READ_ONCE(hevc->watchdog_stall_checks) - 1);
			rearm = true;
		} else if (!rearm) {
			mpsr = amvdec_read_dos(core, HEVC_MPSR);
			cpsr = amvdec_read_dos(core, HEVC_CPSR);
			mpc = amvdec_read_dos(core, HEVC_MPC_E);
			decode_size = amvdec_read_dos(core, HEVC_DECODE_SIZE);
			wait = amvdec_read_dos(core, HEVC_WAIT_FLAG);
			if (hevc->cur_frame)
				slice = hevc->cur_frame->cur_slice_idx;
			WRITE_ONCE(hevc->watchdog_armed, false);
			WRITE_ONCE(sess->hardware_stalled, true);
			hevc->resume_pending = false;
			hevc->input_pending = false;
			hevc->waiting_for_input = false;
			has_frame = hevc->input_has_frame;
			abort = true;
		}
	}
	if (rearm && READ_ONCE(hevc->watchdog_armed))
		mod_delayed_work(system_wq, &hevc->watchdog_work,
				 msecs_to_jiffies(HEVC_WATCHDOG_POLL_MS));
	mutex_unlock(&hevc->lock);

	if (rearm)
		return;
	if (!abort)
		return;

	dev_warn(core->dev,
		 "HEVC firmware made no LCU/input progress for %u ms; aborting session\n",
		 HEVC_WATCHDOG_PROGRESS_MS);
	dev_warn(core->dev,
		 "HEVC timeout state: status=%#x mpsr=%#x cpsr=%#x mpc=%#x lcu=%#x shift=%#x size=%#x wait=%#x frame=%u slice=%u\n",
		 status, mpsr, cpsr, mpc, lcu, shift, decode_size, wait,
		 has_frame, slice);
	amvdec_abort(sess);
	amvdec_m2m_job_yield(sess);
}

static u32 codec_hevc_num_pending_bufs(struct amvdec_session *sess)
{
	struct codec_hevc *hevc;
	struct hevc_frame *frame;
	u32 ret;

	hevc = sess->priv;
	if (!hevc)
		return 0;

	mutex_lock(&hevc->lock);
	ret = hevc->frames_num;
	list_for_each_entry(frame, &hevc->ref_frames_list, list) {
		if (frame->requeued)
			ret++;
	}
	mutex_unlock(&hevc->lock);

	return ret;
}

static bool
codec_hevc_hold_capture_buf(struct amvdec_session *sess,
			    struct vb2_v4l2_buffer *vbuf)
{
	struct codec_hevc *hevc = sess->priv;
	struct hevc_frame *frame;
	bool hold = false;

	if (!hevc)
		return false;

	mutex_lock(&hevc->lock);
	list_for_each_entry(frame, &hevc->ref_frames_list, list) {
		if (frame->vbuf != vbuf)
			continue;

		if (frame->referenced) {
			frame->requeued = true;
			hold = true;
		}
		break;
	}
	mutex_unlock(&hevc->lock);

	return hold;
}

static struct codec_hevc *
codec_hevc_get_context(struct amvdec_session *sess)
{
	struct codec_hevc *hevc = sess->priv;

	if (hevc)
		return hevc;

	hevc = kzalloc_obj(*hevc);
	if (!hevc)
		return NULL;

	INIT_LIST_HEAD(&hevc->ref_frames_list);
	hevc->curr_poc = INVALID_POC;
	hevc->rps_used_bit = RPS_USED_BIT_LEGACY;
	hevc->sess = sess;
	mutex_init(&hevc->lock);
	INIT_DELAYED_WORK(&hevc->watchdog_work, codec_hevc_watchdog_work);
	sess->priv = hevc;

	return hevc;
}

/* Update the L0 and L1 reference lists for a given frame */
static int codec_hevc_update_frame_refs(struct amvdec_session *sess,
					struct hevc_frame *frame)
{
	struct codec_hevc *hevc = sess->priv;
	union rpm_param *params = &hevc->rpm_param;
	u32 rps_used_bit = hevc->rps_used_bit;
	int num_ref_idx_l0_active = params->p.num_ref_idx_l0_active;
	int num_ref_idx_l1_active = params->p.num_ref_idx_l1_active;
	int ref_picset0[MAX_REF_ACTIVE] = { 0 };
	int ref_picset1[MAX_REF_ACTIVE] = { 0 };
	u16 *mod_list = params->p.modification_list;
	int num_neg = 0;
	int num_pos = 0;
	int total_num;
	int i;

	for (i = 0; i < MAX_REF_ACTIVE; i++) {
		frame->ref_poc_list[0][frame->cur_slice_idx][i] = 0;
		frame->ref_poc_list[1][frame->cur_slice_idx][i] = 0;
	}

	for (i = 0; i < MAX_REF_ACTIVE; i++) {
		u16 cur_rps = params->p.CUR_RPS[i];
		int delt = cur_rps & ((1 << (rps_used_bit - 1)) - 1);

		if (cur_rps & 0x8000)
			break;

		if (!((cur_rps >> rps_used_bit) & 1))
			continue;

		if ((cur_rps >> (rps_used_bit - 1)) & 1) {
			ref_picset0[num_neg] =
			       frame->poc - ((1 << (rps_used_bit - 1)) - delt);
			num_neg++;
		} else {
			ref_picset1[num_pos] = frame->poc + delt;
			num_pos++;
		}
	}

	total_num = num_neg + num_pos;

	if (total_num <= 0)
		goto end;

	for (i = 0; i < num_ref_idx_l0_active; i++) {
		int cidx;

		if (params->p.modification_flag & 0x1)
			cidx = mod_list[i];
		else
			cidx = i % total_num;
		if (cidx >= total_num)
			return -EINVAL;

		frame->ref_poc_list[0][frame->cur_slice_idx][i] =
			cidx >= num_neg ? ref_picset1[cidx - num_neg] :
			ref_picset0[cidx];
	}

	if (params->p.slice_type != B_SLICE)
		goto end;

	if (params->p.modification_flag & 0x2) {
		for (i = 0; i < num_ref_idx_l1_active; i++) {
			int cidx;

			if (params->p.modification_flag & 0x1)
				cidx = mod_list[num_ref_idx_l0_active + i];
			else
				cidx = mod_list[i];
			if (cidx >= total_num)
				return -EINVAL;

			frame->ref_poc_list[1][frame->cur_slice_idx][i] =
				(cidx >= num_pos) ? ref_picset0[cidx - num_pos]
				: ref_picset1[cidx];
		}
	} else {
		for (i = 0; i < num_ref_idx_l1_active; i++) {
			int cidx = i % total_num;

			frame->ref_poc_list[1][frame->cur_slice_idx][i] =
				cidx >= num_pos ? ref_picset0[cidx - num_pos] :
				ref_picset1[cidx];
		}
	}

end:
	frame->ref_num[0] = num_ref_idx_l0_active;
	frame->ref_num[1] = num_ref_idx_l1_active;

	dev_dbg(sess->core->dev,
		"Frame %u; slice %u; slice_type %u; num_l0 %u; num_l1 %u\n",
		frame->poc, frame->cur_slice_idx, params->p.slice_type,
		frame->ref_num[0], frame->ref_num[1]);

	return 0;
}

static void codec_hevc_update_ldc_flag(struct codec_hevc *hevc)
{
	struct hevc_frame *frame = hevc->cur_frame;
	u32 slice_type = frame->cur_slice_type;
	u32 slice_idx = frame->cur_slice_idx;
	int i;

	hevc->ldc_flag = 0;

	if (slice_type == I_SLICE)
		return;

	hevc->ldc_flag = 1;
	for (i = 0; (i < frame->ref_num[0]) && hevc->ldc_flag; i++) {
		if (frame->ref_poc_list[0][slice_idx][i] > frame->poc) {
			hevc->ldc_flag = 0;
			break;
		}
	}

	if (slice_type == P_SLICE)
		return;

	for (i = 0; (i < frame->ref_num[1]) && hevc->ldc_flag; i++) {
		if (frame->ref_poc_list[1][slice_idx][i] > frame->poc) {
			hevc->ldc_flag = 0;
			break;
		}
	}
}

/* Tag "old" frames that are no longer referenced */
static void codec_hevc_update_referenced(struct amvdec_session *sess)
{
	struct codec_hevc *hevc = sess->priv;
	union rpm_param *param = &hevc->rpm_param;
	struct hevc_frame *frame;
	u32 rps_used_bit = hevc->rps_used_bit;
	int i;
	u32 curr_poc = hevc->curr_poc;

	list_for_each_entry(frame, &hevc->ref_frames_list, list) {
		int is_referenced = 0;
		u32 poc_tmp;

		if (!frame->referenced)
			continue;

		for (i = 0; i < MAX_REF_ACTIVE; i++) {
			int delt;

			if (param->p.CUR_RPS[i] & 0x8000)
				break;

			delt = param->p.CUR_RPS[i] &
			       ((1 << (rps_used_bit - 1)) - 1);
			if (param->p.CUR_RPS[i] & (1 << (rps_used_bit - 1))) {
				poc_tmp = curr_poc -
				      ((1 << (rps_used_bit - 1)) - delt);
			} else {
				poc_tmp = curr_poc + delt;
			}

			if (poc_tmp == frame->poc) {
				is_referenced = 1;
				break;
			}
		}

		frame->referenced = is_referenced;
		if (!is_referenced && frame->requeued) {
			frame->requeued = false;
			v4l2_m2m_buf_queue(sess->m2m_ctx, frame->vbuf);
		}
	}
}

static struct hevc_frame *
codec_hevc_get_next_ready_frame(struct codec_hevc *hevc)
{
	struct hevc_frame *tmp, *ret = NULL;
	u32 poc = INT_MAX;

	list_for_each_entry(tmp, &hevc->ref_frames_list, list) {
		if ((tmp->poc < poc) && tmp->show) {
			ret = tmp;
			poc = tmp->poc;
		}
	}

	return ret;
}

/* Try to output as many frames as possible */
static void codec_hevc_show_frames(struct amvdec_session *sess)
{
	struct hevc_frame *tmp, *n;
	struct codec_hevc *hevc = sess->priv;

	while ((tmp = codec_hevc_get_next_ready_frame(hevc))) {
		if (hevc->curr_poc &&
		    (hevc->frames_num <= tmp->num_reorder_pic))
			break;

		dev_dbg(sess->core->dev, "DONE frame poc %u; vbuf %u\n",
			tmp->poc, tmp->vbuf->vb2_buf.index);
		if (tmp->timestamp_valid)
			amvdec_dst_buf_done_ts(sess, tmp->vbuf, V4L2_FIELD_NONE,
					       0, &tmp->timestamp);
		else
			amvdec_dst_buf_done_offset(sess, tmp->vbuf, tmp->offset,
						   V4L2_FIELD_NONE, 0, false);

		tmp->show = 0;
		hevc->frames_num--;
	}

	/* clean output frame buffer */
	list_for_each_entry_safe(tmp, n, &hevc->ref_frames_list, list) {
		if (tmp->referenced || tmp->show)
			continue;

		list_del(&tmp->list);
		kfree(tmp);
	}
}

static int
codec_hevc_setup_workspace(struct amvdec_session *sess,
			   struct codec_hevc *hevc)
{
	struct amvdec_core *core = sess->core;
	u32 revision = core->platform->revision;
	dma_addr_t wkaddr;

	if (!hevc->workspace_vaddr) {
		hevc->workspace_vaddr = dma_alloc_coherent(core->dev,
							   SIZE_WORKSPACE,
							   &wkaddr, GFP_KERNEL);
		if (!hevc->workspace_vaddr)
			return -ENOMEM;

		memset(hevc->workspace_vaddr, 0, SIZE_WORKSPACE);
		hevc->workspace_paddr = wkaddr;
	} else {
		wkaddr = hevc->workspace_paddr;
	}

	amvdec_write_dos(core, HEVCD_IPP_LINEBUFF_BASE, wkaddr + IPP_OFFSET);
	amvdec_write_dos(core, HEVC_RPM_BUFFER, wkaddr + RPM_OFFSET);
	amvdec_write_dos(core, HEVC_SHORT_TERM_RPS, wkaddr + SH_TM_RPS_OFFSET);
	amvdec_write_dos(core, HEVC_VPS_BUFFER, wkaddr + VPS_OFFSET);
	amvdec_write_dos(core, HEVC_SPS_BUFFER, wkaddr + SPS_OFFSET);
	amvdec_write_dos(core, HEVC_PPS_BUFFER, wkaddr + PPS_OFFSET);
	amvdec_write_dos(core, HEVC_SAO_UP, wkaddr + SAO_UP_OFFSET);

	if (codec_hevc_use_mmu(revision, sess->pixfmt_cap, hevc->is_10bit)) {
		amvdec_write_dos(core, HEVC_SAO_MMU_VH0_ADDR,
				 wkaddr + MMU_VBH_OFFSET);
		amvdec_write_dos(core, HEVC_SAO_MMU_VH1_ADDR,
				 wkaddr + MMU_VBH_OFFSET + (MMU_VBH_SIZE / 2));

		if (revision >= VDEC_REVISION_G12A)
			amvdec_write_dos(core, HEVC_ASSIST_MMU_MAP_ADDR,
					 hevc->common.mmu_map_paddr);
		else
			amvdec_write_dos(core, H265_MMU_MAP_BUFFER,
					 hevc->common.mmu_map_paddr);
	} else if (revision < VDEC_REVISION_G12A) {
		amvdec_write_dos(core, HEVC_STREAM_SWAP_BUFFER,
				 wkaddr + SWAP_BUF_OFFSET);
		amvdec_write_dos(core, HEVC_STREAM_SWAP_BUFFER2,
				 wkaddr + SWAP_BUF2_OFFSET);
	}

	amvdec_write_dos(core, HEVC_SCALELUT, wkaddr + SCALELUT_OFFSET);
	amvdec_write_dos(core, HEVC_DBLK_CFG4, wkaddr + DBLK_PARA_OFFSET);
	amvdec_write_dos(core, HEVC_DBLK_CFG5, wkaddr + DBLK_DATA_OFFSET);
	if (revision >= VDEC_REVISION_G12A)
		amvdec_write_dos(core, HEVC_DBLK_CFGE,
				 wkaddr + DBLK_DATA2_OFFSET);

	amvdec_write_dos(core, LMEM_DUMP_ADR, wkaddr + LMEM_OFFSET);

	return 0;
}

static int codec_hevc_start(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct amvdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;
	struct codec_hevc *hevc;
	bool multi = codec_ops->irq == AMVDEC_IRQ_MBOX0;
	bool new_session = false;
	u32 val;
	int i;
	int ret;

	new_session = !sess->priv;
	hevc = codec_hevc_get_context(sess);
	if (!hevc)
		return -ENOMEM;
	hevc->rps_used_bit = multi ? RPS_USED_BIT_MULTI :
					 RPS_USED_BIT_LEGACY;

	ret = codec_hevc_setup_workspace(sess, hevc);
	if (ret)
		goto free_hevc;
	if (multi && !hevc->reset_buffers && hevc->width &&
	    hevc->common.ref_buffer_count) {
		codec_hevc_restore_buffers(sess, &hevc->common,
					   hevc->is_10bit);
		codec_hevc_setup_decode_head(sess, hevc->is_10bit);
	}

	val = BIT(0); /* stream_fetch_enable */
	if (core->platform->revision >= VDEC_REVISION_G12A)
		val |= (0xf << 25); /* arwlen_axi_max */
	amvdec_write_dos_bits(core, HEVC_STREAM_CONTROL, val);

	val = amvdec_read_dos(core, HEVC_PARSER_INT_CONTROL) & 0x03ffffff;
	val |= (3 << 29) | BIT(27) | BIT(24) | BIT(22) | BIT(7) | BIT(4) |
	       BIT(0);
	amvdec_write_dos(core, HEVC_PARSER_INT_CONTROL, val);
	amvdec_write_dos_bits(core, HEVC_SHIFT_STATUS, BIT(1) | BIT(0));
	amvdec_write_dos(core, HEVC_SHIFT_CONTROL,
			 (3 << 6) | BIT(5) | BIT(2) | BIT(0));
	amvdec_write_dos(core, HEVC_CABAC_CONTROL, 1);
	amvdec_write_dos(core, HEVC_PARSER_CORE_CONTROL, 1);
	amvdec_write_dos(core, HEVC_DEC_STATUS_REG, 0);

	amvdec_write_dos(core, HEVC_IQIT_SCALELUT_WR_ADDR, 0);
	for (i = 0; i < 1024; ++i)
		amvdec_write_dos(core, HEVC_IQIT_SCALELUT_DATA, 0);

	amvdec_write_dos(core, HEVC_DECODE_SIZE, 0);

	amvdec_write_dos(core, HEVC_PARSER_CMD_WRITE, BIT(16));
	for (i = 0; i < ARRAY_SIZE(vdec_hevc_parser_cmd); ++i)
		amvdec_write_dos(core, HEVC_PARSER_CMD_WRITE,
				 vdec_hevc_parser_cmd[i]);

	amvdec_write_dos(core, HEVC_PARSER_CMD_SKIP_0, PARSER_CMD_SKIP_CFG_0);
	amvdec_write_dos(core, HEVC_PARSER_CMD_SKIP_1, PARSER_CMD_SKIP_CFG_1);
	amvdec_write_dos(core, HEVC_PARSER_CMD_SKIP_2, PARSER_CMD_SKIP_CFG_2);
	amvdec_write_dos(core, HEVC_PARSER_IF_CONTROL,
			 BIT(5) | BIT(2) | BIT(0));

	amvdec_write_dos(core, HEVCD_IPP_TOP_CNTL, BIT(0));
	amvdec_write_dos(core, HEVCD_IPP_TOP_CNTL, BIT(1));

	amvdec_write_dos(core, HEVC_WAIT_FLAG, 1);

	/* Clear and enable the firmware mailbox interrupt. */
	if (multi) {
		amvdec_write_dos(core, HEVC_ASSIST_MBOX0_CLR_REG, 1);
		amvdec_write_dos(core, HEVC_ASSIST_MBOX0_MASK, 1);
	} else {
		amvdec_write_dos(core, HEVC_ASSIST_MBOX1_CLR_REG, 1);
		amvdec_write_dos(core, HEVC_ASSIST_MBOX1_MASK, 1);
	}
	/* disable PSCALE for hardware sharing */
	amvdec_write_dos(core, HEVC_PSCALE_CTRL, 0);
	/* Let the uCode do all the parsing. */
	amvdec_write_dos(core, NAL_SEARCH_CTL,
			 multi && hevc->nal_search_done ? NAL_SEARCH_AUTO :
			 multi ? NAL_SEARCH_MULTI : NAL_SEARCH_LEGACY);

	amvdec_write_dos(core, DECODE_STOP_POS, 0);
	if (multi) {
		amvdec_write_dos(core, HEVC_DECODE_MODE,
				 DECODE_MODE_MBOX0 |
				 (hevc->start_decoding_flag << 16) |
				 DECODE_MODE_MULTI_FRAMEBASE);
		amvdec_write_dos(core, HEVC_DECODE_MODE2, hevc->rps_set_id);
	} else {
		amvdec_write_dos(core, HEVC_DECODE_MODE, DECODE_MODE_SINGLE);
		amvdec_write_dos(core, HEVC_DECODE_MODE2, 0);
	}

	/* AUX buffers */
	if (!hevc->aux_vaddr) {
		hevc->aux_vaddr = dma_alloc_coherent(core->dev, SIZE_AUX,
						     &hevc->aux_paddr,
						     GFP_KERNEL);
		if (!hevc->aux_vaddr) {
			ret = -ENOMEM;
			goto free_hevc;
		}
	}

	amvdec_write_dos(core, HEVC_AUX_ADR, hevc->aux_paddr);
	amvdec_write_dos(core, HEVC_AUX_DATA_SIZE,
			 (((SIZE_AUX) >> 4) << 16) | 0);
	if (multi)
		amvdec_write_dos(core, HEVC_STREAM_SWAP_BUFFER2,
				 hevc->fw_swap_paddr);
	return 0;

free_hevc:
	if (new_session) {
		if (hevc->workspace_vaddr)
			dma_free_coherent(core->dev, SIZE_WORKSPACE,
					  hevc->workspace_vaddr,
					  hevc->workspace_paddr);
		mutex_destroy(&hevc->lock);
		kfree(hevc);
		sess->priv = NULL;
	}
	return ret;
}

static void codec_hevc_flush_output_mode(struct amvdec_session *sess,
					 bool recycle_held)
{
	struct codec_hevc *hevc = sess->priv;
	struct hevc_frame *tmp, *n;

	while ((tmp = codec_hevc_get_next_ready_frame(hevc))) {
		if (tmp->timestamp_valid)
			amvdec_dst_buf_done_ts(sess, tmp->vbuf, V4L2_FIELD_NONE,
					       0, &tmp->timestamp);
		else
			amvdec_dst_buf_done(sess, tmp->vbuf, V4L2_FIELD_NONE, 0);
		tmp->show = 0;
		hevc->frames_num--;
	}

	list_for_each_entry_safe(tmp, n, &hevc->ref_frames_list, list) {
		if (tmp->requeued) {
			if (recycle_held)
				v4l2_m2m_buf_queue(sess->m2m_ctx, tmp->vbuf);
			else
				v4l2_m2m_buf_done(tmp->vbuf,
						  VB2_BUF_STATE_ERROR);
		}

		list_del(&tmp->list);
		kfree(tmp);
	}

	hevc->frames_num = 0;
	hevc->cur_frame = NULL;
	hevc->col_frame = NULL;
}

static void codec_hevc_flush_output(struct amvdec_session *sess)
{
	codec_hevc_flush_output_mode(sess, false);
}

static void codec_hevc_flush_source_change(struct amvdec_session *sess)
{
	/* Held reference-only buffers remain queued; they are not decoded frames. */
	codec_hevc_flush_output_mode(sess, true);
}

static void codec_hevc_signal_source_change(struct amvdec_session *sess)
{
	struct vb2_v4l2_buffer *vbuf;
	unsigned int i;

	vbuf = v4l2_m2m_dst_buf_remove(sess->m2m_ctx);
	if (!vbuf) {
		dev_warn(sess->core->dev,
			 "No capture buffer available for source change\n");
		return;
	}

	for (i = 0; i < vbuf->vb2_buf.num_planes; i++)
		vb2_set_plane_payload(&vbuf->vb2_buf, i, 0);
	vbuf->sequence = sess->sequence_cap++;
	vbuf->field = V4L2_FIELD_NONE;
	v4l2_m2m_last_buffer_done(sess->m2m_ctx, vbuf);
	sess->sequence_cap = 0;
}

static int codec_hevc_prepare_firmware(struct amvdec_session *sess,
				       const u8 *data, u32 len)
{
	struct codec_hevc *hevc = sess->priv;
	bool new_session = false;
	bool new_swap = false;

	if (len < FW_SWAP_OFFSET + SIZE_FW_SWAP) {
		dev_err(sess->core->dev,
			"HEVC multi firmware is too small: %u bytes\n", len);
		return -EINVAL;
	}

	new_session = !hevc;
	hevc = codec_hevc_get_context(sess);
	if (!hevc)
		return -ENOMEM;

	if (!hevc->fw_swap_vaddr) {
		hevc->fw_swap_vaddr = dma_alloc_coherent(sess->core->dev,
							 SIZE_FW_SWAP,
							 &hevc->fw_swap_paddr,
							 GFP_KERNEL);
		if (!hevc->fw_swap_vaddr) {
			if (new_session) {
				mutex_destroy(&hevc->lock);
				kfree(hevc);
				sess->priv = NULL;
			}
			return -ENOMEM;
		}
		new_swap = true;
	}

	if (new_swap)
		memcpy(hevc->fw_swap_vaddr, data + FW_SWAP_OFFSET, SIZE_FW_SWAP);
	return 0;
}

static int codec_hevc_keep_context(struct amvdec_session *sess)
{
	struct codec_hevc *hevc = sess->priv;

	if (hevc)
		codec_hevc_watchdog_cancel_sync(hevc);
	return 0;
}

static int codec_hevc_stop(struct amvdec_session *sess)
{
	struct codec_hevc *hevc = sess->priv;
	struct amvdec_core *core = sess->core;

	codec_hevc_watchdog_cancel_sync(hevc);
	mutex_lock(&hevc->lock);
	codec_hevc_flush_output(sess);

	if (hevc->workspace_vaddr)
		dma_free_coherent(core->dev, SIZE_WORKSPACE,
				  hevc->workspace_vaddr,
				  hevc->workspace_paddr);

	if (hevc->aux_vaddr)
		dma_free_coherent(core->dev, SIZE_AUX,
				  hevc->aux_vaddr, hevc->aux_paddr);

	codec_hevc_free_fbc_buffers(sess, &hevc->common);
	mutex_unlock(&hevc->lock);
	mutex_destroy(&hevc->lock);

	return 0;
}

static void codec_hevc_release(struct amvdec_session *sess)
{
	struct codec_hevc *hevc = sess->priv;
	struct amvdec_core *core = sess->core;

	if (!hevc)
		return;

	codec_hevc_watchdog_cancel_sync(hevc);
	mutex_lock(&hevc->lock);
	codec_hevc_flush_output(sess);

	if (hevc->workspace_vaddr)
		dma_free_coherent(core->dev, SIZE_WORKSPACE,
				  hevc->workspace_vaddr,
				  hevc->workspace_paddr);
	if (hevc->aux_vaddr)
		dma_free_coherent(core->dev, SIZE_AUX,
				  hevc->aux_vaddr, hevc->aux_paddr);
	if (hevc->fw_swap_vaddr)
		dma_free_coherent(core->dev, SIZE_FW_SWAP,
				  hevc->fw_swap_vaddr,
				  hevc->fw_swap_paddr);
	if (hevc->input_vbuf) {
		v4l2_m2m_buf_done(hevc->input_vbuf, VB2_BUF_STATE_ERROR);
		hevc->input_vbuf = NULL;
	}

	codec_hevc_free_fbc_buffers(sess, &hevc->common);
	mutex_unlock(&hevc->lock);
	mutex_destroy(&hevc->lock);
	kfree(hevc);
	sess->priv = NULL;
}

static struct hevc_frame *
codec_hevc_get_frame_by_poc(struct codec_hevc *hevc, u32 poc)
{
	struct hevc_frame *tmp;

	list_for_each_entry(tmp, &hevc->ref_frames_list, list) {
		if (tmp->poc == poc)
			return tmp;
	}

	return NULL;
}

static struct hevc_frame *
codec_hevc_nearest_frame(struct codec_hevc *hevc, u32 poc, bool prefer_past)
{
	struct hevc_frame *past = NULL;
	struct hevc_frame *future = NULL;
	struct hevc_frame *tmp;
	u64 past_delta = U64_MAX;
	u64 future_delta = U64_MAX;
	s64 target = (s32)poc;

	list_for_each_entry(tmp, &hevc->ref_frames_list, list) {
		s64 delta;

		if (tmp == hevc->cur_frame)
			continue;

		delta = (s64)(s32)tmp->poc - target;
		if (delta < 0 && -delta < past_delta) {
			past_delta = -delta;
			past = tmp;
		} else if (delta > 0 && delta < future_delta) {
			future_delta = delta;
			future = tmp;
		}
	}

	if (prefer_past)
		return past ? past : future;

	return future ? future : past;
}

static void codec_hevc_conceal_missing_refs(struct amvdec_session *sess,
					    struct hevc_frame *frame)
{
	struct codec_hevc *hevc = sess->priv;
	u32 slice_idx = frame->cur_slice_idx;
	u32 list_count;
	u32 list;

	if (frame->cur_slice_type == I_SLICE)
		return;
	list_count = frame->cur_slice_type == B_SLICE ? 2 : 1;

	for (list = 0; list < list_count; list++) {
		u32 *ref_poc_list = frame->ref_poc_list[list][slice_idx];
		u32 i;

		for (i = 0; i < frame->ref_num[list]; i++) {
			struct hevc_frame *ref_frame;

			ref_frame = codec_hevc_get_frame_by_poc(hevc, ref_poc_list[i]);
			if (ref_frame)
				continue;

			ref_frame = codec_hevc_nearest_frame(hevc, ref_poc_list[i], !list);
			if (!ref_frame)
				ref_frame = frame;
			dev_warn_ratelimited(sess->core->dev,
					     "Missing L%u reference POC %d, using POC %d\n",
					     list, (s32)ref_poc_list[i],
					     (s32)ref_frame->poc);
			ref_poc_list[i] = ref_frame->poc;
		}
	}
}

static struct hevc_frame *
codec_hevc_prepare_new_frame(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct hevc_frame *new_frame = NULL;
	struct codec_hevc *hevc = sess->priv;
	struct vb2_v4l2_buffer *vbuf;
	union rpm_param *params = &hevc->rpm_param;

	new_frame = kzalloc(sizeof(*new_frame), GFP_KERNEL);
	if (!new_frame)
		return NULL;

	vbuf = v4l2_m2m_dst_buf_remove(sess->m2m_ctx);
	if (!vbuf) {
		dev_err(sess->core->dev, "No dst buffer available\n");
		kfree(new_frame);
		return NULL;
	}

	new_frame->vbuf = vbuf;
	new_frame->referenced = 1;
	new_frame->show = 1;
	new_frame->poc = hevc->curr_poc;
	new_frame->cur_slice_type = params->p.slice_type;
	new_frame->num_reorder_pic = params->p.sps_num_reorder_pics_0;
	if (sess->fmt_out->codec_ops->irq == AMVDEC_IRQ_MBOX0) {
		if (amvdec_take_ts(sess, &new_frame->timestamp)) {
			dev_err(core->dev_dec,
				"No timestamp for capture buffer %u\n",
				vbuf->vb2_buf.index);
			v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
			kfree(new_frame);
			return NULL;
		}
		new_frame->timestamp_valid = true;
		hevc->input_has_frame = true;
	} else {
		new_frame->offset = amvdec_read_dos(core,
						    HEVC_SHIFT_BYTE_COUNT);
	}

	list_add_tail(&new_frame->list, &hevc->ref_frames_list);
	hevc->frames_num++;

	return new_frame;
}

static void
codec_hevc_set_sao(struct amvdec_session *sess, struct hevc_frame *frame)
{
	struct amvdec_core *core = sess->core;
	struct codec_hevc *hevc = sess->priv;
	struct vb2_buffer *vb = &frame->vbuf->vb2_buf;
	union rpm_param *param = &hevc->rpm_param;
	bool multi = sess->fmt_out->codec_ops->irq == AMVDEC_IRQ_MBOX0;
	u32 pic_height_cu =
		(hevc->height + hevc->lcu_size - 1) / hevc->lcu_size;
	u32 sao_mem_unit = (hevc->lcu_size == 16 ? 9 :
			   hevc->lcu_size == 32 ? 14 : 24) << 4;
	u32 sao_vb_size = (sao_mem_unit + (2 << 4)) * pic_height_cu;
	u32 misc_flag0 = param->p.misc_flag0;
	dma_addr_t buf_y_paddr;
	dma_addr_t buf_u_v_paddr;
	u32 slice_deblocking_filter_disabled_flag;
	u32 val, val_2;

	val = (amvdec_read_dos(core, HEVC_SAO_CTRL0) & ~0xf) |
	      ilog2(hevc->lcu_size);
	amvdec_write_dos(core, HEVC_SAO_CTRL0, val);

	amvdec_write_dos(core, HEVC_SAO_PIC_SIZE,
			 hevc->width | (hevc->height << 16));
	amvdec_write_dos(core, HEVC_SAO_PIC_SIZE_LCU,
			 (hevc->lcu_x_num - 1) | (hevc->lcu_y_num - 1) << 16);

	if (codec_hevc_use_mmu(core->platform->revision, sess->pixfmt_cap,
			       hevc->is_10bit))
		buf_y_paddr = 0;
	else if (codec_hevc_use_downsample(sess->pixfmt_cap, hevc->is_10bit))
		buf_y_paddr =
		     hevc->common.fbc_buffer_paddr[vb->index];
	else
		buf_y_paddr =
		       vb2_dma_contig_plane_dma_addr(vb, 0);

	if (codec_hevc_use_fbc(sess->pixfmt_cap, hevc->is_10bit)) {
		val = amvdec_read_dos(core, HEVC_SAO_CTRL5) & ~0xff0000;
		amvdec_write_dos(core, HEVC_SAO_CTRL5, val);
		amvdec_write_dos(core, HEVC_CM_BODY_START_ADDR, buf_y_paddr);
	}

	if (sess->pixfmt_cap == V4L2_PIX_FMT_NV12M) {
		buf_y_paddr =
		       vb2_dma_contig_plane_dma_addr(vb, 0);
		buf_u_v_paddr =
		       vb2_dma_contig_plane_dma_addr(vb, 1);
		amvdec_write_dos(core, HEVC_SAO_Y_START_ADDR, buf_y_paddr);
		amvdec_write_dos(core, HEVC_SAO_C_START_ADDR, buf_u_v_paddr);
		amvdec_write_dos(core, HEVC_SAO_Y_WPTR, buf_y_paddr);
		amvdec_write_dos(core, HEVC_SAO_C_WPTR, buf_u_v_paddr);
	}

	if (codec_hevc_use_mmu(core->platform->revision, sess->pixfmt_cap,
			       hevc->is_10bit)) {
		dma_addr_t header_adr = vb2_dma_contig_plane_dma_addr(vb, 0);

		if (codec_hevc_use_downsample(sess->pixfmt_cap, hevc->is_10bit))
			header_adr = hevc->common.mmu_header_paddr[vb->index];
		amvdec_write_dos(core, HEVC_CM_HEADER_START_ADDR, header_adr);
		/* use HEVC_CM_HEADER_START_ADDR */
		amvdec_write_dos_bits(core, HEVC_SAO_CTRL5, BIT(10));
		amvdec_write_dos_bits(core, HEVC_SAO_CTRL9, BIT(0));
	}

	amvdec_write_dos(core, HEVC_SAO_Y_LENGTH,
			 amvdec_get_output_size(sess));
	amvdec_write_dos(core, HEVC_SAO_C_LENGTH,
			 (amvdec_get_output_size(sess) / 2));

	if (frame->cur_slice_idx == 0) {
		if (core->platform->revision >= VDEC_REVISION_G12A) {
			if (core->platform->revision >= VDEC_REVISION_SM1)
				val = 0xfc << 8;
			else
				val = 0x54 << 8;

			/* enable first, compressed write */
			if (codec_hevc_use_fbc(sess->pixfmt_cap,
					       hevc->is_10bit))
				val |= BIT(8);

			/* enable second, uncompressed write */
			if (sess->pixfmt_cap == V4L2_PIX_FMT_NV12M)
				val |= BIT(9);

			/* dblk pipeline mode=1 for performance */
			if (hevc->width >= 1280)
				val |= BIT(4);

			amvdec_write_dos(core, HEVC_DBLK_CFGB, val);
			amvdec_write_dos(core, HEVC_DBLK_STS1 + 16, BIT(28));
		}

		amvdec_write_dos(core, HEVC_DBLK_CFG2,
				 hevc->width | (hevc->height << 16));

		val = 0;
		if ((misc_flag0 >> PCM_ENABLE_FLAG_BIT) & 0x1)
			val |= ((misc_flag0 >>
				 PCM_LOOP_FILTER_DISABLED_FLAG_BIT) & 0x1) << 3;

		val |= (param->p.pps_cb_qp_offset & 0x1f) << 4;
		val |= (param->p.pps_cr_qp_offset & 0x1f) << 9;
		val |= (hevc->lcu_size == 64) ? 0 :
		       ((hevc->lcu_size == 32) ? 1 : 2);
		amvdec_write_dos(core, HEVC_DBLK_CFG1, val);
	}

	val = amvdec_read_dos(core, HEVC_SAO_CTRL1);
	if (multi) {
		val &= ~0xfff3;
		val |= 0xff0; /* Set endianness for 2-byte swaps (NV12). */
		/* Linear pitch ALIGN(width, 32); 2 doubles it and overruns the plane. */
		val |= 1 << 14;
		if (!codec_hevc_use_fbc(sess->pixfmt_cap, hevc->is_10bit))
			val |= BIT(0); /* Disable compressed output. */
	} else {
		val &= ~0x3ff3;
		val |= 0xff0; /* Set endianness for 2-byte swaps (NV12). */
		if (core->platform->revision < VDEC_REVISION_G12A) {
			if (!codec_hevc_use_fbc(sess->pixfmt_cap, hevc->is_10bit))
				val |= BIT(0); /* Disable compressed output. */
			/* TOFIX: Handle Amlogic Framebuffer compression */
		}
	}

	amvdec_write_dos(core, HEVC_SAO_CTRL1, val);

	if (!codec_hevc_use_fbc(sess->pixfmt_cap, hevc->is_10bit)) {
		/* no downscale for NV12 */
		val = amvdec_read_dos(core, HEVC_SAO_CTRL5) & ~0xff0000;
		amvdec_write_dos(core, HEVC_SAO_CTRL5, val);
	}

	val = amvdec_read_dos(core, HEVCD_IPP_AXIIF_CONFIG);
	if (multi) {
		val &= ~(BIT(12) | GENMASK(9, 8) | GENMASK(5, 0));
		val |= (1 << 8) | 0xf; /* NV12, linear, same pitch as SAO_CTRL1. */
	} else {
		val &= ~0x30;
		val |= 0xf;
	}
	amvdec_write_dos(core, HEVCD_IPP_AXIIF_CONFIG, val);

	val = 0;
	val_2 = amvdec_read_dos(core, HEVC_SAO_CTRL0);
	val_2 &= (~0x300);

	slice_deblocking_filter_disabled_flag = (misc_flag0 >>
			SLICE_DEBLOCKING_FILTER_DISABLED_FLAG_BIT) & 0x1;
	if ((misc_flag0 & (1 << DEBLOCKING_FILTER_OVERRIDE_ENABLED_FLAG_BIT)) &&
	    (misc_flag0 & (1 << DEBLOCKING_FILTER_OVERRIDE_FLAG_BIT))) {
		val |= slice_deblocking_filter_disabled_flag << 2;

		if (!slice_deblocking_filter_disabled_flag) {
			val |= (param->p.slice_beta_offset_div2 & 0xf) << 3;
			val |= (param->p.slice_tc_offset_div2 & 0xf) << 7;
		}
	} else {
		val |=
			((misc_flag0 >>
			  PPS_DEBLOCKING_FILTER_DISABLED_FLAG_BIT) & 0x1) << 2;

		if (((misc_flag0 >> PPS_DEBLOCKING_FILTER_DISABLED_FLAG_BIT) &
			0x1) == 0) {
			val |= (param->p.pps_beta_offset_div2 & 0xf) << 3;
			val |= (param->p.pps_tc_offset_div2 & 0xf) << 7;
		}
	}
	if ((misc_flag0 & (1 << PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT)) &&
	    ((misc_flag0 & (1 << SLICE_SAO_LUMA_FLAG_BIT)) ||
	   (misc_flag0 & (1 << SLICE_SAO_CHROMA_FLAG_BIT)) ||
	   (!slice_deblocking_filter_disabled_flag))) {
		val |=
			((misc_flag0 >>
			  SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT)
			 & 0x1)	<< 1;
		val_2 |=
			((misc_flag0 >>
			  SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT)
			& 0x1) << 9;
	} else {
		val |=
			((misc_flag0 >>
			  PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT)
			 & 0x1) << 1;
		val_2 |=
			((misc_flag0 >>
			  PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT)
			 & 0x1) << 9;
	}

	amvdec_write_dos(core, HEVC_DBLK_CFG9, val);
	amvdec_write_dos(core, HEVC_SAO_CTRL0, val_2);

	amvdec_write_dos(core, HEVC_sao_mem_unit, sao_mem_unit);
	amvdec_write_dos(core, HEVC_SAO_ABV,
			 hevc->workspace_paddr + SAO_ABV_OFFSET);
	amvdec_write_dos(core, HEVC_sao_vb_size, sao_vb_size);
	amvdec_write_dos(core, HEVC_SAO_VB,
			 hevc->workspace_paddr + SAO_VB_OFFSET);
}

static dma_addr_t codec_hevc_get_frame_mv_paddr(struct codec_hevc *hevc,
						struct hevc_frame *frame)
{
	return hevc->workspace_paddr + MPRED_MV_OFFSET +
		(frame->vbuf->vb2_buf.index * MPRED_MV_BUF_SIZE);
}

static void
codec_hevc_set_mpred_ctrl(struct amvdec_core *core, struct codec_hevc *hevc)
{
	union rpm_param *param = &hevc->rpm_param;
	u32 slice_type = param->p.slice_type;
	u32 lcu_size_log2 = ilog2(hevc->lcu_size);
	u32 val;

	val = slice_type |
	      MPRED_CTRL0_ABOVE_EN |
	      MPRED_CTRL0_MV_WR_EN |
	      MPRED_CTRL0_BUF_LINEAR |
	      (lcu_size_log2 << 16) |
	      (3 << 20) | /* cu_size_log2 */
	      (param->p.log2_parallel_merge_level << 24);

	if (slice_type != I_SLICE)
		val |= MPRED_CTRL0_MV_RD_EN;

	if (param->p.collocated_from_l0_flag)
		val |= MPRED_CTRL0_COL_FROM_L0;

	if (param->p.slice_temporal_mvp_enable_flag)
		val |= MPRED_CTRL0_TMVP;

	if (hevc->ldc_flag)
		val |= MPRED_CTRL0_LDC;

	if (param->p.dependent_slice_segment_flag)
		val |= MPRED_CTRL0_NEW_SLI_SEG;

	if (param->p.slice_segment_address == 0)
		val |= MPRED_CTRL0_NEW_PIC |
		       MPRED_CTRL0_NEW_TILE;

	amvdec_write_dos(core, HEVC_MPRED_CTRL0, val);

	val = (5 - param->p.five_minus_max_num_merge_cand) |
	      (AMVP_MAX_NUM_CANDS << 4) |
	      (AMVP_MAX_NUM_CANDS_MEM << 8) |
	      (NUM_CHROMA_MODE << 12) |
	      (DM_CHROMA_IDX << 16);
	amvdec_write_dos(core, HEVC_MPRED_CTRL1, val);
}

static void codec_hevc_set_mpred_mv(struct amvdec_core *core,
				    struct codec_hevc *hevc,
				    struct hevc_frame *frame,
				    struct hevc_frame *col_frame)
{
	union rpm_param *param = &hevc->rpm_param;
	u32 lcu_size_log2 = ilog2(hevc->lcu_size);
	u32 mv_mem_unit = lcu_size_log2 == 6 ? 0x200 :
			  lcu_size_log2 == 5 ? 0x80 : 0x20;
	dma_addr_t col_mv_rd_start_addr, col_mv_rd_ptr, col_mv_rd_end_addr;
	dma_addr_t mpred_mv_wr_ptr;

	amvdec_read_dos(core, HEVC_MPRED_CURR_LCU);

	col_mv_rd_start_addr = codec_hevc_get_frame_mv_paddr(hevc, col_frame);
	mpred_mv_wr_ptr = codec_hevc_get_frame_mv_paddr(hevc, frame) +
			  (hevc->slice_addr * mv_mem_unit);
	col_mv_rd_ptr = col_mv_rd_start_addr +
			(hevc->slice_addr * mv_mem_unit);
	col_mv_rd_end_addr = col_mv_rd_start_addr +
			     (hevc->lcu_total * mv_mem_unit);

	amvdec_write_dos(core, HEVC_MPRED_MV_WR_START_ADDR,
			 codec_hevc_get_frame_mv_paddr(hevc, frame));
	amvdec_write_dos(core, HEVC_MPRED_MV_RD_START_ADDR,
			 col_mv_rd_start_addr);

	if (param->p.slice_segment_address == 0) {
		amvdec_write_dos(core, HEVC_MPRED_ABV_START_ADDR,
				 hevc->workspace_paddr + MPRED_ABV_OFFSET);
		amvdec_write_dos(core, HEVC_MPRED_MV_WPTR, mpred_mv_wr_ptr);
		amvdec_write_dos(core, HEVC_MPRED_MV_RPTR,
				 col_mv_rd_start_addr);
	} else {
		amvdec_write_dos(core, HEVC_MPRED_MV_RPTR, col_mv_rd_ptr);
	}

	amvdec_write_dos(core, HEVC_MPRED_MV_RD_END_ADDR, col_mv_rd_end_addr);
}

/* Update motion prediction with the current slice */
static void codec_hevc_set_mpred(struct amvdec_session *sess,
				 struct hevc_frame *frame,
				 struct hevc_frame *col_frame)
{
	struct amvdec_core *core = sess->core;
	struct codec_hevc *hevc = sess->priv;
	u32 *ref_num = frame->ref_num;
	u32 *ref_poc_l0 = frame->ref_poc_list[0][frame->cur_slice_idx];
	u32 *ref_poc_l1 = frame->ref_poc_list[1][frame->cur_slice_idx];
	u32 val;
	int i;

	codec_hevc_set_mpred_ctrl(core, hevc);
	codec_hevc_set_mpred_mv(core, hevc, frame, col_frame);

	amvdec_write_dos(core, HEVC_MPRED_PIC_SIZE,
			 hevc->width | (hevc->height << 16));

	val = ((hevc->lcu_x_num - 1) | (hevc->lcu_y_num - 1) << 16);
	amvdec_write_dos(core, HEVC_MPRED_PIC_SIZE_LCU, val);

	amvdec_write_dos(core, HEVC_MPRED_REF_NUM,
			 (ref_num[1] << 8) | ref_num[0]);
	amvdec_write_dos(core, HEVC_MPRED_REF_EN_L0, (1 << ref_num[0]) - 1);
	amvdec_write_dos(core, HEVC_MPRED_REF_EN_L1, (1 << ref_num[1]) - 1);

	amvdec_write_dos(core, HEVC_MPRED_CUR_POC, hevc->curr_poc);
	amvdec_write_dos(core, HEVC_MPRED_COL_POC, hevc->col_poc);

	for (i = 0; i < MAX_REF_ACTIVE; ++i) {
		amvdec_write_dos(core, HEVC_MPRED_L0_REF00_POC + i * 4,
				 ref_poc_l0[i]);
		amvdec_write_dos(core, HEVC_MPRED_L1_REF00_POC + i * 4,
				 ref_poc_l1[i]);
	}
}

/*  motion compensation reference cache controller */
static void codec_hevc_set_mcrcc(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_hevc *hevc = sess->priv;
	u32 val, val_2;
	int l0_cnt = 0;
	int l1_cnt = 0x7fff;

	if (!codec_hevc_use_fbc(sess->pixfmt_cap, hevc->is_10bit)) {
		l0_cnt = hevc->cur_frame->ref_num[0];
		l1_cnt = hevc->cur_frame->ref_num[1];
	}

	if (hevc->cur_frame->cur_slice_type == I_SLICE) {
		amvdec_write_dos(core, HEVCD_MCRCC_CTL1, 0);
		return;
	}

	if (hevc->cur_frame->cur_slice_type == P_SLICE) {
		amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR,
				 BIT(1));
		val = amvdec_read_dos(core, HEVCD_MPP_ANC_CANVAS_DATA_ADDR);
		val &= 0xffff;
		val |= (val << 16);
		amvdec_write_dos(core, HEVCD_MCRCC_CTL2, val);

		if (l0_cnt == 1) {
			amvdec_write_dos(core, HEVCD_MCRCC_CTL3, val);
		} else {
			val = amvdec_read_dos(core,
					      HEVCD_MPP_ANC_CANVAS_DATA_ADDR);
			val &= 0xffff;
			val |= (val << 16);
			amvdec_write_dos(core, HEVCD_MCRCC_CTL3, val);
		}
	} else { /* B_SLICE */
		amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, 0);
		val = amvdec_read_dos(core, HEVCD_MPP_ANC_CANVAS_DATA_ADDR);
		val &= 0xffff;
		val |= (val << 16);
		amvdec_write_dos(core, HEVCD_MCRCC_CTL2, val);

		amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR,
				 BIT(12) | BIT(1));
		val_2 = amvdec_read_dos(core, HEVCD_MPP_ANC_CANVAS_DATA_ADDR);
		val_2 &= 0xffff;
		val_2 |= (val_2 << 16);
		if (val == val_2 && l1_cnt > 1) {
			val_2 = amvdec_read_dos(core,
						HEVCD_MPP_ANC_CANVAS_DATA_ADDR);
			val_2 &= 0xffff;
			val_2 |= (val_2 << 16);
		}
		amvdec_write_dos(core, HEVCD_MCRCC_CTL3, val);
	}

	/* enable mcrcc progressive-mode */
	amvdec_write_dos(core, HEVCD_MCRCC_CTL1, 0xff0);
}

static void codec_hevc_set_ref_list(struct amvdec_session *sess,
				    u32 ref_num, u32 *ref_poc_list)
{
	struct codec_hevc *hevc = sess->priv;
	struct hevc_frame *ref_frame;
	struct amvdec_core *core = sess->core;
	int i;
	u32 buf_id_y;
	u32 buf_id_uv;

	for (i = 0; i < ref_num; i++) {
		ref_frame = codec_hevc_get_frame_by_poc(hevc, ref_poc_list[i]);

		if (!ref_frame)
			ref_frame = hevc->cur_frame;

		if (codec_hevc_use_fbc(sess->pixfmt_cap, hevc->is_10bit)) {
			buf_id_y = ref_frame->vbuf->vb2_buf.index;
			buf_id_uv = buf_id_y;
		} else {
			buf_id_y = ref_frame->vbuf->vb2_buf.index * 2;
			buf_id_uv = buf_id_y + 1;
		}

		amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_DATA_ADDR,
				 (buf_id_uv << 16) |
				 (buf_id_uv << 8) |
				 buf_id_y);
	}
}

static void codec_hevc_set_mc(struct amvdec_session *sess,
			      struct hevc_frame *frame)
{
	struct amvdec_core *core = sess->core;

	if (frame->cur_slice_type == I_SLICE)
		return;

	amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, 1);
	codec_hevc_set_ref_list(sess, frame->ref_num[0],
				frame->ref_poc_list[0][frame->cur_slice_idx]);

	if (frame->cur_slice_type == P_SLICE)
		return;

	amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR,
			 BIT(12) | BIT(0));
	codec_hevc_set_ref_list(sess, frame->ref_num[1],
				frame->ref_poc_list[1][frame->cur_slice_idx]);
}

static void codec_hevc_update_col_frame(struct codec_hevc *hevc)
{
	struct hevc_frame *cur_frame = hevc->cur_frame;
	union rpm_param *param = &hevc->rpm_param;
	u32 list_no = 0;
	u32 col_ref = param->p.collocated_ref_idx;
	u32 col_from_l0 = param->p.collocated_from_l0_flag;
	u32 cur_slice_idx = cur_frame->cur_slice_idx;

	if (cur_frame->cur_slice_type == B_SLICE)
		list_no = 1 - col_from_l0;

	if (col_ref >= cur_frame->ref_num[list_no])
		hevc->col_poc = INVALID_POC;
	else
		hevc->col_poc = cur_frame->ref_poc_list[list_no]
						       [cur_slice_idx]
						       [col_ref];

	if (cur_frame->cur_slice_type == I_SLICE)
		goto end;

	if (hevc->col_poc != INVALID_POC)
		hevc->col_frame = codec_hevc_get_frame_by_poc(hevc,
							      hevc->col_poc);
	else
		hevc->col_frame = hevc->cur_frame;

end:
	if (!hevc->col_frame)
		hevc->col_frame = hevc->cur_frame;
}

static void codec_hevc_update_pocs(struct amvdec_session *sess)
{
	struct codec_hevc *hevc = sess->priv;
	union rpm_param *param = &hevc->rpm_param;
	u32 nal_unit_type = param->p.m_nalUnitType;
	u32 temporal_id = param->p.m_temporalId & 0x7;
	int max_poc_lsb =
		BIT(param->p.log2_max_pic_order_cnt_lsb_minus4 + 4);
	int prev_poc_lsb;
	int prev_poc_msb;
	int poc_msb;
	int poc_lsb = param->p.POClsb;

	if (nal_unit_type == NAL_UNIT_CODED_SLICE_IDR ||
	    nal_unit_type == NAL_UNIT_CODED_SLICE_IDR_N_LP) {
		hevc->curr_poc = 0;
		if ((temporal_id - 1) == 0)
			hevc->prev_tid0_poc = hevc->curr_poc;

		return;
	}

	prev_poc_lsb = hevc->prev_tid0_poc % max_poc_lsb;
	prev_poc_msb = hevc->prev_tid0_poc - prev_poc_lsb;

	if ((poc_lsb < prev_poc_lsb) &&
	    ((prev_poc_lsb - poc_lsb) >= (max_poc_lsb / 2)))
		poc_msb = prev_poc_msb + max_poc_lsb;
	else if ((poc_lsb > prev_poc_lsb) &&
		 ((poc_lsb - prev_poc_lsb) > (max_poc_lsb / 2)))
		poc_msb = prev_poc_msb - max_poc_lsb;
	else
		poc_msb = prev_poc_msb;

	if (nal_unit_type == NAL_UNIT_CODED_SLICE_BLA   ||
	    nal_unit_type == NAL_UNIT_CODED_SLICE_BLANT ||
	    nal_unit_type == NAL_UNIT_CODED_SLICE_BLA_N_LP)
		poc_msb = 0;

	hevc->curr_poc = (poc_msb + poc_lsb);
	if ((temporal_id - 1) == 0)
		hevc->prev_tid0_poc = hevc->curr_poc;
}

static int codec_hevc_process_segment_header(struct amvdec_session *sess)
{
	struct codec_hevc *hevc = sess->priv;
	union rpm_param *param = &hevc->rpm_param;
	u32 slice_segment_address = param->p.slice_segment_address;
	u32 max_poc_lsb;

	if (param->p.log2_max_pic_order_cnt_lsb_minus4 > 12)
		return -EINVAL;
	max_poc_lsb = BIT(param->p.log2_max_pic_order_cnt_lsb_minus4 + 4);
	if (param->p.slice_type > I_SLICE ||
	    param->p.num_ref_idx_l0_active > MAX_REF_ACTIVE ||
	    param->p.num_ref_idx_l1_active > MAX_REF_ACTIVE ||
	    !hevc->lcu_total || slice_segment_address >= hevc->lcu_total ||
	    param->p.POClsb >= max_poc_lsb ||
	    !(param->p.m_temporalId & 0x7) ||
	    param->p.collocated_from_l0_flag > 1 ||
	    param->p.sps_num_reorder_pics_0 >= hevc->dpb_size ||
	    param->p.log2_parallel_merge_level > 6 ||
	    param->p.five_minus_max_num_merge_cand > 4)
		return -EINVAL;

	if (param->p.first_slice_segment_in_pic_flag == 0) {
		hevc->slice_segment_addr = slice_segment_address;
		if (!param->p.dependent_slice_segment_flag)
			hevc->slice_addr = hevc->slice_segment_addr;
	} else {
		hevc->slice_segment_addr = 0;
		hevc->slice_addr = 0;
	}

	codec_hevc_update_pocs(sess);

	return 0;
}

static int codec_hevc_process_segment(struct amvdec_session *sess,
				      bool resume_current)
{
	struct codec_hevc *hevc = sess->priv;
	struct amvdec_core *core = sess->core;
	union rpm_param *param = &hevc->rpm_param;
	u32 slice_segment_address = param->p.slice_segment_address;
	int ret;

	/* First slice: new frame */
	if (slice_segment_address == 0) {
		if (!resume_current) {
			codec_hevc_update_referenced(sess);
			codec_hevc_show_frames(sess);

			hevc->cur_frame = codec_hevc_prepare_new_frame(sess);
			if (!hevc->cur_frame)
				return -1;
		} else if (!hevc->cur_frame) {
			return -EINVAL;
		}
	} else {
		if (!hevc->cur_frame ||
		    hevc->cur_frame->cur_slice_idx >= MAX_SLICE_NUM - 1)
			return -EINVAL;
		if (!resume_current)
			hevc->cur_frame->cur_slice_idx++;
	}

	if (codec_hevc_update_frame_refs(sess, hevc->cur_frame))
		return -EINVAL;
	codec_hevc_conceal_missing_refs(sess, hevc->cur_frame);
	codec_hevc_update_col_frame(hevc);
	codec_hevc_update_ldc_flag(hevc);
	if (codec_hevc_use_mmu(core->platform->revision, sess->pixfmt_cap,
			       hevc->is_10bit)) {
		ret = codec_hevc_fill_mmu_map(sess, &hevc->common,
					      &hevc->cur_frame->vbuf->vb2_buf,
					      hevc->is_10bit);
		if (ret) {
			dev_err(core->dev, "Failed to build MMU map: %d\n", ret);
			return ret;
		}
	}
	codec_hevc_set_mc(sess, hevc->cur_frame);
	codec_hevc_set_mcrcc(sess);
	codec_hevc_set_mpred(sess, hevc->cur_frame, hevc->col_frame);
	codec_hevc_set_sao(sess, hevc->cur_frame);

	amvdec_write_dos_bits(core, HEVC_WAIT_FLAG, BIT(1));
	amvdec_write_dos(core, HEVC_DEC_STATUS_REG,
			 HEVC_CODED_SLICE_SEGMENT_DAT);

	/* Interrupt the firmware's processor */
	amvdec_write_dos(core, HEVC_MCPU_INTR_REQ, AMRISC_MAIN_REQ);

	return 0;
}

static int codec_hevc_process_rpm(struct amvdec_session *sess)
{
	struct codec_hevc *hevc = sess->priv;
	union rpm_param *param = &hevc->rpm_param;
	int src_changed = 0;
	u32 crop_width, crop_height;
	u32 dst_width, dst_height;
	u32 log2_lcu_size;
	u32 dpb_size = 16;
	u32 lcu_size;
	u32 is_10bit = 0;

	if (param->p.slice_segment_address	||
	    !param->p.pic_width_in_luma_samples	||
	    !param->p.pic_height_in_luma_samples)
		return 0;

	if (param->p.bit_depth)
		is_10bit = 1;

	if (param->p.pic_width_in_luma_samples > sess->fmt_out->max_width ||
	    param->p.pic_height_in_luma_samples > sess->fmt_out->max_height ||
	    param->p.chroma_format_idc > 3 ||
	    param->p.log2_max_pic_order_cnt_lsb_minus4 > 12)
		return -EINVAL;

	hevc->width = param->p.pic_width_in_luma_samples;
	hevc->height = param->p.pic_height_in_luma_samples;
	dst_width = hevc->width;
	dst_height = hevc->height;
	if (sess->fmt_out->codec_ops->irq == AMVDEC_IRQ_MBOX0) {
		dpb_size = hevc->max_dec_pic_buffering + 1 +
			   HEVC_DPB_BUFFER_MARGIN;
		dpb_size = clamp(dpb_size, sess->fmt_out->min_buffers,
				 sess->fmt_out->max_buffers);
	}

	log2_lcu_size = param->p.log2_min_coding_block_size_minus3 + 3 +
			param->p.log2_diff_max_min_coding_block_size;
	if (log2_lcu_size < 4 || log2_lcu_size > 6)
		return -EINVAL;
	lcu_size = BIT(log2_lcu_size);

	hevc->lcu_x_num = (hevc->width + lcu_size - 1) / lcu_size;
	hevc->lcu_y_num = (hevc->height + lcu_size - 1) / lcu_size;
	hevc->lcu_total = hevc->lcu_x_num * hevc->lcu_y_num;

	if (param->p.conformance_window_flag) {
		u32 sub_width = 1, sub_height = 1;

		switch (param->p.chroma_format_idc) {
		case 1:
			sub_height = 2;
			fallthrough;
		case 2:
			sub_width = 2;
			break;
		}

		crop_width = sub_width * (param->p.conf_win_left_offset +
					  param->p.conf_win_right_offset);
		crop_height = sub_height * (param->p.conf_win_top_offset +
					    param->p.conf_win_bottom_offset);
		if (crop_width >= dst_width || crop_height >= dst_height)
			return -EINVAL;
		dst_width -= crop_width;
		dst_height -= crop_height;
	}

	if (dst_width != hevc->dst_width ||
	    dst_height != hevc->dst_height ||
	    lcu_size != hevc->lcu_size ||
	    is_10bit != hevc->is_10bit ||
	    dpb_size != hevc->dpb_size)
		src_changed = 1;

	hevc->dst_width = dst_width;
	hevc->dst_height = dst_height;
	hevc->dpb_size = dpb_size;
	hevc->lcu_size = lcu_size;
	hevc->is_10bit = is_10bit;

	return src_changed;
}

/*
 * The RPM section within the workspace contains
 * many information regarding the parsed bitstream
 */
static void codec_hevc_fetch_rpm(struct amvdec_session *sess)
{
	struct codec_hevc *hevc = sess->priv;
	u16 *rpm_vaddr = hevc->workspace_vaddr + RPM_OFFSET;
	u16 raw[RPM_SIZE];
	int i, j;

	dma_rmb();
	for (i = 0; i < RPM_SIZE; i += 4) {
		for (j = 0; j < 4; j++)
			raw[i + j] = rpm_vaddr[i + 3 - j];
	}

	if (sess->fmt_out->codec_ops->irq != AMVDEC_IRQ_MBOX0) {
		memcpy(hevc->rpm_param.l.data, raw, sizeof(raw));
		return;
	}

	memset(hevc->rpm_param.l.data, 0, sizeof(hevc->rpm_param.l.data));
	memcpy(hevc->rpm_param.l.data, raw,
	       RPM_LEGACY_TILE_HEIGHT_OFFSET * sizeof(*raw));
	memcpy(&hevc->rpm_param.l.data[RPM_LEGACY_TILE_HEIGHT_OFFSET],
	       &raw[RPM_MULTI_TILE_HEIGHT_OFFSET],
	       RPM_TILE_HEIGHT_WORDS * sizeof(*raw));
	memcpy(&hevc->rpm_param.l.data[RPM_LEGACY_POST_TILE_OFFSET],
	       &raw[RPM_MULTI_POST_TILE_OFFSET],
	       RPM_POST_TILE_WORDS * sizeof(*raw));
	hevc->max_dec_pic_buffering = raw[RPM_MULTI_DPB_OFFSET];
}

static void codec_hevc_start_cpu(struct amvdec_core *core)
{
	amvdec_read_dos(core, DOS_SW_RESET3);
	amvdec_read_dos(core, DOS_SW_RESET3);
	amvdec_read_dos(core, DOS_SW_RESET3);
	amvdec_write_dos(core, DOS_SW_RESET3, BIT(12) | BIT(11));
	amvdec_write_dos(core, DOS_SW_RESET3, 0);
	amvdec_read_dos(core, DOS_SW_RESET3);
	amvdec_read_dos(core, DOS_SW_RESET3);
	amvdec_read_dos(core, DOS_SW_RESET3);
	amvdec_write_dos_bits(core, HEVC_WRRSP_LMEM, 7 << 25);
	amvdec_write_dos(core, HEVC_MPSR, 1);
}

static int codec_hevc_resume_context(struct amvdec_session *sess,
				     bool restart_input, bool restart_cpu)
{
	struct codec_hevc *hevc = sess->priv;
	struct amvdec_core *core = sess->core;
	unsigned long flags;
	bool active;
	int ret = 0;

	mutex_lock(&hevc->lock);
	if (sess->status == STATUS_NEEDS_RESUME)
		hevc->reset_buffers = true;

	spin_lock_irqsave(&core->irq_lock, flags);
	active = core->cur_sess == sess;
	spin_unlock_irqrestore(&core->irq_lock, flags);
	if (!active) {
		WRITE_ONCE(hevc->resume_pending, true);
		mutex_unlock(&hevc->lock);
		return 0;
	}

	if (hevc->reset_buffers)
		codec_hevc_free_fbc_buffers(sess, &hevc->common);
	ret = codec_hevc_setup_buffers(sess, &hevc->common, hevc->is_10bit);
	if (ret) {
		amvdec_abort(sess);
		mutex_unlock(&hevc->lock);
		return ret;
	}
	hevc->reset_buffers = false;
	ret = codec_hevc_setup_workspace(sess, hevc);
	if (ret) {
		amvdec_abort(sess);
		mutex_unlock(&hevc->lock);
		return ret;
	}

	codec_hevc_setup_decode_head(sess, hevc->is_10bit);
	if (restart_input) {
		ret = esparser_replay_src(sess, hevc->input_vbuf,
					  hevc->input_parser_size);
		if (ret) {
			amvdec_abort(sess);
			mutex_unlock(&hevc->lock);
			return ret;
		}
		amvdec_write_dos(core, HEVC_WAIT_FLAG, 0);
		amvdec_write_dos(core, HEVC_SHIFT_BYTE_COUNT, 0);
		amvdec_write_dos(core, HEVC_DECODE_SIZE, hevc->input_size);
		amvdec_write_dos(core, HEVC_DEC_STATUS_REG, HEVC_ACTION_DONE);
		codec_hevc_start_cpu(core);
		codec_hevc_watchdog_arm(hevc);
	} else {
		if (restart_cpu)
			codec_hevc_start_cpu(core);
		if (restart_cpu) {
			ret = codec_hevc_process_segment(sess, true);
		} else {
			ret = codec_hevc_process_segment_header(sess);
			if (!ret)
				ret = codec_hevc_process_segment(sess, false);
		}
		if (ret)
			amvdec_abort(sess);
		else if (restart_cpu)
			codec_hevc_watchdog_arm(hevc);
	}
	mutex_unlock(&hevc->lock);

	return ret;
}

static int codec_hevc_resume(struct amvdec_session *sess)
{
	return codec_hevc_resume_context(sess, false, false);
}

static int codec_hevc_run(struct amvdec_session *sess)
{
	struct codec_hevc *hevc = sess->priv;
	bool partial;

	if (!hevc)
		return 0;
	if (READ_ONCE(hevc->resume_pending)) {
		partial = READ_ONCE(hevc->input_has_frame);
		WRITE_ONCE(hevc->resume_pending, false);
		return codec_hevc_resume_context(sess, !partial, partial);
	}

	mutex_lock(&hevc->lock);
	if (!hevc->input_pending) {
		hevc->waiting_for_input = true;
		mutex_unlock(&hevc->lock);
		return 0;
	}

	amvdec_write_dos(sess->core, HEVC_WAIT_FLAG, 0);
	amvdec_write_dos(sess->core, HEVC_SHIFT_BYTE_COUNT, 0);
	amvdec_write_dos(sess->core, HEVC_DECODE_SIZE, hevc->input_size);
	amvdec_write_dos(sess->core, HEVC_DEC_STATUS_REG, HEVC_ACTION_DONE);
	codec_hevc_start_cpu(sess->core);
	codec_hevc_watchdog_arm(hevc);
	mutex_unlock(&hevc->lock);

	return 0;
}

static void
codec_hevc_input_queued(struct amvdec_session *sess,
			struct vb2_v4l2_buffer *vbuf, u32 payload_size,
			u32 parser_size)
{
	struct codec_hevc *hevc = sess->priv;

	if (!hevc) {
		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
		atomic_dec_if_positive(&sess->esparser_queued_bufs);
		return;
	}

	mutex_lock(&hevc->lock);
	hevc->input_has_frame = false;
	hevc->input_size = payload_size;
	hevc->input_parser_size = parser_size;
	hevc->input_vbuf = vbuf;
	WRITE_ONCE(hevc->input_pending, true);
	if (!hevc->waiting_for_input) {
		mutex_unlock(&hevc->lock);
		return;
	}

	hevc->waiting_for_input = false;
	amvdec_write_dos(sess->core, HEVC_WAIT_FLAG, 0);
	amvdec_write_dos(sess->core, HEVC_SHIFT_BYTE_COUNT, 0);
	amvdec_write_dos(sess->core, HEVC_DECODE_SIZE, payload_size);
	amvdec_write_dos(sess->core, HEVC_DEC_STATUS_REG, HEVC_ACTION_DONE);
	codec_hevc_start_cpu(sess->core);
	codec_hevc_watchdog_arm(hevc);
	mutex_unlock(&hevc->lock);
}

static bool codec_hevc_can_queue_input(struct amvdec_session *sess)
{
	struct codec_hevc *hevc = sess->priv;

	return hevc && !READ_ONCE(hevc->input_pending);
}

static bool codec_hevc_has_pending_job(struct amvdec_session *sess)
{
	struct codec_hevc *hevc = sess->priv;

	return hevc && (READ_ONCE(hevc->resume_pending) ||
			READ_ONCE(hevc->input_pending));
}

static bool codec_hevc_job_ready(struct amvdec_session *sess)
{
	return sess->streamon_cap &&
		v4l2_m2m_num_dst_bufs_ready(sess->m2m_ctx) > 0;
}

static void codec_hevc_finish_job(struct amvdec_session *sess)
{
	struct codec_hevc *hevc = sess->priv;
	struct amvdec_timestamp_info timestamp;
	u32 decode_info;
	bool last_input;

	codec_hevc_watchdog_disarm(hevc);

	decode_info = amvdec_read_dos(sess->core, HEVC_RPM_BUFFER);
	hevc->start_decoding_flag |= decode_info & 0xff;
	hevc->rps_set_id = (decode_info >> 8) & 0xff;
	codec_hevc_show_frames(sess);

	/*
	 * Vendor frame-based HEVC stops MPSR/CPSR and drains firmware DMA in
	 * the terminal IRQ before dirtying the compressed input.  Keep the
	 * public M2M job in COMPLETING while doing the same here, so neither a
	 * returned CAPTURE nor OUTPUT buffer can restart this hardware owner.
	 */
	vdec_hevc_quiesce(sess);

	WRITE_ONCE(hevc->input_pending, false);
	if (hevc->input_vbuf) {
		v4l2_m2m_buf_done(hevc->input_vbuf, VB2_BUF_STATE_DONE);
		hevc->input_vbuf = NULL;
	}
	if (!hevc->input_has_frame) {
		amvdec_take_ts(sess, &timestamp);
		atomic_dec_if_positive(&sess->esparser_queued_bufs);
	}

	last_input = sess->draining &&
		!v4l2_m2m_num_src_bufs_ready(sess->m2m_ctx);
	if (last_input) {
		sess->should_stop = 1;
		codec_hevc_flush_output(sess);
		v4l2_m2m_mark_stopped(sess->m2m_ctx);
		sess->draining = false;
	}
}

static irqreturn_t codec_hevc_threaded_isr(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_hevc *hevc = sess->priv;
	u32 dec_status = amvdec_read_dos(core, HEVC_DEC_STATUS_REG);
	bool yield = false;
	bool finish_yield = false;
	int ret;

	if (!hevc)
		return IRQ_HANDLED;

	mutex_lock(&hevc->lock);
	if (sess->fmt_out->codec_ops->irq == AMVDEC_IRQ_MBOX0 &&
	    (dec_status == HEVC_DECPIC_DATA_DONE ||
	     dec_status == HEVC_DECODE_BUFEMPTY ||
	     dec_status == HEVC_SEARCH_BUFEMPTY ||
	     dec_status == HEVC_DECODE_BUFEMPTY2)) {
		/*
		 * Publish COMPLETING before returning the retained OUTPUT buffer.
		 * An immediate userspace QBUF can then only feed the next M2M job;
		 * it cannot enter this job while the IRQ tail releases HEVC.
		 */
		if (!amvdec_m2m_begin_yield(sess))
			goto unlock;
		codec_hevc_finish_job(sess);
		finish_yield = true;
		goto unlock;
	}
	if (dec_status != HEVC_SLICE_SEGMENT_DONE) {
		dev_err(core->dev_dec, "Unrecognized dec_status: %08X\n",
			dec_status);
		amvdec_abort(sess);
		yield = true;
		goto unlock;
	}

	sess->keyframe_found = 1;
	codec_hevc_fetch_rpm(sess);
	ret = codec_hevc_process_rpm(sess);
	if (ret < 0) {
		amvdec_abort(sess);
		yield = true;
		goto unlock;
	}
	if (sess->fmt_out->codec_ops->irq == AMVDEC_IRQ_MBOX0)
		hevc->nal_search_done = true;
	if (ret > 0) {
		u32 width = hevc->dst_width;
		u32 height = hevc->dst_height;
		u32 dpb_size = hevc->dpb_size;
		u8 bitdepth = hevc->is_10bit ? 10 : 8;
		bool capture_streaming = sess->streamon_cap;

		if (capture_streaming)
			codec_hevc_flush_source_change(sess);

		/* The source-change helper updates session state and may resume us. */
		mutex_unlock(&hevc->lock);
		amvdec_src_change(sess, width, height, dpb_size, bitdepth);
		mutex_lock(&hevc->lock);
		if (sess->status == STATUS_NEEDS_RESUME) {
			if (capture_streaming)
				codec_hevc_signal_source_change(sess);
			yield = true;
		}
		goto unlock;
	}

	if (codec_hevc_process_segment_header(sess) ||
	    codec_hevc_process_segment(sess, false)) {
		amvdec_abort(sess);
		yield = true;
	}

unlock:
	mutex_unlock(&hevc->lock);
	if (finish_yield)
		amvdec_m2m_finish_yield(sess);
	else if (yield)
		amvdec_m2m_job_yield(sess);
	return IRQ_HANDLED;
}

static irqreturn_t codec_hevc_isr(struct amvdec_session *sess)
{
	struct codec_hevc *hevc = sess->priv;

	if (hevc && READ_ONCE(hevc->watchdog_armed)) {
		codec_hevc_watchdog_note_progress(hevc);
		mod_delayed_work(system_wq, &hevc->watchdog_work,
				 msecs_to_jiffies(HEVC_WATCHDOG_POLL_MS));
	}
	if (sess->fmt_out->codec_ops->irq == AMVDEC_IRQ_MBOX0)
		amvdec_write_dos(sess->core, HEVC_ASSIST_MBOX0_CLR_REG, 1);
	else
		amvdec_write_dos(sess->core, HEVC_ASSIST_MBOX1_CLR_REG, 1);

	return IRQ_WAKE_THREAD;
}

struct amvdec_codec_ops codec_hevc_ops = {
	.start = codec_hevc_start,
	.stop = codec_hevc_stop,
	.isr = codec_hevc_isr,
	.threaded_isr = codec_hevc_threaded_isr,
	.num_pending_bufs = codec_hevc_num_pending_bufs,
	.drain = codec_hevc_flush_output,
	.resume = codec_hevc_resume,
};

struct amvdec_codec_ops codec_hevc_g12a_ops = {
	.canvas_height_align = 32,
	.start = codec_hevc_start,
	.run = codec_hevc_run,
	.input_queued_buf = codec_hevc_input_queued,
	.can_queue_input = codec_hevc_can_queue_input,
	.async_drain = true,
	.stop = codec_hevc_keep_context,
	.release = codec_hevc_release,
	.irq = AMVDEC_IRQ_MBOX0,
	.context_switching = true,
	.prepare_firmware = codec_hevc_prepare_firmware,
	.has_pending_job = codec_hevc_has_pending_job,
	.job_ready = codec_hevc_job_ready,
	.isr = codec_hevc_isr,
	.threaded_isr = codec_hevc_threaded_isr,
	.num_pending_bufs = codec_hevc_num_pending_bufs,
	.hold_capture_buf = codec_hevc_hold_capture_buf,
	.drain = codec_hevc_flush_output,
	.resume = codec_hevc_resume,
};
