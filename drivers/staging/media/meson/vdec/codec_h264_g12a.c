// SPDX-License-Identifier: GPL-2.0+

#include <linux/dma-mapping.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/log2.h>
#include <linux/overflow.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <media/v4l2-h264.h>
#include <media/v4l2-mem2mem.h>

#include "vdec_helpers.h"
#include "dos_regs.h"
#include "codec_h264_g12a.h"
#include "codec_h264_g12a_dpb.h"
#include "codec_h264_g12a_lmem.h"

#define H264_MULTI_FW_PAGES	7
#define H264_MULTI_SWAP_PAGES	9
#define H264_MULTI_PAGE_SIZE	SZ_4K
#define H264_MULTI_FW_SIZE	(H264_MULTI_FW_PAGES * H264_MULTI_PAGE_SIZE)
#define H264_MULTI_SWAP_SIZE	(H264_MULTI_SWAP_PAGES * H264_MULTI_PAGE_SIZE)
#define H264_MULTI_LMEM_WORDS	(PAGE_SIZE / sizeof(u16))
#define H264_MULTI_WORKSPACE_SIZE	(ALIGN((SZ_2M + SZ_32K + SZ_128K + 128), PAGE_SIZE))

#define H264_MULTI_MB_WIDTH_MASK		GENMASK(7, 0)
#define H264_MULTI_MB_TOTAL_MASK		GENMASK(23, 8)
#define H264_MULTI_CHROMA_FORMAT_MASK	GENMASK(14, 13)
#define H264_MULTI_FRAME_MBS_ONLY	BIT(15)
#define H264_MULTI_MAX_REFS_MASK		GENMASK(15, 8)
#define H264_MULTI_SPS_BITSTREAM_RESTRICTION	BIT(3)
#define H264_MULTI_MAX_DPB_SIZE		16
#define H264_MULTI_DCAC_READ_MARGIN	SZ_64K

#define H264_MULTI_INIT_FLAG		AV_SCRATCH_2
#define H264_MULTI_HEAD_PADDING		AV_SCRATCH_3
#define H264_MULTI_DECODE_MODE		AV_SCRATCH_4
#define H264_MULTI_DECODE_SEQINFO	AV_SCRATCH_5
#define H264_MULTI_SEQ_INFO2		AV_SCRATCH_1
#define H264_MULTI_SEQ_INFO		AV_SCRATCH_2
#define H264_MULTI_CROP_INFO		AV_SCRATCH_6
#define H264_MULTI_PARAM4		AV_SCRATCH_B
#define H264_MULTI_FRAME_COUNTER		AV_SCRATCH_I
#define H264_MULTI_DPB_STATUS		AV_SCRATCH_J
#define H264_MULTI_LMEM_ADDR		AV_SCRATCH_L
#define H264_MULTI_DPB_CONFIG		AV_SCRATCH_7
#define H264_MULTI_NAL_SEARCH_CTL	AV_SCRATCH_9
#define H264_MULTI_DECODE_SIZE		AV_SCRATCH_E
#define H264_MULTI_MPC_E		0x0c20
#define H264_MULTI_MBY_MBX		0x301c

#define H264_MULTI_BUFFER_INFO_DATA	0x3088
#define H264_MULTI_BUFFER_INFO_INDEX	0x3090
#define H264_MULTI_CURRENT_POC_INDEX	0x30c0
#define H264_MULTI_CURRENT_POC		0x30c8
#define H264_MULTI_DBKR_CANVAS_ADDR	0x26c0
#define H264_MULTI_DBKW_CANVAS_ADDR	0x26c4
#define H264_MULTI_REC_CANVAS_ADDR	0x26c8
#define H264_MULTI_CURR_CANVAS_CTRL	0x26cc
#define H264_MULTI_CO_MB_WR_ADDR		0x30e0
#define H264_MULTI_CO_MB_RD_ADDR		0x30e4
#define H264_MULTI_CO_MB_RW_CTL		0x30f4

#define H264_MULTI_CO_MB_SIZE		96
#define H264_MULTI_RUN_READY_MIN_BUFS	2
#define H264_MULTI_WATCHDOG_POLL_MS	100
#define H264_MULTI_WATCHDOG_STALL_MS	2000

#define H264_MULTI_DECODE_MODE_FRAME	1

enum h264_multi_fw_page {
	H264_MULTI_FW_MAIN_0,
	H264_MULTI_FW_MAIN_1,
	H264_MULTI_FW_DATA,
	H264_MULTI_FW_LIST,
	H264_MULTI_FW_HEADER,
	H264_MULTI_FW_SLICE,
	H264_MULTI_FW_MMCO,
};

enum h264_multi_swap_page {
	H264_MULTI_SWAP_HEADER,
	H264_MULTI_SWAP_DATA,
	H264_MULTI_SWAP_MMCO,
	H264_MULTI_SWAP_LIST,
	H264_MULTI_SWAP_SLICE,
	H264_MULTI_SWAP_MAIN_0,
	H264_MULTI_SWAP_MAIN_1,
	H264_MULTI_SWAP_MAIN_DATA,
	H264_MULTI_SWAP_MAIN_SLICE,
};

struct h264_multi_frame {
	struct list_head list;
	struct vb2_v4l2_buffer *vbuf;
	struct amvdec_timestamp_info timestamp;
	s32 poc;
	u32 buffer_index;
	u32 type;
};

struct codec_h264_multi {
	struct amvdec_session *sess;
	struct mutex lock; /* Protects per-context codec and DPB state. */
	struct delayed_work watchdog_work;
	void *fw_swap_vaddr;
	dma_addr_t fw_swap_paddr;
	void *lmem_vaddr;
	dma_addr_t lmem_paddr;
	void *workspace_vaddr;
	dma_addr_t workspace_paddr;
	void *mv_vaddr;
	dma_addr_t mv_paddr;
	size_t mv_size;
	size_t mv_slot_size;
	struct h264_multi_dpb dpb;
	struct h264_multi_dpb_picture pic_state;
	struct h264_multi_config config;
	struct v4l2_ctrl_h264_decode_params decode_params;
	struct v4l2_ctrl_h264_sps sps;
	struct v4l2_h264_reflist_builder reflist_builder;
	struct v4l2_h264_reference ref_list0[V4L2_H264_REF_LIST_LEN];
	struct v4l2_h264_reference ref_list1[V4L2_H264_REF_LIST_LEN];
	struct h264_multi_picture pending_picture;
	struct h264_multi_marking pic_marking;
	struct list_head frames;
	struct mutex frames_lock; /* Protects frames and frame_count. */
	unsigned int frame_count;
	u32 scratch_f;
	u32 iqidct_control;
	u32 vcop_control;
	u32 vld_decode_control;
	u32 frame_counter;
	u32 decode_seqinfo;
	u32 irq_status;
	u32 input_size;
	u32 config_search_rp;
	u32 watchdog_last_status;
	u32 watchdog_last_mby;
	u32 watchdog_last_vld_level;
	u32 watchdog_last_vififo_rp;
	unsigned long watchdog_progress_at;
	atomic_t watchdog_event;
	int watchdog_last_event;
	unsigned int capture_buf_count;
	bool config_valid;
	bool config_search_valid;
	bool configuring;
	bool restore_config_rp;
	bool initialized;
	bool watchdog_armed;
	bool watchdog_progress_valid;
	bool input_pending;
	bool waiting_for_input;
	bool pic_done_pending;
	bool resume_pending;
	bool slice_pending;
	union {
		u16 words[H264_MULTI_LMEM_WORDS];
		struct h264_multi_lmem data;
	} lmem;
};

static int codec_h264_multi_resume_pending_picture(struct amvdec_session *sess);
static int codec_h264_multi_finish_picture(struct amvdec_session *sess);
static int
codec_h264_multi_alloc_mv(struct amvdec_session *sess,
			  const struct h264_multi_config *config,
			  unsigned int capture_buf_count);

static void codec_h264_multi_watchdog_disarm(struct codec_h264_multi *h264)
{
	h264->watchdog_armed = false;
	h264->watchdog_progress_valid = false;
	cancel_delayed_work(&h264->watchdog_work);
}

static void codec_h264_multi_watchdog_arm(struct codec_h264_multi *h264)
{
	h264->watchdog_armed = true;
	h264->watchdog_progress_valid = false;
	h264->watchdog_last_event = atomic_read(&h264->watchdog_event);
	mod_delayed_work(system_wq, &h264->watchdog_work,
			 msecs_to_jiffies(H264_MULTI_WATCHDOG_POLL_MS));
}

static bool codec_h264_multi_decode_progress_state(u32 status)
{
	return status == H264_MULTI_ACTION_DECODE_SLICE ||
	       status == H264_MULTI_ACTION_CONFIG_DONE ||
	       status == H264_MULTI_ACTION_DECODE_NEWPIC ||
	       status == H264_MULTI_SEI_DATA_READY ||
	       status == H264_MULTI_SEI_DATA_DONE ||
	       status == H264_MULTI_SLICE_HEAD_DONE;
}

static bool codec_h264_multi_parsing_state(u32 status)
{
	return status == H264_MULTI_ACTION_SEARCH_HEAD ||
	       (status & 0xf0) == 0x80;
}

static void codec_h264_multi_watchdog_work(struct work_struct *work)
{
	struct codec_h264_multi *h264 =
		container_of(to_delayed_work(work), struct codec_h264_multi,
			     watchdog_work);
	struct amvdec_session *sess = h264->sess;
	struct amvdec_core *core = sess->core;
	bool timed_out = false;
	bool progressed = false;
	u32 bit_count = 0;
	u32 cpsr = 0;
	u32 co_ctl = 0;
	u32 co_rd = 0;
	u32 co_wr = 0;
	u32 curr = 0;
	u32 curr_canvas = 0;
	u32 dbkr_canvas = 0;
	u32 dbkw_canvas = 0;
	u32 decode_size = 0;
	u32 dpb_buffer_mask = 0;
	u32 dpb_slot_mask = 0;
	u32 level = 0;
	u32 mpc = 0;
	u32 mby = 0;
	u32 mpsr = 0;
	u32 nal_search = 0;
	u32 pic_dc_status = 0;
	u32 rec_canvas = 0;
	u32 rp = 0;
	u32 search_rp = 0;
	u32 status = 0;
	u32 wp = 0;
	bool config_search_valid = false;
	bool configuring = false;
	bool initialized = false;
	bool resume_pending = false;
	unsigned int capture_buf_count = 0;
	unsigned int current_buffer = 0;
	unsigned int first_mb = 0;
	unsigned int frame_count = 0;
	unsigned int frame_num = 0;
	unsigned int i;
	unsigned int l0_count = 0;
	unsigned int l1_count = 0;
	unsigned int slice_type = 0;
	int event;

	mutex_lock(&h264->lock);
	if (!h264->watchdog_armed || !h264->input_pending)
		goto unlock;
	if (!amvdec_session_is_current(sess))
		goto rearm;

	{
		status = amvdec_read_dos(core, H264_MULTI_DPB_STATUS);
		mby = amvdec_read_dos(core, H264_MULTI_MBY_MBX);
		level = amvdec_read_dos(core, VLD_MEM_VIFIFO_LEVEL);
		rp = amvdec_read_dos(core, VLD_MEM_VIFIFO_RP);
		event = atomic_read(&h264->watchdog_event);

		if (!h264->watchdog_progress_valid ||
		    event != h264->watchdog_last_event ||
		    status != h264->watchdog_last_status) {
			progressed = true;
		} else if (codec_h264_multi_decode_progress_state(status)) {
			progressed = mby != h264->watchdog_last_mby;
		} else if (codec_h264_multi_parsing_state(status)) {
			progressed = level != h264->watchdog_last_vld_level ||
				     rp != h264->watchdog_last_vififo_rp;
		} else {
			progressed = mby != h264->watchdog_last_mby ||
				     level != h264->watchdog_last_vld_level ||
				     rp != h264->watchdog_last_vififo_rp;
		}

		h264->watchdog_last_status = status;
		h264->watchdog_last_mby = mby;
		h264->watchdog_last_vld_level = level;
		h264->watchdog_last_vififo_rp = rp;
		h264->watchdog_last_event = event;
		if (progressed || !h264->watchdog_progress_valid) {
			h264->watchdog_progress_valid = true;
			h264->watchdog_progress_at = jiffies;
			goto rearm;
		}
		if (time_before(jiffies, h264->watchdog_progress_at +
				msecs_to_jiffies(H264_MULTI_WATCHDOG_STALL_MS)))
			goto rearm;

		mpsr = amvdec_read_dos(core, MPSR);
		cpsr = amvdec_read_dos(core, CPSR);
		mpc = amvdec_read_dos(core, H264_MULTI_MPC_E);
		pic_dc_status = amvdec_read_dos(core, MDEC_PIC_DC_STATUS);
		nal_search = amvdec_read_dos(core, H264_MULTI_NAL_SEARCH_CTL);
		curr_canvas = amvdec_read_dos(core, H264_MULTI_CURR_CANVAS_CTRL);
		rec_canvas = amvdec_read_dos(core, H264_MULTI_REC_CANVAS_ADDR);
		dbkr_canvas = amvdec_read_dos(core, H264_MULTI_DBKR_CANVAS_ADDR);
		dbkw_canvas = amvdec_read_dos(core, H264_MULTI_DBKW_CANVAS_ADDR);
		co_wr = amvdec_read_dos(core, H264_MULTI_CO_MB_WR_ADDR);
		co_rd = amvdec_read_dos(core, H264_MULTI_CO_MB_RD_ADDR);
		co_ctl = amvdec_read_dos(core, H264_MULTI_CO_MB_RW_CTL);
		wp = amvdec_read_dos(core, VLD_MEM_VIFIFO_WP);
		curr = amvdec_read_dos(core, VLD_MEM_VIFIFO_CURR_PTR);
		decode_size = amvdec_read_dos(core, H264_MULTI_DECODE_SIZE);
		bit_count = amvdec_read_dos(core, VIFF_BIT_CNT);
		search_rp = h264->config_search_rp;
		config_search_valid = h264->config_search_valid;
		configuring = h264->configuring;
		initialized = h264->initialized;
		resume_pending = h264->resume_pending;
		current_buffer = h264->pic_state.buffer_index;
		frame_num = h264->pic_state.picture.frame_num;
		first_mb = h264->pic_state.picture.first_mb_in_slice;
		slice_type = h264->pic_state.picture.slice_type;
		l0_count = h264->pic_state.picture.num_ref_idx_l0_active;
		l1_count = h264->pic_state.picture.num_ref_idx_l1_active;
		frame_count = READ_ONCE(h264->frame_count);
		capture_buf_count = h264->capture_buf_count;
		for (i = 0; i < H264_MULTI_DPB_SIZE; i++) {
			const struct h264_multi_dpb_slot *slot =
				&h264->dpb.slots[i];

			if (!slot->active)
				continue;
			dpb_slot_mask |= BIT(i);
			if (slot->buffer_index < 32)
				dpb_buffer_mask |= BIT(slot->buffer_index);
		}
		h264->watchdog_armed = false;
		h264->watchdog_progress_valid = false;
		h264->input_pending = false;
		h264->resume_pending = false;
		h264->slice_pending = false;
		h264->waiting_for_input = false;
		WRITE_ONCE(sess->hardware_stalled, true);
		timed_out = true;
	}
	goto unlock;

rearm:
	mod_delayed_work(system_wq, &h264->watchdog_work,
			 msecs_to_jiffies(H264_MULTI_WATCHDOG_POLL_MS));
unlock:
	mutex_unlock(&h264->lock);

	if (!timed_out)
		return;

	dev_err(sess->core->dev, "H.264 firmware made no progress for %u ms\n",
		H264_MULTI_WATCHDOG_STALL_MS);
	dev_err(sess->core->dev,
		"H.264 timeout state: status=%#x mpsr=%#x cpsr=%#x mpc=%#x mby=%#x level=%#x rp=%#x wp=%#x curr=%#x search_rp=%#x size=%#x bits=%#x cfg_valid=%u configuring=%u initialized=%u resume=%u session=%u cap=%u\n",
		status, mpsr, cpsr, mpc, mby, level, rp, wp, curr,
		search_rp, decode_size, bit_count,
		config_search_valid, configuring, initialized, resume_pending,
		sess->status, sess->streamon_cap);
	dev_err(sess->core->dev,
		"H.264 timeout picture: buffer=%u frame_num=%u first_mb=%u slice=%u l0=%u l1=%u queued_frames=%u dpb_slots=%#x dpb_buffers=%#x capture=%u allocated=%u curr_canvas=%#x rec=%#x dbkr=%#x dbkw=%#x co_wr=%#x co_rd=%#x co_ctl=%#x pic_dc=%#x nal=%#x\n",
		current_buffer, frame_num, first_mb, slice_type, l0_count,
		l1_count, frame_count, dpb_slot_mask, dpb_buffer_mask,
		capture_buf_count, sess->num_dst_bufs, curr_canvas, rec_canvas,
		dbkr_canvas, dbkw_canvas, co_wr, co_rd, co_ctl, pic_dc_status,
		nal_search);
	amvdec_abort(sess);
	amvdec_m2m_job_yield(sess);
}

static void h264_multi_copy_page(void *dst, unsigned int dst_page,
				 const u8 *src, unsigned int src_page)
{
	memcpy(dst + dst_page * H264_MULTI_PAGE_SIZE,
	       src + src_page * H264_MULTI_PAGE_SIZE,
	       H264_MULTI_PAGE_SIZE);
}

static void h264_multi_build_swap_image(void *dst, const u8 *src)
{
	h264_multi_copy_page(dst, H264_MULTI_SWAP_HEADER,
			     src, H264_MULTI_FW_HEADER);
	h264_multi_copy_page(dst, H264_MULTI_SWAP_DATA,
			     src, H264_MULTI_FW_DATA);
	h264_multi_copy_page(dst, H264_MULTI_SWAP_MMCO,
			     src, H264_MULTI_FW_MMCO);
	h264_multi_copy_page(dst, H264_MULTI_SWAP_LIST,
			     src, H264_MULTI_FW_LIST);
	h264_multi_copy_page(dst, H264_MULTI_SWAP_SLICE,
			     src, H264_MULTI_FW_SLICE);
	h264_multi_copy_page(dst, H264_MULTI_SWAP_MAIN_0,
			     src, H264_MULTI_FW_MAIN_0);
	h264_multi_copy_page(dst, H264_MULTI_SWAP_MAIN_1,
			     src, H264_MULTI_FW_MAIN_1);
	h264_multi_copy_page(dst, H264_MULTI_SWAP_MAIN_DATA,
			     src, H264_MULTI_FW_DATA);
	h264_multi_copy_page(dst, H264_MULTI_SWAP_MAIN_SLICE,
			     src, H264_MULTI_FW_SLICE);
}

int codec_h264_multi_prepare_firmware(struct amvdec_session *sess,
				      const u8 *data, u32 len)
{
	struct amvdec_core *core = sess->core;
	struct codec_h264_multi *h264;

	if (len < H264_MULTI_FW_SIZE)
		return -EINVAL;
	if (sess->priv)
		return 0;

	h264 = kzalloc_obj(*h264);
	if (!h264)
		return -ENOMEM;

	h264->fw_swap_vaddr = dma_alloc_coherent(core->dev,
						 H264_MULTI_SWAP_SIZE,
						 &h264->fw_swap_paddr,
						 GFP_KERNEL);
	if (!h264->fw_swap_vaddr)
		goto free_h264;

	h264->lmem_vaddr = dma_alloc_coherent(core->dev, PAGE_SIZE,
					      &h264->lmem_paddr, GFP_KERNEL);
	if (!h264->lmem_vaddr)
		goto free_swap;
	h264->workspace_vaddr = dma_alloc_coherent(core->dev,
						   H264_MULTI_WORKSPACE_SIZE,
						   &h264->workspace_paddr,
						   GFP_KERNEL);
	if (!h264->workspace_vaddr)
		goto free_lmem;

	h264_multi_build_swap_image(h264->fw_swap_vaddr, data);
	memset(h264->lmem_vaddr, 0, PAGE_SIZE);
	memset(h264->workspace_vaddr, 0, H264_MULTI_WORKSPACE_SIZE);
	h264_multi_dpb_reset(&h264->dpb);
	INIT_LIST_HEAD(&h264->frames);
	h264->sess = sess;
	mutex_init(&h264->lock);
	mutex_init(&h264->frames_lock);
	INIT_DELAYED_WORK(&h264->watchdog_work,
			  codec_h264_multi_watchdog_work);
	atomic_set(&h264->watchdog_event, 0);
	sess->priv = h264;

	return 0;

free_lmem:
	dma_free_coherent(core->dev, PAGE_SIZE,
			  h264->lmem_vaddr, h264->lmem_paddr);
free_swap:
	dma_free_coherent(core->dev, H264_MULTI_SWAP_SIZE,
			  h264->fw_swap_vaddr, h264->fw_swap_paddr);
free_h264:
	kfree(h264);
	return -ENOMEM;
}

static struct h264_multi_frame *
codec_h264_multi_next_frame(struct codec_h264_multi *h264)
{
	struct h264_multi_frame *frame;
	struct h264_multi_frame *next = NULL;

	list_for_each_entry(frame, &h264->frames, list) {
		if (!next || frame->poc < next->poc)
			next = frame;
	}

	return next;
}

static void codec_h264_multi_output_frame(struct amvdec_session *sess,
					  struct h264_multi_frame *frame)
{
	struct codec_h264_multi *h264 = sess->priv;

	list_del(&frame->list);
	h264->frame_count--;
	amvdec_dst_buf_done_ts(sess, frame->vbuf, V4L2_FIELD_NONE, frame->type,
			       &frame->timestamp);
	kfree(frame);
}

static void __codec_h264_multi_flush_output(struct amvdec_session *sess)
{
	struct codec_h264_multi *h264 = sess->priv;
	struct h264_multi_frame *frame;

	while ((frame = codec_h264_multi_next_frame(h264)))
		codec_h264_multi_output_frame(sess, frame);
}

static void codec_h264_multi_flush_output(struct amvdec_session *sess)
{
	struct codec_h264_multi *h264 = sess->priv;

	/* A session abort can release the context before the drain completes. */
	if (!h264)
		return;

	mutex_lock(&h264->frames_lock);
	__codec_h264_multi_flush_output(sess);
	mutex_unlock(&h264->frames_lock);
}

static struct h264_multi_frame *
codec_h264_multi_flush_before_source_change(struct amvdec_session *sess)
{
	struct codec_h264_multi *h264 = sess->priv;
	struct h264_multi_frame *frame;

	mutex_lock(&h264->frames_lock);
	while (h264->frame_count > 1) {
		frame = codec_h264_multi_next_frame(h264);
		codec_h264_multi_output_frame(sess, frame);
	}
	frame = codec_h264_multi_next_frame(h264);
	if (frame) {
		list_del(&frame->list);
		h264->frame_count--;
	}
	mutex_unlock(&h264->frames_lock);

	return frame;
}

static void __codec_h264_multi_discard_output(struct amvdec_session *sess)
{
	struct codec_h264_multi *h264 = sess->priv;
	struct h264_multi_frame *frame;
	struct h264_multi_frame *next;

	list_for_each_entry_safe(frame, next, &h264->frames, list) {
		list_del(&frame->list);
		v4l2_m2m_buf_done(frame->vbuf, VB2_BUF_STATE_ERROR);
		atomic_dec_if_positive(&sess->esparser_queued_bufs);
		kfree(frame);
	}
	h264->frame_count = 0;
}

static void codec_h264_multi_discard_output(struct amvdec_session *sess)
{
	struct codec_h264_multi *h264 = sess->priv;

	mutex_lock(&h264->frames_lock);
	__codec_h264_multi_discard_output(sess);
	mutex_unlock(&h264->frames_lock);
}

void codec_h264_multi_release_firmware(struct amvdec_session *sess)
{
	struct codec_h264_multi *h264 = sess->priv;
	struct amvdec_core *core = sess->core;

	if (!h264)
		return;
	cancel_delayed_work_sync(&h264->watchdog_work);
	mutex_lock(&h264->lock);
	if (h264->pic_state.vbuf)
		v4l2_m2m_buf_done(h264->pic_state.vbuf, VB2_BUF_STATE_ERROR);
	codec_h264_multi_discard_output(sess);
	if (h264->mv_vaddr)
		dma_free_coherent(core->dev, h264->mv_size,
				  h264->mv_vaddr, h264->mv_paddr);

	dma_free_coherent(core->dev, H264_MULTI_WORKSPACE_SIZE,
			  h264->workspace_vaddr, h264->workspace_paddr);
	dma_free_coherent(core->dev, PAGE_SIZE,
			  h264->lmem_vaddr, h264->lmem_paddr);
	dma_free_coherent(core->dev, H264_MULTI_SWAP_SIZE,
			  h264->fw_swap_vaddr, h264->fw_swap_paddr);
	mutex_unlock(&h264->lock);
	mutex_destroy(&h264->frames_lock);
	mutex_destroy(&h264->lock);
	kfree(h264);
	sess->priv = NULL;
}

static int codec_h264_multi_setup_canvases(struct amvdec_session *sess)
{
	struct codec_h264_multi *h264 = sess->priv;
	unsigned int i;
	int ret = 0;

	if (sess->canvas_reg_count) {
		amvdec_restore_canvases(sess);
	} else {
		ret = amvdec_set_canvases(sess,
					  (u32[]){ ANC0_CANVAS_ADDR, 0 },
					  (u32[]){ 24, 0 });
		if (ret)
			return ret;
	}
	for (i = 0; i < sess->canvas_num; i++)
		amvdec_write_dos(sess->core, VDEC_ASSIST_CANVAS_BLK32,
				 BIT(11) | BIT(8) | sess->canvas_alloc[i]);

	amvdec_write_dos(sess->core, H264_MULTI_DPB_CONFIG,
			 ((h264->config.max_refs + 4) << 24) |
			 (h264->capture_buf_count << 16) |
			 (h264->capture_buf_count << 8));
	return 0;
}

static int codec_h264_multi_start(struct amvdec_session *sess)
{
	struct codec_h264_multi *h264 = sess->priv;
	struct amvdec_core *core = sess->core;
	int ret;

	if (!h264)
		return -EINVAL;

	amvdec_write_dos_bits(core, POWER_CTL_VLD, BIT(9) | BIT(6));
	amvdec_clear_dos_bits(core, VDEC_ASSIST_MMC_CTRL1, BIT(3));
	amvdec_write_dos(core, PSCALE_CTRL, 0);
	amvdec_write_dos_bits(core, MDEC_PIC_DC_CTRL, 0xbf << 24);
	amvdec_clear_dos_bits(core, MDEC_PIC_DC_CTRL, 0xbf << 24);
	amvdec_clear_dos_bits(core, MDEC_PIC_DC_MUX_CTRL, BIT(31));
	amvdec_write_dos(core, MDEC_EXTIF_CFG1, 0);
	amvdec_write_dos(core, MDEC_PIC_DC_THRESH, 0x404038aa);

	amvdec_write_dos(core, AV_SCRATCH_8,
			 h264->workspace_paddr + H264_MULTI_DCAC_READ_MARGIN);
	amvdec_write_dos(core, AV_SCRATCH_G, h264->fw_swap_paddr);
	amvdec_write_dos(core, H264_MULTI_LMEM_ADDR, h264->lmem_paddr);
	amvdec_write_dos(core, M4_CONTROL_REG, BIT(13));
	amvdec_write_dos(core, H264_MULTI_DECODE_SIZE,
			 h264->input_size ?: S32_MAX);
	amvdec_write_dos(core, VIFF_BIT_CNT,
			 (h264->input_size ?: S32_MAX) * 8U);
	amvdec_write_dos(core, AV_SCRATCH_M, 1);
	amvdec_write_dos(core, AV_SCRATCH_N, 0);
	amvdec_write_dos(core, AV_SCRATCH_F,
			 (h264->scratch_f & 0xffffffc3) | BIT(4));
	amvdec_clear_dos_bits(core, AV_SCRATCH_F, BIT(6));
	if (h264->config_valid && sess->streamon_cap) {
		ret = codec_h264_multi_setup_canvases(sess);
		if (ret)
			return ret;
	}

	if (h264->initialized) {
		amvdec_write_dos(core, IQIDCT_CONTROL,
				 h264->iqidct_control ?: 0x200);
		amvdec_write_dos(core, VCOP_CTRL_REG, h264->vcop_control);
		amvdec_write_dos(core, VLD_DECODE_CONTROL,
				 h264->vld_decode_control);
	}

	amvdec_write_dos(core, H264_MULTI_DECODE_MODE,
			 H264_MULTI_DECODE_MODE_FRAME);
	amvdec_write_dos(core, H264_MULTI_DECODE_SEQINFO,
			 h264->decode_seqinfo);
	amvdec_write_dos(core, H264_MULTI_HEAD_PADDING, 0);
	amvdec_write_dos(core, H264_MULTI_INIT_FLAG,
			 h264->initialized);
	amvdec_write_dos(core, H264_MULTI_FRAME_COUNTER, h264->frame_counter);
	amvdec_write_dos(core, H264_MULTI_NAL_SEARCH_CTL,
			 BIT(2) |
			 (h264->config.bitstream_restriction ? BIT(15) : 0) |
			 (h264->config.level_idc << 7));
	amvdec_write_dos_bits(core, MDEC_EXTIF_CFG2, BIT(5));
	if (h264->slice_pending) {
		dev_err(core->dev,
			"cannot restore H.264 picture waiting for capture\n");
		return -EPIPE;
	}
	if (h264->resume_pending) {
		dev_info(core->dev,
			 "H.264 hardware start pending source resume: status=%u streamon_cap=%u input=%u search_valid=%u\n",
			 sess->status, sess->streamon_cap, h264->input_pending,
			 h264->config_search_valid);
		amvdec_write_dos(core, H264_MULTI_DPB_STATUS, 0);
	} else if (h264->initialized) {
		amvdec_write_dos(core, H264_MULTI_DPB_STATUS,
				 H264_MULTI_ACTION_DECODE_START);
	} else {
		amvdec_write_dos(core, H264_MULTI_DPB_STATUS, 0);
	}

	return 0;
}

static int codec_h264_multi_run(struct amvdec_session *sess)
{
	struct codec_h264_multi *h264 = sess->priv;
	int ret = 0;

	if (!h264)
		return -EINVAL;

	mutex_lock(&h264->lock);
	if (h264->slice_pending)
		goto unlock;
	if (h264->resume_pending) {
		h264->resume_pending = false;
		dev_info(sess->core->dev,
			 "H.264 firmware source restart: status=%u streamon_cap=%u input=%u search_valid=%u\n",
			 sess->status, sess->streamon_cap, h264->input_pending,
			 h264->config_search_valid);
		codec_h264_multi_watchdog_arm(h264);
		amvdec_write_dos(sess->core, H264_MULTI_DPB_STATUS,
				 H264_MULTI_ACTION_SEARCH_HEAD);
		goto unlock;
	}
	if (!h264->input_pending) {
		WRITE_ONCE(h264->waiting_for_input, true);
		goto unlock;
	}
	if (!h264->config_search_valid) {
		h264->config_search_rp = ALIGN_DOWN(sess->input_start, 8);
		h264->config_search_valid = true;
	}
	codec_h264_multi_watchdog_arm(h264);
	amvdec_write_dos(sess->core, H264_MULTI_DPB_STATUS,
			 H264_MULTI_ACTION_SEARCH_HEAD);

unlock:
	mutex_unlock(&h264->lock);
	return ret;
}

static void codec_h264_multi_input_queued(struct amvdec_session *sess,
					  u32 payload_size)
{
	struct codec_h264_multi *h264 = sess->priv;

	if (!h264)
		return;

	mutex_lock(&h264->lock);
	h264->input_size = payload_size;
	h264->input_pending = true;
	/* A resolution change must replay this chunk, not the first SPS. */
	h264->config_search_valid = false;
	amvdec_write_dos(sess->core, H264_MULTI_DECODE_SIZE, payload_size);
	amvdec_write_dos(sess->core, VIFF_BIT_CNT, payload_size * 8);
	if (!READ_ONCE(h264->waiting_for_input))
		goto unlock;

	if (!h264->config_search_valid) {
		h264->config_search_rp = ALIGN_DOWN(sess->input_start, 8);
		h264->config_search_valid = true;
	}
	WRITE_ONCE(h264->waiting_for_input, false);
	codec_h264_multi_watchdog_arm(h264);
	amvdec_write_dos(sess->core, H264_MULTI_DPB_STATUS,
			 H264_MULTI_ACTION_SEARCH_HEAD);

unlock:
	mutex_unlock(&h264->lock);
}

static bool codec_h264_multi_can_queue_input(struct amvdec_session *sess)
{
	struct codec_h264_multi *h264 = sess->priv;
	bool can_queue;

	if (!h264)
		return false;

	mutex_lock(&h264->lock);
	can_queue = !h264->input_pending;
	mutex_unlock(&h264->lock);

	return can_queue;
}

static int codec_h264_multi_stop(struct amvdec_session *sess)
{
	struct codec_h264_multi *h264 = sess->priv;
	struct amvdec_core *core = sess->core;

	if (!h264)
		return 0;

	mutex_lock(&h264->lock);
	codec_h264_multi_watchdog_disarm(h264);
	h264->scratch_f = amvdec_read_dos(core, AV_SCRATCH_F);
	h264->iqidct_control = amvdec_read_dos(core, IQIDCT_CONTROL);
	h264->vcop_control = amvdec_read_dos(core, VCOP_CTRL_REG);
	h264->vld_decode_control = amvdec_read_dos(core, VLD_DECODE_CONTROL);
	h264->frame_counter = amvdec_read_dos(core, H264_MULTI_FRAME_COUNTER);
	/*
	 * The firmware loses its parsed slice when the hardware stops. A picture
	 * that has not started yet can be parsed again from this chunk's start.
	 */
	if (h264->slice_pending && !h264->pic_state.active &&
	    h264->config_search_valid) {
		WRITE_ONCE(h264->slice_pending, false);
		h264->restore_config_rp = true;
	}
	if (h264->restore_config_rp) {
		sess->vififo_curr = h264->config_search_rp;
		sess->vififo_rp = h264->config_search_rp;
		h264->restore_config_rp = false;
		h264->config_search_valid = false;
	}
	mutex_unlock(&h264->lock);
	return 0;
}

static int codec_h264_multi_resume(struct amvdec_session *sess)
{
	struct codec_h264_multi *h264 = sess->priv;
	struct amvdec_core *core = sess->core;
	unsigned long flags;
	bool active;
	int ret = 0;

	if (!h264)
		return -EINVAL;

	mutex_lock(&h264->lock);
	/* Replace a resolution-sized MV buffer only after the old run stopped. */
	if (h264->config_valid) {
		/* Userspace may allocate more than the minimum; use all of them. */
		if (sess->streamon_cap && sess->num_dst_bufs > h264->capture_buf_count)
			h264->capture_buf_count =
				min_t(unsigned int, sess->num_dst_bufs,
				      sess->fmt_out->max_buffers);
		ret = codec_h264_multi_alloc_mv(sess, &h264->config,
						h264->capture_buf_count);
		if (ret)
			goto unlock;
	}
	spin_lock_irqsave(&core->irq_lock, flags);
	active = core->cur_sess == sess;
	spin_unlock_irqrestore(&core->irq_lock, flags);
	dev_info(core->dev,
		 "H.264 source resume: active=%u configuring=%u status=%u changed=%u stopped=%u streamon_cap=%u config_valid=%u capture=%u allocated=%u\n",
		 active, h264->configuring, sess->status, sess->changed_format,
		 v4l2_m2m_has_stopped(sess->m2m_ctx), sess->streamon_cap,
		 h264->config_valid,
		 h264->capture_buf_count, sess->num_dst_bufs);

	if (active) {
		ret = codec_h264_multi_setup_canvases(sess);
		if (ret) {
			amvdec_abort(sess);
			goto unlock;
		}
	}
	if (active && !h264->configuring) {
		/* Vendor restarts NAL/SPS search after every restored run. */
		amvdec_write_dos(core, H264_MULTI_DPB_STATUS,
				 H264_MULTI_ACTION_SEARCH_HEAD);
	} else {
		h264->resume_pending = true;
	}

unlock:
	mutex_unlock(&h264->lock);
	return ret;
}

static irqreturn_t codec_h264_multi_isr(struct amvdec_session *sess)
{
	struct codec_h264_multi *h264 = sess->priv;
	u32 status;

	if (!h264)
		return IRQ_NONE;

	status = amvdec_read_dos(sess->core, H264_MULTI_DPB_STATUS);
	WRITE_ONCE(h264->irq_status, status);
	atomic_inc(&h264->watchdog_event);
	if (READ_ONCE(h264->watchdog_armed))
		mod_delayed_work(system_wq, &h264->watchdog_work,
				 msecs_to_jiffies(H264_MULTI_WATCHDOG_POLL_MS));
	amvdec_write_dos(sess->core, ASSIST_MBOX1_CLR_REG, 1);
	return IRQ_WAKE_THREAD;
}

static int
codec_h264_multi_alloc_mv(struct amvdec_session *sess,
			  const struct h264_multi_config *config,
			  unsigned int capture_buf_count)
{
	struct codec_h264_multi *h264 = sess->priv;
	struct amvdec_core *core = sess->core;
	dma_addr_t paddr;
	void *vaddr;
	size_t slot_size;
	size_t size;
	u32 mb_height;
	u32 mb_width;

	mb_width = ALIGN(DIV_ROUND_UP(config->coded_width, 16), 4);
	mb_height = ALIGN(DIV_ROUND_UP(config->coded_height, 16), 4);
	if (check_mul_overflow((size_t)mb_width, (size_t)mb_height,
			       &slot_size) ||
	    check_mul_overflow(slot_size, (size_t)H264_MULTI_CO_MB_SIZE,
			       &slot_size) ||
	    check_mul_overflow(slot_size, (size_t)capture_buf_count, &size))
		return -EOVERFLOW;
	size = PAGE_ALIGN(size);
	if (h264->mv_vaddr && h264->mv_size == size) {
		h264->mv_slot_size = slot_size;
		return 0;
	}

	vaddr = dma_alloc_coherent(core->dev, size, &paddr, GFP_KERNEL);
	if (!vaddr)
		return -ENOMEM;
	memset(vaddr, 0, size);
	if (h264->mv_vaddr)
		dma_free_coherent(core->dev, h264->mv_size,
				  h264->mv_vaddr, h264->mv_paddr);
	h264->mv_vaddr = vaddr;
	h264->mv_paddr = paddr;
	h264->mv_size = size;
	h264->mv_slot_size = slot_size;

	return 0;
}

static int codec_h264_multi_configure(struct amvdec_session *sess)
{
	struct codec_h264_multi *h264 = sess->priv;
	struct amvdec_core *core = sess->core;
	struct h264_multi_config config;
	struct h264_multi_frame *last_frame = NULL;
	unsigned int capture_buf_count;
	bool capture_change;
	bool config_changed;
	u32 seq_info2;
	u32 seq_info;
	u32 crop_info;
	u32 param4;
	int ret;

	seq_info2 = amvdec_read_dos(core, H264_MULTI_SEQ_INFO2);
	seq_info = amvdec_read_dos(core, H264_MULTI_SEQ_INFO);
	crop_info = amvdec_read_dos(core, H264_MULTI_CROP_INFO);
	param4 = amvdec_read_dos(core, H264_MULTI_PARAM4);
	ret = codec_h264_multi_parse_config(sess, seq_info2, seq_info,
					    crop_info, param4, &config);
	if (ret)
		return ret;

	ret = h264_multi_dpb_buf_count(&config, &capture_buf_count);
	if (ret)
		return ret;
	capture_buf_count = max(capture_buf_count, 11U);
	config_changed = h264->config_valid &&
		memcmp(&h264->config, &config, sizeof(config));
	capture_change = sess->streamon_cap &&
		(sess->width != config.width || sess->height != config.height ||
		 (sess->bitdepth && sess->bitdepth != 8) ||
		 capture_buf_count > sess->num_dst_bufs);
	if (config_changed) {
		/*
		 * Vendor closes the old sequence before replacing its DPB.  A new
		 * SPS can arrive instead of another slice header, so finish the
		 * active picture explicitly and release all reordered output first.
		 */
		if (h264->pic_state.active) {
			ret = codec_h264_multi_finish_picture(sess);
			if (ret)
				return ret;
		}
		/*
		 * Keep the final real frame separate so the V4L2 source-change event
		 * can be queued before it is returned as the LAST old-sequence frame.
		 * This preserves vendor's complete DPB flush while following the
		 * stateful decoder resolution-change protocol.
		 */
		if (capture_change)
			last_frame =
				codec_h264_multi_flush_before_source_change(sess);
		else
			codec_h264_multi_flush_output(sess);
		h264->pic_done_pending = false;
	}
	if (!capture_change) {
		ret = codec_h264_multi_alloc_mv(sess, &config,
						capture_buf_count);
		if (ret)
			return ret;
	}

	if (!h264->config_valid || config_changed) {
		h264_multi_dpb_reset(&h264->dpb);
		h264_multi_dpb_picture_reset(&h264->pic_state);
	}

	h264->config = config;
	h264->config_valid = true;
	h264->capture_buf_count = capture_buf_count;
	h264->decode_seqinfo = seq_info2;

	h264->configuring = true;
	mutex_unlock(&h264->lock);
	amvdec_src_change(sess, config.width, config.height,
			  capture_buf_count, 8);
	mutex_lock(&h264->lock);
	h264->configuring = false;
	if (last_frame) {
		amvdec_dst_buf_done_ts_last(sess, last_frame->vbuf,
					    V4L2_FIELD_NONE,
					    last_frame->type,
					    &last_frame->timestamp);
		kfree(last_frame);
	} else if (capture_change &&
		   !amvdec_dst_buf_done_empty_last(sess))
		dev_warn(core->dev,
			 "H.264 source change has no capture buffer for empty LAST\n");
	if (capture_change)
		dev_info(core->dev,
			 "H.264 source change queued: status=%u changed=%u stopped=%u pending=%u streamon_cap=%u capture=%u allocated=%u\n",
			 sess->status, sess->changed_format,
			 v4l2_m2m_has_stopped(sess->m2m_ctx),
			 READ_ONCE(sess->source_change_pending),
			 sess->streamon_cap, h264->capture_buf_count,
			 sess->num_dst_bufs);
	return 0;
}

static int
codec_h264_multi_write_ref_list(struct amvdec_session *sess,
				const struct v4l2_h264_reference *refs,
				unsigned int count)
{
	struct codec_h264_multi *h264 = sess->priv;
	struct amvdec_core *core = sess->core;
	unsigned int writes = 0;
	unsigned int packed = 0;
	unsigned int last_ref = 0;
	u32 value = 0;
	unsigned int i;

	for (i = 0; i < count; i++) {
		const struct h264_multi_dpb_slot *slot;
		u8 ref;

		if (refs[i].index >= H264_MULTI_DPB_SIZE)
			return -EINVAL;
		slot = &h264->dpb.slots[refs[i].index];
		if (!slot->active || slot->buffer_index >= h264->capture_buf_count)
			return -EINVAL;

		ref = slot->buffer_index & GENMASK(4, 0);
		if (refs[i].fields == V4L2_H264_FRAME_REF)
			ref |= 3 << 5;
		else if (refs[i].fields == V4L2_H264_TOP_FIELD_REF)
			ref |= 1 << 5;
		else if (refs[i].fields == V4L2_H264_BOTTOM_FIELD_REF)
			ref |= 2 << 5;
		else
			return -EINVAL;

		last_ref = ref;
		value = (value << 8) | ref;
		if (++packed == 4) {
			amvdec_write_dos(core, H264_MULTI_BUFFER_INFO_DATA, value);
			writes++;
			packed = 0;
			value = 0;
		}
	}

	if (packed) {
		while (packed++ < 4)
			value = (value << 8) | last_ref;
		amvdec_write_dos(core, H264_MULTI_BUFFER_INFO_DATA, value);
		writes++;
	}

	value = last_ref * 0x01010101;
	while (writes++ < 8)
		amvdec_write_dos(core, H264_MULTI_BUFFER_INFO_DATA, value);

	return 0;
}

static int codec_h264_multi_configure_references(struct amvdec_session *sess)
{
	struct codec_h264_multi *h264 = sess->priv;
	struct h264_multi_picture *picture = &h264->pic_state.picture;
	struct v4l2_ctrl_h264_decode_params *decode = &h264->decode_params;
	struct v4l2_h264_reflist_builder *builder = &h264->reflist_builder;
	struct v4l2_ctrl_h264_sps *sps = &h264->sps;
	unsigned int l0_count = 0;
	unsigned int l1_count = 0;
	int ret;

	memset(decode, 0, sizeof(*decode));
	memset(sps, 0, sizeof(*sps));
	h264_multi_dpb_to_v4l2(&h264->dpb, &h264->config, picture,
			       decode->dpb);
	decode->frame_num = picture->frame_num;
	decode->top_field_order_cnt = h264->pic_state.poc.top;
	decode->bottom_field_order_cnt = h264->pic_state.poc.bottom;
	sps->log2_max_frame_num_minus4 = ilog2(h264->config.max_frame_num) - 4;

	v4l2_h264_init_reflist_builder(builder, decode, sps, decode->dpb);
	if (picture->slice_type == V4L2_H264_SLICE_TYPE_B) {
		v4l2_h264_build_b_ref_lists(builder, h264->ref_list0,
					    h264->ref_list1);
		l0_count = picture->num_ref_idx_l0_active;
		l1_count = picture->num_ref_idx_l1_active;
	} else if (picture->slice_type == V4L2_H264_SLICE_TYPE_P ||
		   picture->slice_type == V4L2_H264_SLICE_TYPE_SP) {
		v4l2_h264_build_p_ref_list(builder, h264->ref_list0);
		l0_count = picture->num_ref_idx_l0_active;
	}
	if (l0_count > ARRAY_SIZE(h264->ref_list0) ||
	    l1_count > ARRAY_SIZE(h264->ref_list1)) {
		dev_err(sess->core->dev,
			"invalid H.264 reference counts: L0=%u L1=%u\n",
			l0_count, l1_count);
		return -EINVAL;
	}
	if (l0_count) {
		ret = h264_multi_dpb_reorder_reflist(&h264->dpb,
						     &h264->config, picture,
						     h264->ref_list0,
						     builder->num_valid,
						     l0_count,
						     h264->lmem.data.mmco.l0_reorder,
						     H264_MULTI_LMEM_REORDER_WORDS);
		if (ret) {
			dev_err(sess->core->dev,
				"unable to reorder H.264 L0 references: %d\n",
				ret);
			return ret;
		}
	}
	if (l1_count) {
		ret = h264_multi_dpb_reorder_reflist(&h264->dpb,
						     &h264->config, picture,
						     h264->ref_list1,
						     builder->num_valid,
						     l1_count,
						     h264->lmem.data.mmco.l1_reorder,
						     H264_MULTI_LMEM_REORDER_WORDS);
		if (ret) {
			dev_err(sess->core->dev,
				"unable to reorder H.264 L1 references: %d\n", ret);
			return ret;
		}
	}

	amvdec_write_dos(sess->core, H264_MULTI_BUFFER_INFO_INDEX, 0);
	ret = codec_h264_multi_write_ref_list(sess, h264->ref_list0, l0_count);
	if (ret) {
		dev_err(sess->core->dev,
			"unable to write H.264 L0 references: %d\n", ret);
		return ret;
	}
	amvdec_write_dos(sess->core, H264_MULTI_BUFFER_INFO_INDEX, 8);
	ret = codec_h264_multi_write_ref_list(sess, h264->ref_list1, l1_count);
	if (ret)
		dev_err(sess->core->dev,
			"unable to write H.264 L1 references: %d\n", ret);

	return ret;
}

static int codec_h264_multi_configure_mv(struct amvdec_session *sess)
{
	struct codec_h264_multi *h264 = sess->priv;
	struct h264_multi_picture *picture = &h264->pic_state.picture;
	struct amvdec_core *core = sess->core;
	const struct h264_multi_dpb_slot *ref_slot;
	dma_addr_t addr;
	size_t offset;
	size_t slot_size;
	u16 mode_flags;
	u32 value;
	u8 ref_type;
	bool compact;
	int ret;

	if (!h264->mv_vaddr)
		return -EINVAL;
	mode_flags = h264->lmem.data.params[H264_MULTI_PARAM_MODE_8X8_FLAGS];
	compact = (mode_flags & BIT(2)) && (mode_flags & BIT(1));
	slot_size = h264->mv_slot_size >> (compact ? 2 : 0);
	if (check_mul_overflow((size_t)picture->first_mb_in_slice,
			       (size_t)(H264_MULTI_CO_MB_SIZE >>
					(compact ? 2 : 0)), &offset) ||
	    offset >= slot_size)
		return -ERANGE;

	ret = read_poll_timeout(amvdec_read_dos, value, !(value & BIT(11)),
				1, 1000, false, core, H264_MULTI_CO_MB_RW_CTL);
	if (ret)
		return ret;

	addr = h264->mv_paddr + slot_size * h264->pic_state.buffer_index;
	amvdec_write_dos(core, H264_MULTI_CO_MB_WR_ADDR, addr + offset);
	if (picture->slice_type != V4L2_H264_SLICE_TYPE_B)
		return 0;
	if (!picture->num_ref_idx_l1_active ||
	    h264->ref_list1[0].index >= H264_MULTI_DPB_SIZE)
		return -EINVAL;

	ref_slot = &h264->dpb.slots[h264->ref_list1[0].index];
	if (!ref_slot->active ||
	    ref_slot->buffer_index >= h264->capture_buf_count)
		return -EINVAL;
	addr = h264->mv_paddr + slot_size * ref_slot->buffer_index + offset;
	ref_type = abs(h264->pic_state.poc.top -
		       ref_slot->top_field_order_cnt) <
		   abs(h264->pic_state.poc.top -
		       ref_slot->bottom_field_order_cnt) ? 0 : 1;
	amvdec_write_dos(core, H264_MULTI_CO_MB_RD_ADDR,
			 (2 << 30) | (ref_type << 29) |
			 ((addr >> 3) & GENMASK(28, 0)));

	return 0;
}

static u32 h264_multi_buffer_info(const struct h264_multi_dpb_slot *slot,
				  bool is_current,
				  const struct h264_multi_poc *poc)
{
	u32 info = 0xf480;

	if (slot && slot->long_term)
		info |= BIT(4) | BIT(5);
	if ((slot && slot->bottom_field_order_cnt < slot->top_field_order_cnt) ||
	    (is_current && poc->bottom < poc->top))
		info |= BIT(8);
	if (is_current)
		info |= 0xf;

	return info;
}

static const struct h264_multi_dpb_slot *
codec_h264_multi_find_buffer(struct codec_h264_multi *h264,
			     unsigned int buffer_index)
{
	unsigned int i;

	for (i = 0; i < H264_MULTI_DPB_SIZE; i++) {
		if (h264->dpb.slots[i].active &&
		    h264->dpb.slots[i].buffer_index == buffer_index)
			return &h264->dpb.slots[i];
	}

	return NULL;
}

static int codec_h264_multi_buffer_index(struct amvdec_session *sess,
					 unsigned int vb2_index,
					 u32 *buffer_index)
{
	unsigned int i;

	for (i = 0; i < sess->num_dst_bufs; i++) {
		if (sess->fw_idx_to_vb2_idx[i] != vb2_index)
			continue;
		*buffer_index = i;
		return 0;
	}

	return -ENOENT;
}

static int codec_h264_multi_get_buffer(struct amvdec_session *sess,
				       struct vb2_v4l2_buffer **vbuf,
				       u32 *buffer_index)
{
	struct codec_h264_multi *h264 = sess->priv;
	struct vb2_v4l2_buffer *candidate;
	unsigned int attempts;
	u32 index;

	for (attempts = 0; attempts < sess->num_dst_bufs; attempts++) {
		candidate = v4l2_m2m_dst_buf_remove(sess->m2m_ctx);
		if (!candidate)
			break;
		if (!codec_h264_multi_buffer_index(sess,
						   candidate->vb2_buf.index,
						   &index)) {
			if (index < h264->capture_buf_count &&
			    !codec_h264_multi_find_buffer(h264, index)) {
				*vbuf = candidate;
				*buffer_index = index;
				return 0;
			}
		}
		v4l2_m2m_buf_queue(sess->m2m_ctx, candidate);
	}

	return -ENOBUFS;
}

static bool codec_h264_multi_job_ready(struct amvdec_session *sess)
{
	struct v4l2_m2m_queue_ctx *queue = &sess->m2m_ctx->cap_q_ctx;
	struct codec_h264_multi *h264 = sess->priv;
	struct v4l2_m2m_buffer *buffer;
	unsigned long flags;
	unsigned int free_buffers = 0;
	unsigned int min_buffers = H264_MULTI_RUN_READY_MIN_BUFS;
	u32 index;

	if (!h264)
		return false;

	mutex_lock(&h264->lock);
	if (h264->resume_pending) {
		mutex_unlock(&h264->lock);
		return true;
	}
	if (!h264->slice_pending && !h264->input_pending) {
		mutex_unlock(&h264->lock);
		return false;
	}
	/* The first run must parse the sequence before the DPB size is known. */
	if (!h264->config_valid) {
		mutex_unlock(&h264->lock);
		return true;
	}
	if (h264->slice_pending)
		min_buffers = 1;

	spin_lock_irqsave(&queue->rdy_spinlock, flags);
	list_for_each_entry(buffer, &queue->rdy_queue, list) {
		if (codec_h264_multi_buffer_index(sess,
						  buffer->vb.vb2_buf.index,
						  &index))
			continue;
		if (index < h264->capture_buf_count &&
		    !codec_h264_multi_find_buffer(h264, index) &&
		    ++free_buffers >= min_buffers)
			break;
	}
	spin_unlock_irqrestore(&queue->rdy_spinlock, flags);
	mutex_unlock(&h264->lock);

	return free_buffers >= min_buffers;
}

static bool codec_h264_multi_has_pending_job(struct amvdec_session *sess)
{
	struct codec_h264_multi *h264 = sess->priv;

	return h264 && (READ_ONCE(h264->resume_pending) ||
			READ_ONCE(h264->slice_pending) ||
			READ_ONCE(h264->input_pending));
}

static bool codec_h264_multi_has_queued_input(struct amvdec_session *sess)
{
	unsigned long flags;
	bool queued;

	spin_lock_irqsave(&sess->ts_spinlock, flags);
	queued = !list_empty(&sess->timestamps);
	spin_unlock_irqrestore(&sess->ts_spinlock, flags);

	return queued;
}

static bool codec_h264_multi_is_last_input(struct amvdec_session *sess)
{
	return sess->draining &&
		!v4l2_m2m_num_src_bufs_ready(sess->m2m_ctx) &&
		!codec_h264_multi_has_queued_input(sess);
}

static void codec_h264_multi_finish_drain(struct amvdec_session *sess)
{
	sess->should_stop = 1;
	codec_h264_multi_flush_output(sess);
	v4l2_m2m_mark_stopped(sess->m2m_ctx);
	sess->draining = false;
}

static int
codec_h264_multi_configure_picture(struct amvdec_session *sess,
				   enum h264_multi_slice_action action)
{
	struct codec_h264_multi *h264 = sess->priv;
	struct amvdec_core *core = sess->core;
	unsigned int i;
	u32 fw_action;
	u32 canvas;
	int ret;

	if (!h264 || !h264->pic_state.active ||
	    h264->pic_state.buffer_index >= sess->num_dst_bufs)
		return -EINVAL;
	if (codec_h264_multi_find_buffer(h264,
					 h264->pic_state.buffer_index))
		return -EBUSY;
	amvdec_write_dos(core, H264_MULTI_CURRENT_POC_INDEX, 0);
	amvdec_write_dos(core, H264_MULTI_CURRENT_POC,
			 min(h264->pic_state.poc.top,
			     h264->pic_state.poc.bottom));
	amvdec_write_dos(core, H264_MULTI_CURRENT_POC,
			 h264->pic_state.poc.top);
	amvdec_write_dos(core, H264_MULTI_CURRENT_POC,
			 h264->pic_state.poc.bottom);

	amvdec_write_dos(core, H264_MULTI_CURR_CANVAS_CTRL,
			 h264->pic_state.buffer_index << 24);
	canvas = amvdec_read_dos(core, H264_MULTI_CURR_CANVAS_CTRL) & 0xffffff;
	amvdec_write_dos(core, H264_MULTI_REC_CANVAS_ADDR, canvas);
	amvdec_write_dos(core, H264_MULTI_DBKR_CANVAS_ADDR, canvas);
	amvdec_write_dos(core, H264_MULTI_DBKW_CANVAS_ADDR, canvas);

	amvdec_write_dos(core, H264_MULTI_BUFFER_INFO_INDEX, 16);
	for (i = 0; i < h264->capture_buf_count; i++) {
		const struct h264_multi_dpb_slot *slot;
		bool is_current = i == h264->pic_state.buffer_index;
		s32 top = 0;
		s32 bottom = 0;

		slot = codec_h264_multi_find_buffer(h264, i);
		if (slot) {
			top = slot->top_field_order_cnt;
			bottom = slot->bottom_field_order_cnt;
		}
		if (i == h264->pic_state.buffer_index) {
			top = h264->pic_state.poc.top;
			bottom = h264->pic_state.poc.bottom;
		}
		if (slot || is_current) {
			u32 info = h264_multi_buffer_info(slot, is_current,
							  &h264->pic_state.poc);

			amvdec_write_dos(core, H264_MULTI_BUFFER_INFO_DATA, info);
			amvdec_write_dos(core, H264_MULTI_BUFFER_INFO_DATA, top);
			amvdec_write_dos(core, H264_MULTI_BUFFER_INFO_DATA, bottom);
		} else {
			amvdec_write_dos(core, H264_MULTI_BUFFER_INFO_DATA, 0);
			amvdec_write_dos(core, H264_MULTI_BUFFER_INFO_DATA, 0);
			amvdec_write_dos(core, H264_MULTI_BUFFER_INFO_DATA, 0);
		}
	}
	ret = codec_h264_multi_configure_references(sess);
	if (ret) {
		dev_err(core->dev, "unable to configure H.264 references: %d\n",
			ret);
		return ret;
	}
	ret = codec_h264_multi_configure_mv(sess);
	if (ret) {
		dev_err(core->dev, "unable to configure H.264 MV state: %d\n",
			ret);
		return ret;
	}
	fw_action = action == H264_MULTI_SLICE_NEW_PICTURE ?
		H264_MULTI_ACTION_DECODE_NEWPIC :
		H264_MULTI_ACTION_DECODE_SLICE;
	amvdec_write_dos(core, H264_MULTI_DPB_STATUS, fw_action);

	return 0;
}

static int
codec_h264_multi_process_picture(struct amvdec_session *sess,
				 const struct h264_multi_picture *picture)
{
	struct codec_h264_multi *h264 = sess->priv;
	struct vb2_v4l2_buffer *vbuf = NULL;
	enum h264_multi_slice_action action;
	u32 buffer_index = 0;
	int ret = 0;

	if (h264->pic_state.active)
		buffer_index = h264->pic_state.buffer_index;
	else
		ret = codec_h264_multi_get_buffer(sess, &vbuf, &buffer_index);
	if (!h264->pic_state.active && ret)
		return ret;

	ret = h264_multi_dpb_picture_begin(&h264->dpb, &h264->config,
					   &h264->pic_state, picture,
					   buffer_index, &action);
	if (ret)
		dev_err(sess->core->dev,
			"unable to begin H.264 picture: %d (active=%u first_mb=%u)\n",
			ret, h264->pic_state.active, picture->first_mb_in_slice);
	if (!ret && vbuf)
		h264->pic_state.vbuf = vbuf;
	if (!ret)
		ret = codec_h264_multi_configure_picture(sess, action);
	if (ret && vbuf) {
		h264_multi_dpb_picture_reset(&h264->pic_state);
		v4l2_m2m_buf_queue(sess->m2m_ctx, vbuf);
	}

	return ret;
}

static int codec_h264_multi_resume_pending_picture(struct amvdec_session *sess)
{
	struct codec_h264_multi *h264 = sess->priv;
	int ret;

	dev_info(sess->core->dev,
		 "H.264 pending picture resume: frame_num=%u first_mb=%u active=%u pic_done_pending=%u\n",
		 h264->pending_picture.frame_num,
		 h264->pending_picture.first_mb_in_slice,
		 h264->pic_state.active, h264->pic_done_pending);
	ret = codec_h264_multi_process_picture(sess, &h264->pending_picture);
	if (!ret)
		WRITE_ONCE(h264->slice_pending, false);

	return ret;
}

static void codec_h264_multi_capture_queued(struct amvdec_session *sess)
{
	struct codec_h264_multi *h264 = sess->priv;
	struct amvdec_core *core = sess->core;
	int ret;

	if (!h264 || !READ_ONCE(h264->slice_pending))
		return;

	mutex_lock(&core->hw_lock);
	if (!amvdec_session_is_current(sess) ||
	    atomic_read(&sess->m2m_job_running) != AMVDEC_M2M_JOB_RUNNING ||
	    !codec_h264_multi_job_ready(sess))
		goto unlock_hardware;

	mutex_lock(&h264->lock);
	if (!h264->slice_pending)
		goto unlock_codec;

	ret = codec_h264_multi_resume_pending_picture(sess);
	if (!ret) {
		codec_h264_multi_watchdog_arm(h264);
	} else if (ret != -ENOBUFS) {
		dev_err(core->dev,
			"unable to resume H.264 picture after capture queue: %d\n",
			ret);
		amvdec_abort(sess);
	}

unlock_codec:
	mutex_unlock(&h264->lock);
unlock_hardware:
	mutex_unlock(&core->hw_lock);
}

static u32 codec_h264_multi_picture_type(const struct h264_multi_picture *picture)
{
	if (picture->nal_unit_type == 5)
		return 4;
	if (picture->slice_type == 2 || picture->slice_type == 4)
		return 1;
	if (picture->slice_type == 1)
		return 3;
	return 2;
}

static void codec_h264_multi_queue_frame(struct amvdec_session *sess,
					 struct h264_multi_frame *frame,
					 bool no_output_of_prior_pics)
{
	struct codec_h264_multi *h264 = sess->priv;
	unsigned int reorder_limit;

	mutex_lock(&h264->frames_lock);
	if (frame->type == 4) {
		if (no_output_of_prior_pics)
			__codec_h264_multi_discard_output(sess);
		else
			__codec_h264_multi_flush_output(sess);
	}

	list_add_tail(&frame->list, &h264->frames);
	h264->frame_count++;
	if (h264->config.bitstream_restriction)
		reorder_limit = h264->config.num_reorder_frames;
	else
		reorder_limit = min_t(unsigned int, h264->config.max_refs,
				      h264->capture_buf_count - 1);
	while (h264->frame_count > reorder_limit) {
		struct h264_multi_frame *next;

		next = codec_h264_multi_next_frame(h264);
		if (codec_h264_multi_find_buffer(h264,
						 next->buffer_index))
			break;
		codec_h264_multi_output_frame(sess, next);
	}
	mutex_unlock(&h264->frames_lock);
}

static int codec_h264_multi_finish_picture(struct amvdec_session *sess)
{
	struct codec_h264_multi *h264 = sess->priv;
	struct h264_multi_frame *frame;
	u32 buffer_index;
	int ret;

	if (!h264->pic_state.active || !h264->pic_state.vbuf)
		return -EINVAL;

	frame = kzalloc_obj(*frame);
	if (!frame) {
		v4l2_m2m_buf_done(h264->pic_state.vbuf, VB2_BUF_STATE_ERROR);
		h264_multi_dpb_picture_reset(&h264->pic_state);
		return -ENOMEM;
	}

	buffer_index = h264->pic_state.buffer_index;
	frame->vbuf = h264->pic_state.vbuf;
	frame->buffer_index = buffer_index;
	frame->poc = min(h264->pic_state.poc.top,
			 h264->pic_state.poc.bottom);
	frame->type = codec_h264_multi_picture_type(&h264->pic_state.picture);
	ret = amvdec_take_ts(sess, &frame->timestamp);
	if (ret) {
		v4l2_m2m_buf_done(frame->vbuf, VB2_BUF_STATE_ERROR);
		h264_multi_dpb_picture_reset(&h264->pic_state);
		goto free_frame;
	}

	ret = h264_multi_dpb_picture_finish(&h264->dpb, &h264->config,
					    &h264->pic_state,
					    &h264->pic_marking,
					    frame->timestamp.timestamp,
					    buffer_index);
	if (ret) {
		v4l2_m2m_buf_done(frame->vbuf, VB2_BUF_STATE_ERROR);
		atomic_dec_if_positive(&sess->esparser_queued_bufs);
		h264_multi_dpb_picture_reset(&h264->pic_state);
		goto free_frame;
	}
	codec_h264_multi_queue_frame(sess, frame,
				     h264->pic_marking.no_output_of_prior_pics);
	return 0;

free_frame:
	kfree(frame);
	return ret;
}

static irqreturn_t codec_h264_multi_threaded_isr(struct amvdec_session *sess)
{
	struct codec_h264_multi *h264 = sess->priv;
	irqreturn_t ret = IRQ_HANDLED;
	bool yield = false;
	u32 status;

	if (!h264)
		return IRQ_NONE;

	mutex_lock(&h264->lock);
	status = READ_ONCE(h264->irq_status);
	if (status == H264_MULTI_CONFIG_REQUEST)
		h264->restore_config_rp = true;

	if (status == H264_MULTI_WRRSP_REQUEST) {
		amvdec_write_dos(sess->core, H264_MULTI_DPB_STATUS,
				 H264_MULTI_WRRSP_DONE);
		goto unlock;
	}
	if (status == H264_MULTI_SEI_DATA_READY) {
		amvdec_write_dos(sess->core, H264_MULTI_DPB_STATUS,
				 H264_MULTI_SEI_DATA_DONE);
		goto unlock;
	}
	if (status == H264_MULTI_SEARCH_BUFEMPTY) {
		if (!h264->initialized) {
			if (codec_h264_multi_read_lmem(sess)) {
				ret = IRQ_NONE;
				goto unlock;
			}
			h264->initialized = true;
		}
		yield = true;
		goto unlock;
	}
	if (status == H264_MULTI_AUX_DATA_READY) {
		yield = true;
		goto unlock;
	}
	if (status == H264_MULTI_DECODE_TIMEOUT ||
	    status == H264_MULTI_DECODE_OVER_SIZE ||
	    status == H264_MULTI_DECODE_ERROR_RESET ||
	    status == H264_MULTI_DECODE_INIT_RESET) {
		dev_err(sess->core->dev,
			"fatal H.264 firmware status: %#x\n", status);
		amvdec_abort(sess);
		yield = true;
		goto unlock;
	}

	if (status == H264_MULTI_CONFIG_REQUEST ||
	    status == H264_MULTI_SLICE_HEAD_DONE ||
	    status == H264_MULTI_PIC_DATA_DONE ||
	    status == H264_MULTI_DATA_REQUEST ||
	    status == H264_MULTI_DECODE_BUFEMPTY) {
		if (codec_h264_multi_read_lmem(sess)) {
			ret = IRQ_NONE;
			goto unlock;
		}
	}

	if (status == H264_MULTI_CONFIG_REQUEST) {
		int err = codec_h264_multi_configure(sess);

		if (err) {
			dev_err(sess->core->dev,
				"unable to configure H.264 sequence: %d\n", err);
			amvdec_abort(sess);
			yield = true;
			goto unlock;
		}
		amvdec_write_dos(sess->core, H264_MULTI_DPB_STATUS,
				 H264_MULTI_ACTION_CONFIG_DONE);
		h264->initialized = true;
		yield = true;
		goto unlock;
	}

	if (status == H264_MULTI_SLICE_HEAD_DONE) {
		struct h264_multi_picture picture;
		struct h264_multi_marking marking;
		const char *operation = "parse picture";
		int ret;

		ret = codec_h264_multi_parse_picture(sess, &picture);
		if (!ret && !picture.first_mb_in_slice) {
			operation = "parse marking";
			ret = codec_h264_multi_parse_marking(sess, &marking);
		}
		if (!ret && !picture.first_mb_in_slice &&
		    h264->pic_state.active) {
			operation = "finish previous picture";
			ret = codec_h264_multi_finish_picture(sess);
			if (!ret)
				h264->pic_done_pending = true;
		}
		if (!ret) {
			operation = "process picture";
			ret = codec_h264_multi_process_picture(sess, &picture);
		}
		if (!ret && !picture.first_mb_in_slice)
			h264->pic_marking = marking;
		if (ret == -ENOBUFS) {
			dev_info(sess->core->dev,
				 "H.264 picture waits for capture: frame_num=%u first_mb=%u active=%u pic_done_pending=%u\n",
				 picture.frame_num, picture.first_mb_in_slice,
				 h264->pic_state.active,
				 h264->pic_done_pending);
			if (!picture.first_mb_in_slice)
				h264->pic_marking = marking;
			h264->pending_picture = picture;
			WRITE_ONCE(h264->slice_pending, true);
			codec_h264_multi_watchdog_disarm(h264);
			/*
			 * capture_queued() ignores buffers queued before
			 * slice_pending was set, so look again after setting it.
			 */
			ret = codec_h264_multi_resume_pending_picture(sess);
			if (!ret) {
				codec_h264_multi_watchdog_arm(h264);
			} else if (ret != -ENOBUFS) {
				dev_err(sess->core->dev,
					"unable to resume H.264 picture: %d\n", ret);
				amvdec_abort(sess);
				yield = true;
			}
			ret = 0;
			goto unlock;
		}
		if (ret) {
			dev_err(sess->core->dev,
				"unable to %s: %d\n", operation, ret);
			amvdec_abort(sess);
			yield = true;
		}
		goto unlock;
	}
	if (status == H264_MULTI_DATA_REQUEST ||
	    status == H264_MULTI_DECODE_BUFEMPTY) {
		int ret = 0;

		/* Vendor frame mode completes the chunk on request or empty. */
		if (h264->pic_state.active)
			ret = codec_h264_multi_finish_picture(sess);
		if (ret) {
			dev_err(sess->core->dev,
				"unable to finish current H.264 picture: %d\n",
				ret);
			amvdec_abort(sess);
		}
		h264->input_pending = false;
		if (!ret && codec_h264_multi_is_last_input(sess))
			codec_h264_multi_finish_drain(sess);
		yield = true;
		goto unlock;
	}

	if (status == H264_MULTI_PIC_DATA_DONE) {
		int ret;

		if (h264->pic_done_pending) {
			dev_info(sess->core->dev,
				 "H.264 deferred picture done: active=%u frame_num=%u first_mb=%u slice_pending=%u\n",
				 h264->pic_state.active,
				 h264->pic_state.picture.frame_num,
				 h264->pic_state.picture.first_mb_in_slice,
				 h264->slice_pending);
			h264->pic_done_pending = false;
			h264->input_pending =
				codec_h264_multi_has_queued_input(sess);
			if (codec_h264_multi_is_last_input(sess))
				codec_h264_multi_finish_drain(sess);
			yield = true;
			goto unlock;
		}
		ret = codec_h264_multi_finish_picture(sess);
		if (ret) {
			dev_err(sess->core->dev,
				"unable to finish H.264 picture: %d\n", ret);
			amvdec_abort(sess);
		}
		h264->input_pending = codec_h264_multi_has_queued_input(sess);
		if (!ret && codec_h264_multi_is_last_input(sess))
			codec_h264_multi_finish_drain(sess);
		yield = true;
	}

unlock:
	if (yield)
		codec_h264_multi_watchdog_disarm(h264);
	mutex_unlock(&h264->lock);
	if (yield)
		amvdec_m2m_job_yield(sess);
	return ret;
}

struct amvdec_codec_ops codec_h264_g12a_ops = {
	.start = codec_h264_multi_start,
	.run = codec_h264_multi_run,
	.input_queued = codec_h264_multi_input_queued,
	.can_queue_input = codec_h264_multi_can_queue_input,
	.capture_queued = codec_h264_multi_capture_queued,
	.async_drain = true,
	.stop = codec_h264_multi_stop,
	.release = codec_h264_multi_release_firmware,
	.context_switching = true,
	.canvas_height_align = 16,
	.capture_stride_align = 64,
	.prepare_firmware = codec_h264_multi_prepare_firmware,
	.has_pending_job = codec_h264_multi_has_pending_job,
	.job_ready = codec_h264_multi_job_ready,
	.drain = codec_h264_multi_flush_output,
	.resume = codec_h264_multi_resume,
	.isr = codec_h264_multi_isr,
	.threaded_isr = codec_h264_multi_threaded_isr,
};

int codec_h264_multi_read_lmem(struct amvdec_session *sess)
{
	struct codec_h264_multi *h264 = sess->priv;
	const u16 *src;
	unsigned int i;
	unsigned int j;

	if (!h264 || !h264->lmem_vaddr)
		return -EINVAL;

	src = h264->lmem_vaddr;
	dma_rmb();
	for (i = 0; i < H264_MULTI_LMEM_WORDS; i += 4) {
		for (j = 0; j < 4; j++)
			h264->lmem.words[i + j] = src[i + 3 - j];
	}

	return 0;
}

u16 codec_h264_multi_lmem_word(struct amvdec_session *sess,
			       unsigned int index)
{
	struct codec_h264_multi *h264 = sess->priv;

	if (!h264 || index >= H264_MULTI_LMEM_WORDS)
		return 0;

	return h264->lmem.words[index];
}

int codec_h264_multi_parse_config(struct amvdec_session *sess,
				  u32 seq_info2, u32 seq_info, u32 crop_info,
				  u32 param4,
				  struct h264_multi_config *config)
{
	struct codec_h264_multi *h264 = sess->priv;
	u32 crop_bottom;
	u32 crop_left;
	u32 crop_right;
	u32 crop_top;
	u32 mb_height;
	u32 mb_total;
	u32 mb_width;
	u16 sps_flags;
	int ret;

	if (!h264 || !config)
		return -EINVAL;

	memset(config, 0, sizeof(*config));
	mb_width = FIELD_GET(H264_MULTI_MB_WIDTH_MASK, seq_info2);
	mb_total = FIELD_GET(H264_MULTI_MB_TOTAL_MASK, seq_info2);
	if (!mb_width && mb_total)
		mb_width = 256;
	if (!mb_width || !mb_total || mb_total % mb_width)
		return -EINVAL;

	mb_height = mb_total / mb_width;
	config->coded_width = mb_width * 16;
	config->coded_height = mb_height * 16;
	if (config->coded_width > sess->fmt_out->max_width ||
	    config->coded_height > sess->fmt_out->max_height)
		return -ERANGE;

	config->chroma_format_idc =
		FIELD_GET(H264_MULTI_CHROMA_FORMAT_MASK, seq_info);
	config->frame_mbs_only = !!(seq_info & H264_MULTI_FRAME_MBS_ONLY);
	config->max_refs = FIELD_GET(H264_MULTI_MAX_REFS_MASK, param4);
	config->profile_idc = h264->lmem.data.params
		[H264_MULTI_PARAM_PROFILE_IDC_MMCO] >> 8;
	config->level_idc = param4 & 0xff;
	config->num_reorder_frames = h264->lmem.data.params
		[H264_MULTI_PARAM_NUM_REORDER_FRAMES];
	config->max_dec_frame_buffering = h264->lmem.data.params
		[H264_MULTI_PARAM_MAX_BUFFER_FRAME];
	config->pic_order_cnt_type = h264->lmem.data.params
		[H264_MULTI_PARAM_PIC_ORDER_CNT_TYPE];
	config->num_ref_frames_in_poc_cycle = h264->lmem.data.params
		[H264_MULTI_PARAM_NUM_REF_FRAMES_IN_POC_CYCLE];
	config->offset_for_non_ref_pic = (s16)h264->lmem.data.params
		[H264_MULTI_PARAM_OFFSET_FOR_NON_REF_PIC];
	config->offset_for_top_to_bottom_field = (s16)h264->lmem.data.params
		[H264_MULTI_PARAM_OFFSET_FOR_TOP_TO_BOTTOM_FIELD];
	config->delta_pic_order_always_zero = !!h264->lmem.data.params
		[H264_MULTI_PARAM_DELTA_POC_ALWAYS_ZERO];
	config->frame_num_gap_allowed = !!h264->lmem.data.params
		[H264_MULTI_PARAM_FRAME_NUM_GAP_ALLOWED];
	sps_flags = h264->lmem.data.params[H264_MULTI_PARAM_SPS_FLAGS_2];
	config->bitstream_restriction =
		!!(sps_flags & H264_MULTI_SPS_BITSTREAM_RESTRICTION);

	if ((!config->level_idc && !config->bitstream_restriction) ||
	    (config->level_idc && config->level_idc < 9) ||
	    config->level_idc > 52 ||
	    config->max_refs > H264_MULTI_MAX_DPB_SIZE ||
	    config->pic_order_cnt_type > 2 ||
	    config->num_ref_frames_in_poc_cycle > H264_MULTI_LMEM_REF_WORDS)
		return -EINVAL;
	if (config->bitstream_restriction &&
	    (config->max_dec_frame_buffering > H264_MULTI_MAX_DPB_SIZE ||
	     config->num_reorder_frames > config->max_dec_frame_buffering))
		return -EINVAL;
	if (h264->lmem.data.params[H264_MULTI_PARAM_LOG2_MAX_FRAME_NUM] < 4 ||
	    h264->lmem.data.params[H264_MULTI_PARAM_LOG2_MAX_FRAME_NUM] > 16)
		return -EINVAL;
	config->max_frame_num = 1U << h264->lmem.data.params
		[H264_MULTI_PARAM_LOG2_MAX_FRAME_NUM];
	if (config->pic_order_cnt_type == 0) {
		u16 log2_max_poc = h264->lmem.data.params
			[H264_MULTI_PARAM_LOG2_MAX_PIC_ORDER_CNT_LSB];

		if (log2_max_poc < 4 || log2_max_poc > 16)
			return -EINVAL;
		config->max_pic_order_cnt_lsb = 1U << log2_max_poc;
	}
	for (ret = 0; ret < config->num_ref_frames_in_poc_cycle; ret++)
		config->offset_for_ref_frame[ret] =
			(s16)h264->lmem.data.mmco.offset_for_ref_frame[ret];

	crop_left = FIELD_GET(GENMASK(31, 24), crop_info);
	crop_right = FIELD_GET(GENMASK(23, 16), crop_info);
	crop_top = FIELD_GET(GENMASK(15, 8), crop_info);
	crop_bottom = FIELD_GET(GENMASK(7, 0), crop_info);
	if (crop_left + crop_right >= config->coded_width ||
	    crop_top + crop_bottom >= config->coded_height)
		return -EINVAL;

	config->width = config->coded_width - crop_left - crop_right;
	config->height = config->coded_height - crop_top - crop_bottom;

	return 0;
}

static int h264_multi_mmco_param(const u16 *commands, unsigned int *pos,
				 u16 *value)
{
	if (*pos >= H264_MULTI_LMEM_MMCO_CMD_WORDS)
		return -EINVAL;

	*value = commands[(*pos)++];
	return 0;
}

int codec_h264_multi_parse_marking(struct amvdec_session *sess,
				   struct h264_multi_marking *marking)
{
	struct codec_h264_multi *h264 = sess->priv;
	const struct h264_multi_lmem_dpb *dpb;
	const u16 *commands;
	unsigned int pos = 0;
	u16 nal_info;
	u16 opcode;
	int ret;

	if (!h264 || !marking)
		return -EINVAL;

	memset(marking, 0, sizeof(*marking));
	dpb = &h264->lmem.data.dpb;
	commands = h264->lmem.data.mmco.commands;
	nal_info = dpb->nal_info;

	if ((nal_info & GENMASK(4, 0)) == 5) {
		marking->long_term_reference = !!(commands[0] & BIT(0));
		marking->no_output_of_prior_pics = !!(commands[0] & BIT(1));
		return 0;
	}

	if (!FIELD_GET(GENMASK(6, 5), nal_info))
		return 0;

	while (pos < H264_MULTI_LMEM_MMCO_CMD_WORDS) {
		struct h264_multi_mmco *op;

		opcode = commands[pos++];
		if (!opcode)
			return 0;
		if (opcode > 6 || marking->count == H264_MULTI_MAX_MMCO_OPS)
			return -EINVAL;

		op = &marking->ops[marking->count++];
		op->opcode = opcode;
		marking->adaptive = true;

		switch (opcode) {
		case 1:
			ret = h264_multi_mmco_param(commands, &pos,
						    &op->difference_of_pic_nums_minus1);
			break;
		case 2:
			ret = h264_multi_mmco_param(commands, &pos,
						    &op->long_term_pic_num);
			break;
		case 3:
			ret = h264_multi_mmco_param(commands, &pos,
						    &op->difference_of_pic_nums_minus1);
			if (!ret)
				ret = h264_multi_mmco_param(commands, &pos,
							    &op->long_term_frame_idx);
			break;
		case 4:
			ret = h264_multi_mmco_param(commands, &pos,
						    &op->max_long_term_frame_idx_plus1);
			break;
		case 5:
			ret = 0;
			break;
		case 6:
			ret = h264_multi_mmco_param(commands, &pos,
						    &op->long_term_frame_idx);
			break;
		default:
			return -EINVAL;
		}
		if (ret)
			return ret;
	}

	return -EINVAL;
}

static s32 h264_multi_lmem_s32(const u16 value[2])
{
	return (s32)((u32)value[0] | (u32)value[1] << 16);
}

int codec_h264_multi_parse_picture(struct amvdec_session *sess,
				   struct h264_multi_picture *picture)
{
	struct codec_h264_multi *h264 = sess->priv;
	const struct h264_multi_lmem_dpb *dpb;
	u16 picture_structure;
	u16 slice_type;

	if (!h264 || !picture)
		return -EINVAL;

	memset(picture, 0, sizeof(*picture));
	dpb = &h264->lmem.data.dpb;
	picture->nal_unit_type = dpb->nal_info & GENMASK(4, 0);
	picture->nal_ref_idc = FIELD_GET(GENMASK(6, 5), dpb->nal_info);
	slice_type = h264->lmem.data.params[H264_MULTI_PARAM_SLICE_TYPE];
	if (slice_type > 9)
		return -EINVAL;
	picture->slice_type = slice_type % 5;
	picture->frame_num = dpb->frame_num;
	picture->pic_order_cnt_lsb = dpb->pic_order_cnt_lsb;
	picture->delta_pic_order_cnt_bottom =
		h264_multi_lmem_s32(dpb->delta_pic_order_cnt_bottom);
	picture->delta_pic_order_cnt[0] =
		h264_multi_lmem_s32(dpb->delta_pic_order_cnt[0]);
	picture->delta_pic_order_cnt[1] =
		h264_multi_lmem_s32(dpb->delta_pic_order_cnt[1]);
	picture->first_mb_in_slice =
		h264->lmem.data.params[H264_MULTI_PARAM_FIRST_MB_IN_SLICE];

	if (dpb->num_ref_idx_l0_active_minus1 >= V4L2_H264_REF_LIST_LEN ||
	    dpb->num_ref_idx_l1_active_minus1 >= V4L2_H264_REF_LIST_LEN)
		return -EINVAL;
	picture->num_ref_idx_l0_active =
		dpb->num_ref_idx_l0_active_minus1 + 1;
	picture->num_ref_idx_l1_active =
		dpb->num_ref_idx_l1_active_minus1 + 1;

	picture_structure = h264->lmem.data.params
		[H264_MULTI_PARAM_NEW_PICTURE_STRUCTURE];
	if (picture_structure > 3)
		return -EINVAL;
	picture->field_pic = picture_structure == 1 || picture_structure == 2;
	picture->bottom_field = picture_structure == 2;

	return 0;
}
