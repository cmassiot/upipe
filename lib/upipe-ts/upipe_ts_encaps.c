/*
 * Copyright (C) 2013-2017 OpenHeadend S.A.R.L.
 *
 * Authors: Christophe Massiot
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2.1 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston MA 02110-1301, USA.
 */

/** @file
 * @short Upipe module encapsulating (adding TS header) PES and PSI access units
 */

#include <upipe/ubase.h>
#include <upipe/ulist.h>
#include <upipe/uprobe.h>
#include <upipe/uref.h>
#include <upipe/uref_flow.h>
#include <upipe/uref_block.h>
#include <upipe/uref_block_flow.h>
#include <upipe/uref_pic.h>
#include <upipe/uref_clock.h>
#include <upipe/ubuf.h>
#include <upipe/uclock.h>
#include <upipe/upipe.h>
#include <upipe/upipe_helper_upipe.h>
#include <upipe/upipe_helper_urefcount.h>
#include <upipe/upipe_helper_void.h>
#include <upipe/upipe_helper_ubuf_mgr.h>
#include <upipe/upipe_helper_output.h>
#include <upipe/upipe_helper_input.h>
#include <upipe-ts/upipe_ts_encaps.h>
#include <upipe-ts/upipe_ts_mux.h>
#include <upipe-ts/uref_ts_flow.h>

#include <stdlib.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include <assert.h>

#include <bitstream/mpeg/ts.h>
#include <bitstream/mpeg/pes.h>

/** we only accept blocks */
#define EXPECTED_FLOW_DEF "block."
/** flow definition for PSI */
#define FLOW_DEF_PSI "block.mpegtspsi."
/** 2^33 (max resolution of PCR, PTS and DTS) */
#define POW2_33 UINT64_C(8589934592)
/** ratio between Upipe freq and MPEG PES freq */
#define SCALE_33 (UCLOCK_FREQ / 90000)
/** T-STD standard max retention time - 1 s */
#define T_STD_MAX_RETENTION UCLOCK_FREQ
/** PID for padding stream */
#define PADDING_PID 8191
/** TB buffer size in octets (T-STD model) */
#define TB_SIZE 512
/** define to get header verbosity */
#undef VERBOSE_HEADERS
/** define to get timing verbosity */
#undef VERBOSE_TIMING

/** @hidden */
static int upipe_ts_encaps_check(struct upipe *upipe, struct uref *flow_format);

/** @internal @This is the private context of a ts_encaps pipe. */
struct upipe_ts_encaps {
    /** refcount management structure */
    struct urefcount urefcount;

    /** ubuf manager */
    struct ubuf_mgr *ubuf_mgr;
    /** flow format packet */
    struct uref *flow_format;
    /** ubuf manager request */
    struct urequest ubuf_mgr_request;

    /** pipe acting as output */
    struct upipe *output;
    /** output flow definition packet */
    struct uref *flow_def;
    /** output state */
    enum upipe_helper_output_state output_state;
    /** list of output requests */
    struct uchain request_list;

    /** current uref being worked on */
    struct uref *uref;
    /** cr_sys of the current uref */
    uint64_t uref_cr_sys;
    /** dts_sys of the current uref */
    uint64_t uref_dts_sys;
    /** size of the current uref */
    size_t uref_size;
    /** true if the uref is ready to be muxed */
    bool uref_ready;
    /** temporary uref storage */
    struct uchain urefs;
    /** nb urefs in storage */
    unsigned int nb_urefs;
    /** max urefs in storage */
    unsigned int max_urefs;
    /** list of blockers (used during udeal) */
    struct uchain blockers;
    /** size of the current access unit */
    size_t au_size;

    /** PID */
    uint16_t pid;
    /** octetrate */
    uint64_t octetrate;
    /** T-STD TB size */
    size_t tb_size;
    /** T-STD TB rate */
    uint64_t tb_rate;
    /** T-STD max retention time */
    uint64_t max_delay;
    /** true if we chop PSI sections */
    bool psi;

    /** PCR interval (or 0) */
    uint64_t pcr_interval;

    /** PES stream ID */
    uint8_t pes_id;
    /** minimum PES header size */
    uint8_t pes_header_size;
    /** minimum PES duration */
    uint64_t pes_min_duration;
    /** PES alignment */
    bool pes_alignment;

    /** a padding packet for PSI streams */
    struct ubuf *padding;
    /** last continuity counter for this PID */
    uint8_t last_cc;
    /** last time prepare was called */
    uint64_t last_splice;
    /** available buffer space in TB (octets) */
    size_t tb_buffer;
    /** muxing date of the last PCR */
    uint64_t last_pcr;
    /** cr_prog of the last cr_sys/cr_prog reference */
    uint64_t sys_prog_last_cr_prog;
    /** cr_sys of the last cr_sys/cr_prog reference */
    uint64_t sys_prog_last_cr_sys;
    /** drift between cr_sys and cr_prog */
    struct urational sys_prog_drift_rate;
    /** offset between cr_prog and coded value */
    int64_t cr_prog_offset;
    /** true if end of stream was received */
    bool eos;

    /** true if update_ready needs to be called */
    bool need_ready;
    /** true if update_status needs to be called */
    bool need_status;
    /** last cr_sys sent by status */
    uint64_t last_cr_sys;
    /** last dts_sys sent by status */
    uint64_t last_dts_sys;
    /** last pcr_sys sent by status */
    uint64_t last_pcr_sys;
    /** last ready flag sent by status */
    bool last_ready;

    /** public upipe structure */
    struct upipe upipe;
};

UPIPE_HELPER_UPIPE(upipe_ts_encaps, upipe, UPIPE_TS_ENCAPS_SIGNATURE)
UPIPE_HELPER_UREFCOUNT(upipe_ts_encaps, urefcount, upipe_ts_encaps_free)
UPIPE_HELPER_VOID(upipe_ts_encaps)
UPIPE_HELPER_OUTPUT(upipe_ts_encaps, output, flow_def, output_state, request_list)
UPIPE_HELPER_UBUF_MGR(upipe_ts_encaps, ubuf_mgr, flow_format, ubuf_mgr_request,
                      upipe_ts_encaps_check,
                      upipe_ts_encaps_register_output_request,
                      upipe_ts_encaps_unregister_output_request)
UPIPE_HELPER_INPUT(upipe_ts_encaps, urefs, nb_urefs, max_urefs, blockers, NULL)

/** @internal @This allocates a ts_encaps pipe.
 *
 * @param mgr common management structure
 * @param uprobe structure used to raise events
 * @param signature signature of the pipe allocator
 * @param args optional arguments
 * @return pointer to upipe or NULL in case of allocation error
 */
static struct upipe *upipe_ts_encaps_alloc(struct upipe_mgr *mgr,
                                           struct uprobe *uprobe,
                                           uint32_t signature, va_list args)
{
    struct upipe *upipe = upipe_ts_encaps_alloc_void(mgr, uprobe, signature,
                                                     args);
    if (unlikely(upipe == NULL))
        return NULL;

    struct upipe_ts_encaps *upipe_ts_encaps = upipe_ts_encaps_from_upipe(upipe);
    upipe_ts_encaps_init_urefcount(upipe);
    upipe_ts_encaps_init_output(upipe);
    upipe_ts_encaps_init_ubuf_mgr(upipe);
    upipe_ts_encaps_init_input(upipe);
    upipe_ts_encaps->uref = NULL;
    upipe_ts_encaps->uref_size = 0;
    upipe_ts_encaps->uref_cr_sys = UINT64_MAX;
    upipe_ts_encaps->uref_dts_sys = UINT64_MAX;
    upipe_ts_encaps->uref_ready = false;
    upipe_ts_encaps->au_size = 0;
    upipe_ts_encaps->pid = 8192;
    upipe_ts_encaps->octetrate = 0;
    upipe_ts_encaps->tb_size = TB_SIZE;
    upipe_ts_encaps->tb_rate = 0;
    upipe_ts_encaps->max_delay = T_STD_MAX_RETENTION;
    upipe_ts_encaps->psi = false;
    upipe_ts_encaps->pcr_interval = 0;
    upipe_ts_encaps->pes_id = 0;
    upipe_ts_encaps->pes_header_size = 0;
    upipe_ts_encaps->pes_min_duration = 0;
    upipe_ts_encaps->pes_alignment = true;
    upipe_ts_encaps->padding = NULL;
    upipe_ts_encaps->last_cc = 0;
    upipe_ts_encaps->last_splice = 0;
    upipe_ts_encaps->last_pcr = 0;
    upipe_ts_encaps->tb_buffer = TB_SIZE;
    upipe_ts_encaps->sys_prog_last_cr_prog = UINT64_MAX;
    upipe_ts_encaps->sys_prog_last_cr_sys = UINT64_MAX;
    upipe_ts_encaps->sys_prog_drift_rate.num =
        upipe_ts_encaps->sys_prog_drift_rate.den = 1;
    upipe_ts_encaps->cr_prog_offset = 0;
    upipe_ts_encaps->eos = false;
    upipe_ts_encaps->need_ready = false;
    upipe_ts_encaps->need_status = false;
    upipe_ts_encaps->last_cr_sys = upipe_ts_encaps->last_dts_sys =
        upipe_ts_encaps->last_pcr_sys = UINT64_MAX;
    upipe_ts_encaps->last_ready = false;
    upipe_throw_ready(upipe);
    return upipe;
}

/** @This updates the status to the mux.
 *
 * @param upipe description structure of the pipe
 */
static void upipe_ts_encaps_update_status(struct upipe *upipe)
{
    struct upipe_ts_encaps *encaps = upipe_ts_encaps_from_upipe(upipe);
    uint64_t cr_sys = UINT64_MAX;
    uint64_t dts_sys = UINT64_MAX;
    uint64_t pcr_sys = UINT64_MAX;
    encaps->need_status = false;

    if (encaps->ubuf_mgr == NULL || !encaps->octetrate || !encaps->tb_rate)
        goto upipe_ts_encaps_update_status_done;
    if (encaps->uref == NULL)
        goto upipe_ts_encaps_update_status_pcr;

    cr_sys = encaps->uref_cr_sys -
            (uint64_t)encaps->uref_size * UCLOCK_FREQ / encaps->octetrate;
    if (encaps->uref_dts_sys != UINT64_MAX)
        dts_sys = encaps->uref_dts_sys -
            (uint64_t)encaps->uref_size * UCLOCK_FREQ / encaps->tb_rate;
    uint64_t tb_buffer = encaps->tb_buffer;

    if (encaps->last_splice < cr_sys) {
        tb_buffer += (cr_sys - encaps->last_splice) * encaps->tb_rate /
                             UCLOCK_FREQ;
        if (tb_buffer > encaps->tb_size)
            tb_buffer = encaps->tb_size;
    }

    if (unlikely(tb_buffer < TS_SIZE - TS_HEADER_SIZE)) {
        uint64_t tb_cr_sys = encaps->last_splice +
            (uint64_t)(TS_SIZE - TS_HEADER_SIZE - tb_buffer) * UCLOCK_FREQ /
            encaps->tb_rate;

        if (tb_cr_sys > cr_sys) {
            upipe_verbose(upipe, "delaying to avoid overflowing T-STD buffer");
            cr_sys = tb_cr_sys;
            if (unlikely(cr_sys > dts_sys))
                cr_sys = dts_sys;
        }
    } else {
        /* Move forward the buffer wrt. the TB buffer. */
        int64_t tb_diff = (uint64_t)(tb_buffer - (TS_SIZE - TS_HEADER_SIZE)) *
                          UCLOCK_FREQ / encaps->tb_rate;
        if (tb_diff > encaps->max_delay - (dts_sys - cr_sys))
            tb_diff = encaps->max_delay - (dts_sys - cr_sys);
        cr_sys -= tb_diff;
    }

upipe_ts_encaps_update_status_pcr:
    if (encaps->pcr_interval && encaps->sys_prog_last_cr_prog != UINT64_MAX) {
        if (encaps->last_pcr)
            pcr_sys = encaps->last_pcr + encaps->pcr_interval;
        else
            pcr_sys = cr_sys;
    }

upipe_ts_encaps_update_status_done:
    if (encaps->last_cr_sys == cr_sys && encaps->last_dts_sys == dts_sys &&
        encaps->last_pcr_sys == pcr_sys &&
        encaps->last_ready == encaps->uref_ready)
        return;

    encaps->last_cr_sys = cr_sys;
    encaps->last_dts_sys = dts_sys;
    encaps->last_pcr_sys = pcr_sys;
    encaps->last_ready = encaps->uref_ready;

#ifdef VERBOSE_TIMING
    upipe_verbose_va(upipe,
            "status cr_sys=%"PRIu64" dts_sys=%"PRIu64" pcr_sys=%"PRIu64" %s",
            cr_sys, dts_sys, pcr_sys,
            encaps->uref_ready ? "ready" : "not ready");
#endif
    upipe_throw(upipe, UPROBE_TS_ENCAPS_STATUS, UPIPE_TS_ENCAPS_SIGNATURE,
                cr_sys, dts_sys, pcr_sys, encaps->uref_ready ? 1 : 0);
}

/** @This updates the ready flag.
 *
 * @param upipe description structure of the pipe
 */
static void upipe_ts_encaps_update_ready(struct upipe *upipe)
{
    struct upipe_ts_encaps *encaps = upipe_ts_encaps_from_upipe(upipe);
    bool uref_ready = false;
    encaps->need_ready = false;

    if (encaps->uref == NULL || encaps->ubuf_mgr == NULL)
        goto upipe_ts_encaps_update_ready_done;

    if (encaps->eos || encaps->psi) {
        uref_ready = true;
        goto upipe_ts_encaps_update_ready_done;
    }

    struct uchain *uchain = &encaps->urefs;
    if (encaps->pes_min_duration) {
        uint64_t duration;
        if (!ubase_check(uref_clock_get_duration(encaps->uref, &duration))) {
            uref_ready = true;
            goto upipe_ts_encaps_update_ready_done;
        }

        while (duration < encaps->pes_min_duration) {
            if (ulist_is_last(&encaps->urefs, uchain))
                goto upipe_ts_encaps_update_ready_done;

            uchain = uchain->next;
            struct uref *uref = uref_from_uchain(uchain);
            const char *def;
            uint64_t uref_duration;
            if (ubase_check(uref_flow_get_def(uref, &def)) ||
                !ubase_check(uref_clock_get_duration(uref,
                                                     &uref_duration))) {
                uref_ready = true;
                goto upipe_ts_encaps_update_ready_done;
            }
            duration += uref_duration;
        }
    }

    if (encaps->pes_alignment) {
        uref_ready = true;
        goto upipe_ts_encaps_update_ready_done;
    }

    while (!ulist_is_last(&encaps->urefs, uchain)) {
        uchain = uchain->next;
        struct uref *uref = uref_from_uchain(uchain);
        const char *def;
        if (ubase_check(uref_flow_get_def(uref, &def)) ||
            ubase_check(uref_block_get_start(uref))) {
            uref_ready = true;
            goto upipe_ts_encaps_update_ready_done;
        }
    }

upipe_ts_encaps_update_ready_done:
    if (uref_ready != encaps->uref_ready) {
        encaps->uref_ready = uref_ready;
        encaps->need_status = true;
    }
}

/** @This checks if something has to be updated.
 *
 * @param upipe description structure of the pipe
 */
static void upipe_ts_encaps_check_status(struct upipe *upipe)
{
    struct upipe_ts_encaps *encaps = upipe_ts_encaps_from_upipe(upipe);
    if (encaps->need_ready)
        upipe_ts_encaps_update_ready(upipe);
    if (encaps->need_status)
        upipe_ts_encaps_update_status(upipe);
}

/** @This promotes a uref to the temporary buffer, checking for flow def
 * changes.
 *
 * @param upipe description structure of the pipe
 */
static void upipe_ts_encaps_promote_uref(struct upipe *upipe)
{
    struct upipe_ts_encaps *encaps = upipe_ts_encaps_from_upipe(upipe);
    encaps->need_status = true;

    for ( ; ; ) {
        struct uref *uref = upipe_ts_encaps_pop_input(upipe);
        if (uref == NULL) {
            encaps->need_ready = true;
            return;
        }
        upipe_ts_encaps_unblock_input(upipe);

        bool has_cr = ubase_check(uref_block_get_end(uref));
        const char *def;
        uint64_t cr_prog = 0, cr_sys;
        size_t uref_size;
        if (unlikely(ubase_check(uref_flow_get_def(uref, &def)))) {
            encaps->psi = !ubase_ncmp(def, "block.mpegts.mpegtspsi.");
            uref_flow_set_def(uref, "void.");
            uref_block_flow_get_octetrate(uref, &encaps->octetrate);
            uref_ts_flow_get_tb_rate(uref, &encaps->tb_rate);
            uint64_t pid = PADDING_PID;
            uref_ts_flow_get_pid(uref, &pid);
            encaps->pid = pid;
            encaps->max_delay = T_STD_MAX_RETENTION;
            uref_ts_flow_get_max_delay(uref, &encaps->max_delay);
            if (ubase_ncmp(def, FLOW_DEF_PSI)) {
                uref_ts_flow_get_pes_id(uref, &encaps->pes_id);
                encaps->pes_header_size = 0;
                uref_ts_flow_get_pes_header(uref, &encaps->pes_header_size);
                encaps->pes_min_duration = 0;
                uref_ts_flow_get_pes_min_duration(uref,
                                                  &encaps->pes_min_duration);
                encaps->pes_alignment =
                    ubase_check(uref_ts_flow_get_pes_alignment(uref));
            }

            upipe_ts_encaps_store_flow_def(upipe, uref);
            /* trigger set_flow_def */
            upipe_ts_encaps_output(upipe, NULL, NULL);
            encaps->need_ready = true;
            continue;

        } else if (unlikely(!ubase_check(uref_clock_get_cr_sys(uref,
                                                               &cr_sys)) ||
                            !ubase_check(uref_block_size(uref, &uref_size)) ||
                            (encaps->pcr_interval && has_cr &&
                             !ubase_check(uref_clock_get_cr_prog(uref,
                                                                 &cr_prog))))) {
            upipe_warn(upipe, "dropping non-dated packet (internal error)");
            uref_free(uref);
            encaps->need_ready = true;
            continue;

        }

        encaps->uref = uref;
        encaps->uref_cr_sys = cr_sys;
        assert(cr_sys);
        encaps->uref_dts_sys = UINT64_MAX;
        uref_clock_get_dts_sys(uref, &encaps->uref_dts_sys);
        encaps->uref_size = uref_size;
        if (!encaps->pcr_interval)
            encaps->sys_prog_last_cr_prog = UINT64_MAX;
        else if (has_cr) {
            encaps->sys_prog_last_cr_prog = cr_prog;
            encaps->sys_prog_last_cr_sys = cr_sys;
            encaps->sys_prog_drift_rate.num =
                encaps->sys_prog_drift_rate.den = 1;
            uref_clock_get_rate(uref, &encaps->sys_prog_drift_rate);
            assert(encaps->sys_prog_drift_rate.num);
        }
        if (ubase_check(uref_block_get_start(uref)))
            encaps->need_ready = true;
        break;
    }
}

/** @This consumes a uref and promotes a new one. Protect this with
 * @ref upipe_use/@ref upipe_release.
 *
 * @param upipe description structure of the pipe
 */
static void upipe_ts_encaps_consume_uref(struct upipe *upipe)
{
    struct upipe_ts_encaps *upipe_ts_encaps = upipe_ts_encaps_from_upipe(upipe);
    uref_free(upipe_ts_encaps->uref);
    upipe_ts_encaps->uref = NULL;
    upipe_ts_encaps_promote_uref(upipe);
}

/** @internal @This receives and queues data.
 *
 * @param upipe description structure of the pipe
 * @param uref uref structure
 * @param upump_p reference to pump that generated the buffer
 */
static void upipe_ts_encaps_input(struct upipe *upipe, struct uref *uref,
                                  struct upump **upump_p)
{
    struct upipe_ts_encaps *encaps = upipe_ts_encaps_from_upipe(upipe);
    uref_block_set_start(uref);
    uref_block_set_end(uref);
    uref_attr_set_priv(uref, 0);

    const char *def;
    if (unlikely(ubase_check(uref_flow_get_def(uref, &def)))) {
        upipe_ts_encaps_hold_input(upipe, uref);
        if (encaps->uref == NULL)
            upipe_ts_encaps_promote_uref(upipe);
        return;
    }

    if (unlikely(encaps->max_urefs &&
                 encaps->nb_urefs >= encaps->max_urefs * 2)) {
        upipe_warn(upipe, "too many queued packets, dropping");
        uref_free(uref);
        /* Also drop the first packet because it may have an invalid date. */
        upipe_ts_encaps_consume_uref(upipe);
        encaps->au_size = 0;
        encaps->need_ready = encaps->need_status = true;

        /* Flush the rest of the access unit. */
        while (encaps->uref != NULL &&
               !ubase_check(uref_block_get_start(encaps->uref)))
            upipe_ts_encaps_consume_uref(upipe);

        upipe_ts_encaps_check_status(upipe);
        return;
    }

    uint64_t cr_sys, cr_prog;
    if (unlikely(!ubase_check(uref_clock_get_cr_sys(uref, &cr_sys)) ||
                 (encaps->pcr_interval &&
                  !ubase_check(uref_clock_get_cr_prog(uref, &cr_prog))))) {
        upipe_warn(upipe, "dropping non-dated packet");
        uref_free(uref);
        return;
    }

    if (cr_sys < encaps->last_splice)
        upipe_dbg_va(upipe, "late packet received (%"PRIu64" ms)",
                     (encaps->last_splice - cr_sys) * 1000 / UCLOCK_FREQ);

    size_t uref_size;
    if (unlikely(!ubase_check(uref_block_size(uref, &uref_size)) ||
                 !uref_size)) {
        upipe_warn(upipe, "dropping empty packet");
        uref_free(uref);
        return;
    }

    upipe_ts_encaps_hold_input(upipe, uref);
    upipe_ts_encaps_block_input(upipe, upump_p);
    if (encaps->uref == NULL)
        upipe_ts_encaps_promote_uref(upipe);
    else if (!encaps->uref_ready)
        encaps->need_ready = true;
    upipe_ts_encaps_check_status(upipe);
}

/** @internal @This checks if the input may start.
 *
 * @param upipe description structure of the pipe
 * @param flow_format amended flow format
 * @return an error code
 */
static int upipe_ts_encaps_check(struct upipe *upipe, struct uref *flow_format)
{
    struct upipe_ts_encaps *encaps = upipe_ts_encaps_from_upipe(upipe);
    if (flow_format != NULL)
        uref_free(flow_format);

    if (encaps->ubuf_mgr != NULL && encaps->padding == NULL) {
        struct ubuf *padding = ubuf_block_alloc(encaps->ubuf_mgr,
                                                TS_SIZE - TS_HEADER_SIZE);
        uint8_t *buffer;
        int size = -1;
        if (unlikely(padding == NULL ||
                     !ubase_check(ubuf_block_write(padding, 0,
                                                   &size, &buffer)))) {
            ubuf_free(padding);
            return UBASE_ERR_ALLOC;
        }
        memset(buffer, 0xff, size);
        ubuf_block_unmap(padding, 0);
        encaps->padding = padding;

        encaps->need_ready = encaps->need_status = true;
        upipe_ts_encaps_check_status(upipe);
    }

    return UBASE_ERR_NONE;
}

/** @internal @This sets the input flow definition.
 *
 * @param upipe description structure of the pipe
 * @param flow_def flow definition packet
 * @return an error code
 */
static int upipe_ts_encaps_set_flow_def(struct upipe *upipe,
                                        struct uref *flow_def)
{
    if (flow_def == NULL)
        return UBASE_ERR_INVALID;
    const char *def;
    uint64_t pid;
    uint64_t octetrate, tb_rate;
    UBASE_RETURN(uref_flow_get_def(flow_def, &def))
    if (ubase_ncmp(def, EXPECTED_FLOW_DEF) ||
        !ubase_check(uref_block_flow_get_octetrate(flow_def, &octetrate)) ||
        !octetrate ||
        !ubase_check(uref_ts_flow_get_tb_rate(flow_def, &tb_rate)) ||
        !ubase_check(uref_ts_flow_get_pid(flow_def, &pid)))
        return UBASE_ERR_INVALID;

    uint8_t pes_id;
    if (ubase_ncmp(def, FLOW_DEF_PSI)) {
        if (!ubase_check(uref_ts_flow_get_pes_id(flow_def, &pes_id)))
            return UBASE_ERR_INVALID;
    }

    struct uref *flow_def_dup;
    if (unlikely((flow_def_dup = uref_dup(flow_def)) == NULL)) {
        upipe_throw_fatal(upipe, UBASE_ERR_ALLOC);
        return UBASE_ERR_ALLOC;
    }

    if (ubase_ncmp(def, FLOW_DEF_PSI)) {
        if (unlikely(!ubase_check(uref_flow_set_def_va(flow_def_dup,
                            "block.mpegts.mpegtspes.%s",
                            def + strlen(EXPECTED_FLOW_DEF)))))
            upipe_throw_fatal(upipe, UBASE_ERR_ALLOC);
    } else {
        if (unlikely(!ubase_check(uref_flow_set_def_va(flow_def_dup,
                            "block.mpegts.%s",
                            def + strlen(EXPECTED_FLOW_DEF)))))
            upipe_throw_fatal(upipe, UBASE_ERR_ALLOC);
    }
    upipe_input(upipe, flow_def_dup, NULL);

    struct upipe_ts_encaps *upipe_ts_encaps = upipe_ts_encaps_from_upipe(upipe);
    if (unlikely(upipe_ts_encaps->ubuf_mgr == NULL)) {
        if (unlikely((flow_def_dup = uref_dup(flow_def)) == NULL)) {
            upipe_throw_fatal(upipe, UBASE_ERR_ALLOC);
            return UBASE_ERR_ALLOC;
        }
        upipe_ts_encaps_require_ubuf_mgr(upipe, flow_def_dup);
    }
    return UBASE_ERR_NONE;
}

/** @internal @This returns the currently configured PCR interval.
 *
 * @param upipe description structure of the pipe
 * @param pcr_interval_p filled in with the PCR interval
 * @return an error code
 */
static int upipe_ts_encaps_get_pcr_interval(struct upipe *upipe,
                                            uint64_t *pcr_interval_p)
{
    struct upipe_ts_encaps *upipe_ts_encaps = upipe_ts_encaps_from_upipe(upipe);
    assert(pcr_interval_p != NULL);
    *pcr_interval_p = upipe_ts_encaps->pcr_interval;
    return UBASE_ERR_NONE;
}

/** @internal @This sets the PCR interval. To cancel insertion of PCRs, set it
 * to 0.
 *
 * @param upipe description structure of the pipe
 * @param pcr_interval new PCR interval
 * @return an error code
 */
static int upipe_ts_encaps_set_pcr_interval(struct upipe *upipe,
                                            uint64_t pcr_interval)
{
    struct upipe_ts_encaps *upipe_ts_encaps = upipe_ts_encaps_from_upipe(upipe);
    upipe_ts_encaps->pcr_interval = pcr_interval;
    upipe_ts_encaps->need_status = true;
    upipe_ts_encaps_check_status(upipe);
    return UBASE_ERR_NONE;
}

/** @This sets the cr_prog of the next access unit.
 *
 * @param upipe description structure of the pipe
 * @param cr_prog cr_prog of the next access unit
 * @return an error code
 */
static int upipe_ts_encaps_set_cr_prog(struct upipe *upipe, uint64_t cr_prog)
{
    struct upipe_ts_encaps *encaps = upipe_ts_encaps_from_upipe(upipe);
    if (encaps->uref == NULL)
        return UBASE_ERR_INVALID;

    uint64_t uref_cr_prog;
    if (unlikely(!ubase_check(uref_clock_get_cr_prog(encaps->uref,
                                                     &uref_cr_prog)))) {
        upipe_warn(upipe, "non-dated packet");
        return UBASE_ERR_UNHANDLED;
    }
    uref_cr_prog -= (uint64_t)encaps->uref_size * UCLOCK_FREQ /
                    encaps->octetrate;
    uref_cr_prog -= (uint64_t)(encaps->tb_buffer - (TS_SIZE - TS_HEADER_SIZE)) *
                    UCLOCK_FREQ / encaps->tb_rate;
    encaps->cr_prog_offset = cr_prog - uref_cr_prog;
    return UBASE_ERR_NONE;
}

/** @This sets the size of the TB buffer.
 *
 * @param upipe description structure of the pipe
 * @param tb_size size of the TB buffer
 * @return an error code
 */
static int _upipe_ts_encaps_set_tb_size(struct upipe *upipe,
                                        unsigned int tb_size)
{
    struct upipe_ts_encaps *encaps = upipe_ts_encaps_from_upipe(upipe);
    encaps->tb_buffer += tb_size - encaps->tb_size;
    encaps->tb_size = tb_size;
    encaps->need_status = true;
    upipe_ts_encaps_check_status(upipe);
    return UBASE_ERR_NONE;
}

/** @internal @This returns the size of the next PES header.
 *
 * @param upipe description structure of the pipe
 * @param pts_prog value of the PTS field, in 27 MHz units
 * @param dts_prog value of the DTS field, in 27 MHz units
 * @return size of the next PES header
 */
static size_t upipe_ts_encaps_pes_header_size(struct upipe *upipe,
                                              uint64_t pts_prog,
                                              uint64_t dts_prog)
{
    struct upipe_ts_encaps *encaps = upipe_ts_encaps_from_upipe(upipe);
    size_t header_size;
    if (encaps->pes_id != PES_STREAM_ID_PRIVATE_2) {
        if (pts_prog != UINT64_MAX) {
            if (dts_prog != UINT64_MAX &&
                ((pts_prog / SCALE_33) % POW2_33) !=
                    ((dts_prog / SCALE_33) % POW2_33))
                header_size = PES_HEADER_SIZE_PTSDTS;
            else
                header_size = PES_HEADER_SIZE_PTS;
        } else
            header_size = PES_HEADER_SIZE_NOPTS;
    } else
        header_size = PES_HEADER_SIZE;
    if (header_size < encaps->pes_header_size)
        header_size = encaps->pes_header_size;
    return header_size;
}

/** @internal @This builds a PES header.
 *
 * @param upipe description structure of the pipe
 * @param payload_size size of the payload
 * @param alignment true if the data alignment flag must be set
 * @param pts_prog value of the PTS field, in 27 MHz units
 * @param dts_prog value of the DTS field, in 27 MHz units
 * @return allocated PES header
 */
static struct ubuf *upipe_ts_encaps_build_pes(struct upipe *upipe,
                                              size_t payload_size,
                                              bool alignment,
                                              uint64_t pts_prog,
                                              uint64_t dts_prog)
{
    struct upipe_ts_encaps *encaps = upipe_ts_encaps_from_upipe(upipe);
    size_t header_size = upipe_ts_encaps_pes_header_size(upipe,
                                                         pts_prog, dts_prog);

#ifdef VERBOSE_HEADERS
    upipe_verbose_va(upipe, "preparing PES header (size %zu)", header_size);
#endif
    struct ubuf *ubuf = ubuf_block_alloc(encaps->ubuf_mgr, header_size);
    uint8_t *buffer;
    int size = -1;
    if (unlikely(ubuf == NULL ||
                 !ubase_check(ubuf_block_write(ubuf, 0, &size, &buffer)))) {
        ubuf_free(ubuf);
        upipe_throw_fatal(upipe, UBASE_ERR_ALLOC);
        return NULL;
    }

    pes_init(buffer);
    pes_set_streamid(buffer, encaps->pes_id);
    size_t pes_length = payload_size + header_size - PES_HEADER_SIZE;
    if (pes_length > UINT16_MAX) {
        if (unlikely((encaps->pes_id & PES_STREAM_ID_VIDEO_MPEG) !=
                     PES_STREAM_ID_VIDEO_MPEG))
            upipe_warn(upipe, "PES length > 65535 for a non-video stream");
        pes_set_length(buffer, 0);
    } else
        pes_set_length(buffer, pes_length);

    if (encaps->pes_id != PES_STREAM_ID_PRIVATE_2) {
        pes_set_headerlength(buffer, header_size - PES_HEADER_SIZE_NOPTS);
        if (alignment)
            pes_set_dataalignment(buffer);
        if (pts_prog != UINT64_MAX) {
            pes_set_pts(buffer,
                    ((pts_prog + encaps->cr_prog_offset) / SCALE_33) % POW2_33);
            if (dts_prog != UINT64_MAX &&
                ((pts_prog / SCALE_33) % POW2_33) !=
                    ((dts_prog / SCALE_33) % POW2_33))
                pes_set_dts(buffer,
                            ((dts_prog + encaps->cr_prog_offset) / SCALE_33) %
                            POW2_33);
        }
    }

    ubuf_block_unmap(ubuf, 0);
    return ubuf;
}

/** @internal @This counts the number of PCRs we plan to insert for the next
 * access unit.
 *
 * @param upipe description structure of the pipe
 * @param nb_pcr_p filled with number of PCRs after the first packet
 * @return true if the first packet has a PCR
 */
static bool upipe_ts_encaps_count_pcr_au(struct upipe *upipe,
                                         size_t *nb_pcr_p)
{
    struct upipe_ts_encaps *encaps = upipe_ts_encaps_from_upipe(upipe);
    *nb_pcr_p = 0;
    if (!encaps->pcr_interval)
        return false;

    uint64_t last_pcr = encaps->last_pcr;
    struct uref *uref = encaps->uref;
    struct uchain *uchain = &encaps->urefs;
    for ( ; ; ) {
        uint64_t cr_sys;
        if (unlikely(!ubase_check(uref_clock_get_cr_sys(uref, &cr_sys))))
            break;

        while (last_pcr + encaps->pcr_interval <= cr_sys) {
            last_pcr += encaps->pcr_interval;
            (*nb_pcr_p)++;
        }

        if (ulist_is_last(&encaps->urefs, uchain))
            break;
        uchain = uchain->next;
        uref = uref_from_uchain(uchain);
        if (ubase_check(uref_block_get_start(uref)))
            break;
    }
    return encaps->last_splice == encaps->last_pcr;
}

/** @internal @This copies the last incomplete TS of an access unit to the
 * beginning of the next access unit.
 *
 * @param upipe description structure of the pipe
 * @param uref_au1 uref containing the first access unit
 * @param uchain place where to insert the overlapped uref
 * @param last_ts_size number of octets to transfer to access unit 2
 * @return an error code
 */
static int upipe_ts_encaps_overlap_au(struct upipe *upipe,
                                      struct uref *uref_au1,
                                      struct uchain *uchain,
                                      size_t last_ts_size)
{
    struct upipe_ts_encaps *encaps = upipe_ts_encaps_from_upipe(upipe);
    size_t size;
    UBASE_RETURN(uref_block_size(uref_au1, &size));
    struct uref *uref_overlap = uref_block_split(uref_au1, size - last_ts_size);
    UBASE_ALLOC_RETURN(uref_overlap);
    struct uref *uref_au2 = uref_from_uchain(uchain->next);
    ulist_insert(uchain, uchain->next, uref_to_uchain(uref_overlap));
    encaps->nb_urefs++;

    uint64_t dts_sys = UINT64_MAX;
    uref_clock_get_dts_sys(uref_au1, &dts_sys);
    uint64_t cr_sys;
    UBASE_RETURN(uref_clock_get_cr_sys(uref_au1, &cr_sys));
    uint64_t cr_prog = UINT64_MAX;
    uref_clock_get_cr_prog(uref_au1, &cr_prog);

    /* Adjust AU1's dts_sys and cr_sys */
    cr_sys -= (uint64_t)last_ts_size * UCLOCK_FREQ / encaps->octetrate;
    uref_clock_set_cr_sys(uref_au1, cr_sys);
    if (cr_prog != UINT64_MAX) {
        cr_prog -= (uint64_t)last_ts_size * UCLOCK_FREQ / encaps->octetrate;
        uref_clock_set_cr_prog(uref_au1, cr_prog);
    }

    if (dts_sys != UINT64_MAX) {
        dts_sys -= (uint64_t)last_ts_size * UCLOCK_FREQ / encaps->tb_rate;
        uref_clock_set_cr_dts_delay(uref_au1, dts_sys - cr_sys);
    }

    uint64_t dts_pts_delay = 0;
    uref_clock_get_dts_pts_delay(uref_au2, &dts_pts_delay);
    uint64_t dts_prog = UINT64_MAX;
    uref_clock_get_dts_prog(uref_au2, &dts_prog);

    /* Adjust overlap's dts_prog and pts_prog */
    uref_clock_delete_date_prog(uref_overlap);
    uref_clock_delete_dts_pts_delay(uref_overlap);
    if (dts_prog != UINT64_MAX) {
        uref_clock_set_dts_prog(uref_overlap, dts_prog);
        uref_clock_set_dts_pts_delay(uref_overlap, dts_pts_delay);
    }
    uref_block_set_start(uref_overlap);
    uref_block_delete_end(uref_overlap);
    uref_flow_delete_discontinuity(uref_overlap);
    uref_flow_delete_random(uref_overlap);

    /* AU2 is no longer the start of an access unit */
    uref_block_delete_start(uref_au2);
    return UBASE_ERR_NONE;
}

/** @internal @This prepares a new access unit for splicing.
 *
 * @param upipe description structure of the pipe
 * @return an error code
 */
static int upipe_ts_encaps_promote_au(struct upipe *upipe)
{
    struct upipe_ts_encaps *encaps = upipe_ts_encaps_from_upipe(upipe);
    if (encaps->psi) {
        /* Prepend pointer_field */
#ifdef VERBOSE_HEADERS
        upipe_verbose_va(upipe, "preparing PSI pointer_field");
#endif
        struct ubuf *ubuf = ubuf_block_alloc(encaps->ubuf_mgr, 1);
        uint8_t *buffer;
        int size = -1;
        if (unlikely(ubuf == NULL ||
                     !ubase_check(ubuf_block_write(ubuf, 0, &size, &buffer)))) {
            ubuf_free(ubuf);
            return UBASE_ERR_ALLOC;
        }
        assert(size == 1);
        buffer[0] = 0;
        ubuf_block_unmap(ubuf, 0);
        struct ubuf *section = uref_detach_ubuf(encaps->uref);
        uref_attach_ubuf(encaps->uref, ubuf);
        uref_attr_set_priv(encaps->uref, 1);
        if (unlikely(!ubase_check(uref_block_append(encaps->uref, section)))) {
            ubuf_free(section);
            return UBASE_ERR_ALLOC;
        }
        encaps->uref_size++;
        encaps->au_size = encaps->uref_size;
        return UBASE_ERR_NONE;
    }

    size_t au_size = encaps->uref_size;
    uint64_t duration = 0;
    struct uchain *uchain = &encaps->urefs;
    if (encaps->pes_min_duration)
        uref_clock_get_duration(encaps->uref, &duration);
    while (!ulist_is_last(&encaps->urefs, uchain)) {
        if (ubase_check(uref_block_get_start(uref_from_uchain(uchain->next))))
            break;
        uchain = uchain->next;
        struct uref *uref = uref_from_uchain(uchain);
        size_t uref_size;
        UBASE_RETURN(uref_block_size(uref, &uref_size));
        au_size += uref_size;
        if (encaps->pes_min_duration) {
            uint64_t uref_duration = 0;
            uref_clock_get_duration(uref, &uref_duration);
            duration += uref_duration;
        }
    }

#ifdef VERBOSE_HEADERS
    upipe_verbose_va(upipe, "promoting a%s access unit of size %zu",
            ubase_check(uref_flow_get_random(encaps->uref)) ? " random" : "n",
            au_size);
#endif

    uint64_t pts_prog = UINT64_MAX, dts_prog = UINT64_MAX;
    uref_clock_get_pts_prog(encaps->uref, &pts_prog);
    uref_clock_get_dts_prog(encaps->uref, &dts_prog);

    while (duration < encaps->pes_min_duration) {
        uint64_t uref_duration;
        size_t uref_size;
        const char *def;
        if (ulist_is_last(&encaps->urefs, uchain) ||
            ubase_check(uref_flow_get_def(uref_from_uchain(uchain->next), &def)) ||
            ubase_check(uref_flow_get_random(uref_from_uchain(uchain->next))) ||
            ubase_check(uref_flow_get_discontinuity(uref_from_uchain(uchain->next))) ||
            !ubase_check(uref_clock_get_duration(uref_from_uchain(uchain->next), &uref_duration)) ||
            !ubase_check(uref_block_size(uref_from_uchain(uchain->next), &uref_size)))
            break;

        uchain = uchain->next;
        struct uref *uref = uref_from_uchain(uchain);
        duration += uref_duration;
        au_size += uref_size;
        uref_block_delete_start(uref);
        upipe_verbose_va(upipe, "aggregating an access unit");
    }

    const char *def;
    if (!encaps->pes_alignment && !ulist_is_last(&encaps->urefs, uchain) &&
        !ubase_check(uref_flow_get_def(uref_from_uchain(uchain->next), &def)) &&
        !ubase_check(uref_flow_get_random(uref_from_uchain(uchain->next))) &&
        !ubase_check(uref_flow_get_discontinuity(uref_from_uchain(uchain->next)))) {
        size_t nb_pcr;
        bool first_has_pcr = upipe_ts_encaps_count_pcr_au(upipe, &nb_pcr);
        size_t pes_header_size = upipe_ts_encaps_pes_header_size(upipe,
                                    pts_prog, dts_prog);

        size_t next_ts_size = TS_SIZE - pes_header_size -
            (first_has_pcr ? TS_HEADER_SIZE_PCR :
             (ubase_check(uref_flow_get_random(encaps->uref)) ||
              ubase_check(uref_flow_get_discontinuity(encaps->uref))) ?
             TS_HEADER_SIZE_AF : TS_HEADER_SIZE);
        if (au_size > next_ts_size) {
            size_t last_ts_size = au_size;
            while (last_ts_size > next_ts_size) {
                last_ts_size -= next_ts_size;
                next_ts_size = TS_SIZE -
                               (nb_pcr ? TS_HEADER_SIZE_PCR : TS_HEADER_SIZE);
                if (nb_pcr)
                    nb_pcr--;
            }

            size_t uref_size = 0;
            if (&encaps->urefs == uchain)
                uref_block_size(encaps->uref, &uref_size);
            else
                uref_block_size(uref_from_uchain(uchain), &uref_size);
            if (last_ts_size && uref_size > last_ts_size && !nb_pcr) {
                upipe_verbose_va(upipe, "overlapping an access unit (%zu)",
                                 last_ts_size);
                if (&encaps->urefs == uchain) {
                    UBASE_RETURN(upipe_ts_encaps_overlap_au(upipe, encaps->uref,
                                uchain, last_ts_size));
                    uref_clock_get_cr_sys(encaps->uref, &encaps->uref_cr_sys);
                    uref_clock_get_dts_sys(encaps->uref, &encaps->uref_dts_sys);
                    encaps->uref_size -= last_ts_size;
                } else {
                    UBASE_RETURN(upipe_ts_encaps_overlap_au(upipe,
                                uref_from_uchain(uchain), uchain, last_ts_size));
                }
                au_size -= last_ts_size;
            }
        }
    }

    struct ubuf *ubuf = upipe_ts_encaps_build_pes(upipe, au_size,
            ubase_check(uref_block_get_end(encaps->uref)), pts_prog, dts_prog);
    size_t header_size = 0;
    ubuf_block_size(ubuf, &header_size);
    uref_attr_set_priv(encaps->uref, header_size);
    struct ubuf *section = uref_detach_ubuf(encaps->uref);
    uref_attach_ubuf(encaps->uref, ubuf);
    if (unlikely(!ubase_check(uref_block_append(encaps->uref, section)))) {
        ubuf_free(section);
        return UBASE_ERR_ALLOC;
    }
    encaps->uref_size += header_size;
    encaps->au_size = au_size + header_size;
    assert(encaps->uref_size <= encaps->au_size);
    return UBASE_ERR_NONE;
}

/** @internal @This builds a TS header.
 *
 * @param upipe description structure of the pipe
 * @param payload_size available size of the payload
 * @param start true if it's the first packet of the access unit
 * @param pcr_prog value of the PCR field, in 27 MHz units, or UINT64_MAX
 * @param random true if the packet is a random access point
 * @param discontinuity true if the packet must have the discontinuity flag
 * @return allocated TS header
 */
static struct ubuf *upipe_ts_encaps_build_ts(struct upipe *upipe,
                                             size_t payload_size, bool start,
                                             uint64_t pcr_prog, bool random,
                                             bool discontinuity)
{
    struct upipe_ts_encaps *encaps = upipe_ts_encaps_from_upipe(upipe);
    size_t header_size;
    if (unlikely(pcr_prog != UINT64_MAX))
        header_size = TS_HEADER_SIZE_PCR;
    else if (unlikely(discontinuity || random))
        header_size = TS_HEADER_SIZE_AF;
    else
        header_size = TS_HEADER_SIZE;

    if (!encaps->psi && payload_size < TS_SIZE - header_size)
        header_size = TS_SIZE - payload_size;

#ifdef VERBOSE_HEADERS
    upipe_verbose_va(upipe, "preparing TS header (size %zu%s%s%s%s)",
            header_size, start ? ", start" : "", random ? ", random" : "",
            discontinuity ? ", disc" : "",
            pcr_prog != UINT64_MAX ? ", pcr" : "");
#endif
    struct ubuf *ubuf = ubuf_block_alloc(encaps->ubuf_mgr, header_size);
    uint8_t *buffer;
    int size = -1;
    if (unlikely(ubuf == NULL ||
                 !ubase_check(ubuf_block_write(ubuf, 0, &size, &buffer)))) {
        ubuf_free(ubuf);
        return NULL;
    }
    assert(size == header_size);

    ts_init(buffer);
    ts_set_pid(buffer, encaps->pid);
    if (payload_size) {
        encaps->last_cc++;
        encaps->last_cc &= 0xf;
        ts_set_payload(buffer);
    }
    ts_set_cc(buffer, encaps->last_cc);
    if (start)
        ts_set_unitstart(buffer);

    if (header_size > TS_HEADER_SIZE) {
        ts_set_adaptation(buffer, header_size - TS_HEADER_SIZE - 1);
        if (discontinuity)
            tsaf_set_discontinuity(buffer);
        if (random)
            tsaf_set_randomaccess(buffer);
        if (pcr_prog != UINT64_MAX) {
            pcr_prog += encaps->cr_prog_offset;
            tsaf_set_pcr(buffer, (pcr_prog / SCALE_33) % POW2_33);
            tsaf_set_pcrext(buffer, pcr_prog % SCALE_33);
        }
    }

    ubuf_block_unmap(ubuf, 0);
    return ubuf;
}

/** @internal @This splices the input uref and appends to the given ubuf to
 * build a complete TS packet. For PSI sections it may also append padding.
 *
 * @param upipe description structure of the pipe
 * @param ubuf_p appended with the payload of the packet
 * @param dts_sys_p filled in with the DTS, or UINT64_MAX
 * @return an error code
 */
static int upipe_ts_encaps_complete(struct upipe *upipe, struct ubuf **ubuf_p,
                                    uint64_t *dts_sys_p)
{
    struct upipe_ts_encaps *encaps = upipe_ts_encaps_from_upipe(upipe);
    encaps->need_status = true;
    *dts_sys_p = UINT64_MAX;

    size_t ubuf_size;
    UBASE_RETURN(ubuf_block_size(*ubuf_p, &ubuf_size));
    assert(ubuf_size < TS_SIZE);

    for ( ; ; ) {
        size_t uref_size = encaps->uref_size;
        uint64_t header_size = 0;
        uref_attr_get_priv(encaps->uref, &header_size);

        uint64_t dts_sys = UINT64_MAX;
        uref_clock_get_dts_sys(encaps->uref, &dts_sys);

        if (dts_sys != UINT64_MAX && *dts_sys_p == UINT64_MAX)
            *dts_sys_p = dts_sys -
                (uint64_t)(uref_size - header_size) * UCLOCK_FREQ /
                encaps->tb_rate;

        struct ubuf *payload = uref_detach_ubuf(encaps->uref);
        if (uref_size >= TS_SIZE - ubuf_size) {
            size_t payload_size = TS_SIZE - ubuf_size;
            assert(payload_size);
            uref_attach_ubuf(encaps->uref,
                             ubuf_block_split(payload, payload_size));
            encaps->uref_size -= payload_size;
            encaps->au_size -= payload_size;
            if (payload_size >= header_size)
                uref_attr_set_priv(encaps->uref, 0);
            else
                uref_attr_set_priv(encaps->uref, header_size - payload_size);
            encaps->tb_buffer -= payload_size;
        } else {
            encaps->tb_buffer -= uref_size;
            encaps->au_size -= uref_size;
        }

        if (unlikely(payload == NULL ||
                     !ubase_check(ubuf_block_append(*ubuf_p, payload)))) {
            ubuf_free(payload);
            ubuf_free(*ubuf_p);
            return UBASE_ERR_ALLOC;
        }

        if (uref_size <= TS_SIZE - ubuf_size)
            upipe_ts_encaps_consume_uref(upipe);

        if (uref_size >= TS_SIZE - ubuf_size) {
            ubuf_size = TS_SIZE;
            break;
        }

        ubuf_size += uref_size;
        if (encaps->uref == NULL ||
            ubase_check(uref_block_get_start(encaps->uref))) {
            assert(!encaps->au_size);
            break;
        }
    }

    if (ubuf_size < TS_SIZE) {
        /* With PSI, pad with 0xff */
        struct ubuf *padding = ubuf_dup(encaps->padding);
        if (unlikely(padding == NULL ||
                     !ubase_check(ubuf_block_resize(padding, 0,
                             TS_SIZE - ubuf_size)) ||
                     !ubase_check(ubuf_block_append(*ubuf_p, padding)))) {
            ubuf_free(padding);
            ubuf_free(*ubuf_p);
            return UBASE_ERR_ALLOC;
        }
    }

    return UBASE_ERR_NONE;
}

/** @This returns a ubuf containing a TS packet, and the dts_sys of the packet.
 *
 * @param upipe description structure of the pipe
 * @param cr_sys_min date at which the packet will be muxed
 * @param cr_sys_max maximum date allowed for muxing
 * @param ubuf_p filled in with a pointer to the ubuf (may be NULL)
 * @param dts_sys_p filled in with the dts_sys, or UINT64_MAX
 * @return an error code
 */
static int _upipe_ts_encaps_splice(struct upipe *upipe, uint64_t cr_sys_min,
        uint64_t cr_sys_max, struct ubuf **ubuf_p, uint64_t *dts_sys_p)
{
    struct upipe_ts_encaps *encaps = upipe_ts_encaps_from_upipe(upipe);
    if (encaps->ubuf_mgr == NULL)
        return UBASE_ERR_INVALID;

    if (encaps->last_splice < cr_sys_min) {
        encaps->tb_buffer += (cr_sys_min - encaps->last_splice) *
                             encaps->tb_rate / UCLOCK_FREQ;
        if (encaps->tb_buffer > encaps->tb_size)
            encaps->tb_buffer = encaps->tb_size;
    }
    encaps->last_splice = cr_sys_min;

    if (ubuf_p == NULL) {
        /* Flush until cr_sys_min */
        while (encaps->uref != NULL) {
            if (encaps->uref_dts_sys != UINT64_MAX) {
                uint64_t dts_sys = encaps->uref_dts_sys -
                    (uint64_t)encaps->uref_size * UCLOCK_FREQ / encaps->tb_rate;
                if (dts_sys >= cr_sys_min)
                    break;

                upipe_warn_va(upipe, "dropping late packet (%"PRIu64" ms)",
                              (cr_sys_min - dts_sys) * 1000 / UCLOCK_FREQ);
                upipe_ts_encaps_consume_uref(upipe);
                encaps->au_size = 0;
                encaps->need_ready = encaps->need_status = true;

                /* Flush the rest of the access unit. */
                while (encaps->uref != NULL &&
                       !ubase_check(uref_block_get_start(encaps->uref)))
                    upipe_ts_encaps_consume_uref(upipe);
            }
        }
        upipe_ts_encaps_check_status(upipe);
        return UBASE_ERR_NONE;
    }

    uint64_t pcr_prog = UINT64_MAX;
    if (encaps->pcr_interval && encaps->sys_prog_last_cr_prog != UINT64_MAX &&
        encaps->last_pcr + encaps->pcr_interval <= encaps->last_splice) {
        pcr_prog = (int64_t)encaps->sys_prog_last_cr_prog +
            ((int64_t)encaps->last_splice -
             (int64_t)encaps->sys_prog_last_cr_sys) *
            (int64_t)encaps->sys_prog_drift_rate.den /
            (int64_t)encaps->sys_prog_drift_rate.num;
        encaps->last_pcr = encaps->last_splice;
    }

    if (encaps->uref == NULL ||
        (encaps->last_cr_sys > cr_sys_min &&
         encaps->last_dts_sys > cr_sys_max)) {
        if (unlikely(pcr_prog == UINT64_MAX))
            upipe_dbg(upipe, "adding unnecessary padding (internal error)");

        *ubuf_p = upipe_ts_encaps_build_ts(upipe, 0, false, pcr_prog, false,
                                           false);
        *dts_sys_p = pcr_prog != UINT64_MAX ? cr_sys_min : UINT64_MAX;
        encaps->need_status = true;
        upipe_ts_encaps_check_status(upipe);
        return UBASE_ERR_NONE;
    }

    bool start = ubase_check(uref_block_get_start(encaps->uref));
    if (start) {
        UBASE_RETURN(upipe_ts_encaps_promote_au(upipe));
    }
    assert(encaps->uref_size);
    assert(encaps->au_size);

    *ubuf_p = upipe_ts_encaps_build_ts(upipe, encaps->au_size, start, pcr_prog,
            ubase_check(uref_flow_get_random(encaps->uref)),
            ubase_check(uref_flow_get_discontinuity(encaps->uref)));
    UBASE_ALLOC_RETURN(*ubuf_p);
    uref_block_delete_start(encaps->uref);
    uref_flow_delete_random(encaps->uref);
    uref_flow_delete_discontinuity(encaps->uref);

    UBASE_RETURN(upipe_ts_encaps_complete(upipe, ubuf_p, dts_sys_p));
    if (pcr_prog != UINT64_MAX)
        *dts_sys_p = encaps->last_splice;

    upipe_ts_encaps_check_status(upipe);
    return UBASE_ERR_NONE;
}

/** @internal @This processes control commands on a ts encaps pipe.
 *
 * @param upipe description structure of the pipe
 * @param command type of command to process
 * @param args arguments of the command
 * @return an error code
 */
static int upipe_ts_encaps_control(struct upipe *upipe,
                                   int command, va_list args)
{
    switch (command) {
        case UPIPE_REGISTER_REQUEST: {
            struct urequest *request = va_arg(args, struct urequest *);
            if (request->type == UREQUEST_UREF_MGR)
                return upipe_throw_provide_request(upipe, request);
            return upipe_ts_encaps_alloc_output_proxy(upipe, request);
        }
        case UPIPE_UNREGISTER_REQUEST: {
            struct urequest *request = va_arg(args, struct urequest *);
            if (request->type == UREQUEST_UREF_MGR)
                return UBASE_ERR_NONE;
            return upipe_ts_encaps_free_output_proxy(upipe, request);
        }
        case UPIPE_SET_FLOW_DEF: {
            struct uref *flow_def = va_arg(args, struct uref *);
            return upipe_ts_encaps_set_flow_def(upipe, flow_def);
        }
        case UPIPE_GET_FLOW_DEF:
        case UPIPE_GET_OUTPUT:
        case UPIPE_SET_OUTPUT:
            return upipe_ts_encaps_control_output(upipe, command, args);
        case UPIPE_GET_MAX_LENGTH: {
            unsigned int *p = va_arg(args, unsigned int *);
            return upipe_ts_encaps_get_max_length(upipe, p);
        }
        case UPIPE_SET_MAX_LENGTH: {
            unsigned int max_length = va_arg(args, unsigned int);
            return upipe_ts_encaps_set_max_length(upipe, max_length);
        }
        case UPIPE_FLUSH:
            return upipe_ts_encaps_flush_input(upipe);

        case UPIPE_TS_MUX_GET_CC: {
            UBASE_SIGNATURE_CHECK(args, UPIPE_TS_MUX_SIGNATURE)
            struct upipe_ts_encaps *encaps = upipe_ts_encaps_from_upipe(upipe);
            unsigned int *cc_p = va_arg(args, unsigned int *);
            assert(cc_p != NULL);
            *cc_p = encaps->last_cc;
            return UBASE_ERR_NONE;
        }
        case UPIPE_TS_MUX_SET_CC: {
            UBASE_SIGNATURE_CHECK(args, UPIPE_TS_MUX_SIGNATURE)
            struct upipe_ts_encaps *encaps = upipe_ts_encaps_from_upipe(upipe);
            encaps->last_cc = va_arg(args, unsigned int);
            return UBASE_ERR_NONE;
        }
        case UPIPE_TS_MUX_GET_PCR_INTERVAL: {
            UBASE_SIGNATURE_CHECK(args, UPIPE_TS_MUX_SIGNATURE)
            uint64_t *pcr_interval_p = va_arg(args, uint64_t *);
            return upipe_ts_encaps_get_pcr_interval(upipe, pcr_interval_p);
        }
        case UPIPE_TS_MUX_SET_PCR_INTERVAL: {
            UBASE_SIGNATURE_CHECK(args, UPIPE_TS_MUX_SIGNATURE)
            uint64_t pcr_interval = va_arg(args, uint64_t);
            return upipe_ts_encaps_set_pcr_interval(upipe, pcr_interval);
        }
        case UPIPE_TS_MUX_SET_CR_PROG: {
            UBASE_SIGNATURE_CHECK(args, UPIPE_TS_MUX_SIGNATURE)
            uint64_t cr_prog = va_arg(args, uint64_t);
            return upipe_ts_encaps_set_cr_prog(upipe, cr_prog);
        }
        case UPIPE_TS_ENCAPS_SET_TB_SIZE: {
            UBASE_SIGNATURE_CHECK(args, UPIPE_TS_ENCAPS_SIGNATURE)
            unsigned int tb_size = va_arg(args, unsigned int);
            return _upipe_ts_encaps_set_tb_size(upipe, tb_size);
        }
        case UPIPE_TS_ENCAPS_SPLICE: {
            UBASE_SIGNATURE_CHECK(args, UPIPE_TS_ENCAPS_SIGNATURE)
            uint64_t cr_sys_min = va_arg(args, uint64_t);
            uint64_t cr_sys_max = va_arg(args, uint64_t);
            struct ubuf **ubuf_p = va_arg(args, struct ubuf **);
            uint64_t *dts_sys_p = va_arg(args, uint64_t *);
            return _upipe_ts_encaps_splice(upipe, cr_sys_min, cr_sys_max,
                                           ubuf_p, dts_sys_p);
        }
        case UPIPE_TS_ENCAPS_EOS: {
            UBASE_SIGNATURE_CHECK(args, UPIPE_TS_ENCAPS_SIGNATURE)
            struct upipe_ts_encaps *encaps = upipe_ts_encaps_from_upipe(upipe);
            encaps->eos = true;
            encaps->need_ready = true;
            upipe_ts_encaps_check_status(upipe);
            return UBASE_ERR_NONE;
        }
        default:
            return UBASE_ERR_UNHANDLED;
    }
}

/** @This frees a upipe.
 *
 * @param upipe description structure of the pipe
 */
static void upipe_ts_encaps_free(struct upipe *upipe)
{
    struct upipe_ts_encaps *upipe_ts_encaps = upipe_ts_encaps_from_upipe(upipe);
    upipe_throw(upipe, UPROBE_TS_MUX_LAST_CC, UPIPE_TS_MUX_SIGNATURE,
                (unsigned int)upipe_ts_encaps->last_cc);
    upipe_throw_dead(upipe);

    uref_free(upipe_ts_encaps->uref);
    ubuf_free(upipe_ts_encaps->padding);
    upipe_ts_encaps_clean_input(upipe);
    upipe_ts_encaps_clean_output(upipe);
    upipe_ts_encaps_clean_ubuf_mgr(upipe);
    upipe_ts_encaps_clean_urefcount(upipe);
    upipe_ts_encaps_free_void(upipe);
}

/** @This returns a description string for local commands.
 *
 * @param cmd control command
 * @return description string
 */
static const char *upipe_ts_encaps_command_str(int cmd)
{
    if (cmd < UPIPE_TS_MUX_ENCAPS)
        return upipe_ts_mux_command_str(cmd);

    switch (cmd) {
        UBASE_CASE_TO_STR(UPIPE_TS_ENCAPS_SET_TB_SIZE);
        UBASE_CASE_TO_STR(UPIPE_TS_ENCAPS_SPLICE);
        UBASE_CASE_TO_STR(UPIPE_TS_ENCAPS_EOS);
        default: break;
    }
    return NULL;
}

/** @This returns a description string for local events.
 *
 * @param event event
 * @return description string
 */
static const char *upipe_ts_encaps_event_str(int event)
{
    if (event < UPROBE_TS_MUX_ENCAPS)
        return upipe_ts_mux_event_str(event);

    switch (event) {
        UBASE_CASE_TO_STR(UPROBE_TS_ENCAPS_STATUS);
        default: break;
    }
    return NULL;
}

/** module manager static descriptor */
static struct upipe_mgr upipe_ts_encaps_mgr = {
    .refcount = NULL,
    .signature = UPIPE_TS_ENCAPS_SIGNATURE,

    .upipe_command_str = upipe_ts_encaps_command_str,
    .upipe_event_str = upipe_ts_encaps_event_str,

    .upipe_alloc = upipe_ts_encaps_alloc,
    .upipe_input = upipe_ts_encaps_input,
    .upipe_control = upipe_ts_encaps_control,

    .upipe_mgr_control = NULL
};

/** @This returns the management structure for all ts_encaps pipes.
 *
 * @return pointer to manager
 */
struct upipe_mgr *upipe_ts_encaps_mgr_alloc(void)
{
    return &upipe_ts_encaps_mgr;
}
