/*
 * Copyright (C) 2012-2013 OpenHeadend S.A.R.L.
 *
 * Authors: Christophe Massiot
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston MA 02110-1301, USA.
 */

/** @file
 * @short Upipe module splitting PIDs of a transport stream
 */

#include <upipe/ubase.h>
#include <upipe/urefcount.h>
#include <upipe/ulist.h>
#include <upipe/uprobe.h>
#include <upipe/uref.h>
#include <upipe/uref_block.h>
#include <upipe/ubuf.h>
#include <upipe/upipe.h>
#include <upipe/upipe_helper_upipe.h>
#include <upipe/upipe_helper_output.h>
#include <upipe-ts/uref_ts_flow.h>
#include <upipe-ts/upipe_ts_split.h>

#include <stdlib.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include <assert.h>

#include <bitstream/mpeg/ts.h>

/** we only accept blocks containing exactly one TS packet */
#define EXPECTED_FLOW_DEF "block.mpegts."
/** maximum number of PIDs */
#define MAX_PIDS 8192

/** @internal @This keeps internal information about a PID. */
struct upipe_ts_split_pid {
    /** outputs specific to that PID */
    struct ulist outputs;
    /** true if we asked for this PID */
    bool set;
};

/** @internal @This is the private context of a ts_split pipe. */
struct upipe_ts_split {
    /** true if we received a compatible flow definition */
    bool flow_def_ok;

    /** PIDs array */
    struct upipe_ts_split_pid pids[MAX_PIDS];

    /** manager to create outputs */
    struct upipe_mgr output_mgr;

    /** refcount management structure */
    urefcount refcount;
    /** public upipe structure */
    struct upipe upipe;
};

UPIPE_HELPER_UPIPE(upipe_ts_split, upipe)

/** @internal @This returns the public output_mgr structure.
 *
 * @param upipe_ts_split pointer to the private upipe_ts_split structure
 * @return pointer to the public output_mgr structure
 */
static inline struct upipe_mgr *
    upipe_ts_split_to_output_mgr(struct upipe_ts_split *s)
{
    return &s->output_mgr;
}

/** @internal @This returns the private upipe_ts_split structure.
 *
 * @param output_mgr public output_mgr structure of the pipe
 * @return pointer to the private upipe_ts_split structure
 */
static inline struct upipe_ts_split *
    upipe_ts_split_from_output_mgr(struct upipe_mgr *output_mgr)
{
    return container_of(output_mgr, struct upipe_ts_split, output_mgr);
}

/** @internal @This is the private context of an output of a ts_split pipe. */
struct upipe_ts_split_output {
    /** structure for double-linked lists */
    struct uchain uchain;

    /** pipe acting as output */
    struct upipe *output;
    /** flow definition packet on this output */
    struct uref *flow_def;
    /** true if the flow definition has already been sent */
    bool flow_def_sent;

    /** refcount management structure */
    urefcount refcount;
    /** public upipe structure */
    struct upipe upipe;
};

UPIPE_HELPER_UPIPE(upipe_ts_split_output, upipe)
UPIPE_HELPER_OUTPUT(upipe_ts_split_output, output, flow_def, flow_def_sent)

/** @hidden */
static void upipe_ts_split_pid_set(struct upipe *upipe, uint16_t pid,
                                   struct upipe_ts_split_output *output);
/** @hidden */
static void upipe_ts_split_pid_unset(struct upipe *upipe, uint16_t pid,
                                     struct upipe_ts_split_output *output);

/** @This returns the high-level upipe_ts_split_output structure.
 *
 * @param uchain pointer to the uchain structure wrapped into the
 * upipe_ts_split_output
 * @return pointer to the upipe_ts_split_output structure
 */
static inline struct upipe_ts_split_output *
    upipe_ts_split_output_from_uchain(struct uchain *uchain)
{
    return container_of(uchain, struct upipe_ts_split_output, uchain);
}

/** @This returns the uchain structure used for FIFO, LIFO and lists.
 *
 * @param upipe_ts_split_output upipe_ts_split_output structure
 * @return pointer to the uchain structure
 */
static inline struct uchain *
    upipe_ts_split_output_to_uchain(struct upipe_ts_split_output *upipe_ts_split_output)
{
    return &upipe_ts_split_output->uchain;
}

/** @internal @This allocates an output subpipe of a ts_split pipe.
 *
 * @param mgr common management structure
 * @param uprobe structure used to raise events
 * @return pointer to upipe or NULL in case of allocation error
 */
static struct upipe *upipe_ts_split_output_alloc(struct upipe_mgr *mgr,
                                                 struct uprobe *uprobe)
{
    struct upipe_ts_split_output *upipe_ts_split_output =
        malloc(sizeof(struct upipe_ts_split_output));
    if (unlikely(upipe_ts_split_output == NULL))
        return NULL;
    struct upipe *upipe = upipe_ts_split_output_to_upipe(upipe_ts_split_output);
    upipe_init(upipe, mgr, uprobe);
    uchain_init(&upipe_ts_split_output->uchain);
    upipe_ts_split_output_init_output(upipe);
    urefcount_init(&upipe_ts_split_output->refcount);
    upipe_throw_ready(upipe);
    return upipe;
}

/** @internal @This sets the flow definition on an output.
 *
 * The attribute t.pid must be set on the flow definition packet.
 *
 * @param upipe description structure of the pipe
 * @param flow_def flow definition packet
 * @return false in case of error
 */
static bool upipe_ts_split_output_set_flow_def(struct upipe *upipe,
                                               struct uref *flow_def)
{
    struct upipe_ts_split_output *upipe_ts_split_output =
        upipe_ts_split_output_from_upipe(upipe);
    if (upipe_ts_split_output->flow_def != NULL) {
        uint64_t pid;
        if (uref_ts_flow_get_pid(upipe_ts_split_output->flow_def, &pid))
            upipe_ts_split_pid_unset(upipe, pid, upipe_ts_split_output);
        upipe_ts_split_output_store_flow_def(upipe, NULL);
    }

    uint64_t pid;
    if (unlikely(!uref_ts_flow_get_pid(flow_def, &pid) || pid >= MAX_PIDS))
        return false;

    struct uref *uref = uref_dup(flow_def);
    if (unlikely(uref == NULL)) {
        upipe_throw_aerror(upipe);
        return false;
    }
    upipe_ts_split_output_store_flow_def(upipe, uref);
    struct upipe_ts_split *upipe_ts_split =
        upipe_ts_split_from_output_mgr(upipe->mgr);
    upipe_ts_split_pid_set(upipe_ts_split_to_upipe(upipe_ts_split), pid,
                           upipe_ts_split_output);
    return true;
}

/** @internal @This processes control commands on an output subpipe of a
 * ts_split pipe.
 *
 * @param upipe description structure of the pipe
 * @param command type of command to process
 * @param args arguments of the command
 * @return false in case of error
 */
static bool upipe_ts_split_output_control(struct upipe *upipe,
                                        enum upipe_command command,
                                        va_list args)
{
    switch (command) {
        case UPIPE_GET_OUTPUT: {
            struct upipe **p = va_arg(args, struct upipe **);
            return upipe_ts_split_output_get_output(upipe, p);
        }
        case UPIPE_SET_OUTPUT: {
            struct upipe *output = va_arg(args, struct upipe *);
            return upipe_ts_split_output_set_output(upipe, output);
        }
        case UPIPE_GET_FLOW_DEF: {
            struct uref **p = va_arg(args, struct uref **);
            return upipe_ts_split_output_get_flow_def(upipe, p);
        }
        case UPIPE_SET_FLOW_DEF: {
            struct uref *flow_def = va_arg(args, struct uref *);
            return upipe_ts_split_output_set_flow_def(upipe, flow_def);
        }

        default:
            return false;
    }
}

/** @This increments the reference count of a upipe.
 *
 * @param upipe description structure of the pipe
 */
static void upipe_ts_split_output_use(struct upipe *upipe)
{
    struct upipe_ts_split_output *upipe_ts_split_output =
        upipe_ts_split_output_from_upipe(upipe);
    urefcount_use(&upipe_ts_split_output->refcount);
}

/** @This decrements the reference count of a upipe or frees it.
 *
 * @param upipe description structure of the pipe
 */
static void upipe_ts_split_output_release(struct upipe *upipe)
{
    struct upipe_ts_split_output *upipe_ts_split_output =
        upipe_ts_split_output_from_upipe(upipe);
    if (unlikely(urefcount_release(&upipe_ts_split_output->refcount))) {
        struct upipe_ts_split *upipe_ts_split =
            upipe_ts_split_from_output_mgr(upipe->mgr);

        /* remove output from the outputs list */
        if (upipe_ts_split_output->flow_def != NULL) {
            uint64_t pid;
            if (uref_ts_flow_get_pid(upipe_ts_split_output->flow_def, &pid))
                upipe_ts_split_pid_unset(
                        upipe_ts_split_to_upipe(upipe_ts_split), pid,
                        upipe_ts_split_output);
        }

        upipe_throw_dead(upipe);
        upipe_ts_split_output_clean_output(upipe);

        upipe_clean(upipe);
        urefcount_clean(&upipe_ts_split_output->refcount);
        free(upipe_ts_split_output);
    }
}

/** @This increments the reference count of a upipe manager.
 *
 * @param mgr pointer to upipe manager
 */
static void upipe_ts_split_output_mgr_use(struct upipe_mgr *mgr)
{
    struct upipe_ts_split *upipe_ts_split = upipe_ts_split_from_output_mgr(mgr);
    upipe_use(upipe_ts_split_to_upipe(upipe_ts_split));
}

/** @This decrements the reference count of a upipe manager or frees it.
 *
 * @param mgr pointer to upipe manager.
 */
static void upipe_ts_split_output_mgr_release(struct upipe_mgr *mgr)
{
    struct upipe_ts_split *upipe_ts_split = upipe_ts_split_from_output_mgr(mgr);
    upipe_release(upipe_ts_split_to_upipe(upipe_ts_split));
}

/** @internal @This initializes the output manager for a ts_split pipe.
 *
 * @param upipe description structure of the pipe
 * @return pointer to output upipe manager
 */
static struct upipe_mgr *upipe_ts_split_init_output_mgr(struct upipe *upipe)
{
    struct upipe_ts_split *upipe_ts_split = upipe_ts_split_from_upipe(upipe);
    struct upipe_mgr *output_mgr = &upipe_ts_split->output_mgr;
    output_mgr->signature = UPIPE_TS_SPLIT_OUTPUT_SIGNATURE;
    output_mgr->upipe_alloc = upipe_ts_split_output_alloc;
    output_mgr->upipe_input = NULL;
    output_mgr->upipe_control = upipe_ts_split_output_control;
    output_mgr->upipe_use = upipe_ts_split_output_use;
    output_mgr->upipe_release = upipe_ts_split_output_release;
    output_mgr->upipe_mgr_use = upipe_ts_split_output_mgr_use;
    output_mgr->upipe_mgr_release = upipe_ts_split_output_mgr_release;
    return output_mgr;
}

/** @internal @This allocates a ts_split pipe.
 *
 * @param mgr common management structure
 * @param uprobe structure used to raise events
 * @return pointer to upipe or NULL in case of allocation error
 */
static struct upipe *upipe_ts_split_alloc(struct upipe_mgr *mgr,
                                          struct uprobe *uprobe)
{
    struct upipe_ts_split *upipe_ts_split =
        malloc(sizeof(struct upipe_ts_split));
    if (unlikely(upipe_ts_split == NULL))
        return NULL;
    struct upipe *upipe = upipe_ts_split_to_upipe(upipe_ts_split);
    upipe_split_init(upipe, mgr, uprobe, upipe_ts_split_init_output_mgr(upipe));
    upipe_ts_split->flow_def_ok = false;

    int i;
    for (i = 0; i < MAX_PIDS; i++) {
        ulist_init(&upipe_ts_split->pids[i].outputs);
        upipe_ts_split->pids[i].set = false;
    }
    urefcount_init(&upipe_ts_split->refcount);
    upipe_throw_ready(upipe);
    return upipe;
}

/** @internal @This checks the status of the PID, and sends the split_set_pid
 * or split_unset_pid event if it has not already been sent.
 *
 * @param upipe description structure of the pipe
 * @param pid PID to check
 */
static void upipe_ts_split_pid_check(struct upipe *upipe, uint16_t pid)
{
    assert(pid < MAX_PIDS);
    struct upipe_ts_split *upipe_ts_split = upipe_ts_split_from_upipe(upipe);
    if (!ulist_empty(&upipe_ts_split->pids[pid].outputs)) {
        if (!upipe_ts_split->pids[pid].set) {
            upipe_ts_split->pids[pid].set = true;
            upipe_throw(upipe, UPROBE_TS_SPLIT_SET_PID,
                        UPIPE_TS_SPLIT_SIGNATURE, (unsigned int)pid);
        }
    } else {
        if (upipe_ts_split->pids[pid].set) {
            upipe_ts_split->pids[pid].set = false;
            upipe_throw(upipe, UPROBE_TS_SPLIT_UNSET_PID,
                        UPIPE_TS_SPLIT_SIGNATURE, (unsigned int)pid);
        }
    }
}

/** @internal @This adds an output to a given PID.
 *
 * @param upipe description structure of the pipe
 * @param pid PID
 * @param output output sub-structure
 */
static void upipe_ts_split_pid_set(struct upipe *upipe, uint16_t pid,
                                   struct upipe_ts_split_output *output)
{
    assert(pid < MAX_PIDS);
    struct upipe_ts_split *upipe_ts_split = upipe_ts_split_from_upipe(upipe);
    ulist_add(&upipe_ts_split->pids[pid].outputs,
              upipe_ts_split_output_to_uchain(output));
    upipe_ts_split_pid_check(upipe, pid);
}

/** @internal @This removes an output from a given PID.
 *
 * @param upipe description structure of the pipe
 * @param pid PID
 * @param output output sub-structure
 */
static void upipe_ts_split_pid_unset(struct upipe *upipe, uint16_t pid,
                                     struct upipe_ts_split_output *output)
{
    assert(pid < MAX_PIDS);
    struct upipe_ts_split *upipe_ts_split = upipe_ts_split_from_upipe(upipe);
    struct uchain *uchain;
    ulist_delete_foreach (&upipe_ts_split->pids[pid].outputs, uchain) {
        if (output == upipe_ts_split_output_from_uchain(uchain)) {
            ulist_delete(&upipe_ts_split->pids[pid].outputs, uchain);
        }
    }
    upipe_ts_split_pid_check(upipe, pid);
}

/** @internal @This demuxes a TS packet to the appropriate output(s).
 *
 * @param upipe description structure of the pipe
 * @param uref uref structure
 * @param upump pump that generated the buffer
 */
static void upipe_ts_split_work(struct upipe *upipe, struct uref *uref,
                                struct upump *upump)
{
    struct upipe_ts_split *upipe_ts_split = upipe_ts_split_from_upipe(upipe);
    uint8_t buffer[TS_HEADER_SIZE];
    const uint8_t *ts_header = uref_block_peek(uref, 0, TS_HEADER_SIZE,
                                               buffer);
    if (unlikely(ts_header == NULL)) {
        upipe_throw_aerror(upipe);
        uref_free(uref);
        return;
    }
    uint16_t pid = ts_get_pid(ts_header);
    bool ret = uref_block_peek_unmap(uref, 0, TS_HEADER_SIZE,
                                     buffer, ts_header);
    assert(ret);

    struct uchain *uchain;
    ulist_foreach (&upipe_ts_split->pids[pid].outputs, uchain) {
        struct upipe_ts_split_output *output =
                upipe_ts_split_output_from_uchain(uchain);
        if (likely(uchain->next == NULL)) {
            upipe_ts_split_output_output(upipe_ts_split_output_to_upipe(output),
                                         uref, upump);
            uref = NULL;
        } else {
            struct uref *new_uref = uref_dup(uref);
            if (likely(new_uref != NULL))
                upipe_ts_split_output_output(
                        upipe_ts_split_output_to_upipe(output),
                        new_uref, upump);
            else {
                uref_free(uref);
                upipe_throw_aerror(upipe);
                return;
            }
        }
    }
    if (uref != NULL)
        uref_free(uref);
}

/** @internal @This receives data.
 *
 * @param upipe description structure of the pipe
 * @param uref uref structure
 * @param upump pump that generated the buffer
 */
static void upipe_ts_split_input(struct upipe *upipe, struct uref *uref,
                                 struct upump *upump)
{
    struct upipe_ts_split *upipe_ts_split = upipe_ts_split_from_upipe(upipe);
    const char *def;
    if (unlikely(uref_flow_get_def(uref, &def))) {
        if (unlikely(ubase_ncmp(def, EXPECTED_FLOW_DEF))) {
            uref_free(uref);
            upipe_ts_split->flow_def_ok = false;
            upipe_throw_flow_def_error(upipe, uref);
            return;
        }

        upipe_dbg_va(upipe, "flow definition: %s", def);
        upipe_ts_split->flow_def_ok = true;
        uref_free(uref);
        return;
    }

    if (unlikely(!upipe_ts_split->flow_def_ok)) {
        uref_free(uref);
        upipe_throw_flow_def_error(upipe, uref);
        return;
    }

    if (unlikely(uref->ubuf == NULL)) {
        uref_free(uref);
        return;
    }

    upipe_ts_split_work(upipe, uref, upump);
}

/** @This increments the reference count of a upipe.
 *
 * @param upipe description structure of the pipe
 */
static void upipe_ts_split_use(struct upipe *upipe)
{
    struct upipe_ts_split *upipe_ts_split = upipe_ts_split_from_upipe(upipe);
    urefcount_use(&upipe_ts_split->refcount);
}

/** @This decrements the reference count of a upipe or frees it.
 *
 * @param upipe description structure of the pipe
 */
static void upipe_ts_split_release(struct upipe *upipe)
{
    struct upipe_ts_split *upipe_ts_split = upipe_ts_split_from_upipe(upipe);
    if (unlikely(urefcount_release(&upipe_ts_split->refcount))) {
        upipe_throw_dead(upipe);

        /* we can only arrive here if there is no output anymore, so no
         * need to empty the outputs list */
        upipe_clean(upipe);
        urefcount_clean(&upipe_ts_split->refcount);
        free(upipe_ts_split);
    }
}

/** module manager static descriptor */
static struct upipe_mgr upipe_ts_split_mgr = {
    .signature = UPIPE_TS_SPLIT_SIGNATURE,

    .upipe_alloc = upipe_ts_split_alloc,
    .upipe_input = upipe_ts_split_input,
    .upipe_control = NULL,
    .upipe_use = upipe_ts_split_use,
    .upipe_release = upipe_ts_split_release,

    .upipe_mgr_use = NULL,
    .upipe_mgr_release = NULL
};

/** @This returns the management structure for all ts_split pipes.
 *
 * @return pointer to manager
 */
struct upipe_mgr *upipe_ts_split_mgr_alloc(void)
{
    return &upipe_ts_split_mgr;
}
