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
 * @short Upipe higher-level module demuxing elementary streams of a TS
 */

#ifndef _UPIPE_TS_UPIPE_TS_DEMUX_H_
/** @hidden */
#define _UPIPE_TS_UPIPE_TS_DEMUX_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <upipe/upipe.h>
#include <upipe-ts/upipe_ts.h>

#define UPIPE_TS_DEMUX_SIGNATURE UBASE_FOURCC('t','s','d','x')
#define UPIPE_TS_DEMUX_PROGRAM_SIGNATURE UBASE_FOURCC('t','s','d','p')
#define UPIPE_TS_DEMUX_OUTPUT_SIGNATURE UBASE_FOURCC('t','s','d','o')

/** @This extends uprobe_event with specific events for ts demux. */
enum uprobe_ts_demux_event {
    UPROBE_TS_DEMUX_SENTINEL = UPROBE_LOCAL,

    /** ts_split events begin here */
    UPROBE_TS_DEMUX_SPLIT = UPROBE_LOCAL + 0x1000
};

/** @This extends upipe_command with specific commands for ts demux. */
enum upipe_ts_demux_command {
    UPIPE_TS_DEMUX_SENTINEL = UPIPE_CONTROL_LOCAL,

    /** returns the currently detected conformance (int *) */
    UPIPE_TS_DEMUX_GET_CONFORMANCE,
    /** sets the conformance (int) */
    UPIPE_TS_DEMUX_SET_CONFORMANCE,
    /** sets the BISS-CA private key file (const char *) */
    UPIPE_TS_DEMUX_SET_PRIVATE_KEY,
    /** enables or disables EITp/f decoding (int) */
    UPIPE_TS_DEMUX_SET_EIT_ENABLED,
    /** enables  or disables EITs table ID decoding (int) */
    UPIPE_TS_DEMUX_SET_EITS_ENABLED,
};

/** @This returns the currently detected conformance mode. It cannot return
 * UPIPE_TS_CONFORMANCE_AUTO.
 *
 * @param upipe description structure of the pipe
 * @param conformance_p filled in with the conformance
 * @return an error code
 */
static inline int
    upipe_ts_demux_get_conformance(struct upipe *upipe,
                                   enum upipe_ts_conformance *conformance_p)
{
    return upipe_control(upipe, UPIPE_TS_DEMUX_GET_CONFORMANCE,
                         UPIPE_TS_DEMUX_SIGNATURE, conformance_p);
}

/** @This sets the conformance mode.
 *
 * @param upipe description structure of the pipe
 * @param conformance conformance mode
 * @return an error code
 */
static inline int
    upipe_ts_demux_set_conformance(struct upipe *upipe,
                                   enum upipe_ts_conformance conformance)
{
    return upipe_control(upipe, UPIPE_TS_DEMUX_SET_CONFORMANCE,
                         UPIPE_TS_DEMUX_SIGNATURE, conformance);
}

/** @This sets the BISS-CA private key.
 *
 * @param upipe description structure of the pipe
 * @param private_key the private_key file
 * @return an error code
 */
static inline int upipe_ts_demux_set_private_key(struct upipe *upipe,
    const char *private_key)
{
    return upipe_control(upipe, UPIPE_TS_DEMUX_SET_PRIVATE_KEY,
            UPIPE_TS_DEMUX_SIGNATURE, private_key);
}

/** @This enables or disables EITp/f decoding.
 *
 * @param upipe description structure of the pipe
 * @param enabled true to enable decoding, false otherwise
 * @return an error code
 */
static inline int upipe_ts_demux_set_eit_enabled(struct upipe *upipe,
                                                 bool enabled)
{
    return upipe_control(upipe, UPIPE_TS_DEMUX_SET_EIT_ENABLED,
                         UPIPE_TS_DEMUX_SIGNATURE, enabled ? 1 : 0);
}

/** @This enables or disables EITs table ID decoding.
 *
 * @param upipe description structure of the pipe
 * @param enabled true to enable decoding, false otherwise
 * @return an error code
 */
static inline int upipe_ts_demux_set_eits_enabled(struct upipe *upipe,
                                                  bool enabled)
{
    return upipe_control(upipe, UPIPE_TS_DEMUX_SET_EITS_ENABLED,
                         UPIPE_TS_DEMUX_SIGNATURE, enabled ? 1 : 0);
}

/** @This returns the management structure for all ts_demux pipes.
 *
 * @return pointer to manager
 */
struct upipe_mgr *upipe_ts_demux_mgr_alloc(void);

/** @This extends upipe_mgr_command with specific commands for ts_demux. */
enum upipe_ts_demux_mgr_command {
    UPIPE_TS_DEMUX_MGR_SENTINEL = UPIPE_MGR_CONTROL_LOCAL,

/** @hidden */
#define UPIPE_TS_DEMUX_MGR_GET_SET_MGR(name, NAME)                          \
    /** returns the current manager for name inner pipes                    \
     * (struct upipe_mgr **) */                                             \
    UPIPE_TS_DEMUX_MGR_GET_##NAME##_MGR,                                    \
    /** sets the manager for name inner pipes (struct upipe_mgr *) */       \
    UPIPE_TS_DEMUX_MGR_SET_##NAME##_MGR,

    UPIPE_TS_DEMUX_MGR_GET_SET_MGR(null, NULL)
    UPIPE_TS_DEMUX_MGR_GET_SET_MGR(setrap, SETRAP)
    UPIPE_TS_DEMUX_MGR_GET_SET_MGR(idem, IDEM)
    UPIPE_TS_DEMUX_MGR_GET_SET_MGR(probe_uref, PROBE_UREF)

    UPIPE_TS_DEMUX_MGR_GET_SET_MGR(ts_split, TS_SPLIT)
    UPIPE_TS_DEMUX_MGR_GET_SET_MGR(ts_sync, TS_SYNC)
    UPIPE_TS_DEMUX_MGR_GET_SET_MGR(ts_check, TS_CHECK)
    UPIPE_TS_DEMUX_MGR_GET_SET_MGR(ts_decaps, TS_DECAPS)
    UPIPE_TS_DEMUX_MGR_GET_SET_MGR(ts_psim, TS_PSIM)
    UPIPE_TS_DEMUX_MGR_GET_SET_MGR(ts_psi_split, TS_PSI_SPLIT)
    UPIPE_TS_DEMUX_MGR_GET_SET_MGR(ts_patd, TS_PATD)
    UPIPE_TS_DEMUX_MGR_GET_SET_MGR(ts_nitd, TS_NITD)
    UPIPE_TS_DEMUX_MGR_GET_SET_MGR(ts_sdtd, TS_SDTD)
    UPIPE_TS_DEMUX_MGR_GET_SET_MGR(ts_tdtd, TS_TDTD)
    UPIPE_TS_DEMUX_MGR_GET_SET_MGR(ts_pmtd, TS_PMTD)
    UPIPE_TS_DEMUX_MGR_GET_SET_MGR(ts_eitd, TS_EITD)
    UPIPE_TS_DEMUX_MGR_GET_SET_MGR(ts_pesd, TS_PESD)
    UPIPE_TS_DEMUX_MGR_GET_SET_MGR(ts_scte35d, TS_SCTE35D)

    UPIPE_TS_DEMUX_MGR_GET_SET_MGR(autof, AUTOF)
#undef UPIPE_TS_DEMUX_MGR_GET_SET_MGR
};

/** @hidden */
#define UPIPE_TS_DEMUX_MGR_GET_SET_MGR2(name, NAME)                         \
/** @This returns the current manager for name inner pipes.                 \
 *                                                                          \
 * @param mgr pointer to manager                                            \
 * @param p filled in with the name manager                                 \
 * @return an error code                                                    \
 */                                                                         \
static inline int                                                           \
    upipe_ts_demux_mgr_get_##name##_mgr(struct upipe_mgr *mgr,              \
                                        struct upipe_mgr *p)                \
{                                                                           \
    return upipe_mgr_control(mgr, UPIPE_TS_DEMUX_MGR_GET_##NAME##_MGR,      \
                             UPIPE_TS_DEMUX_SIGNATURE, p);                  \
}                                                                           \
/** @This sets the manager for name inner pipes. This may only be called    \
 * before any pipe has been allocated.                                      \
 *                                                                          \
 * @param mgr pointer to manager                                            \
 * @param m pointer to name manager                                         \
 * @return an error code                                                    \
 */                                                                         \
static inline int                                                           \
    upipe_ts_demux_mgr_set_##name##_mgr(struct upipe_mgr *mgr,              \
                                        struct upipe_mgr *m)                \
{                                                                           \
    return upipe_mgr_control(mgr, UPIPE_TS_DEMUX_MGR_SET_##NAME##_MGR,      \
                             UPIPE_TS_DEMUX_SIGNATURE, m);                  \
}

UPIPE_TS_DEMUX_MGR_GET_SET_MGR2(null, NULL)
UPIPE_TS_DEMUX_MGR_GET_SET_MGR2(setrap, SETRAP)
UPIPE_TS_DEMUX_MGR_GET_SET_MGR2(idem, IDEM)
UPIPE_TS_DEMUX_MGR_GET_SET_MGR2(probe_uref, PROBE_UREF)

UPIPE_TS_DEMUX_MGR_GET_SET_MGR2(ts_split, TS_SPLIT)
UPIPE_TS_DEMUX_MGR_GET_SET_MGR2(ts_sync, TS_SYNC)
UPIPE_TS_DEMUX_MGR_GET_SET_MGR2(ts_check, TS_CHECK)
UPIPE_TS_DEMUX_MGR_GET_SET_MGR2(ts_decaps, TS_DECAPS)
UPIPE_TS_DEMUX_MGR_GET_SET_MGR2(ts_psim, TS_PSIM)
UPIPE_TS_DEMUX_MGR_GET_SET_MGR2(ts_psi_split, TS_PSI_SPLIT)
UPIPE_TS_DEMUX_MGR_GET_SET_MGR2(ts_patd, TS_PATD)
UPIPE_TS_DEMUX_MGR_GET_SET_MGR2(ts_nitd, TS_NITD)
UPIPE_TS_DEMUX_MGR_GET_SET_MGR2(ts_sdtd, TS_SDTD)
UPIPE_TS_DEMUX_MGR_GET_SET_MGR2(ts_tdtd, TS_TDTD)
UPIPE_TS_DEMUX_MGR_GET_SET_MGR2(ts_pmtd, TS_PMTD)
UPIPE_TS_DEMUX_MGR_GET_SET_MGR2(ts_eitd, TS_EITD)
UPIPE_TS_DEMUX_MGR_GET_SET_MGR2(ts_pesd, TS_PESD)
UPIPE_TS_DEMUX_MGR_GET_SET_MGR2(ts_scte35d, TS_SCTE35D)

UPIPE_TS_DEMUX_MGR_GET_SET_MGR2(autof, AUTOF)
#undef UPIPE_TS_DEMUX_MGR_GET_SET_MGR2

#ifdef __cplusplus
}
#endif
#endif
