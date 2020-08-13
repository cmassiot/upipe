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
 * @short Upipe module building frames from an ISO 13818-3 or 7 stream
 * This framer supports levels 1, 2, 3 of ISO/IEC 11179-3 and ISO/IEC 13818-3,
 * and ISO/IEC 13818-7 (ADTS AAC) and ISO/IEC 14496-3 (raw AAC) streams
 */

#include <upipe/ubase.h>
#include <upipe/ubits.h>
#include <upipe/ulist.h>
#include <upipe/uprobe.h>
#include <upipe/uref.h>
#include <upipe/uref_flow.h>
#include <upipe/uref_block.h>
#include <upipe/uref_block_flow.h>
#include <upipe/uref_sound_flow.h>
#include <upipe/uref_clock.h>
#include <upipe/uclock.h>
#include <upipe/ubuf.h>
#include <upipe/ubuf_block.h>
#include <upipe/ubuf_block_stream.h>
#include <upipe/upipe.h>
#include <upipe/upipe_helper_upipe.h>
#include <upipe/upipe_helper_urefcount.h>
#include <upipe/upipe_helper_void.h>
#include <upipe/upipe_helper_sync.h>
#include <upipe/upipe_helper_ubuf_mgr.h>
#include <upipe/upipe_helper_uref_stream.h>
#include <upipe/upipe_helper_output.h>
#include <upipe/upipe_helper_input.h>
#include <upipe/upipe_helper_flow_format.h>
#include <upipe/upipe_helper_flow_def.h>
#include <upipe-framers/upipe_mpga_framer.h>
#include <upipe-framers/uref_mpga_flow.h>

#include <stdlib.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include <inttypes.h>
#include <assert.h>

#include <bitstream/mpeg/mpga.h>
#include <bitstream/mpeg/aac.h>

/** From ETSI TS 101 154 V2.3.1 and SCTE 193-2 2014 */
#define LATM_CONFIG_PERIOD (UCLOCK_FREQ / 2)
/** 73 bits */
#define MAX_ASC_SIZE 10
/** max number of octetrate changes before considering it is free octetrate
 * (mp3 only) */
#define MAX_OCTETRATE_CHANGES 10

/** @This returns the octetrate / 1000 of an MPEG-1 or 2 audio stream. */
static const uint8_t mpeg_octetrate_table[2][3][16] = {
    { /* MPEG-1 */
        /* layer 1 */
        { 0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 0 },
        /* layer 2 */
        { 0, 4, 6,  7,  8, 10, 12, 14, 16, 20, 24, 28, 32, 40, 48, 0 },
        /* layer 3 */
        { 0, 4, 5,  6,  7,  8, 10, 12, 14, 16, 20, 24, 28, 32, 40, 0 },
    }, { /* MPEG-2 */
        /* layer 1 */
        { 0, 4, 6,  7,  8, 10, 12, 14, 16, 18, 20, 22, 24, 28, 32, 0 },
        /* layer 2 */
        { 0, 1, 2,  3,  4,  5,  6,  7,  8, 10, 12, 14, 16, 18, 20, 0 },
        /* layer 3 */
        { 0, 1, 2,  3,  4,  5,  6,  7,  8, 10, 12, 14, 16, 18, 20, 0 }
    }
};

/** @This returns the sample rate in Hz of an MPEG-1 or 2 audio stream. */
static const unsigned int mpeg_samplerate_table[2][4] = {
    { 44100, 48000, 32000, 0 }, /* MPEG-1 */
    { 22050, 24000, 16000, 0 }  /* MPEG-2 */
};

/** @This returns the sample rate in Hz of an AAC audio stream. */
static const unsigned int aac_samplerate_table[16] = {
    96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050,
    16000, 12000, 11025, 8000,  7350,  0,     0,     0
};

/** @This represents the coding type of the audio ES. */
enum upipe_mpgaf_type {
    /** MPEG-1 and 2 layers 1, 2 and 3 */
    UPIPE_MPGAF_MP2 = 0,
    /** MPEG-2/4 Advanced Audio Coding */
    UPIPE_MPGAF_AAC,
    /** Unknown */
    UPIPE_MPGAF_UNKNOWN
};

/** @internal @This is the private context of an mpgaf pipe. */
struct upipe_mpgaf {
    /** refcount management structure */
    struct urefcount urefcount;

    /* output stuff */
    /** pipe acting as output */
    struct upipe *output;
    /** output flow definition packet */
    struct uref *flow_def;
    /** output state */
    enum upipe_helper_output_state output_state;
    /** list of output requests */
    struct uchain request_list;
    /** input flow definition packet */
    struct uref *flow_def_input;
    /** attributes in the sequence header */
    struct uref *flow_def_attr;
    /** requested flow definition */
    struct uref *flow_def_requested;
    /** input AAC encapsulation */
    enum uref_mpga_encaps encaps_input;
    /** output AAC encapsulation */
    enum uref_mpga_encaps encaps_output;
    /** complete input */
    bool complete_input;

    /** flow format request */
    struct urequest request;
    /** temporary uref storage (used during urequest) */
    struct uchain request_urefs;
    /** nb urefs in storage */
    unsigned int nb_urefs;
    /** max urefs in storage */
    unsigned int max_urefs;
    /** list of blockers (used during urequest) */
    struct uchain blockers;
    /** buffered output uref (used during urequest) */
    struct uref *uref_output;

    /** ubuf manager */
    struct ubuf_mgr *ubuf_mgr;
    /** flow format packet */
    struct uref *flow_format;
    /** ubuf manager request */
    struct urequest ubuf_mgr_request;

    /** latency in the input flow */
    uint64_t input_latency;
    /** coding type */
    enum upipe_mpgaf_type type;

    /* sync parsing stuff */
    /** number of octets in a frame */
    size_t frame_size;
    /** number of octets in a frame with padding enabled */
    size_t frame_size_padding;
    /** number of samples in a frame */
    size_t samples;
    /** has CRC (ADTS) */
    bool has_crc;
    /** audio object type (ASC) */
    uint8_t asc_aot;
    /** base audio object type (ASC) */
    uint8_t asc_base_aot;
    /** frame length flag (ASC) */
    bool asc_frame_length;
    /** sample rate index (AAC) */
    uint8_t samplerate_idx;
    /** number of samples per second */
    uint64_t samplerate;
    /** base sample rate index (HE AAC) */
    uint8_t base_samplerate_idx;
    /** number of samples per second (HE AAC) */
    uint64_t base_samplerate;
    /** number of channels */
    uint8_t channels;
    /** frame length type (LATM) */
    uint8_t frame_length_type;
    /** octet rate */
    uint64_t octetrate;
    /* number of octetrate changes */
    unsigned int octetrate_changes;
    /** residue of the duration in 27 MHz units */
    uint64_t duration_residue;
    /** true we have had a discontinuity recently */
    bool got_discontinuity;
    /** sync header */
    uint8_t sync_header[MPGA_HEADER_SIZE + ADTS_HEADER_SIZE]; // to be sure
    /** number of bits the header (LATM is not octet-aligned) */
    int latm_header_size;
    /** duration since last LATM mux configuration */
    uint64_t latm_config_duration;

    /* octet stream stuff */
    /** next uref to be processed */
    struct uref *next_uref;
    /** original size of the next uref */
    size_t next_uref_size;
    /** urefs received after next uref */
    struct uchain urefs;

    /* octet stream parser stuff */
    /** current size of next frame (in next_uref) */
    ssize_t next_frame_size;
    /** pseudo-packet containing date information for the next frame */
    struct uref au_uref_s;
    /** drift rate of the next frame */
    struct urational drift_rate;
    /** true if we have thrown the sync_acquired event (that means we found a
     * header) */
    bool acquired;

    /** public upipe structure */
    struct upipe upipe;
};

/** @hidden */
static void upipe_mpgaf_promote_uref(struct upipe *upipe);
/** @hidden */
static bool upipe_mpgaf_handle(struct upipe *upipe, struct uref *uref,
                               struct upump **upump_p);
/** @hidden */
static int upipe_mpgaf_check_flow_format(struct upipe *upipe,
                                         struct uref *flow_format);
/** @hidden */
static int upipe_mpgaf_check_ubuf_mgr(struct upipe *upipe,
                                      struct uref *flow_format);

UPIPE_HELPER_UPIPE(upipe_mpgaf, upipe, UPIPE_MPGAF_SIGNATURE)
UPIPE_HELPER_UREFCOUNT(upipe_mpgaf, urefcount, upipe_mpgaf_free)
UPIPE_HELPER_VOID(upipe_mpgaf)
UPIPE_HELPER_SYNC(upipe_mpgaf, acquired)
UPIPE_HELPER_UREF_STREAM(upipe_mpgaf, next_uref, next_uref_size, urefs,
                         upipe_mpgaf_promote_uref)

UPIPE_HELPER_OUTPUT(upipe_mpgaf, output, flow_def, output_state, request_list)
UPIPE_HELPER_INPUT(upipe_mpgaf, request_urefs, nb_urefs, max_urefs, blockers, upipe_mpgaf_handle)
UPIPE_HELPER_FLOW_FORMAT(upipe_mpgaf, request, upipe_mpgaf_check_flow_format,
                         upipe_mpgaf_register_output_request,
                         upipe_mpgaf_unregister_output_request)
UPIPE_HELPER_FLOW_DEF(upipe_mpgaf, flow_def_input, flow_def_attr)
UPIPE_HELPER_UBUF_MGR(upipe_mpgaf, ubuf_mgr, flow_format,
                      ubuf_mgr_request, upipe_mpgaf_check_ubuf_mgr,
                      upipe_mpgaf_register_output_request,
                      upipe_mpgaf_unregister_output_request)

/** @internal @This flushes all dates.
 *
 * @param upipe description structure of the pipe
 */
static void upipe_mpgaf_flush_dates(struct upipe *upipe)
{
    struct upipe_mpgaf *upipe_mpgaf = upipe_mpgaf_from_upipe(upipe);
    uref_clock_set_date_sys(&upipe_mpgaf->au_uref_s, UINT64_MAX,
                            UREF_DATE_NONE);
    uref_clock_set_date_prog(&upipe_mpgaf->au_uref_s, UINT64_MAX,
                             UREF_DATE_NONE);
    uref_clock_set_date_orig(&upipe_mpgaf->au_uref_s, UINT64_MAX,
                             UREF_DATE_NONE);
    uref_clock_delete_dts_pts_delay(&upipe_mpgaf->au_uref_s);
}

/** @internal @This allocates an mpgaf pipe.
 *
 * @param mgr common management structure
 * @param uprobe structure used to raise events
 * @param signature signature of the pipe allocator
 * @param args optional arguments
 * @return pointer to upipe or NULL in case of allocation error
 */
static struct upipe *upipe_mpgaf_alloc(struct upipe_mgr *mgr,
                                       struct uprobe *uprobe,
                                       uint32_t signature, va_list args)
{
    struct upipe *upipe = upipe_mpgaf_alloc_void(mgr, uprobe, signature, args);
    if (unlikely(upipe == NULL))
        return NULL;

    struct upipe_mpgaf *upipe_mpgaf = upipe_mpgaf_from_upipe(upipe);
    upipe_mpgaf_init_urefcount(upipe);
    upipe_mpgaf_init_sync(upipe);
    upipe_mpgaf_init_uref_stream(upipe);
    upipe_mpgaf_init_output(upipe);
    upipe_mpgaf_init_input(upipe);
    upipe_mpgaf_init_flow_format(upipe);
    upipe_mpgaf_init_flow_def(upipe);
    upipe_mpgaf_init_ubuf_mgr(upipe);
    upipe_mpgaf->flow_def_requested = NULL;
    upipe_mpgaf->encaps_input = upipe_mpgaf->encaps_output =
        UREF_MPGA_ENCAPS_ADTS;
    upipe_mpgaf->complete_input = false;
    upipe_mpgaf->uref_output = NULL;
    upipe_mpgaf->type = UPIPE_MPGAF_UNKNOWN;
    upipe_mpgaf->input_latency = 0;
    upipe_mpgaf->has_crc = 0;
    upipe_mpgaf->asc_aot = upipe_mpgaf->asc_base_aot = 0;
    upipe_mpgaf->asc_frame_length = false;
    upipe_mpgaf->samplerate_idx = upipe_mpgaf->base_samplerate_idx = 0;
    upipe_mpgaf->samplerate = upipe_mpgaf->base_samplerate = 0;
    upipe_mpgaf->frame_length_type = 0;
    upipe_mpgaf->octetrate = 0;
    upipe_mpgaf->octetrate_changes = 0;
    upipe_mpgaf->duration_residue = 0;
    upipe_mpgaf->got_discontinuity = false;
    upipe_mpgaf->next_frame_size = -1;
    uref_init(&upipe_mpgaf->au_uref_s);
    upipe_mpgaf_flush_dates(upipe);
    upipe_mpgaf->drift_rate.num = upipe_mpgaf->drift_rate.den = 0;
    upipe_mpgaf->sync_header[0] = 0x0;
    upipe_mpgaf->latm_header_size = 0;
    upipe_mpgaf->latm_config_duration = 0;
    upipe_throw_ready(upipe);
    return upipe;
}

/** @internal @This parses a new MPEG layers 1, 2 or 3 header.
 *
 * @param upipe description structure of the pipe
 * @return false in case the header is inconsistent
 */
static bool upipe_mpgaf_parse_mpeg(struct upipe *upipe)
{
    struct upipe_mpgaf *upipe_mpgaf = upipe_mpgaf_from_upipe(upipe);
    uint8_t header[MPGA_HEADER_SIZE];
    if (!ubase_check(uref_block_extract(upipe_mpgaf->next_uref, 0,
                     MPGA_HEADER_SIZE, header)))
        return true; /* not enough data */

    bool mpeg25 = mpga_get_mpeg25(header);
    uint8_t id = mpga_get_id(header) == MPGA_ID_2 ? 1 : 0;
    uint8_t layer = 4 - mpga_get_layer(header);
    uint8_t octetrate = mpeg_octetrate_table[id][layer - 1]
                            [mpga_get_bitrate_index(header)];
    uint8_t max_octetrate = mpeg_octetrate_table[id][layer - 1][14];
    uint8_t padding = mpga_get_padding(header) ? 1 : 0;
    uint8_t mode = mpga_get_mode(header);
    bool copyright = mpga_get_copyright(header);
    bool original = mpga_get_original(header);
    if (!octetrate)
        max_octetrate *= 2;

    if (likely(mpga_sync_compare_formats(header, upipe_mpgaf->sync_header))) {
        /* identical sync */
        upipe_mpgaf->next_frame_size = mpga_get_padding(header) ?
                                       upipe_mpgaf->frame_size_padding :
                                       upipe_mpgaf->frame_size;
        return true;
    }

    if (mpga_get_bitrate_index(header) == MPGA_BITRATE_INVALID ||
        mpga_get_sampling_freq(header) == MPGA_SAMPLERATE_INVALID ||
        mpga_get_emphasis(header) == MPGA_EMPHASIS_INVALID)
        return false;

    if (layer == 3 &&
        mpga_sync_compare_formats_free(header, upipe_mpgaf->sync_header)) {
        if (upipe_mpgaf->octetrate_changes > MAX_OCTETRATE_CHANGES) {
            if (id || mpeg25) {
                upipe_mpgaf->frame_size =
                    576000 * octetrate / upipe_mpgaf->samplerate;
                upipe_mpgaf->frame_size_padding =
                    576000 * octetrate / upipe_mpgaf->samplerate + 1;
            } else {
                upipe_mpgaf->frame_size =
                    1152000 * octetrate / upipe_mpgaf->samplerate;
                upipe_mpgaf->frame_size_padding =
                    1152000 * octetrate / upipe_mpgaf->samplerate + 1;
            }
            memcpy(upipe_mpgaf->sync_header, header, MPGA_HEADER_SIZE);
            upipe_mpgaf->next_frame_size = mpga_get_padding(header) ?
                                           upipe_mpgaf->frame_size_padding :
                                           upipe_mpgaf->frame_size;
            return true;
        }
        upipe_mpgaf->octetrate_changes++;
    }

    struct uref *flow_def = upipe_mpgaf_alloc_flow_def_attr(upipe);
    if (unlikely(flow_def == NULL)) {
        upipe_throw_fatal(upipe, UBASE_ERR_ALLOC);
        return false;
    }
    UBASE_FATAL(upipe, uref_flow_set_complete(flow_def))
    UBASE_FATAL(upipe, uref_mpga_flow_set_encaps(flow_def,
                upipe_mpgaf->encaps_input))

    memcpy(upipe_mpgaf->sync_header, header, MPGA_HEADER_SIZE);
    upipe_mpgaf->samplerate = mpeg_samplerate_table[id]
                                [mpga_get_sampling_freq(header)];
    if (mpeg25)
        upipe_mpgaf->samplerate /= 2;

    switch (layer) {
        case 1:
            upipe_mpgaf->frame_size =
                (96000 * octetrate / upipe_mpgaf->samplerate) * 4;
            upipe_mpgaf->frame_size_padding =
                (96000 * octetrate / upipe_mpgaf->samplerate + 1) * 4;
            upipe_mpgaf->samples = 384;
            UBASE_FATAL(upipe, uref_flow_set_def(flow_def, "block.mp2.sound."))
            break;
        case 2:
            upipe_mpgaf->frame_size =
                1152000 * octetrate / upipe_mpgaf->samplerate;
            upipe_mpgaf->frame_size_padding =
                1152000 * octetrate / upipe_mpgaf->samplerate + 1;
            upipe_mpgaf->samples = 1152;
            UBASE_FATAL(upipe, uref_flow_set_def(flow_def, "block.mp2.sound."))
            break;
        case 3:
            if (id || mpeg25) {
                upipe_mpgaf->frame_size =
                    576000 * octetrate / upipe_mpgaf->samplerate;
                upipe_mpgaf->frame_size_padding =
                    576000 * octetrate / upipe_mpgaf->samplerate + 1;
                upipe_mpgaf->samples = 576;
            } else {
                upipe_mpgaf->frame_size =
                    1152000 * octetrate / upipe_mpgaf->samplerate;
                upipe_mpgaf->frame_size_padding =
                    1152000 * octetrate / upipe_mpgaf->samplerate + 1;
                upipe_mpgaf->samples = 1152;
            }
            UBASE_FATAL(upipe, uref_flow_set_def(flow_def, "block.mp3.sound."))

            if (upipe_mpgaf->octetrate_changes > MAX_OCTETRATE_CHANGES) {
                upipe_notice(upipe, "octetrate changes too often, using max octetrate");
                octetrate = max_octetrate;
            }
            break;
    }

    upipe_mpgaf->channels = mode == MPGA_MODE_MONO ? 1 : 2;

    UBASE_FATAL(upipe, uref_mpga_flow_set_mode(flow_def, mode))
    UBASE_FATAL(upipe, uref_sound_flow_set_channels(flow_def, upipe_mpgaf->channels))
    UBASE_FATAL(upipe, uref_sound_flow_set_rate(flow_def, upipe_mpgaf->samplerate))
    UBASE_FATAL(upipe, uref_sound_flow_set_samples(flow_def, upipe_mpgaf->samples))
    if (!upipe_mpgaf->complete_input)
        UBASE_FATAL(upipe, uref_clock_set_latency(flow_def,
                    upipe_mpgaf->input_latency +
                    UCLOCK_FREQ * upipe_mpgaf->samples /
                    upipe_mpgaf->samplerate))
    upipe_mpgaf->octetrate = (uint64_t)octetrate * 1000;
    if (upipe_mpgaf->octetrate) {
        UBASE_FATAL(upipe, uref_block_flow_set_octetrate(flow_def,
                                                   upipe_mpgaf->octetrate))
    }
    UBASE_FATAL(upipe, uref_block_flow_set_max_octetrate(flow_def,
            (uint64_t)max_octetrate * 1000))
    if (copyright)
        UBASE_FATAL(upipe, uref_flow_set_copyright(flow_def))
    if (original)
        UBASE_FATAL(upipe, uref_flow_set_original(flow_def))

    upipe_mpgaf_store_flow_def(upipe, NULL);
    uref_free(upipe_mpgaf->flow_def_requested);
    upipe_mpgaf->flow_def_requested = NULL;
    flow_def = upipe_mpgaf_store_flow_def_attr(upipe, flow_def);
    if (flow_def != NULL)
        upipe_mpgaf_require_flow_format(upipe, flow_def);

    upipe_mpgaf->next_frame_size = padding ?
                                   upipe_mpgaf->frame_size_padding :
                                   upipe_mpgaf->frame_size;
    return true;
}

/** @internal @This parses a new ADTS AAC header.
 *
 * @param upipe description structure of the pipe
 * @return false in case the header is inconsistent
 */
static bool upipe_mpgaf_parse_adts(struct upipe *upipe)
{
    struct upipe_mpgaf *upipe_mpgaf = upipe_mpgaf_from_upipe(upipe);
    uint8_t header[ADTS_HEADER_SIZE];
    if (!ubase_check(uref_block_extract(upipe_mpgaf->next_uref, 0,
                     ADTS_HEADER_SIZE, header)))
        return true; /* not enough data */

    uint8_t sampling_freq = adts_get_sampling_freq(header);
    uint64_t samplerate = aac_samplerate_table[sampling_freq];
    size_t samples = ADTS_SAMPLES_PER_BLOCK * (1 + adts_get_num_blocks(header));
    uint16_t adts_length = adts_get_length(header);
    uint8_t adts_profile = adts_get_profile(header);
    uint8_t asc_aot = adts_profile +1;
    uint8_t asc_base_aot = asc_aot;

    if (!samplerate) {
        upipe_warn(upipe, "invalid samplerate");
        return false;
    }

    if (adts_length < ADTS_HEADER_SIZE) {
        upipe_warn(upipe, "invalid header");
        return false;
    }

    if (samplerate <= 24000) {
        /* assume SBR on low frequency streams */
        samplerate *= 2;
        samples *= 2;
        asc_aot = ASC_TYPE_SBR;
    }

    /* Calculate octetrate assuming the stream is CBR. */
    uint64_t octetrate = adts_length * samplerate / samples;
    /* Round up to a multiple of 8 kbits/s. */
    octetrate += 999;
    octetrate -= octetrate % 1000;

    if (likely(adts_sync_compare_formats(header, upipe_mpgaf->sync_header) &&
        samples == upipe_mpgaf->samples &&
        octetrate <= upipe_mpgaf->octetrate)) {
        /* identical sync */
        goto upipe_mpgaf_parse_adts_shortcut;
    }

    memcpy(upipe_mpgaf->sync_header, header, ADTS_HEADER_SIZE);

    struct uref *flow_def = upipe_mpgaf_alloc_flow_def_attr(upipe);
    if (unlikely(flow_def == NULL)) {
        upipe_throw_fatal(upipe, UBASE_ERR_ALLOC);
        return false;
    }
    UBASE_FATAL(upipe, uref_flow_set_complete(flow_def))

    upipe_mpgaf->has_crc = !adts_get_protection_absent(header);
    upipe_mpgaf->asc_aot = asc_aot;
    upipe_mpgaf->asc_base_aot = asc_base_aot;
    upipe_mpgaf->samplerate_idx = upipe_mpgaf->base_samplerate_idx =
        sampling_freq;
    upipe_mpgaf->samplerate = upipe_mpgaf->base_samplerate = samplerate;
    upipe_mpgaf->samples = samples;
    upipe_mpgaf->channels = adts_get_channels(header);
    if (upipe_mpgaf->channels == 7)
        upipe_mpgaf->channels = 8;
    upipe_mpgaf->asc_frame_length = false;

    UBASE_FATAL(upipe, uref_flow_set_def(flow_def, "block.aac.sound."))
    if (upipe_mpgaf->channels) {
        UBASE_FATAL(upipe,
                uref_sound_flow_set_channels(flow_def, upipe_mpgaf->channels))
    }
    UBASE_FATAL(upipe,
            uref_mpga_flow_set_encaps(flow_def, UREF_MPGA_ENCAPS_ADTS))
    UBASE_FATAL(upipe,
            uref_sound_flow_set_rate(flow_def, upipe_mpgaf->samplerate))
    UBASE_FATAL(upipe,
            uref_sound_flow_set_samples(flow_def, upipe_mpgaf->samples))
    if (!upipe_mpgaf->complete_input)
        UBASE_FATAL(upipe, uref_clock_set_latency(flow_def,
                    upipe_mpgaf->input_latency +
                    UCLOCK_FREQ * upipe_mpgaf->samples /
                    upipe_mpgaf->samplerate))
    if (adts_get_copy(header))
        UBASE_FATAL(upipe, uref_flow_set_copyright(flow_def))
    if (adts_get_home(header))
        UBASE_FATAL(upipe, uref_flow_set_original(flow_def))

    upipe_mpgaf->octetrate = octetrate;
    UBASE_FATAL(upipe, uref_block_flow_set_octetrate(flow_def, octetrate))

    upipe_mpgaf_store_flow_def(upipe, NULL);
    uref_free(upipe_mpgaf->flow_def_requested);
    upipe_mpgaf->flow_def_requested = NULL;
    flow_def = upipe_mpgaf_store_flow_def_attr(upipe, flow_def);
    if (flow_def != NULL)
        upipe_mpgaf_require_flow_format(upipe, flow_def);

upipe_mpgaf_parse_adts_shortcut:
    upipe_mpgaf->next_frame_size = adts_length;
    return true;
}

/** @internal @This parses a new LOAS header.
 *
 * @param upipe description structure of the pipe
 * @return false in case the header is inconsistent
 */
static bool upipe_mpgaf_parse_loas(struct upipe *upipe)
{
    struct upipe_mpgaf *upipe_mpgaf = upipe_mpgaf_from_upipe(upipe);
    uint8_t header[LOAS_HEADER_SIZE];
    if (!ubase_check(uref_block_extract(upipe_mpgaf->next_uref, 0,
                     LOAS_HEADER_SIZE, header)))
        return true; /* not enough data */

    upipe_mpgaf->next_frame_size = loas_get_length(header) + LOAS_HEADER_SIZE;
    return true;
}

/** @internal @This parses a new header.
 *
 * @param upipe description structure of the pipe
 * @return false in case the header is inconsistent
 */
static bool upipe_mpgaf_parse_header(struct upipe *upipe)
{
    struct upipe_mpgaf *upipe_mpgaf = upipe_mpgaf_from_upipe(upipe);
    if (upipe_mpgaf->encaps_input == UREF_MPGA_ENCAPS_LOAS)
        return upipe_mpgaf_parse_loas(upipe);

    uint8_t header[2];
    if (unlikely(!ubase_check(uref_block_extract(upipe_mpgaf->next_uref, 0, 2, header))))
        return true;

    enum upipe_mpgaf_type type;
    switch (mpga_get_layer(header)) {
        case MPGA_LAYER_1:
        case MPGA_LAYER_2:
        case MPGA_LAYER_3:
            type = UPIPE_MPGAF_MP2;
            break;
        case MPGA_LAYER_ADTS:
            type = UPIPE_MPGAF_AAC;
            break;
        default:
            return false;
    }

    if (unlikely(upipe_mpgaf->type != UPIPE_MPGAF_UNKNOWN &&
                 upipe_mpgaf->type != type)) {
        upipe_warn(upipe, "invalid header");
        return false;
    }
    upipe_mpgaf->type = type;
    if (type == UPIPE_MPGAF_MP2)
        return upipe_mpgaf_parse_mpeg(upipe);
    return upipe_mpgaf_parse_adts(upipe);
}

/** @internal @This parses an LATM value.
 *
 * @param upipe description structure of the pipe
 * @param s ubuf block stream
 * @return value
 */
static uint32_t upipe_mpgaf_parse_latm_value(struct upipe *upipe,
                                             struct ubuf_block_stream *s)
{
    ubuf_block_stream_fill_bits(s, 2);
    /* The spec is ambiguous about + 1 or not. */
    uint8_t bytes = ubuf_block_stream_show_bits(s, 2) + 1;
    ubuf_block_stream_skip_bits(s, 2);

    uint32_t value = 0;
    while (bytes-- > 0) {
        ubuf_block_stream_fill_bits(s, 8);
        value = (value << 8) | ubuf_block_stream_show_bits(s, 8);
        ubuf_block_stream_skip_bits(s, 8);
    }
    return value;
}

/** @internal @This parses an audio object type.
 *
 * @param s ubuf block stream
 * @return audio object type
 */
static uint16_t upipe_mpgaf_handle_asc_aot(struct ubuf_block_stream *s)
{
    ubuf_block_stream_fill_bits(s, 5);
    uint16_t type = ubuf_block_stream_show_bits(s, 5);
    ubuf_block_stream_skip_bits(s, 5);

    if (type != 31)
        return type;

    ubuf_block_stream_fill_bits(s, 6);
    type = 32 + ubuf_block_stream_show_bits(s, 6);
    ubuf_block_stream_skip_bits(s, 6);
    return type;
}

/** @internal @This handles a sample rate.
 *
 * @param s ubuf block stream
 * @param samplerate_idx_p reference to sample rate index variable to fill
 * @param samplerate_p reference to sample rate variable to fill
 */
static void upipe_mpgaf_handle_asc_sample_rate(struct ubuf_block_stream *s,
        uint8_t *samplerate_idx_p, uint64_t *samplerate_p)
{
    ubuf_block_stream_fill_bits(s, 4);
    *samplerate_idx_p = ubuf_block_stream_show_bits(s, 4);
    ubuf_block_stream_skip_bits(s, 4);

    if (*samplerate_idx_p != 0xf) {
        *samplerate_p = aac_samplerate_table[*samplerate_idx_p];
        return;
    }

    ubuf_block_stream_fill_bits(s, 24);
    *samplerate_p = ubuf_block_stream_show_bits(s, 24);
    ubuf_block_stream_skip_bits(s, 24);
}

/** @internal @This handles a GASpecificConfig bitstream.
 *
 * @param upipe description structure of the pipe
 * @param s ubuf block stream
 * @return false if the stream is invalid or unsupported
 */
static bool upipe_mpgaf_handle_asc_ga(struct upipe *upipe,
                                      struct ubuf_block_stream *s)
{
    struct upipe_mpgaf *upipe_mpgaf = upipe_mpgaf_from_upipe(upipe);

    ubuf_block_stream_fill_bits(s, 2);
    upipe_mpgaf->asc_frame_length = ubuf_block_stream_show_bits(s, 1);
    ubuf_block_stream_skip_bits(s, 1);
    bool depends_on_core_coder = ubuf_block_stream_show_bits(s, 1);
    ubuf_block_stream_skip_bits(s, 1);

    switch (upipe_mpgaf->asc_base_aot) {
        default:
            upipe_mpgaf->samples = upipe_mpgaf->asc_frame_length ? 960 : 1024;
            break;
        case ASC_TYPE_SSR:
            upipe_mpgaf->samples = 256;
            break;
    }

    if (depends_on_core_coder) {
        ubuf_block_stream_fill_bits(s, 14);
        ubuf_block_stream_skip_bits(s, 14); /* coreCoderDelay */
    }

    ubuf_block_stream_fill_bits(s, 1);
    bool extension_flag = ubuf_block_stream_show_bits(s, 1);
    ubuf_block_stream_skip_bits(s, 1);

    /* programs > 0 so no PCE */

    if (upipe_mpgaf->asc_base_aot == ASC_TYPE_SCALABLE) {
        ubuf_block_stream_fill_bits(s, 3);
        ubuf_block_stream_skip_bits(s, 3); /* layerNr */
    }

    if (extension_flag) {
        ubuf_block_stream_fill_bits(s, 1);
        bool extension_flag3 = ubuf_block_stream_show_bits(s, 1);
        ubuf_block_stream_skip_bits(s, 1);

        if (extension_flag3) {
            upipe_err(upipe, "unsupported extension flag version 3");
            return false;
        }
    }
    return true;
}

/** @internal @This handles an AudioSpecificConfig bitstream.  *
 * @param upipe description structure of the pipe
 * @param s ubuf block stream
 * @return false if the stream is invalid or unsupported
 */
static bool upipe_mpgaf_handle_asc(struct upipe *upipe,
                                   struct ubuf_block_stream *s)
{
    struct upipe_mpgaf *upipe_mpgaf = upipe_mpgaf_from_upipe(upipe);

    upipe_mpgaf->asc_aot = upipe_mpgaf->asc_base_aot =
        upipe_mpgaf_handle_asc_aot(s);
    upipe_mpgaf_handle_asc_sample_rate(s,
            &upipe_mpgaf->base_samplerate_idx, &upipe_mpgaf->base_samplerate);
    upipe_mpgaf->samplerate_idx = upipe_mpgaf->base_samplerate_idx;
    upipe_mpgaf->samplerate = upipe_mpgaf->base_samplerate;

    ubuf_block_stream_fill_bits(s, 4);
    upipe_mpgaf->channels = ubuf_block_stream_show_bits(s, 4);
    ubuf_block_stream_skip_bits(s, 4);
    if (upipe_mpgaf->channels == 0) {
        upipe_err(upipe, "unsupported channel configuration");
        return false;
    } else if (upipe_mpgaf->channels == 7)
        upipe_mpgaf->channels = 8;

    if (upipe_mpgaf->asc_aot == ASC_TYPE_SBR ||
        upipe_mpgaf->asc_aot == ASC_TYPE_PS) {
        upipe_mpgaf_handle_asc_sample_rate(s,
                &upipe_mpgaf->samplerate_idx, &upipe_mpgaf->samplerate);
        upipe_mpgaf->asc_base_aot = upipe_mpgaf_handle_asc_aot(s);
        if (upipe_mpgaf->asc_base_aot == ASC_TYPE_ER_BSAC) {
            /* extensionChannelConfiguration */
            ubuf_block_stream_fill_bits(s, 4);
            ubuf_block_stream_skip_bits(s, 4);
        }
    }

    switch (upipe_mpgaf->asc_base_aot) {
        case ASC_TYPE_MAIN:
        case ASC_TYPE_LC:
        case ASC_TYPE_SSR:
        case ASC_TYPE_LTP:
        case ASC_TYPE_SCALABLE:
        case ASC_TYPE_TWINVQ:
            break;
        default:
            upipe_err_va(upipe, "unsupported Audio Object Type %" PRIu8,
                         upipe_mpgaf->asc_base_aot);
            return false;
    }

    if (upipe_mpgaf->asc_aot == ASC_TYPE_SBR ||
        upipe_mpgaf->asc_aot == ASC_TYPE_PS)
        upipe_mpgaf->samples *= 2;

    return upipe_mpgaf_handle_asc_ga(upipe, s);
}

/** @internal @This parses an LATM mux configuration.
 *
 * @param upipe description structure of the pipe
 * @param s ubuf block stream
 * @return false if the header is invalid or we miss the mux configuration
 */
static bool upipe_mpgaf_handle_latm_config(struct upipe *upipe,
                                           struct ubuf_block_stream *s)
{
    struct upipe_mpgaf *upipe_mpgaf = upipe_mpgaf_from_upipe(upipe);
    ubuf_block_stream_fill_bits(s, 8);
    bool mux_version = ubuf_block_stream_show_bits(s, 1);
    ubuf_block_stream_skip_bits(s, 1);

    if (mux_version) {
        bool mux_version_a = ubuf_block_stream_show_bits(s, 1);
        ubuf_block_stream_skip_bits(s, 1);

        if (mux_version_a) {
            upipe_err(upipe, "unsupported LATM mux version");
            return false;
        }

        upipe_mpgaf_parse_latm_value(upipe, s); /* taraBufferFullness */
    }

    ubuf_block_stream_fill_bits(s, 11);
    bool same_time_framing = ubuf_block_stream_show_bits(s, 1);
    ubuf_block_stream_skip_bits(s, 1);
    uint8_t sub_frames = 1 + ubuf_block_stream_show_bits(s, 6);
    ubuf_block_stream_skip_bits(s, 6);
    uint8_t programs = 1 + ubuf_block_stream_show_bits(s, 4);
    ubuf_block_stream_skip_bits(s, 4);

    if (!same_time_framing) {
        upipe_err(upipe, "unsupported LATM without same time framing");
        return false;
    }
    if (sub_frames > 1) {
        upipe_err(upipe, "unsupported LATM with sub frames");
        return false;
    }
    if (programs > 1) {
        upipe_err(upipe, "unsupported LATM with programs");
        return false;
    }

    ubuf_block_stream_fill_bits(s, 3);
    uint8_t layers = 1 + ubuf_block_stream_show_bits(s, 3);
    ubuf_block_stream_skip_bits(s, 3);

    if (layers > 1) {
        upipe_err(upipe, "unsupported LATM with layers");
        return false;
    }

    int asc_end = 0;
    if (mux_version)
        asc_end = ubuf_block_stream_position(s) +
                  upipe_mpgaf_parse_latm_value(upipe, s);

    if (!upipe_mpgaf_handle_asc(upipe, s))
        return false;

    if (mux_version) {
        asc_end -= ubuf_block_stream_position(s);
        if (asc_end < 0) {
            upipe_err_va(upipe, "ASC %d bits longer than expected",
                         -asc_end);
            return false;
        }
        while (asc_end > 24) {
            ubuf_block_stream_fill_bits(s, 24);
            ubuf_block_stream_skip_bits(s, 24);
            asc_end -= 24;
        }
        ubuf_block_stream_fill_bits(s, asc_end);
        ubuf_block_stream_skip_bits(s, asc_end);
    }

    ubuf_block_stream_fill_bits(s, 3);
    upipe_mpgaf->frame_length_type = ubuf_block_stream_show_bits(s, 3);
    ubuf_block_stream_skip_bits(s, 3);

    switch (upipe_mpgaf->frame_length_type) {
        case ASC_FLT_VARIABLE:
            ubuf_block_stream_fill_bits(s, 8);
            ubuf_block_stream_skip_bits(s, 8); /* latmBufferFullness */
            break;
        case ASC_FLT_FIXED:
            ubuf_block_stream_fill_bits(s, 9);
            /* currently unused */
            /* frame_length = (ubuf_block_stream_show_bits(s, 9) + 20) * 8; */
            ubuf_block_stream_skip_bits(s, 9);
            break;
        case ASC_FLT_CELP_2:
        case ASC_FLT_CELP_FIXED:
        case ASC_FLT_CELP_4:
            upipe_err(upipe, "unsupported CELP payload");
            return false;
        case ASC_FLT_HVXC_FIXED:
        case ASC_FLT_HVXC_4:
            upipe_err(upipe, "unsupported HVXC payload");
            return false;
        default:
            break;
    }

    ubuf_block_stream_fill_bits(s, 1);
    bool other_data_present = ubuf_block_stream_show_bits(s, 1);
    ubuf_block_stream_skip_bits(s, 1);

    if (other_data_present) {
        if (mux_version) {
            upipe_mpgaf_parse_latm_value(upipe, s);
        } else {
            bool escape;
            do {
                ubuf_block_stream_fill_bits(s, 9);
                escape = ubuf_block_stream_show_bits(s, 1);
                ubuf_block_stream_skip_bits(s, 9);
            } while (escape);
        }
    }

    ubuf_block_stream_fill_bits(s, 1);
    bool crc_check = ubuf_block_stream_show_bits(s, 1);
    ubuf_block_stream_skip_bits(s, 1);

    if (crc_check) {
        ubuf_block_stream_fill_bits(s, 8);
        ubuf_block_stream_skip_bits(s, 8);
    }

    return true;
}

/** @internal @This parses an LATM frame.
 *
 * @param upipe description structure of the pipe
 * @param ubuf ubuf containing frame
 * @return false if the header is invalid or we miss the mux configuration
 */
static bool upipe_mpgaf_handle_latm(struct upipe *upipe, struct ubuf *ubuf,
                                    size_t frame_offset, size_t frame_length)
{
    struct upipe_mpgaf *upipe_mpgaf = upipe_mpgaf_from_upipe(upipe);
    struct ubuf_block_stream s;
    if (!ubase_check(ubuf_block_stream_init(&s, ubuf, frame_offset))) {
        upipe_throw_fatal(upipe, UBASE_ERR_ALLOC);
        return false;
    }

    ubuf_block_stream_fill_bits(&s, 8);
    bool same_stream_mux = ubuf_block_stream_show_bits(&s, 1);
    ubuf_block_stream_skip_bits(&s, 1);

    if (same_stream_mux) {
        if (upipe_mpgaf->flow_def_attr == NULL) {
            upipe_warn(upipe, "discarding payload without configuration");
            return false;
        }
    } else if (!upipe_mpgaf_handle_latm_config(upipe, &s)) {
        ubuf_block_stream_clean(&s);
        return false;
    }

    switch (upipe_mpgaf->frame_length_type) {
        case ASC_FLT_VARIABLE: {
            uint8_t tmp;
            do {
                ubuf_block_stream_fill_bits(&s, 8);
                tmp = ubuf_block_stream_show_bits(&s, 8);
                ubuf_block_stream_skip_bits(&s, 8);
            } while (tmp == 255);
            break;
        }
        case ASC_FLT_CELP_2:
        case ASC_FLT_CELP_4:
        case ASC_FLT_HVXC_4:
            ubuf_block_stream_fill_bits(&s, 2);
            ubuf_block_stream_skip_bits(&s, 2);
            break;
        default:
            break;
    }

    upipe_mpgaf->latm_header_size = ubuf_block_stream_position(&s);
    ubuf_block_stream_clean(&s);

    /* Calculate octetrate. */
    uint64_t octetrate = frame_length * upipe_mpgaf->samplerate /
                         upipe_mpgaf->samples;
    /* Round up to a multiple of 8 kbits/s. */
    octetrate += 999;
    octetrate -= octetrate % 1000;

    if (same_stream_mux && octetrate <= upipe_mpgaf->octetrate)
        return true;

    if (octetrate > upipe_mpgaf->octetrate)
        upipe_mpgaf->octetrate = octetrate;

    struct uref *flow_def = upipe_mpgaf_alloc_flow_def_attr(upipe);
    if (unlikely(flow_def == NULL)) {
        upipe_throw_fatal(upipe, UBASE_ERR_ALLOC);
        return false;
    }

    UBASE_FATAL(upipe, uref_flow_set_complete(flow_def))
    UBASE_FATAL(upipe, uref_flow_set_def(flow_def, "block.aac_latm.sound."))
    UBASE_FATAL(upipe,
            uref_sound_flow_set_channels(flow_def, upipe_mpgaf->channels))
    UBASE_FATAL(upipe,
            uref_mpga_flow_set_encaps(flow_def, UREF_MPGA_ENCAPS_LOAS))
    UBASE_FATAL(upipe,
            uref_sound_flow_set_rate(flow_def, upipe_mpgaf->samplerate))
    UBASE_FATAL(upipe,
            uref_sound_flow_set_samples(flow_def, upipe_mpgaf->samples))
    if (!upipe_mpgaf->complete_input)
        UBASE_FATAL(upipe, uref_clock_set_latency(flow_def,
                    upipe_mpgaf->input_latency +
                    UCLOCK_FREQ * upipe_mpgaf->samples /
                    upipe_mpgaf->samplerate))

    if (likely(upipe_mpgaf->flow_def_attr != NULL)) {
        UBASE_FATAL(upipe,
                uref_block_flow_set_octetrate(flow_def, upipe_mpgaf->octetrate))

        if (likely(!udict_cmp(upipe_mpgaf->flow_def_attr->udict,
                              flow_def->udict))) {
            uref_free(flow_def);
            return true;
        }
    }

    UBASE_FATAL(upipe, uref_block_flow_set_octetrate(flow_def, octetrate))

    upipe_mpgaf_store_flow_def(upipe, NULL);
    uref_free(upipe_mpgaf->flow_def_requested);
    upipe_mpgaf->flow_def_requested = NULL;
    flow_def = upipe_mpgaf_store_flow_def_attr(upipe, flow_def);
    if (flow_def != NULL)
        upipe_mpgaf_require_flow_format(upipe, flow_def);
    return true;
}

/** @internal @This parses an LATM frame with a LOAS header.
 *
 * @param upipe description structure of the pipe
 * @param ubuf ubuf containing frame
 * @return false if the header is invalid or we miss the mux configuration
 */
static bool upipe_mpgaf_handle_loas(struct upipe *upipe, struct ubuf *ubuf)
{
    uint8_t loas_header[LOAS_HEADER_SIZE];
    if (!ubase_check(ubuf_block_extract(ubuf, 0, LOAS_HEADER_SIZE,
                                        loas_header)))
        return false;

    uint16_t frame_length = loas_get_length(loas_header);
    return upipe_mpgaf_handle_latm(upipe, ubuf, LOAS_HEADER_SIZE, frame_length);
}

/** @internal @This checks if there are global headers and uses them.
 *
 * @param upipe description structure of the pipe
 */
static void upipe_mpgaf_handle_global(struct upipe *upipe)
{
    struct upipe_mpgaf *upipe_mpgaf = upipe_mpgaf_from_upipe(upipe);
    upipe_mpgaf_sync_lost(upipe);

    /* set sensible defaults */
    upipe_mpgaf->has_crc = false;
    upipe_mpgaf->asc_aot = upipe_mpgaf->asc_base_aot = ASC_TYPE_LC;
    upipe_mpgaf->samplerate_idx = upipe_mpgaf->base_samplerate_idx = 3;
    upipe_mpgaf->samplerate = upipe_mpgaf->base_samplerate = 48000;
    upipe_mpgaf->samples = ADTS_SAMPLES_PER_BLOCK;
    upipe_mpgaf->channels = 2;

    const uint8_t *p;
    size_t size;
    if (!ubase_check(uref_flow_get_headers(upipe_mpgaf->flow_def_input,
                                           &p, &size)) || size < 2)
        return;

    struct ubuf_block_stream s;
    ubuf_block_stream_init_from_opaque(&s, p, size);
    upipe_mpgaf_handle_asc(upipe, &s);
    ubuf_block_stream_clean(&s);
}

/** @internal @This builds the AudioSpecificConfig sample rate structure.
 *
 * @param bw pointer to ubits structure
 * @param samplerate_idx sample rate index
 * @param samplerate samplerate
 */
static void upipe_mpgaf_build_asc_samplerate(struct ubits *bw,
        uint8_t samplerate_idx, uint64_t samplerate)
{
    ubits_put(bw, 4, samplerate_idx);
    if (samplerate_idx == 0xf)
        ubits_put(bw, 24, samplerate);
}

/** @internal @This builds the global headers of the flow definition packet.
 *
 * @param upipe description structure of the pipe
 * @param bw pointer to ubits structure
 */
static void upipe_mpgaf_build_asc(struct upipe *upipe, struct ubits *bw)
{
    struct upipe_mpgaf *upipe_mpgaf = upipe_mpgaf_from_upipe(upipe);

    /* AudioSpecificConfig */
    ubits_put(bw, 5, upipe_mpgaf->asc_aot);
    upipe_mpgaf_build_asc_samplerate(bw, upipe_mpgaf->base_samplerate_idx,
                                     upipe_mpgaf->base_samplerate);
    ubits_put(bw, 4, upipe_mpgaf->channels);

    if (upipe_mpgaf->asc_aot == ASC_TYPE_SBR ||
        upipe_mpgaf->asc_aot == ASC_TYPE_PS) {
        upipe_mpgaf_build_asc_samplerate(bw, upipe_mpgaf->samplerate_idx,
                                         upipe_mpgaf->samplerate);
        ubits_put(bw, 5, upipe_mpgaf->asc_base_aot);
    }

    /* GASpecificConfig */
    ubits_put(bw, 1, upipe_mpgaf->asc_frame_length ? 1 : 0);
    ubits_put(bw, 1, 0); /* !core coder */
    ubits_put(bw, 1, 0); /* !extension */
}

/** @internal @This builds the global headers of the flow definition packet.
 *
 * @param upipe description structure of the pipe
 * @param flow_def flow definition packet
 */
static void upipe_mpgaf_build_global(struct upipe *upipe, struct uref *flow_def)
{
    struct upipe_mpgaf *upipe_mpgaf = upipe_mpgaf_from_upipe(upipe);
    if (upipe_mpgaf->type != UPIPE_MPGAF_AAC) {
        upipe_warn(upipe, "no global headers for mpga streams");
        return;
    }

    uint8_t headers[MAX_ASC_SIZE];
    struct ubits bw;
    ubits_init(&bw, headers, sizeof(headers), UBITS_WRITE);

    upipe_mpgaf_build_asc(upipe, &bw);

    uint8_t *headers_end;
    int err = ubits_clean(&bw, &headers_end);
    ubase_assert(err);
    UBASE_FATAL(upipe,
            uref_flow_set_headers(flow_def, headers, headers_end - headers))
}

/** @internal @This builds the flow definition packet.
 *
 * @param upipe description structure of the pipe
 */
static void upipe_mpgaf_build_flow_def(struct upipe *upipe)
{
    struct upipe_mpgaf *upipe_mpgaf = upipe_mpgaf_from_upipe(upipe);
    if (upipe_mpgaf->flow_def_requested == NULL)
        return;

    struct uref *flow_def = uref_dup(upipe_mpgaf->flow_def_requested);
    if (unlikely(flow_def == NULL)) {
        upipe_throw_fatal(upipe, UBASE_ERR_ALLOC);
        return;
    }

    const uint8_t *p;
    size_t size;
    if (!ubase_check(uref_flow_get_global(flow_def)))
        uref_flow_delete_headers(flow_def);
    else if (!ubase_check(uref_flow_get_headers(flow_def, &p, &size)))
        upipe_mpgaf_build_global(upipe, flow_def);

    if (upipe_mpgaf->samplerate && !upipe_mpgaf->complete_input) {
        UBASE_FATAL(upipe, uref_clock_set_latency(flow_def,
                    upipe_mpgaf->input_latency +
                    UCLOCK_FREQ * upipe_mpgaf->samples /
                    upipe_mpgaf->samplerate))
    }

    if (upipe_mpgaf->type != UPIPE_MPGAF_MP2) {
        if (upipe_mpgaf->encaps_output == UREF_MPGA_ENCAPS_LOAS)
            UBASE_FATAL(upipe,
                    uref_flow_set_def(flow_def, "block.aac_latm.sound."))
        else
            UBASE_FATAL(upipe,
                    uref_flow_set_def(flow_def, "block.aac.sound."))
    }

    upipe_mpgaf_store_flow_def(upipe, flow_def);
    /* force sending flow definition immediately */
    upipe_mpgaf_output(upipe, NULL, NULL);
}

/** @internal @This handles a frame to prepare for output.
 *
 * @param upipe description structure of the pipe
 * @return pointer to uref
 */
static struct uref *upipe_mpgaf_handle_frame(struct upipe *upipe)
{
    struct upipe_mpgaf *upipe_mpgaf = upipe_mpgaf_from_upipe(upipe);

    struct uref au_uref_s = upipe_mpgaf->au_uref_s;
    struct urational drift_rate = upipe_mpgaf->drift_rate;
    /* From now on, PTS declaration only impacts the next frame. */
    upipe_mpgaf_flush_dates(upipe);

    struct uref *uref = upipe_mpgaf_extract_uref_stream(upipe,
            upipe_mpgaf->next_frame_size);
    if (unlikely(uref == NULL)) {
        upipe_throw_fatal(upipe, UBASE_ERR_ALLOC);
        return NULL;
    }

    if (upipe_mpgaf->encaps_input == UREF_MPGA_ENCAPS_LOAS &&
        !upipe_mpgaf_handle_loas(upipe, uref->ubuf)) {
        uref_free(uref);
        return NULL;
    }

    lldiv_t div = lldiv((uint64_t)upipe_mpgaf->samples * UCLOCK_FREQ +
                        upipe_mpgaf->duration_residue,
                        upipe_mpgaf->samplerate);
    uint64_t duration = div.quot;
    upipe_mpgaf->duration_residue = div.rem;

    /* We work on encoded data so in the DTS domain. Rebase on DTS. */
    uint64_t date;
#define SET_DATE(dv)                                                        \
    if (ubase_check(uref_clock_get_dts_##dv(&au_uref_s, &date))) {          \
        uref_clock_set_dts_##dv(uref, date);                                \
        if (!ubase_check(uref_clock_get_dts_##dv(&upipe_mpgaf->au_uref_s,   \
                                                 NULL)))                    \
            uref_clock_set_dts_##dv(&upipe_mpgaf->au_uref_s,                \
                                    date + duration);                       \
    } else if (ubase_check(uref_clock_get_dts_##dv(uref, &date)))           \
        uref_clock_set_date_##dv(uref, UINT64_MAX, UREF_DATE_NONE);
    SET_DATE(sys)
    SET_DATE(prog)
    SET_DATE(orig)
#undef SET_DATE

    /* PTS = DTS for MPEG audio */
    uref_clock_set_dts_pts_delay(uref, 0);
    if (drift_rate.den)
        uref_clock_set_rate(uref, drift_rate);
    else
        uref_clock_delete_rate(uref);

    UBASE_FATAL(upipe, uref_clock_set_duration(uref, duration))

    if (upipe_mpgaf->got_discontinuity)
        uref_flow_set_discontinuity(uref);
    upipe_mpgaf->got_discontinuity = false;

    return uref;
}

/** @internal @This builds an LATM stream mux.
 *
 * @param upipe description structure of the pipe
 * @param bw pointer to ubits structure
 */
static void upipe_mpgaf_build_latm_config(struct upipe *upipe, struct ubits *bw)
{
    ubits_put(bw, 1, 0); /* !audioMuxVersion */
    ubits_put(bw, 1, 1); /* allStreamsSameTimeFraming */
    ubits_put(bw, 6, 0); /* numSubFrames */
    ubits_put(bw, 4, 0); /* numProgram */
    ubits_put(bw, 3, 0); /* numLayer */

    upipe_mpgaf_build_asc(upipe, bw);

    ubits_put(bw, 3, 0); /* frameLengthType */
    ubits_put(bw, 8, 0xff); /* latmBufferFullness */
    ubits_put(bw, 1, 0); /* otherDataPresent */
    ubits_put(bw, 1, 0); /* crcCheckPresent */
}

/** @internal @This encapsulates a raw frame.
 *
 * @param upipe description structure of the pipe
 * @param uref pointer to uref
 * @return an error code
 */
static int upipe_mpgaf_encaps_frame(struct upipe *upipe, struct uref *uref)
{
    struct upipe_mpgaf *upipe_mpgaf = upipe_mpgaf_from_upipe(upipe);
    if (upipe_mpgaf->encaps_output == UREF_MPGA_ENCAPS_RAW)
        return UBASE_ERR_NONE;

    size_t size = 0;
    uref_block_size(uref, &size);

    if (upipe_mpgaf->encaps_output == UREF_MPGA_ENCAPS_ADTS) {
        uint8_t buffer[ADTS_HEADER_SIZE];
        adts_set_sync(buffer);
        switch (upipe_mpgaf->asc_aot) {
            case ASC_TYPE_MAIN:
                adts_set_profile(buffer, ADTS_PROFILE_MAIN);
                break;
            case ASC_TYPE_SSR:
                adts_set_profile(buffer, ADTS_PROFILE_SSR);
                break;
            default:
                adts_set_profile(buffer, ADTS_PROFILE_LC);
        }
        adts_set_sampling_freq(buffer, upipe_mpgaf->base_samplerate_idx);
        adts_set_channels(buffer, upipe_mpgaf->channels);
        adts_set_length(buffer, size + ADTS_HEADER_SIZE);
        adts_set_num_blocks(buffer, 0);

        struct ubuf *ubuf = ubuf_block_alloc_from_opaque(upipe_mpgaf->ubuf_mgr,
                buffer, ADTS_HEADER_SIZE);
        if (unlikely(ubuf == NULL))
            return UBASE_ERR_ALLOC;

        ubuf_block_append(ubuf, uref_detach_ubuf(uref));
        uref_attach_ubuf(uref, ubuf);
        return UBASE_ERR_NONE;
    }

    /* LOAS/LATM */
    uint64_t duration = 0;
    uref_clock_get_duration(uref, &duration);
    bool config = !upipe_mpgaf->latm_config_duration ||
        (upipe_mpgaf->latm_config_duration + duration) > LATM_CONFIG_PERIOD;
    if (config)
        upipe_mpgaf->latm_config_duration = 0;
    upipe_mpgaf->latm_config_duration += duration;

    int ubuf_size = size + 2 + (config ? MAX_ASC_SIZE + 4 : 0);
    int i;
    for (i = 0; i + 255 <= size; i += 255)
        ubuf_size++;
    if (upipe_mpgaf->encaps_output == UREF_MPGA_ENCAPS_LOAS)
        ubuf_size += LOAS_HEADER_SIZE;
    struct ubuf *ubuf = ubuf_block_alloc(upipe_mpgaf->ubuf_mgr, ubuf_size);
    uint8_t *w;
    if (unlikely(ubuf == NULL ||
                 !ubase_check(ubuf_block_write(ubuf, 0, &ubuf_size, &w)))) {
        ubuf_free(ubuf);
        return UBASE_ERR_ALLOC;
    }

    struct ubits bw;
    ubits_init(&bw, w, ubuf_size, UBITS_WRITE);
    if (upipe_mpgaf->encaps_output == UREF_MPGA_ENCAPS_LOAS)
        ubits_put(&bw, 24, 0);

    ubits_put(&bw, 1, config ? 0 : 1);
    if (config)
        upipe_mpgaf_build_latm_config(upipe, &bw);

    /* PayloadLengthInfo */
    for (i = 0; i + 255 <= size; i += 255)
        ubits_put(&bw, 8, 255);
    ubits_put(&bw, 8, size - i);

    int err = uref_block_extract_bits(uref, 0, size, &bw);
    if (!ubase_check(err)) {
        ubuf_block_unmap(ubuf, 0);
        ubuf_free(ubuf);
        return UBASE_ERR_INVALID;
    }

    uint8_t *end;
    err = ubits_clean(&bw, &end);
    ubase_assert(err);

    if (upipe_mpgaf->encaps_output == UREF_MPGA_ENCAPS_LOAS) {
        if (end - w - LOAS_HEADER_SIZE > 0x1fff)
            upipe_warn_va(upipe, "LATM packet too large (%td)",
                          end - w - LOAS_HEADER_SIZE);

        loas_set_sync(w);
        loas_set_length(w, end - w - LOAS_HEADER_SIZE);
    }
    ubuf_block_unmap(ubuf, 0);
    ubuf_block_resize(ubuf, 0, end - w);

    uref_attach_ubuf(uref, ubuf);
    return UBASE_ERR_NONE;
}

/** @internal @This extracts a LOAS/LATM payload.
 *
 * @param upipe description structure of the pipe
 * @param uref pointer to uref
 * @return ubuf containing payload
 */
static struct ubuf *upipe_mpgaf_extract_latm(struct upipe *upipe,
                                             struct uref *uref)
{
    struct upipe_mpgaf *upipe_mpgaf = upipe_mpgaf_from_upipe(upipe);
    size_t uref_size;
    if (!ubase_check(uref_block_size(uref, &uref_size))) {
        upipe_throw_error(upipe, UBASE_ERR_INVALID);
        return NULL;
    }
    uref_size -= (upipe_mpgaf->latm_header_size + 7) / 8;

    struct ubuf *ubuf = ubuf_block_alloc(upipe_mpgaf->ubuf_mgr, uref_size);
    uint8_t *p;
    int size = uref_size;
    struct ubuf_block_stream s;
    if (unlikely(ubuf == NULL) ||
        !ubase_check(ubuf_block_write(ubuf, 0, &size, &p))) {
        ubuf_free(ubuf);
        upipe_throw_fatal(upipe, UBASE_ERR_ALLOC);
        return NULL;
    }

    if (unlikely(!ubase_check(ubuf_block_stream_init_bits(&s, uref->ubuf,
                              upipe_mpgaf->latm_header_size)))) {
        ubuf_block_unmap(ubuf, 0);
        ubuf_free(ubuf);
        upipe_throw_fatal(upipe, UBASE_ERR_ALLOC);
        return NULL;
    }

    while (uref_size > 0) {
        ubuf_block_stream_fill_bits(&s, 8);
        *p++ = ubuf_block_stream_show_bits(&s, 8);
        ubuf_block_stream_skip_bits(&s, 8);
        uref_size--;
    }

    ubuf_block_stream_clean(&s);
    ubuf_block_unmap(ubuf, 0);
    return ubuf;
}

/** @internal @This decapsulates a frame to raw.
 *
 * @param upipe description structure of the pipe
 * @param uref pointer to uref
 * @return an error code
 */
static int upipe_mpgaf_decaps_frame(struct upipe *upipe, struct uref *uref)
{
    struct upipe_mpgaf *upipe_mpgaf = upipe_mpgaf_from_upipe(upipe);
    if (upipe_mpgaf->encaps_input == UREF_MPGA_ENCAPS_RAW)
        return UBASE_ERR_NONE;

    if (upipe_mpgaf->encaps_input == UREF_MPGA_ENCAPS_ADTS)
        return uref_block_resize(uref, ADTS_HEADER_SIZE +
                (upipe_mpgaf->has_crc ? ADTS_CRC_SIZE : 0), -1);

    struct ubuf *ubuf = upipe_mpgaf_extract_latm(upipe, uref);

    if (unlikely(ubuf == NULL)) {
        uref_free(uref);
        return UBASE_ERR_INVALID;
    }
    uref_attach_ubuf(uref, ubuf);
    return UBASE_ERR_NONE;
}

/** @internal @This outputs a frame.
 *
 * @param upipe description structure of the pipe
 * @param uref pointer to uref
 * @param upump_p reference to pump that generated the buffer
 */
static void upipe_mpgaf_output_frame(struct upipe *upipe, struct uref *uref,
                                     struct upump **upump_p)
{
    struct upipe_mpgaf *upipe_mpgaf = upipe_mpgaf_from_upipe(upipe);
    if (upipe_mpgaf->type != UPIPE_MPGAF_AAC ||
        upipe_mpgaf->encaps_input == upipe_mpgaf->encaps_output) {
        upipe_mpgaf_output(upipe, uref, upump_p);
        return;
    }

    if (!ubase_check(upipe_mpgaf_decaps_frame(upipe, uref))) {
        uref_free(uref);
        return;
    }

    if (!ubase_check(upipe_mpgaf_encaps_frame(upipe, uref))) {
        uref_free(uref);
        return;
    }

    upipe_mpgaf_output(upipe, uref, upump_p);
}

/** @internal @This is called back by @ref upipe_mpgaf_append_uref_stream
 * whenever a new uref is promoted in next_uref.
 *
 * @param upipe description structure of the pipe
 */
static void upipe_mpgaf_promote_uref(struct upipe *upipe)
{
    struct upipe_mpgaf *upipe_mpgaf = upipe_mpgaf_from_upipe(upipe);
    uint64_t date;
#define SET_DATE(dv)                                                        \
    if (ubase_check(uref_clock_get_dts_##dv(upipe_mpgaf->next_uref, &date)))\
        uref_clock_set_dts_##dv(&upipe_mpgaf->au_uref_s, date);
    SET_DATE(sys)
    SET_DATE(prog)
    SET_DATE(orig)
#undef SET_DATE

    uref_clock_get_rate(upipe_mpgaf->next_uref, &upipe_mpgaf->drift_rate);
    upipe_mpgaf->duration_residue = 0;
}

/** @internal @This scans for a sync word.
 *
 * @param upipe description structure of the pipe
 * @param dropped_p filled with the number of octets to drop before the sync
 * @return true if a sync word was found
 */
static bool upipe_mpgaf_scan(struct upipe *upipe, size_t *dropped_p)
{
    struct upipe_mpgaf *upipe_mpgaf = upipe_mpgaf_from_upipe(upipe);
    uint8_t sync = upipe_mpgaf->encaps_input == UREF_MPGA_ENCAPS_LOAS ?
                   0x56 : 0xff;
    while (ubase_check(uref_block_scan(upipe_mpgaf->next_uref, dropped_p,
                                       sync))) {
        uint8_t word;
        if (!ubase_check(uref_block_extract(upipe_mpgaf->next_uref,
                         *dropped_p + 1, 1, &word)))
            return false;

        if ((word & 0xe0) == 0xe0)
            return true;
        (*dropped_p)++;
    }
    return false;
}

/** @internal @This checks if a sync word begins just after the end of the
 * next frame.
 *
 * @param upipe description structure of the pipe
 * @param ready_p filled with true if a sync word was found
 * @param false if no sync word was found and resync is needed
 */
static bool upipe_mpgaf_check_frame(struct upipe *upipe, bool *ready_p)
{
    struct upipe_mpgaf *upipe_mpgaf = upipe_mpgaf_from_upipe(upipe);
    uint8_t sync = upipe_mpgaf->encaps_input == UREF_MPGA_ENCAPS_LOAS ?
                   0x56 : 0xff;
    size_t size;
    *ready_p = false;
    if (!ubase_check(uref_block_size(upipe_mpgaf->next_uref, &size)))
        return false;
    if (size < upipe_mpgaf->next_frame_size)
        return true;

    uint8_t words[2];
    if (!ubase_check(uref_block_extract(upipe_mpgaf->next_uref,
                            upipe_mpgaf->next_frame_size, 2, words))) {
        /* not enough data */
        if (upipe_mpgaf->acquired || upipe_mpgaf->complete_input)
            /* avoid delaying packets unnecessarily */
            *ready_p = true;
        return true;
    }
    if (words[0] != sync || (words[1] & 0xe0) != 0xe0)
        return false;
    *ready_p = true;
    return true;
}

/** @internal @This tries to output frames from the queue of input buffers.
 *
 * @param upipe description structure of the pipe
 * @param upump_p reference to pump that generated the buffer
 */
static void upipe_mpgaf_work(struct upipe *upipe, struct upump **upump_p)
{
    struct upipe_mpgaf *upipe_mpgaf = upipe_mpgaf_from_upipe(upipe);
    while (upipe_mpgaf->next_uref != NULL) {
        if (unlikely(!upipe_mpgaf->acquired)) {
            size_t dropped = 0;
            bool ret = upipe_mpgaf_scan(upipe, &dropped);
            upipe_mpgaf_consume_uref_stream(upipe, dropped);
            if (!ret)
                return;
        }
        if (upipe_mpgaf->next_frame_size == -1 &&
            !upipe_mpgaf_parse_header(upipe)) {
            upipe_warn(upipe, "invalid header");
            upipe_mpgaf_consume_uref_stream(upipe, 1);
            upipe_mpgaf_sync_lost(upipe);
            continue;
        }
        if (upipe_mpgaf->next_frame_size == -1)
            return; /* not enough data */

        if (!upipe_mpgaf->next_frame_size) {
            /* MPEG free bitrate mode, we have to scan */
            size_t next_frame_size = MPGA_HEADER_SIZE;
            if (!upipe_mpgaf_scan(upipe, &next_frame_size))
                return; /* not enough data */
            upipe_mpgaf->next_frame_size = next_frame_size;
        } else {
            bool ready;
            if (unlikely(!upipe_mpgaf_check_frame(upipe, &ready))) {
                upipe_warn(upipe, "invalid frame");
                upipe_mpgaf_consume_uref_stream(upipe, 1);
                upipe_mpgaf->next_frame_size = -1;
                upipe_mpgaf_sync_lost(upipe);
                continue;
            }
            if (!ready)
                return; /* not enough data */
        }

        upipe_mpgaf_sync_acquired(upipe);
        struct uref *uref = upipe_mpgaf_handle_frame(upipe);
        upipe_mpgaf->next_frame_size = -1;

        if (unlikely(upipe_mpgaf->flow_def_requested == NULL)) {
            upipe_mpgaf->uref_output = uref;
            return;
        }

        upipe_mpgaf_output_frame(upipe, uref, upump_p);
    }
}

/** @internal @This works on incoming raw frames (supposedly one frame per uref
 * guaranteed by demux).
 *
 * @param upipe description structure of the pipe
 * @param uref uref structure containing one frame
 * @param upump_p reference to pump that generated the buffer
 * @return true if the packet was handled
 */
static bool upipe_mpgaf_work_raw(struct upipe *upipe, struct uref *uref,
                                 struct upump **upump_p)
{
    struct upipe_mpgaf *upipe_mpgaf = upipe_mpgaf_from_upipe(upipe);
    size_t size;
    if (unlikely(!ubase_check(uref_block_size(uref, &size)) || !size)) {
        upipe_throw_error(upipe, UBASE_ERR_INVALID);
        uref_free(uref);
        return true;
    }

    if (upipe_mpgaf->encaps_input == UREF_MPGA_ENCAPS_LATM &&
        !upipe_mpgaf_handle_latm(upipe, uref->ubuf, 0, size))
        return true;

    /* Calculate octetrate assuming the stream is CBR. */
    uint64_t octetrate = size * upipe_mpgaf->samplerate /
        upipe_mpgaf->samples;

    if (unlikely(!upipe_mpgaf->acquired || octetrate > upipe_mpgaf->octetrate)) {
        upipe_mpgaf_sync_acquired(upipe);
        struct uref *flow_def = upipe_mpgaf_alloc_flow_def_attr(upipe);
        if (unlikely(flow_def == NULL)) {
            upipe_throw_fatal(upipe, UBASE_ERR_ALLOC);
            return true;
        }

        UBASE_FATAL(upipe, uref_flow_set_def(flow_def, "block.aac.sound."))
        if (upipe_mpgaf->channels) {
            UBASE_FATAL(upipe, uref_sound_flow_set_channels(flow_def, upipe_mpgaf->channels))
        }
        UBASE_FATAL(upipe,
                uref_mpga_flow_set_encaps(flow_def, upipe_mpgaf->encaps_input))
        UBASE_FATAL(upipe,
                uref_sound_flow_set_rate(flow_def, upipe_mpgaf->samplerate))
        UBASE_FATAL(upipe,
                uref_sound_flow_set_samples(flow_def, upipe_mpgaf->samples))

        /* Round up to a multiple of 8 kbits/s. */
        octetrate += 999;
        octetrate -= octetrate % 1000;
        upipe_mpgaf->octetrate = octetrate;
        UBASE_FATAL(upipe, uref_block_flow_set_octetrate(flow_def, octetrate))

        upipe_mpgaf_store_flow_def(upipe, NULL);
        uref_free(upipe_mpgaf->flow_def_requested);
        upipe_mpgaf->flow_def_requested = NULL;
        flow_def = upipe_mpgaf_store_flow_def_attr(upipe, flow_def);
        if (flow_def != NULL)
            upipe_mpgaf_require_flow_format(upipe, flow_def);
    }

    if (unlikely(upipe_mpgaf->flow_def_requested == NULL))
        return false;

    lldiv_t div = lldiv((uint64_t)upipe_mpgaf->samples * UCLOCK_FREQ +
                        upipe_mpgaf->duration_residue,
                        upipe_mpgaf->samplerate);
    uint64_t duration = div.quot;
    upipe_mpgaf->duration_residue = div.rem;

    /* We work on encoded data so in the DTS domain. Rebase on DTS. */
    uint64_t date;
#define SET_DATE(dv)                                                        \
    if (ubase_check(uref_clock_get_dts_##dv(uref, &date)))                  \
        uref_clock_set_dts_##dv(&upipe_mpgaf->au_uref_s, date + duration);  \
    else if (ubase_check(uref_clock_get_dts_##dv(&upipe_mpgaf->au_uref_s,   \
                                                 &date))) {                 \
        uref_clock_set_dts_##dv(uref, date);                                \
        uref_clock_set_dts_##dv(&upipe_mpgaf->au_uref_s, date + duration);  \
    }
    SET_DATE(sys)
    SET_DATE(prog)
    SET_DATE(orig)
#undef SET_DATE

    /* PTS = DTS for MPEG audio */
    uref_clock_set_dts_pts_delay(uref, 0);

    UBASE_FATAL(upipe, uref_clock_set_duration(uref, duration))

    upipe_mpgaf_output_frame(upipe, uref, upump_p);
    return true;
}

/** @internal @This receives data.
 *
 * @param upipe description structure of the pipe
 * @param uref uref structure
 * @param upump_p reference to pump that generated the buffer
 * @return true if the packet was handled
 */
static bool upipe_mpgaf_handle(struct upipe *upipe, struct uref *uref,
                               struct upump **upump_p)
{
    struct upipe_mpgaf *upipe_mpgaf = upipe_mpgaf_from_upipe(upipe);
    const char *def;
    if (unlikely(ubase_check(uref_flow_get_def(uref, &def)))) {
        upipe_mpgaf->input_latency = 0;
        uref_clock_get_latency(uref, &upipe_mpgaf->input_latency);
        upipe_mpgaf->encaps_input = uref_mpga_flow_infer_encaps(uref);
        upipe_mpgaf->type = UPIPE_MPGAF_UNKNOWN;
        if (!ubase_ncmp(def, "block.aac_latm.") ||
            upipe_mpgaf->encaps_input == UREF_MPGA_ENCAPS_LOAS)
            upipe_mpgaf->type = UPIPE_MPGAF_AAC;
        else if (!ubase_ncmp(def, "block.mp2.") ||
                 !ubase_ncmp(def, "block.mp3."))
            upipe_mpgaf->type = UPIPE_MPGAF_MP2;
        else if (!ubase_ncmp(def, "block.aac."))
            upipe_mpgaf->type = UPIPE_MPGAF_AAC;
        upipe_mpgaf->complete_input = ubase_check(uref_flow_get_complete(uref));
        upipe_mpgaf_store_flow_def(upipe, NULL);
        uref_free(upipe_mpgaf->flow_def_requested);
        upipe_mpgaf->flow_def_requested = NULL;
        uref = upipe_mpgaf_store_flow_def_input(upipe, uref);
        if (uref != NULL)
            upipe_mpgaf_require_flow_format(upipe, uref);
        if (upipe_mpgaf->type == UPIPE_MPGAF_AAC &&
            (upipe_mpgaf->encaps_input == UREF_MPGA_ENCAPS_RAW ||
             upipe_mpgaf->encaps_input == UREF_MPGA_ENCAPS_LATM))
            upipe_mpgaf_handle_global(upipe);
        return true;
    }

    if (upipe_mpgaf->flow_def_requested == NULL &&
        upipe_mpgaf->flow_def_attr != NULL)
        return false;

    if (upipe_mpgaf->flow_def_input == NULL) {
        upipe_throw_error(upipe, UBASE_ERR_INVALID);
        uref_free(uref);
        return true;
    }

    if (upipe_mpgaf->type == UPIPE_MPGAF_AAC) {
        switch (upipe_mpgaf->encaps_input) {
            case UREF_MPGA_ENCAPS_RAW:
            case UREF_MPGA_ENCAPS_LATM:
                return upipe_mpgaf_work_raw(upipe, uref, upump_p);
            default:
                break;
        }
    }

    if (unlikely(ubase_check(uref_flow_get_discontinuity(uref)))) {
        /* Drop the current frame and resync. */
        upipe_mpgaf_clean_uref_stream(upipe);
        upipe_mpgaf_init_uref_stream(upipe);
        upipe_mpgaf->got_discontinuity = true;
        upipe_mpgaf->next_frame_size = -1;
        upipe_mpgaf_sync_lost(upipe);
    }

    upipe_mpgaf_append_uref_stream(upipe, uref);
    upipe_mpgaf_work(upipe, upump_p);
    return true;
}

/** @internal @This inputs data.
 *
 * @param upipe description structure of the pipe
 * @param uref uref structure
 * @param upump_p reference to pump that generated the buffer
 */
static void upipe_mpgaf_input(struct upipe *upipe, struct uref *uref,
                              struct upump **upump_p)
{
    if (!upipe_mpgaf_check_input(upipe)) {
        upipe_mpgaf_hold_input(upipe, uref);
        upipe_mpgaf_block_input(upipe, upump_p);
    } else if (!upipe_mpgaf_handle(upipe, uref, upump_p)) {
        upipe_mpgaf_hold_input(upipe, uref);
        upipe_mpgaf_block_input(upipe, upump_p);
        /* Increment upipe refcount to avoid disappearing before all packets
         * have been sent. */
        upipe_use(upipe);
    }
}

/** @internal @This receives the result of a flow format request.
 *
 * @param upipe description structure of the pipe
 * @param flow_format amended flow format
 * @return an error code
 */
static int upipe_mpgaf_check_flow_format(struct upipe *upipe,
                                         struct uref *flow_format)
{
    struct upipe_mpgaf *upipe_mpgaf = upipe_mpgaf_from_upipe(upipe);
    if (flow_format == NULL)
        return UBASE_ERR_INVALID;

    uref_free(upipe_mpgaf->flow_def_requested);
    upipe_mpgaf->flow_def_requested = NULL;

    upipe_mpgaf_require_ubuf_mgr(upipe, flow_format);
    return UBASE_ERR_NONE;
}

/** @internal @This receives the result of a ubuf manager request.
 *
 * @param upipe description structure of the pipe
 * @param flow_format amended flow format
 * @return an error code
 */
static int upipe_mpgaf_check_ubuf_mgr(struct upipe *upipe,
                                      struct uref *flow_format)
{
    struct upipe_mpgaf *upipe_mpgaf = upipe_mpgaf_from_upipe(upipe);
    if (flow_format == NULL)
        return UBASE_ERR_INVALID;
    if (upipe_mpgaf->flow_def_attr == NULL) {
        /* temporary ubuf manager, will be overwritten later */
        uref_free(flow_format);
        return UBASE_ERR_NONE;
    }

    uref_free(upipe_mpgaf->flow_def_requested);
    upipe_mpgaf->flow_def_requested = flow_format;
    upipe_mpgaf->encaps_output = uref_mpga_flow_infer_encaps(flow_format);

    upipe_mpgaf_build_flow_def(upipe);

    if (upipe_mpgaf->uref_output) {
        upipe_mpgaf_output_frame(upipe, upipe_mpgaf->uref_output, NULL);
        upipe_mpgaf->uref_output = NULL;
    }

    bool was_buffered = !upipe_mpgaf_check_input(upipe);
    upipe_mpgaf_output_input(upipe);
    upipe_mpgaf_unblock_input(upipe);
    if (was_buffered && upipe_mpgaf_check_input(upipe)) {
        /* All packets have been output, release again the pipe that has been
         * used in @ref upipe_mpgaf_input. */
        upipe_release(upipe);
    }
    return UBASE_ERR_NONE;
}

/** @internal @This sets the input flow definition.
 *
 * @param upipe description structure of the pipe
 * @param flow_def flow definition packet
 * @return an error code
 */
static int upipe_mpgaf_set_flow_def(struct upipe *upipe, struct uref *flow_def)
{
    if (flow_def == NULL)
        return UBASE_ERR_INVALID;
    const char *def;
    if (unlikely(!ubase_check(uref_flow_get_def(flow_def, &def)) ||
                 (ubase_ncmp(def, "block.mp2.") &&
                  ubase_ncmp(def, "block.mp3.") &&
                  ubase_ncmp(def, "block.aac.") &&
                  ubase_ncmp(def, "block.aac_latm.") &&
                  strcmp(def, "block."))))
        return UBASE_ERR_INVALID;
    struct uref *flow_def_dup;
    if (unlikely((flow_def_dup = uref_dup(flow_def)) == NULL)) {
        upipe_throw_fatal(upipe, UBASE_ERR_ALLOC);
        return UBASE_ERR_ALLOC;
    }
    upipe_input(upipe, flow_def_dup, NULL);
    return UBASE_ERR_NONE;
}

/** @internal @This processes control commands on a mpgaf pipe.
 *
 * @param upipe description structure of the pipe
 * @param command type of command to process
 * @param args arguments of the command
 * @return an error code
 */
static int upipe_mpgaf_control(struct upipe *upipe, int command, va_list args)
{
    UBASE_HANDLED_RETURN(upipe_mpgaf_control_output(upipe, command, args));
    switch (command) {
        case UPIPE_SET_FLOW_DEF: {
            struct uref *flow_def = va_arg(args, struct uref *);
            return upipe_mpgaf_set_flow_def(upipe, flow_def);
        }
        default:
            return UBASE_ERR_UNHANDLED;
    }
}

/** @This frees a upipe.
 *
 * @param upipe description structure of the pipe
 */
static void upipe_mpgaf_free(struct upipe *upipe)
{
    struct upipe_mpgaf *upipe_mpgaf = upipe_mpgaf_from_upipe(upipe);
    upipe_throw_dead(upipe);

    upipe_mpgaf_clean_uref_stream(upipe);
    upipe_mpgaf_clean_input(upipe);
    upipe_mpgaf_clean_output(upipe);
    uref_free(upipe_mpgaf->flow_def_requested);
    uref_free(upipe_mpgaf->uref_output);
    upipe_mpgaf_clean_flow_format(upipe);
    upipe_mpgaf_clean_flow_def(upipe);
    upipe_mpgaf_clean_ubuf_mgr(upipe);
    upipe_mpgaf_clean_sync(upipe);

    upipe_mpgaf_clean_urefcount(upipe);
    upipe_mpgaf_free_void(upipe);
}

/** module manager static descriptor */
static struct upipe_mgr upipe_mpgaf_mgr = {
    .refcount = NULL,
    .signature = UPIPE_MPGAF_SIGNATURE,

    .upipe_alloc = upipe_mpgaf_alloc,
    .upipe_input = upipe_mpgaf_input,
    .upipe_control = upipe_mpgaf_control,

    .upipe_mgr_control = NULL
};

/** @This returns the management structure for all mpgaf pipes.
 *
 * @return pointer to manager
 */
struct upipe_mgr *upipe_mpgaf_mgr_alloc(void)
{
    return &upipe_mpgaf_mgr;
}
