/*
 * Copyright (C) 2013-2016 OpenHeadend S.A.R.L.
 *
 * Authors: Benjamin Cohen
 *          Christophe Massiot
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston MA 02110-1301, USA.
 */

/** @file
 * @short Upipe source module for BlackMagic Design SDI cards
 */

#define __STDC_LIMIT_MACROS    1
#define __STDC_FORMAT_MACROS   1
#define __STDC_CONSTANT_MACROS 1

#include <upipe/ubase.h>
#include <upipe/uatomic.h>
#include <upipe/uprobe.h>
#include <upipe/uclock.h>
#include <upipe/uref.h>
#include <upipe/uref_pic.h>
#include <upipe/uref_pic_flow.h>
#include <upipe/uref_sound.h>
#include <upipe/uref_sound_flow.h>
#include <upipe/uref_clock.h>
#include <upipe/upump.h>
#include <upipe/ubuf.h>
#include <upipe/ufifo.h>
#include <upipe/uqueue.h>
#include <upipe/upipe.h>
#include <upipe/upipe_helper_upipe.h>
#include <upipe/upipe_helper_urefcount.h>
#include <upipe/upipe_helper_void.h>
#include <upipe/upipe_helper_flow.h>
#include <upipe/upipe_helper_sync.h>
#include <upipe/upipe_helper_uref_mgr.h>
#include <upipe/upipe_helper_output.h>
#include <upipe/upipe_helper_upump_mgr.h>
#include <upipe/upipe_helper_upump.h>
#include <upipe/upipe_helper_uclock.h>
#include <upipe-blackmagic/upipe_blackmagic_source.h>
#include <upipe-blackmagic/ubuf_pic_blackmagic.h>
#include <upipe-blackmagic/ubuf_sound_blackmagic.h>

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <assert.h>
#include <pthread.h>

#include "include/DeckLinkAPI.h"

/** uqueue length */
#define MAX_QUEUE_LENGTH 255
/** ubuf pool depth */
#define UBUF_POOL_DEPTH 25
/** lowest possible prog PTS (just an arbitrarily high time) */
#define BMD_CLOCK_MIN UINT32_MAX
/** fixed sample rate */
#define BMD_SAMPLERATE 48000
/** fixed channels number */
#define BMD_CHANNELS 16
/** blackmagic uri separator */
#define URI_SEP "://"

const static struct {
    const char *name;
    BMDVideoConnection bmdConn;
} upipe_bmd_src_video_conns[] = {
    {"sdi",         bmdVideoConnectionSDI},
    {"hdmi",        bmdVideoConnectionHDMI},
    {"opticalsdi",  bmdVideoConnectionOpticalSDI},
    {"component",   bmdVideoConnectionComponent},
    {"composite",   bmdVideoConnectionComposite},
    {"svideo",      bmdVideoConnectionSVideo},

    {NULL, 0}
};

const static struct {
    const char *name;
    BMDAudioConnection bmdConn;
} upipe_bmd_src_audio_conns[] = {
    {"embedded",    bmdAudioConnectionEmbedded},
    {"aesebu",      bmdAudioConnectionAESEBU},
    {"analog",      bmdAudioConnectionAnalog},
    {"analogxlr",   bmdAudioConnectionAnalogXLR},
    {"analogrca",   bmdAudioConnectionAnalogRCA},
    {"microphone",  bmdAudioConnectionMicrophone},
    {"headphones",  bmdAudioConnectionHeadphones},

    {NULL, 0}
};

const static struct {
    const char *name;
    BMDDisplayMode mode;
} upipe_bmd_src_display_modes[] = {
    /* SD modes */
    {"ntsc",        bmdModeNTSC},
    {"ntsc2398",    bmdModeNTSC2398},
    {"pal",         bmdModePAL},
    {"ntscp",       bmdModeNTSCp},
    {"palp",        bmdModePALp},

    /* HD 1080 modes */
    {"1080p2398",   bmdModeHD1080p2398},
    {"1080p24",     bmdModeHD1080p24},
    {"1080p25",     bmdModeHD1080p25},
    {"1080p2997",   bmdModeHD1080p2997},
    {"1080p30",     bmdModeHD1080p30},
    {"1080i50",     bmdModeHD1080i50},
    {"1080i5994",   bmdModeHD1080i5994},
    {"1080i6000",   bmdModeHD1080i6000},
    {"1080p50",     bmdModeHD1080p50},
    {"1080p5994",   bmdModeHD1080p5994},
    {"1080p6000",   bmdModeHD1080p6000},

    /* HD 720 modes */
    {"720p50",      bmdModeHD720p50},
    {"720p5994",    bmdModeHD720p5994},
    {"720p60",      bmdModeHD720p60},

    /* 2k modes */
    {"2k2398",      bmdMode2k2398},
    {"2k24",        bmdMode2k24},
    {"2k25",        bmdMode2k25},

    /* 4k modes */
    {"2160p2398",   bmdMode4K2160p2398},
    {"2160p24",     bmdMode4K2160p24},
    {"2160p25",     bmdMode4K2160p25},
    {"2160p2997",   bmdMode4K2160p2997},
    {"2160p30",     bmdMode4K2160p30},
    {"2160p50",     bmdMode4K2160p50},
    {"2160p5994",   bmdMode4K2160p5994},
    {"2160p60",     bmdMode4K2160p60},

    {NULL, 0}
};

/** @internal @This is the class that retrieves frames in a private thread. */
class DeckLinkCaptureDelegate : public IDeckLinkInputCallback
{
public:
    DeckLinkCaptureDelegate(struct upipe *_upipe) : upipe(_upipe) {
        uatomic_store(&refcount, 1);
    }

    ~DeckLinkCaptureDelegate(void) {
        uatomic_clean(&refcount);
    }

    virtual ULONG STDMETHODCALLTYPE AddRef(void) {
        return uatomic_fetch_add(&refcount, 1) + 1;
    }

    virtual ULONG STDMETHODCALLTYPE Release(void) {
        uint32_t new_ref = uatomic_fetch_sub(&refcount, 1) - 1;
        if (new_ref == 0)
            delete this;
        return new_ref;
    }

    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID iid, LPVOID *ppv) {
        return E_NOINTERFACE;
    }

    virtual HRESULT STDMETHODCALLTYPE VideoInputFormatChanged(
                BMDVideoInputFormatChangedEvents, IDeckLinkDisplayMode*,
                BMDDetectedVideoInputFormatFlags);

    virtual HRESULT STDMETHODCALLTYPE VideoInputFrameArrived(
                IDeckLinkVideoInputFrame*, IDeckLinkAudioInputPacket*);

private:
    uatomic_uint32_t refcount;

    struct upipe *upipe;
};

/** @internal packet type */
enum upipe_bmd_src_type {
    /** packet for pic subpipe */
    UPIPE_BMD_SRC_PIC,
    /** packet for pic subpipe, without sync on input */
    UPIPE_BMD_SRC_PIC_NO_INPUT,
    /** packet for sound subpipe */
    UPIPE_BMD_SRC_SOUND
};

/** @internal @This is the private context of an output of a bmdsrc pipe */
struct upipe_bmd_src_output {
    /** ubuf manager */
    struct ubuf_mgr *ubuf_mgr;

    /** pipe acting as output */
    struct upipe *output;
    /** flow definition packet */
    struct uref *flow_def;
    /** output state */
    enum upipe_helper_output_state output_state;
    /** list of output requests */
    struct uchain request_list;

    /** public upipe structure */
    struct upipe upipe;
};

UPIPE_HELPER_UPIPE(upipe_bmd_src_output, upipe, UPIPE_BMD_SRC_OUTPUT_SIGNATURE)
UPIPE_HELPER_OUTPUT(upipe_bmd_src_output, output, flow_def, output_state,
                    request_list)

/** @internal @This is the private context of a bmdsrc pipe. */
struct upipe_bmd_src {
    /** refcount management structure */
    struct urefcount urefcount;

    /** upump manager */
    struct upump_mgr *upump_mgr;
    /** pump */
    struct upump *upump;

    /** uref manager */
    struct uref_mgr *uref_mgr;
    /** uref manager request */
    struct urequest uref_mgr_request;

    /** uclock structure */
    struct uclock *uclock;
    /** uclock request */
    struct urequest uclock_request;

    /** pseudo-output */
    struct upipe *output;
    /** subpipe manager */
    struct upipe_mgr sub_mgr;
    /** pic subpipe */
    struct upipe_bmd_src_output pic_subpipe;
    /** sound subpipe */
    struct upipe_bmd_src_output sound_subpipe;

    /** URI */
    char *uri;
    /** queue between blackmagic thread and pipe thread */
    struct uqueue uqueue;
    /** handle to decklink card */
    IDeckLink *deckLink;
    /** handle to decklink card input */
    IDeckLinkInput *deckLinkInput;
    /** handle to decklink configuration */
    IDeckLinkConfiguration *deckLinkConfiguration;
    /** handle to decklink delegate */
    DeckLinkCaptureDelegate *deckLinkCaptureDelegate;
    /** pixel format */
    BMDPixelFormat pixel_format;
    /** yuv pixel format (UYVY or v210) */
    BMDPixelFormat yuv_pixel_format;
    /** offset between bmd timestamps and Upipe timestamps */
    int64_t timestamp_offset;
    /** highest Upipe timestamp given to a frame */
    uint64_t timestamp_highest;
    /** last cr sys */
    uint64_t last_cr_sys;
    /** current frame rate */
    struct urational fps;
    /** true for progressive frames - for use by the private thread */
    bool progressive;
    /** true for top field first - for use by the private thread */
    bool tff;
    /** true if we have thrown the sync_acquired event */
    bool acquired;
    /** had input */
    bool had_input;

    /** public upipe structure */
    struct upipe upipe;

    /** extra data for the queue structures */
    uint8_t uqueue_extra[];
};

UPIPE_HELPER_UPIPE(upipe_bmd_src, upipe, UPIPE_BMD_SRC_SIGNATURE)
UPIPE_HELPER_UREFCOUNT(upipe_bmd_src, urefcount, upipe_bmd_src_free)
UPIPE_HELPER_SYNC(upipe_bmd_src, acquired)
UPIPE_HELPER_UREF_MGR(upipe_bmd_src, uref_mgr, uref_mgr_request, NULL,
                      upipe_throw_provide_request, NULL)
UPIPE_HELPER_UCLOCK(upipe_bmd_src, uclock, uclock_request, NULL,
                    upipe_throw_provide_request, NULL)

UPIPE_HELPER_UPUMP_MGR(upipe_bmd_src, upump_mgr)
UPIPE_HELPER_UPUMP(upipe_bmd_src, upump, upump_mgr)

UBASE_FROM_TO(upipe_bmd_src, upipe_mgr, sub_mgr, sub_mgr)
UBASE_FROM_TO(upipe_bmd_src, upipe_bmd_src_output, pic_subpipe, pic_subpipe)
UBASE_FROM_TO(upipe_bmd_src, upipe_bmd_src_output, sound_subpipe, sound_subpipe)

/** @internal @This prepares the pipe for a new video configuration.
 *
 * @param upipe super-pipe structure
 * @param mode decklink display mode
 * @return an error code
 */
static int upipe_bmd_src_build_video(struct upipe *upipe,
                                     IDeckLinkDisplayMode *mode)
{
    struct upipe_bmd_src *upipe_bmd_src = upipe_bmd_src_from_upipe(upipe);
    if (upipe_bmd_src->pic_subpipe.ubuf_mgr == NULL)
        upipe_bmd_src->pic_subpipe.ubuf_mgr =
            ubuf_pic_bmd_mgr_alloc(UBUF_POOL_DEPTH,
                                   upipe_bmd_src->pixel_format);
    if (upipe_bmd_src->pic_subpipe.ubuf_mgr == NULL)
        return UBASE_ERR_ALLOC;

    struct uref *flow_def;
    if (upipe_bmd_src->pixel_format == bmdFormat8BitYUV) {
        flow_def = uref_pic_flow_alloc_def(upipe_bmd_src->uref_mgr, 2);
        uref_pic_flow_add_plane(flow_def, 1, 1, 4, "u8y8v8y8");
    } else if (upipe_bmd_src->pixel_format == bmdFormat10BitYUV) {
        flow_def = uref_pic_flow_alloc_def(upipe_bmd_src->uref_mgr, 6);
        uref_pic_flow_add_plane(flow_def, 1, 1, 16,
                "u10y10v10y10u10y10v10y10u10y10v10y10");
    } else {
        flow_def = uref_pic_flow_alloc_def(upipe_bmd_src->uref_mgr, 1);
        uref_pic_flow_add_plane(flow_def, 1, 1, 4, "a8r8g8b8");
    }

    UBASE_RETURN(uref_pic_flow_set_hsize(flow_def, mode->GetWidth()));
    UBASE_RETURN(uref_pic_flow_set_vsize(flow_def, mode->GetHeight()));

    /* This is supposed to be fixed later by user or ancillary data */
    struct urational sar;
    sar.num = 16 * mode->GetHeight();
    sar.den =  9 * mode->GetWidth();
    urational_simplify(&sar);
    UBASE_RETURN(uref_pic_flow_set_sar(flow_def, sar));

    struct urational fps;
    BMDTimeValue frameDuration;
    BMDTimeScale timeScale;
    mode->GetFrameRate(&frameDuration, &timeScale);
    fps.num = timeScale;
    fps.den = frameDuration;
    urational_simplify(&fps);
    UBASE_RETURN(uref_pic_flow_set_fps(flow_def, fps));

    BMDFieldDominance field = mode->GetFieldDominance();
    switch (field) {
        case bmdLowerFieldFirst:
            uref_pic_delete_tff(flow_def);
            uref_pic_delete_progressive(flow_def);
            upipe_bmd_src->tff = false;
            upipe_bmd_src->progressive = false;
            break;
        default:
        case bmdUnknownFieldDominance:
            /* sensible defaults */
        case bmdUpperFieldFirst:
            UBASE_RETURN(uref_pic_set_tff(flow_def));
            uref_pic_delete_progressive(flow_def);
            upipe_bmd_src->tff = true;
            upipe_bmd_src->progressive = false;
            break;
        case bmdProgressiveFrame:
        case bmdProgressiveSegmentedFrame:
            uref_pic_delete_tff(flow_def);
            UBASE_RETURN(uref_pic_set_progressive(flow_def));
            upipe_bmd_src->tff = false;
            upipe_bmd_src->progressive = true;
            break;
    }

    uref_attr_set_priv(flow_def, UPIPE_BMD_SRC_PIC);

    if (unlikely(!uqueue_push(&upipe_bmd_src->uqueue, flow_def)))
        uref_free(flow_def);
    return UBASE_ERR_NONE;
}

/** @internal @This prepares the pipe for a new audio configuration.
 *
 * @param upipe super-pipe structure
 * @param sample_format 16 or 32 bits audio sample depth
 * @return an error code
 */
static int upipe_bmd_src_build_audio(struct upipe *upipe,
        BMDAudioSampleType sample_format)
{
    struct upipe_bmd_src *upipe_bmd_src = upipe_bmd_src_from_upipe(upipe);
    if (upipe_bmd_src->sound_subpipe.ubuf_mgr == NULL)
        upipe_bmd_src->sound_subpipe.ubuf_mgr =
        ubuf_sound_bmd_mgr_alloc(UBUF_POOL_DEPTH,
                sample_format, BMD_CHANNELS, "ALL");
    if (upipe_bmd_src->sound_subpipe.ubuf_mgr == NULL)
        return UBASE_ERR_ALLOC;

    struct uref *flow_def;
    if (sample_format == bmdAudioSampleType16bitInteger) {
        flow_def = uref_sound_flow_alloc_def(upipe_bmd_src->uref_mgr,
                "s16.", BMD_CHANNELS, sizeof(int16_t) * BMD_CHANNELS);
    } else {
        flow_def = uref_sound_flow_alloc_def(upipe_bmd_src->uref_mgr,
                "s32.", BMD_CHANNELS, sizeof(int32_t) * BMD_CHANNELS);
    }
    uref_sound_flow_add_plane(flow_def, "ALL");
    uref_sound_flow_set_rate(flow_def, BMD_SAMPLERATE);

    uref_attr_set_priv(flow_def, UPIPE_BMD_SRC_SOUND);

    if (unlikely(!uqueue_push(&upipe_bmd_src->uqueue, flow_def)))
        uref_free(flow_def);
    return UBASE_ERR_NONE;
}

/** @internal @This is called when receiving a video or audio frame.
 *
 * @param VideoFrame video frame
 * @param AudioPacket audio packet
 * @return S_OK
 */
HRESULT DeckLinkCaptureDelegate::VideoInputFormatChanged(
                BMDVideoInputFormatChangedEvents events,
                IDeckLinkDisplayMode *mode,
                BMDDetectedVideoInputFormatFlags flags)
{
    struct upipe_bmd_src *upipe_bmd_src = upipe_bmd_src_from_upipe(upipe);
    /* Assumes default format is YUV, check bmdDetectedVideoInputYCbCr422? */
    BMDPixelFormat pixel_format = upipe_bmd_src->yuv_pixel_format;
    if (events & bmdVideoInputColorspaceChanged) {
        if (flags & bmdDetectedVideoInputRGB444)
            pixel_format = bmdFormat8BitARGB;
    }

    upipe_bmd_src->deckLinkInput->StopStreams();

    if (pixel_format != upipe_bmd_src->pixel_format) {
        ubuf_mgr_release(upipe_bmd_src->pic_subpipe.ubuf_mgr);
        upipe_bmd_src->pic_subpipe.ubuf_mgr = NULL;
        upipe_bmd_src->pixel_format = pixel_format;
    }
    upipe_bmd_src_build_video(upipe, mode);

    upipe_bmd_src->deckLinkInput->EnableVideoInput(mode->GetDisplayMode(),
            upipe_bmd_src->pixel_format, bmdVideoInputEnableFormatDetection);
    upipe_bmd_src->deckLinkInput->FlushStreams();
    upipe_bmd_src->deckLinkInput->StartStreams();

    return S_OK;
}

/** @internal @This is called when receiving a video or audio frame.
 *
 * @param VideoFrame video frame
 * @param AudioPacket audio packet
 * @return S_OK
 */
HRESULT DeckLinkCaptureDelegate::VideoInputFrameArrived(
                        IDeckLinkVideoInputFrame *VideoFrame,
                        IDeckLinkAudioInputPacket *AudioPacket)
{
    struct upipe_bmd_src *upipe_bmd_src = upipe_bmd_src_from_upipe(upipe);
    uint64_t cr_sys = UINT64_MAX;
    if (upipe_bmd_src->uclock)
        cr_sys = uclock_now(upipe_bmd_src->uclock);

    if (VideoFrame) {
        struct ubuf *ubuf =
            ubuf_pic_bmd_alloc(upipe_bmd_src->pic_subpipe.ubuf_mgr, VideoFrame);
        if (likely(ubuf != NULL)) {
            struct uref *uref = uref_alloc(upipe_bmd_src->uref_mgr);
            bool has_input =
                !(VideoFrame->GetFlags() & bmdFrameHasNoInputSource);
            uref_attach_ubuf(uref, ubuf);
            uref_attr_set_priv(uref, has_input ? UPIPE_BMD_SRC_PIC :
                               UPIPE_BMD_SRC_PIC_NO_INPUT);
/* from:
 * https://www.blackmagicdesign.com/developer/support/faq/desktop-video-developer-support-faqs
 *
 * Audio / Video stream time offset when capturing with DeckLink Duo 2 / Quad 2
 * in Half Duplex
 *
 * When starting a capture operation with the DeckLink Duo 2 / Quad 2
 * configured as half-duplex [1], it may take a short time before input locks,
 * as in this configuration the sub-devices which default to playback may take
 * a short amount of time to switch from playback to capture.
 *
 * During this interval, there is an observable offset in the video and audio
 * stream times as determined via IDeckLinkVideoInputFrame::GetStreamTime() [2]
 * / IDeckLinkAudioInputPacket::GetPacketTime() [3].
 *
 * The recommendation is that applications should restart the streams at the no
 * signal -> signal transition point, which will ensure synchronisation between
 * the stream times, e.g. (in IDeckLinkInputCallback::VideoInputFrameArrived()
 * [4]):
 *
 * if(hasValidInputSource && !hadValidInputSource) {
 *     deckLinkInput->StopStreams();
 *     deckLinkInput->FlushStreams();
 *     deckLinkInput->StartStreams();
 * }
 * hadValidInputSource = hasValidInputSource;
 *
 * This will then result in the capture continuing as expected with
 * synchronised stream times.
 *
 * [1] DeckLink SDK Manual, 2.4.11 Configurable duplex mode
 * [2] 2.5.11.1 IDeckLinkVideoInputFrame::GetStreamTime method
 * [3] 2.5.12.3 IDeckLinkAudioInputPacket::GetPacketTime method
 * [4] 2.5.10.1 IDeckLinkInputCallback::VideoInputFrameArrived method
 */
            if (!upipe_bmd_src->had_input && has_input) {
                upipe_notice_va(upipe, "restart stream");
                upipe_bmd_src->deckLinkInput->StopStreams();
                upipe_bmd_src->deckLinkInput->FlushStreams();
                upipe_bmd_src->deckLinkInput->StartStreams();
            }
            upipe_bmd_src->had_input = has_input;

            if (cr_sys != UINT64_MAX)
                uref_clock_set_cr_sys(uref, cr_sys);
            BMDTimeValue FrameTime, FrameDuration;
            VideoFrame->GetStreamTime(&FrameTime, &FrameDuration, UCLOCK_FREQ);
            uref_clock_set_pts_orig(uref, FrameTime);
            uref_clock_set_dts_pts_delay(uref, 0);
            uref_clock_set_duration(uref, FrameDuration);

            if (upipe_bmd_src->progressive)
                uref_pic_set_progressive(uref);
            else if (upipe_bmd_src->tff)
                uref_pic_set_tff(uref);

            if (!uqueue_push(&upipe_bmd_src->uqueue, uref))
                uref_free(uref);
        }
    }

    if (AudioPacket) {
        struct ubuf *ubuf =
            ubuf_sound_bmd_alloc(upipe_bmd_src->sound_subpipe.ubuf_mgr,
                                 AudioPacket);
        if (likely(ubuf != NULL)) {
            struct uref *uref = uref_alloc(upipe_bmd_src->uref_mgr);
            uref_attach_ubuf(uref, ubuf);
            uref_attr_set_priv(uref, UPIPE_BMD_SRC_SOUND);

            if (cr_sys != UINT64_MAX)
                uref_clock_set_cr_sys(uref, cr_sys);
            BMDTimeValue PacketTime;
            AudioPacket->GetPacketTime(&PacketTime, UCLOCK_FREQ);
            uref_clock_set_pts_orig(uref, PacketTime);
            uref_clock_set_dts_pts_delay(uref, 0);
            uref_clock_set_duration(uref, AudioPacket->GetSampleFrameCount() *
                                          UCLOCK_FREQ / BMD_SAMPLERATE);

            if (!uqueue_push(&upipe_bmd_src->uqueue, uref))
                uref_free(uref);
        }
    }
    return S_OK;
}

/** @internal @This initializes an output subpipe of a bmd source pipe.
 *
 * @param upipe pointer to subpipe
 * @param sub_mgr manager of the subpipe
 * @param uprobe structure used to raise events by the subpipe
 */
static void upipe_bmd_src_output_init(struct upipe *upipe,
        struct upipe_mgr *sub_mgr, struct uprobe *uprobe)
{
    struct upipe_bmd_src *upipe_bmd_src = upipe_bmd_src_from_sub_mgr(sub_mgr);
    upipe_init(upipe, sub_mgr, uprobe);
    upipe->refcount = &upipe_bmd_src->urefcount;

    struct upipe_bmd_src_output *output = upipe_bmd_src_output_from_upipe(upipe);
    upipe_bmd_src_output_init_output(upipe);
    output->ubuf_mgr = NULL;

    upipe_throw_ready(upipe);
}

/** @internal @This processes control commands on a blackmagic output pipe.
 *
 * @param upipe description structure of the pipe
 * @param command type of command to process
 * @param args arguments of the command
 * @return an error code
 */
static int upipe_bmd_src_output_control(struct upipe *upipe,
                                        int command, va_list args)
{
    switch (command) {
        case UPIPE_GET_FLOW_DEF:
        case UPIPE_GET_OUTPUT:
        case UPIPE_SET_OUTPUT:
            return upipe_bmd_src_output_control_output(upipe, command, args);
        case UPIPE_SUB_GET_SUPER: {
            struct upipe **p = va_arg(args, struct upipe **);
            *p = upipe_bmd_src_to_upipe(upipe_bmd_src_from_sub_mgr(upipe->mgr));
            return UBASE_ERR_NONE;
        }

        default:
            return UBASE_ERR_UNHANDLED;
    }
}


/** @This clean up an output subpipe.
 *
 * @param upipe description structure of the pipe
 */
static void upipe_bmd_src_output_clean(struct upipe *upipe)
{
    upipe_throw_dead(upipe);

    upipe_bmd_src_output_clean_output(upipe);

    upipe_clean(upipe);
}

/** @internal @This initializes the output manager for a blackmagic pipe.
 *
 * @param upipe description structure of the pipe
 */
static void upipe_bmd_src_init_sub_mgr(struct upipe *upipe)
{
    struct upipe_bmd_src *upipe_bmd_src = upipe_bmd_src_from_upipe(upipe);
    struct upipe_mgr *sub_mgr = &upipe_bmd_src->sub_mgr;
    sub_mgr->refcount = NULL;
    sub_mgr->signature = UPIPE_BMD_SRC_OUTPUT_SIGNATURE;
    sub_mgr->upipe_alloc = NULL;
    sub_mgr->upipe_input = NULL;
    sub_mgr->upipe_control = upipe_bmd_src_output_control;
    sub_mgr->upipe_mgr_control = NULL;
}

/** @internal @This allocates a bmd source pipe.
 *
 * @param mgr common management structure
 * @param uprobe structure used to raise events
 * @param signature signature of the pipe allocator
 * @param args optional arguments
 * @return pointer to upipe or NULL in case of allocation error
 */
static struct upipe *_upipe_bmd_src_alloc(struct upipe_mgr *mgr,
                                          struct uprobe *uprobe,
                                          uint32_t signature, va_list args)
{
    if (signature != UPIPE_BMD_SRC_SIGNATURE)
        return NULL;
    struct uprobe *uprobe_pic = va_arg(args, struct uprobe *);
    struct uprobe *uprobe_sound = va_arg(args, struct uprobe *);

    struct upipe_bmd_src *upipe_bmd_src =
        (struct upipe_bmd_src *)malloc(sizeof(struct upipe_bmd_src) +
                                       uqueue_sizeof(MAX_QUEUE_LENGTH));
    if (unlikely(upipe_bmd_src == NULL)) {
        uprobe_release(uprobe_pic);
        uprobe_release(uprobe_sound);
        return NULL;
    }

    struct upipe *upipe = upipe_bmd_src_to_upipe(upipe_bmd_src);
    upipe_init(upipe, mgr, uprobe);

    upipe_bmd_src_init_urefcount(upipe);
    upipe_bmd_src_init_sync(upipe);
    upipe_bmd_src_init_uref_mgr(upipe);
    upipe_bmd_src_init_uclock(upipe);
    upipe_bmd_src_init_upump_mgr(upipe);
    upipe_bmd_src_init_upump(upipe);
    upipe_bmd_src->output = NULL;
    upipe_bmd_src_init_sub_mgr(upipe);

    upipe_bmd_src_output_init(upipe_bmd_src_output_to_upipe(
                                upipe_bmd_src_to_pic_subpipe(upipe_bmd_src)),
                              &upipe_bmd_src->sub_mgr, uprobe_pic);
    upipe_bmd_src_output_init(upipe_bmd_src_output_to_upipe(
                                upipe_bmd_src_to_sound_subpipe(upipe_bmd_src)),
                              &upipe_bmd_src->sub_mgr, uprobe_sound);

    uqueue_init(&upipe_bmd_src->uqueue, MAX_QUEUE_LENGTH,
                upipe_bmd_src->uqueue_extra);
    upipe_bmd_src->uri = NULL;
    upipe_bmd_src->deckLink = NULL;
    upipe_bmd_src->deckLinkInput = NULL;
    upipe_bmd_src->deckLinkConfiguration = NULL;
    upipe_bmd_src->deckLinkCaptureDelegate = NULL;
    upipe_bmd_src->progressive = false;
    upipe_bmd_src->timestamp_offset = 0;
    upipe_bmd_src->timestamp_highest = BMD_CLOCK_MIN;
    upipe_bmd_src->last_cr_sys = UINT64_MAX;
    upipe_bmd_src->fps.num = 25;
    upipe_bmd_src->fps.den = 1;
    upipe_bmd_src->tff = true;
    upipe_bmd_src->had_input = false;

    upipe_throw_ready(upipe);
    return upipe;
}

/** @internal @This reads data from the queue and outputs it.
 *
 * @param upipe description structure of the pipe
 * @param upump description structure of the pump
 */
static void upipe_bmd_src_work(struct upipe *upipe, struct upump *upump)
{
    struct upipe_bmd_src *upipe_bmd_src = upipe_bmd_src_from_upipe(upipe);
    struct uref *uref;

    /* unqueue urefs */
    while ((uref = uqueue_pop(&upipe_bmd_src->uqueue, struct uref *))) {
        uint64_t type;
        struct upipe *subpipe;
        if (unlikely(!ubase_check(uref_attr_get_priv(uref, &type)))) {
            upipe_throw_error(upipe, UBASE_ERR_UNKNOWN);
            uref_free(uref);
            continue;
        }
        uref_attr_delete_priv(uref);

        switch (type) {
            case UPIPE_BMD_SRC_PIC_NO_INPUT:
                upipe_bmd_src_sync_lost(upipe);
                uref_free(uref);
                continue;
            case UPIPE_BMD_SRC_PIC:
                subpipe = upipe_bmd_src_output_to_upipe(
                        upipe_bmd_src_to_pic_subpipe(upipe_bmd_src));
                upipe_bmd_src_sync_acquired(upipe);
                break;
            case UPIPE_BMD_SRC_SOUND:
                if (!upipe_bmd_src->acquired) {
                    uref_free(uref);
                    continue;
                }
                subpipe = upipe_bmd_src_output_to_upipe(
                        upipe_bmd_src_to_sound_subpipe(upipe_bmd_src));
                break;
            default:
                upipe_throw_error(upipe, UBASE_ERR_UNKNOWN);
                uref_free(uref);
                continue;
        }

        const char *def;
        if (unlikely(ubase_check(uref_flow_get_def(uref, &def)))) {
            upipe_bmd_src->fps.num = 25;
            upipe_bmd_src->fps.den = 1;
            uref_pic_flow_get_fps(uref, &upipe_bmd_src->fps);
            upipe_bmd_src_output_store_flow_def(subpipe, uref);
            continue;
        }

        uint64_t cr_sys = UINT64_MAX;
        uint64_t cr_sys_delta = 0;
        if (likely(ubase_check(uref_clock_get_cr_sys(uref, &cr_sys))) &&
                upipe_bmd_src->last_cr_sys != UINT64_MAX &&
                cr_sys >= upipe_bmd_src->last_cr_sys)
            cr_sys_delta = cr_sys - upipe_bmd_src->last_cr_sys;

        uint64_t pts_orig;
        uint64_t pts_prog = UINT64_MAX;
        if (likely(ubase_check(uref_clock_get_pts_orig(uref, &pts_orig)))) {
            pts_prog = pts_orig + upipe_bmd_src->timestamp_offset;

            if (type == UPIPE_BMD_SRC_PIC) {
                if (unlikely(pts_prog <= upipe_bmd_src->timestamp_highest)) {
                    uint64_t old = pts_prog;
                    uint64_t highest = upipe_bmd_src->timestamp_highest;

                    pts_prog = upipe_bmd_src->timestamp_highest + cr_sys_delta;
                    upipe_warn_va(upipe, "timestamp is in the past, "
                            "resetting %" PRIu64 " to %" PRIu64 " "
                            "highest %" PRIu64 " "
                            "orig %" PRIu64 " "
                            "delta %" PRIu64 " ",
                            old / (UCLOCK_FREQ / 1000),
                            pts_prog / (UCLOCK_FREQ / 1000),
                            highest / (UCLOCK_FREQ / 1000),
                            pts_orig / (UCLOCK_FREQ / 1000),
                            cr_sys_delta / (UCLOCK_FREQ / 1000));
                    upipe_bmd_src->timestamp_offset = pts_prog - pts_orig;
                }
                if (pts_prog > upipe_bmd_src->timestamp_highest)
                    upipe_bmd_src->timestamp_highest = pts_prog;
                uref_clock_set_pts_prog(uref, pts_prog);
                upipe_bmd_src->last_cr_sys = cr_sys;
            }
            else
                uref_clock_set_pts_prog(uref, upipe_bmd_src->timestamp_highest);
        }

        if (type == UPIPE_BMD_SRC_PIC || type == UPIPE_BMD_SRC_PIC_NO_INPUT) {
            if (likely(pts_prog != UINT64_MAX))
                upipe_throw_clock_ref(subpipe, uref, pts_prog, 0);
        }
        upipe_throw_clock_ts(subpipe, uref);
        upipe_bmd_src_output_output(subpipe, uref, &upipe_bmd_src->upump);
    }
}

/** @internal @This reads data from the source and outputs it.
 * It is called upon receiving new data from the card.
 *
 * @param upump description structure of the read watcher
 */
static void upipe_bmd_src_worker(struct upump *upump)
{
    struct upipe *upipe = upump_get_opaque(upump, struct upipe *);
    upipe_bmd_src_work(upipe, upump);
}

/** @internal @This returns a pointer to the current pseudo-output.
 *
 * @param upipe description structure of the pipe
 * @param p filled in with a pointer to the pseudo-output
 * @return an error code
 */
static int upipe_bmd_src_get_output(struct upipe *upipe, struct upipe **p)
{
    struct upipe_bmd_src *upipe_bmd_src = upipe_bmd_src_from_upipe(upipe);
    assert(p != NULL);
    *p = upipe_bmd_src->output;
    return UBASE_ERR_NONE;
}

/** @internal @This sets the pointer to the current pseudo-output.
 *
 * @param upipe description structure of the pipe
 * @param output pointer to the pseudo-output
 * @return an error code
 */
static int upipe_bmd_src_set_output(struct upipe *upipe, struct upipe *output)
{
    struct upipe_bmd_src *upipe_bmd_src = upipe_bmd_src_from_upipe(upipe);

    if (unlikely(upipe_bmd_src->output != NULL))
        upipe_release(upipe_bmd_src->output);
    if (unlikely(output == NULL))
        return UBASE_ERR_NONE;

    upipe_bmd_src->output = output;
    upipe_use(output);
    return UBASE_ERR_NONE;
}

/** @internal @This returns the currently opened device.
 *
 * @param upipe description structure of the pipe
 * @param uri_p filled in with the currently opened device
 * @return an error code
 */
static int upipe_bmd_src_get_uri(struct upipe *upipe, const char **uri_p)
{
    struct upipe_bmd_src *upipe_bmd_src = upipe_bmd_src_from_upipe(upipe);
    assert(uri_p != NULL);
    *uri_p = upipe_bmd_src->uri;
    return UBASE_ERR_NONE;
}

/** @internal @This is a helper for @ref upipe_bmd_src_set_uri.
 *
 * @param string option string
 * @return duplicated string
 */
static char *config_stropt(char *string)
{
    char *ret, *tmp;
    if (!string || strlen(string) == 0)
        return NULL;
    ret = tmp = strdup(string);
    while (*tmp) {
        if (*tmp == '_')
            *tmp = ' ';
        if (*tmp == '/') {
            *tmp = '\0';
            break;
        }
        tmp++;
    }
    return ret;
}

/** @internal @This asks to open the given device.
 *
 * @param upipe description structure of the pipe
 * @param uri URI
 * @return an error code
 */
static int upipe_bmd_src_set_uri(struct upipe *upipe, const char *uri)
{
    struct upipe_bmd_src *upipe_bmd_src = upipe_bmd_src_from_upipe(upipe);

    if (unlikely(upipe_bmd_src->uri != NULL)) {
        upipe_err(upipe, "unable to reopen device");
        return UBASE_ERR_INVALID;
    }

    if (unlikely(uri == NULL))
        return UBASE_ERR_NONE;

    if (unlikely(!upipe_bmd_src_demand_uref_mgr(upipe)))
        return UBASE_ERR_ALLOC;
    upipe_bmd_src_check_upump_mgr(upipe);

    const char *idx = strstr(uri, URI_SEP);
    if (unlikely(!idx)) {
        idx = uri;
    } else {
        idx += strlen(URI_SEP);
    }
    free(upipe_bmd_src->uri);
    upipe_bmd_src->uri = strdup(uri);
    upipe_notice_va(upipe, "opening device %s", upipe_bmd_src->uri);

    IDeckLinkIterator *deckLinkIterator;
    IDeckLink *deckLink = NULL;

    /* decklink interface interator */
    deckLinkIterator = CreateDeckLinkIteratorInstance();
    if (!deckLinkIterator) {
        upipe_err(upipe, "decklink drivers not found");
        return UBASE_ERR_EXTERNAL;
    }
    HRESULT result = E_NOINTERFACE;

    if (*idx == '@') {
        uint64_t card_topology;
        if (sscanf(idx + 1, "%" SCNu64, &card_topology) != 1) {
            upipe_err_va(upipe, "invalid URI '%s'", uri);
            return UBASE_ERR_INVALID;
        }

        /* get decklink interface handler */
        for ( ; ; ) {
            if (deckLink)
                deckLink->Release();
            result = deckLinkIterator->Next(&deckLink);
            if (result != S_OK)
                break;

            IDeckLinkAttributes *deckLinkAttributes = NULL;
            if (deckLink->QueryInterface(IID_IDeckLinkAttributes,
                                         (void**)&deckLinkAttributes) == S_OK) {
                int64_t deckLinkTopologicalId = 0;
                HRESULT result =
                    deckLinkAttributes->GetInt(BMDDeckLinkTopologicalID,
                            &deckLinkTopologicalId);
                deckLinkAttributes->Release();
                if (result == S_OK &&
                    (uint64_t)deckLinkTopologicalId == card_topology)
                    break;
            }
        }
    } else {
        int card_idx = atoi(idx);

        /* get decklink interface handler */
        for (int i = 0; i <= card_idx; i++) {
            if (deckLink)
                deckLink->Release();
            result = deckLinkIterator->Next(&deckLink);
            if (result != S_OK)
                break;
        }
    }
    deckLinkIterator->Release();

    if (result != S_OK) {
        upipe_err_va(upipe, "decklink card not found (%s)", uri);
        if (deckLink)
            deckLink->Release();
        return UBASE_ERR_EXTERNAL;
    }

    char *model_name;
    if (deckLink->GetModelName((const char **)&model_name) == S_OK) {
        upipe_notice_va(upipe, "detected card type %s", model_name);
        free(model_name);
    }

    /* get decklink status handler */
    IDeckLinkStatus *deckLinkStatus;
    if (deckLink->QueryInterface(IID_IDeckLinkStatus,
                                 (void**)&deckLinkStatus) != S_OK) {
        upipe_err(upipe, "decklink card has no status");
        deckLink->Release();
        return UBASE_ERR_EXTERNAL;
    }

    int64_t duplex;
    if (deckLinkStatus->GetInt(bmdDeckLinkStatusDuplexMode, &duplex) != S_OK) {
        upipe_warn(upipe, "couldn't query duplex status");
    } else if (duplex == bmdDuplexStatusInactive) {
        upipe_err(upipe, "decklink card has no input connector");
        deckLinkStatus->Release();
        deckLink->Release();
        return UBASE_ERR_INVALID;
    }
    deckLinkStatus->Release();

    /* get decklink input handler */
    IDeckLinkInput *deckLinkInput;
    if (deckLink->QueryInterface(IID_IDeckLinkInput,
                                 (void**)&deckLinkInput) != S_OK) {
        upipe_err(upipe, "decklink card has no input");
        deckLink->Release();
        return UBASE_ERR_EXTERNAL;
    }

    /* card configuration */
    IDeckLinkConfiguration *deckLinkConfiguration;
    if (deckLink->QueryInterface(IID_IDeckLinkConfiguration,
                (void**)&deckLinkConfiguration) != S_OK) {
        upipe_err(upipe, "decklink card has no configuration");
        deckLinkInput->Release();
        deckLink->Release();
        return UBASE_ERR_EXTERNAL;
    }

    /* decklink input connection */
    if (idx != uri) {
        int i = 0;
        BMDVideoConnection conn = 0;
        for (i = 0; upipe_bmd_src_video_conns[i].name; i++) {
            if (!ubase_ncmp(uri, upipe_bmd_src_video_conns[i].name)) {
                conn = upipe_bmd_src_video_conns[i].bmdConn;
                break;
            }
        }

        if (conn != 0) {
            deckLinkConfiguration->SetInt(
                bmdDeckLinkConfigVideoInputConnection, conn);
        } else
            upipe_warn_va(upipe, "unknown video connection '%s'", uri);
    }

    /* parse uri parameters */
    char *mode = NULL;
    char *audio = NULL;
    char *video_bits = NULL;
    char *audio_bits = NULL;
    char *passthrough = NULL;
    const char *params = strchr(idx, '/');
    if (params) {
        char *paramsdup = strdup(params);
        char *token = paramsdup;
        do {
            *token++ = '\0';
#define IS_OPTION(option) (!strncasecmp(token, option, strlen(option)))
#define ARG_OPTION(option) (token + strlen(option))
            if (IS_OPTION("mode=")) {
                free(mode);
                mode = config_stropt(ARG_OPTION("mode="));
            } else if (IS_OPTION("audio=")) {
                free(audio);
                audio = config_stropt(ARG_OPTION("audio="));
            } else if (IS_OPTION("audio_bits=")) {
                free(audio_bits);
                audio_bits = config_stropt(ARG_OPTION("audio_bits="));
            } else if (IS_OPTION("video_bits=")) {
                free(video_bits);
                video_bits = config_stropt(ARG_OPTION("video_bits="));
            } else if (IS_OPTION("passthrough=")) {
                free(passthrough);
                passthrough = config_stropt(ARG_OPTION("passthrough="));
            }
#undef IS_OPTION
#undef ARG_OPTION
        } while ((token = strchr(token, '/')) != NULL);

        free(paramsdup);
    }

    if (passthrough != NULL) {
        BMDDeckLinkCapturePassthroughMode passthrough_mode;

        if (!strcmp(passthrough, "disabled"))
            passthrough_mode = bmdDeckLinkCapturePassthroughModeDisabled;
        else if (!strcmp(passthrough, "direct"))
            passthrough_mode = bmdDeckLinkCapturePassthroughModeDirect;
        else if (!strcmp(passthrough, "clean switch"))
            passthrough_mode = bmdDeckLinkCapturePassthroughModeCleanSwitch;
        else {
            upipe_err_va(upipe, "invalid passthrough mode: %s", passthrough);
            deckLinkInput->Release();
            deckLink->Release();
            return UBASE_ERR_EXTERNAL;
        }

        upipe_notice_va(upipe, "passthrough mode: %s", passthrough);
        deckLinkConfiguration->SetInt(
                bmdDeckLinkConfigCapturePassThroughMode,
                passthrough_mode);
        free(passthrough);
    }

    if (audio != NULL) {
        int i = 0;
        BMDAudioConnection conn = 0;
        for (i = 0; upipe_bmd_src_audio_conns[i].name; i++) {
            if (!ubase_ncmp(audio, upipe_bmd_src_audio_conns[i].name)) {
                conn = upipe_bmd_src_audio_conns[i].bmdConn;
                break;
            }
        }

        if (conn != 0) {
            deckLinkConfiguration->SetInt(
                bmdDeckLinkConfigAudioInputConnection, conn);
        } else
            upipe_warn_va(upipe, "unknown audio connection '%s'", audio);
        free(audio);
    }

    /** audio sample depth */
    BMDAudioSampleType sample_format = bmdAudioSampleType16bitInteger;
    if (audio_bits != NULL) {
        if (!strcmp(audio_bits, "32")) {
            sample_format = bmdAudioSampleType32bitInteger;
        } else if (!strcmp(audio_bits, "16")) {
            sample_format = bmdAudioSampleType16bitInteger;
        } else {
            upipe_warn_va(upipe, "unknown audio_bits setting '%s'", audio_bits);
        }
        free(audio_bits);
    }

    /* save YUV format, useful when switching between yuv and ARGB */
    upipe_bmd_src->yuv_pixel_format = bmdFormat8BitYUV;
    if (video_bits != NULL) {
        if (!strcmp(video_bits, "10")) {
            upipe_bmd_src->yuv_pixel_format = bmdFormat10BitYUV;
        } else if (!strcmp(video_bits, "8")) {
            upipe_bmd_src->yuv_pixel_format = bmdFormat8BitYUV;
        } else {
            upipe_warn_va(upipe, "unknown video_bits setting '%s'", video_bits);
        }
        free(video_bits);
    }

    /* parse display mode */
    BMDDisplayMode displayModeId = bmdModeHD1080i50;
    if (mode) {
        int i = 0;
        for (i = 0; upipe_bmd_src_display_modes[i].name; i++) {
            if (!strcmp(mode, upipe_bmd_src_display_modes[i].name)) {
                displayModeId = upipe_bmd_src_display_modes[i].mode;
                break;
            }
        }
        free(mode);
    }

    /* find display mode */
    IDeckLinkDisplayModeIterator *displayModeIterator = NULL;
    if (deckLinkInput->GetDisplayModeIterator(&displayModeIterator) != S_OK) {
        deckLinkInput->Release();
        deckLink->Release();
        return UBASE_ERR_EXTERNAL;
    }
    /* iterate through available display modes and compare */
    IDeckLinkDisplayMode *displayMode = NULL;
    while (displayModeIterator->Next(&displayMode) == S_OK) {
        if (displayMode->GetDisplayMode() == displayModeId)
            break;
        displayMode->Release();
    }
    displayModeIterator->Release();

    if (unlikely(!displayMode)) {
        upipe_err(upipe, "display mode not available");
        deckLinkInput->Release();
        deckLink->Release();
        return UBASE_ERR_EXTERNAL;
    }

    char *display_name;
    if (displayMode->GetName((const char **)&display_name) == S_OK) {
        upipe_notice_va(upipe, "configuring mode %s", display_name);
        free(display_name);
    }

    upipe_bmd_src->pixel_format = upipe_bmd_src->yuv_pixel_format;
    BMDDisplayModeSupport displayModeSupported;
    if (deckLinkInput->DoesSupportVideoMode(displayMode->GetDisplayMode(),
                upipe_bmd_src->pixel_format, bmdVideoInputFlagDefault,
                &displayModeSupported, NULL) != S_OK ||
        displayModeSupported == bmdDisplayModeNotSupported) {
        upipe_err(upipe, "display mode not supported");
        deckLinkInput->Release();
        deckLink->Release();
        return UBASE_ERR_EXTERNAL;
    }

    /* format detection available? */
    IDeckLinkAttributes *deckLinkAttr = NULL;
    if (deckLink->QueryInterface(IID_IDeckLinkAttributes,
                                 (void**)&deckLinkAttr) != S_OK) {
        deckLinkInput->Release();
        deckLink->Release();
        return UBASE_ERR_EXTERNAL;
    }
    bool detectFormat = false;
    deckLinkAttr->GetFlag(BMDDeckLinkSupportsInputFormatDetection,
                          &detectFormat);
    deckLinkAttr->Release();
    if (!detectFormat) {
        upipe_warn(upipe, "automatic input format detection not supported");
    }

    /* configure input */
    if (deckLinkInput->EnableVideoInput(displayMode->GetDisplayMode(),
            upipe_bmd_src->pixel_format,
            (detectFormat ? bmdVideoInputEnableFormatDetection :
                            bmdVideoInputFlagDefault)) != S_OK) {
        upipe_err(upipe, "pixel format not supported");
        deckLinkInput->Release();
        deckLink->Release();
        return UBASE_ERR_EXTERNAL;
    }

    if (deckLinkInput->EnableAudioInput(bmdAudioSampleRate48kHz,
            sample_format, BMD_CHANNELS) != S_OK) {
        upipe_err(upipe, "sample format not supported");
        deckLinkInput->Release();
        deckLink->Release();
        return UBASE_ERR_EXTERNAL;
    }

    if (unlikely(!ubase_check(upipe_bmd_src_build_video(upipe, displayMode)) ||
                 !ubase_check(upipe_bmd_src_build_audio(upipe, sample_format)))) {
        deckLinkInput->Release();
        deckLink->Release();
        return UBASE_ERR_ALLOC;
    }
    displayMode->Release();

    upipe_bmd_src->deckLink = deckLink;
    upipe_bmd_src->deckLinkInput = deckLinkInput;
    upipe_bmd_src->deckLinkConfiguration = deckLinkConfiguration;
    /* callback helper */
    upipe_bmd_src->deckLinkCaptureDelegate = new DeckLinkCaptureDelegate(upipe);
    deckLinkInput->SetCallback(upipe_bmd_src->deckLinkCaptureDelegate);

    if (deckLinkInput->StartStreams() != S_OK) {
        upipe_err(upipe, "decklink card doesn't start");
        return UBASE_ERR_EXTERNAL;
    }

    return UBASE_ERR_NONE;
}

/** @internal @This processes control commands on a blackmagic source pipe.
 *
 * @param upipe description structure of the pipe
 * @param command type of command to process
 * @param args arguments of the command
 * @return an error code
 */
static int _upipe_bmd_src_control(struct upipe *upipe,
                                  int command, va_list args)
{
    switch (command) {
        case UPIPE_ATTACH_UPUMP_MGR: {
            upipe_bmd_src_set_upump(upipe, NULL);
            return upipe_bmd_src_attach_upump_mgr(upipe);
        }
        case UPIPE_ATTACH_UCLOCK:
            upipe_bmd_src_set_upump(upipe, NULL);
            upipe_bmd_src_require_uclock(upipe);
            return UBASE_ERR_NONE;

        case UPIPE_GET_OUTPUT: {
            struct upipe **p = va_arg(args, struct upipe **);
            return upipe_bmd_src_get_output(upipe, p);
        }
        case UPIPE_SET_OUTPUT: {
            struct upipe *output = va_arg(args, struct upipe *);
            return upipe_bmd_src_set_output(upipe, output);
        }
        case UPIPE_GET_URI: {
            const char **uri_p = va_arg(args, const char **);
            return upipe_bmd_src_get_uri(upipe, uri_p);
        }
        case UPIPE_SET_URI: {
            const char *uri = va_arg(args, const char *);
            return upipe_bmd_src_set_uri(upipe, uri);
        }

        case UPIPE_BMD_SRC_GET_PIC_SUB: {
            UBASE_SIGNATURE_CHECK(args, UPIPE_BMD_SRC_SIGNATURE)
            struct upipe **upipe_p = va_arg(args, struct upipe **);
            *upipe_p = upipe_bmd_src_output_to_upipe(
                    upipe_bmd_src_to_pic_subpipe(
                        upipe_bmd_src_from_upipe(upipe)));
            return UBASE_ERR_NONE;
        }
        case UPIPE_BMD_SRC_GET_SOUND_SUB: {
            UBASE_SIGNATURE_CHECK(args, UPIPE_BMD_SRC_SIGNATURE)
            struct upipe **upipe_p = va_arg(args, struct upipe **);
            *upipe_p = upipe_bmd_src_output_to_upipe(
                    upipe_bmd_src_to_sound_subpipe(
                        upipe_bmd_src_from_upipe(upipe)));
            return UBASE_ERR_NONE;
        }
        default:
            return UBASE_ERR_UNHANDLED;
    }
}

/** @internal @This processes control commands on a bmd source pipe, and
 * checks the status of the pipe afterwards.
 *
 * @param upipe description structure of the pipe
 * @param command type of command to process
 * @param args arguments of the command
 * @return an error code
 */
static int upipe_bmd_src_control(struct upipe *upipe, int command, va_list args)
{
    UBASE_RETURN(_upipe_bmd_src_control(upipe, command, args));
    upipe_bmd_src_check_upump_mgr(upipe);

    struct upipe_bmd_src *upipe_bmd_src = upipe_bmd_src_from_upipe(upipe);
    if (upipe_bmd_src->upump_mgr != NULL && upipe_bmd_src->uri != NULL &&
        upipe_bmd_src->upump == NULL) {
        struct upump *upump =
            uqueue_upump_alloc_pop(&upipe_bmd_src->uqueue,
                    upipe_bmd_src->upump_mgr, upipe_bmd_src_worker, upipe,
                    upipe->refcount);
        if (unlikely(upump == NULL)) {
            upipe_throw_fatal(upipe, UBASE_ERR_UPUMP);
            return UBASE_ERR_UPUMP;
        }
        upipe_bmd_src_set_upump(upipe, upump);
        upump_start(upump);
    }

    return UBASE_ERR_NONE;
}

/** @This frees a upipe.
 *
 * @param upipe description structure of the pipe
 */
static void upipe_bmd_src_free(struct upipe *upipe)
{
    struct upipe_bmd_src *upipe_bmd_src = upipe_bmd_src_from_upipe(upipe);

    if (upipe_bmd_src->deckLinkConfiguration)
        upipe_bmd_src->deckLinkConfiguration->Release();
    if (upipe_bmd_src->deckLinkInput) {
        upipe_bmd_src->deckLinkInput->StopStreams();
        upipe_bmd_src->deckLinkInput->Release();
    }
    if (upipe_bmd_src->deckLinkCaptureDelegate)
        upipe_bmd_src->deckLinkCaptureDelegate->Release();
    if (upipe_bmd_src->deckLink)
        upipe_bmd_src->deckLink->Release();
    upipe_bmd_src_work(upipe, NULL);
    uqueue_clean(&upipe_bmd_src->uqueue);

    ubuf_mgr_release(upipe_bmd_src->pic_subpipe.ubuf_mgr);
    ubuf_mgr_release(upipe_bmd_src->sound_subpipe.ubuf_mgr);

    upipe_bmd_src_output_clean(upipe_bmd_src_output_to_upipe(
                upipe_bmd_src_to_pic_subpipe(upipe_bmd_src)));
    upipe_bmd_src_output_clean(upipe_bmd_src_output_to_upipe(
                upipe_bmd_src_to_sound_subpipe(upipe_bmd_src)));

    upipe_throw_dead(upipe);

    free(upipe_bmd_src->uri);

    if (upipe_bmd_src->output != NULL)
        upipe_release(upipe_bmd_src->output);
    upipe_bmd_src_clean_uref_mgr(upipe);
    upipe_bmd_src_clean_upump(upipe);
    upipe_bmd_src_clean_upump_mgr(upipe);
    upipe_bmd_src_clean_uclock(upipe);
    upipe_bmd_src_clean_urefcount(upipe);
    upipe_bmd_src_clean_sync(upipe);

    upipe_clean(upipe);
    free(upipe_bmd_src);
}

extern "C" {
/** module manager static descriptor */
static struct upipe_mgr upipe_bmd_src_mgr = {
    /* .refcount = */ NULL,
    /* .signature = */ UPIPE_BMD_SRC_SIGNATURE,

    /* .upipe_err_str = */ NULL,
    /* .upipe_command_str = */ NULL,
    /* .upipe_event_str = */ NULL,

    /* .upipe_alloc = */ _upipe_bmd_src_alloc,
    /* .upipe_input = */ NULL,
    /* .upipe_control = */ upipe_bmd_src_control,

    /* .upipe_mgr_control = */ NULL
};
}

/** @This returns the management structure for all bmd source pipes.
 *
 * @return pointer to manager
 */
struct upipe_mgr *upipe_bmd_src_mgr_alloc(void)
{
    return &upipe_bmd_src_mgr;
}
