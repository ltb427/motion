/*   This file is part of Motion.
 *
 *   Motion is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   Motion is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with Motion.  If not, see <https://www.gnu.org/licenses/>.
 */

/***********************************************************
 *  In the top section are the functions that are used
 *  when processing the camera feed.  Since these functions
 *  are internal to the RTSP module, and many require FFmpeg
 *  structures in their declarations, they are within the
 *  HAVE_FFMPEG block that eliminates them entirely when
 *  FFmpeg is not present.
 *
 *  The functions:
 *      netcam_rtsp_setup
 *      netcam_rtsp_next
 *      netcam_rtsp_cleanup
 *  are called from video_common.c therefore must be defined even
 *  if FFmpeg is not present.  They must also not have FFmpeg
 *  structures in the declarations.  Simple error
 *  messages are raised if called when no FFmpeg is found.
 *
 *  Additional note:  Although this module is called netcam_rtsp,
 *  it actually handles more camera types than just rtsp.
 *  Within its current construct, it could be set up to handle
 *  whatever types of capture devices that ffmpeg can use.
 *  As of this writing it includes rtsp, http, files and v4l2.
 *
 ***********************************************************/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "motion.h"
#include "translate.h"
#include "util.h"
#include "logger.h"
#include "rotate.h"
#include "netcam.h"
#include "netcam_rtsp.h"

#ifdef HAVE_FFTW3
#include <fftw3.h>
#endif

#ifdef HAVE_FFMPEG

#include "ffmpeg.h"

#define RTSP_RECORD_QUEUE_SIZE 512
#define RTSP_AUDIO_QUEUE_SIZE 512
#define RTSP_AUDIO_TRIGGER_RATIO 0.60
#define RTSP_AUDIO_BAND_LOW 300.0
#define RTSP_AUDIO_BAND_HIGH 3400.0
#define RTSP_AUDIO_TRIGGER_WINDOW_SEC 5
#define RTSP_AUDIO_TRIGGER_HITS 3

#if defined(FF_API_OLD_CHANNEL_LAYOUT)
    #if defined(AV_CHANNEL_LAYOUT_MONO)
        #define MY_RTSP_USE_NEW_CHANNEL_LAYOUT_API 1
    #else
        #define MY_RTSP_USE_NEW_CHANNEL_LAYOUT_API (!FF_API_OLD_CHANNEL_LAYOUT)
    #endif
#elif defined(LIBAVUTIL_VERSION_MAJOR)
    #define MY_RTSP_USE_NEW_CHANNEL_LAYOUT_API (LIBAVUTIL_VERSION_MAJOR >= 59)
#else
    #define MY_RTSP_USE_NEW_CHANNEL_LAYOUT_API 0
#endif

static int netcam_rtsp_record_write(struct rtsp_context *rtsp_data, AVPacket *packet);
static int netcam_rtsp_record_enqueue(struct rtsp_context *rtsp_data, AVPacket *packet);
static void *netcam_rtsp_record_handler(void *arg);
static int netcam_rtsp_audio_enqueue(struct rtsp_context *rtsp_data, AVPacket *packet);
static void *netcam_rtsp_audio_handler(void *arg);

static int netcam_rtsp_valid_timebase(AVRational tb)
{

    return ((tb.num > 0) && (tb.den > 0));
}

static void netcam_rtsp_free_pkt(struct rtsp_context *rtsp_data)
{
    if (rtsp_data->packet_recv != NULL) {
        my_packet_free(rtsp_data->packet_recv);
    }
    rtsp_data->packet_recv = NULL;
}

static int netcam_rtsp_check_pixfmt(struct rtsp_context *rtsp_data)
{
    /* Determine if the format is YUV420P */
    int retcd;

    retcd = -1;
    if (((enum AVPixelFormat)rtsp_data->frame->format == MY_PIX_FMT_YUV420P) ||
        ((enum AVPixelFormat)rtsp_data->frame->format == MY_PIX_FMT_YUVJ420P)) {
        retcd = 0;
    }

    return retcd;

}

static void netcam_rtsp_pktarray_free(struct rtsp_context *rtsp_data)
{

    int indx;
    pthread_mutex_lock(&rtsp_data->mutex_pktarray);
        if (rtsp_data->pktarray_size > 0) {
            for(indx = 0; indx < rtsp_data->pktarray_size; indx++) {
                my_packet_free(rtsp_data->pktarray[indx].packet);
                rtsp_data->pktarray[indx].packet = NULL;
            }
        }
        free(rtsp_data->pktarray);
        rtsp_data->pktarray = NULL;
        rtsp_data->pktarray_size = 0;
        rtsp_data->pktarray_index = -1;
    pthread_mutex_unlock(&rtsp_data->mutex_pktarray);

}

static void netcam_rtsp_null_context(struct rtsp_context *rtsp_data)
{

    rtsp_data->swsctx          = NULL;
    rtsp_data->swsframe_in     = NULL;
    rtsp_data->swsframe_out    = NULL;
    rtsp_data->frame           = NULL;
    rtsp_data->codec_context   = NULL;
    rtsp_data->format_context  = NULL;
    rtsp_data->transfer_format = NULL;
    rtsp_data->record_format   = NULL;
    rtsp_data->record_stream_map = NULL;
    rtsp_data->record_stream_map_size = 0;
    rtsp_data->record_last_dts = NULL;
    rtsp_data->record_last_pts = NULL;
    rtsp_data->record_base_dts = NULL;
    rtsp_data->record_base_pts = NULL;
    rtsp_data->record_active   = FALSE;

}

static void netcam_rtsp_record_close_lck(struct rtsp_context *rtsp_data)
{

    if (rtsp_data->record_format != NULL) {
        av_write_trailer(rtsp_data->record_format);
        if (rtsp_data->record_format->pb != NULL) {
            avio_close(rtsp_data->record_format->pb);
        }
        avformat_free_context(rtsp_data->record_format);
        rtsp_data->record_format = NULL;
    }
    if (rtsp_data->record_stream_map != NULL) {
        free(rtsp_data->record_stream_map);
        rtsp_data->record_stream_map = NULL;
    }
    if (rtsp_data->record_last_dts != NULL) {
        free(rtsp_data->record_last_dts);
        rtsp_data->record_last_dts = NULL;
    }
    if (rtsp_data->record_last_pts != NULL) {
        free(rtsp_data->record_last_pts);
        rtsp_data->record_last_pts = NULL;
    }
    if (rtsp_data->record_base_dts != NULL) {
        free(rtsp_data->record_base_dts);
        rtsp_data->record_base_dts = NULL;
    }
    if (rtsp_data->record_base_pts != NULL) {
        free(rtsp_data->record_base_pts);
        rtsp_data->record_base_pts = NULL;
    }
    rtsp_data->record_stream_map_size = 0;
}

static void netcam_rtsp_record_queue_clear_lck(struct rtsp_context *rtsp_data)
{

    AVPacket *pkt;

    if ((rtsp_data->record_pktqueue == NULL) || (rtsp_data->record_pktqueue_size < 1)) {
        rtsp_data->record_pktqueue_head = 0;
        rtsp_data->record_pktqueue_tail = 0;
        rtsp_data->record_pktqueue_count = 0;
        pthread_cond_broadcast(&rtsp_data->cond_recordq);
        return;
    }

    while (rtsp_data->record_pktqueue_count > 0) {
        pkt = rtsp_data->record_pktqueue[rtsp_data->record_pktqueue_head];
        rtsp_data->record_pktqueue[rtsp_data->record_pktqueue_head] = NULL;
        rtsp_data->record_pktqueue_head =
            (rtsp_data->record_pktqueue_head + 1) % rtsp_data->record_pktqueue_size;
        rtsp_data->record_pktqueue_count--;
        if (pkt != NULL) {
            my_packet_free(pkt);
        }
    }

    rtsp_data->record_pktqueue_head = 0;
    rtsp_data->record_pktqueue_tail = 0;
    pthread_cond_broadcast(&rtsp_data->cond_recordq);
}

static void netcam_rtsp_close_context(struct rtsp_context *rtsp_data)
{

    netcam_rtsp_record_stop(rtsp_data);

    if (rtsp_data->swsctx != NULL) {
        sws_freeContext(rtsp_data->swsctx);
    }
    if (rtsp_data->swsframe_in != NULL) {
        my_frame_free(rtsp_data->swsframe_in);
    }
    if (rtsp_data->swsframe_out != NULL) {
        my_frame_free(rtsp_data->swsframe_out);
    }
    if (rtsp_data->frame != NULL) {
        my_frame_free(rtsp_data->frame);
    }
    if (rtsp_data->pktarray != NULL) {
        netcam_rtsp_pktarray_free(rtsp_data);
    }
    if (rtsp_data->codec_context != NULL) {
        my_avcodec_close(rtsp_data->codec_context);
    }
    if (rtsp_data->format_context != NULL) {
        avformat_close_input(&rtsp_data->format_context);
    }
    if (rtsp_data->transfer_format != NULL) {
        avformat_close_input(&rtsp_data->transfer_format);
    }
    #if (MYFFVER >= 57083)
        if (rtsp_data->hw_device_ctx   != NULL) av_buffer_unref(&rtsp_data->hw_device_ctx);
    #endif
    netcam_rtsp_null_context(rtsp_data);

}

static int netcam_rtsp_record_enqueue(struct rtsp_context *rtsp_data, AVPacket *packet)
{

    AVPacket *pkt_copy;

    pthread_mutex_lock(&rtsp_data->mutex_record);
        if (!rtsp_data->record_active) {
            pthread_mutex_unlock(&rtsp_data->mutex_record);
            return 0;
        }
    pthread_mutex_unlock(&rtsp_data->mutex_record);

    pkt_copy = my_packet_alloc(NULL);
    if (my_copy_packet(pkt_copy, packet) < 0) {
        my_packet_free(pkt_copy);
        return -1;
    }

    pthread_mutex_lock(&rtsp_data->mutex_recordq);
        if ((rtsp_data->record_pktqueue == NULL) || (rtsp_data->record_pktqueue_size < 1)) {
            pthread_mutex_unlock(&rtsp_data->mutex_recordq);
            my_packet_free(pkt_copy);
            return -1;
        }

        if (rtsp_data->record_pktqueue_count >= rtsp_data->record_pktqueue_size) {
            pthread_mutex_unlock(&rtsp_data->mutex_recordq);
            my_packet_free(pkt_copy);
            MOTION_LOG(WRN, TYPE_NETCAM, NO_ERRNO
                ,_("%s: Event record queue full; dropping packet"), rtsp_data->cameratype);
            return 0;
        }

        rtsp_data->record_pktqueue[rtsp_data->record_pktqueue_tail] = pkt_copy;
        rtsp_data->record_pktqueue_tail =
            (rtsp_data->record_pktqueue_tail + 1) % rtsp_data->record_pktqueue_size;
        rtsp_data->record_pktqueue_count++;
        pthread_cond_signal(&rtsp_data->cond_recordq);
    pthread_mutex_unlock(&rtsp_data->mutex_recordq);

    return 0;
}

int netcam_rtsp_record_start(struct rtsp_context *rtsp_data, const char *filename)
{

    unsigned int indx;
    int retcd;
    int mapped_streams;
    AVFormatContext *record_format;
    AVStream *stream_in;
    AVStream *stream_out;

    if ((rtsp_data == NULL) || (filename == NULL)) {
        return -1;
    }

    if (!rtsp_data->record_sync_init) {
        return -1;
    }

    netcam_rtsp_record_stop(rtsp_data);

    pthread_mutex_lock(&rtsp_data->mutex_record);
        if (rtsp_data->format_context == NULL) {
            pthread_mutex_unlock(&rtsp_data->mutex_record);
            return -1;
        }

        if (mycreate_path(filename) != 0) {
            MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO
                ,_("%s: Could not create path for %s")
                ,rtsp_data->cameratype, filename);
            pthread_mutex_unlock(&rtsp_data->mutex_record);
            return -1;
        }

        record_format = NULL;
        retcd = avformat_alloc_output_context2(&record_format, NULL, "mp4", filename);
        if ((retcd < 0) || (record_format == NULL)) {
            pthread_mutex_unlock(&rtsp_data->mutex_record);
            return (retcd < 0) ? retcd : -1;
        }

        retcd = avio_open(&record_format->pb, filename, MY_FLAG_WRITE);
        if (retcd < 0) {
            avformat_free_context(record_format);
            pthread_mutex_unlock(&rtsp_data->mutex_record);
            return retcd;
        }

        rtsp_data->record_stream_map_size = rtsp_data->format_context->nb_streams;
        if (rtsp_data->record_stream_map_size <= 0) {
            avio_close(record_format->pb);
            avformat_free_context(record_format);
            pthread_mutex_unlock(&rtsp_data->mutex_record);
            MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO
                ,_("%s: Camera stream list is empty; cannot start mp4 recording")
                ,rtsp_data->cameratype);
            return -1;
        }

        rtsp_data->record_stream_map = mymalloc(sizeof(int) * rtsp_data->record_stream_map_size);
        rtsp_data->record_last_dts = mymalloc(sizeof(int64_t) * rtsp_data->record_stream_map_size);
        rtsp_data->record_last_pts = mymalloc(sizeof(int64_t) * rtsp_data->record_stream_map_size);
        rtsp_data->record_base_dts = mymalloc(sizeof(int64_t) * rtsp_data->record_stream_map_size);
        rtsp_data->record_base_pts = mymalloc(sizeof(int64_t) * rtsp_data->record_stream_map_size);
        for (indx = 0; indx < (unsigned int)rtsp_data->record_stream_map_size; indx++) {
            rtsp_data->record_stream_map[indx] = -1;
            rtsp_data->record_last_dts[indx] = AV_NOPTS_VALUE;
            rtsp_data->record_last_pts[indx] = AV_NOPTS_VALUE;
            rtsp_data->record_base_dts[indx] = AV_NOPTS_VALUE;
            rtsp_data->record_base_pts[indx] = AV_NOPTS_VALUE;
        }

        mapped_streams = 0;
        for (indx = 0; indx < rtsp_data->format_context->nb_streams; indx++) {
            stream_in = rtsp_data->format_context->streams[indx];
            if ((stream_in == NULL) || (stream_in->codecpar == NULL)) {
                MOTION_LOG(WRN, TYPE_NETCAM, NO_ERRNO
                    ,_("%s: Skipping invalid stream metadata (stream %d)")
                    ,rtsp_data->cameratype, indx);
                continue;
            }

            if ((stream_in->codecpar->codec_type != AVMEDIA_TYPE_VIDEO) &&
                (stream_in->codecpar->codec_type != AVMEDIA_TYPE_AUDIO)) {
                continue;
            }

            if (!netcam_rtsp_valid_timebase(stream_in->time_base)) {
                MOTION_LOG(WRN, TYPE_NETCAM, NO_ERRNO
                    ,_("%s: Skipping stream with invalid time base (stream %d)")
                    ,rtsp_data->cameratype, indx);
                continue;
            }

            if (!avformat_query_codec(record_format->oformat
                , stream_in->codecpar->codec_id, FF_COMPLIANCE_NORMAL)) {
                MOTION_LOG(WRN, TYPE_NETCAM, NO_ERRNO
                    ,_("%s: Skipping unsupported codec for mp4 (stream %d, codec_id=%d)")
                    ,rtsp_data->cameratype, indx, stream_in->codecpar->codec_id);
                continue;
            }

            stream_out = avformat_new_stream(record_format, NULL);
            if (stream_out == NULL) {
                avio_close(record_format->pb);
                avformat_free_context(record_format);
                free(rtsp_data->record_stream_map);
                rtsp_data->record_stream_map = NULL;
                free(rtsp_data->record_last_dts);
                rtsp_data->record_last_dts = NULL;
                free(rtsp_data->record_last_pts);
                rtsp_data->record_last_pts = NULL;
                free(rtsp_data->record_base_dts);
                rtsp_data->record_base_dts = NULL;
                free(rtsp_data->record_base_pts);
                rtsp_data->record_base_pts = NULL;
                rtsp_data->record_stream_map_size = 0;
                pthread_mutex_unlock(&rtsp_data->mutex_record);
                return -1;
            }

            retcd = avcodec_parameters_copy(stream_out->codecpar, stream_in->codecpar);
            if (retcd < 0) {
                avio_close(record_format->pb);
                avformat_free_context(record_format);
                free(rtsp_data->record_stream_map);
                rtsp_data->record_stream_map = NULL;
                free(rtsp_data->record_last_dts);
                rtsp_data->record_last_dts = NULL;
                free(rtsp_data->record_last_pts);
                rtsp_data->record_last_pts = NULL;
                free(rtsp_data->record_base_dts);
                rtsp_data->record_base_dts = NULL;
                free(rtsp_data->record_base_pts);
                rtsp_data->record_base_pts = NULL;
                rtsp_data->record_stream_map_size = 0;
                pthread_mutex_unlock(&rtsp_data->mutex_record);
                return retcd;
            }

            stream_out->codecpar->codec_tag = 0;
            stream_out->time_base = stream_in->time_base;
            rtsp_data->record_stream_map[indx] = stream_out->index;
            mapped_streams++;
        }

        if (mapped_streams == 0) {
            avio_close(record_format->pb);
            avformat_free_context(record_format);
            free(rtsp_data->record_stream_map);
            rtsp_data->record_stream_map = NULL;
            free(rtsp_data->record_last_dts);
            rtsp_data->record_last_dts = NULL;
            free(rtsp_data->record_last_pts);
            rtsp_data->record_last_pts = NULL;
            free(rtsp_data->record_base_dts);
            rtsp_data->record_base_dts = NULL;
            free(rtsp_data->record_base_pts);
            rtsp_data->record_base_pts = NULL;
            rtsp_data->record_stream_map_size = 0;
            pthread_mutex_unlock(&rtsp_data->mutex_record);
            MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO
                ,_("%s: No mp4-compatible streams found for event recording")
                ,rtsp_data->cameratype);
            return -1;
        }

        retcd = avformat_write_header(record_format, NULL);
        if (retcd < 0) {
            avio_close(record_format->pb);
            avformat_free_context(record_format);
            free(rtsp_data->record_stream_map);
            rtsp_data->record_stream_map = NULL;
            free(rtsp_data->record_last_dts);
            rtsp_data->record_last_dts = NULL;
            free(rtsp_data->record_last_pts);
            rtsp_data->record_last_pts = NULL;
            free(rtsp_data->record_base_dts);
            rtsp_data->record_base_dts = NULL;
            free(rtsp_data->record_base_pts);
            rtsp_data->record_base_pts = NULL;
            rtsp_data->record_stream_map_size = 0;
            pthread_mutex_unlock(&rtsp_data->mutex_record);
            return retcd;
        }

        rtsp_data->record_format = record_format;
        rtsp_data->record_active = TRUE;
    pthread_mutex_unlock(&rtsp_data->mutex_record);

    MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
        ,_("%s: Recording event mp4 to %s"), rtsp_data->cameratype, filename);

    return 0;
}

void netcam_rtsp_record_stop(struct rtsp_context *rtsp_data)
{

    int wait_limit;
    int wait_elapsed = 0;

    if (rtsp_data == NULL) {
        return;
    }

    if (!rtsp_data->record_sync_init) {
        rtsp_data->record_active = FALSE;
        netcam_rtsp_record_close_lck(rtsp_data);
        return;
    }

    pthread_mutex_lock(&rtsp_data->mutex_record);
        rtsp_data->record_active = FALSE;
    pthread_mutex_unlock(&rtsp_data->mutex_record);

    if ((rtsp_data->cnt != NULL) && (rtsp_data->cnt->conf.watchdog_tmo > 1)) {
        wait_limit = rtsp_data->cnt->conf.watchdog_tmo - 1;
    } else {
        wait_limit = 5;
    }

    pthread_mutex_lock(&rtsp_data->mutex_recordq);
        while (rtsp_data->record_pktqueue_count > 0) {
            if (wait_elapsed >= wait_limit) {
                MOTION_LOG(WRN, TYPE_NETCAM, NO_ERRNO
                    ,_("%s: Timed out waiting for RTSP event recording to stop; forcing close")
                    ,rtsp_data->cameratype);
                break;
            }
            pthread_mutex_unlock(&rtsp_data->mutex_recordq);
            SLEEP(1, 0);
            wait_elapsed++;
            pthread_mutex_lock(&rtsp_data->mutex_recordq);
        }
    pthread_mutex_unlock(&rtsp_data->mutex_recordq);

    pthread_mutex_lock(&rtsp_data->mutex_record);
        netcam_rtsp_record_close_lck(rtsp_data);
    pthread_mutex_unlock(&rtsp_data->mutex_record);
}

static void *netcam_rtsp_record_handler(void *arg)
{

    struct rtsp_context *rtsp_data = arg;
    AVPacket *pkt;

    util_threadname_set("nr", rtsp_data->threadnbr, rtsp_data->camera_name);

    pthread_mutex_lock(&rtsp_data->mutex_recordq);
        rtsp_data->record_thread_finished = FALSE;
    pthread_mutex_unlock(&rtsp_data->mutex_recordq);

    while (TRUE) {
        pthread_mutex_lock(&rtsp_data->mutex_recordq);
            while ((rtsp_data->record_pktqueue_count == 0) && (!rtsp_data->record_thread_finish)) {
                pthread_cond_wait(&rtsp_data->cond_recordq, &rtsp_data->mutex_recordq);
            }

            if ((rtsp_data->record_pktqueue_count == 0) && (rtsp_data->record_thread_finish)) {
                pthread_mutex_unlock(&rtsp_data->mutex_recordq);
                break;
            }

            pkt = rtsp_data->record_pktqueue[rtsp_data->record_pktqueue_head];
            rtsp_data->record_pktqueue[rtsp_data->record_pktqueue_head] = NULL;
            rtsp_data->record_pktqueue_head =
                (rtsp_data->record_pktqueue_head + 1) % rtsp_data->record_pktqueue_size;
            rtsp_data->record_pktqueue_count--;
            if (rtsp_data->record_pktqueue_count == 0) {
                pthread_cond_broadcast(&rtsp_data->cond_recordq);
            }
        pthread_mutex_unlock(&rtsp_data->mutex_recordq);

        if (pkt != NULL) {
            netcam_rtsp_record_write(rtsp_data, pkt);
            my_packet_free(pkt);
        }
    }

    pthread_mutex_lock(&rtsp_data->mutex_recordq);
        rtsp_data->record_thread_finished = TRUE;
        pthread_cond_broadcast(&rtsp_data->cond_recordq);
    pthread_mutex_unlock(&rtsp_data->mutex_recordq);

    pthread_exit(NULL);
}

static void netcam_rtsp_audio_queue_clear_lck(struct rtsp_context *rtsp_data)
{

    AVPacket *pkt;

    if ((rtsp_data->audio_pktqueue == NULL) || (rtsp_data->audio_pktqueue_size < 1)) {
        rtsp_data->audio_pktqueue_head = 0;
        rtsp_data->audio_pktqueue_tail = 0;
        rtsp_data->audio_pktqueue_count = 0;
        pthread_cond_broadcast(&rtsp_data->cond_audioq);
        return;
    }

    while (rtsp_data->audio_pktqueue_count > 0) {
        pkt = rtsp_data->audio_pktqueue[rtsp_data->audio_pktqueue_head];
        rtsp_data->audio_pktqueue[rtsp_data->audio_pktqueue_head] = NULL;
        rtsp_data->audio_pktqueue_head =
            (rtsp_data->audio_pktqueue_head + 1) % rtsp_data->audio_pktqueue_size;
        rtsp_data->audio_pktqueue_count--;
        if (pkt != NULL) {
            my_packet_free(pkt);
        }
    }

    rtsp_data->audio_pktqueue_head = 0;
    rtsp_data->audio_pktqueue_tail = 0;
    pthread_cond_broadcast(&rtsp_data->cond_audioq);
}

static int netcam_rtsp_audio_enqueue(struct rtsp_context *rtsp_data, AVPacket *packet)
{

    AVPacket *pkt_copy;

    pthread_mutex_lock(&rtsp_data->mutex_audio);
        if ((!rtsp_data->audio_sync_init) || (rtsp_data->audio_thread_finish) ||
            (!rtsp_data->cnt->audio_detection_enabled) ||
            (!rtsp_data->cnt->conf.netcam_audio_detection)) {
            pthread_mutex_unlock(&rtsp_data->mutex_audio);
            return 0;
        }
    pthread_mutex_unlock(&rtsp_data->mutex_audio);

    pkt_copy = my_packet_alloc(NULL);
    if (my_copy_packet(pkt_copy, packet) < 0) {
        my_packet_free(pkt_copy);
        return -1;
    }

    pthread_mutex_lock(&rtsp_data->mutex_audioq);
        if ((rtsp_data->audio_pktqueue == NULL) || (rtsp_data->audio_pktqueue_size < 1)) {
            pthread_mutex_unlock(&rtsp_data->mutex_audioq);
            my_packet_free(pkt_copy);
            return -1;
        }

        if (rtsp_data->audio_pktqueue_count >= rtsp_data->audio_pktqueue_size) {
            pthread_mutex_unlock(&rtsp_data->mutex_audioq);
            my_packet_free(pkt_copy);
            MOTION_LOG(WRN, TYPE_NETCAM, NO_ERRNO
                ,_("%s: Audio queue full; dropping packet"), rtsp_data->cameratype);
            return 0;
        }

        rtsp_data->audio_pktqueue[rtsp_data->audio_pktqueue_tail] = pkt_copy;
        rtsp_data->audio_pktqueue_tail =
            (rtsp_data->audio_pktqueue_tail + 1) % rtsp_data->audio_pktqueue_size;
        rtsp_data->audio_pktqueue_count++;
        pthread_cond_signal(&rtsp_data->cond_audioq);
    pthread_mutex_unlock(&rtsp_data->mutex_audioq);

    return 0;
}

static void netcam_rtsp_audio_close_lck(struct rtsp_context *rtsp_data)
{

#ifdef HAVE_FFTW3
    if (rtsp_data->audio_fft_plan_ready) {
        fftw_destroy_plan(rtsp_data->audio_fft_plan);
        rtsp_data->audio_fft_plan_ready = FALSE;
    }
    if (rtsp_data->audio_fft_in != NULL) {
        fftw_free(rtsp_data->audio_fft_in);
        rtsp_data->audio_fft_in = NULL;
    }
    if (rtsp_data->audio_fft_out != NULL) {
        fftw_free(rtsp_data->audio_fft_out);
        rtsp_data->audio_fft_out = NULL;
    }
#endif
    rtsp_data->audio_fft_size = 0;

    if (rtsp_data->audio_swr != NULL) {
        swr_free(&rtsp_data->audio_swr);
    }
    if (rtsp_data->audio_codec_context != NULL) {
        my_avcodec_close(rtsp_data->audio_codec_context);
        rtsp_data->audio_codec_context = NULL;
    }
    if (rtsp_data->audio_frame != NULL) {
        my_frame_free(rtsp_data->audio_frame);
        rtsp_data->audio_frame = NULL;
    }
}

static int netcam_rtsp_audio_prepare_fft(struct rtsp_context *rtsp_data, int sample_count)
{

#ifdef HAVE_FFTW3
    if ((sample_count < 2) || (sample_count == rtsp_data->audio_fft_size && rtsp_data->audio_fft_plan_ready)) {
        return 0;
    }

    if (rtsp_data->audio_fft_plan_ready) {
        fftw_destroy_plan(rtsp_data->audio_fft_plan);
        rtsp_data->audio_fft_plan_ready = FALSE;
    }
    if (rtsp_data->audio_fft_in != NULL) {
        fftw_free(rtsp_data->audio_fft_in);
        rtsp_data->audio_fft_in = NULL;
    }
    if (rtsp_data->audio_fft_out != NULL) {
        fftw_free(rtsp_data->audio_fft_out);
        rtsp_data->audio_fft_out = NULL;
    }

    rtsp_data->audio_fft_in = fftw_malloc(sizeof(double) * sample_count);
    rtsp_data->audio_fft_out = fftw_malloc(sizeof(fftw_complex) * (sample_count / 2 + 1));
    if ((rtsp_data->audio_fft_in == NULL) || (rtsp_data->audio_fft_out == NULL)) {
        if (rtsp_data->audio_fft_in != NULL) {
            fftw_free(rtsp_data->audio_fft_in);
            rtsp_data->audio_fft_in = NULL;
        }
        if (rtsp_data->audio_fft_out != NULL) {
            fftw_free(rtsp_data->audio_fft_out);
            rtsp_data->audio_fft_out = NULL;
        }
        rtsp_data->audio_fft_size = 0;
        return -1;
    }

    rtsp_data->audio_fft_plan = fftw_plan_dft_r2c_1d(sample_count, rtsp_data->audio_fft_in,
                                                     rtsp_data->audio_fft_out, FFTW_ESTIMATE);
    if (rtsp_data->audio_fft_plan == NULL) {
        fftw_free(rtsp_data->audio_fft_in);
        fftw_free(rtsp_data->audio_fft_out);
        rtsp_data->audio_fft_in = NULL;
        rtsp_data->audio_fft_out = NULL;
        rtsp_data->audio_fft_size = 0;
        return -1;
    }

    rtsp_data->audio_fft_size = sample_count;
    rtsp_data->audio_fft_plan_ready = TRUE;
    return 0;
#else
    (void)rtsp_data;
    (void)sample_count;
    return -1;
#endif
}

static int netcam_rtsp_audio_analyze_frame(struct rtsp_context *rtsp_data, AVFrame *frame)
{

#ifdef HAVE_FFTW3
    int converted_samples;
    int out_samples;
    uint8_t **out_data = NULL;
    int out_linesize = 0;
    double total_energy = 0.0;
    double band_energy = 0.0;
    double ratio = 0.0;
    double trigger_ratio;
    double sample_rate;
    double low_bin;
    double high_bin;
    double band_low_hz;
    double band_high_hz;
    double conf_ratio;
    const char *ratio_cfg;
    int trigger_window_sec;
    int trigger_hits_required;
    int64_t now_ts;
    int indx;
    const double pi = 3.14159265358979323846;

    if ((rtsp_data->audio_swr == NULL) || (frame == NULL) || (frame->nb_samples <= 0)) {
        return 0;
    }

    if (!rtsp_data->cnt->conf.netcam_audio_detection) {
        rtsp_data->audio_trigger_hits = 0;
        rtsp_data->audio_trigger_last_ts = 0;
        return 0;
    }

    trigger_ratio = RTSP_AUDIO_TRIGGER_RATIO;
    ratio_cfg = rtsp_data->cnt->conf.netcam_audio_trigger_ratio;
    if ((ratio_cfg != NULL) && (ratio_cfg[0] != '\0')) {
        conf_ratio = atof(ratio_cfg);
        if ((conf_ratio > 0.0) && (conf_ratio <= 1.0)) {
            trigger_ratio = conf_ratio;
        } else if ((conf_ratio > 1.0) && (conf_ratio <= 100.0)) {
            trigger_ratio = conf_ratio / 100.0;
        }
    }

    band_low_hz = RTSP_AUDIO_BAND_LOW;
    if (rtsp_data->cnt->conf.netcam_audio_band_low > 0) {
        band_low_hz = (double)rtsp_data->cnt->conf.netcam_audio_band_low;
    }

    band_high_hz = RTSP_AUDIO_BAND_HIGH;
    if (rtsp_data->cnt->conf.netcam_audio_band_high > (int)RTSP_AUDIO_BAND_LOW) {
        band_high_hz = (double)rtsp_data->cnt->conf.netcam_audio_band_high;
    }
    if (band_high_hz <= band_low_hz) {
        band_high_hz = band_low_hz + 1.0;
    }

    trigger_window_sec = RTSP_AUDIO_TRIGGER_WINDOW_SEC;
    if (rtsp_data->cnt->conf.netcam_audio_trigger_window_sec > 0) {
        trigger_window_sec = rtsp_data->cnt->conf.netcam_audio_trigger_window_sec;
    }

    trigger_hits_required = RTSP_AUDIO_TRIGGER_HITS;
    if (rtsp_data->cnt->conf.netcam_audio_trigger_hits > 0) {
        trigger_hits_required = rtsp_data->cnt->conf.netcam_audio_trigger_hits;
    }

    out_samples = frame->nb_samples + 32;
    if (netcam_rtsp_audio_prepare_fft(rtsp_data, out_samples) < 0) {
        return -1;
    }

    if (av_samples_alloc_array_and_samples(&out_data, &out_linesize, 1, out_samples,
                                           AV_SAMPLE_FMT_DBL, 0) < 0) {
        return -1;
    }

    converted_samples = swr_convert(rtsp_data->audio_swr, out_data, out_samples,
                                    (const uint8_t **)frame->extended_data, frame->nb_samples);
    if (converted_samples <= 0) {
        av_freep(&out_data[0]);
        av_freep(&out_data);
        return 0;
    }

    if (converted_samples < 2) {
        av_freep(&out_data[0]);
        av_freep(&out_data);
        return 0;
    }

    if (netcam_rtsp_audio_prepare_fft(rtsp_data, converted_samples) < 0) {
        av_freep(&out_data[0]);
        av_freep(&out_data);
        return -1;
    }

    memcpy(rtsp_data->audio_fft_in, out_data[0], sizeof(double) * converted_samples);
    av_freep(&out_data[0]);
    av_freep(&out_data);

    for (indx = 0; indx < converted_samples; indx++) {
        double window = 0.5 * (1.0 - cos((2.0 * pi * indx) / (converted_samples - 1)));
        rtsp_data->audio_fft_in[indx] *= window;
    }

    fftw_execute(rtsp_data->audio_fft_plan);

    sample_rate = (double)rtsp_data->audio_sample_rate;
    if (sample_rate <= 0.0) {
        return 0;
    }

    low_bin = (band_low_hz * converted_samples) / sample_rate;
    high_bin = (band_high_hz * converted_samples) / sample_rate;
    if (high_bin > (converted_samples / 2)) {
        high_bin = converted_samples / 2;
    }

    for (indx = 1; indx <= (converted_samples / 2); indx++) {
        double real = rtsp_data->audio_fft_out[indx][0];
        double imag = rtsp_data->audio_fft_out[indx][1];
        double energy = real * real + imag * imag;

        total_energy += energy;
        if (((double)indx >= low_bin) && ((double)indx <= high_bin)) {
            band_energy += energy;
        }
    }

    if (total_energy <= 0.0) {
        return 0;
    }

    ratio = band_energy / total_energy;
    if (ratio >= trigger_ratio) {
        now_ts = (int64_t)time(NULL);
        if ((rtsp_data->audio_trigger_last_ts <= 0) ||
            ((now_ts - rtsp_data->audio_trigger_last_ts) > trigger_window_sec)) {
            rtsp_data->audio_trigger_hits = 1;
        } else {
            rtsp_data->audio_trigger_hits++;
        }
        rtsp_data->audio_trigger_last_ts = now_ts;

        if (rtsp_data->audio_trigger_hits >= trigger_hits_required) {
            rtsp_data->cnt->audio_event_user = TRUE;
            rtsp_data->cnt->event_trigger_source = TRIGGER_SOURCE_AUDIO;
            rtsp_data->audio_trigger_hits = 0;
            MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO
                ,_("%s: Audio event triggered (band energy ratio %.2f, ratio %.2f, %d hits/%ds)")
                , rtsp_data->cameratype, ratio, trigger_ratio, trigger_hits_required, trigger_window_sec);
        }
    } else if (rtsp_data->audio_trigger_last_ts > 0) {
        now_ts = (int64_t)time(NULL);
        if ((now_ts - rtsp_data->audio_trigger_last_ts) > trigger_window_sec) {
            rtsp_data->audio_trigger_hits = 0;
        }
    }

    return 0;
#else
    (void)rtsp_data;
    (void)frame;
    return -1;
#endif
}

static int netcam_rtsp_audio_open_codec(struct rtsp_context *rtsp_data)
{

#if ( MYFFVER >= 57041)
    int retcd;
    my_AVCodec *audio_decoder = NULL;
    #if MY_RTSP_USE_NEW_CHANNEL_LAYOUT_API
        AVChannelLayout in_ch_layout;
        AVChannelLayout out_ch_layout;
        int have_in_ch_layout = FALSE;
    #else
        int64_t channel_layout;
    #endif

    if ((rtsp_data == NULL) || (rtsp_data->format_context == NULL) ||
        (rtsp_data->format_context->nb_streams <= 0) ||
        (rtsp_data->format_context->streams == NULL)) {
        MOTION_LOG(DBG, TYPE_NETCAM, NO_ERRNO
            ,_("%s: Audio stream context not ready yet")
            , rtsp_data ? rtsp_data->cameratype : _("unknown"));
        return 0;
    }

    rtsp_data->audio_stream_index = -1;
    retcd = av_find_best_stream(rtsp_data->format_context, AVMEDIA_TYPE_AUDIO, -1, -1,
                                &audio_decoder, 0);
    if (retcd < 0) {
        MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
            ,_("%s: No audio stream found; audio analysis disabled")
            , rtsp_data->cameratype);
        return 0;
    }

    rtsp_data->audio_stream_index = retcd;

    rtsp_data->audio_codec_context = avcodec_alloc_context3(audio_decoder);
    if (rtsp_data->audio_codec_context == NULL) {
        return -1;
    }

    retcd = avcodec_parameters_to_context(rtsp_data->audio_codec_context,
                                          rtsp_data->format_context->streams[rtsp_data->audio_stream_index]->codecpar);
    if (retcd < 0) {
        return -1;
    }

    retcd = avcodec_open2(rtsp_data->audio_codec_context, audio_decoder, NULL);
    if (retcd < 0) {
        return -1;
    }

    rtsp_data->audio_frame = my_frame_alloc();
    if (rtsp_data->audio_frame == NULL) {
        return -1;
    }

    rtsp_data->audio_sample_rate = rtsp_data->audio_codec_context->sample_rate;
    if (rtsp_data->audio_sample_rate <= 0) {
        rtsp_data->audio_sample_rate = rtsp_data->format_context->streams[rtsp_data->audio_stream_index]->codecpar->sample_rate;
    }
    if (rtsp_data->audio_sample_rate <= 0) {
        MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO
            ,_("%s: Audio stream has no sample rate; audio analysis disabled")
            , rtsp_data->cameratype);
        return -1;
    }
    #if MY_RTSP_USE_NEW_CHANNEL_LAYOUT_API
        rtsp_data->audio_channels = rtsp_data->audio_codec_context->ch_layout.nb_channels;
        if (rtsp_data->audio_channels <= 0) {
            MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO
                ,_("%s: audio_channels is <= 0")
                , rtsp_data->cameratype);
        }
    #else
        rtsp_data->audio_channels = rtsp_data->audio_codec_context->channels;
        if (rtsp_data->audio_channels <= 0) {
            rtsp_data->audio_channels = av_get_channel_layout_nb_channels(rtsp_data->audio_codec_context->channel_layout);
        }
    #endif
    if (rtsp_data->audio_channels <= 0) {
        rtsp_data->audio_channels = 1;
    }

    #if MY_RTSP_USE_NEW_CHANNEL_LAYOUT_API
        memset(&in_ch_layout, 0, sizeof(in_ch_layout));
        memset(&out_ch_layout, 0, sizeof(out_ch_layout));

        if (rtsp_data->audio_codec_context->ch_layout.nb_channels > 0) {
            retcd = av_channel_layout_copy(&in_ch_layout, &rtsp_data->audio_codec_context->ch_layout);
            if (retcd < 0) {
                return -1;
            }
            have_in_ch_layout = TRUE;
        } else {
            av_channel_layout_default(&in_ch_layout, rtsp_data->audio_channels);
            have_in_ch_layout = TRUE;
        }

        av_channel_layout_default(&out_ch_layout, 1);

        retcd = swr_alloc_set_opts2(&rtsp_data->audio_swr,
            &out_ch_layout,
            AV_SAMPLE_FMT_DBL,
            rtsp_data->audio_sample_rate,
            &in_ch_layout,
            rtsp_data->audio_codec_context->sample_fmt,
            rtsp_data->audio_sample_rate,
            0,
            NULL);

        if (have_in_ch_layout) {
            av_channel_layout_uninit(&in_ch_layout);
        }
        av_channel_layout_uninit(&out_ch_layout);

        if ((retcd < 0) || (rtsp_data->audio_swr == NULL)) {
            return -1;
        }
    #else
        channel_layout = rtsp_data->audio_codec_context->channel_layout;
        if (channel_layout == 0) {
            channel_layout = av_get_default_channel_layout(rtsp_data->audio_channels);
        }

        rtsp_data->audio_swr = swr_alloc_set_opts(NULL,
            AV_CH_LAYOUT_MONO,
            AV_SAMPLE_FMT_DBL,
            rtsp_data->audio_sample_rate,
            channel_layout,
            rtsp_data->audio_codec_context->sample_fmt,
            rtsp_data->audio_sample_rate,
            0,
            NULL);
        if (rtsp_data->audio_swr == NULL) {
            return -1;
        }
    #endif

    retcd = swr_init(rtsp_data->audio_swr);
    if (retcd < 0) {
        return -1;
    }

    MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
        ,_("%s: Audio stream opened for frequency analysis (stream %d)")
        , rtsp_data->cameratype, rtsp_data->audio_stream_index);
    return 0;
#else
    (void)rtsp_data;
    return 0;
#endif
}

static void *netcam_rtsp_audio_handler(void *arg)
{

    struct rtsp_context *rtsp_data = arg;
    AVPacket *pkt;
    int retcd;

    util_threadname_set("na", rtsp_data->threadnbr, rtsp_data->camera_name);

    pthread_mutex_lock(&rtsp_data->mutex_audioq);
        rtsp_data->audio_thread_finished = FALSE;
    pthread_mutex_unlock(&rtsp_data->mutex_audioq);

    while (TRUE) {
        if ((rtsp_data->audio_stream_index < 0) && (!rtsp_data->audio_thread_finish)) {
            /* Re-try codec open after reconnect/late stream availability. */
            retcd = netcam_rtsp_audio_open_codec(rtsp_data);
            if (retcd < 0) {
                pthread_mutex_lock(&rtsp_data->mutex_audioq);
                    rtsp_data->audio_thread_finished = TRUE;
                    pthread_cond_broadcast(&rtsp_data->cond_audioq);
                pthread_mutex_unlock(&rtsp_data->mutex_audioq);
                pthread_exit(NULL);
            }
            /* Avoid tight CPU spin while waiting for audio stream availability. */
            SLEEP(0, 200000000);
            continue;
        }
        pthread_mutex_lock(&rtsp_data->mutex_audioq);
            while ((rtsp_data->audio_pktqueue_count == 0) && (!rtsp_data->audio_thread_finish)) {
                pthread_cond_wait(&rtsp_data->cond_audioq, &rtsp_data->mutex_audioq);
            }

            if ((rtsp_data->audio_pktqueue_count == 0) && (rtsp_data->audio_thread_finish)) {
                pthread_mutex_unlock(&rtsp_data->mutex_audioq);
                break;
            }

            pkt = rtsp_data->audio_pktqueue[rtsp_data->audio_pktqueue_head];
            rtsp_data->audio_pktqueue[rtsp_data->audio_pktqueue_head] = NULL;
            rtsp_data->audio_pktqueue_head =
                (rtsp_data->audio_pktqueue_head + 1) % rtsp_data->audio_pktqueue_size;
            rtsp_data->audio_pktqueue_count--;
            if (rtsp_data->audio_pktqueue_count == 0) {
                pthread_cond_broadcast(&rtsp_data->cond_audioq);
            }
        pthread_mutex_unlock(&rtsp_data->mutex_audioq);

        if (pkt != NULL) {
            if (pkt->stream_index == rtsp_data->audio_stream_index) {
#if ( MYFFVER >= 57041)
                if (rtsp_data->audio_codec_context != NULL) {
                    int send_ret = avcodec_send_packet(rtsp_data->audio_codec_context, pkt);
                    if ((send_ret >= 0) || (send_ret == AVERROR(EAGAIN))) {
                        while (TRUE) {
                            int frame_ret = avcodec_receive_frame(rtsp_data->audio_codec_context, rtsp_data->audio_frame);
                            if (frame_ret == 0) {
                                netcam_rtsp_audio_analyze_frame(rtsp_data, rtsp_data->audio_frame);
                                av_frame_unref(rtsp_data->audio_frame);
                            } else if ((frame_ret == AVERROR(EAGAIN)) || (frame_ret == AVERROR_EOF)) {
                                break;
                            } else {
                                break;
                            }
                        }
                    }
                }
#endif
            }
            my_packet_free(pkt);
        }
    }

    pthread_mutex_lock(&rtsp_data->mutex_audioq);
        rtsp_data->audio_thread_finished = TRUE;
        pthread_cond_broadcast(&rtsp_data->cond_audioq);
    pthread_mutex_unlock(&rtsp_data->mutex_audioq);

    pthread_exit(NULL);
}

static void netcam_rtsp_pktarray_resize(struct context *cnt, int is_highres)
{
    /* This is called from netcam_rtsp_next and is on the motion loop thread
     * The rtsp_data->mutex is locked around the call to this function.
    */

    /* Remember that this is a ring and we have two threads chasing around it
     * the ffmpeg is writing out of this ring while we are filling it up.  "Bad"
     * things will occur if the "add" thread catches up with the "write" thread.
     * We need this ring to be big enough so they don't collide.
     * The alternative is that we'd need to make a copy of the entire packet
     * array in the ffmpeg module and do our writing from that copy.  The
     * downside is that is a lot to be copying around for each image we want
     * to write out.  And putting a mutex on the array during adding function would
     * slow down the capture thread to the speed of the writing thread.  And that
     * writing thread operates at the user specified FPS which could be really slow
     * ...So....make this array big enough so we never catch our tail.  :)
     */

    int64_t               idnbr_last, idnbr_first;
    int                   indx;
    struct rtsp_context  *rtsp_data;
    struct packet_item   *tmp;
    int                   newsize;

    if (is_highres) {
        idnbr_last = cnt->imgs.image_ring[cnt->imgs.image_ring_out].idnbr_high;
        idnbr_first = cnt->imgs.image_ring[cnt->imgs.image_ring_in].idnbr_high;
        rtsp_data = cnt->rtsp_high;
    } else {
        idnbr_last = cnt->imgs.image_ring[cnt->imgs.image_ring_out].idnbr_norm;
        idnbr_first = cnt->imgs.image_ring[cnt->imgs.image_ring_in].idnbr_norm;
        rtsp_data = cnt->rtsp;
    }

    if (!rtsp_data->passthrough) {
        return;
    }

    /* The 30 is arbitrary */
    /* Double the size plus double last diff so we don't catch our tail */
    newsize =((idnbr_first - idnbr_last) * 2 ) + ((rtsp_data->idnbr - idnbr_last ) * 2);
    if (newsize < 30) {
        newsize = 30;
    }

    pthread_mutex_lock(&rtsp_data->mutex_pktarray);
        if ((rtsp_data->pktarray_size < newsize) ||  (rtsp_data->pktarray_size < 30)) {
            tmp = mymalloc(newsize * sizeof(struct packet_item));
            if (rtsp_data->pktarray_size > 0 ) {
                memcpy(tmp, rtsp_data->pktarray, sizeof(struct packet_item) * rtsp_data->pktarray_size);
            }
            for(indx = rtsp_data->pktarray_size; indx < newsize; indx++) {
                tmp[indx].packet = my_packet_alloc(tmp[indx].packet);
                tmp[indx].idnbr = 0;
                tmp[indx].iskey = FALSE;
                tmp[indx].iswritten = FALSE;
            }

            if (rtsp_data->pktarray != NULL) {
                free(rtsp_data->pktarray);
            }
            rtsp_data->pktarray = tmp;
            rtsp_data->pktarray_size = newsize;

            MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
                ,_("%s: Resized packet array to %d"), rtsp_data->cameratype,newsize);
        }
    pthread_mutex_unlock(&rtsp_data->mutex_pktarray);

}

static void netcam_rtsp_pktarray_add(struct rtsp_context *rtsp_data)
{

    int indx_next;
    int retcd;
    char errstr[128];

    pthread_mutex_lock(&rtsp_data->mutex_pktarray);

        if (rtsp_data->pktarray_size == 0) {
            pthread_mutex_unlock(&rtsp_data->mutex_pktarray);
            return;
        }

        /* Recall pktarray_size is one based but pktarray is zero based */
        if (rtsp_data->pktarray_index == (rtsp_data->pktarray_size-1) ) {
            indx_next = 0;
        } else {
            indx_next = rtsp_data->pktarray_index + 1;
        }

        rtsp_data->pktarray[indx_next].idnbr = rtsp_data->idnbr;

        my_packet_free(rtsp_data->pktarray[indx_next].packet);
        rtsp_data->pktarray[indx_next].packet = NULL;

        rtsp_data->pktarray[indx_next].packet = my_packet_alloc(rtsp_data->pktarray[indx_next].packet);
        retcd = my_copy_packet(rtsp_data->pktarray[indx_next].packet, rtsp_data->packet_recv);
        if ((rtsp_data->interrupted) || (retcd < 0)) {
            av_strerror(retcd, errstr, sizeof(errstr));
            MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
                ,_("%s: av_copy_packet: %s ,Interrupt: %s")
                ,rtsp_data->cameratype
                ,errstr, rtsp_data->interrupted ? _("True"):_("False"));
            my_packet_free(rtsp_data->pktarray[indx_next].packet);
            rtsp_data->pktarray[indx_next].packet = NULL;
        }

        if (rtsp_data->pktarray[indx_next].packet->flags & AV_PKT_FLAG_KEY) {
            rtsp_data->pktarray[indx_next].iskey = TRUE;
        } else {
            rtsp_data->pktarray[indx_next].iskey = FALSE;
        }
        rtsp_data->pktarray[indx_next].iswritten = FALSE;
        rtsp_data->pktarray[indx_next].timestamp_tv.tv_sec = rtsp_data->img_recv->image_time.tv_sec;
        rtsp_data->pktarray[indx_next].timestamp_tv.tv_usec = rtsp_data->img_recv->image_time.tv_usec;
        rtsp_data->pktarray_index = indx_next;
    pthread_mutex_unlock(&rtsp_data->mutex_pktarray);

}

static int netcam_decode_sw(struct rtsp_context *rtsp_data)
{

    #if ( MYFFVER >= 57041)
        int retcd;
        char errstr[128];

        retcd = avcodec_receive_frame(rtsp_data->codec_context, rtsp_data->frame);
        if ((rtsp_data->interrupted) || (rtsp_data->finish) || (retcd < 0) ) {
            if (retcd == AVERROR(EAGAIN)) {
                retcd = 0;
            } else if (retcd == AVERROR_INVALIDDATA) {
                MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
                    ,_("%s: Ignoring packet with invalid data")
                    ,rtsp_data->cameratype);
                retcd = 0;
            } else if (retcd < 0) {
                av_strerror(retcd, errstr, sizeof(errstr));
                    MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
                        ,_("%s: Rec frame error: %s")
                        ,rtsp_data->cameratype, errstr);
                retcd = -1;
            } else {
                retcd = -1;
            }
            return retcd;
        }

        return 1;
    #else
        int retcd, check=0;
        char errstr[128];

        retcd = avcodec_decode_video2(rtsp_data->codec_context, rtsp_data->frame, &check, rtsp_data->packet_recv);
        if ((rtsp_data->interrupted) || (rtsp_data->finish)) {
            return -1;
        }

        if (retcd == AVERROR_INVALIDDATA) {
            MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO, _("Ignoring packet with invalid data"));
            return 0;
        }

        if (retcd < 0) {
            av_strerror(retcd, errstr, sizeof(errstr));
            MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO, _("Error decoding packet: %s"),errstr);
            return -1;
        }

        if (check == 0 || retcd == 0) {
            return 0;
        }

        return 1;

    #endif
}

static int netcam_decode_vaapi(struct rtsp_context *rtsp_data)
{

    #if ( MYFFVER >= 57083)

        int retcd;
        char errstr[128];
        AVFrame *hw_frame = NULL;

        hw_frame = my_frame_alloc();

        retcd = avcodec_receive_frame(rtsp_data->codec_context, hw_frame);
        if ((rtsp_data->interrupted) || (rtsp_data->finish) || (retcd < 0) ) {
            if (retcd == AVERROR(EAGAIN)) {
                retcd = 0;
            } else if (retcd == AVERROR_INVALIDDATA) {
                MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
                    ,_("%s: Ignoring packet with invalid data")
                    ,rtsp_data->cameratype);
                retcd = 0;
            } else if (retcd < 0) {
                av_strerror(retcd, errstr, sizeof(errstr));
                    MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
                        ,_("%s: Rec frame error: %s")
                        ,rtsp_data->cameratype, errstr);
                retcd = -1;
            } else {
                retcd = -1;
            }
            my_frame_free(hw_frame);
            return retcd;
        }
        rtsp_data->frame->format=AV_PIX_FMT_YUV420P;

        retcd = av_hwframe_transfer_data(rtsp_data->frame, hw_frame, 0);
        if (retcd < 0) {
            MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
                ,_("%s: Error transferring HW decoded to system memory")
                ,rtsp_data->cameratype);
            my_frame_free(hw_frame);
            return -1;
        }

        my_frame_free(hw_frame);

        return 1;
    #else
        (void)rtsp_data;
        return 1;
    #endif
}

static int netcam_decode_cuda(struct rtsp_context *rtsp_data)
{

    #if ( MYFFVER >= 57083)

        int retcd;
        char errstr[128];
        AVFrame *hw_frame = NULL;

        hw_frame = my_frame_alloc();

        retcd = avcodec_receive_frame(rtsp_data->codec_context, hw_frame);
        if ((rtsp_data->interrupted) || (rtsp_data->finish) || (retcd < 0) ){
            if (retcd == AVERROR(EAGAIN)){
                retcd = 0;
            } else if (retcd == AVERROR_INVALIDDATA) {
                MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
                    ,_("%s: Ignoring packet with invalid data")
                    ,rtsp_data->cameratype);
                retcd = 0;
            } else if (retcd < 0) {
                av_strerror(retcd, errstr, sizeof(errstr));
                    MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
                        ,_("%s: Rec frame error: %s")
                        ,rtsp_data->cameratype, errstr);
                retcd = -1;
            } else {
                retcd = -1;
            }
            my_frame_free(hw_frame);
            return retcd;
        }
        rtsp_data->frame->format=AV_PIX_FMT_NV12;

        retcd = av_hwframe_transfer_data(rtsp_data->frame, hw_frame, 0);
        if (retcd < 0) {
            MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
                ,_("%s: Error transferring HW decoded to system memory")
                ,rtsp_data->cameratype);
            my_frame_free(hw_frame);
            return -1;
        }

        my_frame_free(hw_frame);

        return 1;
    #else
        (void)rtsp_data;
        return 1;
    #endif
}

/* netcam_rtsp_decode_video
 *
 * Return values:
 *   <0 error
 *   0 invalid but continue
 *   1 valid data
 */
static int netcam_rtsp_decode_video(struct rtsp_context *rtsp_data)
{

    #if ( MYFFVER >= 57041)

        int retcd;
        char errstr[128];

        /* The Invalid data problem comes frequently.  Usually at startup of rtsp cameras.
        * We now ignore those packets so this function would need to fail on a different error.
        * We should consider adding a maximum count of these errors and reset every time
        * we get a good image.
        */
        if (rtsp_data->finish) {
            /* This just speeds up the shutdown time */
            return 0;
        }

        retcd = avcodec_send_packet(rtsp_data->codec_context, rtsp_data->packet_recv);
        if ((rtsp_data->interrupted) || (rtsp_data->finish)) {
            return -1;
        }
        if (retcd == AVERROR_INVALIDDATA) {
            MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
                ,_("Ignoring packet with invalid data"));
            return 0;
        }
        if (retcd < 0 && retcd != AVERROR_EOF) {
            av_strerror(retcd, errstr, sizeof(errstr));
            MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
                ,_("Error sending packet to codec: %s"), errstr);
            return -1;
        }

        if (mystrceq(rtsp_data->decoder_nm,"vaapi")) {
            retcd = netcam_decode_vaapi(rtsp_data);
        } else if (mystrceq(rtsp_data->decoder_nm,"cuda")) {
            retcd = netcam_decode_cuda(rtsp_data);
        } else {
            retcd = netcam_decode_sw(rtsp_data);
        }

        return retcd;

    #else

        int retcd;

        if (rtsp_data->finish) {
            return 0;   /* This just speeds up the shutdown time */
        }

        if (mystrceq(rtsp_data->decoder_nm,"vaapi")) {
            retcd = netcam_decode_vaapi(rtsp_data);
        } else if (mystrceq(rtsp_data->decoder_nm,"cuda")) {
            retcd = netcam_decode_cuda(rtsp_data);
        } else {
            retcd = netcam_decode_sw(rtsp_data);
        }

        return retcd;

    #endif

}

static int netcam_rtsp_decode_packet(struct rtsp_context *rtsp_data)
{

    int frame_size;
    int retcd;

    if (rtsp_data->finish) {
        /* This just speeds up the shutdown time */
        return -1;
    }

    retcd = netcam_rtsp_decode_video(rtsp_data);
    if (retcd <= 0) {
        return retcd;
    }

    frame_size = my_image_get_buffer_size((enum AVPixelFormat) rtsp_data->frame->format
                                        ,rtsp_data->frame->width
                                        ,rtsp_data->frame->height);

    netcam_check_buffsize(rtsp_data->img_recv, frame_size);
    netcam_check_buffsize(rtsp_data->img_latest, frame_size);

    retcd = my_image_copy_to_buffer(rtsp_data->frame
                                    ,(uint8_t *)rtsp_data->img_recv->ptr
                                    ,(enum AVPixelFormat) rtsp_data->frame->format
                                    ,rtsp_data->frame->width
                                    ,rtsp_data->frame->height
                                    ,frame_size);
    if ((retcd < 0) || (rtsp_data->interrupted)) {
        MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO
            ,_("Error decoding video packet: Copying to buffer"));
        return -1;
    }

    rtsp_data->img_recv->used = frame_size;

    return frame_size;
}

static void netcam_hwdecoders(struct rtsp_context *rtsp_data)
{

    #if ( MYFFVER >= 57083)

        /* High Res pass through does not decode images into frames*/
        if (rtsp_data->high_resolution && rtsp_data->passthrough) {
            return;
        }

        if ((rtsp_data->hw_type == AV_HWDEVICE_TYPE_NONE) && (rtsp_data->first_image)) {
            MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
                ,_("%s: HW Devices: ")
                , rtsp_data->cameratype);
            while((rtsp_data->hw_type = av_hwdevice_iterate_types(rtsp_data->hw_type)) != AV_HWDEVICE_TYPE_NONE){
                if (rtsp_data->hw_type == AV_HWDEVICE_TYPE_VAAPI || rtsp_data->hw_type == AV_HWDEVICE_TYPE_CUDA) {
                    MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
                        ,_("%s: %s (available)")
                        , rtsp_data->cameratype
                        , av_hwdevice_get_type_name(rtsp_data->hw_type));
                } else {
                    MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
                        ,_("%s: %s (not implemented)")
                        , rtsp_data->cameratype
                        , av_hwdevice_get_type_name(rtsp_data->hw_type));
                }
            }
        }

        return;
    #else
        if (mystrcne(rtsp_data->decoder_nm,"NULL")) {
            MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
                ,_("%s: netcam_decoder %s disabled.")
                , rtsp_data->cameratype, rtsp_data->decoder_nm);
            free(rtsp_data->decoder_nm);
            rtsp_data->decoder_nm = mymalloc(5);
            snprintf(rtsp_data->decoder_nm, 5, "%s","NULL");
        }

        return;
    #endif
}

static enum AVPixelFormat netcam_getfmt_vaapi(AVCodecContext *avctx, const enum AVPixelFormat *pix_fmts)
{
    #if ( MYFFVER >= 57083)
        const enum AVPixelFormat *p;
        (void)avctx;

        for (p = pix_fmts; *p != -1; p++) {
            if (*p == AV_PIX_FMT_VAAPI) {
                return *p;
            }
        }

        MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO,_("Failed to get vaapi pix format"));
        return AV_PIX_FMT_NONE;
    #else
        (void)avctx;
        (void)pix_fmts;
        return AV_PIX_FMT_NONE;
    #endif
}

static enum AVPixelFormat netcam_getfmt_cuda(AVCodecContext *avctx, const enum AVPixelFormat *pix_fmts)
{
    #if ( MYFFVER >= 57083)
        const enum AVPixelFormat *p;
        (void)avctx;

        for (p = pix_fmts; *p != -1; p++) {
            if (*p == AV_PIX_FMT_CUDA) return *p;
        }

        MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO,_("Failed to get cuda pix format"));
        return AV_PIX_FMT_NONE;
    #else
        (void)avctx;
        (void)pix_fmts;
        return AV_PIX_FMT_NONE;
    #endif
}

static void netcam_rtsp_decoder_error(struct rtsp_context *rtsp_data, int retcd, const char* fnc_nm)
{

    char errstr[128];
    int indx;

    if (rtsp_data->interrupted) {
        MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO
            ,_("%s: Interrupted."),rtsp_data->cameratype);
    } else {
        if (retcd < 0) {
            av_strerror(retcd, errstr, sizeof(errstr));
            MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO
                ,_("%s: %s: %s")
                ,rtsp_data->cameratype,fnc_nm, errstr);
        } else {
            MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO
                ,_("%s: %s: Failed"),rtsp_data->cameratype,fnc_nm);
        }
    }

    if (mystrcne(rtsp_data->decoder_nm,"NULL")) {
        MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO
            ,_("%s: Decoder %s did not work.")
            ,rtsp_data->cameratype, rtsp_data->decoder_nm);
        MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO
            ,_("%s: Ignoring and removing the user requested decoder %s")
            ,rtsp_data->cameratype, rtsp_data->decoder_nm);

        for (indx = 0; indx < rtsp_data->parameters->params_count; indx++) {
            if (mystreq(rtsp_data->parameters->params_array[indx].param_name,"decoder") ) {
                free(rtsp_data->parameters->params_array[indx].param_value);
                rtsp_data->parameters->params_array[indx].param_value = mymalloc(5);
                snprintf(rtsp_data->parameters->params_array[indx].param_value, 5, "%s","NULL");
                break;
            }
        }

        free(rtsp_data->decoder_nm);
        rtsp_data->decoder_nm = mymalloc(5);
        snprintf(rtsp_data->decoder_nm, 5, "%s","NULL");

        if (rtsp_data->high_resolution) {
            util_parms_update(rtsp_data->parameters, rtsp_data->cnt, "netcam_high_params");
        } else {
            util_parms_update(rtsp_data->parameters, rtsp_data->cnt, "netcam_params");
        }
    }

}

static int netcam_rtsp_record_write(struct rtsp_context *rtsp_data, AVPacket *packet)
{

    int retcd;
    int in_index;
    int out_index;
    int64_t pkt_dts;
    int64_t pkt_pts;
    int64_t pkt_duration;
    int64_t last_dts;
    int64_t last_pts;
    int64_t base_dts;
    int64_t base_pts;
    int64_t step;
    AVStream *stream_in;
    AVStream *stream_out;

    pthread_mutex_lock(&rtsp_data->mutex_record);
        if ((rtsp_data->record_format == NULL) ||
            (rtsp_data->format_context == NULL) ||
            (rtsp_data->record_stream_map == NULL) ||
            (rtsp_data->record_last_dts == NULL) ||
            (rtsp_data->record_last_pts == NULL) ||
            (rtsp_data->record_base_dts == NULL) ||
            (rtsp_data->record_base_pts == NULL) ||
            (packet->stream_index < 0) ||
            (packet->stream_index >= rtsp_data->record_stream_map_size)) {
            pthread_mutex_unlock(&rtsp_data->mutex_record);
            return 0;
        }

        in_index = packet->stream_index;
        out_index = rtsp_data->record_stream_map[in_index];

        if (out_index < 0) {
            pthread_mutex_unlock(&rtsp_data->mutex_record);
            return 0;
        }

        if (((unsigned int)in_index >= rtsp_data->format_context->nb_streams) ||
            ((unsigned int)out_index >= rtsp_data->record_format->nb_streams)) {
            pthread_mutex_unlock(&rtsp_data->mutex_record);
            return 0;
        }

        stream_in = rtsp_data->format_context->streams[in_index];
        stream_out = rtsp_data->record_format->streams[out_index];
        if ((stream_in == NULL) || (stream_out == NULL)) {
            pthread_mutex_unlock(&rtsp_data->mutex_record);
            return 0;
        }
        if ((!netcam_rtsp_valid_timebase(stream_in->time_base)) ||
            (!netcam_rtsp_valid_timebase(stream_out->time_base))) {
            MOTION_LOG(WRN, TYPE_NETCAM, NO_ERRNO
                ,_("%s: Dropping packet due to invalid stream time base")
                ,rtsp_data->cameratype);
            pthread_mutex_unlock(&rtsp_data->mutex_record);
            return 0;
        }

        pkt_dts = packet->dts;
        pkt_pts = packet->pts;
        pkt_duration = packet->duration;

        if ((pkt_dts == AV_NOPTS_VALUE) && (pkt_pts != AV_NOPTS_VALUE)) {
            pkt_dts = pkt_pts;
        }
        if ((pkt_pts == AV_NOPTS_VALUE) && (pkt_dts != AV_NOPTS_VALUE)) {
            pkt_pts = pkt_dts;
        }

        if ((pkt_dts == AV_NOPTS_VALUE) && (pkt_pts == AV_NOPTS_VALUE)) {
            pthread_mutex_unlock(&rtsp_data->mutex_record);
            return 0;
        }

        if (pkt_dts != AV_NOPTS_VALUE) {
            pkt_dts = av_rescale_q_rnd(pkt_dts, stream_in->time_base, stream_out->time_base
                ,AV_ROUND_NEAR_INF | AV_ROUND_PASS_MINMAX);
        }
        if (pkt_pts != AV_NOPTS_VALUE) {
            pkt_pts = av_rescale_q_rnd(pkt_pts, stream_in->time_base, stream_out->time_base
                ,AV_ROUND_NEAR_INF | AV_ROUND_PASS_MINMAX);
        }
        if (pkt_duration > 0) {
            pkt_duration = av_rescale_q(pkt_duration, stream_in->time_base, stream_out->time_base);
        }

        if (pkt_dts != AV_NOPTS_VALUE) {
            base_dts = rtsp_data->record_base_dts[in_index];
            if (base_dts == AV_NOPTS_VALUE) {
                base_dts = pkt_dts;
                rtsp_data->record_base_dts[in_index] = base_dts;
            }
            pkt_dts -= base_dts;
            if (pkt_dts < 0) {
                pkt_dts = 0;
            }
        }

        if (pkt_pts != AV_NOPTS_VALUE) {
            base_pts = rtsp_data->record_base_pts[in_index];
            if (base_pts == AV_NOPTS_VALUE) {
                base_pts = pkt_pts;
                rtsp_data->record_base_pts[in_index] = base_pts;
            }
            pkt_pts -= base_pts;
            if (pkt_pts < 0) {
                pkt_pts = 0;
            }
        }

        if (pkt_dts != AV_NOPTS_VALUE) {
            last_dts = rtsp_data->record_last_dts[in_index];
            if ((last_dts != AV_NOPTS_VALUE) && (pkt_dts <= last_dts)) {
                step = (pkt_duration > 0) ? pkt_duration : 1;
                pkt_dts = last_dts + step;
            }
            rtsp_data->record_last_dts[in_index] = pkt_dts;
        }

        if (pkt_pts != AV_NOPTS_VALUE) {
            last_pts = rtsp_data->record_last_pts[in_index];
            if ((last_pts != AV_NOPTS_VALUE) && (pkt_pts <= last_pts)) {
                step = (pkt_duration > 0) ? pkt_duration : 1;
                pkt_pts = last_pts + step;
            }
            if ((pkt_dts != AV_NOPTS_VALUE) && (pkt_pts < pkt_dts)) {
                pkt_pts = pkt_dts;
            }
            rtsp_data->record_last_pts[in_index] = pkt_pts;
        }

        packet->dts = pkt_dts;
        packet->pts = pkt_pts;
        packet->duration = (pkt_duration > 0) ? pkt_duration : 0;
        packet->stream_index = out_index;
        packet->pos = -1;

        retcd = av_interleaved_write_frame(rtsp_data->record_format, packet);
        if (retcd < 0) {
            char errstr[128];

            av_strerror(retcd, errstr, sizeof(errstr));
            MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO
                ,_("%s: Error writing mp4 packet: %s")
                ,rtsp_data->cameratype, errstr);
            rtsp_data->record_active = FALSE;
            netcam_rtsp_record_close_lck(rtsp_data);
        }
    pthread_mutex_unlock(&rtsp_data->mutex_record);

    return retcd;
}

static int netcam_init_vaapi(struct rtsp_context *rtsp_data)
{

    #if ( MYFFVER >= 57083)

        int retcd;

        MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
            ,_("%s: Initializing vaapi decoder"),rtsp_data->cameratype);

        rtsp_data->hw_type = av_hwdevice_find_type_by_name("vaapi");
        if (rtsp_data->hw_type == AV_HWDEVICE_TYPE_NONE) {
            MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO,_("%s: Unable to find vaapi hw device")
                , rtsp_data->cameratype);
            netcam_rtsp_decoder_error(rtsp_data, 0, "av_hwdevice");
            return -1;
        }

        rtsp_data->codec_context = avcodec_alloc_context3(rtsp_data->decoder);
        if ((rtsp_data->codec_context == NULL) || (rtsp_data->interrupted)) {
            netcam_rtsp_decoder_error(rtsp_data, 0, "avcodec_alloc_context3");
            return -1;
        }

        retcd = avcodec_parameters_to_context(rtsp_data->codec_context,rtsp_data->strm->codecpar);
        if ((retcd < 0) || (rtsp_data->interrupted)) {
            netcam_rtsp_decoder_error(rtsp_data, retcd, "avcodec_parameters_to_context");
            return -1;
        }

        rtsp_data->hw_pix_fmt = AV_PIX_FMT_VAAPI;
        rtsp_data->codec_context->get_format  = netcam_getfmt_vaapi;
        av_opt_set_int(rtsp_data->codec_context, "refcounted_frames", 1, 0);
        rtsp_data->codec_context->sw_pix_fmt = AV_PIX_FMT_YUV420P;
        rtsp_data->codec_context->hwaccel_flags=
            AV_HWACCEL_FLAG_ALLOW_PROFILE_MISMATCH |
            AV_HWACCEL_FLAG_IGNORE_LEVEL;

        retcd = av_hwdevice_ctx_create(&rtsp_data->hw_device_ctx, rtsp_data->hw_type, NULL, NULL, 0);
        if (retcd < 0) {
            netcam_rtsp_decoder_error(rtsp_data, retcd, "hwctx");
            return -1;
        }
        rtsp_data->codec_context->hw_device_ctx = av_buffer_ref(rtsp_data->hw_device_ctx);

        return 0;
    #else
        (void)rtsp_data;
        (void)netcam_getfmt_vaapi;
        return 0;
    #endif
}

static int netcam_init_cuda(struct rtsp_context *rtsp_data)
{

    #if ( MYFFVER >= 57083)

        int retcd;

        MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
            ,_("%s: Initializing cuda decoder"),rtsp_data->cameratype);

        rtsp_data->hw_type = av_hwdevice_find_type_by_name("cuda");
        if (rtsp_data->hw_type == AV_HWDEVICE_TYPE_NONE){
            MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO,_("%s: Unable to find cuda hw device")
                , rtsp_data->cameratype);
            netcam_rtsp_decoder_error(rtsp_data, 0, "av_hwdevice");
            return -1;
        }

        rtsp_data->codec_context = avcodec_alloc_context3(rtsp_data->decoder);
        if ((rtsp_data->codec_context == NULL) || (rtsp_data->interrupted)){
            netcam_rtsp_decoder_error(rtsp_data, 0, "avcodec_alloc_context3");
            return -1;
        }

        retcd = avcodec_parameters_to_context(rtsp_data->codec_context,rtsp_data->strm->codecpar);
        if ((retcd < 0) || (rtsp_data->interrupted)) {
            netcam_rtsp_decoder_error(rtsp_data, retcd, "avcodec_parameters_to_context");
            return -1;
        }

        rtsp_data->hw_pix_fmt = AV_PIX_FMT_CUDA;
        rtsp_data->codec_context->get_format  = netcam_getfmt_cuda;
        av_opt_set_int(rtsp_data->codec_context, "refcounted_frames", 1, 0);
        rtsp_data->codec_context->sw_pix_fmt = AV_PIX_FMT_YUV420P;
        rtsp_data->codec_context->hwaccel_flags=
            AV_HWACCEL_FLAG_ALLOW_PROFILE_MISMATCH |
            AV_HWACCEL_FLAG_IGNORE_LEVEL;

        retcd = av_hwdevice_ctx_create(&rtsp_data->hw_device_ctx, rtsp_data->hw_type, NULL, NULL, 0);
        if (retcd < 0){
            netcam_rtsp_decoder_error(rtsp_data, retcd, "hwctx");
            return -1;
        }
        rtsp_data->codec_context->hw_device_ctx = av_buffer_ref(rtsp_data->hw_device_ctx);

        return 0;
    #else
        (void)rtsp_data;
        (void)netcam_getfmt_cuda;
        return 0;
    #endif
}

static int netcam_init_swdecoder(struct rtsp_context *rtsp_data)
{

    #if ( MYFFVER >= 57041)

        int retcd;

        MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
            ,_("%s: Initializing decoder"),rtsp_data->cameratype);

        if (mystrcne(rtsp_data->decoder_nm,"NULL")) {
            rtsp_data->decoder = avcodec_find_decoder_by_name(rtsp_data->decoder_nm);
            if (rtsp_data->decoder == NULL) {
                netcam_rtsp_decoder_error(rtsp_data, 0, "avcodec_find_decoder_by_name");
            } else {
                MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO,_("%s: Using decoder %s")
                    ,rtsp_data->cameratype, rtsp_data->decoder_nm);
            }
        }
        if (rtsp_data->decoder == NULL) {
            rtsp_data->decoder = avcodec_find_decoder(rtsp_data->strm->codecpar->codec_id);
        }
        if ((rtsp_data->decoder == NULL) || (rtsp_data->interrupted)) {
            netcam_rtsp_decoder_error(rtsp_data, 0, "avcodec_find_decoder");
            return -1;
        }

        rtsp_data->codec_context = avcodec_alloc_context3(rtsp_data->decoder);
        if ((rtsp_data->codec_context == NULL) || (rtsp_data->interrupted)) {
            netcam_rtsp_decoder_error(rtsp_data, 0, "avcodec_alloc_context3");
            return -1;
        }

        retcd = avcodec_parameters_to_context(rtsp_data->codec_context, rtsp_data->strm->codecpar);
        if ((retcd < 0) || (rtsp_data->interrupted)) {
            netcam_rtsp_decoder_error(rtsp_data, retcd, "avcodec_parameters_to_context");
            return -1;
        }

        rtsp_data->codec_context->error_concealment = FF_EC_GUESS_MVS | FF_EC_DEBLOCK;
        rtsp_data->codec_context->err_recognition = AV_EF_EXPLODE;

        return 0;
    #else
        int retcd;
        rtsp_data->codec_context = rtsp_data->strm->codec;
        rtsp_data->decoder = avcodec_find_decoder(rtsp_data->codec_context->codec_id);
        if ((rtsp_data->decoder == NULL) || (rtsp_data->interrupted)) {
            netcam_rtsp_decoder_error(rtsp_data, 0, "avcodec_find_decoder");
            return -1;
        }
        retcd = avcodec_open2(rtsp_data->codec_context, rtsp_data->decoder, NULL);
        if ((retcd < 0) || (rtsp_data->interrupted)) {
            netcam_rtsp_decoder_error(rtsp_data, retcd, "avcodec_open2");
            return -1;
        }
        return 0;
    #endif
}

static int netcam_rtsp_open_codec(struct rtsp_context *rtsp_data)
{

    #if ( MYFFVER >= 57041)
        int retcd;

        if (rtsp_data->finish) {
            /* This just speeds up the shutdown time */
            return -1;
        }

        netcam_hwdecoders(rtsp_data);

        rtsp_data->decoder=NULL;
        retcd = av_find_best_stream(rtsp_data->format_context
            , AVMEDIA_TYPE_VIDEO, -1, -1, &rtsp_data->decoder, 0);
        if ((retcd < 0) || (rtsp_data->interrupted)) {
            netcam_rtsp_decoder_error(rtsp_data, retcd, "av_find_best_stream");
            return -1;
        }
        rtsp_data->video_stream_index = retcd;
        rtsp_data->strm = rtsp_data->format_context->streams[rtsp_data->video_stream_index];

        if (mystrceq(rtsp_data->decoder_nm,"vaapi")) {
            retcd = netcam_init_vaapi(rtsp_data);
        } else if (mystrceq(rtsp_data->decoder_nm,"cuda")){
            retcd = netcam_init_cuda(rtsp_data);
        } else {
            retcd = netcam_init_swdecoder(rtsp_data);
        }
        if ((retcd < 0) || (rtsp_data->interrupted)) {
            netcam_rtsp_decoder_error(rtsp_data, retcd, "initdecoder");
            return -1;
        }

        retcd = avcodec_open2(rtsp_data->codec_context,rtsp_data->decoder, NULL);
        if ((retcd < 0) || (rtsp_data->interrupted)) {
            netcam_rtsp_decoder_error(rtsp_data, retcd, "avcodec_open2");
            return -1;
        }

        MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
            ,_("%s: Decoder opened"), rtsp_data->cameratype);

        return 0;
    #else

        int retcd;

        if (rtsp_data->finish) {
            /* This just speeds up the shutdown time */
            return -1;
        }

        netcam_hwdecoders(rtsp_data);
        rtsp_data->decoder = NULL;

        retcd = av_find_best_stream(rtsp_data->format_context, AVMEDIA_TYPE_VIDEO, -1, -1, NULL, 0);
        if ((retcd < 0) || (rtsp_data->interrupted)) {
            netcam_rtsp_decoder_error(rtsp_data, retcd, "av_find_best_stream");
            return -1;
        }
        rtsp_data->video_stream_index = retcd;
        rtsp_data->strm = rtsp_data->format_context->streams[rtsp_data->video_stream_index];

        /* This is currently always true for older ffmpeg until it is built */
        if (mystrceq(rtsp_data->decoder_nm,"vaapi")) {
            retcd = netcam_init_vaapi(rtsp_data);
        } else if (mystrceq(rtsp_data->decoder_nm,"cuda")){
            retcd = netcam_init_cuda(rtsp_data);
        } else {
            retcd = netcam_init_swdecoder(rtsp_data);
        }
        return retcd;
    #endif


}

static struct rtsp_context *rtsp_new_context(void)
{
    struct rtsp_context *ret;

    /* Note that mymalloc will exit on any problem. */
    ret = mymalloc(sizeof(struct rtsp_context));

    memset(ret, 0, sizeof(struct rtsp_context));

    return ret;
}

static int netcam_rtsp_interrupt(void *ctx)
{
    struct rtsp_context *rtsp_data = ctx;

    if (rtsp_data->finish && rtsp_data->handler_finished) {
        rtsp_data->interrupted = TRUE;
        return TRUE;
    }

    if (rtsp_data->status == RTSP_CONNECTED) {
        return FALSE;
    } else if (rtsp_data->status == RTSP_READINGIMAGE) {
        if (gettimeofday(&rtsp_data->interruptcurrenttime, NULL) < 0) {
            MOTION_LOG(ERR, TYPE_NETCAM, SHOW_ERRNO, "gettimeofday");
        }
        if ((rtsp_data->interruptcurrenttime.tv_sec - rtsp_data->interruptstarttime.tv_sec ) > rtsp_data->interruptduration){
            MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
                ,_("%s: Camera reading (%s) timed out")
                , rtsp_data->cameratype, rtsp_data->camera_name);
            rtsp_data->interrupted = TRUE;
            return TRUE;
        } else{
            return FALSE;
        }
    } else {
        /* This is for NOTCONNECTED and RECONNECTING status.  We give these
         * options more time because all the ffmpeg calls that are inside the
         * rtsp_connect function will use the same start time.  Otherwise we
         * would need to reset the time before each call to a ffmpeg function.
        */
        if (gettimeofday(&rtsp_data->interruptcurrenttime, NULL) < 0) {
            MOTION_LOG(ERR, TYPE_NETCAM, SHOW_ERRNO, "gettimeofday");
        }
        if ((rtsp_data->interruptcurrenttime.tv_sec - rtsp_data->interruptstarttime.tv_sec ) > rtsp_data->interruptduration){
            MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
                ,_("%s: Camera (%s) timed out")
                , rtsp_data->cameratype, rtsp_data->camera_name);
            rtsp_data->interrupted = TRUE;
            return TRUE;
        } else{
            return FALSE;
        }
    }

    /* should not be possible to get here */
    return FALSE;
}

static int netcam_rtsp_open_sws(struct rtsp_context *rtsp_data)
{

    if (rtsp_data->finish) {
        /* This just speeds up the shutdown time */
        return -1;
    }

    rtsp_data->swsframe_in = my_frame_alloc();
    if (rtsp_data->swsframe_in == NULL) {
        if (rtsp_data->status == RTSP_NOTCONNECTED) {
            MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO, _("Unable to allocate swsframe_in."));
        }
        netcam_rtsp_close_context(rtsp_data);
        return -1;
    }

    rtsp_data->swsframe_out = my_frame_alloc();
    if (rtsp_data->swsframe_out == NULL) {
        if (rtsp_data->status == RTSP_NOTCONNECTED) {
            MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO, _("Unable to allocate swsframe_out."));
        }
        netcam_rtsp_close_context(rtsp_data);
        return -1;
    }

    /*
     *  The scaling context is used to change dimensions to config file and
     *  also if the format sent by the camera is not YUV420.
     */
    rtsp_data->swsctx = sws_getContext(
         rtsp_data->frame->width
        ,rtsp_data->frame->height
        ,(enum AVPixelFormat) rtsp_data->frame->format
        ,rtsp_data->imgsize.width
        ,rtsp_data->imgsize.height
        ,MY_PIX_FMT_YUV420P
        ,SWS_BICUBIC,NULL,NULL,NULL);
    if (rtsp_data->swsctx == NULL) {
        if (rtsp_data->status == RTSP_NOTCONNECTED) {
            MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO, _("Unable to allocate scaling context."));
        }
        netcam_rtsp_close_context(rtsp_data);
        return -1;
    }

    rtsp_data->swsframe_size = my_image_get_buffer_size(
            MY_PIX_FMT_YUV420P
            ,rtsp_data->imgsize.width
            ,rtsp_data->imgsize.height);
    if (rtsp_data->swsframe_size <= 0) {
        if (rtsp_data->status == RTSP_NOTCONNECTED) {
            MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO, _("Error determining size of frame out"));
        }
        netcam_rtsp_close_context(rtsp_data);
        return -1;
    }

    /* the image buffers must be big enough to hold the final frame after resizing */
    netcam_check_buffsize(rtsp_data->img_recv, rtsp_data->swsframe_size);
    netcam_check_buffsize(rtsp_data->img_latest, rtsp_data->swsframe_size);

    return 0;

}

static int netcam_rtsp_resize(struct rtsp_context *rtsp_data)
{

    int      retcd;
    char     errstr[128];
    uint8_t *buffer_out;

    if (rtsp_data->finish) {
        /* This just speeds up the shutdown time */
        return -1;
    }

    if (rtsp_data->swsctx == NULL) {
        if (netcam_rtsp_open_sws(rtsp_data) < 0) {
            return -1;
        }
    }

    retcd=my_image_fill_arrays(
        rtsp_data->swsframe_in
        ,(uint8_t*)rtsp_data->img_recv->ptr
        ,(enum AVPixelFormat)rtsp_data->frame->format
        ,rtsp_data->frame->width
        ,rtsp_data->frame->height);
    if (retcd < 0) {
        if (rtsp_data->status == RTSP_NOTCONNECTED) {
            av_strerror(retcd, errstr, sizeof(errstr));
            MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO
                ,_("Error allocating picture in: %s"), errstr);
        }
        netcam_rtsp_close_context(rtsp_data);
        return -1;
    }

    buffer_out=(uint8_t *)av_malloc(rtsp_data->swsframe_size*sizeof(uint8_t));

    retcd=my_image_fill_arrays(
        rtsp_data->swsframe_out
        ,buffer_out
        ,MY_PIX_FMT_YUV420P
        ,rtsp_data->imgsize.width
        ,rtsp_data->imgsize.height);
    if (retcd < 0) {
        if (rtsp_data->status == RTSP_NOTCONNECTED) {
            av_strerror(retcd, errstr, sizeof(errstr));
            MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO
                ,_("Error allocating picture out: %s"), errstr);
        }
        netcam_rtsp_close_context(rtsp_data);
        return -1;
    }

    retcd = sws_scale(
        rtsp_data->swsctx
        ,(const uint8_t* const *)rtsp_data->swsframe_in->data
        ,rtsp_data->swsframe_in->linesize
        ,0
        ,rtsp_data->frame->height
        ,rtsp_data->swsframe_out->data
        ,rtsp_data->swsframe_out->linesize);
    if (retcd < 0) {
        if (rtsp_data->status == RTSP_NOTCONNECTED) {
            av_strerror(retcd, errstr, sizeof(errstr));
            MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO
                ,_("Error resizing/reformatting: %s"), errstr);
        }
        netcam_rtsp_close_context(rtsp_data);
        return -1;
    }

    retcd=my_image_copy_to_buffer(
         rtsp_data->swsframe_out
        ,(uint8_t *)rtsp_data->img_recv->ptr
        ,MY_PIX_FMT_YUV420P
        ,rtsp_data->imgsize.width
        ,rtsp_data->imgsize.height
        ,rtsp_data->swsframe_size);
    if (retcd < 0) {
        if (rtsp_data->status == RTSP_NOTCONNECTED) {
            av_strerror(retcd, errstr, sizeof(errstr));
            MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO
                ,_("Error putting frame into output buffer: %s"), errstr);
        }
        netcam_rtsp_close_context(rtsp_data);
        return -1;
    }
    rtsp_data->img_recv->used = rtsp_data->swsframe_size;

    av_free(buffer_out);

    return 0;

}

static int netcam_rtsp_read_image(struct rtsp_context *rtsp_data)
{

    int  size_decoded;
    int  retcd, nodata, haveimage;
    char errstr[128];
    netcam_buff *xchg;

    if (rtsp_data->finish) {
        /* This just speeds up the shutdown time */
        return -1;
    }

    rtsp_data->packet_recv = my_packet_alloc(rtsp_data->packet_recv);

    rtsp_data->interrupted=FALSE;
    if (gettimeofday(&rtsp_data->interruptstarttime, NULL) < 0) {
        MOTION_LOG(ERR, TYPE_NETCAM, SHOW_ERRNO, "gettimeofday");
    }
    rtsp_data->interruptduration = 10;

    rtsp_data->status = RTSP_READINGIMAGE;
    rtsp_data->img_recv->used = 0;
    size_decoded = 0;
    nodata = 0;
    haveimage = FALSE;

    while ((haveimage == FALSE) && (rtsp_data->interrupted == FALSE)) {
        retcd = av_read_frame(rtsp_data->format_context, rtsp_data->packet_recv);
        /* The 2000 for nodata tries is arbritrary*/
        if ((rtsp_data->interrupted) || (retcd < 0 ) || (nodata > 2000)) {
            if (rtsp_data->interrupted) {
                MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
                    ,_("%s: Interrupted"),rtsp_data->cameratype);
            } else if (retcd < 0) {
                av_strerror(retcd, errstr, sizeof(errstr));
                MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
                    ,_("%s: av_read_frame: %s")
                    ,rtsp_data->cameratype, errstr);
            } else {
                MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
                    ,_("%s: Excessive tries to get data from camera %d")
                    ,rtsp_data->cameratype, nodata);
            }
            netcam_rtsp_free_pkt(rtsp_data);
            netcam_rtsp_close_context(rtsp_data);
            return -1;
        } else {
            if (rtsp_data->record_active) {
                retcd = netcam_rtsp_record_enqueue(rtsp_data, rtsp_data->packet_recv);
                if (retcd < 0) {
                    /* Queueing failed; keep capture alive and continue image processing. */
                    retcd = 0;
                }
            }

            if (rtsp_data->packet_recv->stream_index == rtsp_data->video_stream_index) {
                /* For a high resolution pass-through we don't decode the image */
                if (rtsp_data->high_resolution && rtsp_data->passthrough) {
                    if (rtsp_data->packet_recv->data != NULL) {
                        size_decoded = 1;
                    }
                } else {
                    size_decoded = netcam_rtsp_decode_packet(rtsp_data);
                }
            } else if ((rtsp_data->packet_recv->stream_index == rtsp_data->audio_stream_index) &&
                       rtsp_data->audio_sync_init) {
                if (netcam_rtsp_audio_enqueue(rtsp_data, rtsp_data->packet_recv) < 0) {
                    MOTION_LOG(WRN, TYPE_NETCAM, NO_ERRNO
                        ,_("%s: Failed to enqueue audio packet")
                        , rtsp_data->cameratype);
                }
            }

            if (size_decoded > 0) {
                haveimage = TRUE;
            } else if (size_decoded == 0) {
                if (rtsp_data->packet_recv->stream_index != rtsp_data->video_stream_index) {
                    netcam_rtsp_free_pkt(rtsp_data);
                    rtsp_data->packet_recv = my_packet_alloc(rtsp_data->packet_recv);
                    continue;
                }
                /* Did not fail, just didn't get anything.  Try again */
                nodata++;
                netcam_rtsp_free_pkt(rtsp_data);
                rtsp_data->packet_recv = my_packet_alloc(rtsp_data->packet_recv);
            } else {
                netcam_rtsp_free_pkt(rtsp_data);
                netcam_rtsp_close_context(rtsp_data);
                return -1;
            }
        }
    }

    if (gettimeofday(&rtsp_data->img_recv->image_time, NULL) < 0) {
        MOTION_LOG(ERR, TYPE_NETCAM, SHOW_ERRNO, "gettimeofday");
    }

    /* Skip status change on our first image to keep the "next" function waiting
     * until the handler thread gets going
     */
    if (!rtsp_data->first_image) {
        rtsp_data->status = RTSP_CONNECTED;
    }

    /* Skip resize/pix format for high pass-through */
    if (!(rtsp_data->high_resolution && rtsp_data->passthrough)) {
        if ((rtsp_data->imgsize.width  != rtsp_data->frame->width) ||
            (rtsp_data->imgsize.height != rtsp_data->frame->height) ||
            (netcam_rtsp_check_pixfmt(rtsp_data) != 0)) {
            if (netcam_rtsp_resize(rtsp_data) < 0) {
                netcam_rtsp_free_pkt(rtsp_data);
                netcam_rtsp_close_context(rtsp_data);
                return -1;
            }
        }
    }

    pthread_mutex_lock(&rtsp_data->mutex);
        rtsp_data->idnbr++;
        if (rtsp_data->passthrough) {
            netcam_rtsp_pktarray_add(rtsp_data);
        }
        if (!(rtsp_data->high_resolution && rtsp_data->passthrough)) {
            xchg = rtsp_data->img_latest;
            rtsp_data->img_latest = rtsp_data->img_recv;
            rtsp_data->img_recv = xchg;
        }
    pthread_mutex_unlock(&rtsp_data->mutex);

    netcam_rtsp_free_pkt(rtsp_data);

    if (rtsp_data->format_context->streams[rtsp_data->video_stream_index]->avg_frame_rate.den > 0) {
        rtsp_data->src_fps = (
            (rtsp_data->format_context->streams[rtsp_data->video_stream_index]->avg_frame_rate.num /
            rtsp_data->format_context->streams[rtsp_data->video_stream_index]->avg_frame_rate.den) +
            0.5);
        if (rtsp_data->capture_rate < 1) {
            rtsp_data->capture_rate = rtsp_data->src_fps + 1;
            MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
                    ,_("%s: capture_rate not specified in netcam_params. Using %d")
                    ,rtsp_data->cameratype,rtsp_data->capture_rate);
        }
    } else {
        if (rtsp_data->capture_rate < 1) {
            rtsp_data->capture_rate = rtsp_data->conf->framerate;
            MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
                    ,_("%s: capture_rate not specified in netcam_params. Using framerate %d")
                    ,rtsp_data->cameratype,rtsp_data->capture_rate);
        }
    }

    return 0;
}

static int netcam_rtsp_ntc(struct rtsp_context *rtsp_data)
{

    if ((rtsp_data->finish) ||
        (!rtsp_data->first_image) ||
        (rtsp_data->high_resolution && rtsp_data->passthrough)) {
        return 0;
    }

    if ((rtsp_data->imgsize.width  != rtsp_data->frame->width) ||
        (rtsp_data->imgsize.height != rtsp_data->frame->height) ||
        (netcam_rtsp_check_pixfmt(rtsp_data) != 0)) {
        MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO, "");
        MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO, "******************************************************");
        if ((rtsp_data->imgsize.width  != rtsp_data->frame->width) ||
            (rtsp_data->imgsize.height != rtsp_data->frame->height)) {
            if (netcam_rtsp_check_pixfmt(rtsp_data) != 0) {
                MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO, _("The network camera is sending pictures in a different"));
                MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO, _("size than specified in the config and also a "));
                MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO, _("different picture format.  The picture is being"));
                MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO, _("transcoded to YUV420P and into the size requested"));
                MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO, _("in the config file.  If possible change netcam to"));
                MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO, _("be in YUV420P format and the size requested in the"));
                MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO, _("config to possibly lower CPU usage."));
            } else {
                MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO, _("The network camera is sending pictures in a different"));
                MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO, _("size than specified in the configuration file."));
                MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO, _("The picture is being transcoded into the size "));
                MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO, _("requested in the configuration.  If possible change"));
                MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO, _("netcam or configuration to indicate the same size"));
                MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO, _("to possibly lower CPU usage."));
            }
            MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO, _("Netcam: %d x %d => Config: %d x %d")
            ,rtsp_data->frame->width,rtsp_data->frame->height
            ,rtsp_data->imgsize.width,rtsp_data->imgsize.height);
        } else {
            MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO, _("The image sent is being "));
            MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO, _("trancoded to YUV420P.  If possible change netcam "));
            MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO, _("picture format to YUV420P to possibly lower CPU usage."));
        }
        MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO, "******************************************************");
        MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO, "");
    }

    return 0;

}

static void netcam_rtsp_set_options(struct rtsp_context *rtsp_data)
{

    int indx;
    char *tmpval;

    /* The log messages are a bit short in this function intentionally.
     * The function name is printed in each message so that is being
     * considered as part of the message.
     */

    tmpval = mymalloc(PATH_MAX);

    if ((strncmp(rtsp_data->service, "rtsp", 4) == 0 ) ||
        (strncmp(rtsp_data->service, "rtmp", 4) == 0 )) {
        MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO,_("%s: Setting rtsp/rtmp")
            ,rtsp_data->cameratype);
        util_parms_add_default(rtsp_data->parameters,"rtsp_transport","tcp");

    } else if (strncmp(rtsp_data->service, "http", 4) == 0 ) {
        MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
            ,_("%s: Setting input_format mjpeg"),rtsp_data->cameratype);
        rtsp_data->format_context->iformat = av_find_input_format("mjpeg");
        util_parms_add_default(rtsp_data->parameters,"reconnect_on_network_error","1");
        util_parms_add_default(rtsp_data->parameters,"reconnect_at_eof","1");
        util_parms_add_default(rtsp_data->parameters,"reconnect","1");
        util_parms_add_default(rtsp_data->parameters,"multiple_requests","1");
        util_parms_add_default(rtsp_data->parameters,"reconnect_streamed","1");

    } else if (strncmp(rtsp_data->service, "v4l2", 4) == 0 ) {
        MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
            ,_("%s: Setting input_format video4linux2"),rtsp_data->cameratype);
        rtsp_data->format_context->iformat = av_find_input_format("video4linux2");

        sprintf(tmpval,"%d",rtsp_data->conf->framerate);
        util_parms_add_default(rtsp_data->parameters,"framerate", tmpval);

        sprintf(tmpval,"%dx%d",rtsp_data->conf->width, rtsp_data->conf->height);
        util_parms_add_default(rtsp_data->parameters,"video_size", tmpval);

        /* Allow a bit more time for the v4l2 device to start up */
        rtsp_data->cnt->watchdog = 60;
        rtsp_data->interruptduration = 55;


    } else if (strncmp(rtsp_data->service, "file", 4) == 0 ) {
        MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
            ,_("%s: Setting up movie file"),rtsp_data->cameratype);

    } else {
        MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO,_("%s: Setting up %s")
            ,rtsp_data->cameratype, rtsp_data->service);
    }

    free(tmpval);

    /* Write the options to the context, while skipping the Motion ones */
    for (indx = 0; indx < rtsp_data->parameters->params_count; indx++) {
        if (mystrne(rtsp_data->parameters->params_array[indx].param_name,"decoder") &&
            mystrne(rtsp_data->parameters->params_array[indx].param_name,"capture_rate")) {
            av_dict_set(&rtsp_data->opts
                , rtsp_data->parameters->params_array[indx].param_name
                , rtsp_data->parameters->params_array[indx].param_value
                , 0);
            if (rtsp_data->status == RTSP_NOTCONNECTED) {
                MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO,_("%s: option: %s = %s")
                    ,rtsp_data->cameratype
                    ,rtsp_data->parameters->params_array[indx].param_name
                    ,rtsp_data->parameters->params_array[indx].param_value
                );
            }
        }
    }

}

static void netcam_rtsp_set_path (struct context *cnt, struct rtsp_context *rtsp_data )
{

    char        *userpass = NULL;
    struct url_t url;

    rtsp_data->path = NULL;
    rtsp_data->service = NULL;

    memset(&url, 0, sizeof(url));

    if (rtsp_data->high_resolution) {
        netcam_url_parse(&url, cnt->conf.netcam_high_url);
    } else {
        netcam_url_parse(&url, cnt->conf.netcam_url);
    }

    if (cnt->conf.netcam_userpass != NULL) {
        userpass = mystrdup(cnt->conf.netcam_userpass);
    } else if (url.userpass != NULL) {
        userpass = mystrdup(url.userpass);
    }

    if (mystreq(url.service, "v4l2")) {
        rtsp_data->path = mymalloc(strlen(url.path) + 1);
        sprintf(rtsp_data->path, "%s",url.path);
        MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
            ,_("Setting up v4l2 via netcam"));
    } else if (mystreq(url.service, "file")) {
        rtsp_data->path = mymalloc(strlen(url.path) + 1);
        sprintf(rtsp_data->path, "%s",url.path);
        MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
            ,_("Setting up file via netcam"));
    } else {
        MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
            ,_("Setting up %s via netcam"),url.service);
        if (userpass != NULL) {
            rtsp_data->path = mymalloc(strlen(url.service) + 3 + strlen(userpass)
                  + 1 + strlen(url.host) + 6 + strlen(url.path) + 2 );
            if (url.port > 0) {
                sprintf((char *)rtsp_data->path, "%s://%s@%s:%d%s",
                    url.service, userpass, url.host, url.port, url.path);
            } else {
                sprintf((char *)rtsp_data->path, "%s://%s@%s%s",
                    url.service, userpass, url.host, url.path);
            }
        } else {
            rtsp_data->path = mymalloc(strlen(url.service) + 3 + strlen(url.host)
                  + 6 + strlen(url.path) + 2);
            if (url.port > 0) {
                sprintf((char *)rtsp_data->path, "%s://%s:%d%s"
                    , url.service, url.host, url.port, url.path);
            } else {
                sprintf((char *)rtsp_data->path, "%s://%s%s"
                    , url.service, url.host, url.path);
            }
        }
    }

    rtsp_data->service = mymalloc(strlen(url.service)+1);
    sprintf(rtsp_data->service, "%s",url.service);

    netcam_url_free(&url);
    if (userpass) {
        free (userpass);
    }

}

static void netcam_rtsp_set_parms (struct context *cnt, struct rtsp_context *rtsp_data )
{
    /* Set the parameters to be used with our camera */

    int indx, val_len;

    rtsp_data->conf = &cnt->conf;

    if (rtsp_data->high_resolution) {
        rtsp_data->imgsize.width = 0;
        rtsp_data->imgsize.height = 0;
        snprintf(rtsp_data->cameratype,29, "%s",_("highres"));
        rtsp_data->parameters = mymalloc(sizeof(struct params_context));
        rtsp_data->parameters->update_params = TRUE;
        util_parms_parse(rtsp_data->parameters,(char*)cnt->conf.netcam_high_params, TRUE);
    } else {
        rtsp_data->imgsize.width = cnt->conf.width;
        rtsp_data->imgsize.height = cnt->conf.height;
        snprintf(rtsp_data->cameratype,29, "%s",_("norm"));
        rtsp_data->parameters = mymalloc(sizeof(struct params_context));
        rtsp_data->parameters->update_params = TRUE;
        util_parms_parse(rtsp_data->parameters, (char*)cnt->conf.netcam_params, TRUE);
    }
    MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
        ,_("Setting up %s stream."),rtsp_data->cameratype);

    rtsp_data->status = RTSP_NOTCONNECTED;

    util_parms_add_default(rtsp_data->parameters,"decoder","NULL");

    rtsp_data->camera_name = cnt->conf.camera_name;
    rtsp_data->img_recv = mymalloc(sizeof(netcam_buff));
    rtsp_data->img_recv->ptr = mymalloc(NETCAM_BUFFSIZE);
    rtsp_data->img_latest = mymalloc(sizeof(netcam_buff));
    rtsp_data->img_latest->ptr = mymalloc(NETCAM_BUFFSIZE);
    rtsp_data->pktarray_size = 0;
    rtsp_data->pktarray_index = -1;
    rtsp_data->pktarray = NULL;
    rtsp_data->packet_recv = NULL;
    rtsp_data->handler_finished = TRUE;
    rtsp_data->first_image = TRUE;
    rtsp_data->reconnect_count = 0;
    rtsp_data->record_format = NULL;
    rtsp_data->record_stream_map = NULL;
    rtsp_data->record_stream_map_size = 0;
    rtsp_data->record_last_dts = NULL;
    rtsp_data->record_last_pts = NULL;
    rtsp_data->record_base_dts = NULL;
    rtsp_data->record_base_pts = NULL;
    rtsp_data->record_active = FALSE;
    rtsp_data->record_pktqueue = NULL;
    rtsp_data->record_pktqueue_size = 0;
    rtsp_data->record_pktqueue_head = 0;
    rtsp_data->record_pktqueue_tail = 0;
    rtsp_data->record_pktqueue_count = 0;
    rtsp_data->record_thread_finish = FALSE;
    rtsp_data->record_thread_finished = TRUE;
    rtsp_data->record_sync_init = FALSE;
    rtsp_data->audio_codec_context = NULL;
    rtsp_data->audio_frame = NULL;
    rtsp_data->audio_pktqueue = NULL;
    rtsp_data->audio_pktqueue_size = 0;
    rtsp_data->audio_pktqueue_head = 0;
    rtsp_data->audio_pktqueue_tail = 0;
    rtsp_data->audio_pktqueue_count = 0;
    rtsp_data->audio_thread_finish = FALSE;
    rtsp_data->audio_thread_finished = TRUE;
    rtsp_data->audio_sync_init = FALSE;
    rtsp_data->audio_stream_index = -1;
    rtsp_data->audio_swr = NULL;
    rtsp_data->audio_sample_rate = 0;
    rtsp_data->audio_channels = 0;
    rtsp_data->audio_fft_in = NULL;
    rtsp_data->audio_fft_out = NULL;
    rtsp_data->audio_fft_size = 0;
    rtsp_data->audio_fft_plan_ready = FALSE;
    rtsp_data->audio_trigger_hits = 0;
    rtsp_data->audio_trigger_last_ts = 0;
    rtsp_data->cnt = cnt;
    rtsp_data->src_fps =  -99; /* Default to invalid value so we can test for whether real value exist */

    rtsp_data->capture_rate = -1;
    for (indx = 0; indx < rtsp_data->parameters->params_count; indx++) {
        if ( mystreq(rtsp_data->parameters->params_array[indx].param_name,"decoder")) {
            val_len = strlen(rtsp_data->parameters->params_array[indx].param_value) + 1;
            rtsp_data->decoder_nm = mymalloc(val_len);
            snprintf(rtsp_data->decoder_nm, val_len
                , "%s",rtsp_data->parameters->params_array[indx].param_value);
        }

        if ( mystreq(rtsp_data->parameters->params_array[indx].param_name,"capture_rate")) {
            rtsp_data->capture_rate = atoi(rtsp_data->parameters->params_array[indx].param_value);
        }

    }

    /* If this is the norm and we have a highres, then disable passthru on the norm */
    if ((!rtsp_data->high_resolution) && (cnt->conf.netcam_high_url)) {
        rtsp_data->passthrough = FALSE;
    } else {
        rtsp_data->passthrough = util_check_passthrough(cnt);
    }

    rtsp_data->interruptduration = 5;
    rtsp_data->interrupted = FALSE;
    if (gettimeofday(&rtsp_data->interruptstarttime, NULL) < 0) {
        MOTION_LOG(ERR, TYPE_NETCAM, SHOW_ERRNO, "gettimeofday");
    }
    if (gettimeofday(&rtsp_data->interruptcurrenttime, NULL) < 0) {
        MOTION_LOG(ERR, TYPE_NETCAM, SHOW_ERRNO, "gettimeofday");
    }
    if (gettimeofday(&rtsp_data->frame_curr_tm, NULL) < 0) {
        MOTION_LOG(ERR, TYPE_NETCAM, SHOW_ERRNO, "gettimeofday");
    }
    if (gettimeofday(&rtsp_data->frame_prev_tm, NULL) < 0) {
        MOTION_LOG(ERR, TYPE_NETCAM, SHOW_ERRNO, "gettimeofday");
    }

    snprintf(rtsp_data->threadname, 15, "%s",_("Unknown"));

    netcam_rtsp_set_path(cnt, rtsp_data);

}

static int netcam_rtsp_set_dimensions (struct context *cnt)
{

    cnt->imgs.width = 0;
    cnt->imgs.height = 0;
    cnt->imgs.size_norm = 0;
    cnt->imgs.motionsize = 0;

    cnt->imgs.width_high  = 0;
    cnt->imgs.height_high = 0;
    cnt->imgs.size_high   = 0;

    if (cnt->conf.width % 8) {
        MOTION_LOG(CRT, TYPE_NETCAM, NO_ERRNO
            ,_("Image width (%d) requested is not modulo 8."), cnt->conf.width);
        cnt->conf.width = cnt->conf.width - (cnt->conf.width % 8) + 8;
        MOTION_LOG(CRT, TYPE_NETCAM, NO_ERRNO
            ,_("Adjusting width to next higher multiple of 8 (%d)."), cnt->conf.width);
    }
    if (cnt->conf.height % 8) {
        MOTION_LOG(CRT, TYPE_NETCAM, NO_ERRNO
            ,_("Image height (%d) requested is not modulo 8."), cnt->conf.height);
        cnt->conf.height = cnt->conf.height - (cnt->conf.height % 8) + 8;
        MOTION_LOG(CRT, TYPE_NETCAM, NO_ERRNO
            ,_("Adjusting height to next higher multiple of 8 (%d)."), cnt->conf.height);
    }

    /* Fill in camera details into context structure. */
    cnt->imgs.width = cnt->conf.width;
    cnt->imgs.height = cnt->conf.height;
    cnt->imgs.size_norm = (cnt->conf.width * cnt->conf.height * 3) / 2;
    cnt->imgs.motionsize = cnt->conf.width * cnt->conf.height;

    return 0;
}

static int netcam_rtsp_copy_stream(struct rtsp_context *rtsp_data)
{
    /* Make a static copy of the stream information for use in passthrough processing */
    #if ( MYFFVER >= 57041)
        AVStream  *transfer_stream, *stream_in;
        int        retcd;

        pthread_mutex_lock(&rtsp_data->mutex_transfer);
            if (rtsp_data->transfer_format != NULL) {
                avformat_close_input(&rtsp_data->transfer_format);
            }
            rtsp_data->transfer_format = avformat_alloc_context();
            transfer_stream = avformat_new_stream(rtsp_data->transfer_format, NULL);
            stream_in = rtsp_data->format_context->streams[rtsp_data->video_stream_index];
            retcd = avcodec_parameters_copy(transfer_stream->codecpar, stream_in->codecpar);
            if (retcd < 0) {
                MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO
                    ,_("Unable to copy codec parameters"));
                pthread_mutex_unlock(&rtsp_data->mutex_transfer);
                return -1;
            }
            transfer_stream->time_base         = stream_in->time_base;
        pthread_mutex_unlock(&rtsp_data->mutex_transfer);

        MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO, _("Stream copied for pass-through"));
        return 0;
    #else

        AVStream  *transfer_stream, *stream_in;
        int        retcd;

        pthread_mutex_lock(&rtsp_data->mutex_transfer);
            if (rtsp_data->transfer_format != NULL) {
                avformat_close_input(&rtsp_data->transfer_format);
            }
            rtsp_data->transfer_format = avformat_alloc_context();
            transfer_stream = avformat_new_stream(rtsp_data->transfer_format, NULL);
            stream_in = rtsp_data->format_context->streams[rtsp_data->video_stream_index];
            retcd = avcodec_copy_context(transfer_stream->codec, stream_in->codec);
            if (retcd < 0) {
                MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO, _("Unable to copy codec parameters"));
                pthread_mutex_unlock(&rtsp_data->mutex_transfer);
                return -1;
            }
            transfer_stream->time_base         = stream_in->time_base;
        pthread_mutex_unlock(&rtsp_data->mutex_transfer);

        MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO, _("Stream copied for pass-through"));
        return 0;
    #endif

}

static int netcam_rtsp_open_context(struct rtsp_context *rtsp_data)
{

    int  retcd;
    char errstr[128];

    if (rtsp_data->finish) {
        return -1;
    }

    if (rtsp_data->path == NULL) {
        if (rtsp_data->status == RTSP_NOTCONNECTED) {
            MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO, _("Null path passed to connect"));
        }
        return -1;
    }

    rtsp_data->opts = NULL;
    rtsp_data->format_context = avformat_alloc_context();
    rtsp_data->format_context->interrupt_callback.callback = netcam_rtsp_interrupt;
    rtsp_data->format_context->interrupt_callback.opaque = rtsp_data;
    rtsp_data->interrupted = FALSE;

    if (gettimeofday(&rtsp_data->interruptstarttime, NULL) < 0) {
        MOTION_LOG(ERR, TYPE_NETCAM, SHOW_ERRNO, "gettimeofday");
    }

    rtsp_data->interruptduration = 20;

    netcam_rtsp_set_options(rtsp_data);

    retcd = avformat_open_input(&rtsp_data->format_context, rtsp_data->path, NULL, &rtsp_data->opts);
    if ((retcd < 0) || (rtsp_data->interrupted) || (rtsp_data->finish)) {
        if (rtsp_data->status == RTSP_NOTCONNECTED) {
            av_strerror(retcd, errstr, sizeof(errstr));
            MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO
                ,_("%s: Unable to open camera(%s): %s")
                , rtsp_data->cameratype, rtsp_data->camera_name, errstr);
        }
        av_dict_free(&rtsp_data->opts);
        if (rtsp_data->interrupted) {
            netcam_rtsp_close_context(rtsp_data);
        }
        return -1;
    }
    av_dict_free(&rtsp_data->opts);
    MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
        ,_("%s: Opened camera(%s)"), rtsp_data->cameratype, rtsp_data->camera_name);

    /* fill out stream information */
    retcd = avformat_find_stream_info(rtsp_data->format_context, NULL);
    if ((retcd < 0) || (rtsp_data->interrupted) || (rtsp_data->finish)) {
        if (rtsp_data->status == RTSP_NOTCONNECTED) {
            av_strerror(retcd, errstr, sizeof(errstr));
            MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO
                ,_("%s: Unable to find stream info: %s")
                ,rtsp_data->cameratype, errstr);
        }
        netcam_rtsp_close_context(rtsp_data);
        return -1;
    }

    /* there is no way to set the avcodec thread names, but they inherit
     * our thread name - so temporarily change our thread name to the
     * desired name */

    util_threadname_get(rtsp_data->threadname);

    util_threadname_set("av",rtsp_data->threadnbr,rtsp_data->camera_name);

    retcd = netcam_rtsp_open_codec(rtsp_data);

    util_threadname_set(NULL, 0, rtsp_data->threadname);

    if ((retcd < 0) || (rtsp_data->interrupted) || (rtsp_data->finish)) {
        if (rtsp_data->status == RTSP_NOTCONNECTED) {
            av_strerror(retcd, errstr, sizeof(errstr));
            MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO
                ,_("%s: Unable to open codec context: %s")
                ,rtsp_data->cameratype, errstr);
        }
        netcam_rtsp_close_context(rtsp_data);
        return -1;
    }

    if (rtsp_data->codec_context->width <= 0 ||
        rtsp_data->codec_context->height <= 0) {
        MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO
            ,_("%s: Camera image size is invalid"),rtsp_data->cameratype);
        netcam_rtsp_close_context(rtsp_data);
        return -1;
    }

    if (rtsp_data->high_resolution) {
        rtsp_data->imgsize.width = rtsp_data->codec_context->width;
        rtsp_data->imgsize.height = rtsp_data->codec_context->height;
    }

    rtsp_data->frame = my_frame_alloc();
    if (rtsp_data->frame == NULL) {
        if (rtsp_data->status == RTSP_NOTCONNECTED) {
            MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO
                ,_("%s: Unable to allocate frame."),rtsp_data->cameratype);
        }
        netcam_rtsp_close_context(rtsp_data);
        return -1;
    }

    if (rtsp_data->passthrough) {
        retcd = netcam_rtsp_copy_stream(rtsp_data);
        if ((retcd < 0) || (rtsp_data->interrupted)) {
            if (rtsp_data->status == RTSP_NOTCONNECTED) {
                MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO
                    ,_("%s: Failed to copy stream for pass-through.")
                    ,rtsp_data->cameratype);
            }
            rtsp_data->passthrough = FALSE;
        }
    }

    /* Validate that the previous steps opened the camera */
    retcd = netcam_rtsp_read_image(rtsp_data);
    if ((retcd < 0) || (rtsp_data->interrupted)) {
        if (rtsp_data->status == RTSP_NOTCONNECTED) {
            MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO
                ,_("%s: Failed to read first image"),rtsp_data->cameratype);
        }
        netcam_rtsp_close_context(rtsp_data);
        return -1;
    }

    return 0;

}

static int netcam_rtsp_connect(struct rtsp_context *rtsp_data)
{

    if (netcam_rtsp_open_context(rtsp_data) < 0) {
        return -1;
    }

    if (netcam_rtsp_ntc(rtsp_data) < 0) {
        return -1;
    }

    if (netcam_rtsp_read_image(rtsp_data) < 0) {
        return -1;
    }

    /* We use the status for determining whether to grab a image from
     * the Motion loop(see "next" function).  When we are initially starting,
     * we open and close the context and during this process we do not want the
     * Motion loop to start quite yet on this first image so we do
     * not set the status to connected
     */
    if (!rtsp_data->first_image) {

        rtsp_data->status = RTSP_CONNECTED;

        MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO
            ,_("%s: Camera (%s) connected")
            ,rtsp_data->cameratype,rtsp_data->camera_name);

        MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
            ,_("%s: Netcam capture_rate is %d.")
            ,rtsp_data->cameratype, rtsp_data->capture_rate);

        if (rtsp_data->src_fps > 0) {
            MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
                ,_("%s: Camera source is %d FPS")
                ,rtsp_data->cameratype, rtsp_data->src_fps);
        } else {
            MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO
                ,_("%s: Unable to determine the camera source FPS.")
                ,rtsp_data->cameratype);
        }

        if (rtsp_data->capture_rate < rtsp_data->src_fps) {
            MOTION_LOG(WRN, TYPE_NETCAM, NO_ERRNO
                ,_("%s: capture_rate is less than camera FPS.")
                , rtsp_data->cameratype);
            MOTION_LOG(WRN, TYPE_NETCAM, NO_ERRNO
                ,_("%s: Decoding errors will occur.")
                , rtsp_data->cameratype);
            MOTION_LOG(WRN, TYPE_NETCAM, NO_ERRNO
                , _("%s: capture_rate slightly larger than camera FPS is recommended.")
                , rtsp_data->cameratype);
        }
    }

    return 0;
}

static void netcam_rtsp_shutdown(struct rtsp_context *rtsp_data)
{

    if (rtsp_data) {
        if (rtsp_data->audio_sync_init) {
            int wait_limit;
            int wait_elapsed = 0;

            pthread_mutex_lock(&rtsp_data->mutex_audioq);
                rtsp_data->audio_thread_finish = TRUE;
                pthread_cond_broadcast(&rtsp_data->cond_audioq);
            pthread_mutex_unlock(&rtsp_data->mutex_audioq);

            if ((rtsp_data->cnt != NULL) && (rtsp_data->cnt->conf.watchdog_tmo > 1)) {
                wait_limit = rtsp_data->cnt->conf.watchdog_tmo - 1;
            } else {
                wait_limit = 5;
            }

            pthread_mutex_lock(&rtsp_data->mutex_audioq);
                while ((!rtsp_data->audio_thread_finished) && (wait_elapsed < wait_limit)) {
                    pthread_mutex_unlock(&rtsp_data->mutex_audioq);
                    SLEEP(1,0);
                    wait_elapsed++;
                    pthread_mutex_lock(&rtsp_data->mutex_audioq);
                }
            pthread_mutex_unlock(&rtsp_data->mutex_audioq);

            if (!rtsp_data->audio_thread_finished) {
                MOTION_LOG(WRN, TYPE_NETCAM, NO_ERRNO
                    ,_("%s: Timed out waiting for RTSP audio analysis to stop; forcing close")
                    , rtsp_data->cameratype);
            }

            pthread_mutex_lock(&rtsp_data->mutex_audioq);
                netcam_rtsp_audio_queue_clear_lck(rtsp_data);
            pthread_mutex_unlock(&rtsp_data->mutex_audioq);

            pthread_mutex_lock(&rtsp_data->mutex_audio);
                netcam_rtsp_audio_close_lck(rtsp_data);
            pthread_mutex_unlock(&rtsp_data->mutex_audio);

            if (rtsp_data->audio_pktqueue != NULL) {
                free(rtsp_data->audio_pktqueue);
                rtsp_data->audio_pktqueue = NULL;
            }
            rtsp_data->audio_pktqueue_size = 0;
        }

        netcam_rtsp_close_context(rtsp_data);

        if (rtsp_data->path != NULL) {
            free(rtsp_data->path);
        }
        rtsp_data->path = NULL;

        if (rtsp_data->service != NULL) {
            free(rtsp_data->service);
        }
        rtsp_data->service = NULL;

        if (rtsp_data->img_latest != NULL) {
            free(rtsp_data->img_latest->ptr);
            free(rtsp_data->img_latest);
        }
        rtsp_data->img_latest = NULL;

        if (rtsp_data->img_recv != NULL) {
            free(rtsp_data->img_recv->ptr);
            free(rtsp_data->img_recv);
        }
        rtsp_data->img_recv   = NULL;

        if (rtsp_data->decoder_nm != NULL) {
            free(rtsp_data->decoder_nm);
        }
        rtsp_data->decoder_nm = NULL;

        util_parms_free (rtsp_data->parameters);

        if (rtsp_data->parameters != NULL) {
            free(rtsp_data->parameters);
        }
        rtsp_data->parameters = NULL;

    }

}

static void netcam_rtsp_handler_wait(struct rtsp_context *rtsp_data)
{

    long usec_maxrate, usec_delay;

    if (rtsp_data->capture_rate < 1 ) {
        rtsp_data->capture_rate = 1;
    }

    usec_maxrate = (1000000L / rtsp_data->capture_rate);

    if (gettimeofday(&rtsp_data->frame_curr_tm, NULL) < 0) {
        MOTION_LOG(ERR, TYPE_NETCAM, SHOW_ERRNO, "gettimeofday");
    }

    usec_delay = usec_maxrate -
        ((rtsp_data->frame_curr_tm.tv_sec - rtsp_data->frame_prev_tm.tv_sec) * 1000000L) -
        (rtsp_data->frame_curr_tm.tv_usec - rtsp_data->frame_prev_tm.tv_usec);
    if ((usec_delay > 0) && (usec_delay < 1000000L)) {
        SLEEP(0, usec_delay * 1000);
    }

}

static void netcam_rtsp_handler_reconnect(struct rtsp_context *rtsp_data)
{

    int retcd;

    if ((rtsp_data->status == RTSP_CONNECTED) ||
        (rtsp_data->status == RTSP_READINGIMAGE)) {
        MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO
            ,_("%s: Reconnecting with camera...."),rtsp_data->cameratype);
    }
    rtsp_data->status = RTSP_RECONNECTING;

    /*
    * The retry count of 100 is arbritrary.
    * We want to try many times quickly to not lose too much information
    * before we go into the long wait phase
    */
    retcd = netcam_rtsp_connect(rtsp_data);
    if (retcd < 0) {
        if (rtsp_data->reconnect_count < 100) {
            rtsp_data->reconnect_count++;
        } else if (rtsp_data->reconnect_count == 100) {
            MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO
                ,_("%s: Camera did not reconnect."), rtsp_data->cameratype);
            MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO
                ,_("%s: Checking for camera every 10 seconds."),rtsp_data->cameratype);
            rtsp_data->reconnect_count++;
            SLEEP(10,0);
        } else {
            SLEEP(10,0);
        }
    } else {
        rtsp_data->reconnect_count = 0;
    }

}

static void *netcam_rtsp_handler(void *arg)
{

    struct rtsp_context *rtsp_data = arg;

    rtsp_data->handler_finished = FALSE;

    util_threadname_set("nc",rtsp_data->threadnbr, rtsp_data->camera_name);

    pthread_setspecific(tls_key_threadnr, (void *)((unsigned long)rtsp_data->threadnbr));

    MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO
        ,_("%s: Camera handler thread [%d] started")
        ,rtsp_data->cameratype, rtsp_data->threadnbr);

    while (!rtsp_data->finish) {
        if (!rtsp_data->format_context) {      /* We must have disconnected.  Try to reconnect */
            if (gettimeofday(&rtsp_data->frame_prev_tm, NULL) < 0) {
                MOTION_LOG(ERR, TYPE_NETCAM, SHOW_ERRNO, "gettimeofday");
            }
            netcam_rtsp_handler_reconnect(rtsp_data);
            continue;
        } else {            /* We think we are connected...*/
            if (gettimeofday(&rtsp_data->frame_prev_tm, NULL) < 0) {
                MOTION_LOG(ERR, TYPE_NETCAM, SHOW_ERRNO, "gettimeofday");
            }
            if (netcam_rtsp_read_image(rtsp_data) < 0) {
                if (!rtsp_data->finish) {   /* Nope.  We are not or got bad image.  Reconnect*/
                    netcam_rtsp_handler_reconnect(rtsp_data);
                }
                continue;
            }
            netcam_rtsp_handler_wait(rtsp_data);
        }
    }

    MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
        ,_("%s: Handler loop finished."),rtsp_data->cameratype);
    netcam_rtsp_shutdown(rtsp_data);

    SLEEP(1,0);

    /* Our thread is finished - decrement motion's thread count. */
    pthread_mutex_lock(&global_lock);
        threads_running--;
    pthread_mutex_unlock(&global_lock);

    MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
        ,_("netcam camera handler: finish set, exiting"));
    rtsp_data->handler_finished = TRUE;

    pthread_exit(NULL);
}

static int netcam_rtsp_start_handler(struct rtsp_context *rtsp_data)
{

    int retcd, wait_counter;
    pthread_attr_t handler_attribute;
    pthread_attr_t record_attribute;
    pthread_attr_t audio_attribute;

    pthread_mutex_init(&rtsp_data->mutex, NULL);
    pthread_mutex_init(&rtsp_data->mutex_pktarray, NULL);
    pthread_mutex_init(&rtsp_data->mutex_transfer, NULL);
    pthread_mutex_init(&rtsp_data->mutex_record, NULL);
    pthread_mutex_init(&rtsp_data->mutex_recordq, NULL);
    pthread_cond_init(&rtsp_data->cond_recordq, NULL);
    rtsp_data->record_sync_init = TRUE;

    pthread_mutex_init(&rtsp_data->mutex_audio, NULL);
    pthread_mutex_init(&rtsp_data->mutex_audioq, NULL);
    pthread_cond_init(&rtsp_data->cond_audioq, NULL);
    rtsp_data->audio_sync_init = TRUE;

    rtsp_data->record_pktqueue_size = RTSP_RECORD_QUEUE_SIZE;
    rtsp_data->record_pktqueue = mymalloc(sizeof(AVPacket *) * rtsp_data->record_pktqueue_size);
    memset(rtsp_data->record_pktqueue, 0, sizeof(AVPacket *) * rtsp_data->record_pktqueue_size);
    rtsp_data->record_pktqueue_head = 0;
    rtsp_data->record_pktqueue_tail = 0;
    rtsp_data->record_pktqueue_count = 0;
    rtsp_data->record_thread_finish = FALSE;
    rtsp_data->record_thread_finished = FALSE;

    rtsp_data->audio_pktqueue_size = RTSP_AUDIO_QUEUE_SIZE;
    rtsp_data->audio_pktqueue = mymalloc(sizeof(AVPacket *) * rtsp_data->audio_pktqueue_size);
    memset(rtsp_data->audio_pktqueue, 0, sizeof(AVPacket *) * rtsp_data->audio_pktqueue_size);
    rtsp_data->audio_pktqueue_head = 0;
    rtsp_data->audio_pktqueue_tail = 0;
    rtsp_data->audio_pktqueue_count = 0;
    rtsp_data->audio_thread_finish = FALSE;
    rtsp_data->audio_thread_finished = FALSE;

    pthread_attr_init(&handler_attribute);
    pthread_attr_setdetachstate(&handler_attribute, PTHREAD_CREATE_DETACHED);

    pthread_attr_init(&record_attribute);
    pthread_attr_setdetachstate(&record_attribute, PTHREAD_CREATE_DETACHED);

    pthread_attr_init(&audio_attribute);
    pthread_attr_setdetachstate(&audio_attribute, PTHREAD_CREATE_DETACHED);

    pthread_mutex_lock(&global_lock);
        rtsp_data->threadnbr = ++threads_running;
    pthread_mutex_unlock(&global_lock);

    retcd = pthread_create(&rtsp_data->record_thread_id, &record_attribute, &netcam_rtsp_record_handler, rtsp_data);
    if (retcd < 0) {
        MOTION_LOG(ALR, TYPE_NETCAM, SHOW_ERRNO
            ,_("%s: Error starting record thread"),rtsp_data->cameratype);
        pthread_attr_destroy(&handler_attribute);
        pthread_attr_destroy(&record_attribute);
        pthread_attr_destroy(&audio_attribute);
        return -1;
    }
    pthread_attr_destroy(&record_attribute);

    retcd = pthread_create(&rtsp_data->audio_thread_id, &audio_attribute, &netcam_rtsp_audio_handler, rtsp_data);
    if (retcd < 0) {
        MOTION_LOG(ALR, TYPE_NETCAM, SHOW_ERRNO
            ,_("%s: Error starting audio thread"),rtsp_data->cameratype);
        pthread_attr_destroy(&handler_attribute);
        pthread_attr_destroy(&audio_attribute);
        pthread_mutex_lock(&rtsp_data->mutex_recordq);
            rtsp_data->record_thread_finish = TRUE;
            pthread_cond_broadcast(&rtsp_data->cond_recordq);
        pthread_mutex_unlock(&rtsp_data->mutex_recordq);
        pthread_mutex_lock(&rtsp_data->mutex_audioq);
            rtsp_data->audio_thread_finish = TRUE;
            pthread_cond_broadcast(&rtsp_data->cond_audioq);
        pthread_mutex_unlock(&rtsp_data->mutex_audioq);
        return -1;
    }
    pthread_attr_destroy(&audio_attribute);

    retcd = pthread_create(&rtsp_data->thread_id, &handler_attribute, &netcam_rtsp_handler, rtsp_data);
    if (retcd < 0) {
        MOTION_LOG(ALR, TYPE_NETCAM, SHOW_ERRNO
            ,_("%s: Error starting handler thread"),rtsp_data->cameratype);
        pthread_attr_destroy(&handler_attribute);
        pthread_mutex_lock(&rtsp_data->mutex_recordq);
            rtsp_data->record_thread_finish = TRUE;
            pthread_cond_broadcast(&rtsp_data->cond_recordq);
        pthread_mutex_unlock(&rtsp_data->mutex_recordq);
        pthread_mutex_lock(&rtsp_data->mutex_audioq);
            rtsp_data->audio_thread_finish = TRUE;
            pthread_cond_broadcast(&rtsp_data->cond_audioq);
        pthread_mutex_unlock(&rtsp_data->mutex_audioq);
        return -1;
    }
    pthread_attr_destroy(&handler_attribute);


    /* Now give a few tries to check that an image has been captured.
     * This ensures that by the time the setup routine exits, the
     * handler is completely set up and has images available
     */
    wait_counter = 60;
    while (wait_counter > 0) {
        pthread_mutex_lock(&rtsp_data->mutex);
            if (rtsp_data->img_latest->ptr != NULL) {
                wait_counter = -1;
            }
        pthread_mutex_unlock(&rtsp_data->mutex);

        if (wait_counter > 0) {
            MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
                ,_("%s: Waiting for first image from the handler."),rtsp_data->cameratype);
            SLEEP(0,5000000);
            wait_counter--;
        }
    }

    return 0;

}

/*********************************************************
 *  This ends the section of functions that rely upon FFmpeg
 ***********************************************************/
#endif /* End HAVE_FFMPEG */


int netcam_rtsp_setup(struct context *cnt)
{
    #ifdef HAVE_FFMPEG

        int retcd;
        int indx_cam, indx_max;
        struct rtsp_context *rtsp_data;

        cnt->rtsp = NULL;
        cnt->rtsp_high = NULL;

        if (netcam_rtsp_set_dimensions(cnt) < 0) {
            return -1;
        }

        indx_cam = 1;
        if (cnt->conf.netcam_high_url) {
            indx_max = 2;
        } else {
            indx_max = 1;
        }

        while (indx_cam <= indx_max) {
            if (indx_cam == 1) {
                cnt->rtsp = rtsp_new_context();
                if (cnt->rtsp == NULL) {
                    MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO
                        ,_("unable to create rtsp context"));
                    return -1;
                }
                rtsp_data = cnt->rtsp;
                rtsp_data->high_resolution = FALSE;           /* Set flag for this being the normal resolution camera */
            } else {
                cnt->rtsp_high = rtsp_new_context();
                if (cnt->rtsp_high == NULL) {
                    MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO
                        ,_("unable to create rtsp high context"));
                    return -1;
                }
                rtsp_data = cnt->rtsp_high;
                rtsp_data->high_resolution = TRUE;            /* Set flag for this being the high resolution camera */
            }

            netcam_rtsp_null_context(rtsp_data);

            netcam_rtsp_set_parms(cnt, rtsp_data);

            if (netcam_rtsp_connect(rtsp_data) < 0) {
                return -1;
            }

            retcd = netcam_rtsp_read_image(rtsp_data);
            if (retcd < 0) {
                MOTION_LOG(CRT, TYPE_NETCAM, NO_ERRNO
                    ,_("Failed trying to read first image - retval:%d"), retcd);
                rtsp_data->status = RTSP_NOTCONNECTED;
                return -1;
            }
            /* When running dual, there seems to be contamination across norm/high with codec functions. */
            netcam_rtsp_close_context(rtsp_data);       /* Close in this thread to open it again within handler thread */
            rtsp_data->status = RTSP_RECONNECTING;      /* Set as reconnecting to avoid excess messages when starting */
            rtsp_data->first_image = FALSE;             /* Set flag that we are not processing our first image */

            /* For normal resolution, we resize the image to the config parms so we do not need
            * to set the dimension parameters here (it is done in the set_parms).  For high res
            * we must get the dimensions from the first image captured
            */
            if (rtsp_data->high_resolution) {
                cnt->imgs.width_high = rtsp_data->imgsize.width;
                cnt->imgs.height_high = rtsp_data->imgsize.height;
            }

            if (netcam_rtsp_start_handler(rtsp_data) < 0) {
                return -1;
            }

            indx_cam++;
        }

        return 0;

    #else  /* No FFmpeg/Libav */
        (void)cnt;
        MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO, _("No FFmpeg support"));
        return -1;
    #endif /* End #ifdef HAVE_FFMPEG */
}

int netcam_rtsp_next(struct context *cnt, struct image_data *img_data)
{
    #ifdef HAVE_FFMPEG
        /* This is called from the motion loop thread */

        if ((cnt->rtsp->status == RTSP_RECONNECTING) ||
            (cnt->rtsp->status == RTSP_NOTCONNECTED)) {
                return 1;
        }
        pthread_mutex_lock(&cnt->rtsp->mutex);
            netcam_rtsp_pktarray_resize(cnt, FALSE);
            memcpy(img_data->image_norm
                , cnt->rtsp->img_latest->ptr
                , cnt->rtsp->img_latest->used);
            img_data->idnbr_norm = cnt->rtsp->idnbr;
        pthread_mutex_unlock(&cnt->rtsp->mutex);

        if (cnt->rtsp_high) {
            if ((cnt->rtsp_high->status == RTSP_RECONNECTING) ||
                (cnt->rtsp_high->status == RTSP_NOTCONNECTED)) {
                return 1;
            }
            pthread_mutex_lock(&cnt->rtsp_high->mutex);
                netcam_rtsp_pktarray_resize(cnt, TRUE);
                if (!cnt->rtsp_high->passthrough) {
                    memcpy(img_data->image_high
                        ,cnt->rtsp_high->img_latest->ptr
                        ,cnt->rtsp_high->img_latest->used);
                }
                img_data->idnbr_high = cnt->rtsp_high->idnbr;
            pthread_mutex_unlock(&cnt->rtsp_high->mutex);
        }

        /* Rotate images if requested */
        rotate_map(cnt, img_data);

        return 0;

    #else  /* No FFmpeg/Libav */
        (void)cnt;
        (void)img_data;
        return -1;
    #endif /* End #ifdef HAVE_FFMPEG */
}

void netcam_rtsp_cleanup(struct context *cnt, int init_retry_flag)
{
    #ifdef HAVE_FFMPEG
        /*
        * If the init_retry_flag is not set this function was
        * called while retrying the initial connection and there is
        * no camera-handler started yet and thread_running must
        * not be decremented.
        */
        int wait_counter;
        int indx_cam, indx_max;
        struct rtsp_context *rtsp_data;

        indx_cam = 1;
        if (cnt->rtsp_high) {
            indx_max = 2;
        } else {
            indx_max = 1;
        }

        while (indx_cam <= indx_max) {
            if (indx_cam == 1) {
                rtsp_data = cnt->rtsp;
            } else {
                rtsp_data = cnt->rtsp_high;
            }

            if (rtsp_data) {
                MOTION_LOG(INF, TYPE_NETCAM, NO_ERRNO
                    ,_("%s: Shutting down network camera."),rtsp_data->cameratype);

                /* Throw the finish flag in context and wait a bit for it to finish its work and close everything
                * This is shutting down the thread so for the moment, we are not worrying about the
                * cross threading and protecting these variables with mutex's
                */
                rtsp_data->finish = TRUE;
                rtsp_data->interruptduration = 0;
                pthread_mutex_lock(&rtsp_data->mutex_recordq);
                    rtsp_data->record_thread_finish = TRUE;
                    pthread_cond_broadcast(&rtsp_data->cond_recordq);
                pthread_mutex_unlock(&rtsp_data->mutex_recordq);
                wait_counter = 0;
                while ((!rtsp_data->handler_finished) && (wait_counter < 10)) {
                    SLEEP(1,0);
                    wait_counter++;
                }
                if (!rtsp_data->handler_finished) {
                    MOTION_LOG(ERR, TYPE_NETCAM, NO_ERRNO
                        ,_("%s: No response from handler thread."),rtsp_data->cameratype);
                    /* Last resort.  Kill the thread. Not safe for posix but if no response, what to do...*/
                    /* pthread_kill(rtsp_data->thread_id); */
                    pthread_cancel(rtsp_data->thread_id);
                    pthread_kill(rtsp_data->thread_id, SIGVTALRM); /* This allows the cancel to be processed */
                    if (!init_retry_flag) {
                        pthread_mutex_lock(&global_lock);
                            threads_running--;
                        pthread_mutex_unlock(&global_lock);
                    }
                }

                wait_counter = 0;
                while ((!rtsp_data->record_thread_finished) && (wait_counter < 10)) {
                    SLEEP(1,0);
                    wait_counter++;
                }

                /* If we never connect we don't have a handler but we still need to clean up some */
                netcam_rtsp_shutdown(rtsp_data);

                pthread_mutex_lock(&rtsp_data->mutex_recordq);
                    netcam_rtsp_record_queue_clear_lck(rtsp_data);
                pthread_mutex_unlock(&rtsp_data->mutex_recordq);

                if (rtsp_data->record_pktqueue != NULL) {
                    free(rtsp_data->record_pktqueue);
                    rtsp_data->record_pktqueue = NULL;
                }
                rtsp_data->record_pktqueue_size = 0;

                pthread_mutex_destroy(&rtsp_data->mutex);
                pthread_mutex_destroy(&rtsp_data->mutex_pktarray);
                pthread_mutex_destroy(&rtsp_data->mutex_transfer);
                pthread_mutex_destroy(&rtsp_data->mutex_record);
                pthread_mutex_destroy(&rtsp_data->mutex_recordq);
                if (rtsp_data->audio_sync_init) {
                    pthread_mutex_destroy(&rtsp_data->mutex_audio);
                    pthread_mutex_destroy(&rtsp_data->mutex_audioq);
                    pthread_cond_destroy(&rtsp_data->cond_audioq);
                    rtsp_data->audio_sync_init = FALSE;
                }
                pthread_cond_destroy(&rtsp_data->cond_recordq);
                rtsp_data->record_sync_init = FALSE;

                free(rtsp_data);
                rtsp_data = NULL;
                if (indx_cam == 1) {
                    MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO
                        ,_("Normal resolution: Shut down complete."));
                } else {
                    MOTION_LOG(NTC, TYPE_NETCAM, NO_ERRNO
                        ,_("High resolution: Shut down complete."));
                }
            }
            indx_cam++;
        }
        cnt->rtsp = NULL;
        cnt->rtsp_high = NULL;

    #else  /* No FFmpeg/Libav */
        (void)cnt;
        (void)init_retry_flag;
        return;
    #endif /* End #ifdef HAVE_FFMPEG */

}

