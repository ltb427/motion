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

/*
 *  netcam_rtsp.h
 *    Headers associated with functions in the netcam_rtsp.c module.
 */

#ifndef _INCLUDE_NETCAM_RTSP_H
#define _INCLUDE_NETCAM_RTSP_H

struct context;
struct image_data;

enum RTSP_STATUS {
    RTSP_CONNECTED,      /* The camera is currently connected */
    RTSP_READINGIMAGE,   /* Motion is reading a image from camera */
    RTSP_NOTCONNECTED,   /* The camera has never connected */
    RTSP_RECONNECTING   /* Motion is trying to reconnect to camera */
};

struct imgsize_context {
    int                   width;
    int                   height;
};

#ifdef HAVE_FFMPEG

    #ifdef HAVE_FFTW3
        #include <fftw3.h>
    #endif

    struct packet_item{
        AVPacket                 *packet;
        int64_t                   idnbr;
        int                       iskey;
        int                       iswritten;
        struct timeval            timestamp_tv;
    };

    struct rtsp_context {
        AVFormatContext          *format_context;        /* Main format context for the camera */
        AVCodecContext           *codec_context;         /* Codec being sent from the camera */
        AVStream                 *strm;
        AVFrame                  *frame;                 /* Reusable frame for images from camera */
        AVFrame                  *swsframe_in;           /* Used when resizing image sent from camera */
        AVFrame                  *swsframe_out;          /* Used when resizing image sent from camera */
        struct SwsContext        *swsctx;                /* Context for the resizing of the image */
        AVPacket                 *packet_recv;           /* The packet that is currently being processed */
        AVFormatContext          *transfer_format;       /* Format context just for transferring to pass-through */
        AVFormatContext          *record_format;         /* Format context used for event recording */
        AVCodecContext           *audio_codec_context;   /* Codec context for audio analysis */
        AVFrame                  *audio_frame;           /* Reusable frame for decoded audio */
        AVPacket                **audio_pktqueue;        /* Queue of packets pending audio analysis */
        int                       audio_pktqueue_size;   /* Queue capacity */
        int                       audio_pktqueue_head;   /* Queue read index */
        int                       audio_pktqueue_tail;   /* Queue write index */
        int                       audio_pktqueue_count;  /* Number of queued packets */
        int                       audio_thread_finish;   /* Request the audio thread to finish */
        int                       audio_thread_finished; /* Whether audio thread has exited */
        int                       audio_sync_init;       /* Whether audio mutex/cond are initialized */
        pthread_t                 audio_thread_id;       /* thread id for audio analysis thread */
        int                       audio_stream_index;    /* Stream index associated with audio from camera */
        SwrContext               *audio_swr;             /* Audio resampler to mono double */
        int                       audio_sample_rate;     /* Audio sample rate */
        int                       audio_channels;        /* Audio channel count */
    #ifdef HAVE_FFTW3
        double                   *audio_fft_in;          /* FFT input buffer */
        fftw_complex             *audio_fft_out;         /* FFT output buffer */
        fftw_plan                 audio_fft_plan;        /* FFT plan for current window */
    #else
        void                     *audio_fft_in;
        void                     *audio_fft_out;
        void                     *audio_fft_plan;
    #endif
        int                       audio_fft_size;        /* Current FFT window size */
        int                       audio_fft_plan_ready;   /* Whether FFT plan is valid */
        pthread_mutex_t           mutex_audio;           /* Mutex protecting audio state */
        pthread_mutex_t           mutex_audioq;          /* Mutex protecting the audio queue */
        pthread_cond_t            cond_audioq;           /* Condition variable used by audio queue */
        int                      *record_stream_map;     /* Input stream index to output stream index */
        int                       record_stream_map_size;/* Number of entries in record_stream_map */
        int64_t                  *record_last_dts;       /* Last written dts for each input stream */
        int64_t                  *record_last_pts;       /* Last written pts for each input stream */
        int64_t                  *record_base_dts;       /* First dts per input stream, used to rebase to 0 */
        int64_t                  *record_base_pts;       /* First pts per input stream, used to rebase to 0 */
        int                       record_active;         /* Whether event recording is active */
        AVPacket                **record_pktqueue;       /* Queue of packets pending event recording */
        int                       record_pktqueue_size;  /* Queue capacity */
        int                       record_pktqueue_head;  /* Queue read index */
        int                       record_pktqueue_tail;  /* Queue write index */
        int                       record_pktqueue_count; /* Number of queued packets */
        int                       record_thread_finish;  /* Request the record thread to finish */
        int                       record_thread_finished;/* Whether record thread has exited */
        int                       record_sync_init;      /* Whether record mutex/cond are initialized */
        struct packet_item       *pktarray;              /* Pointer to array of packets for passthru processing */
        int                       pktarray_size;         /* The number of packets in array.  1 based */
        int                       pktarray_index;        /* The index to the most current packet in array */
        int64_t                   idnbr;                 /* A ID number to track the packet vs image */
        AVDictionary             *opts;                  /* AVOptions when opening the format context */
        int                       swsframe_size;         /* The size of the image after resizing */
        int                       video_stream_index;    /* Stream index associated with video from camera */
        #if (MYFFVER >= 57083)
            enum AVHWDeviceType       hw_type;
            enum AVPixelFormat        hw_pix_fmt;
            AVBufferRef               *hw_device_ctx;
        #endif
        my_AVCodec               *decoder;

        enum RTSP_STATUS          status;                /* Status of whether the camera is connecting, closed, etc*/
        struct timeval            interruptstarttime;    /* The time set before calling the av functions */
        struct timeval            interruptcurrenttime;  /* Time during the interrupt to determine duration since start*/
        int                       interruptduration;      /* Seconds permitted before triggering a interrupt */

        netcam_buff_ptr           img_recv;         /* The image buffer that is currently being processed */
        netcam_buff_ptr           img_latest;       /* The most recent image buffer that finished processing */

        int                       interrupted;      /* Boolean for whether interrupt has been tripped */
        int                       finish;           /* Boolean for whether we are finishing the application */
        int                       high_resolution;  /* Boolean for whether this context is the Norm or High */
        int                       handler_finished; /* Boolean for whether the handler is running or not */
        int                       first_image;      /* Boolean for whether we have captured the first image */
        int                       passthrough;      /* Boolean for whether we are doing pass-through processing */

        char                     *path;             /* The connection string to use for the camera */
        char                     *service;          /* String specifying the type of camera http, rtsp, v4l2 */
        const char               *camera_name;      /* The name of the camera as provided in the config file */
        char                      cameratype[30];   /* String specifying Normal or High for use in logging */
        struct imgsize_context    imgsize;          /* The image size parameters */

        int                       rtsp_uses_tcp;    /* Flag from config for whether to use tcp transport */
        int                       v4l2_palette;     /* Palette from config for v4l2 devices */
        int                       reconnect_count;  /* Count of the times reconnection is tried*/
        int                       src_fps;          /* The fps provided from source*/
        int                       capture_rate;     /* The framerate for the capture rate*/

        struct timeval            frame_prev_tm;    /* The time set before calling the av functions */
        struct timeval            frame_curr_tm;    /* Time during the interrupt to determine duration since start*/

        struct params_context    *parameters;       /* User specified parameters for the camera */
        struct config            *conf;             /* Pointer to conf parms of parent cnt*/
        char                      *decoder_nm;      /* User requested decoder */
        struct context            *cnt;

        char                      threadname[16];   /* The thread name*/
        int                       threadnbr;        /* The thread number */
        pthread_t                 thread_id;        /* thread i.d. for a camera-handling thread (if required). */
        pthread_t                 record_thread_id; /* thread i.d. for record writing thread */
        pthread_mutex_t           mutex;            /* mutex used with conditional waits */
        pthread_mutex_t           mutex_transfer;   /* mutex used with transferring stream info for pass-through */
        pthread_mutex_t           mutex_pktarray;   /* mutex used with the packet array */
        pthread_mutex_t           mutex_record;     /* mutex used with event recording format state */
        pthread_mutex_t           mutex_recordq;    /* mutex used with event recording queue */
        pthread_cond_t            cond_recordq;     /* condition variable used by event recording queue */

    };

#else /* Do not have FFmpeg */

    struct rtsp_context {
        int                   dummy;
        pthread_t             thread_id;
        int                   handler_finished;
    };

#endif /* end HAVE_FFMPEG  */

int netcam_rtsp_setup(struct context *cnt);
int netcam_rtsp_next(struct context *cnt, struct image_data *img_data);
void netcam_rtsp_cleanup(struct context *cnt, int init_retry_flag);
int netcam_rtsp_record_start(struct rtsp_context *rtsp_data, const char *filename);
void netcam_rtsp_record_stop(struct rtsp_context *rtsp_data);

#endif /* _INCLUDE_NETCAM_RTSP_H */
