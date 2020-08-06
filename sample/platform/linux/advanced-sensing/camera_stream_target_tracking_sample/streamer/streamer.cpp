#include "streamer.hpp"

#include <string>
#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>

namespace streamer
{

#define STREAM_PIX_FMT    AV_PIX_FMT_YUV420P

static int encode_and_write_frame(AVCodecContext *codec_ctx, AVFormatContext *fmt_ctx, AVFrame *frame)
{
    AVPacket pkt={0};
    av_init_packet(&pkt);

    // int ret = avcodec_send_frame(codec_ctx, frame);
    // if (ret < 0)
    // {
    //     fprintf(stderr, "Error sending frame to codec context!\n");
    //     return ret;
    // }

    // ret = avcodec_receive_packet(codec_ctx, &pkt);
    // if (ret < 0)
    // {
    //     fprintf(stderr, "Error receiving packet from codec context!\n" );
    //     return ret;
    // }

    // av_interleaved_write_frame(fmt_ctx, &pkt);
    // av_packet_unref(&pkt);
    int got_output;
    int ret = avcodec_encode_video2(codec_ctx, &pkt, frame, &got_output);
    if (ret < 0) {
        fprintf(stderr, "Error encoding frame\n");
        return ret;
    }
    printf("got_output:%d\n",got_output);
    if (got_output) {
            printf("Write frame (size=%5d)\n", pkt.size);
            av_interleaved_write_frame(fmt_ctx, &pkt);
            av_packet_unref(&pkt);
        }

    
    printf("Send frame ...\n");
    return 0;
}


static int set_options_and_open_encoder(AVFormatContext *fctx, AVStream *stream, AVCodecContext *codec_ctx, AVCodec *codec,
                                        std::string codec_profile, double width, double height,
                                        int fps, int bitrate, AVCodecID codec_id)
{
    const AVRational dst_fps = {fps, 1};

    codec_ctx->codec_tag = 0;
    codec_ctx->codec_id = codec_id;
    codec_ctx->codec_type = AVMEDIA_TYPE_VIDEO;
    codec_ctx->width = width;
    codec_ctx->height = height;
    codec_ctx->gop_size = 12;
    codec_ctx->pix_fmt = STREAM_PIX_FMT;
    codec_ctx->framerate = dst_fps;
    codec_ctx->time_base = av_inv_q(dst_fps);
    codec_ctx->bit_rate = bitrate;
    if (fctx->oformat->flags & AVFMT_GLOBALHEADER)
    {
        codec_ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    }

    stream->time_base = codec_ctx->time_base; //will be set afterwards by avformat_write_header to 1/1000

    // int ret = avcodec_parameters_from_context(stream->codecpar, codec_ctx);
    // if (ret < 0)
    // {
    //     fprintf(stderr, "Could not initialize stream codec parameters!\n");
    //     return 1;
    // }

    //Copy the settings of AVCodecContext
    int ret = avcodec_copy_context(stream->codec, codec_ctx);
    if (ret < 0) {
        printf( "Failed to copy context \n");
        return 1;
    }

    AVDictionary *codec_options = nullptr;
    av_dict_set(&codec_options, "profile", codec_profile.c_str(), 0);
    av_dict_set(&codec_options, "preset", "ultrafast", 0);
    av_dict_set(&codec_options, "tune", "zerolatency", 0);

    // open video encoder
    ret = avcodec_open2(codec_ctx, codec, &codec_options);
    // printf("avcodec_open2 0\n");
    // int ret = avcodec_open2(codec_ctx, codec, NULL);
    // printf("avcodec_open2 1\n");
    if (ret < 0)
    {
        fprintf(stderr, "Could not open video encoder!\n");
        return 1;
    }
    // av_dict_free(&codec_options);
    return 0;
}


Streamer::Streamer()
{
    format_ctx = nullptr;
    out_codec = nullptr;
    out_stream = nullptr;
    out_codec_ctx = nullptr;
    rtmp_server_conn = false;
    av_register_all();
    inv_stream_timebase = 30.0;
    network_init_ok = !avformat_network_init();
}


void Streamer::cleanup()
{
    if(out_codec_ctx) {
        avcodec_close(out_codec_ctx);
        avcodec_free_context(&out_codec_ctx);
    }

    if(format_ctx) {
        if(format_ctx->pb) {
            avio_close(format_ctx->pb);
        }
        avformat_free_context(format_ctx);
        format_ctx = nullptr;
    }
}


Streamer::~Streamer()
{
    cleanup();
    avformat_network_deinit();
}



void Streamer::stream_frame(const uint8_t *data)
{
    if(can_stream()) {
        const int stride[] = {static_cast<int>(config.src_width*3)};
        sws_scale(scaler.ctx, &data, stride, 0, config.src_height, picture.frame->data, picture.frame->linesize);
        picture.frame->pts += av_rescale_q(1, out_codec_ctx->time_base, out_stream->time_base);
        encode_and_write_frame(out_codec_ctx, format_ctx, picture.frame);
    }
}


void Streamer::stream_frame(const uint8_t *data, int64_t frame_duration)
{
    if(can_stream()) {
        const int stride[] = {static_cast<int>(config.src_width*3)};
        sws_scale(scaler.ctx, &data, stride, 0, config.src_height, picture.frame->data, picture.frame->linesize);
        picture.frame->pts += frame_duration; //time of frame in milliseconds
        encode_and_write_frame(out_codec_ctx, format_ctx, picture.frame);
    }
}


void Streamer::enable_av_debug_log()
{
    av_log_set_level(AV_LOG_DEBUG);
}


int Streamer::init(const StreamerConfig &streamer_config)
{
    init_ok = false;
    cleanup();

    config = streamer_config;

    if(!network_init_ok) {
        return 1;
    }

    //initialize format context for output with flv and no filename
    avformat_alloc_output_context2(&format_ctx, NULL, "flv", NULL);
    if(!format_ctx) {
        return 1;
    }

    //AVIOContext for accessing the resource indicated by url
    if (!(format_ctx->oformat->flags & AVFMT_NOFILE)) {
        printf("start avio_open2 0\n");
        int avopen_ret  = avio_open2(&format_ctx->pb, config.server.c_str(),
                                     AVIO_FLAG_WRITE, nullptr, nullptr);
        // int avopen_ret  = avio_open(&format_ctx->pb, config.server.c_str(),AVIO_FLAG_WRITE);
        if (avopen_ret < 0)  {
            fprintf(stderr, "failed to open stream output context, stream will not work\n");
            return 1;
        }
        rtmp_server_conn = true;
    }

    //use selected codec
    AVCodecID codec_id = AV_CODEC_ID_H264;
    out_codec = avcodec_find_encoder(codec_id);
    if (!(out_codec)) {
        fprintf(stderr, "Could not find encoder for '%s'\n",
                avcodec_get_name(codec_id));
        return 1;
    }

    out_stream = avformat_new_stream(format_ctx, out_codec);
    if (!out_stream) {
        fprintf(stderr, "Could not allocate stream\n");
        return 1;
    }

    out_codec_ctx = avcodec_alloc_context3(out_codec);

    if(set_options_and_open_encoder(format_ctx, out_stream, out_codec_ctx, out_codec, config.profile,
                                    config.dst_width, config.dst_height, config.fps, config.bitrate, codec_id)) {
        return 1;
    }

    out_stream->codec->extradata_size = out_codec_ctx->extradata_size;
    out_stream->codec->extradata = static_cast<uint8_t*>(av_mallocz(out_codec_ctx->extradata_size));
    memcpy(out_stream->codec->extradata, out_codec_ctx->extradata, out_codec_ctx->extradata_size);

    //Dump Format------------------
    av_dump_format(format_ctx, 0, config.server.c_str(), 1);

    picture.init(out_codec_ctx->pix_fmt, config.dst_width, config.dst_height);
    scaler.init(out_codec_ctx, config.src_width, config.src_height,config.dst_width, config.dst_height, SWS_BILINEAR);
    printf("start write header\n");
    printf("%d\n",format_ctx->oformat->flags);
    std::cout << AVFMT_NODIMENSIONS << std::endl;
    std::cout << !(format_ctx->oformat->flags & AVFMT_NODIMENSIONS) << std::endl;

    int i;
    AVStream *st;
    AVDictionary *tmp = NULL;
    AVCodecContext *codec = NULL;
    printf("format_ctx->nb_streams: %d\n",format_ctx->nb_streams);

    for (i = 0; i < format_ctx->nb_streams; i++) {
        st    = format_ctx->streams[i];
        codec = st->codec;
        printf("codec->width: %d, codec->height: %d \n",codec->width,codec->height);
        if ((codec->width <= 0 || codec->height <= 0) &&
                !(format_ctx->oformat->flags & AVFMT_NODIMENSIONS)) {
                printf("failed\n");
            }
    }

    if (avformat_write_header(format_ctx, NULL) < 0)
    {
        fprintf(stderr, "Could not write header!\n");
        return 1;
    }

    printf("stream time base = %d / %d \n", out_stream->time_base.num, out_stream->time_base.den);

    inv_stream_timebase = (double)out_stream->time_base.den/(double)out_stream->time_base.num;

    init_ok = true;
    return 0;
}

} // namespace streamer
