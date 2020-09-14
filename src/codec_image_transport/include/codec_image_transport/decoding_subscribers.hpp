#ifndef CODEC_IMAGE_TRANSPORT_DECODING_SUBSCRIBERS_HPP
#define CODEC_IMAGE_TRANSPORT_DECODING_SUBSCRIBERS_HPP

#include <string>

#include <image_transport/simple_subscriber_plugin.h>
#include <image_transport/transport_hints.h>
#include <ros/console.h>
#include <ros/exception.h>
#include <ros/node_handle.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/mpl/string.hpp>
#include <boost/shared_ptr.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

namespace codec_image_transport {

template < AVCodecID CodecID, typename TransportName >
class DecodingSubscriber
    : public image_transport::SimpleSubscriberPlugin< sensor_msgs::CompressedImage > {
public:
  DecodingSubscriber() {}

  virtual ~DecodingSubscriber() {}

  virtual std::string getTransportName() const { return boost::mpl::c_str< TransportName >::value; }

private:
  virtual void subscribeImpl(ros::NodeHandle &nh, const std::string &base_topic,
                             uint32_t queue_size, const Callback &callback,
                             const ros::VoidPtr &tracked_object,
                             const image_transport::TransportHints &transport_hints) {
    if (!decoder_ctx_) {
      // init libavformat
      avcodec_register_all();
      av_log_set_level(AV_LOG_FATAL);

      // find a decoder
      AVCodec *const decoder(avcodec_find_decoder(CodecID));
      if (!decoder) {
        throw ros::Exception("Cannot find a decoder");
      }

      // allocate a decoder context
      decoder_ctx_.reset(avcodec_alloc_context3(decoder), deleteAVCodecContext);
      if (!decoder_ctx_) {
        throw ros::Exception("Cannot allocate a decoder context");
      }

      // open decoder
      if (avcodec_open2(decoder_ctx_.get(), decoder, NULL) < 0) {
        throw ros::Exception("Failed to open codec");
      }
    }

    Base::subscribeImpl(nh, base_topic, queue_size, callback, tracked_object, transport_hints);
  }

  virtual void internalCallback(const sensor_msgs::CompressedImage::ConstPtr &message,
                                const Callback &user_cb) {
    // set packet data from the input message
    AVPacket packet;
    av_init_packet(&packet);
    packet.size = message->data.size();
    packet.data = const_cast< uint8_t * >(&message->data[0]);

    // send the packet to the decoder
    if (avcodec_send_packet(decoder_ctx_.get(), &packet) < 0) {
      ROS_ERROR("Cannot send a packet to decoder");
      return;
    }

    while (true) {
      // allocate a frame for decoded data
      boost::shared_ptr< AVFrame > frame(av_frame_alloc(), deleteAVFrame);
      if (!frame) {
        ROS_ERROR("Cannot allocate frame");
        return;
      }

      // receive the decoded data from the decoder
      const int res(avcodec_receive_frame(decoder_ctx_.get(), frame.get()));
      if (res == AVERROR(EAGAIN) || res == AVERROR_EOF) {
        // no more frames in the packet
        return;
      } else if (res < 0) {
        ROS_ERROR("Cannot receive a frame");
        return;
      }

      // allocate output message
      const sensor_msgs::ImagePtr out(new sensor_msgs::Image());
      out->header = message->header;
      out->height = frame->height;
      out->width = frame->width;
      out->encoding = sensor_msgs::image_encodings::BGR8;
      out->step = 3 * frame->width;
      out->data.resize(3 * frame->width * frame->height);

      // layout data by converting color spaces (YUV -> RGB)
      boost::shared_ptr< SwsContext > convert_ctx(
          sws_getContext(
              // src formats
              frame->width, frame->height,
              toUndeprecated(static_cast< AVPixelFormat >(frame->format)),
              // dst formats
              frame->width, frame->height, AV_PIX_FMT_BGR24,
              // flags & filters
              SWS_FAST_BILINEAR, NULL, NULL, NULL),
          sws_freeContext);
      int stride(out->step);
      uint8_t *dst(&out->data[0]);
      sws_scale(convert_ctx.get(),
                // src data
                frame->data, frame->linesize, 0, frame->height,
                // dst data
                &dst, &stride);

      // exec user callback
      user_cb(out);
    }
  }

  // Deleter for auto free/close of libav objects
  static void deleteAVFrame(AVFrame *frame) {
    if (frame) {
      av_frame_free(&frame);
    }
  }

  static void deleteAVCodecContext(AVCodecContext *ctx) {
    if (ctx) {
      avcodec_free_context(&ctx);
    }
  }

  // Workaround to avoid deprecated pixel format warning
  static AVPixelFormat toUndeprecated(const AVPixelFormat format) {
    switch (format) {
    case AV_PIX_FMT_YUVJ420P:
      return AV_PIX_FMT_YUV420P;
    case AV_PIX_FMT_YUVJ411P:
      return AV_PIX_FMT_YUV411P;
    case AV_PIX_FMT_YUVJ422P:
      return AV_PIX_FMT_YUV422P;
    case AV_PIX_FMT_YUVJ440P:
      return AV_PIX_FMT_YUV440P;
    case AV_PIX_FMT_YUVJ444P:
      return AV_PIX_FMT_YUV444P;
    default:
      return format;
    }
  }

private:
  typedef image_transport::SimpleSubscriberPlugin< sensor_msgs::CompressedImage > Base;

  boost::shared_ptr< AVCodecContext > decoder_ctx_;
};

typedef DecodingSubscriber< AV_CODEC_ID_H264, boost::mpl::string< 'h', '2', '6', '4' > >
    H264Subscriber;

} // namespace decoding_image_transport

#endif
