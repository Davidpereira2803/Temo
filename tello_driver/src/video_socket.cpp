#include "tello_driver.hpp"
#include "get_camera_info.cpp"

#include <libavutil/frame.h>
#include <opencv2/highgui.hpp>

namespace tello_driver {

// Notes on Tello video:
// -- frames are always 960x720.
// -- frames are split into UDP packets of length 1460.
// -- normal frames are ~10k, or about 8 UDP packets.
// -- keyframes are ~35k, or about 25 UDP packets.
// -- keyframes are always preceded by an 8-byte UDP packet and a 13-byte UDP packet -- markers?
// -- the h264 parser will consume the 8-byte packet, the 13-byte packet and the entire keyframe without
//    generating a frame. Presumably the keyframe is stored in the parser and referenced later.

VideoSocket::VideoSocket(TelloDriver *driver) : TelloSocket(driver, 11111)
{
  if (!get_camera_info(camera_info_msg_)) {
    RCLCPP_ERROR(driver_->get_logger(), "Cannot get camera info");
  }

  buffer_ = std::vector<unsigned char>(2048);
  seq_buffer_ = std::vector<unsigned char>(65536);
  listen();
}

// Process a video packet from the drone
void VideoSocket::process_packet(size_t r)
{
  std::lock_guard<std::mutex> lock(mtx_);

  receive_time_ = driver_->now();

  if (!receiving_) {
    // First packet
    RCLCPP_INFO(driver_->get_logger(), "Receiving video");
    receiving_ = true;
    seq_buffer_next_ = 0;
    seq_buffer_num_packets_ = 0;
  }

  if (seq_buffer_next_ + r >= seq_buffer_.size()) {
    RCLCPP_ERROR(driver_->get_logger(), "Video buffer overflow, dropping sequence");
    seq_buffer_next_ = 0;
    seq_buffer_num_packets_ = 0;
    return;
  }

  std::copy(buffer_.begin(), buffer_.begin() + r, seq_buffer_.begin() + seq_buffer_next_);
  seq_buffer_next_ += r;
  seq_buffer_num_packets_++;

  // If the packet is < 1460 bytes then it's the last packet in the sequence
  if (r < 1460) {
    decode_frames();

    seq_buffer_next_ = 0;
    seq_buffer_num_packets_ = 0;
  }
}

// Decode frames
void VideoSocket::decode_frames()
{
  size_t next = 0;

  try
  {
    while (next < seq_buffer_next_) {
      // Parse h264
      ssize_t consumed = decoder_.parse(seq_buffer_.data() + next, seq_buffer_next_ - next);

      // Is a frame available?
      if (decoder_.is_frame_available()) {
        // Decode the frame
        const AVFrame &frame = decoder_.decode_frame();

        // Convert pixels from YUV420P to BGR24
        int size = converter_.predict_size(frame.width, frame.height);
        unsigned char bgr24[size];
        converter_.convert(frame, bgr24);

        // Convert to cv::Mat
        cv::Mat mat{frame.height, frame.width, CV_8UC3, bgr24};

        // Display
        cv::imshow("frame", mat);
        cv::waitKey(1);

        // Synchronize ROS messages
        auto stamp = driver_->now();

        if (driver_->count_subscribers(driver_->image_pub_->get_topic_name()) > 0) {
          std_msgs::msg::Header header{};
          header.frame_id = "camera_frame";
          header.stamp = stamp;
          cv_bridge::CvImage image{header, sensor_msgs::image_encodings::BGR8, mat};
          driver_->image_pub_->publish(image.toImageMsg());
        }

        if (driver_->count_subscribers(driver_->camera_info_pub_->get_topic_name()) > 0) {
          camera_info_msg_.header.stamp = stamp;
          driver_->camera_info_pub_->publish(camera_info_msg_);
        }
      }

      next += consumed;
    }
  }
  catch (std::runtime_error e)
  {
    RCLCPP_ERROR(driver_->get_logger(), e.what());
  }
}

} // namespace tello_driver