#pragma once

#include <deque>
#include <mutex>
#include <thread>

#include "om.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgproc.hpp"
#include "semaphore.hpp"
#include "spdlog/spdlog.h"
#include "timer.hpp"

class Camera {
 private:
  component::Recorder cam_recorder_ = component::Recorder("CameraThread");
  virtual void GrabPrepare() = 0;
  virtual void GrabLoop() = 0;

  void GrabThread() {
    SPDLOG_DEBUG("[GrabThread] Started.");
    GrabPrepare();
    while (grabing) {
      GrabLoop();
      cam_recorder_.Record();
    }
    SPDLOG_DEBUG("[GrabThread] Stoped.");
  }

  virtual bool OpenPrepare(unsigned int index) = 0;

 public:
  unsigned int frame_h_, frame_w_;

  bool grabing = false;
  std::thread grab_thread_;
  std::mutex frame_mutex_;
  cv::Mat frame_;

  Message::Topic<cv::Mat> cam_topic_;

  Camera() : cam_topic_("cam_topic") {}

  /**
   * @brief 设置相机参数
   *
   * @param width 输出图像宽度
   * @param height 输出图像高度
   */
  void Setup(unsigned int width, unsigned int height) {
    frame_w_ = width;
    frame_h_ = height;
  }

  /**
   * @brief 打开相机设备
   *
   * @param index 相机索引号
   * @return true 打开成功
   * @return false 打开失败
   */
  bool Open(unsigned int index) {
    if (OpenPrepare(index)) {
      grabing = true;
      grab_thread_ = std::thread(&Camera::GrabThread, this);
      return true;
    }
    return false;
  }

  /**
   * @brief Get the Frame object
   *
   * @return cv::Mat 拍摄的图像
   */
  // [[deprecated("Use Pub&Sub mode")]]
  virtual bool GetFrame(cv::Mat& frame) {
    /*
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (!frame_stack_.empty()) {
        frame_signal_.Take();
        cv::resize(frame_stack_.front(), frame, cv::Size(frame_w_, frame_h_));
        frame_stack_.clear();
      } else {
        // SPDLOG_ERROR("Empty frame stack!");
        return false;
      }
      return true;
    */
    return false;
  }

  /**
   * @brief 关闭相机设备
   *
   * @return int 状态代码
   */
  virtual int Close() = 0;
};
