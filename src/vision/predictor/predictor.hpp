#pragma once

#include <chrono>
#include <vector>

#include "common.hpp"
#include "opencv2/opencv.hpp"
#include "spdlog/spdlog.h"

template <typename Target, typename Param, typename Filter>
class Predictor {
 private:
  virtual void InitDefaultParams(const std::string &path) = 0;
  virtual bool PrepareParams(const std::string &path) = 0;

 public:
  std::vector<Target> predicts_;
  Param params_;
  Filter filter_;
  component::Direction direction_ = component::Direction::kUNKNOWN;

  void LoadParams(const std::string &path) {
    if (!PrepareParams(path)) {
      InitDefaultParams(path);
      PrepareParams(path);
      SPDLOG_WARN("Can not find params file. Created and reloaded.");
    }
    SPDLOG_DEBUG("Params loaded.");
  }

  virtual const std::vector<Target> &Predict() = 0;
  virtual void VisualizePrediction(const cv::Mat &output, bool add_lable) = 0;
};