#pragma once

typedef struct {
  Armor frame;
  std::vector<cv::Point2f> frame_vertices;
  cv::Mat physical_vertices;
} DetectorPacked;