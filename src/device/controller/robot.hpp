#pragma once

#include <mutex>
#include <om.hpp>
#include <stack>
#include <thread>

#include "common.hpp"
#include "crc16.hpp"
#include "opencv2/core/quaternion.hpp"
#include "opencv2/opencv.hpp"
#include "protocol.h"
#include "semaphore.hpp"
#include "serial.hpp"
#include "timer.hpp"

class Robot {
 private:
  Serial serial_;
  bool thread_continue = false;
  std::thread thread_recv_, thread_trans_;

  std::deque<Protocol_DownData_t> commandq_;
  Protocol_UpDataReferee_t ref_;
  Protocol_UpDataMCU_t mcu_;

  std::mutex mutex_command_, mutex_ref_, mutex_mcu_;
  component::Semaphore pack_signal_;
  component::Recorder recorder_ = component::Recorder("recv");
  void ThreadRecv();
  void ThreadTrans();

 public:
  Robot();
  explicit Robot(const std::string &dev_path);
  ~Robot();

  void Init(const std::string &dev_path);

  game::Team GetEnemyTeam();
  game::Race GetRace();
  double GetTime();
  game::RFID GetRFID();
  int GetBaseHP();
  int GetSentryHP();
  int GetBalletRemain();
  game::Arm GetArm();

  component::Euler GetEuler();
  cv::Mat GetRotMat();
  float GetBalletSpeed();
  float GetChassicSpeed();
  bool GetNotice();
  Message::Topic<Protocol_DownData_t> robot_topic;

  void Pack(Protocol_DownData_t &data, const double distance);
};
