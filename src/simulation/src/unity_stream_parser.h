#pragma once

#include <cstdint>
#include <string>
#include "TCPStreamReader.h"

enum UnityMessageType : uint32_t {
  UNITY_STATE = 0,
  UNITY_CAMERA = 1,
  UNITY_IMU = 2,
  UNITY_DEPTH = 3,
  UNITY_FISHEYE = 4,
  UNITY_DETECTIONS = 5,
  MESSAGE_TYPE_COUNT = 6
};

struct UnityHeader {
  UnityMessageType type;
  double timestamp;
  std::string name;
};

class UnityStreamParser {
public:
  virtual ~UnityStreamParser() = default;
  virtual bool ParseMessage(const UnityHeader &header,
                            TCPStreamReader &stream_reader,
                            double time_offset = 0.0) = 0;
};
