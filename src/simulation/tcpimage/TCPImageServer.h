#pragma once

#include <cstdint>
#include <memory>
#include "TCPStreamReader.h"

struct ImageData {
  uint32_t time_sec = 0;
  uint32_t time_nsec = 0;
  int width = 0;
  int height = 0;
  std::shared_ptr<uint8_t> data;
};

class TCPImageServer {
public:
  TCPImageServer(TCPStreamReader *stream_reader, bool flip_image = false);
  ImageData GetImage();
  bool Good() const;

private:
  ImageData ReadImage(int width, int height);
  ImageData FlipImage(const ImageData &img);

  TCPStreamReader *stream_reader_;
  bool flip_;
};
