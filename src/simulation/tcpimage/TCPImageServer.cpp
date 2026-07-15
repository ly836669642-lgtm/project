#include "TCPImageServer.h"

#include <cstring>

namespace {
constexpr int BYTES_PER_PIXEL = 3;
}

TCPImageServer::TCPImageServer(TCPStreamReader *stream_reader, bool flip_image)
    : stream_reader_(stream_reader), flip_(flip_image) {}

ImageData TCPImageServer::GetImage() {
  const int width = stream_reader_->ReadInt();
  const int height = stream_reader_->ReadInt();
  ImageData img = ReadImage(width, height);
  return flip_ ? FlipImage(img) : img;
}

ImageData TCPImageServer::ReadImage(int width, int height) {
  ImageData img;
  img.width = width;
  img.height = height;
  img.data = stream_reader_->ReadBytes(static_cast<size_t>(width) * static_cast<size_t>(height) * BYTES_PER_PIXEL);
  return img;
}

bool TCPImageServer::Good() const { return stream_reader_->Good(); }

ImageData TCPImageServer::FlipImage(const ImageData &img) {
  const int stride = img.width * BYTES_PER_PIXEL;
  ImageData flipped;
  flipped.width = img.width;
  flipped.height = img.height;
  flipped.data = std::shared_ptr<uint8_t>(new uint8_t[static_cast<size_t>(img.height) * stride], std::default_delete<uint8_t[]>());

  for (int row = 0; row < img.height; ++row) {
    std::memcpy(flipped.data.get() + row * stride,
                img.data.get() + (img.height - row - 1) * stride,
                stride);
  }
  return flipped;
}
