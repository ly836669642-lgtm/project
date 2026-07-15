#pragma once

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

class TCPStreamReaderException : public std::runtime_error {
public:
  explicit TCPStreamReaderException(const std::string &message)
      : std::runtime_error(message) {}
};

class TCPStreamReader {
public:
  TCPStreamReader(const std::string &host, const std::string &port);
  ~TCPStreamReader();

  void WaitConnect();
  bool Good() const;
  void Shutdown();

  int32_t ReadInt();
  uint32_t ReadUInt();
  uint64_t ReadUInt64();
  float ReadFloat();
  std::string ReadString();
  std::shared_ptr<uint8_t> ReadBytes(size_t num_bytes);

private:
  void ReadExact(void *buffer, size_t num_bytes);

  std::string host_;
  std::string port_;
  int server_fd_{-1};
  int client_fd_{-1};
  bool good_{false};
};
