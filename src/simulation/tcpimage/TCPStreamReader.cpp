#include "TCPStreamReader.h"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

namespace {
constexpr size_t CHUNK_SIZE = 4096;

void close_fd(int &fd) {
  if (fd >= 0) {
    ::close(fd);
    fd = -1;
  }
}
}  // namespace

TCPStreamReader::TCPStreamReader(const std::string &host, const std::string &port)
    : host_(host), port_(port) {
  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_PASSIVE;

  addrinfo *result = nullptr;
  const char *node = host_.empty() || host_ == "0.0.0.0" ? nullptr : host_.c_str();
  const int rc = ::getaddrinfo(node, port_.c_str(), &hints, &result);
  if (rc != 0) {
    throw TCPStreamReaderException(std::string("getaddrinfo failed: ") + gai_strerror(rc));
  }

  for (addrinfo *rp = result; rp != nullptr; rp = rp->ai_next) {
    server_fd_ = ::socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
    if (server_fd_ < 0) {
      continue;
    }

    int yes = 1;
    ::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    if (::bind(server_fd_, rp->ai_addr, rp->ai_addrlen) == 0 && ::listen(server_fd_, 1) == 0) {
      break;
    }

    close_fd(server_fd_);
  }

  ::freeaddrinfo(result);

  if (server_fd_ < 0) {
    throw TCPStreamReaderException("failed to bind/listen on " + host_ + ":" + port_ + ": " + std::strerror(errno));
  }
}

TCPStreamReader::~TCPStreamReader() { Shutdown(); }

void TCPStreamReader::WaitConnect() {
  sockaddr_storage peer_addr{};
  socklen_t peer_len = sizeof(peer_addr);
  client_fd_ = ::accept(server_fd_, reinterpret_cast<sockaddr *>(&peer_addr), &peer_len);
  if (client_fd_ < 0) {
    throw TCPStreamReaderException(std::string("accept failed: ") + std::strerror(errno));
  }
  good_ = true;
}

bool TCPStreamReader::Good() const { return good_; }

void TCPStreamReader::Shutdown() {
  good_ = false;
  close_fd(client_fd_);
  close_fd(server_fd_);
}

void TCPStreamReader::ReadExact(void *buffer, size_t num_bytes) {
  auto *dst = static_cast<uint8_t *>(buffer);
  size_t total = 0;
  while (total < num_bytes) {
    const ssize_t received = ::recv(client_fd_, dst + total, num_bytes - total, 0);
    if (received == 0) {
      good_ = false;
      throw TCPStreamReaderException("TCP connection closed by peer");
    }
    if (received < 0) {
      if (errno == EINTR) {
        continue;
      }
      good_ = false;
      throw TCPStreamReaderException(std::string("recv failed: ") + std::strerror(errno));
    }
    total += static_cast<size_t>(received);
  }
}

int32_t TCPStreamReader::ReadInt() {
  int32_t value{};
  ReadExact(&value, sizeof(value));
  return value;
}

uint32_t TCPStreamReader::ReadUInt() {
  uint32_t value{};
  ReadExact(&value, sizeof(value));
  return value;
}

uint64_t TCPStreamReader::ReadUInt64() {
  uint64_t value{};
  ReadExact(&value, sizeof(value));
  return value;
}

float TCPStreamReader::ReadFloat() {
  float value{};
  ReadExact(&value, sizeof(value));
  return value;
}

std::string TCPStreamReader::ReadString() {
  std::string value;
  while (true) {
    char c = 0;
    ReadExact(&c, 1);
    if (c == '\0') {
      break;
    }
    value.push_back(c);
  }
  return value;
}

std::shared_ptr<uint8_t> TCPStreamReader::ReadBytes(size_t num_bytes) {
  auto bytes = std::shared_ptr<uint8_t>(new uint8_t[num_bytes], std::default_delete<uint8_t[]>());
  size_t remaining = num_bytes;
  size_t offset = 0;
  while (remaining > 0) {
    const size_t chunk = std::min(remaining, CHUNK_SIZE);
    ReadExact(bytes.get() + offset, chunk);
    offset += chunk;
    remaining -= chunk;
  }
  return bytes;
}
