// Transport repair for the author's 2013 BPMPD pipe serializer. The optimizer
// and QP numerical code are unchanged. POSIX permits partial reads/writes.
#pragma once
#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <ctime>
#include <poll.h>
#include <stdexcept>
#include <unistd.h>

namespace trajopt_io {
inline double now() {
  timespec stamp;
  clock_gettime(CLOCK_MONOTONIC, &stamp);
  return stamp.tv_sec + stamp.tv_nsec * 1e-9;
}
inline void wait(int fd, short events) {
  const char* text = std::getenv("TRAJOPT_BPMPD_DEADLINE_MONOTONIC");
  const double deadline = text ? std::strtod(text, NULL) : now()+60.;
  while (true) {
    const double remaining = deadline-now();
    if (remaining <= 0)
      throw std::runtime_error("BPMPD transport deadline exceeded");
    pollfd descriptor = {fd, events, 0};
    const int timeout = std::max(1, std::min(1000, (int)(1000*remaining)));
    const int result = poll(&descriptor, 1, timeout);
    if (result < 0 && errno == EINTR) continue;
    if (result < 0) throw std::runtime_error("BPMPD transport poll failed");
    if (!result) continue;
    if (descriptor.revents & events) return;
    if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))
      throw std::runtime_error("BPMPD transport peer closed");
  }
}
inline ssize_t read_full(int fd, void* buffer, size_t size) {
  size_t completed = 0;
  while (completed < size) {
    wait(fd, POLLIN);
    const ssize_t count = ::read(fd, static_cast<char*>(buffer)+completed,
                                 std::min(size-completed, size_t(4096)));
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) throw std::runtime_error("BPMPD transport short read/EOF");
    completed += count;
  }
  return completed;
}
inline ssize_t write_full(int fd, const void* buffer, size_t size) {
  size_t completed = 0;
  while (completed < size) {
    wait(fd, POLLOUT);
    const ssize_t count = ::write(fd, static_cast<const char*>(buffer)+completed,
                                  std::min(size-completed, size_t(4096)));
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) throw std::runtime_error("BPMPD transport short write");
    completed += count;
  }
  return completed;
}
}
