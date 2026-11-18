// Print target that collects text in a fixed buffer (command replies).
#pragma once

#include <Print.h>

template <size_t N>
class BufferPrint : public Print {
 public:
  size_t write(uint8_t c) override {
    if (len_ < N - 1) buf_[len_++] = static_cast<char>(c);
    return 1;
  }
  const char* c_str() {
    buf_[len_] = '\0';
    return buf_;
  }
  size_t length() const { return len_; }
  void clear() { len_ = 0; }

 private:
  char buf_[N];
  size_t len_ = 0;
};
