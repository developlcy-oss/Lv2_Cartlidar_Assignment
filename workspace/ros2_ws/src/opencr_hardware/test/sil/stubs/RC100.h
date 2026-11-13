// ROBOTIS RC100 라이브러리 대역 (테스트 더블, 2026-11-13). 시험이 g_board.rc100_rx 에 넣은 바이트를 실제 라이브러리처럼
// 한 번에 1 바이트씩 읽고, FF 55 L ~L H ~H 가 맞으면 available() 이 true · readData() = L | H << 8.
// 실제 라이브러리 (OpenCR 1.5.3 libraries/RC100) 와 같은 점: begin(1) = Serial2 · begin(2) = Serial4, 1 바이트씩, 보수가 틀리면 버림
#ifndef SIL_RC100_H_
#define SIL_RC100_H_

class RC100
{
public:
  void begin(int num) {g_board.rc100_port = num;}
  int available()
  {
    if (g_board.rc100_rx.empty()) {return false;}
    const uint8_t d = g_board.rc100_rx.front();
    g_board.rc100_rx.pop_front();
    switch (state_) {
      case 0: state_ = (d == 0xFF) ? 1 : 0; return false;
      case 1: state_ = (d == 0x55) ? 2 : 0; return false;
      default:
        buf_[state_ - 2] = d;
        if (++state_ < 6) {return false;}
        state_ = 0;
        if (static_cast<uint8_t>(buf_[0] ^ buf_[1]) != 0xFF || static_cast<uint8_t>(buf_[2] ^ buf_[3]) != 0xFF) {return false;}
        data_ = static_cast<uint16_t>(buf_[0] | (buf_[2] << 8));
        return true;
    }
  }
  uint16_t readData() {return data_;}

private:
  int state_ = 0;
  uint8_t buf_[4] = {0, 0, 0, 0};
  uint16_t data_ = 0;
};

#endif  // SIL_RC100_H_
