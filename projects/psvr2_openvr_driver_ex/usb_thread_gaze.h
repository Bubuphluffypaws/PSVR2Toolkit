#pragma once

#include <cstdint>

class CaesarUsbThreadGaze {
public:
  static void Reset();
  static CaesarUsbThreadGaze *Instance();

  void dtor_CaesarUsbThreadGaze();

  void close();

  uint8_t getUsbInf();
  uint8_t getReadPipeId();

  int poll();

private:
  static CaesarUsbThreadGaze *m_pInstance;

  void **m_ppVTable;
  char m_pBaseData[0x218]; // Total size (excluding VTable pointer) of CaesarUsbThread.
};
