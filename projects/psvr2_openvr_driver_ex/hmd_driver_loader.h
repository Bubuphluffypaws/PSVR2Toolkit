#pragma once

#include <windows.h>

#include <cstdint>

namespace psvr2_toolkit {

  // Provides a thin interface between our proxy library and the original HMD driver.
  class HmdDriverLoader {
  public:
    HmdDriverLoader();

    static HmdDriverLoader *Instance();

    bool Initialized();
    void Initialize();

    HMODULE GetHandle();
    uintptr_t GetBaseAddress();

    void *(*pfnHmdDriverFactory)(const char *pInterfaceName, int *pReturnCode);

  private:
    bool m_initialized;
    HMODULE m_handle;

    bool GetHmdDllPath(wchar_t *pszHmdDllPath);
  };

} // psvr2_toolkit

extern psvr2_toolkit::HmdDriverLoader *g_pHmdDriverLoader;
