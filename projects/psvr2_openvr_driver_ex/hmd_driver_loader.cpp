#include "hmd_driver_loader.h"

#include <shlwapi.h>

#define HMD_DLL_NAME L"driver_playstation_vr2_orig.dll"

extern "C" IMAGE_DOS_HEADER __ImageBase;

psvr2_toolkit::HmdDriverLoader *g_pHmdDriverLoader;

// Allows the C++ standard library load the original HMD driver automatically for us.
class HmdDriverLoaderInitializer {
public:
  HmdDriverLoaderInitializer() {
    psvr2_toolkit::HmdDriverLoader::Instance();
  }
};
HmdDriverLoaderInitializer initializer;

namespace psvr2_toolkit {

  HmdDriverLoader::HmdDriverLoader()
    : pfnHmdDriverFactory(nullptr)
    , m_initialized(false)
    , m_handle(nullptr)
  {
    Initialize();
  }

  HmdDriverLoader *HmdDriverLoader::Instance() {
    if (!g_pHmdDriverLoader) {
      g_pHmdDriverLoader = new HmdDriverLoader;
    }

    return g_pHmdDriverLoader;
  }

  bool HmdDriverLoader::Initialized() {
    return m_initialized;
  }

  void HmdDriverLoader::Initialize() {
    if (m_initialized) {
      return;
    }

    wchar_t pszHmdDllPath[MAX_PATH] = {0};
    if (GetHmdDllPath(pszHmdDllPath)) {
      m_handle = LoadLibraryW(pszHmdDllPath);
      if (m_handle) {
        pfnHmdDriverFactory = decltype(pfnHmdDriverFactory)(GetProcAddress(m_handle, "HmdDriverFactory"));
      }
    }

    m_initialized = true;
  }

  HMODULE HmdDriverLoader::GetHandle() {
    return m_handle;
  }

  uintptr_t HmdDriverLoader::GetBaseAddress() {
    return reinterpret_cast<uintptr_t>(m_handle);
  }

  bool HmdDriverLoader::GetHmdDllPath(wchar_t *pszHmdDllPath) {
    if (!pszHmdDllPath) {
      return false;
    }

    wchar_t pszPath[MAX_PATH] = {0};
    DWORD dwLength = GetModuleFileNameW(reinterpret_cast<HINSTANCE>(&__ImageBase), pszPath, MAX_PATH);
    if (dwLength > 0 && dwLength < MAX_PATH) {
      if (PathRemoveFileSpecW(pszPath)) {
        if (PathCombineW(pszHmdDllPath, pszPath, HMD_DLL_NAME)) {
          return true;
        }
      }
    }

    return false;
  }

} // psvr2_toolkit
