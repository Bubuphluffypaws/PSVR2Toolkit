#include "usb_thread_gaze.h"

#include "hmd_driver_loader.h"
#include "hmd_device_hooks.h"
#include "eyelid_estimator.h"
#include "original_eyelid_estimator.h"
#include "modern_eyelid_estimator.h"
#include "headset_calibrator.h"
#include "hmd2_gaze.h"
#include "ipc_server.h"
#include "util.h"

#include <cstdlib>
#include <chrono>
#include <windows.h>

#include <winusb.h>

#define GAZE_MAGIC_0 0x47
#define GAZE_MAGIC_1_CAL 0x43
#define GAZE_MAGIC_1_RAW 0x52
#define GAZE_MAGIC_1_STATE 0x53

using namespace psvr2_toolkit;
using namespace psvr2_toolkit::ipc;

void **ppVTable = nullptr; // We need to keep track of our customized CaesarUsbThread VTable here, so we may restore it.

void *(*Framework__Mutex__lock)(void *thisptr, uint32_t timeout) = nullptr;
void *(*Framework__Mutex__unlock)(void *thisptr) = nullptr;
void *(*Framework__Thread__stop)(void *thisptr) = nullptr;

void *(*CaesarUsbThread__CaesarUsbThread)(void *thisptr) = nullptr;
void *(*CaesarUsbThread__dtor_CaesarUsbThread)(void *thisptr, char a2) = nullptr;
int (*CaesarUsbThread__read)(void *thisptr, uint8_t pipeId, char *buffer, size_t length) = nullptr;

CaesarUsbThreadGaze *CaesarUsbThreadGaze::m_pInstance = nullptr;

// Eye tracking logging globals
static FILE* g_eyeTrackingLogFile = nullptr;
static std::chrono::steady_clock::time_point g_loggingStartTime;
static bool g_loggingActive = false;
static const int LOG_DURATION_SECONDS = 180;
static int g_framesSinceFlush = 0;

// Smoothing configuration - change these to test different methods
static constexpr int SMOOTHING_METHOD = 1;  // 0=LowPass, 1=StrongAveraging(500ms), 2=Kalman
static constexpr bool ENABLE_INDEPENDENT_EYES = true;  // Each eye tracks independently

// A/B Testing configuration
static constexpr bool ENABLE_AB_TESTING = false;  // Disabled - using modern implementation for both eyes
static constexpr bool USE_NEW_IMPLEMENTATION_BOTH_EYES = true;  // Use modern implementation for both eyes

// A/B Testing: True baseline vs modern implementation
psvr2_toolkit::ModernEyelidEstimator leftEyelidEstimator;     // MODERN implementation for left eye
psvr2_toolkit::ModernEyelidEstimator rightEyelidEstimator;    // MODERN implementation for right eye

// Headset calibrator for geometric compensation
psvr2_toolkit::HeadsetCalibrator headsetCalibrator;

// Static dummy variable for GetModuleHandleExA
static int g_dummyForModuleHandle = 0;

// Get the directory where the driver DLL is located
static std::string GetDllDirectory() {
  char dllPath[MAX_PATH];
  HMODULE hModule = NULL;

  // Get the handle to this DLL using address of a static variable in this module
  if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                         GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                         (LPCSTR)&g_dummyForModuleHandle,
                         &hModule)) {
    // Get the full path to the DLL
    if (GetModuleFileNameA(hModule, dllPath, sizeof(dllPath)) > 0) {
      // Find the last backslash to get the directory
      std::string fullPath(dllPath);
      size_t lastSlash = fullPath.find_last_of("\\/");
      if (lastSlash != std::string::npos) {
        return fullPath.substr(0, lastSlash);
      }
    }
  }

  // Fallback to current directory
  return ".";
}

// Initialize eye tracking logging
static void InitializeEyeTrackingLogging() {
  if (g_eyeTrackingLogFile != nullptr || g_loggingActive) {
    return; // Already initialized
  }

  // Get the DLL directory and construct log file path
  std::string dllDir = GetDllDirectory();
  std::string logFilePath = dllDir + "\\psvr2_eye_tracking_data.csv";

  // Open file for writing (overwrite if exists)
  errno_t err = fopen_s(&g_eyeTrackingLogFile, logFilePath.c_str(), "w");
  if (err != 0 || g_eyeTrackingLogFile == nullptr) {
    // Failed to open file - log will be disabled
    return;
  }

  // Write CSV header
  fprintf(g_eyeTrackingLogFile,
    "timestamp_ms,"
    "left_originX,left_originY,left_originZ,"
    "left_dirX,left_dirY,left_dirZ,"
    "left_pupilDia,"
    "left_pupilPosX,left_pupilPosY,"
    "left_guideX,left_guideY,"
    "left_blink,"
    "right_originX,right_originY,right_originZ,"
    "right_dirX,right_dirY,right_dirZ,"
    "right_pupilDia,"
    "right_pupilPosX,right_pupilPosY,"
    "right_guideX,right_guideY,"
    "right_blink,"
    "combined_originX,combined_originY,combined_originZ\n");

  // Initialize timing
  g_loggingStartTime = std::chrono::steady_clock::now();
  g_loggingActive = true;
  g_framesSinceFlush = 0;
}

// Log eye tracking data
static void LogEyeTrackingData(const Hmd2GazeState* pGazeState) {
  if (!g_loggingActive || g_eyeTrackingLogFile == nullptr) {
    return;
  }

  // Check if logging duration has elapsed
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - g_loggingStartTime);
  if (elapsed.count() >= LOG_DURATION_SECONDS) {
    // Close the file and stop logging
    if (g_eyeTrackingLogFile != nullptr) {
      fclose(g_eyeTrackingLogFile);
      g_eyeTrackingLogFile = nullptr;
    }
    g_loggingActive = false;
    return;
  }

  // Calculate timestamp in milliseconds
  auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_loggingStartTime);

  // Log all fields in CSV format with high precision
  fprintf(g_eyeTrackingLogFile,
    "%.3f,"                                                    // timestamp_ms
    "%.6f,%.6f,%.6f,"                                         // left_originX,Y,Z
    "%.6f,%.6f,%.6f,"                                         // left_dirX,Y,Z
    "%.6f,"                                                    // left_pupilDia
    "%.6f,%.6f,"                                               // left_pupilPosX,Y
    "%.6f,%.6f,"                                               // left_guideX,Y
    "%d,"                                                      // left_blink
    "%.6f,%.6f,%.6f,"                                         // right_originX,Y,Z
    "%.6f,%.6f,%.6f,"                                         // right_dirX,Y,Z
    "%.6f,"                                                    // right_pupilDia
    "%.6f,%.6f,"                                               // right_pupilPosX,Y
    "%.6f,%.6f,"                                               // right_guideX,Y
    "%d,"                                                      // right_blink
    "%.6f,%.6f,%.6f\n",                                       // combined_originX,Y,Z
    (double)elapsedMs.count(),
    // Left eye data
    pGazeState->leftEye.gazeOriginMm.x,
    pGazeState->leftEye.gazeOriginMm.y,
    pGazeState->leftEye.gazeOriginMm.z,
    pGazeState->leftEye.gazeDirNorm.x,
    pGazeState->leftEye.gazeDirNorm.y,
    pGazeState->leftEye.gazeDirNorm.z,
    pGazeState->leftEye.pupilDiaMm,
    pGazeState->leftEye.pupilPosInSensor.x,
    pGazeState->leftEye.pupilPosInSensor.y,
    pGazeState->leftEye.posGuide.x,
    pGazeState->leftEye.posGuide.y,
    pGazeState->leftEye.blink ? 1 : 0,
    // Right eye data
    pGazeState->rightEye.gazeOriginMm.x,
    pGazeState->rightEye.gazeOriginMm.y,
    pGazeState->rightEye.gazeOriginMm.z,
    pGazeState->rightEye.gazeDirNorm.x,
    pGazeState->rightEye.gazeDirNorm.y,
    pGazeState->rightEye.gazeDirNorm.z,
    pGazeState->rightEye.pupilDiaMm,
    pGazeState->rightEye.pupilPosInSensor.x,
    pGazeState->rightEye.pupilPosInSensor.y,
    pGazeState->rightEye.posGuide.x,
    pGazeState->rightEye.posGuide.y,
    pGazeState->rightEye.blink ? 1 : 0,
    // Combined gaze data
    pGazeState->combined.gazeOriginMm.x,
    pGazeState->combined.gazeOriginMm.y,
    pGazeState->combined.gazeOriginMm.z);

  // Flush the file every 60 frames to prevent data loss
  g_framesSinceFlush++;
  if (g_framesSinceFlush >= 60) {
    fflush(g_eyeTrackingLogFile);
    g_framesSinceFlush = 0;
  }
}

// Initialize smoothing methods
void InitializeSmoothingMethods() {
  // Use the public enum values directly
  switch (SMOOTHING_METHOD) {
    case 0:
      leftEyelidEstimator.SetSmoothingMethod(psvr2_toolkit::ModernEyelidEstimator::SmoothingSystem::SIMPLE_LOWPASS);
      rightEyelidEstimator.SetSmoothingMethod(psvr2_toolkit::ModernEyelidEstimator::SmoothingSystem::SIMPLE_LOWPASS);
      break;
    case 1:
      leftEyelidEstimator.SetSmoothingMethod(psvr2_toolkit::ModernEyelidEstimator::SmoothingSystem::STRONG_AVERAGING);
      rightEyelidEstimator.SetSmoothingMethod(psvr2_toolkit::ModernEyelidEstimator::SmoothingSystem::STRONG_AVERAGING);
      break;
    case 2:
      leftEyelidEstimator.SetSmoothingMethod(psvr2_toolkit::ModernEyelidEstimator::SmoothingSystem::KALMAN_FILTER);
      rightEyelidEstimator.SetSmoothingMethod(psvr2_toolkit::ModernEyelidEstimator::SmoothingSystem::KALMAN_FILTER);
      break;
    default:
      leftEyelidEstimator.SetSmoothingMethod(psvr2_toolkit::ModernEyelidEstimator::SmoothingSystem::STRONG_AVERAGING);
      rightEyelidEstimator.SetSmoothingMethod(psvr2_toolkit::ModernEyelidEstimator::SmoothingSystem::STRONG_AVERAGING);
      break;
  }
}

void *j_CaesarUsbThreadGaze__dtor_CaesarUsbThreadGaze(CaesarUsbThreadGaze *thisptr, char a2) {
  thisptr->dtor_CaesarUsbThreadGaze();
  void *result = CaesarUsbThread__dtor_CaesarUsbThread(thisptr, a2);
  CaesarUsbThreadGaze::Reset();
  return result;
}

void j_CaesarUsbThreadGaze__close(CaesarUsbThreadGaze *thisptr) {
  return thisptr->close();
}

uint8_t j_CaesarUsbThreadGaze__getUsbInf(CaesarUsbThreadGaze *thisptr) {
  return thisptr->getUsbInf();
}

uint8_t j_CaesarUsbThreadGaze__getReadPipeId(CaesarUsbThreadGaze *thisptr) {
  return thisptr->getReadPipeId();
}

int j_CaesarUsbThreadGaze__poll(CaesarUsbThreadGaze *thisptr) {
  return thisptr->poll();
}

void CaesarUsbThreadGaze::Reset() {
  CaesarUsbThreadGaze::m_pInstance = nullptr;
}

CaesarUsbThreadGaze *CaesarUsbThreadGaze::Instance() {
  static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

  if (!m_pInstance) {
    m_pInstance = static_cast<CaesarUsbThreadGaze *>(malloc(sizeof(CaesarUsbThreadGaze)));
    if (m_pInstance) {
      Framework__Mutex__lock = decltype(Framework__Mutex__lock)(pHmdDriverLoader->GetBaseAddress() + 0x16B5F0);
      Framework__Mutex__unlock = decltype(Framework__Mutex__unlock)(pHmdDriverLoader->GetBaseAddress() + 0x16B850);
      Framework__Thread__stop = decltype(Framework__Thread__stop)(pHmdDriverLoader->GetBaseAddress() + 0x16B540);

      CaesarUsbThread__CaesarUsbThread = decltype(CaesarUsbThread__CaesarUsbThread)(pHmdDriverLoader->GetBaseAddress() + 0x121F30);
      CaesarUsbThread__read = decltype(CaesarUsbThread__read)(pHmdDriverLoader->GetBaseAddress() + 0x127D60);

      // Initialize base class.
      CaesarUsbThread__CaesarUsbThread(m_pInstance);

      if (!ppVTable) {
        // Runtime VTable madness!
        // We must allocate the total size of the CaesarUsbThread VTable (9 virtual functions, multiplied by 8 to account for function pointer size).
        // We'll then copy the VTable initialized by calling CaesarUsbThread::CaesarUsbThread into our allocated VTable.
        // Pretty neat, right?
        ppVTable = static_cast<void **>(malloc(0x48));
        if (ppVTable) {
          memcpy(ppVTable, m_pInstance->m_ppVTable, 0x48);

          CaesarUsbThread__dtor_CaesarUsbThread = decltype(CaesarUsbThread__dtor_CaesarUsbThread)(ppVTable[0]); // Store the original destructor here.

          ppVTable[0] = &j_CaesarUsbThreadGaze__dtor_CaesarUsbThreadGaze;
          ppVTable[2] = &j_CaesarUsbThreadGaze__close;
          ppVTable[4] = &j_CaesarUsbThreadGaze__getUsbInf;
          ppVTable[5] = &j_CaesarUsbThreadGaze__getReadPipeId;
          ppVTable[8] = &j_CaesarUsbThreadGaze__poll;
        }
      }

      m_pInstance->m_ppVTable = ppVTable;
    }
  }

  return m_pInstance;
}

void CaesarUsbThreadGaze::dtor_CaesarUsbThreadGaze() {
  m_ppVTable = ppVTable;
  close();

  // Clean up logging resources
  if (g_eyeTrackingLogFile != nullptr) {
    fclose(g_eyeTrackingLogFile);
    g_eyeTrackingLogFile = nullptr;
  }
  g_loggingActive = false;
}

void CaesarUsbThreadGaze::close() {
  Framework__Mutex__lock((void *)((__int64)(this) + 0x30), 0xFFFFFFFF);
  *(char *)((__int64)(this) + 0x1E0) = 1;
  if (*(int *)((__int64)(this) + 0x28) == 2) {
    WinUsb_AbortPipe(*(WINUSB_INTERFACE_HANDLE *)((__int64)(this) + 0x48), 0x85);
  }
  Framework__Mutex__unlock((void *)((__int64)(this) + 0x30));
  Framework__Thread__stop(this);
}

uint8_t CaesarUsbThreadGaze::getUsbInf() {
  return 5;
}

uint8_t CaesarUsbThreadGaze::getReadPipeId() {
  return 0x85;
}

int CaesarUsbThreadGaze::poll() {
  static IpcServer *pIpcServer = IpcServer::Instance();

  static char buffer[0x200000];
  int result = CaesarUsbThread__read(this, 0x85, buffer, sizeof(buffer));
  if (result < 0) {
    return -1;
  }

  if (buffer[0] == GAZE_MAGIC_0 && buffer[1] == GAZE_MAGIC_1_STATE) {
    Hmd2GazeState *pGazeState = reinterpret_cast<Hmd2GazeState *>(buffer);
    HmdDeviceHooks::UpdateGaze(pGazeState, sizeof(Hmd2GazeState));

    // Initialize eye tracking logging (only once)
    if (!g_loggingActive && g_eyeTrackingLogFile == nullptr) {
      InitializeEyeTrackingLogging();
    }

    // Log eye tracking data
    LogEyeTrackingData(pGazeState);

    // Initialize smoothing methods (only once)
    static bool initialized = false;
    if (!initialized) {
      InitializeSmoothingMethods();
      initialized = true;
    }

    // Update headset calibration with raw eye data
    headsetCalibrator.UpdateCalibration(pGazeState->leftEye, pGazeState->rightEye);

    // Configurable A/B Testing Implementation with Headset Calibration
    float leftEyelidOpenness, rightEyelidOpenness;

    // Apply headset calibration to compensate for mounting position and eye shape
    psvr2_toolkit::CalibratedEyeData leftCalibrated = headsetCalibrator.CalibrateEyeData(pGazeState->leftEye);
    psvr2_toolkit::CalibratedEyeData rightCalibrated = headsetCalibrator.CalibrateEyeData(pGazeState->rightEye);

    if (ENABLE_AB_TESTING && !USE_NEW_IMPLEMENTATION_BOTH_EYES) {
      // A/B Testing Mode: Modern implementation for both eyes (since we changed leftEyelidEstimator to ModernEyelidEstimator)
      // Left eye: Modern implementation with calibrated data
      psvr2_toolkit::EyeData leftEyeData = leftEyelidEstimator.ConvertFromHmd2Gaze(pGazeState->leftEye);
      // Apply calibration compensation
      if (headsetCalibrator.IsCalibrationStable()) {
        leftEyeData.gazeDir = leftCalibrated.compensatedGazeDir;
        leftEyeData.pupilPosY = leftCalibrated.compensatedPupilPos.y;
        leftEyeData.pupilDiaMm = leftCalibrated.compensatedPupilDia;
      }
      psvr2_toolkit::EstimationResult leftResult = leftEyelidEstimator.Estimate(leftEyeData);
      leftEyelidOpenness = leftResult.openness;

      // Right eye: Modern implementation with calibrated data
      psvr2_toolkit::EyeData rightEyeData = rightEyelidEstimator.ConvertFromHmd2Gaze(pGazeState->rightEye);
      // Apply calibration compensation
      if (headsetCalibrator.IsCalibrationStable()) {
        rightEyeData.gazeDir = rightCalibrated.compensatedGazeDir;
        rightEyeData.pupilPosY = rightCalibrated.compensatedPupilPos.y;
        rightEyeData.pupilDiaMm = rightCalibrated.compensatedPupilDia;
      }
      psvr2_toolkit::EstimationResult rightResult = rightEyelidEstimator.Estimate(rightEyeData);
      rightEyelidOpenness = rightResult.openness;
    } else if (USE_NEW_IMPLEMENTATION_BOTH_EYES) {
      // Modern Implementation for Both Eyes with headset calibration
      psvr2_toolkit::EyeData leftEyeData = leftEyelidEstimator.ConvertFromHmd2Gaze(pGazeState->leftEye);
      psvr2_toolkit::EyeData rightEyeData = rightEyelidEstimator.ConvertFromHmd2Gaze(pGazeState->rightEye);

      // Apply calibration compensation if calibration is stable
      if (headsetCalibrator.IsCalibrationStable()) {
        leftEyeData.gazeDir = leftCalibrated.compensatedGazeDir;
        leftEyeData.pupilPosY = leftCalibrated.compensatedPupilPos.y;
        leftEyeData.pupilDiaMm = leftCalibrated.compensatedPupilDia;

        rightEyeData.gazeDir = rightCalibrated.compensatedGazeDir;
        rightEyeData.pupilPosY = rightCalibrated.compensatedPupilPos.y;
        rightEyeData.pupilDiaMm = rightCalibrated.compensatedPupilDia;
      }

      // Use individual eye estimation for independent movement
      psvr2_toolkit::EstimationResult leftResult = leftEyelidEstimator.Estimate(leftEyeData);
      psvr2_toolkit::EstimationResult rightResult = rightEyelidEstimator.Estimate(rightEyeData);

      leftEyelidOpenness = leftResult.openness;
      rightEyelidOpenness = rightResult.openness;
    } else {
      // Fallback: Convert Hmd2GazeEye to EyeData and use modern estimators with calibration
      psvr2_toolkit::EyeData leftEyeData = leftEyelidEstimator.ConvertFromHmd2Gaze(pGazeState->leftEye);
      psvr2_toolkit::EyeData rightEyeData = rightEyelidEstimator.ConvertFromHmd2Gaze(pGazeState->rightEye);

      // Apply calibration compensation if available
      if (headsetCalibrator.IsCalibrationStable()) {
        leftEyeData.gazeDir = leftCalibrated.compensatedGazeDir;
        leftEyeData.pupilPosY = leftCalibrated.compensatedPupilPos.y;
        leftEyeData.pupilDiaMm = leftCalibrated.compensatedPupilDia;

        rightEyeData.gazeDir = rightCalibrated.compensatedGazeDir;
        rightEyeData.pupilPosY = rightCalibrated.compensatedPupilPos.y;
        rightEyeData.pupilDiaMm = rightCalibrated.compensatedPupilDia;
      }

      // Use individual eye estimation for final fallback
      psvr2_toolkit::EstimationResult leftResult = leftEyelidEstimator.Estimate(leftEyeData);
      psvr2_toolkit::EstimationResult rightResult = rightEyelidEstimator.Estimate(rightEyeData);

      leftEyelidOpenness = leftResult.openness;
      rightEyelidOpenness = rightResult.openness;
    }

    pIpcServer->UpdateGazeState(pGazeState, leftEyelidOpenness, rightEyelidOpenness);
  }
  else if (buffer[0] == GAZE_MAGIC_0 && buffer[1] == GAZE_MAGIC_1_RAW) {
    // RAW packet detected - likely contains raw eye camera images!
    static int rawPacketCount = 0;
    static FILE* rawDumpFile = nullptr;

    // Log detection (once)
    if (rawPacketCount == 0) {
      Util::DriverLog("[PSVR2Toolkit] RAW gaze packet detected! Size: %d bytes", result);

      // Get DLL directory
      char dllPath[MAX_PATH];
      HMODULE hModule = nullptr;
      if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                             GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                             (LPCSTR)&g_dummyForModuleHandle, &hModule)) {
        GetModuleFileNameA(hModule, dllPath, MAX_PATH);
        std::string dllDir(dllPath);
        size_t lastSlash = dllDir.find_last_of("\\/");
        if (lastSlash != std::string::npos) {
          dllDir = dllDir.substr(0, lastSlash);
        }

        // Open dump file
        std::string dumpPath = dllDir + "\\psvr2_raw_gaze_packet.bin";
#ifdef _MSC_VER
        fopen_s(&rawDumpFile, dumpPath.c_str(), "wb");
#else
        rawDumpFile = fopen(dumpPath.c_str(), "wb");
#endif
        if (rawDumpFile) {
          Util::DriverLog("[PSVR2Toolkit] Dumping RAW packet to: %s", dumpPath.c_str());
          // Write first RAW packet to file
          fwrite(buffer, 1, result, rawDumpFile);
          fclose(rawDumpFile);
          rawDumpFile = nullptr;
        }
      }
    }

    rawPacketCount++;

    // Log statistics every 60 packets (~1 second at 60Hz)
    if (rawPacketCount % 60 == 0) {
      Util::DriverLog("[PSVR2Toolkit] Received %d RAW gaze packets (size: %d bytes)",
                      rawPacketCount, result);
    }
  }
  else if (buffer[0] == GAZE_MAGIC_0 && buffer[1] == GAZE_MAGIC_1_CAL) {
    // CAL packet detected - likely contains calibrated/enhanced data
    static int calPacketCount = 0;
    static FILE* calDumpFile = nullptr;

    // Log detection (once)
    if (calPacketCount == 0) {
      Util::DriverLog("[PSVR2Toolkit] CAL gaze packet detected! Size: %d bytes", result);

      // Get DLL directory
      char dllPath[MAX_PATH];
      HMODULE hModule = nullptr;
      if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                             GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                             (LPCSTR)&g_dummyForModuleHandle, &hModule)) {
        GetModuleFileNameA(hModule, dllPath, MAX_PATH);
        std::string dllDir(dllPath);
        size_t lastSlash = dllDir.find_last_of("\\/");
        if (lastSlash != std::string::npos) {
          dllDir = dllDir.substr(0, lastSlash);
        }

        // Open dump file
        std::string dumpPath = dllDir + "\\psvr2_cal_gaze_packet.bin";
#ifdef _MSC_VER
        fopen_s(&calDumpFile, dumpPath.c_str(), "wb");
#else
        calDumpFile = fopen(dumpPath.c_str(), "wb");
#endif
        if (calDumpFile) {
          Util::DriverLog("[PSVR2Toolkit] Dumping CAL packet to: %s", dumpPath.c_str());
          // Write first CAL packet to file
          fwrite(buffer, 1, result, calDumpFile);
          fclose(calDumpFile);
          calDumpFile = nullptr;
        }
      }
    }

    calPacketCount++;

    // Log statistics every 60 packets
    if (calPacketCount % 60 == 0) {
      Util::DriverLog("[PSVR2Toolkit] Received %d CAL gaze packets (size: %d bytes)",
                      calPacketCount, result);
    }
  }
  else if (buffer[0] == GAZE_MAGIC_0) {
    // Unknown gaze packet type
    static bool unknownLogged = false;
    if (!unknownLogged) {
      Util::DriverLog("[PSVR2Toolkit] Unknown gaze packet type: 0x%02X 0x%02X",
                      (uint8_t)buffer[0], (uint8_t)buffer[1]);
      unknownLogged = true;
    }
  }

  return 0;
}
