#pragma once

#include "hmd2_gaze.h"
#include <cmath>
#include <vector>

namespace psvr2_toolkit {

  struct Vector3 {
    float x, y, z;
    Vector3(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
    
    float Magnitude() const {
      return std::sqrt(x * x + y * y + z * z);
    }
    
    Vector3 Normalized() const {
      float mag = Magnitude();
      if (mag > 1e-6f) {
        return Vector3(x / mag, y / mag, z / mag);
      }
      return Vector3(0, 0, 1);
    }
    
    float Dot(const Vector3& other) const {
      return x * other.x + y * other.y + z * other.z;
    }
    
    Vector3 Cross(const Vector3& other) const {
      return Vector3(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x
      );
    }
  };

  struct HeadsetCalibration {
    Vector3 cameraPosition;      // Camera position relative to eyes
    Vector3 cameraDirection;     // Camera viewing direction
    float cameraTiltAngle;       // Vertical tilt of camera (degrees)
    float cameraYawAngle;        // Horizontal yaw of camera (degrees)
    float headsetHeightOffset;   // Vertical offset of headset (mm)
    float screenToEyeDistance;   // Distance from screen to eyes (mm)
    float cameraToEyeDistance;   // Distance from camera to eyes (mm)
    float confidence;            // Calibration confidence (0-1)
    bool isCalibrated;           // Whether calibration is complete
    
    HeadsetCalibration() 
      : cameraPosition(0, 0, 0)
      , cameraDirection(0, 0, 1)
      , cameraTiltAngle(0)
      , cameraYawAngle(0)
      , headsetHeightOffset(0)
      , screenToEyeDistance(65.0f)  // Default IPD-like distance
      , cameraToEyeDistance(50.0f)   // Default camera distance
      , confidence(0)
      , isCalibrated(false) {}
  };

  struct CalibratedEyeData {
    Vector3 gazeDir;
    Vector3 pupilPos;
    float pupilDia;
    bool isBlink;
    bool isValid;
    
    // Calibration-compensated values
    Vector3 compensatedGazeDir;
    Vector3 compensatedPupilPos;
    float compensatedPupilDia;
    
    CalibratedEyeData() 
      : gazeDir(0,0,1), pupilPos(0,0,0), pupilDia(0)
      , isBlink(false), isValid(false)
      , compensatedGazeDir(0,0,1), compensatedPupilPos(0,0,0), compensatedPupilDia(0) {}
  };

  class HeadsetCalibrator {
  public:
    HeadsetCalibrator();
    
    // Main calibration function
    void UpdateCalibration(const Hmd2GazeEye& leftEye, const Hmd2GazeEye& rightEye);
    
    // Apply calibration to eye data
    CalibratedEyeData CalibrateEyeData(const Hmd2GazeEye& eye) const;
    
    // Get current calibration
    const HeadsetCalibration& GetCalibration() const { return m_calibration; }
    
    // Force recalibration
    void ResetCalibration();
    
    // Check if calibration is stable
    bool IsCalibrationStable() const;

  private:
    HeadsetCalibration m_calibration;
    
    // Calibration state
    std::vector<Vector3> m_gazeHistory;
    std::vector<Vector3> m_pupilPosHistory;
    int m_sampleCount;
    bool m_calibrationInProgress;
    
    // Configuration
    struct Config {
      int minSamplesForCalibration = 200;  // Minimum samples needed
      float maxGazeDeviation = 0.1f;        // Max gaze deviation for stable calibration
      float learningRate = 0.01f;           // Calibration learning rate (slow, stable)
      float fastLearningRate = 0.05f;       // Fast learning rate (for headset adjustments)
      float confidenceThreshold = 0.7f;     // Minimum confidence for stable calibration
      float changeThreshold = 0.15f;        // Threshold for detecting headset adjustments
      int adaptationWindow = 50;            // Samples to analyze for change detection
    } m_config;
    
    // Calibration methods
    void DetectCameraAngle(const std::vector<Vector3>& gazeHistory);
    void DetectHeadsetPosition(const std::vector<Vector3>& pupilPosHistory);
    void DetectScreenToEyeDistance(const std::vector<Vector3>& gazeHistory, const std::vector<Vector3>& pupilPosHistory);
    void CalculateCameraGeometry();
    
    // Continuous adaptation methods
    bool DetectHeadsetAdjustment();
    void AdaptToHeadsetChange();
    float CalculateChangeMagnitude(const std::vector<Vector3>& recentHistory, const std::vector<Vector3>& olderHistory) const;
    
    // Utility functions
    Vector3 ConvertFromHmd2Gaze(const Hmd2GazeEye& eye) const;
    Vector3 ConvertFromHmd2PupilPos(const Hmd2GazeEye& eye) const;
    float CalculateAngleBetweenVectors(const Vector3& v1, const Vector3& v2) const;
    Vector3 RotateVector(const Vector3& vector, const Vector3& axis, float angle) const;
    
    // Compensation functions
    Vector3 CompensateGazeDirection(const Vector3& gazeDir) const;
    Vector3 CompensatePupilPosition(const Vector3& pupilPos) const;
    float CompensatePupilDiameter(const Vector3& gazeDir, float pupilDia) const;
  };

}
