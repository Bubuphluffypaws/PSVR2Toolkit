#include "headset_calibrator.h"
#include <algorithm>

namespace psvr2_toolkit {

  HeadsetCalibrator::HeadsetCalibrator()
    : m_sampleCount(0)
    , m_calibrationInProgress(false)
  {
    m_gazeHistory.reserve(1000);
    m_pupilPosHistory.reserve(1000);
  }

  void HeadsetCalibrator::UpdateCalibration(const Hmd2GazeEye& leftEye, const Hmd2GazeEye& rightEye) {
    if (!leftEye.isGazeDirValid || !rightEye.isGazeDirValid ||
        !leftEye.isPupilPosInSensorValid || !rightEye.isPupilPosInSensorValid) {
      return;
    }

    // Convert to our coordinate system
    Vector3 leftGaze = ConvertFromHmd2Gaze(leftEye);
    Vector3 rightGaze = ConvertFromHmd2Gaze(rightEye);
    Vector3 leftPupilPos = ConvertFromHmd2PupilPos(leftEye);
    Vector3 rightPupilPos = ConvertFromHmd2PupilPos(rightEye);

    // Calculate average gaze and pupil position
    Vector3 avgGaze = Vector3(
      (leftGaze.x + rightGaze.x) * 0.5f,
      (leftGaze.y + rightGaze.y) * 0.5f,
      (leftGaze.z + rightGaze.z) * 0.5f
    ).Normalized();

    Vector3 avgPupilPos = Vector3(
      (leftPupilPos.x + rightPupilPos.x) * 0.5f,
      (leftPupilPos.y + rightPupilPos.y) * 0.5f,
      (leftPupilPos.z + rightPupilPos.z) * 0.5f
    );

    // Store in history
    m_gazeHistory.push_back(avgGaze);
    m_pupilPosHistory.push_back(avgPupilPos);
    m_sampleCount++;

    // Keep history manageable
    if (m_gazeHistory.size() > 1000) {
      m_gazeHistory.erase(m_gazeHistory.begin(), m_gazeHistory.begin() + 100);
      m_pupilPosHistory.erase(m_pupilPosHistory.begin(), m_pupilPosHistory.begin() + 100);
    }

    // Start calibration after collecting enough samples
    if (m_sampleCount >= m_config.minSamplesForCalibration && !m_calibration.isCalibrated) {
      DetectCameraAngle(m_gazeHistory);
      DetectHeadsetPosition(m_pupilPosHistory);
      DetectScreenToEyeDistance(m_gazeHistory, m_pupilPosHistory);
      CalculateCameraGeometry();
      
      // Check if calibration is stable
      if (IsCalibrationStable()) {
        m_calibration.isCalibrated = true;
        m_calibration.confidence = 0.8f; // Start with good confidence
      }
    }

    // Continue refining calibration and detect headset adjustments
    if (m_calibration.isCalibrated) {
      // Check for headset adjustments more frequently
      if (m_sampleCount % 25 == 0) { // Every 25 samples (more responsive)
        if (DetectHeadsetAdjustment()) {
          AdaptToHeadsetChange();
        }
      }
      
      // Regular calibration refinement
      if (m_sampleCount % 100 == 0) {
        DetectCameraAngle(m_gazeHistory);
        DetectHeadsetPosition(m_pupilPosHistory);
        DetectScreenToEyeDistance(m_gazeHistory, m_pupilPosHistory);
        CalculateCameraGeometry();
      }
    }
  }

  CalibratedEyeData HeadsetCalibrator::CalibrateEyeData(const Hmd2GazeEye& eye) const {
    CalibratedEyeData result;
    
    if (!eye.isGazeDirValid || !eye.isPupilPosInSensorValid || !eye.isPupilDiaValid) {
      return result;
    }

    // Convert raw data
    result.gazeDir = ConvertFromHmd2Gaze(eye);
    result.pupilPos = ConvertFromHmd2PupilPos(eye);
    result.pupilDia = eye.pupilDiaMm;
    result.isBlink = (eye.isBlinkValid && eye.blink == HMD2_BOOL_TRUE);
    result.isValid = true;

    // Apply calibration compensation
    if (m_calibration.isCalibrated) {
      result.compensatedGazeDir = CompensateGazeDirection(result.gazeDir);
      result.compensatedPupilPos = CompensatePupilPosition(result.pupilPos);
      result.compensatedPupilDia = CompensatePupilDiameter(result.gazeDir, result.pupilDia);
    } else {
      // No calibration available, use raw data
      result.compensatedGazeDir = result.gazeDir;
      result.compensatedPupilPos = result.pupilPos;
      result.compensatedPupilDia = result.pupilDia;
    }

    return result;
  }

  void HeadsetCalibrator::ResetCalibration() {
    m_calibration = HeadsetCalibration();
    m_gazeHistory.clear();
    m_pupilPosHistory.clear();
    m_sampleCount = 0;
    m_calibrationInProgress = false;
  }

  bool HeadsetCalibrator::IsCalibrationStable() const {
    if (m_gazeHistory.size() < 100) return false;

    // Check gaze direction stability
    Vector3 avgGaze(0, 0, 0);
    for (const auto& gaze : m_gazeHistory) {
      avgGaze.x += gaze.x;
      avgGaze.y += gaze.y;
      avgGaze.z += gaze.z;
    }
    avgGaze = avgGaze.Normalized();

    float maxDeviation = 0.0f;
    for (const auto& gaze : m_gazeHistory) {
      float deviation = 1.0f - gaze.Dot(avgGaze);
      maxDeviation = std::max(maxDeviation, deviation);
    }

    return maxDeviation < m_config.maxGazeDeviation;
  }

  void HeadsetCalibrator::DetectCameraAngle(const std::vector<Vector3>& gazeHistory) {
    if (gazeHistory.size() < 50) return;

    // Calculate average gaze direction (this represents the user's "natural" forward gaze)
    Vector3 avgGaze(0, 0, 0);
    for (const auto& gaze : gazeHistory) {
      avgGaze.x += gaze.x;
      avgGaze.y += gaze.y;
      avgGaze.z += gaze.z;
    }
    avgGaze = avgGaze.Normalized();

    // The camera direction should be perpendicular to the user's natural gaze
    // If the user is looking "forward" but the camera sees them looking "up",
    // then the camera is tilted downward
    Vector3 expectedCameraDir(0, 0, 1); // Camera should look straight ahead
    Vector3 actualCameraDir = avgGaze;

    // Calculate tilt angle (vertical rotation)
    float tiltAngle = std::asin(std::clamp(std::abs(actualCameraDir.y), 0.0f, 1.0f));
    if (actualCameraDir.y < 0) tiltAngle = -tiltAngle; // Negative = camera looking down

    // Calculate yaw angle (horizontal rotation)
    float yawAngle = std::atan2(actualCameraDir.x, actualCameraDir.z);

    // Update calibration with smoothing
    float learningRate = m_config.learningRate;
    m_calibration.cameraTiltAngle = m_calibration.cameraTiltAngle * (1.0f - learningRate) + 
                                   tiltAngle * learningRate;
    m_calibration.cameraYawAngle = m_calibration.cameraYawAngle * (1.0f - learningRate) + 
                                 yawAngle * learningRate;
    m_calibration.cameraDirection = actualCameraDir;
  }

  void HeadsetCalibrator::DetectHeadsetPosition(const std::vector<Vector3>& pupilPosHistory) {
    if (pupilPosHistory.size() < 50) return;

    // Calculate average pupil position
    Vector3 avgPupilPos(0, 0, 0);
    for (const auto& pos : pupilPosHistory) {
      avgPupilPos.x += pos.x;
      avgPupilPos.y += pos.y;
      avgPupilPos.z += pos.z;
    }
    avgPupilPos.x /= pupilPosHistory.size();
    avgPupilPos.y /= pupilPosHistory.size();
    avgPupilPos.z /= pupilPosHistory.size();

    // The Y component tells us about headset height
    // Positive Y = headset positioned high (looking down at eyes)
    // Negative Y = headset positioned low (looking up at eyes)
    float learningRate = m_config.learningRate;
    m_calibration.headsetHeightOffset = m_calibration.headsetHeightOffset * (1.0f - learningRate) + 
                                       avgPupilPos.y * learningRate;
    m_calibration.cameraPosition = avgPupilPos;
  }

  void HeadsetCalibrator::DetectScreenToEyeDistance(const std::vector<Vector3>& gazeHistory, const std::vector<Vector3>& pupilPosHistory) {
    if (gazeHistory.size() < 50 || pupilPosHistory.size() < 50) return;

    // Method 1: Analyze pupil size variation with gaze angle
    // Closer eyes = more dramatic pupil size changes with gaze angle
    // Further eyes = less dramatic changes
    
    float pupilSizeVariation = 0.0f;
    float gazeAngleVariation = 0.0f;
    int validSamples = 0;
    
    for (size_t i = 0; i < gazeHistory.size() && i < pupilPosHistory.size(); i++) {
      const Vector3& gaze = gazeHistory[i];
      const Vector3& pupilPos = pupilPosHistory[i];
      
      // Calculate gaze angle from forward direction
      float gazeAngle = std::asin(std::clamp(std::abs(gaze.y), 0.0f, 1.0f));
      
      // Use pupil position magnitude as proxy for pupil size
      float pupilMagnitude = pupilPos.Magnitude();
      
      pupilSizeVariation += pupilMagnitude;
      gazeAngleVariation += gazeAngle;
      validSamples++;
    }
    
    if (validSamples > 0) {
      pupilSizeVariation /= validSamples;
      gazeAngleVariation /= validSamples;
      
      // Estimate distance based on pupil size variation
      // Closer eyes = larger pupil size variation
      // Further eyes = smaller pupil size variation
      float estimatedDistance = 50.0f + (pupilSizeVariation * 20.0f); // Rough estimation
      estimatedDistance = std::clamp(estimatedDistance, 30.0f, 100.0f); // Reasonable range
      
      float learningRate = m_config.learningRate;
      m_calibration.screenToEyeDistance = m_calibration.screenToEyeDistance * (1.0f - learningRate) + 
                                        estimatedDistance * learningRate;
    }
    
    // Method 2: Analyze camera-to-eye distance from pupil position patterns
    // Calculate average distance from camera to pupil center
    float avgCameraDistance = 0.0f;
    int distanceSamples = 0;
    
    for (const auto& pupilPos : pupilPosHistory) {
      float distance = pupilPos.Magnitude();
      if (distance > 0.1f && distance < 2.0f) { // Reasonable range
        avgCameraDistance += distance;
        distanceSamples++;
      }
    }
    
    if (distanceSamples > 0) {
      avgCameraDistance /= distanceSamples;
      
      // Convert to millimeters (assuming sensor coordinates are normalized)
      float cameraDistanceMm = avgCameraDistance * 100.0f; // Rough conversion
      cameraDistanceMm = std::clamp(cameraDistanceMm, 20.0f, 80.0f); // Reasonable range
      
      float learningRate = m_config.learningRate;
      m_calibration.cameraToEyeDistance = m_calibration.cameraToEyeDistance * (1.0f - learningRate) + 
                                        cameraDistanceMm * learningRate;
    }
  }

  void HeadsetCalibrator::CalculateCameraGeometry() {
    // Calculate camera position relative to eyes
    // This helps us understand the viewing angle
    Vector3 eyeToCamera = m_calibration.cameraPosition;
    float distance = eyeToCamera.Magnitude();
    
    // Update confidence based on consistency
    if (distance > 0.1f && distance < 2.0f) { // Reasonable distance range
      m_calibration.confidence = std::min(1.0f, m_calibration.confidence + 0.001f);
    } else {
      m_calibration.confidence = std::max(0.0f, m_calibration.confidence - 0.01f);
    }
  }

  Vector3 HeadsetCalibrator::ConvertFromHmd2Gaze(const Hmd2GazeEye& eye) const {
    return Vector3(eye.gazeDirNorm.x, eye.gazeDirNorm.y, eye.gazeDirNorm.z);
  }

  Vector3 HeadsetCalibrator::ConvertFromHmd2PupilPos(const Hmd2GazeEye& eye) const {
    return Vector3(eye.pupilPosInSensor.x, eye.pupilPosInSensor.y, 0);
  }

  float HeadsetCalibrator::CalculateAngleBetweenVectors(const Vector3& v1, const Vector3& v2) const {
    float dot = v1.Dot(v2);
    return std::acos(std::clamp(dot, -1.0f, 1.0f));
  }

  Vector3 HeadsetCalibrator::RotateVector(const Vector3& vector, const Vector3& axis, float angle) const {
    // Rodrigues' rotation formula
    float cosAngle = std::cos(angle);
    float sinAngle = std::sin(angle);
    
    Vector3 result = Vector3(
      vector.x * cosAngle,
      vector.y * cosAngle,
      vector.z * cosAngle
    );
    
    Vector3 cross = axis.Cross(vector);
    result.x += cross.x * sinAngle;
    result.y += cross.y * sinAngle;
    result.z += cross.z * sinAngle;
    
    Vector3 dot = Vector3(axis.x * axis.Dot(vector), axis.y * axis.Dot(vector), axis.z * axis.Dot(vector));
    result.x += dot.x * (1.0f - cosAngle);
    result.y += dot.y * (1.0f - cosAngle);
    result.z += dot.z * (1.0f - cosAngle);
    
    return result;
  }

  Vector3 HeadsetCalibrator::CompensateGazeDirection(const Vector3& gazeDir) const {
    if (!m_calibration.isCalibrated) return gazeDir;

    // Compensate for camera tilt and yaw
    Vector3 compensated = gazeDir;
    
    // Apply tilt compensation (vertical rotation)
    Vector3 tiltAxis(1, 0, 0); // Rotate around X axis
    compensated = RotateVector(compensated, tiltAxis, -m_calibration.cameraTiltAngle);
    
    // Apply yaw compensation (horizontal rotation)
    Vector3 yawAxis(0, 1, 0); // Rotate around Y axis
    compensated = RotateVector(compensated, yawAxis, -m_calibration.cameraYawAngle);
    
    return compensated.Normalized();
  }

  Vector3 HeadsetCalibrator::CompensatePupilPosition(const Vector3& pupilPos) const {
    if (!m_calibration.isCalibrated) return pupilPos;

    // Compensate for headset height offset
    Vector3 compensated = pupilPos;
    compensated.y -= m_calibration.headsetHeightOffset;
    
    // Compensate for Z-distance effects on pupil position
    // Closer eyes = more sensitive to position changes
    // Further eyes = less sensitive to position changes
    float referenceDistance = 65.0f; // Reference distance (mm)
    float distanceRatio = referenceDistance / m_calibration.screenToEyeDistance;
    
    // Apply distance scaling to position sensitivity
    compensated.x *= distanceRatio;
    compensated.y *= distanceRatio;
    
    return compensated;
  }

  float HeadsetCalibrator::CompensatePupilDiameter(const Vector3& gazeDir, float pupilDia) const {
    if (!m_calibration.isCalibrated) return pupilDia;

    // Compensate for viewing angle effects
    Vector3 compensatedGaze = CompensateGazeDirection(gazeDir);
    
    // Calculate the angle between gaze direction and camera direction
    float viewingAngle = CalculateAngleBetweenVectors(compensatedGaze, Vector3(0, 0, 1));
    
    // Apply cosine correction for elliptical projection
    float angleCorrectionFactor = 1.0f / std::max(std::cos(viewingAngle), 0.1f);
    angleCorrectionFactor = std::min(angleCorrectionFactor, 3.0f); // Cap the correction
    
    // Compensate for Z-distance (screen-to-eye distance)
    // Closer eyes = larger apparent pupil size
    // Further eyes = smaller apparent pupil size
    float referenceDistance = 65.0f; // Reference distance (mm)
    float distanceRatio = referenceDistance / m_calibration.screenToEyeDistance;
    float distanceCorrectionFactor = std::clamp(distanceRatio, 0.5f, 2.0f); // Reasonable range
    
    // Combine both corrections
    float totalCorrectionFactor = angleCorrectionFactor * distanceCorrectionFactor;
    totalCorrectionFactor = std::min(totalCorrectionFactor, 4.0f); // Cap total correction
    
    return pupilDia * totalCorrectionFactor;
  }

  bool HeadsetCalibrator::DetectHeadsetAdjustment() {
    if (m_gazeHistory.size() < m_config.adaptationWindow * 2 || 
        m_pupilPosHistory.size() < m_config.adaptationWindow * 2) {
      return false;
    }

    // Compare recent samples vs older samples
    int windowSize = m_config.adaptationWindow;
    int recentStart = m_gazeHistory.size() - windowSize;
    int olderStart = recentStart - windowSize;

    // Extract recent and older history
    std::vector<Vector3> recentGaze(m_gazeHistory.begin() + recentStart, m_gazeHistory.end());
    std::vector<Vector3> olderGaze(m_gazeHistory.begin() + olderStart, m_gazeHistory.begin() + recentStart);
    
    std::vector<Vector3> recentPupilPos(m_pupilPosHistory.begin() + recentStart, m_pupilPosHistory.end());
    std::vector<Vector3> olderPupilPos(m_pupilPosHistory.begin() + olderStart, m_pupilPosHistory.begin() + recentStart);

    // Calculate change magnitude for gaze and pupil position
    float gazeChange = CalculateChangeMagnitude(recentGaze, olderGaze);
    float pupilChange = CalculateChangeMagnitude(recentPupilPos, olderPupilPos);

    // Detect significant changes that might indicate headset adjustment
    bool significantGazeChange = gazeChange > m_config.changeThreshold;
    bool significantPupilChange = pupilChange > m_config.changeThreshold;

    return significantGazeChange || significantPupilChange;
  }

  void HeadsetCalibrator::AdaptToHeadsetChange() {
    // Use faster learning rate for headset adjustments
    float originalLearningRate = m_config.learningRate;
    m_config.learningRate = m_config.fastLearningRate;

    // Recalibrate with recent data only
    int recentWindow = std::min(100, (int)m_gazeHistory.size());
    std::vector<Vector3> recentGaze(m_gazeHistory.end() - recentWindow, m_gazeHistory.end());
    std::vector<Vector3> recentPupilPos(m_pupilPosHistory.end() - recentWindow, m_pupilPosHistory.end());

    // Quick recalibration
    DetectCameraAngle(recentGaze);
    DetectHeadsetPosition(recentPupilPos);
    DetectScreenToEyeDistance(recentGaze, recentPupilPos);
    CalculateCameraGeometry();

    // Restore original learning rate
    m_config.learningRate = originalLearningRate;

    // Reduce confidence temporarily to indicate adaptation
    m_calibration.confidence = std::max(0.3f, m_calibration.confidence - 0.2f);
  }

  float HeadsetCalibrator::CalculateChangeMagnitude(const std::vector<Vector3>& recentHistory, const std::vector<Vector3>& olderHistory) const {
    if (recentHistory.size() != olderHistory.size() || recentHistory.empty()) {
      return 0.0f;
    }

    // Calculate average vectors
    Vector3 recentAvg(0, 0, 0);
    Vector3 olderAvg(0, 0, 0);

    for (size_t i = 0; i < recentHistory.size(); i++) {
      recentAvg.x += recentHistory[i].x;
      recentAvg.y += recentHistory[i].y;
      recentAvg.z += recentHistory[i].z;
      
      olderAvg.x += olderHistory[i].x;
      olderAvg.y += olderHistory[i].y;
      olderAvg.z += olderHistory[i].z;
    }

    recentAvg.x /= recentHistory.size();
    recentAvg.y /= recentHistory.size();
    recentAvg.z /= recentHistory.size();
    
    olderAvg.x /= olderHistory.size();
    olderAvg.y /= olderHistory.size();
    olderAvg.z /= olderHistory.size();

    // Calculate magnitude of change
    Vector3 change = Vector3(
      recentAvg.x - olderAvg.x,
      recentAvg.y - olderAvg.y,
      recentAvg.z - olderAvg.z
    );

    return change.Magnitude();
  }

}
