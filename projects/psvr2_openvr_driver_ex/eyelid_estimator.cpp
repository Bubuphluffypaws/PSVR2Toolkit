#include "eyelid_estimator.h"

#include <algorithm>
#include <cmath>

namespace psvr2_toolkit {

  // parameters
  static constexpr float openLR = 0.001f; // slow open adaptation
  static constexpr float closedLR = 0.01f;  // faster closed adaptation
  static constexpr float smoothAlpha = 0.2f;   // EMA smoothing
  
  // Hybrid approach parameters for gaze angle correction
  static constexpr float gazeAngleCorrectionFactor = 1.5f; // Multiplier for gaze-dependent weighting reduction
                                                           // Higher values = more aggressive reduction of diameter weight at non-neutral gaze
                                                           // Range: 1.0-3.0, recommended: 1.5-2.0
  
  static constexpr float minGazeWeight = 0.2f; // Minimum weight for diameter cue at extreme gaze angles
                                               // Lower values = rely more on position cue when looking up/down
                                               // Range: 0.1-0.5, recommended: 0.2-0.3
  
  static constexpr float positionWeightBoost = 0.3f; // Additional weight given to position cue at non-neutral gaze
                                                     // Higher values = more reliance on pupil position when gaze is extreme
                                                     // Range: 0.1-0.5, recommended: 0.2-0.4
  
  static constexpr float minGazeDotProduct = 0.1f; // Minimum gaze dot product for diameter correction safety
                                                   // Prevents division by very small cos(angle) values
                                                   // Lower values = allow correction at more extreme angles
                                                   // Range: 0.05-0.3, recommended: 0.1-0.2
  
  static constexpr float maxDiameterCorrection = 2.0f; // Maximum factor for diameter correction
                                                       // Prevents unrealistic diameter values at extreme angles
                                                       // Higher values = allow more aggressive correction
                                                       // Range: 1.5-3.0, recommended: 1.8-2.5

  EyelidEstimator::EyelidEstimator()
    : m_openDia(4.0f)
    , m_closedDia(2.0f)
    , m_sensorYOpen(0.55f)
    , m_sensorYClosed(0.45f)
    , m_lastOpenness(1.0f)
  {}

  bool EyelidEstimator::IsNeutral(const Hmd2GazeEye &eye) const {
    if (eye.isGazeDirValid == HMD2_BOOL_FALSE) {
      return false;
    }

    // Forward assumed to be (0,0,1). Require within ~20deg of forward.
    return eye.gazeDirNorm.z > 0.94f;
  }

  float EyelidEstimator::Estimate(const Hmd2GazeEye &eye) {
    if (eye.isBlinkValid && eye.blink == HMD2_BOOL_TRUE) {
      // Update CLOSED reference on blink
      if (eye.isPupilDiaValid) {
        m_closedDia = m_closedDia * (1.0f - closedLR) +
          eye.pupilDiaMm * closedLR;
      }
      if (eye.isPupilPosInSensorValid) {
        m_sensorYClosed = m_sensorYClosed * (1.0f - closedLR) +
          eye.pupilPosInSensor.y * closedLR;
      }
    } else if (eye.isPupilDiaValid && eye.isPupilPosInSensorValid && IsNeutral(eye)) {
      // Update OPEN reference only if gaze is neutral
      bool stable = std::fabs(eye.pupilDiaMm - m_openDia) < 1.0f;
      if (stable) {
        m_openDia = m_openDia * (1.0f - openLR) +
          eye.pupilDiaMm * openLR;
        m_sensorYOpen = m_sensorYOpen * (1.0f - openLR) +
          eye.pupilPosInSensor.y * openLR;
      }
    }

    float opennessRaw = m_lastOpenness;
    if (eye.isBlinkValid && eye.blink == HMD2_BOOL_TRUE) {
      opennessRaw = 0.0f; // explicit blink
    } else if (eye.isPupilDiaValid && eye.isPupilPosInSensorValid) {
      // Calculate gaze-dependent diameter correction to account for pupil ellipticity
      float correctedDia = eye.pupilDiaMm;
      if (eye.isGazeDirValid) {
        // Calculate gaze angle from forward direction (z-component of gaze direction)
        // Clamp to prevent division by very small values (safety bound)
        float gazeDotProduct = std::clamp(eye.gazeDirNorm.z, minGazeDotProduct, 1.0f);
        float correctionFactor = 1.0f / gazeDotProduct; // cos(angle) correction
        
        // Apply correction with maximum bound to prevent unrealistic values
        correctionFactor = std::min(correctionFactor, maxDiameterCorrection);
        correctedDia = eye.pupilDiaMm * correctionFactor;
      }

      // Normalize corrected diameter (0-1 scale)
      float denomDia = std::max(1e-6f, m_openDia - m_closedDia);
      float normDia = (correctedDia - m_closedDia) / denomDia;
      normDia = std::clamp(normDia, 0.0f, 1.0f);

      // Normalize pupil position (0-1 scale)
      float denomPos = std::max(1e-6f, m_sensorYOpen - m_sensorYClosed);
      float normPos = (eye.pupilPosInSensor.y - m_sensorYClosed) / denomPos;
      normPos = std::clamp(normPos, 0.0f, 1.0f);

      // Enhanced gaze-dependent weighting with hybrid approach
      float gazeWeight = 1.0f;
      if (eye.isGazeDirValid) {
        float vertical = eye.gazeDirNorm.y; // +up, -down
        // More aggressive reduction of diameter weight at non-neutral gaze
        gazeWeight = std::clamp(1.0f - std::fabs(vertical) * gazeAngleCorrectionFactor, 
                               minGazeWeight, 1.0f);
      }

      // Dynamic weighting: increase position weight when gaze is non-neutral
      float baseDiameterWeight = 0.65f;
      float basePositionWeight = 0.35f;
      float positionBoost = (1.0f - gazeWeight) * positionWeightBoost;
      
      float diameterWeight = baseDiameterWeight - positionBoost;
      float positionWeight = basePositionWeight + positionBoost;

      // Combine cues with corrected diameter and enhanced weighting
      opennessRaw = gazeWeight * (diameterWeight * normDia + positionWeight * normPos)
        + (1.0f - gazeWeight) * normPos; // Use position only at extreme gaze angles
    } else {
      // Missing data ? decay toward closed
      opennessRaw = m_lastOpenness * 0.9f;
    }

    float openness = m_lastOpenness * (1.0f - smoothAlpha) +
      opennessRaw * smoothAlpha;
    m_lastOpenness = openness;

    return openness; // 0 = closed, 1 = open
  }

}

