#include "original_eyelid_estimator.h"
#include <algorithm>
#include <cmath>

namespace psvr2_toolkit {

  OriginalEyelidEstimator::OriginalEyelidEstimator()
    : m_openDia(4.0f)
    , m_closedDia(2.0f)
    , m_sensorYOpen(0.55f)
    , m_sensorYClosed(0.45f)
    , m_lastOpenness(0.5f)
  {}

  float OriginalEyelidEstimator::Estimate(const Hmd2GazeEye& eye) {
    // Original master implementation with pupil dilation normalization
    
    // Update dilation normalizer
    if (eye.isPupilDiaValid) {
      m_dilationNormalizer.UpdateBaseline(eye.pupilDiaMm);
    }
    
    // Update references based on blink state
    if (eye.isBlinkValid && eye.blink == HMD2_BOOL_TRUE) {
      // Update CLOSED reference
      if (eye.isPupilDiaValid) {
        m_closedDia = m_closedDia * (1.0f - closedLR) + eye.pupilDiaMm * closedLR;
      }
      if (eye.isPupilPosInSensorValid) {
        m_sensorYClosed = m_sensorYClosed * (1.0f - closedLR) + eye.pupilPosInSensor.y * closedLR;
      }
    } else if (eye.isPupilDiaValid && eye.isPupilPosInSensorValid) {
      // Update OPEN reference
      bool stable = std::fabs(eye.pupilDiaMm - m_openDia) < 1.0f;
      if (stable) {
        m_openDia = m_openDia * (1.0f - openLR) + eye.pupilDiaMm * openLR;
        m_sensorYOpen = m_sensorYOpen * (1.0f - openLR) + eye.pupilPosInSensor.y * openLR;
      }
    }

    // Calculate openness
    float opennessRaw = m_lastOpenness;
    if (eye.isBlinkValid && eye.blink == HMD2_BOOL_TRUE) {
      opennessRaw = 0.0f; // explicit blink
    } else if (eye.isPupilDiaValid && eye.isPupilPosInSensorValid) {
      // Apply pupil dilation normalization
      float normalizedDia = m_dilationNormalizer.NormalizeDiameter(eye.pupilDiaMm);
      
      // Original normalization as fallback
      float denomDia = std::max(1e-6f, m_openDia - m_closedDia);
      float refNormDia = (eye.pupilDiaMm - m_closedDia) / denomDia;
      refNormDia = std::clamp(refNormDia, 0.0f, 1.0f);

      // Blend dilation-normalized and reference-normalized values
      float normDia = normalizedDia * 0.7f + refNormDia * 0.3f;
      normDia = std::clamp(normDia, 0.0f, 1.0f);

      // Position normalization (unchanged)
      float denomPos = std::max(1e-6f, m_sensorYOpen - m_sensorYClosed);
      float normPos = (eye.pupilPosInSensor.y - m_sensorYClosed) / denomPos;
      normPos = std::clamp(normPos, 0.0f, 1.0f);

      // Simple weighted average (original 65%/35% split)
      opennessRaw = 0.65f * normDia + 0.35f * normPos;
    }

    // Apply smoothing
    float openness = m_lastOpenness * (1.0f - smoothAlpha) + opennessRaw * smoothAlpha;
    m_lastOpenness = openness;

    return openness;
  }

  void OriginalEyelidEstimator::Reset() {
    m_openDia = 4.0f;
    m_closedDia = 2.0f;
    m_sensorYOpen = 0.55f;
    m_sensorYClosed = 0.45f;
    m_lastOpenness = 0.5f;
    m_dilationNormalizer = PupilDilationNormalizer();
  }

  bool OriginalEyelidEstimator::IsNeutral(const Hmd2GazeEye& eye) const {
    if (eye.isGazeDirValid == HMD2_BOOL_FALSE) return false;
    return eye.gazeDirNorm.z > 0.94f; // Original simple threshold
  }

}
