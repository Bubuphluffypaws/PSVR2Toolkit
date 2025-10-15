#include "modern_eyelid_estimator.h"
#include <algorithm>
#include <cmath>

namespace psvr2_toolkit {

  ModernEyelidEstimator::ModernEyelidEstimator()
    : m_learnedNeutralGaze(0, 0, 1)
    , m_neutralGazeConfidence(0.0f)
    , m_gazeSampleCount(0)
  {}

  EstimationResult ModernEyelidEstimator::Estimate(const EyeData& leftEye, const EyeData& rightEye) {
    // 1. Update learning state
    UpdateNeutralGaze(leftEye, rightEye);
    UpdateReferences(leftEye, m_leftRefs);
    UpdateReferences(rightEye, m_rightRefs);
    
    // 2. Measure all cues for both eyes
    std::vector<CueMeasurement> allCues;
    
    // Left eye cues
    if (leftEye.isValid) {
      allCues.push_back(MeasureDiameterCue(leftEye, m_leftRefs));
      allCues.push_back(MeasurePositionCue(leftEye, m_leftRefs));
      if (leftEye.isBlink) {
        allCues.push_back(MeasureBlinkCue(leftEye));
      }
    }
    
    // Right eye cues
    if (rightEye.isValid) {
      allCues.push_back(MeasureDiameterCue(rightEye, m_rightRefs));
      allCues.push_back(MeasurePositionCue(rightEye, m_rightRefs));
      if (rightEye.isBlink) {
        allCues.push_back(MeasureBlinkCue(rightEye));
      }
    }
    
    // 3. Fuse all cues using uncertainty weighting
    float openness = FuseCues(allCues);
    
    // 4. Apply temporal smoothing
    static float lastOpenness = 0.5f;
    openness = lastOpenness * (1.0f - m_config.smoothingAlpha) + 
               openness * m_config.smoothingAlpha;
    lastOpenness = openness;
    
    // 5. Calculate confidence
    float confidence = CalculateOverallConfidence(allCues);
    
    return {openness, confidence, DeterminePrimaryCue(allCues)};
  }

  EstimationResult ModernEyelidEstimator::Estimate(const EyeData& eye) {
    // Single eye estimation for A/B testing
    UpdateReferences(eye, m_leftRefs);  // Use left refs for single eye
    
    std::vector<CueMeasurement> cues;
    if (eye.isValid) {
      cues.push_back(MeasureDiameterCue(eye, m_leftRefs));
      cues.push_back(MeasurePositionCue(eye, m_leftRefs));
      if (eye.isBlink) {
        cues.push_back(MeasureBlinkCue(eye));
      }
    }
    
    float openness = FuseCues(cues);
    
    // Apply temporal smoothing
    static float lastOpenness = 0.5f;
    openness = lastOpenness * (1.0f - m_config.smoothingAlpha) + 
               openness * m_config.smoothingAlpha;
    lastOpenness = openness;
    
    float confidence = CalculateOverallConfidence(cues);
    
    return {openness, confidence, DeterminePrimaryCue(cues)};
  }

  CueMeasurement ModernEyelidEstimator::MeasureDiameterCue(const EyeData& eye, const GazeAwareReferences& refs) {
    // Correct for gaze angle ellipticity
    float gazeAngle = CalculateGazeAngle(eye.gazeDir);
    float correctionFactor = 1.0f / std::max(std::cos(gazeAngle), 0.1f);
    float correctedDia = eye.pupilDiaMm * correctionFactor;
    
    // Normalize using gaze-aware references
    float denom = std::max(refs.openDia.value - refs.closedDia.value, 1e-6f);
    float normalizedDia = (correctedDia - refs.closedDia.value) / denom;
    normalizedDia = std::clamp(normalizedDia, 0.0f, 1.0f);
    
    // Calculate uncertainty based on gaze angle and reference stability
    float gazeUncertainty = std::sin(gazeAngle);  // Higher angle = more uncertainty
    float refUncertainty = (refs.openDia.stability + refs.closedDia.stability) * 0.5f;
    float totalUncertainty = std::sqrt(gazeUncertainty * gazeUncertainty + 
                                      refUncertainty * refUncertainty);
    
    return {normalizedDia, totalUncertainty, 1.0f - totalUncertainty, "diameter"};
  }

  CueMeasurement ModernEyelidEstimator::MeasurePositionCue(const EyeData& eye, const GazeAwareReferences& refs) {
    // Normalize position
    float denom = std::max(refs.openPosY.value - refs.closedPosY.value, 1e-6f);
    float normalizedPos = (eye.pupilPosY - refs.closedPosY.value) / denom;
    normalizedPos = std::clamp(normalizedPos, 0.0f, 1.0f);
    
    // Position is less affected by gaze angle than diameter
    float gazeAngle = CalculateGazeAngle(eye.gazeDir);
    float uncertainty = std::sin(gazeAngle) * 0.5f;  // Less sensitive to gaze angle
    
    return {normalizedPos, uncertainty, 1.0f - uncertainty, "position"};
  }

  CueMeasurement ModernEyelidEstimator::MeasureBlinkCue(const EyeData& eye) {
    // Blink cue is very reliable but indicates closed state
    return {0.0f, 0.0f, 1.0f, "blink"};
  }

  float ModernEyelidEstimator::CalculateGazeAngle(const Vector3& gazeDir) const {
    // Calculate vertical gaze angle (up/down)
    return std::asin(std::clamp(std::abs(gazeDir.y), 0.0f, 1.0f));
  }

  bool ModernEyelidEstimator::IsNeutralGaze(const Vector3& gazeDir) const {
    if (m_neutralGazeConfidence > 0.5f) {
      // Use learned neutral gaze
      float dotProduct = gazeDir.Dot(m_learnedNeutralGaze);
      return dotProduct > m_config.neutralGazeThreshold;
    } else {
      // Fallback to assumed forward direction
      return gazeDir.z > 0.94f;
    }
  }

  void ModernEyelidEstimator::UpdateReferences(const EyeData& eye, GazeAwareReferences& refs) {
    if (!eye.isValid) return;
    
    // Determine learning rate based on gaze angle and stability
    float gazeAngle = CalculateGazeAngle(eye.gazeDir);
    float angleConfidence = std::cos(gazeAngle);  // Higher confidence at neutral gaze
    
    if (eye.isBlink) {
      // Fast learning for closed references
      refs.closedDia.Update(eye.pupilDiaMm, 0.8f);
      refs.closedPosY.Update(eye.pupilPosY, 0.8f);
    } else if (IsNeutralGaze(eye.gazeDir)) {
      // Slower learning for open references, only at neutral gaze
      refs.openDia.Update(eye.pupilDiaMm, 0.3f * angleConfidence);
      refs.openPosY.Update(eye.pupilPosY, 0.3f * angleConfidence);
    }
    
    // Update angle-specific references
    int angleBin = static_cast<int>(gazeAngle * m_config.gazeAngleBins);
    if (refs.angleSpecificRefs.find(angleBin) == refs.angleSpecificRefs.end()) {
      refs.angleSpecificRefs[angleBin] = AdaptiveReference(eye.pupilDiaMm, 0.01f);
    }
    refs.angleSpecificRefs[angleBin].Update(eye.pupilDiaMm, angleConfidence);
  }

  void ModernEyelidEstimator::UpdateNeutralGaze(const EyeData& leftEye, const EyeData& rightEye) {
    if (!leftEye.isValid || !rightEye.isValid) return;
    
    m_gazeSampleCount++;
    
    // Calculate average gaze direction
    Vector3 avgGaze = Vector3(
      (leftEye.gazeDir.x + rightEye.gazeDir.x) * 0.5f,
      (leftEye.gazeDir.y + rightEye.gazeDir.y) * 0.5f,
      (leftEye.gazeDir.z + rightEye.gazeDir.z) * 0.5f
    ).Normalized();
    
    if (m_gazeSampleCount == 1) {
      // First sample - initialize
      m_learnedNeutralGaze = avgGaze;
      m_neutralGazeConfidence = 0.1f;
    } else {
      // Update using exponential moving average
      float learningRate = std::min(0.01f, 1.0f / m_gazeSampleCount);
      m_learnedNeutralGaze.x = m_learnedNeutralGaze.x * (1.0f - learningRate) + avgGaze.x * learningRate;
      m_learnedNeutralGaze.y = m_learnedNeutralGaze.y * (1.0f - learningRate) + avgGaze.y * learningRate;
      m_learnedNeutralGaze.z = m_learnedNeutralGaze.z * (1.0f - learningRate) + avgGaze.z * learningRate;
      
      // Normalize
      m_learnedNeutralGaze = m_learnedNeutralGaze.Normalized();
      
      // Increase confidence over time
      m_neutralGazeConfidence = std::min(1.0f, m_neutralGazeConfidence + 0.001f);
    }
  }

  float ModernEyelidEstimator::FuseCues(const std::vector<CueMeasurement>& cues) {
    if (cues.empty()) return 0.5f;
    
    // Weight by inverse uncertainty (higher confidence = higher weight)
    float weightedSum = 0.0f;
    float totalWeight = 0.0f;
    
    for (const auto& cue : cues) {
      if (cue.confidence < m_config.minConfidence) continue;
      
      float weight = cue.confidence / (cue.uncertainty + 1e-6f);
      weightedSum += cue.value * weight;
      totalWeight += weight;
    }
    
    return totalWeight > 0.0f ? weightedSum / totalWeight : 0.5f;
  }

  float ModernEyelidEstimator::CalculateOverallConfidence(const std::vector<CueMeasurement>& cues) {
    if (cues.empty()) return 0.0f;
    
    float totalConfidence = 0.0f;
    int validCues = 0;
    
    for (const auto& cue : cues) {
      if (cue.confidence >= m_config.minConfidence) {
        totalConfidence += cue.confidence;
        validCues++;
      }
    }
    
    return validCues > 0 ? totalConfidence / validCues : 0.0f;
  }

  std::string ModernEyelidEstimator::DeterminePrimaryCue(const std::vector<CueMeasurement>& cues) {
    if (cues.empty()) return "none";
    
    CueMeasurement bestCue = cues[0];
    float bestWeight = bestCue.confidence / (bestCue.uncertainty + 1e-6f);
    
    for (const auto& cue : cues) {
      float weight = cue.confidence / (cue.uncertainty + 1e-6f);
      if (weight > bestWeight) {
        bestCue = cue;
        bestWeight = weight;
      }
    }
    
    return bestCue.name;
  }

  EyeData ModernEyelidEstimator::ConvertFromHmd2Gaze(const Hmd2GazeEye& eye) const {
    EyeData data;
    
    if (eye.isPupilDiaValid) {
      data.pupilDiaMm = eye.pupilDiaMm;
    }
    
    if (eye.isPupilPosInSensorValid) {
      data.pupilPosY = eye.pupilPosInSensor.y;
    }
    
    if (eye.isGazeDirValid) {
      data.gazeDir = Vector3(eye.gazeDirNorm.x, eye.gazeDirNorm.y, eye.gazeDirNorm.z);
    }
    
    if (eye.isBlinkValid) {
      data.isBlink = (eye.blink == HMD2_BOOL_TRUE);
    }
    
    data.isValid = eye.isPupilDiaValid && eye.isPupilPosInSensorValid && eye.isGazeDirValid;
    
    return data;
  }

}
