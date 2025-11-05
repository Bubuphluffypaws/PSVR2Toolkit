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
    UpdateReferences(leftEye, m_leftRefs, m_leftEyeProfile);
    UpdateReferences(rightEye, m_rightRefs, m_rightEyeProfile);

    // 2. Measure all cues for both eyes
    std::vector<CueMeasurement> allCues;

    // Left eye cues
    if (leftEye.isValid) {
      allCues.push_back(MeasureDiameterCue(leftEye, m_leftRefs, m_leftEyeProfile));
      allCues.push_back(MeasurePositionCue(leftEye, m_leftRefs, m_leftEyeProfile));
      if (leftEye.isBlink) {
        allCues.push_back(MeasureBlinkCue(leftEye));
      }
    }

    // Right eye cues
    if (rightEye.isValid) {
      allCues.push_back(MeasureDiameterCue(rightEye, m_rightRefs, m_rightEyeProfile));
      allCues.push_back(MeasurePositionCue(rightEye, m_rightRefs, m_rightEyeProfile));
      if (rightEye.isBlink) {
        allCues.push_back(MeasureBlinkCue(rightEye));
      }
    }
    
    // 3. Fuse all cues using uncertainty weighting
    float openness = FuseCues(allCues);
    
    // 4. Direct blink override (like old algorithm) - OVERRIDE EVERYTHING
    if (leftEye.isBlink || rightEye.isBlink) {
      openness = 0.0f; // Immediate eye closure on any blink
      // Skip all smoothing for instant blink response
      float confidence = CalculateOverallConfidence(allCues);
      // Don't apply invertOutput to blinks - they should always be closed
      return {openness, confidence, "blink"};
    }
    
    // 5. Apply temporal smoothing (only for non-blink frames)
    // FIXED: Changed from static to member variable to prevent cross-instance contamination
    openness = m_lastOpenness * (1.0f - m_config.smoothingAlpha) +
               openness * m_config.smoothingAlpha;
    m_lastOpenness = openness;
    
    // 5. Calculate confidence
    float confidence = CalculateOverallConfidence(allCues);
    
    // 6. Apply inversion if needed
    if (m_config.invertOutput) {
      openness = 1.0f - openness;
    }
    
    return {openness, confidence, DeterminePrimaryCue(allCues)};
  }

  EstimationResult ModernEyelidEstimator::Estimate(const EyeData& eye) {
    // Single eye estimation for A/B testing
    UpdateReferences(eye, m_leftRefs, m_leftEyeProfile);  // Use left refs for single eye

    std::vector<CueMeasurement> cues;
    if (eye.isValid) {
      cues.push_back(MeasureDiameterCue(eye, m_leftRefs, m_leftEyeProfile));
      cues.push_back(MeasurePositionCue(eye, m_leftRefs, m_leftEyeProfile));
      if (eye.isBlink) {
        cues.push_back(MeasureBlinkCue(eye));
      }
    }
    
    float openness = FuseCues(cues);
    
    // Update eye geometry calibration
    if (m_config.enableEyeGeometryCalibration) {
      m_eyeGeometryCalibrator.UpdateCalibration(eye);
    }
    
    // Apply eye geometry compensation
    if (m_config.enableEyeGeometryCalibration && m_eyeGeometryCalibrator.IsGeometryCalibrated()) {
      openness = m_eyeGeometryCalibrator.GetCompensatedOpenness(openness, eye);
    }
    
    // Direct blink override (like old algorithm) - OVERRIDE EVERYTHING
    if (eye.isBlink) {
      openness = 0.0f; // Immediate eye closure on blink
      // Skip all smoothing and augmentation for instant blink response
      float confidence = CalculateOverallConfidence(cues);
      // Don't apply invertOutput to blinks - they should always be closed
      return {openness, confidence, "blink"};
    }
    
    // Apply blink augmentation if enabled (only for non-blink frames)
    if (m_config.enableBlinkAugmentation) {
      // Use a small delta time for blink tweener (assuming ~60fps)
      float deltaTime = 1.0f / 60.0f;
      
      // Update blink state and get blink-influenced openness
      float blinkInfluencedOpenness = m_blinkTweener.UpdateBlinkState(openness, deltaTime);
      
      // Blend normal estimation with blink-influenced result
      openness = openness * (1.0f - m_config.blinkOverrideStrength) + 
                 blinkInfluencedOpenness * m_config.blinkOverrideStrength;
    }
    
    // Apply enhanced smoothing system (only for non-blink frames)
    openness = m_smoothingSystem.Filter(openness);

    float confidence = CalculateOverallConfidence(cues);

    // Apply inversion if needed
    if (m_config.invertOutput) {
      openness = 1.0f - openness;
    }

    return {openness, confidence, DeterminePrimaryCue(cues)};
  }

  CueMeasurement ModernEyelidEstimator::MeasureDiameterCue(const EyeData& eye, const GazeAwareReferences& refs, EyeShapeProfile& profile) {
    // Update dilation normalizer with raw diameter
    m_dilationNormalizer.UpdateBaseline(eye.pupilDiaMm);

    // Correct for gaze angle ellipticity
    float gazeAngle = CalculateGazeAngle(eye.gazeDir);
    float correctionFactor = 1.0f / std::max(std::cos(gazeAngle), 0.1f);
    float correctedDia = eye.pupilDiaMm * correctionFactor;

    // Apply gaze-specific compensation from LUT
    float gazeCompensation = GetGazeCompensation(eye.gazeDir, refs);
    correctedDia *= gazeCompensation;

    // BUGFIX: Update profile with CORRECTED diameter, not raw
    // This prevents mismatch between what we track and what we normalize
    profile.UpdateProfile(correctedDia, eye.pupilPosY, eye.isBlink);

    // Apply pupil dilation normalization
    float normalizedDia = m_dilationNormalizer.NormalizeDiameter(correctedDia);

    // Use adaptive eye shape profile for normalization if available
    float profileNormalizedDia = profile.NormalizePupilDia(correctedDia);

    // Further normalize using gaze-aware references (tertiary normalization)
    float denom = std::max(refs.openDia.value - refs.closedDia.value, 1e-6f);
    float refNormalizedDia = (correctedDia - refs.closedDia.value) / denom;
    refNormalizedDia = std::clamp(refNormalizedDia, 0.0f, 1.0f);

    // Blend all normalization methods with adaptive weighting
    float blendedDia;
    if (profile.isProfileLearned) {
      // Use profile-based normalization as primary when available
      blendedDia = profileNormalizedDia * 0.5f + normalizedDia * 0.3f + refNormalizedDia * 0.2f;
    } else {
      // Fall back to dilation + reference when profile not yet learned
      blendedDia = normalizedDia * 0.7f + refNormalizedDia * 0.3f;
    }
    blendedDia = std::clamp(blendedDia, 0.0f, 1.0f);

    // Calculate uncertainty based on gaze angle, reference stability, dilation consistency, and profile learning
    float gazeUncertainty = std::sin(gazeAngle);  // Higher angle = more uncertainty
    float refUncertainty = (refs.openDia.stability + refs.closedDia.stability) * 0.5f;
    float dilationUncertainty = 1.0f - (m_dilationNormalizer.sampleCount > 100 ? 0.8f : 0.3f);
    float profileUncertainty = profile.isProfileLearned ? 0.2f : 0.6f;  // Lower uncertainty when profile is learned
    float totalUncertainty = std::sqrt(gazeUncertainty * gazeUncertainty +
                                      refUncertainty * refUncertainty +
                                      dilationUncertainty * dilationUncertainty +
                                      profileUncertainty * profileUncertainty) * 0.5f;  // Scale down

    return {blendedDia, totalUncertainty, 1.0f - totalUncertainty, "diameter"};
  }

  CueMeasurement ModernEyelidEstimator::MeasurePositionCue(const EyeData& eye, const GazeAwareReferences& refs, const EyeShapeProfile& profile) {
    // Use adaptive eye shape profile for normalization if available
    float profileNormalizedPos = profile.NormalizePupilPosY(eye.pupilPosY);

    // Normalize position using gaze-aware references
    float denom = std::max(refs.openPosY.value - refs.closedPosY.value, 1e-6f);
    float refNormalizedPos = (eye.pupilPosY - refs.closedPosY.value) / denom;
    refNormalizedPos = std::clamp(refNormalizedPos, 0.0f, 1.0f);

    // Apply gaze-specific compensation from LUT
    float gazeCompensation = GetGazeCompensation(eye.gazeDir, refs);
    float compensatedPos = refNormalizedPos * gazeCompensation;
    compensatedPos = std::clamp(compensatedPos, 0.0f, 1.0f);

    // Blend profile-based and reference-based normalization
    float normalizedPos;
    if (profile.isProfileLearned) {
      normalizedPos = profileNormalizedPos * 0.6f + compensatedPos * 0.4f;
    } else {
      normalizedPos = compensatedPos;
    }
    normalizedPos = std::clamp(normalizedPos, 0.0f, 1.0f);

    // Position is less affected by gaze angle than diameter
    float gazeAngle = CalculateGazeAngle(eye.gazeDir);
    float gazeUncertainty = std::sin(gazeAngle) * 0.5f;  // Less sensitive to gaze angle
    float profileUncertainty = profile.isProfileLearned ? 0.1f : 0.4f;
    float uncertainty = std::sqrt(gazeUncertainty * gazeUncertainty + profileUncertainty * profileUncertainty);

    return {normalizedPos, uncertainty, 1.0f - uncertainty, "position"};
  }

  CueMeasurement ModernEyelidEstimator::MeasureBlinkCue(const EyeData& eye) {
    // Blink cue is very reliable but indicates closed state
    return {0.0f, 0.0f, 1.0f, "blink"};
  }

  float ModernEyelidEstimator::CalculateGazeAngle(const Vector3& gazeDir) const {
    // Calculate vertical gaze angle (up/down)
    float absY = (gazeDir.y < 0) ? -gazeDir.y : gazeDir.y;
    float clampedY = (absY < 0.0f) ? 0.0f : (absY > 1.0f) ? 1.0f : absY;
    return std::asin(clampedY);
  }

  bool ModernEyelidEstimator::IsNeutralGaze(const Vector3& gazeDir) {
    if (m_neutralGazeConfidence > 0.5f) {
      // Use learned neutral gaze
      float dotProduct = gazeDir.Dot(m_learnedNeutralGaze);
      return dotProduct > m_config.neutralGazeThreshold;
    } else {
      // Fallback to assumed forward direction
      return gazeDir.z > 0.94f;
    }
  }

  void ModernEyelidEstimator::UpdateReferences(const EyeData& eye, GazeAwareReferences& refs, EyeShapeProfile& profile) {
    if (!eye.isValid) return;

    // Note: Eye shape profile is now updated in MeasureDiameterCue with corrected values

    // Adaptive learning: detect significant changes and enable fast learning
    if (m_config.enableAdaptiveLearning) {
      if (DetectSignificantChange(eye)) {
        EnableFastLearningMode();
      }

      // Update fast learning state
      if (m_fastLearningState.isEnabled) {
        m_fastLearningState.framesSinceTrigger++;
        if (m_fastLearningState.framesSinceTrigger >= m_fastLearningState.fastLearningDuration) {
          DisableFastLearningMode();
        }
      }
    }

    // Determine learning rate based on gaze angle, stability, and fast learning mode
    float gazeAngle = CalculateGazeAngle(eye.gazeDir);
    float angleConfidence = std::cos(gazeAngle);  // Higher confidence at neutral gaze

    // Apply learning rate multiplier if in fast learning mode
    float learningMultiplier = m_fastLearningState.isEnabled ?
                              (m_fastLearningState.fastLearningRate / m_fastLearningState.originalLearningRate) : 1.0f;

    if (eye.isBlink) {
      // Fast learning for closed references when blinking
      float blinkLearningRate = 0.8f * learningMultiplier;
      blinkLearningRate = std::min(blinkLearningRate, 0.95f);  // Cap at 95%
      refs.closedDia.Update(eye.pupilDiaMm, blinkLearningRate);
      refs.closedPosY.Update(eye.pupilPosY, blinkLearningRate);
    } else if (IsNeutralGaze(eye.gazeDir)) {
      // Slower learning for open references, only at neutral gaze
      // Only update if we have a reasonable difference from closed reference
      if (std::abs(eye.pupilDiaMm - refs.closedDia.value) > 0.5f) {
        float openLearningRate = 0.3f * angleConfidence * learningMultiplier;
        openLearningRate = std::min(openLearningRate, 0.8f);  // Cap at 80%
        refs.openDia.Update(eye.pupilDiaMm, openLearningRate);
      }
      if (std::abs(eye.pupilPosY - refs.closedPosY.value) > 0.1f) {
        float openLearningRate = 0.3f * angleConfidence * learningMultiplier;
        openLearningRate = std::min(openLearningRate, 0.8f);  // Cap at 80%
        refs.openPosY.Update(eye.pupilPosY, openLearningRate);
      }
    }

    // Update gaze compensation LUT
    // Learn how gaze angle affects pupil measurements
    if (!eye.isBlink) {
      // Calculate expected vs observed pupil size to learn compensation
      float expectedDia = refs.openDia.value;
      float observedDia = eye.pupilDiaMm;
      if (expectedDia > 0.1f) {
        float compensation = expectedDia / observedDia;
        compensation = std::clamp(compensation, 0.5f, 2.0f);  // Reasonable bounds
        UpdateGazeLUT(eye.gazeDir, compensation, refs);
      }
    }
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

  void ModernEyelidEstimator::ResetDilationNormalizer() {
    m_dilationNormalizer = PupilDilationNormalizer();
  }

  void ModernEyelidEstimator::ResetBlinkTweener() {
    m_blinkTweener = BlinkTweener();
  }

  void ModernEyelidEstimator::ResetEyeGeometryCalibrator() {
    m_eyeGeometryCalibrator = EyeGeometryCalibrator();
  }

  void ModernEyelidEstimator::EnableFastLearningMode() {
    m_fastLearningState.isEnabled = true;
    m_fastLearningState.framesSinceTrigger = 0;
  }

  void ModernEyelidEstimator::DisableFastLearningMode() {
    m_fastLearningState.isEnabled = false;
  }

  bool ModernEyelidEstimator::DetectSignificantChange(const EyeData& eye) {
    if (!eye.isValid || eye.isBlink) return false;

    // Initialize on first call
    if (m_fastLearningState.lastPupilDia == 0.0f) {
      m_fastLearningState.lastPupilDia = eye.pupilDiaMm;
      m_fastLearningState.lastPupilPosY = eye.pupilPosY;
      return false;
    }

    // Calculate change magnitude
    float diaDelta = std::abs(eye.pupilDiaMm - m_fastLearningState.lastPupilDia);
    float posDelta = std::abs(eye.pupilPosY - m_fastLearningState.lastPupilPosY);

    // Normalize changes
    float diaDeltaNorm = diaDelta / std::max(m_fastLearningState.lastPupilDia, 0.1f);
    float posDeltaNorm = posDelta;  // Position is already normalized

    // Detect significant change
    bool significantChange = (diaDeltaNorm > m_fastLearningState.changeThreshold) ||
                            (posDeltaNorm > m_fastLearningState.changeThreshold);

    // Update last values
    m_fastLearningState.lastPupilDia = eye.pupilDiaMm;
    m_fastLearningState.lastPupilPosY = eye.pupilPosY;

    return significantChange;
  }

  // EyeGeometryCalibrator implementation
  void ModernEyelidEstimator::EyeGeometryCalibrator::UpdateCalibration(const EyeData& eye) {
    if (!eye.isValid) return;
    
    // Update eye geometry
    UpdateEyeGeometry(eye, eye.gazeDir);
    
    // Update gaze-dependent behavior
    float currentOpenness = EstimateCurrentOpenness(eye);
    UpdateGazeBehavior(eye, currentOpenness);
  }

  void ModernEyelidEstimator::EyeGeometryCalibrator::UpdateEyeGeometry(const EyeData& eye, const Vector3& gazeDir) {
    // Learn individual eye geometry parameters
    // FIXED: Changed from static to member variable
    m_eyeGeometry.sampleCount++;

    // Estimate pupil center offset from gaze direction
    // When looking straight ahead, pupil should be centered
    if (gazeDir.z > 0.95f) { // Near neutral gaze
      // Learn pupil center offset
      Vector3 currentOffset = EstimatePupilOffset(eye);
      m_eyeGeometry.pupilCenterOffset = Vector3(
        m_eyeGeometry.pupilCenterOffset.x * 0.95f + currentOffset.x * 0.05f,
        m_eyeGeometry.pupilCenterOffset.y * 0.95f + currentOffset.y * 0.05f,
        m_eyeGeometry.pupilCenterOffset.z * 0.95f + currentOffset.z * 0.05f
      );
    }

    // Learn eye radius from pupil diameter and position
    if (eye.pupilDiaMm > 0) {
      float estimatedRadius = EstimateEyeRadius(eye);
      m_eyeGeometry.eyeRadiusMm = m_eyeGeometry.eyeRadiusMm * 0.98f +
                                 estimatedRadius * 0.02f;
    }

    // Mark as calibrated after sufficient samples
    if (m_eyeGeometry.sampleCount > 100) {
      m_eyeGeometry.isCalibrated = true;
    }
  }

  void ModernEyelidEstimator::EyeGeometryCalibrator::UpdateGazeBehavior(const EyeData& eye, float openness) {
    if (!eye.isValid) return;
    
    // Calculate gaze angle
    float gazeAngle = CalculateGazeAngle(eye.gazeDir);
    
    // Learn squint patterns for different gaze directions
    float observedSquint = CalculateObservedSquint(eye, openness);
    m_gazeBehavior.UpdateSquintFactor(gazeAngle, observedSquint);
  }

  float ModernEyelidEstimator::EyeGeometryCalibrator::GetCompensatedOpenness(float rawOpenness, const EyeData& eye) {
    float compensatedOpenness = rawOpenness;
    
    // Apply pupil occlusion compensation
    if (m_occlusionDetector.DetectOcclusion(eye, m_eyeGeometry)) {
      float gazeAngle = CalculateGazeAngle(eye.gazeDir);
      compensatedOpenness = m_occlusionDetector.CompensateOpenness(compensatedOpenness, gazeAngle);
    }
    
    // Apply gaze-dependent behavior compensation
    bool hasLearnedFactors = false;
    for (int i = 0; i < 10; i++) {
      if (m_gazeBehavior.learnedSquintFactors[i] > 0.0f) {
        hasLearnedFactors = true;
        break;
      }
    }
    if (hasLearnedFactors) {
      float gazeAngle = CalculateGazeAngle(eye.gazeDir);
      float squintFactor = m_gazeBehavior.GetSquintFactor(gazeAngle);
      compensatedOpenness = ApplySquintCompensation(compensatedOpenness, squintFactor);
    }
    
    return compensatedOpenness;
  }

  // GazeDependentBehavior implementation
  void ModernEyelidEstimator::EyeGeometryCalibrator::GazeDependentBehavior::UpdateSquintFactor(float gazeAngle, float observedSquint) {
    int gazeBin = static_cast<int>(gazeAngle * 10.0f);
    
    // Bounds check for array
    if (gazeBin >= 0 && gazeBin < 10) {
      // Initialize if not set (sampleCount == 0 indicates uninitialized)
      if (learnedSquintFactors[gazeBin] == 0.0f) {
        learnedSquintFactors[gazeBin] = 1.0f; // Start with neutral factor
      }
      
      // Update learned squint factor
      float currentFactor = learnedSquintFactors[gazeBin];
      float newFactor = currentFactor * (1.0f - learningRate) + observedSquint * learningRate;
      learnedSquintFactors[gazeBin] = (newFactor < 0.1f) ? 0.1f : (newFactor > 2.0f) ? 2.0f : newFactor;
    }
  }

  float ModernEyelidEstimator::EyeGeometryCalibrator::GazeDependentBehavior::GetSquintFactor(float gazeAngle) const {
    int gazeBin = static_cast<int>(gazeAngle * 10.0f);
    
    // Return learned factor if available
    if (gazeBin >= 0 && gazeBin < 10 && learnedSquintFactors[gazeBin] > 0.0f) {
      return learnedSquintFactors[gazeBin];
    }
    
    // Fallback to directional factors
    if (gazeAngle > 0.3f) { // Looking up
      return upGazeSquintFactor;
    } else if (gazeAngle < -0.3f) { // Looking down
      return downGazeSquintFactor;
    } else if (std::abs(gazeAngle) < 0.1f) { // Neutral
      return neutralGazeSquintFactor;
    } else { // Lateral
      return lateralGazeSquintFactor;
    }
  }

  // PupilOcclusionDetector implementation
  bool ModernEyelidEstimator::EyeGeometryCalibrator::PupilOcclusionDetector::DetectOcclusion(const EyeData& eye, const EyeGeometry& geometry) {
    if (!geometry.isCalibrated) return false;
    
    // Calculate expected pupil position based on gaze
    // Since this is a nested struct, we need to access the parent's static function
    Vector3 expectedPupilPos = EyeGeometryCalibrator::CalculateExpectedPupilPosition(eye.gazeDir, geometry);
    Vector3 actualPupilPos = Vector3(0, eye.pupilPosY, 0); // Simplified 2D position
    
    // Calculate displacement
    Vector3 displacementVec = Vector3(
      expectedPupilPos.x - actualPupilPos.x,
      expectedPupilPos.y - actualPupilPos.y,
      expectedPupilPos.z - actualPupilPos.z
    );
    float displacement = displacementVec.Magnitude();
    
    // Detect occlusion if displacement exceeds threshold
    isOccluded = displacement > occlusionThreshold;
    occlusionConfidence = std::min(displacement / occlusionThreshold, 1.0f);
    
    return isOccluded;
  }

  float ModernEyelidEstimator::EyeGeometryCalibrator::PupilOcclusionDetector::CompensateOpenness(float rawOpenness, float gazeAngle) const {
    if (!isOccluded) return rawOpenness;
    
    // Compensate for false closure due to pupil occlusion
    float compensationFactor = 1.0f + (occlusionConfidence * compensationStrength);
    float compensatedOpenness = rawOpenness * compensationFactor;
    
    return std::clamp(compensatedOpenness, 0.0f, 1.0f);
  }

  // Helper functions for eye geometry calibration
  Vector3 ModernEyelidEstimator::EyeGeometryCalibrator::EstimatePupilOffset(const EyeData& eye) {
    // Estimate pupil center offset from sensor position
    // This is a simplified estimation - in reality would need more sophisticated analysis
    return Vector3(0, eye.pupilPosY - 0.5f, 0); // Assume center is at 0.5
  }

  float ModernEyelidEstimator::EyeGeometryCalibrator::EstimateEyeRadius(const EyeData& eye) {
    // Estimate eye radius from pupil diameter and position
    // Typical eye radius is 12mm, pupil diameter varies 2-8mm
    float estimatedRadius = eye.pupilDiaMm * 3.0f; // Rough estimation
    return std::clamp(estimatedRadius, 8.0f, 16.0f);
  }

  float ModernEyelidEstimator::EyeGeometryCalibrator::CalculateObservedSquint(const EyeData& eye, float openness) {
    // Calculate observed squint based on pupil dynamics
    // When squinting, pupil appears smaller and position changes
    float squintIndicator = 1.0f - (eye.pupilDiaMm / 4.0f); // Normalize to 0-1
    return std::clamp(squintIndicator, 0.0f, 1.0f);
  }

  Vector3 ModernEyelidEstimator::EyeGeometryCalibrator::CalculateExpectedPupilPosition(const Vector3& gazeDir, const EyeGeometry& geometry) {
    // Calculate where pupil should be based on gaze direction and eye geometry
    // This is a simplified model - real implementation would be more complex
    Vector3 scaledGaze = Vector3(
      gazeDir.x * geometry.eyeRadiusMm,
      gazeDir.y * geometry.eyeRadiusMm,
      gazeDir.z * geometry.eyeRadiusMm
    );
    Vector3 expectedPos = Vector3(
      geometry.eyeCenter.x + scaledGaze.x,
      geometry.eyeCenter.y + scaledGaze.y,
      geometry.eyeCenter.z + scaledGaze.z
    );
    return expectedPos;
  }

  float ModernEyelidEstimator::EyeGeometryCalibrator::ApplySquintCompensation(float openness, float squintFactor) {
    // Apply squint compensation to openness
    // Higher squint factor means more squinting, so openness should be reduced
    float compensatedOpenness = openness / squintFactor;
    return std::clamp(compensatedOpenness, 0.0f, 1.0f);
  }

  float ModernEyelidEstimator::EyeGeometryCalibrator::EstimateCurrentOpenness(const EyeData& eye) {
    // Estimate current openness for learning purposes
    // This is a simplified estimation - would use actual measurement in real implementation
    float diameterOpenness = std::clamp(eye.pupilDiaMm / 4.0f, 0.0f, 1.0f);
    float positionOpenness = std::clamp(eye.pupilPosY, 0.0f, 1.0f);
    return (diameterOpenness + positionOpenness) * 0.5f;
  }

  float ModernEyelidEstimator::EyeGeometryCalibrator::CalculateGazeAngle(const Vector3& gazeDir) {
    // Calculate vertical gaze angle
    return std::asin(std::clamp(std::abs(gazeDir.y), 0.0f, 1.0f));
  }

  // BlinkTweener implementation
  float ModernEyelidEstimator::BlinkTweener::UpdateBlinkState(float currentOpenness, float deltaTime) {
    // Calculate velocity for sudden closure detection
    if (deltaTime > 0.0f) {
      float currentVelocity = (currentOpenness - lastOpenness) / deltaTime;
      opennessVelocity = opennessVelocity * velocitySmoothing + currentVelocity * (1.0f - velocitySmoothing);
    }
    
    // Detect blink start - either by threshold OR by sudden velocity
    bool thresholdBlink = !isBlinking && !wasBlinking && currentOpenness < blinkDetectionThreshold;
    bool velocityBlink = !isBlinking && !wasBlinking && opennessVelocity < velocityThreshold;
    
    if (thresholdBlink || velocityBlink) {
      isBlinking = true;
      blinkStartTime = 0.0f;
      blinkDuration = 0.0f;
      preBlinkOpenness = lastOpenness; // Use previous value for more stable pre-blink state
      blinkTarget = 0.0f; // Close completely
    }
    
    // Update last openness for next frame
    lastOpenness = currentOpenness;
    
    // Update blink timing
    if (isBlinking) {
      blinkStartTime += deltaTime;
      blinkDuration += deltaTime;
      
      // Check for blink end conditions
      if (blinkDuration > minBlinkDuration && currentOpenness > blinkEndThreshold) {
        isBlinking = false;
        wasBlinking = true;
        blinkTarget = preBlinkOpenness; // Return to pre-blink state
        blinkStartTime = 0.0f;
      }
      // Force end if blink is too long
      else if (blinkDuration > maxBlinkDuration) {
        isBlinking = false;
        wasBlinking = true;
        blinkTarget = preBlinkOpenness;
        blinkStartTime = 0.0f;
      }
    }
    
    // Handle post-blink recovery
    if (wasBlinking) {
      blinkStartTime += deltaTime;
      
      // Check if we've recovered enough
      if (currentOpenness > preBlinkOpenness * 0.9f) {
        wasBlinking = false;
        blinkStartTime = 0.0f;
      }
    }
    
    return GetBlinkInfluencedOpenness(currentOpenness, deltaTime);
  }

  float ModernEyelidEstimator::BlinkTweener::GetBlinkInfluencedOpenness(float normalOpenness, float deltaTime) {
    if (!isBlinking && !wasBlinking) {
      return normalOpenness;
    }

    float targetOpenness = normalOpenness;

    if (isBlinking) {
      // During blink: smoothly close to target
      float closeProgress = std::min(blinkStartTime * blinkCloseSpeed, 1.0f);
      targetOpenness = preBlinkOpenness * (1.0f - closeProgress) + blinkTarget * closeProgress;

      // Add slight overshoot for natural feel
      if (closeProgress > 0.8f) {
        float overshootAmount = blinkOvershoot * (1.0f - closeProgress) / 0.2f;
        targetOpenness = std::max(targetOpenness - overshootAmount, blinkTarget);
      }
    }
    else if (wasBlinking) {
      // Post-blink: smoothly return to normal
      float recoveryProgress = std::min(blinkStartTime * blinkOpenSpeed, 1.0f);
      targetOpenness = blinkTarget * (1.0f - recoveryProgress) + normalOpenness * recoveryProgress;

      // Add slight overshoot when opening
      if (recoveryProgress < 0.3f) {
        float overshootAmount = blinkOvershoot * (0.3f - recoveryProgress) / 0.3f;
        targetOpenness = std::min(targetOpenness + overshootAmount, 1.0f);
      }
    }

    return targetOpenness;
  }

  // Gaze LUT helper functions implementation
  void ModernEyelidEstimator::GetLUTIndices(const Vector3& gazeDir, int& vertIdx, int& horizIdx) const {
    // Convert gaze direction to LUT indices
    // Vertical: based on Y component (up/down gaze)
    // Horizontal: based on X component (left/right gaze)

    constexpr int V_SIZE = GazeAwareReferences::GAZE_LUT_SIZE_V;
    constexpr int H_SIZE = GazeAwareReferences::GAZE_LUT_SIZE_H;

    // Map Y component (-1 to 1) to vertical index (0 to V_SIZE-1)
    float normalizedY = (gazeDir.y + 1.0f) * 0.5f;  // Map to 0-1
    normalizedY = std::clamp(normalizedY, 0.0f, 1.0f);
    vertIdx = static_cast<int>(normalizedY * (V_SIZE - 1));
    vertIdx = std::clamp(vertIdx, 0, V_SIZE - 1);

    // Map X component (-1 to 1) to horizontal index (0 to H_SIZE-1)
    float normalizedX = (gazeDir.x + 1.0f) * 0.5f;  // Map to 0-1
    normalizedX = std::clamp(normalizedX, 0.0f, 1.0f);
    horizIdx = static_cast<int>(normalizedX * (H_SIZE - 1));
    horizIdx = std::clamp(horizIdx, 0, H_SIZE - 1);
  }

  void ModernEyelidEstimator::UpdateGazeLUT(const Vector3& gazeDir, float observedCompensation, GazeAwareReferences& refs) {
    int vertIdx, horizIdx;
    GetLUTIndices(gazeDir, vertIdx, horizIdx);

    // Update LUT with exponential moving average
    float& currentValue = refs.gazeCompensationLUT[vertIdx][horizIdx];
    int& sampleCount = refs.gazeLUTSampleCount[vertIdx][horizIdx];

    if (sampleCount == 0) {
      // First sample for this cell
      currentValue = observedCompensation;
      sampleCount = 1;
    } else {
      // Exponential moving average with adaptive learning rate
      float learningRate = std::min(0.1f, 1.0f / sampleCount);
      currentValue = currentValue * (1.0f - learningRate) + observedCompensation * learningRate;
      sampleCount++;
    }
  }

  float ModernEyelidEstimator::GetGazeCompensation(const Vector3& gazeDir, const GazeAwareReferences& refs) const {
    int vertIdx, horizIdx;
    GetLUTIndices(gazeDir, vertIdx, horizIdx);

    // Get the base compensation value
    float baseCompensation = refs.gazeCompensationLUT[vertIdx][horizIdx];
    int sampleCount = refs.gazeLUTSampleCount[vertIdx][horizIdx];

    // If we don't have enough samples, use bilinear interpolation from neighboring cells
    if (sampleCount < 5) {
      // Collect neighboring values for interpolation
      float sum = 0.0f;
      int count = 0;

      for (int dv = -1; dv <= 1; dv++) {
        for (int dh = -1; dh <= 1; dh++) {
          int nv = vertIdx + dv;
          int nh = horizIdx + dh;

          if (nv >= 0 && nv < GazeAwareReferences::GAZE_LUT_SIZE_V &&
              nh >= 0 && nh < GazeAwareReferences::GAZE_LUT_SIZE_H &&
              refs.gazeLUTSampleCount[nv][nh] > 0) {
            sum += refs.gazeCompensationLUT[nv][nh];
            count++;
          }
        }
      }

      if (count > 0) {
        float neighborAverage = sum / count;
        // Blend base compensation with neighbor average based on sample count
        float blendFactor = sampleCount / 5.0f;
        baseCompensation = baseCompensation * blendFactor + neighborAverage * (1.0f - blendFactor);
      }
    }

    return baseCompensation;
  }

}
