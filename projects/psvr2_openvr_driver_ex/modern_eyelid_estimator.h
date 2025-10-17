#pragma once

#include "hmd2_gaze.h"
#include <map>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

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

  struct CueMeasurement {
    float value;        // Normalized measurement (0-1)
    float uncertainty;  // How unreliable is this measurement (0-1)
    float confidence;   // How confident are we (0-1)
    std::string name;   // Cue identifier for debugging
    
    CueMeasurement(float v = 0.5f, float u = 0.5f, float c = 0.5f, const std::string& n = "")
      : value(v), uncertainty(u), confidence(c), name(n) {}
  };

  struct EyeData {
    float pupilDiaMm;
    float pupilPosY;
    Vector3 gazeDir;
    bool isBlink;
    bool isValid;
    
    EyeData() : pupilDiaMm(0), pupilPosY(0), gazeDir(0,0,1), isBlink(false), isValid(false) {}
  };

  struct EstimationResult {
    float openness;           // 0.0 = closed, 1.0 = open
    float confidence;         // Overall confidence (0-1)
    std::string primaryCue;   // Which cue dominated
    
    EstimationResult(float o = 0.5f, float c = 0.5f, const std::string& p = "unknown")
      : openness(o), confidence(c), primaryCue(p) {}
  };

  class ModernEyelidEstimator {
  public:
    ModernEyelidEstimator();
    
    // Main estimation function - takes both eyes for unified processing
    EstimationResult Estimate(const EyeData& leftEye, const EyeData& rightEye);
    
    // Convenience function for single eye (for A/B testing)
    EstimationResult Estimate(const EyeData& eye);
    
    // Convert from Hmd2GazeEye to EyeData (public for A/B testing)
    EyeData ConvertFromHmd2Gaze(const Hmd2GazeEye& eye) const;
    
    // Reset dilation normalizer (useful for new sessions)
    void ResetDilationNormalizer();
    
    // Reset blink tweener (useful for new sessions)
    void ResetBlinkTweener();
    
    // Reset eye geometry calibrator (useful for new sessions)
    void ResetEyeGeometryCalibrator();

  private:
    // Adaptive learning with exponential moving averages
    struct AdaptiveReference {
      float value;
      float learningRate;
      float stability;
      int sampleCount;
      
      AdaptiveReference(float initialValue = 0.5f, float lr = 0.01f) 
        : value(initialValue), learningRate(lr), stability(0.5f), sampleCount(0) {}
      
      void Update(float newValue, float confidence) {
        sampleCount++;
        float effectiveLR = learningRate * confidence;
        value = value * (1.0f - effectiveLR) + newValue * effectiveLR;
        
        // Update stability based on variance
        float variance = std::abs(newValue - value);
        stability = stability * 0.95f + (1.0f - variance) * 0.05f;
      }
    };
    
    // Gaze-aware references for each eye
    struct GazeAwareReferences {
      AdaptiveReference openDia;
      AdaptiveReference closedDia;
      AdaptiveReference openPosY;
      AdaptiveReference closedPosY;
      
      // Learn different references for different gaze angles
      std::map<int, AdaptiveReference> angleSpecificRefs;
      
      GazeAwareReferences() 
        : openDia(4.0f, 0.005f), closedDia(2.0f, 0.01f)
        , openPosY(0.55f, 0.005f), closedPosY(0.45f, 0.01f) {}
    } m_leftRefs, m_rightRefs;
    
    // Pupil dilation normalization system
    struct PupilDilationNormalizer {
      float baselineDilation;      // Current baseline pupil size
      float dilationRange;         // Typical range of dilation variation
      float adaptationRate;        // How fast to adapt to new baselines
      int sampleCount;             // Number of samples used for baseline
      
      PupilDilationNormalizer() 
        : baselineDilation(3.5f)   // Default 3.5mm baseline
        , dilationRange(1.0f)      // 1mm typical range
        , adaptationRate(0.001f)   // Very slow adaptation
        , sampleCount(0) {}
      
      // Normalize pupil diameter to account for dilation variations
      float NormalizeDiameter(float rawDiameter) const {
        if (dilationRange < 0.1f) return rawDiameter; // No normalization if range is too small
        
        // Normalize to 0-1 range based on current baseline
        float normalized = (rawDiameter - (baselineDilation - dilationRange)) / (2.0f * dilationRange);
        return std::clamp(normalized, 0.0f, 1.0f);
      }
      
      // Update baseline based on recent measurements
      void UpdateBaseline(float newDiameter) {
        sampleCount++;
        
        // Very slow adaptation to baseline changes
        baselineDilation = baselineDilation * (1.0f - adaptationRate) + 
                          newDiameter * adaptationRate;
        
        // Update dilation range based on variance
        float variance = std::abs(newDiameter - baselineDilation);
        dilationRange = dilationRange * (1.0f - adaptationRate) + 
                       variance * adaptationRate;
        
        // Ensure reasonable bounds
        baselineDilation = std::clamp(baselineDilation, 2.0f, 6.0f);
        dilationRange = std::clamp(dilationRange, 0.5f, 2.0f);
      }
    } m_dilationNormalizer;
    
    // Learned neutral gaze (shared between eyes)
    Vector3 m_learnedNeutralGaze;
    float m_neutralGazeConfidence;
    int m_gazeSampleCount;
    
    // Adaptive eye geometry calibration system
    struct EyeGeometryCalibrator {
      // Individual eye geometry parameters
      struct EyeGeometry {
        Vector3 pupilCenterOffset;        // Offset of pupil center from eye center
        float eyeRadiusMm;                 // Estimated eye radius in mm
        float eyelidCurvature;             // Eyelid curvature factor
        float eyelidThickness;             // Eyelid thickness factor
        Vector3 eyeCenter;                 // Estimated eye center position
        bool isCalibrated;                 // Whether geometry is learned
        
        EyeGeometry() : pupilCenterOffset(0,0,0), eyeRadiusMm(12.0f), 
                       eyelidCurvature(1.0f), eyelidThickness(1.0f),
                       eyeCenter(0,0,0), isCalibrated(false) {}
      } m_eyeGeometry;
      
      // Gaze-dependent eyelid behavior modeling
      struct GazeDependentBehavior {
        // Different squinting patterns for different gaze directions
        float upGazeSquintFactor = 0.8f;      // Eyes squint more when looking up
        float downGazeSquintFactor = 0.6f;     // Eyes squint less when looking down
        float lateralGazeSquintFactor = 0.7f;  // Moderate squinting for lateral gaze
        float neutralGazeSquintFactor = 1.0f;  // Baseline squinting
        
        // Learning parameters
        float learningRate = 0.01f;            // How fast to learn gaze patterns
        int minSamplesPerGaze = 50;            // Minimum samples before trusting pattern
        std::map<int, float> learnedSquintFactors; // Learned squint factors per gaze bin
        
        // Update squint factor for specific gaze direction
        void UpdateSquintFactor(float gazeAngle, float observedSquint);
        
        // Get squint factor for current gaze
        float GetSquintFactor(float gazeAngle) const;
      } m_gazeBehavior;
      
      // Pupil occlusion detection and compensation
      struct PupilOcclusionDetector {
        float occlusionThreshold = 0.3f;       // Threshold for detecting occlusion
        float compensationStrength = 0.8f;     // How much to compensate for occlusion
        bool isOccluded = false;               // Current occlusion state
        float occlusionConfidence = 0.0f;      // Confidence in occlusion detection
        
        // Detect if pupil appears occluded due to gaze angle
        bool DetectOcclusion(const EyeData& eye, const EyeGeometry& geometry);
        
        // Compensate openness for detected occlusion
        float CompensateOpenness(float rawOpenness, float gazeAngle) const;
      } m_occlusionDetector;
      
      // Calibration methods
      void UpdateEyeGeometry(const EyeData& eye, const Vector3& gazeDir);
      void UpdateGazeBehavior(const EyeData& eye, float openness);
      bool IsGeometryCalibrated() const { return m_eyeGeometry.isCalibrated; }
      
      // Main calibration update
      void UpdateCalibration(const EyeData& eye);
      
      // Get compensated openness
      float GetCompensatedOpenness(float rawOpenness, const EyeData& eye);
      
      // Helper functions
      Vector3 EstimatePupilOffset(const EyeData& eye);
      float EstimateEyeRadius(const EyeData& eye);
      float CalculateObservedSquint(const EyeData& eye, float openness);
      Vector3 CalculateExpectedPupilPosition(const Vector3& gazeDir, const EyeGeometry& geometry);
      float ApplySquintCompensation(float openness, float squintFactor);
      float EstimateCurrentOpenness(const EyeData& eye);
      float CalculateGazeAngle(const Vector3& gazeDir);
    } m_eyeGeometryCalibrator;

    // Blink detection and tweening system
    struct BlinkTweener {
      bool isBlinking = false;
      bool wasBlinking = false;
      float blinkStartTime = 0.0f;
      float blinkDuration = 0.0f;
      float preBlinkOpenness = 0.0f;
      float blinkTarget = 0.0f;
      
      // Blink detection parameters
      float blinkDetectionThreshold = 0.3f;   // Openness threshold to detect blink start
      float blinkEndThreshold = 0.7f;         // Openness threshold to detect blink end
      float minBlinkDuration = 0.05f;          // Minimum blink duration (50ms)
      float maxBlinkDuration = 0.5f;           // Maximum blink duration (500ms)
      
      // Tweening parameters
      float blinkCloseSpeed = 8.0f;            // Speed of closing during blink
      float blinkOpenSpeed = 4.0f;             // Speed of opening after blink
      float blinkOvershoot = 0.1f;             // Slight overshoot for natural feel
      
      // Update blink state and return tweened openness
      float UpdateBlinkState(float currentOpenness, float deltaTime);
      
      // Check if we should override normal estimation
      bool ShouldOverrideEstimation() const { return isBlinking || wasBlinking; }
      
      // Get current blink-influenced openness
      float GetBlinkInfluencedOpenness(float normalOpenness, float deltaTime);
    } m_blinkTweener;
    
    // Configuration
    struct Config {
      float minLearningRate = 0.001f;
      float maxLearningRate = 0.05f;
      float neutralGazeThreshold = 0.95f;
      int gazeAngleBins = 10;  // Number of angle-specific reference bins
      float smoothingAlpha = 0.1f;
      float minConfidence = 0.1f;
      bool invertOutput = true;  // Set to true if output is inverted
      
      // Blink augmentation parameters
      bool enableBlinkAugmentation = true;     // Whether to use blink data for augmentation
      float blinkOverrideStrength = 0.8f;     // How much blink data overrides estimation (0-1)
      
      // Eye geometry calibration parameters
      bool enableEyeGeometryCalibration = true; // Whether to use adaptive eye geometry calibration
      bool enableGazeDependentBehavior = true;   // Whether to model gaze-dependent eyelid behavior
      bool enablePupilOcclusionCompensation = true; // Whether to compensate for pupil occlusion
      float geometryCalibrationStrength = 0.7f; // How much to trust geometry calibration (0-1)
    } m_config;
    
    // Cue measurement functions
    CueMeasurement MeasureDiameterCue(const EyeData& eye, const GazeAwareReferences& refs);
    CueMeasurement MeasurePositionCue(const EyeData& eye, const GazeAwareReferences& refs);
    CueMeasurement MeasureBlinkCue(const EyeData& eye);
    
    // Gaze angle utilities
    float CalculateGazeAngle(const Vector3& gazeDir) const;
    bool IsNeutralGaze(const Vector3& gazeDir) const;
    
    // Learning functions
    void UpdateReferences(const EyeData& eye, GazeAwareReferences& refs);
    void UpdateNeutralGaze(const EyeData& leftEye, const EyeData& rightEye);
    
    // Fusion and utility functions
    float FuseCues(const std::vector<CueMeasurement>& cues);
    float CalculateOverallConfidence(const std::vector<CueMeasurement>& cues);
    std::string DeterminePrimaryCue(const std::vector<CueMeasurement>& cues);
    
  };

}
