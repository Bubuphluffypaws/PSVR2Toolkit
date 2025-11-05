#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "hmd2_gaze.h"
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

namespace psvr2_toolkit {

  struct Vector3 {
    float x, y, z;
    Vector3(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
    
    float Magnitude() const {
      return sqrtf(x * x + y * y + z * z);
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

    // Adaptive learning mode control
    void EnableFastLearningMode();
    void DisableFastLearningMode();
    bool DetectSignificantChange(const EyeData& eye);

    // Debug helper to inspect learned references
    void GetReferencesForDebug(int eyeIndex, float& openDia, float& closedDia, float& openPosY, float& closedPosY) const;

    // Enhanced smoothing system with multiple options
    struct SmoothingSystem {
      enum SmoothingMethod {
        SIMPLE_LOWPASS,      // Basic low-pass filter
        STRONG_AVERAGING,    // Multi-sample averaging
        KALMAN_FILTER       // Kalman filter (future)
      };
      
      SmoothingMethod method = STRONG_AVERAGING;  // Default to strong averaging for smooth squinting
      
      // Simple low-pass filter
      struct LowPassFilter {
        float alpha = 0.05f;                     // Very aggressive smoothing
        float lastValue = 0.5f;
        bool initialized = false;
        
        float Filter(float input) {
          if (!initialized) {
            lastValue = input;
            initialized = true;
            return input;
          }
          lastValue = alpha * input + (1.0f - alpha) * lastValue;
          return lastValue;
        }
        
        void Reset() {
          initialized = false;
          lastValue = 0.5f;
        }
      } lowPass;
      
      // Strong averaging (moving average) with outlier rejection
      struct StrongAveraging {
        static constexpr int BUFFER_SIZE = 30;   // Average over 30 samples (~500ms at 60fps) for smooth output
        static constexpr float OUTLIER_THRESHOLD = 0.20f;  // Reject single-frame spikes >20% from recent average (blocks extreme noise, allows squinting)
        float buffer[BUFFER_SIZE];
        int currentIndex = 0;
        int sampleCount = 0;

        // FIXED: Added constructor to initialize buffer (was uninitialized!)
        StrongAveraging() : currentIndex(0), sampleCount(0) {
          for (int i = 0; i < BUFFER_SIZE; ++i) {
            buffer[i] = 0.5f;  // Initialize to neutral value
          }
        }

        float Filter(float input) {
          // Outlier rejection: compare against RECENT samples (last 5), not full buffer
          // This allows gradual trends while blocking single-frame noise spikes
          float filteredInput = input;
          if (sampleCount >= 5) {
            // Get average of last 5 samples for comparison
            float recentSum = 0.0f;
            int recentCount = std::min(5, sampleCount);
            for (int i = 0; i < recentCount; ++i) {
              int idx = (currentIndex - 1 - i + BUFFER_SIZE) % BUFFER_SIZE;
              recentSum += buffer[idx];
            }
            float recentAvg = recentSum / recentCount;

            float deviation = std::abs(input - recentAvg);
            if (deviation > OUTLIER_THRESHOLD) {
              // Reject outlier - use recent average instead
              filteredInput = recentAvg;
            }
          }

          buffer[currentIndex] = filteredInput;
          currentIndex = (currentIndex + 1) % BUFFER_SIZE;
          sampleCount = (sampleCount + 1 < BUFFER_SIZE) ? sampleCount + 1 : BUFFER_SIZE;

          float sum = 0.0f;
          for (int i = 0; i < sampleCount; ++i) {
            sum += buffer[i];
          }
          return sum / sampleCount;
        }

        float CalculateMedian() const {
          if (sampleCount == 0) return 0.5f;

          // Copy buffer for sorting (don't modify original)
          float sorted[BUFFER_SIZE];
          for (int i = 0; i < sampleCount; ++i) {
            sorted[i] = buffer[i];
          }

          // Simple bubble sort (fine for small arrays)
          for (int i = 0; i < sampleCount - 1; ++i) {
            for (int j = 0; j < sampleCount - i - 1; ++j) {
              if (sorted[j] > sorted[j + 1]) {
                float temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
              }
            }
          }

          // Return median
          if (sampleCount % 2 == 0) {
            return (sorted[sampleCount/2 - 1] + sorted[sampleCount/2]) / 2.0f;
          } else {
            return sorted[sampleCount/2];
          }
        }

        void Reset() {
          currentIndex = 0;
          sampleCount = 0;
          for (int i = 0; i < BUFFER_SIZE; ++i) {
            buffer[i] = 0.5f;
          }
        }
      } averaging;
      
      // Kalman filter (placeholder for future implementation)
      struct KalmanFilter {
        float state = 0.5f;
        float covariance = 1.0f;
        float processNoise = 0.01f;
        float measurementNoise = 0.1f;
        
        float Filter(float input) {
          // Simple placeholder - just return input for now
          return input;
        }
        
        void Reset() {
          state = 0.5f;
          covariance = 1.0f;
        }
      } kalman;
      
      float Filter(float input) {
        switch (method) {
          case SIMPLE_LOWPASS:
            return lowPass.Filter(input);
          case STRONG_AVERAGING:
            return averaging.Filter(input);
          case KALMAN_FILTER:
            return kalman.Filter(input);
          default:
            return averaging.Filter(input);
        }
      }
      
      void Reset() {
        lowPass.Reset();
        averaging.Reset();
        kalman.Reset();
      }
    };

    // Smoothing system configuration methods
    void SetSmoothingMethod(SmoothingSystem::SmoothingMethod method) {
      m_smoothingSystem.method = method;
    }
    
    void ResetSmoothing() {
      m_smoothingSystem.Reset();
    }
    
    SmoothingSystem::SmoothingMethod GetSmoothingMethod() const {
      return m_smoothingSystem.method;
    }

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
        float variance = (newValue - value < 0) ? -(newValue - value) : (newValue - value);
        stability = stability * 0.95f + (1.0f - variance) * 0.05f;
      }
    };
    
    // Eye shape profile for adaptive calibration
    struct EyeShapeProfile {
      float minPupilSize;            // Observed minimum pupil size
      float maxPupilSize;            // Observed maximum pupil size
      float minPupilPosY;            // Observed minimum Y position
      float maxPupilPosY;            // Observed maximum Y position
      float pupilSizeRange;          // Computed range for normalization
      float eyelidTravelDistance;    // Vertical travel range
      float asymmetryFactor;         // Left/right eye differences
      bool isProfileLearned;         // Whether we have enough data
      int sampleCount;               // Number of samples collected

      EyeShapeProfile()
        : minPupilSize(10.0f), maxPupilSize(0.0f)  // Initialize with inverted range for proper min/max tracking
        , minPupilPosY(10.0f), maxPupilPosY(0.0f)
        , pupilSizeRange(2.0f), eyelidTravelDistance(0.1f)
        , asymmetryFactor(1.0f), isProfileLearned(false), sampleCount(0) {}

      void UpdateProfile(float pupilDia, float pupilPosY, bool isBlink) {
        sampleCount++;

        // Update min/max ranges
        if (isBlink) {
          minPupilSize = std::min(minPupilSize, pupilDia);
          minPupilPosY = std::min(minPupilPosY, pupilPosY);
        } else {
          maxPupilSize = std::max(maxPupilSize, pupilDia);
          maxPupilPosY = std::max(maxPupilPosY, pupilPosY);
        }

        // Update computed values
        pupilSizeRange = std::max(0.5f, maxPupilSize - minPupilSize);
        eyelidTravelDistance = std::max(0.02f, maxPupilPosY - minPupilPosY);

        // Mark as learned after sufficient samples
        if (sampleCount > 50) {
          isProfileLearned = true;
        }
      }

      float NormalizePupilDia(float pupilDia) const {
        if (!isProfileLearned || pupilSizeRange < 0.1f) {
          // Fallback to default normalization
          return std::clamp((pupilDia - 2.0f) / 2.0f, 0.0f, 1.0f);
        }
        return std::clamp((pupilDia - minPupilSize) / pupilSizeRange, 0.0f, 1.0f);
      }

      float NormalizePupilPosY(float pupilPosY) const {
        if (!isProfileLearned || eyelidTravelDistance < 0.01f) {
          // Fallback to default normalization
          return std::clamp((pupilPosY - 0.45f) / 0.1f, 0.0f, 1.0f);
        }
        return std::clamp((pupilPosY - minPupilPosY) / eyelidTravelDistance, 0.0f, 1.0f);
      }
    };

    // Gaze-aware references for each eye
    struct GazeAwareReferences {
      AdaptiveReference openDia;
      AdaptiveReference closedDia;
      AdaptiveReference openPosY;
      AdaptiveReference closedPosY;

      // Continuous 2D gaze compensation lookup table (replaced 10-bin array)
      static constexpr int GAZE_LUT_SIZE_V = 20;  // Vertical resolution
      static constexpr int GAZE_LUT_SIZE_H = 20;  // Horizontal resolution
      float gazeCompensationLUT[GAZE_LUT_SIZE_V][GAZE_LUT_SIZE_H];
      int gazeLUTSampleCount[GAZE_LUT_SIZE_V][GAZE_LUT_SIZE_H];

      GazeAwareReferences()
        : openDia(3.5f, 0.005f), closedDia(1.5f, 0.01f)  // Realistic starting values for squint detection
        , openPosY(0.55f, 0.005f), closedPosY(0.45f, 0.01f) {  // Realistic position range
        // Initialize LUT with neutral values
        for (int v = 0; v < GAZE_LUT_SIZE_V; v++) {
          for (int h = 0; h < GAZE_LUT_SIZE_H; h++) {
            gazeCompensationLUT[v][h] = 1.0f;  // Neutral compensation
            gazeLUTSampleCount[v][h] = 0;
          }
        }
      }
    } m_leftRefs, m_rightRefs;

    // Eye shape profiles for adaptive per-user calibration
    EyeShapeProfile m_leftEyeProfile, m_rightEyeProfile;
    
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
        return (normalized < 0.0f) ? 0.0f : (normalized > 1.0f) ? 1.0f : normalized;
      }
      
      // Update baseline based on recent measurements
      void UpdateBaseline(float newDiameter) {
        sampleCount++;
        
        // Very slow adaptation to baseline changes
        baselineDilation = baselineDilation * (1.0f - adaptationRate) + 
                          newDiameter * adaptationRate;
        
        // Update dilation range based on variance
        float variance = (newDiameter - baselineDilation < 0) ? -(newDiameter - baselineDilation) : (newDiameter - baselineDilation);
        dilationRange = dilationRange * (1.0f - adaptationRate) + 
                       variance * adaptationRate;
        
        // Ensure reasonable bounds
        baselineDilation = (baselineDilation < 2.0f) ? 2.0f : (baselineDilation > 6.0f) ? 6.0f : baselineDilation;
        dilationRange = (dilationRange < 0.5f) ? 0.5f : (dilationRange > 2.0f) ? 2.0f : dilationRange;
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
        int sampleCount;                   // Number of calibration samples (FIXED: was static in cpp)

        EyeGeometry() : pupilCenterOffset(0,0,0), eyeRadiusMm(12.0f),
                       eyelidCurvature(1.0f), eyelidThickness(1.0f),
                       eyeCenter(0,0,0), isCalibrated(false), sampleCount(0) {}
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
        float learnedSquintFactors[10];         // Learned squint factors per gaze bin (simplified)
        
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
      static Vector3 CalculateExpectedPupilPosition(const Vector3& gazeDir, const EyeGeometry& geometry);
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
      
      // Velocity-based blink detection for sudden closures
      float lastOpenness = 0.5f;
      float opennessVelocity = 0.0f;
      float velocityThreshold = -2.0f;        // Negative velocity threshold for sudden closure
      float velocitySmoothing = 0.8f;         // Smoothing factor for velocity calculation
      
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

    SmoothingSystem m_smoothingSystem;  // Instance of the public SmoothingSystem struct

    // Smoothing state (FIXED: was static, now member variable)
    float m_lastOpenness = 0.5f;

    // Fast learning mode state
    struct FastLearningState {
      bool isEnabled = false;
      int framesSinceTrigger = 0;
      int fastLearningDuration = 100;  // Number of frames to use fast learning
      float originalLearningRate = 0.005f;
      float fastLearningRate = 0.05f;

      // Change detection
      float lastPupilDia = 0.0f;
      float lastPupilPosY = 0.0f;
      float changeThreshold = 0.3f;  // Threshold for detecting significant changes
    } m_fastLearningState;

    // Configuration
    struct Config {
      float minLearningRate = 0.001f;
      float maxLearningRate = 0.05f;
      float neutralGazeThreshold = 0.95f;
      int gazeAngleBins = 10;  // Number of angle-specific reference bins
      float smoothingAlpha = 0.2f;  // Increased for better responsiveness (was 0.1f)
      float minConfidence = 0.05f;  // Lowered to be less restrictive (was 0.1f)
      bool invertOutput = false;  // Set to true if output is inverted (false = normal, 1.0 = open, 0.0 = closed)

      // Blink augmentation parameters - DISABLED for instant blinks
      bool enableBlinkAugmentation = false;    // Disabled - blinks should be instant, not gradual
      float blinkOverrideStrength = 0.8f;     // How much blink data overrides estimation (0-1)

      // Eye geometry calibration parameters
      bool enableEyeGeometryCalibration = false; // Disabled to preserve squint detection dynamic range
      bool enableGazeDependentBehavior = true;   // Whether to model gaze-dependent eyelid behavior
      bool enablePupilOcclusionCompensation = true; // Whether to compensate for pupil occlusion
      float geometryCalibrationStrength = 0.7f; // How much to trust geometry calibration (0-1)

      // Adaptive learning parameters
      bool enableAdaptiveLearning = true;  // Enable automatic fast learning on detected changes

      // Extreme gaze angle handling - TUNED for day-to-day robustness
      bool disableEyelidEstimationAtExtremeGaze = true;  // Reduce confidence at extreme gaze angles
      float extremeGazeAngleThreshold = 0.70f;  // ~40 degrees vertical (radians) - truly extreme
      float reducedConfidenceGazeThreshold = 0.52f;  // ~30 degrees - start reducing confidence gradually
      float extremeGazeConfidenceMultiplier = 0.5f;  // Reduce confidence but don't fully disable (was 0.0)

      // Directional gaze thresholds (asymmetric - up/down more restrictive than lateral)
      float upGazeDisableThreshold = 0.70f;  // ~40 degrees up (allow normal looking up)
      float downGazeDisableThreshold = 0.70f;  // ~40 degrees down (allow normal looking down)
      float lateralGazeDisableThreshold = 0.87f;  // ~50 degrees sideways (permissive - "stink eye" is common!)

      // Geometric compensation coordination
      bool pupilDiameterPrecompensated = true;  // True if HeadsetCalibrator already applied ellipticity correction
    } m_config;
    
    // Private helper functions
    void UpdateReferences(const EyeData& eye, GazeAwareReferences& refs, EyeShapeProfile& profile);
    void UpdateNeutralGaze(const EyeData& leftEye, const EyeData& rightEye);
    float FuseCues(const std::vector<CueMeasurement>& cues);
    float CalculateOverallConfidence(const std::vector<CueMeasurement>& cues);
    std::string DeterminePrimaryCue(const std::vector<CueMeasurement>& cues);
    CueMeasurement MeasureDiameterCue(const EyeData& eye, const GazeAwareReferences& refs, EyeShapeProfile& profile);
    CueMeasurement MeasurePositionCue(const EyeData& eye, const GazeAwareReferences& refs, const EyeShapeProfile& profile);
    CueMeasurement MeasureBlinkCue(const EyeData& eye);
    float CalculateGazeAngle(const Vector3& gazeDir) const;
    bool IsNeutralGaze(const Vector3& gazeDir);
    float CalculateGazeConfidenceMultiplier(const Vector3& gazeDir) const;

    // Gaze LUT helper functions
    void UpdateGazeLUT(const Vector3& gazeDir, float observedCompensation, GazeAwareReferences& refs);
    float GetGazeCompensation(const Vector3& gazeDir, const GazeAwareReferences& refs) const;
    void GetLUTIndices(const Vector3& gazeDir, int& vertIdx, int& horizIdx) const;
    
  }; // class ModernEyelidEstimator

} // namespace psvr2_toolkit
