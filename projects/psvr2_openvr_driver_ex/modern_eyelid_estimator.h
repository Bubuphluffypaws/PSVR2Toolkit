#pragma once

#include "hmd2_gaze.h"
#include <map>
#include <vector>
#include <string>

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
    
    // Learned neutral gaze (shared between eyes)
    Vector3 m_learnedNeutralGaze;
    float m_neutralGazeConfidence;
    int m_gazeSampleCount;
    
    // Configuration
    struct Config {
      float minLearningRate = 0.001f;
      float maxLearningRate = 0.05f;
      float neutralGazeThreshold = 0.95f;
      int gazeAngleBins = 10;  // Number of angle-specific reference bins
      float smoothingAlpha = 0.1f;
      float minConfidence = 0.1f;
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
    
    // Convert from Hmd2GazeEye to EyeData
    EyeData ConvertFromHmd2Gaze(const Hmd2GazeEye& eye) const;
  };

}
