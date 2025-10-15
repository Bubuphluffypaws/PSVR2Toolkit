#pragma once

#include "hmd2_gaze.h"
#include <algorithm>

namespace psvr2_toolkit {

  // Fallback clamp function for older C++ standards
  template<typename T>
  constexpr const T& clamp(const T& v, const T& lo, const T& hi) {
    return (v < lo) ? lo : (hi < v) ? hi : v;
  }

  class OriginalEyelidEstimator {
  public:
    OriginalEyelidEstimator();
    
    // Main estimation function
    float Estimate(const Hmd2GazeEye& eye);
    
    // Reset calibration
    void Reset();

  private:
    // Original parameters (from master)
    static constexpr float openLR = 0.001f;      // Original slow open adaptation
    static constexpr float closedLR = 0.01f;     // Original closed adaptation
    static constexpr float smoothAlpha = 0.2f;   // Original smoothing
    
    // Original reference points
    float m_openDia;
    float m_closedDia;
    float m_sensorYOpen;
    float m_sensorYClosed;
    float m_lastOpenness;
    
    // Pupil dilation normalization (added enhancement)
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
        return clamp(normalized, 0.0f, 1.0f);
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
        baselineDilation = clamp(baselineDilation, 2.0f, 6.0f);
        dilationRange = clamp(dilationRange, 0.5f, 2.0f);
      }
    } m_dilationNormalizer;
    
    // Original neutral gaze detection
    bool IsNeutral(const Hmd2GazeEye& eye) const;
  };

}
