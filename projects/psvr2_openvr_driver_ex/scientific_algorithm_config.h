#pragma once

#include <string>

// Scientific Algorithm Configuration
// Enable/disable individual algorithms for testing and comparison

namespace psvr2_toolkit {

  // Algorithm Enable/Disable Flags
  namespace ScientificAlgorithms {
    
    // Individual algorithm toggles
    static constexpr bool ENABLE_PMC6960643_FEATURE_BASED = false;        // "Eye Tracking: A Comprehensive Guide to Methods and Measures" - COMMENTED OUT
    static constexpr bool ENABLE_FRONTIERS2019_DEFORMABLE_SHAPE = false;  // "Deformable Shape Models for Eye Tracking" - COMMENTED OUT
    static constexpr bool ENABLE_PMC8018226_MODEL_BASED = true;           // "Model-Based Eye Image Analysis for Facial Expression Recognition" - ONLY ENABLED
    static constexpr bool ENABLE_SPRINGER2024_ML_BASED = false;           // "Deep Learning for Eye Tracking" + "Machine Learning for Eye Movement Classification" - COMMENTED OUT
    static constexpr bool ENABLE_HYBRID_SCIENTIFIC = false;               // Combines all methods - COMMENTED OUT
    
    // Fusion method selection (when hybrid is enabled)
    enum class FusionMethod {
      WEIGHTED,      // Weighted average based on configured weights
      UNCERTAINTY,   // Uncertainty-weighted fusion
      VOTING         // Median voting
    };
    
    static constexpr FusionMethod DEFAULT_FUSION_METHOD = FusionMethod::UNCERTAINTY;
    
    // Individual algorithm weights (for weighted fusion)
    static constexpr float PMC6960643_FEATURE_BASED_WEIGHT = 0.2f;
    static constexpr float FRONTIERS2019_DEFORMABLE_SHAPE_WEIGHT = 0.3f;
    static constexpr float PMC8018226_MODEL_BASED_WEIGHT = 0.2f;
    static constexpr float SPRINGER2024_ML_BASED_WEIGHT = 0.3f;
    
    // Calibration thresholds
    static constexpr int MIN_SAMPLES_FOR_CALIBRATION = 50;
    static constexpr float MIN_CONFIDENCE_THRESHOLD = 0.3f;
    
    // Performance settings
    static constexpr bool ENABLE_REAL_TIME_PROCESSING = true;
    static constexpr bool ENABLE_ADAPTIVE_LEARNING = true;
    static constexpr bool ENABLE_CONFIDENCE_TRACKING = true;
    
    // Debug settings
    static constexpr bool ENABLE_DEBUG_LOGGING = false;
    static constexpr bool ENABLE_PERFORMANCE_MONITORING = false;
    
  }

  // Algorithm Manager Configuration
  struct ScientificAlgorithmConfig {
    // Enable flags
    bool enablePMC6960643_FeatureBased = ScientificAlgorithms::ENABLE_PMC6960643_FEATURE_BASED;
    bool enableFrontiers2019_DeformableShape = ScientificAlgorithms::ENABLE_FRONTIERS2019_DEFORMABLE_SHAPE;
    bool enablePMC8018226_ModelBased = ScientificAlgorithms::ENABLE_PMC8018226_MODEL_BASED;
    bool enableSpringer2024_MLBased = ScientificAlgorithms::ENABLE_SPRINGER2024_ML_BASED;
    bool enableHybridScientific = ScientificAlgorithms::ENABLE_HYBRID_SCIENTIFIC;
    
    // Fusion settings
    ScientificAlgorithms::FusionMethod fusionMethod = ScientificAlgorithms::DEFAULT_FUSION_METHOD;
    
    // Weights
    float pmc6960643_FeatureBasedWeight = ScientificAlgorithms::PMC6960643_FEATURE_BASED_WEIGHT;
    float frontiers2019_DeformableShapeWeight = ScientificAlgorithms::FRONTIERS2019_DEFORMABLE_SHAPE_WEIGHT;
    float pmc8018226_ModelBasedWeight = ScientificAlgorithms::PMC8018226_MODEL_BASED_WEIGHT;
    float springer2024_MLBasedWeight = ScientificAlgorithms::SPRINGER2024_ML_BASED_WEIGHT;
    
    // Calibration settings
    int minSamplesForCalibration = ScientificAlgorithms::MIN_SAMPLES_FOR_CALIBRATION;
    float minConfidenceThreshold = ScientificAlgorithms::MIN_CONFIDENCE_THRESHOLD;
    
    // Performance settings
    bool enableRealTimeProcessing = ScientificAlgorithms::ENABLE_REAL_TIME_PROCESSING;
    bool enableAdaptiveLearning = ScientificAlgorithms::ENABLE_ADAPTIVE_LEARNING;
    bool enableConfidenceTracking = ScientificAlgorithms::ENABLE_CONFIDENCE_TRACKING;
    
    // Debug settings
    bool enableDebugLogging = ScientificAlgorithms::ENABLE_DEBUG_LOGGING;
    bool enablePerformanceMonitoring = ScientificAlgorithms::ENABLE_PERFORMANCE_MONITORING;
  };

  // Global configuration instance
  extern ScientificAlgorithmConfig g_scientificAlgorithmConfig;

  // Configuration helper functions
  void InitializeScientificAlgorithms();
  void ConfigureScientificAlgorithm(const std::string& name, bool enabled);
  void SetScientificAlgorithmWeight(const std::string& name, float weight);
  void SetScientificFusionMethod(ScientificAlgorithms::FusionMethod method);
  
  // Runtime configuration
  bool IsScientificAlgorithmEnabled(const std::string& name);
  float GetScientificAlgorithmWeight(const std::string& name);
  ScientificAlgorithms::FusionMethod GetScientificFusionMethod();

}
