#pragma once

#include "scientific_algorithms.h"
#include "scientific_algorithm_config.h"
#include "modern_eyelid_estimator.h"

namespace psvr2_toolkit {

  // Example integration class showing how to use scientific algorithms
  class ScientificAlgorithmIntegration {
  public:
    ScientificAlgorithmIntegration();
    ~ScientificAlgorithmIntegration() = default;

    // Main estimation function with scientific algorithm support
    float EstimateOpennessWithScientificAlgorithms(const EyeData& eye);
    
    // Individual algorithm testing
    float TestFeatureBasedAlgorithm(const EyeData& eye);
    float TestDeformableShapeAlgorithm(const EyeData& eye);
    float TestModelBasedAlgorithm(const EyeData& eye);
    float TestMLBasedAlgorithm(const EyeData& eye);
    float TestHybridAlgorithm(const EyeData& eye);
    
    // Configuration
    void EnableScientificAlgorithm(const std::string& name, bool enabled);
    void SetScientificAlgorithmWeight(const std::string& name, float weight);
    void SetFusionMethod(const std::string& method);
    
    // Learning and calibration
    void UpdateLearning(const EyeData& eye, float groundTruth = -1.0f);
    void ResetAllAlgorithms();
    
    // Status and debugging
    std::map<std::string, float> GetAlgorithmConfidences() const;
    std::vector<std::string> GetAvailableAlgorithms() const;
    bool IsAlgorithmCalibrated(const std::string& name) const;

  private:
    ScientificAlgorithmManager m_algorithmManager;
    bool m_scientificAlgorithmsEnabled;
    std::string m_primaryAlgorithm;
  };

  // Implementation
  ScientificAlgorithmIntegration::ScientificAlgorithmIntegration() 
    : m_scientificAlgorithmsEnabled(false), m_primaryAlgorithm("hybrid") {
    // Initialize scientific algorithms
    InitializeScientificAlgorithms();
  }

  float ScientificAlgorithmIntegration::EstimateOpennessWithScientificAlgorithms(const EyeData& eye) {
    // Check if scientific algorithms are enabled
    if (!m_scientificAlgorithmsEnabled) {
      return 0.5f; // Return neutral value if disabled
    }

    // Use the configured primary algorithm
    return m_algorithmManager.EstimateOpenness(eye, m_primaryAlgorithm);
  }

  float ScientificAlgorithmIntegration::TestFeatureBasedAlgorithm(const EyeData& eye) {
    return m_algorithmManager.GetFeatureBasedEstimator().EstimateOpenness(eye);
  }

  float ScientificAlgorithmIntegration::TestDeformableShapeAlgorithm(const EyeData& eye) {
    return m_algorithmManager.GetDeformableShapeEstimator().EstimateOpenness(eye);
  }

  float ScientificAlgorithmIntegration::TestModelBasedAlgorithm(const EyeData& eye) {
    return m_algorithmManager.GetModelBasedEstimator().EstimateOpenness(eye);
  }

  float ScientificAlgorithmIntegration::TestMLBasedAlgorithm(const EyeData& eye) {
    return m_algorithmManager.GetMLBasedEstimator().EstimateOpenness(eye);
  }

  float ScientificAlgorithmIntegration::TestHybridAlgorithm(const EyeData& eye) {
    return m_algorithmManager.GetHybridEstimator().EstimateOpenness(eye);
  }

  void ScientificAlgorithmIntegration::EnableScientificAlgorithm(const std::string& name, bool enabled) {
    m_algorithmManager.EnableAlgorithm(name, enabled);
    ConfigureScientificAlgorithm(name, enabled);
    
    // Update enabled status
    if (name == "hybrid" && enabled) {
      m_scientificAlgorithmsEnabled = true;
      m_primaryAlgorithm = "hybrid";
    }
  }

  void ScientificAlgorithmIntegration::SetScientificAlgorithmWeight(const std::string& name, float weight) {
    m_algorithmManager.SetAlgorithmWeight(name, weight);
    SetScientificAlgorithmWeight(name, weight);
  }

  void ScientificAlgorithmIntegration::SetFusionMethod(const std::string& method) {
    m_algorithmManager.SetFusionMethod(method);
    
    if (method == "weighted") {
      SetScientificFusionMethod(ScientificAlgorithms::FusionMethod::WEIGHTED);
    } else if (method == "uncertainty") {
      SetScientificFusionMethod(ScientificAlgorithms::FusionMethod::UNCERTAINTY);
    } else if (method == "voting") {
      SetScientificFusionMethod(ScientificAlgorithms::FusionMethod::VOTING);
    }
  }

  void ScientificAlgorithmIntegration::UpdateLearning(const EyeData& eye, float groundTruth) {
    m_algorithmManager.UpdateLearning(eye, groundTruth);
  }

  void ScientificAlgorithmIntegration::ResetAllAlgorithms() {
    m_algorithmManager.ResetAll();
  }

  std::map<std::string, float> ScientificAlgorithmIntegration::GetAlgorithmConfidences() const {
    return m_algorithmManager.GetMethodConfidences();
  }

  std::vector<std::string> ScientificAlgorithmIntegration::GetAvailableAlgorithms() const {
    return m_algorithmManager.GetAvailableMethods();
  }

  bool ScientificAlgorithmIntegration::IsAlgorithmCalibrated(const std::string& name) const {
    if (name == "feature") {
      return m_algorithmManager.GetFeatureBasedEstimator().IsCalibrated();
    } else if (name == "deformable") {
      return m_algorithmManager.GetDeformableShapeEstimator().IsCalibrated();
    } else if (name == "model") {
      return m_algorithmManager.GetModelBasedEstimator().IsCalibrated();
    } else if (name == "ml") {
      return m_algorithmManager.GetMLBasedEstimator().IsCalibrated();
    } else if (name == "hybrid") {
      return m_algorithmManager.GetHybridEstimator().IsCalibrated();
    }
    return false;
  }

}
