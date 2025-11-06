#include "scientific_algorithm_config.h"
#include <map>
#include <string>
#include <algorithm>

namespace psvr2_toolkit {

  // Global configuration instance
  ScientificAlgorithmConfig g_scientificAlgorithmConfig;

  void InitializeScientificAlgorithms() {
    // Initialize with default configuration
    g_scientificAlgorithmConfig = ScientificAlgorithmConfig();
  }

  void ConfigureScientificAlgorithm(const std::string& name, bool enabled) {
    if (name == "pmc6960643_feature") {
      g_scientificAlgorithmConfig.enablePMC6960643_FeatureBased = enabled;
    } else if (name == "frontiers2019_deformable") {
      g_scientificAlgorithmConfig.enableFrontiers2019_DeformableShape = enabled;
    } else if (name == "pmc8018226_model") {
      g_scientificAlgorithmConfig.enablePMC8018226_ModelBased = enabled;
    } else if (name == "springer2024_ml") {
      g_scientificAlgorithmConfig.enableSpringer2024_MLBased = enabled;
    } else if (name == "hybrid_scientific") {
      g_scientificAlgorithmConfig.enableHybridScientific = enabled;
    }
  }

  void SetScientificAlgorithmWeight(const std::string& name, float weight) {
    if (name == "pmc6960643_feature") {
      g_scientificAlgorithmConfig.pmc6960643_FeatureBasedWeight = std::clamp(weight, 0.0f, 1.0f);
    } else if (name == "frontiers2019_deformable") {
      g_scientificAlgorithmConfig.frontiers2019_DeformableShapeWeight = std::clamp(weight, 0.0f, 1.0f);
    } else if (name == "pmc8018226_model") {
      g_scientificAlgorithmConfig.pmc8018226_ModelBasedWeight = std::clamp(weight, 0.0f, 1.0f);
    } else if (name == "springer2024_ml") {
      g_scientificAlgorithmConfig.springer2024_MLBasedWeight = std::clamp(weight, 0.0f, 1.0f);
    }
  }

  void SetScientificFusionMethod(ScientificAlgorithms::FusionMethod method) {
    g_scientificAlgorithmConfig.fusionMethod = method;
  }

  bool IsScientificAlgorithmEnabled(const std::string& name) {
    if (name == "pmc6960643_feature") {
      return g_scientificAlgorithmConfig.enablePMC6960643_FeatureBased;
    } else if (name == "frontiers2019_deformable") {
      return g_scientificAlgorithmConfig.enableFrontiers2019_DeformableShape;
    } else if (name == "pmc8018226_model") {
      return g_scientificAlgorithmConfig.enablePMC8018226_ModelBased;
    } else if (name == "springer2024_ml") {
      return g_scientificAlgorithmConfig.enableSpringer2024_MLBased;
    } else if (name == "hybrid_scientific") {
      return g_scientificAlgorithmConfig.enableHybridScientific;
    }
    return false;
  }

  float GetScientificAlgorithmWeight(const std::string& name) {
    if (name == "pmc6960643_feature") {
      return g_scientificAlgorithmConfig.pmc6960643_FeatureBasedWeight;
    } else if (name == "frontiers2019_deformable") {
      return g_scientificAlgorithmConfig.frontiers2019_DeformableShapeWeight;
    } else if (name == "pmc8018226_model") {
      return g_scientificAlgorithmConfig.pmc8018226_ModelBasedWeight;
    } else if (name == "springer2024_ml") {
      return g_scientificAlgorithmConfig.springer2024_MLBasedWeight;
    }
    return 0.0f;
  }

  ScientificAlgorithms::FusionMethod GetScientificFusionMethod() {
    return g_scientificAlgorithmConfig.fusionMethod;
  }

}
