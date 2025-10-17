#pragma once

#include "modern_eyelid_estimator.h"
#include <vector>
#include <map>
#include <memory>

namespace psvr2_toolkit {

  // Forward declarations for all scientific algorithm modules
  class PMC6960643_FeatureBasedEstimator;      // "Eye Tracking: A Comprehensive Guide to Methods and Measures"
  class Frontiers2019_DeformableShapeEstimator; // "Deformable Shape Models for Eye Tracking"
  class PMC8018226_ModelBasedEstimator;        // "Model-Based Eye Image Analysis for Facial Expression Recognition"
  class Springer2024_MLBasedEstimator;         // "Deep Learning for Eye Tracking"
  class HybridScientificEstimator;              // Combines all methods

  // Base class for all scientific estimators
  class ScientificEstimator {
  public:
    virtual ~ScientificEstimator() = default;
    virtual float EstimateOpenness(const EyeData& eye) = 0;
    virtual void UpdateLearning(const EyeData& eye, float groundTruth = -1.0f) = 0;
    virtual bool IsCalibrated() const = 0;
    virtual void Reset() = 0;
    virtual std::string GetName() const = 0;
    virtual float GetConfidence() const = 0;
  };

  // Feature-Based Techniques (Gabor filters, template matching)
  // Based on: PMC6960643 - "Eye Tracking: A Comprehensive Guide to Methods and Measures"
  class PMC6960643_FeatureBasedEstimator : public ScientificEstimator {
  public:
    PMC6960643_FeatureBasedEstimator();
    ~PMC6960643_FeatureBasedEstimator() override = default;

    float EstimateOpenness(const EyeData& eye) override;
    void UpdateLearning(const EyeData& eye, float groundTruth = -1.0f) override;
    bool IsCalibrated() const override;
    void Reset() override;
    std::string GetName() const override { return "PMC6960643_FeatureBased"; }
    float GetConfidence() const override;

  private:
    struct GaborEdgeDetector {
      std::vector<float> orientations;
      std::vector<float> frequencies;
      float DetectEyelidEdges(const EyeData& eye);
      void InitializeFilters();
    } m_gaborDetector;

    struct EyeFeatureDetector {
      struct EyeFeatures {
        float irisVisibility;
        float pupilContrast;
        float eyelidContour;
        float cornerDistance;
      };
      
      EyeFeatures DetectFeatures(const EyeData& eye);
      void LearnFeaturePatterns(const EyeData& eye, float openness);
    } m_featureDetector;

    struct TemplateMatcher {
      std::map<float, std::vector<float>> learnedTemplates; // gaze angle -> template
      float MatchTemplate(const EyeData& eye, float gazeAngle);
      void UpdateTemplate(float gazeAngle, const EyeData& eye, float openness);
    } m_templateMatcher;

    bool m_isCalibrated;
    int m_sampleCount;
    float m_confidence;
  };

  // Deformable Shape Models (landmark detection, shape fitting)
  // Based on: Frontiers2019 - "Deformable Shape Models for Eye Tracking"
  class Frontiers2019_DeformableShapeEstimator : public ScientificEstimator {
  public:
    Frontiers2019_DeformableShapeEstimator();
    ~Frontiers2019_DeformableShapeEstimator() override = default;

    float EstimateOpenness(const EyeData& eye) override;
    void UpdateLearning(const EyeData& eye, float groundTruth = -1.0f) override;
    bool IsCalibrated() const override;
    void Reset() override;
    std::string GetName() const override { return "Frontiers2019_DeformableShape"; }
    float GetConfidence() const override;

  private:
    struct EyeShapeModel {
      struct Landmark {
        Vector2 position;
        float confidence;
        bool isVisible;
      };
      
      std::vector<Landmark> upperEyelidLandmarks;
      std::vector<Landmark> lowerEyelidLandmarks;
      std::vector<Landmark> pupilLandmarks;
      float curvatureFactor;
      float thicknessFactor;
      bool isValid;
    };

    struct ShapeFitter {
      EyeShapeModel FitToData(const EyeData& eye);
      void OptimizeShape(EyeShapeModel& model, const EyeData& eye);
      float CalculateOpenness(const EyeShapeModel& model);
    } m_shapeFitter;

    struct LandmarkDetector {
      std::vector<Landmark> DetectEyelidLandmarks(const EyeData& eye);
      std::vector<Landmark> DetectPupilLandmarks(const EyeData& eye);
      bool ValidateLandmarks(const std::vector<Landmark>& landmarks);
    } m_landmarkDetector;

    EyeShapeModel m_currentModel;
    bool m_isCalibrated;
    int m_sampleCount;
    float m_confidence;
  };

  // Model-Based Analysis (eye region modeling, registration)
  // Based on: PMC8018226 - "Model-Based Eye Image Analysis for Facial Expression Recognition"
  class PMC8018226_ModelBasedEstimator : public ScientificEstimator {
  public:
    PMC8018226_ModelBasedEstimator();
    ~PMC8018226_ModelBasedEstimator() override = default;

    float EstimateOpenness(const EyeData& eye) override;
    void UpdateLearning(const EyeData& eye, float groundTruth = -1.0f) override;
    bool IsCalibrated() const override;
    void Reset() override;
    std::string GetName() const override { return "PMC8018226_ModelBased"; }
    float GetConfidence() const override;

  private:
    struct EyeRegionModel {
      Vector3 eyeCenter;
      float eyeRadius;
      Vector3 pupilCenter;
      float pupilRadius;
      struct EyelidContours {
        std::vector<Vector2> upperContour;
        std::vector<Vector2> lowerContour;
        float curvature;
        float thickness;
      } eyelidContours;
      bool isValid;
    };

    struct ModelRegistrator {
      EyeRegionModel RegisterToData(const EyeData& eye);
      void OptimizeRegistration(EyeRegionModel& model, const EyeData& eye);
      float CalculateOpenness(const EyeRegionModel& model);
    } m_registrator;

    struct MotionStabilizer {
      Vector3 StabilizeHeadMotion(const EyeData& eye);
      void UpdateMotionModel(const EyeData& eye);
    } m_motionStabilizer;

    EyeRegionModel m_currentModel;
    bool m_isCalibrated;
    int m_sampleCount;
    float m_confidence;
  };

  // Machine Learning-Based Estimators (CNN, SVM)
  // Based on: Springer2024 - "Deep Learning for Eye Tracking" + MDPI2024 - "Machine Learning for Eye Movement Classification"
  class Springer2024_MLBasedEstimator : public ScientificEstimator {
  public:
    Springer2024_MLBasedEstimator();
    ~Springer2024_MLBasedEstimator() override = default;

    float EstimateOpenness(const EyeData& eye) override;
    void UpdateLearning(const EyeData& eye, float groundTruth = -1.0f) override;
    bool IsCalibrated() const override;
    void Reset() override;
    std::string GetName() const override { return "Springer2024_MLBased"; }
    float GetConfidence() const override;

  private:
    struct CNNOpennessEstimator {
      struct EyeOpennessCNN {
        // Simplified CNN structure for real-time processing
        std::vector<float> weights;
        std::vector<float> biases;
        int inputSize;
        int outputSize;
        bool isTrained;
      };
      
      float Predict(const EyeData& eye);
      void Train(const std::vector<EyeData>& samples, const std::vector<float>& labels);
      void InitializeNetwork();
    } m_cnnEstimator;

    struct SVMEyeClassifier {
      struct EyeFeatures {
        float pupilDiameter;
        float pupilPositionY;
        float irisVisibility;
        float eyelidContour;
        Vector2 gazeDirection;
        float blinkIndicator;
      };
      
      float Classify(const EyeFeatures& features);
      void Train(const std::vector<EyeFeatures>& features, const std::vector<float>& labels);
      EyeFeatures ExtractFeatures(const EyeData& eye);
    } m_svmClassifier;

    struct FeatureExtractor {
      std::vector<float> ExtractFeatures(const EyeData& eye);
      void NormalizeFeatures(std::vector<float>& features);
    } m_featureExtractor;

    bool m_isCalibrated;
    int m_sampleCount;
    float m_confidence;
    bool m_useCNN;
    bool m_useSVM;
  };

  // Hybrid Estimator (combines all methods)
  // Combines: PMC6960643 + Frontiers2019 + PMC8018226 + Springer2024
  class HybridScientificEstimator : public ScientificEstimator {
  public:
    HybridScientificEstimator();
    ~HybridScientificEstimator() override = default;

    float EstimateOpenness(const EyeData& eye) override;
    void UpdateLearning(const EyeData& eye, float groundTruth = -1.0f) override;
    bool IsCalibrated() const override;
    void Reset() override;
    std::string GetName() const override { return "HybridScientific"; }
    float GetConfidence() const override;

    // Configuration
    void EnableEstimator(const std::string& name, bool enabled);
    void SetEstimatorWeight(const std::string& name, float weight);
    void SetFusionMethod(const std::string& method); // "weighted", "uncertainty", "voting"

  private:
    struct EstimatorConfig {
      std::unique_ptr<ScientificEstimator> estimator;
      bool enabled;
      float weight;
      float confidence;
    };

    std::map<std::string, EstimatorConfig> m_estimators;
    std::string m_fusionMethod;

    float FuseEstimates(const std::map<std::string, float>& estimates);
    float WeightedFusion(const std::map<std::string, float>& estimates);
    float UncertaintyFusion(const std::map<std::string, float>& estimates);
    float VotingFusion(const std::map<std::string, float>& estimates);
  };

  // Scientific Algorithm Manager (handles all modules)
  class ScientificAlgorithmManager {
  public:
    ScientificAlgorithmManager();
    ~ScientificAlgorithmManager() = default;

    // Individual estimator access
    PMC6960643_FeatureBasedEstimator& GetPMC6960643_FeatureBasedEstimator() { return m_pmc6960643_featureBased; }
    Frontiers2019_DeformableShapeEstimator& GetFrontiers2019_DeformableShapeEstimator() { return m_frontiers2019_deformableShape; }
    PMC8018226_ModelBasedEstimator& GetPMC8018226_ModelBasedEstimator() { return m_pmc8018226_modelBased; }
    Springer2024_MLBasedEstimator& GetSpringer2024_MLBasedEstimator() { return m_springer2024_mlBased; }
    HybridScientificEstimator& GetHybridScientificEstimator() { return m_hybridScientific; }

    // Configuration
    void EnableAlgorithm(const std::string& name, bool enabled);
    void SetAlgorithmWeight(const std::string& name, float weight);
    void SetFusionMethod(const std::string& method);

    // Main interface
    float EstimateOpenness(const EyeData& eye, const std::string& method = "hybrid");
    void UpdateLearning(const EyeData& eye, float groundTruth = -1.0f);
    void ResetAll();
    std::vector<std::string> GetAvailableMethods() const;
    std::map<std::string, float> GetMethodConfidences() const;

  private:
    PMC6960643_FeatureBasedEstimator m_pmc6960643_featureBased;
    Frontiers2019_DeformableShapeEstimator m_frontiers2019_deformableShape;
    PMC8018226_ModelBasedEstimator m_pmc8018226_modelBased;
    Springer2024_MLBasedEstimator m_springer2024_mlBased;
    HybridScientificEstimator m_hybridScientific;

    std::map<std::string, bool> m_enabledMethods;
    std::string m_primaryMethod;
  };

}
