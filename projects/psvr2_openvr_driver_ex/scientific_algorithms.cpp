#include "scientific_algorithms.h"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace psvr2_toolkit {

  // PMC6960643 Feature-Based Estimator Implementation
  // Based on: "Eye Tracking: A Comprehensive Guide to Methods and Measures"
  // COMMENTED OUT - Requires image data not available
  /*
  PMC6960643_FeatureBasedEstimator::PMC6960643_FeatureBasedEstimator() 
    : m_isCalibrated(false), m_sampleCount(0), m_confidence(0.0f) {
    m_gaborDetector.InitializeFilters();
  }

  float PMC6960643_FeatureBasedEstimator::EstimateOpenness(const EyeData& eye) {
    if (!eye.isValid) return 0.5f;

    // 1. Detect eyelid edges using Gabor filters
    float edgeStrength = m_gaborDetector.DetectEyelidEdges(eye);

    // 2. Extract eye features
    auto features = m_featureDetector.DetectFeatures(eye);

    // 3. Match against learned templates
    float gazeAngle = std::asin(std::clamp(std::abs(eye.gazeDir.y), 0.0f, 1.0f));
    float templateMatch = m_templateMatcher.MatchTemplate(eye, gazeAngle);

    // 4. Combine features for openness estimation
    float openness = 0.0f;
    if (m_isCalibrated) {
      // Use learned weights
      openness = 0.3f * edgeStrength + 
                 0.4f * features.irisVisibility + 
                 0.2f * features.eyelidContour + 
                 0.1f * templateMatch;
    } else {
      // Use basic heuristics
      openness = 0.5f * edgeStrength + 0.5f * features.irisVisibility;
    }

    m_confidence = CalculateConfidence(features, edgeStrength);
    return std::clamp(openness, 0.0f, 1.0f);
  }

  void PMC6960643_FeatureBasedEstimator::UpdateLearning(const EyeData& eye, float groundTruth) {
    if (!eye.isValid) return;

    m_sampleCount++;
    
    // Learn feature patterns
    if (groundTruth >= 0.0f) {
      m_featureDetector.LearnFeaturePatterns(eye, groundTruth);
      
      // Update templates
      float gazeAngle = std::asin(std::clamp(std::abs(eye.gazeDir.y), 0.0f, 1.0f));
      m_templateMatcher.UpdateTemplate(gazeAngle, eye, groundTruth);
    }

    // Mark as calibrated after sufficient samples
    if (m_sampleCount > 50) {
      m_isCalibrated = true;
    }
  }

  bool PMC6960643_FeatureBasedEstimator::IsCalibrated() const {
    return m_isCalibrated;
  }

  void PMC6960643_FeatureBasedEstimator::Reset() {
    m_isCalibrated = false;
    m_sampleCount = 0;
    m_confidence = 0.0f;
    m_templateMatcher.learnedTemplates.clear();
  }

  float PMC6960643_FeatureBasedEstimator::GetConfidence() const {
    return m_confidence;
  }

  // GaborEdgeDetector Implementation
  void PMC6960643_FeatureBasedEstimator::GaborEdgeDetector::InitializeFilters() {
    // Initialize Gabor filter orientations and frequencies
    orientations = {0.0f, 45.0f, 90.0f, 135.0f}; // degrees
    frequencies = {0.1f, 0.2f, 0.3f}; // cycles per pixel
  }

  float PMC6960643_FeatureBasedEstimator::GaborEdgeDetector::DetectEyelidEdges(const EyeData& eye) {
    // Simplified Gabor edge detection for eyelid contours
    // In a real implementation, this would process actual image data
    
    // Use pupil position as proxy for eyelid edge detection
    float edgeStrength = 0.0f;
    
    // Simulate edge detection based on pupil dynamics
    if (eye.pupilDiaMm > 0) {
      // Higher pupil diameter suggests more open eyes
      edgeStrength = std::clamp(eye.pupilDiaMm / 4.0f, 0.0f, 1.0f);
    }
    
    return edgeStrength;
  }

  // EyeFeatureDetector Implementation
  PMC6960643_FeatureBasedEstimator::EyeFeatureDetector::EyeFeatures 
  PMC6960643_FeatureBasedEstimator::EyeFeatureDetector::DetectFeatures(const EyeData& eye) {
    EyeFeatures features;
    
    // Calculate iris visibility (simplified)
    features.irisVisibility = std::clamp(eye.pupilDiaMm / 4.0f, 0.0f, 1.0f);
    
    // Calculate pupil contrast (simplified)
    features.pupilContrast = 0.8f; // Assume good contrast in IR
    
    // Calculate eyelid contour (simplified)
    features.eyelidContour = std::clamp(eye.pupilPosY, 0.0f, 1.0f);
    
    // Calculate corner distance (simplified)
    features.cornerDistance = 0.5f; // Assume standard distance
    
    return features;
  }

  void PMC6960643_FeatureBasedEstimator::EyeFeatureDetector::LearnFeaturePatterns(
    const EyeData& eye, float openness) {
    // Learn patterns from ground truth data
    // This would update internal models in a real implementation
  }

  // TemplateMatcher Implementation
  float PMC6960643_FeatureBasedEstimator::TemplateMatcher::MatchTemplate(
    const EyeData& eye, float gazeAngle) {
    // Find closest gaze angle template
    float bestMatch = 0.5f;
    float minDistance = std::numeric_limits<float>::max();
    
    for (const auto& pair : learnedTemplates) {
      float distance = std::abs(pair.first - gazeAngle);
      if (distance < minDistance) {
        minDistance = distance;
        bestMatch = pair.second[0]; // Simplified - use first template value
      }
    }
    
    return bestMatch;
  }

  void PMC6960643_FeatureBasedEstimator::TemplateMatcher::UpdateTemplate(
    float gazeAngle, const EyeData& eye, float openness) {
    // Update or create template for this gaze angle
    learnedTemplates[gazeAngle] = {openness}; // Simplified template
  }
  */

  // Frontiers2019 Deformable Shape Estimator Implementation
  // Based on: "Deformable Shape Models for Eye Tracking"
  // COMMENTED OUT - Requires image data and landmark detection not available
  /*
  Frontiers2019_DeformableShapeEstimator::Frontiers2019_DeformableShapeEstimator() 
    : m_isCalibrated(false), m_sampleCount(0), m_confidence(0.0f) {
    m_currentModel.isValid = false;
  }

  float Frontiers2019_DeformableShapeEstimator::EstimateOpenness(const EyeData& eye) {
    if (!eye.isValid) return 0.5f;

    // 1. Detect landmarks
    auto upperLandmarks = m_landmarkDetector.DetectEyelidLandmarks(eye);
    auto lowerLandmarks = m_landmarkDetector.DetectEyelidLandmarks(eye);
    auto pupilLandmarks = m_landmarkDetector.DetectPupilLandmarks(eye);

    // 2. Fit shape model
    m_currentModel = m_shapeFitter.FitToData(eye);

    // 3. Calculate openness from shape model
    float openness = 0.5f;
    if (m_currentModel.isValid) {
      openness = m_shapeFitter.CalculateOpenness(m_currentModel);
    }

    m_confidence = CalculateShapeConfidence(m_currentModel);
    return std::clamp(openness, 0.0f, 1.0f);
  }

  void Frontiers2019_DeformableShapeEstimator::UpdateLearning(const EyeData& eye, float groundTruth) {
    if (!eye.isValid) return;

    m_sampleCount++;
    
    // Learn shape parameters from ground truth
    if (groundTruth >= 0.0f && m_currentModel.isValid) {
      // Update model parameters based on ground truth
      UpdateShapeModel(m_currentModel, groundTruth);
    }

    // Mark as calibrated after sufficient samples
    if (m_sampleCount > 30) {
      m_isCalibrated = true;
    }
  }

  bool Frontiers2019_DeformableShapeEstimator::IsCalibrated() const {
    return m_isCalibrated;
  }

  void Frontiers2019_DeformableShapeEstimator::Reset() {
    m_isCalibrated = false;
    m_sampleCount = 0;
    m_confidence = 0.0f;
    m_currentModel.isValid = false;
  }

  float Frontiers2019_DeformableShapeEstimator::GetConfidence() const {
    return m_confidence;
  }

  // ShapeFitter Implementation
  Frontiers2019_DeformableShapeEstimator::EyeShapeModel 
  Frontiers2019_DeformableShapeEstimator::ShapeFitter::FitToData(const EyeData& eye) {
    EyeShapeModel model;
    
    // Simplified shape fitting
    model.upperEyelidLandmarks = {
      {{0.0f, eye.pupilPosY + 0.1f}, 0.8f, true},
      {{0.5f, eye.pupilPosY + 0.05f}, 0.9f, true},
      {{1.0f, eye.pupilPosY + 0.1f}, 0.8f, true}
    };
    
    model.lowerEyelidLandmarks = {
      {{0.0f, eye.pupilPosY - 0.1f}, 0.8f, true},
      {{0.5f, eye.pupilPosY - 0.05f}, 0.9f, true},
      {{1.0f, eye.pupilPosY - 0.1f}, 0.8f, true}
    };
    
    model.pupilLandmarks = {
      {{0.5f, eye.pupilPosY}, 0.95f, true}
    };
    
    model.curvatureFactor = 1.0f;
    model.thicknessFactor = 1.0f;
    model.isValid = true;
    
    return model;
  }

  float Frontiers2019_DeformableShapeEstimator::ShapeFitter::CalculateOpenness(
    const EyeShapeModel& model) {
    if (!model.isValid) return 0.5f;

    // Calculate openness from eyelid landmark distances
    float upperY = 0.0f, lowerY = 0.0f;
    
    for (const auto& landmark : model.upperEyelidLandmarks) {
      if (landmark.isVisible) {
        upperY += landmark.position.y;
      }
    }
    
    for (const auto& landmark : model.lowerEyelidLandmarks) {
      if (landmark.isVisible) {
        lowerY += landmark.position.y;
      }
    }
    
    if (!model.upperEyelidLandmarks.empty() && !model.lowerEyelidLandmarks.empty()) {
      upperY /= model.upperEyelidLandmarks.size();
      lowerY /= model.lowerEyelidLandmarks.size();
      
      float openness = (upperY - lowerY) / 0.2f; // Normalize to 0-1
      return std::clamp(openness, 0.0f, 1.0f);
    }
    
    return 0.5f;
  }

  // LandmarkDetector Implementation
  std::vector<Frontiers2019_DeformableShapeEstimator::EyeShapeModel::Landmark> 
  Frontiers2019_DeformableShapeEstimator::LandmarkDetector::DetectEyelidLandmarks(const EyeData& eye) {
    // Simplified landmark detection
    std::vector<EyeShapeModel::Landmark> landmarks;
    
    // Create landmarks based on pupil position
    landmarks.push_back({{0.0f, eye.pupilPosY + 0.1f}, 0.8f, true});
    landmarks.push_back({{0.5f, eye.pupilPosY + 0.05f}, 0.9f, true});
    landmarks.push_back({{1.0f, eye.pupilPosY + 0.1f}, 0.8f, true});
    
    return landmarks;
  }

  std::vector<Frontiers2019_DeformableShapeEstimator::EyeShapeModel::Landmark> 
  Frontiers2019_DeformableShapeEstimator::LandmarkDetector::DetectPupilLandmarks(const EyeData& eye) {
    // Simplified pupil landmark detection
    std::vector<EyeShapeModel::Landmark> landmarks;
    
    landmarks.push_back({{0.5f, eye.pupilPosY}, 0.95f, true});
    
    return landmarks;
  }

  bool Frontiers2019_DeformableShapeEstimator::LandmarkDetector::ValidateLandmarks(
    const std::vector<EyeShapeModel::Landmark>& landmarks) {
    // Validate landmark quality
    int visibleCount = 0;
    for (const auto& landmark : landmarks) {
      if (landmark.isVisible && landmark.confidence > 0.5f) {
        visibleCount++;
      }
    }
    
    return visibleCount >= landmarks.size() / 2;
  }
  */

  // PMC8018226 Model-Based Estimator Implementation
  // Based on: "Model-Based Eye Image Analysis for Facial Expression Recognition"
  PMC8018226_ModelBasedEstimator::PMC8018226_ModelBasedEstimator() 
    : m_isCalibrated(false), m_sampleCount(0), m_confidence(0.0f) {
    m_currentModel.isValid = false;
  }

  float PMC8018226_ModelBasedEstimator::EstimateOpenness(const EyeData& eye) {
    if (!eye.isValid) return 0.5f;

    // 1. Stabilize head motion
    Vector3 stabilizedGaze = m_motionStabilizer.StabilizeHeadMotion(eye);

    // 2. Register model to current data
    m_currentModel = m_registrator.RegisterToData(eye);

    // 3. Calculate openness from registered model
    float openness = 0.5f;
    if (m_currentModel.isValid) {
      openness = m_registrator.CalculateOpenness(m_currentModel);
    }

    m_confidence = 0.8f; // Placeholder confidence
    return std::clamp(openness, 0.0f, 1.0f);
  }

  void PMC8018226_ModelBasedEstimator::UpdateLearning(const EyeData& eye, float groundTruth) {
    if (!eye.isValid) return;

    m_sampleCount++;
    
    // Update motion model
    m_motionStabilizer.UpdateMotionModel(eye);
    
    // Learn model parameters from ground truth
    if (groundTruth >= 0.0f && m_currentModel.isValid) {
      // Placeholder learning - update model parameters based on ground truth
      m_currentModel.eyeRadius = m_currentModel.eyeRadius * 0.95f + groundTruth * 0.05f;
    }

    // Mark as calibrated after sufficient samples
    if (m_sampleCount > 40) {
      m_isCalibrated = true;
    }
  }

  bool PMC8018226_ModelBasedEstimator::IsCalibrated() const {
    return m_isCalibrated;
  }

  void PMC8018226_ModelBasedEstimator::Reset() {
    m_isCalibrated = false;
    m_sampleCount = 0;
    m_confidence = 0.0f;
    m_currentModel.isValid = false;
  }

  float PMC8018226_ModelBasedEstimator::GetConfidence() const {
    return m_confidence;
  }

  // ModelRegistrator Implementation
  PMC8018226_ModelBasedEstimator::EyeRegionModel 
  PMC8018226_ModelBasedEstimator::ModelRegistrator::RegisterToData(const EyeData& eye) {
    EyeRegionModel model;
    
    // Simplified model registration
    model.eyeCenter = Vector3(0, 0, 0);
    model.eyeRadius = 12.0f; // mm
    model.pupilCenter = Vector3(0, eye.pupilPosY, 0);
    model.pupilRadius = eye.pupilDiaMm / 2.0f;
    
    // Create eyelid contours
    model.eyelidContours.upperContour.clear();
    model.eyelidContours.upperContour.push_back(Vector2(0.0f, eye.pupilPosY + 0.1f));
    model.eyelidContours.upperContour.push_back(Vector2(0.5f, eye.pupilPosY + 0.05f));
    model.eyelidContours.upperContour.push_back(Vector2(1.0f, eye.pupilPosY + 0.1f));
    
    model.eyelidContours.lowerContour.clear();
    model.eyelidContours.lowerContour.push_back(Vector2(0.0f, eye.pupilPosY - 0.1f));
    model.eyelidContours.lowerContour.push_back(Vector2(0.5f, eye.pupilPosY - 0.05f));
    model.eyelidContours.lowerContour.push_back(Vector2(1.0f, eye.pupilPosY - 0.1f));
    
    model.eyelidContours.curvature = 1.0f;
    model.eyelidContours.thickness = 1.0f;
    model.isValid = true;
    
    return model;
  }

  float PMC8018226_ModelBasedEstimator::ModelRegistrator::CalculateOpenness(
    const EyeRegionModel& model) {
    if (!model.isValid) return 0.5f;

    // Calculate openness from eyelid contours
    float upperY = 0.0f, lowerY = 0.0f;
    
    for (const auto& point : model.eyelidContours.upperContour) {
      upperY += point.y;
    }
    
    for (const auto& point : model.eyelidContours.lowerContour) {
      lowerY += point.y;
    }
    
    if (!model.eyelidContours.upperContour.empty() && 
        !model.eyelidContours.lowerContour.empty()) {
      upperY /= model.eyelidContours.upperContour.size();
      lowerY /= model.eyelidContours.lowerContour.size();
      
      float openness = (upperY - lowerY) / 0.2f; // Normalize to 0-1
      return std::clamp(openness, 0.0f, 1.0f);
    }
    
    return 0.5f;
  }

  // MotionStabilizer Implementation
  Vector3 PMC8018226_ModelBasedEstimator::MotionStabilizer::StabilizeHeadMotion(const EyeData& eye) {
    // Simplified head motion stabilization
    return eye.gazeDir;
  }

  void PMC8018226_ModelBasedEstimator::MotionStabilizer::UpdateMotionModel(const EyeData& eye) {
    // Update motion model parameters
  }

  // Springer2024 ML-Based Estimator Implementation
  // Based on: "Deep Learning for Eye Tracking" + "Machine Learning for Eye Movement Classification"
  // COMMENTED OUT - Requires ML model training and more complex implementation
  /*
  Springer2024_MLBasedEstimator::Springer2024_MLBasedEstimator() 
    : m_isCalibrated(false), m_sampleCount(0), m_confidence(0.0f), 
      m_useCNN(true), m_useSVM(true) {
    m_cnnEstimator.InitializeNetwork();
  }

  float Springer2024_MLBasedEstimator::EstimateOpenness(const EyeData& eye) {
    if (!eye.isValid) return 0.5f;

    float openness = 0.5f;
    
    if (m_useCNN) {
      // Use CNN for estimation
      float cnnEstimate = m_cnnEstimator.Predict(eye);
      openness = cnnEstimate;
    }
    
    if (m_useSVM) {
      // Use SVM for estimation
      auto features = m_svmClassifier.ExtractFeatures(eye);
      float svmEstimate = m_svmClassifier.Classify(features);
      
      if (m_useCNN) {
        // Combine CNN and SVM estimates
        openness = 0.6f * openness + 0.4f * svmEstimate;
      } else {
        openness = svmEstimate;
      }
    }

    m_confidence = CalculateMLConfidence(eye);
    return std::clamp(openness, 0.0f, 1.0f);
  }

  void Springer2024_MLBasedEstimator::UpdateLearning(const EyeData& eye, float groundTruth) {
    if (!eye.isValid) return;

    m_sampleCount++;
    
    // Update ML models with ground truth
    if (groundTruth >= 0.0f) {
      // In a real implementation, this would update the trained models
      UpdateMLModels(eye, groundTruth);
    }

    // Mark as calibrated after sufficient samples
    if (m_sampleCount > 100) {
      m_isCalibrated = true;
    }
  }

  bool Springer2024_MLBasedEstimator::IsCalibrated() const {
    return m_isCalibrated;
  }

  void Springer2024_MLBasedEstimator::Reset() {
    m_isCalibrated = false;
    m_sampleCount = 0;
    m_confidence = 0.0f;
    m_cnnEstimator.InitializeNetwork();
  }

  float Springer2024_MLBasedEstimator::GetConfidence() const {
    return m_confidence;
  }

  // CNNOpennessEstimator Implementation
  void Springer2024_MLBasedEstimator::CNNOpennessEstimator::InitializeNetwork() {
    // Initialize simplified CNN structure
    inputSize = 64; // 64x64 input image
    outputSize = 1; // Single openness value
    isTrained = false;
    
    // Initialize weights and biases (simplified)
    weights.resize(inputSize * outputSize, 0.1f);
    biases.resize(outputSize, 0.0f);
  }

  float Springer2024_MLBasedEstimator::CNNOpennessEstimator::Predict(const EyeData& eye) {
    if (!isTrained) {
      // Return basic estimation if not trained
      return std::clamp(eye.pupilDiaMm / 4.0f, 0.0f, 1.0f);
    }
    
    // Simplified CNN prediction
    // In a real implementation, this would process actual image data
    float prediction = 0.5f;
    
    // Use pupil data as input features
    prediction = std::clamp(eye.pupilDiaMm / 4.0f, 0.0f, 1.0f);
    
    return prediction;
  }

  // SVMEyeClassifier Implementation
  Springer2024_MLBasedEstimator::SVMEyeClassifier::EyeFeatures 
  Springer2024_MLBasedEstimator::SVMEyeClassifier::ExtractFeatures(const EyeData& eye) {
    EyeFeatures features;
    
    features.pupilDiameter = eye.pupilDiaMm;
    features.pupilPositionY = eye.pupilPosY;
    features.irisVisibility = std::clamp(eye.pupilDiaMm / 4.0f, 0.0f, 1.0f);
    features.eyelidContour = eye.pupilPosY;
    features.gazeDirection = Vector2(eye.gazeDir.x, eye.gazeDir.y);
    features.blinkIndicator = eye.isBlink ? 1.0f : 0.0f;
    
    return features;
  }

  float Springer2024_MLBasedEstimator::SVMEyeClassifier::Classify(const EyeFeatures& features) {
    // Simplified SVM classification
    // In a real implementation, this would use trained SVM weights
    
    float score = 0.0f;
    score += 0.3f * features.pupilDiameter / 4.0f;
    score += 0.3f * features.irisVisibility;
    score += 0.2f * features.eyelidContour;
    score += 0.2f * (1.0f - features.blinkIndicator);
    
    return std::clamp(score, 0.0f, 1.0f);
  }

  // Hybrid Scientific Estimator Implementation
  // Combines: PMC6960643 + Frontiers2019 + PMC8018226 + Springer2024
  // COMMENTED OUT - Only works when other algorithms are enabled
  /*
  HybridScientificEstimator::HybridScientificEstimator() {
    // Initialize all estimators
    m_estimators["pmc6960643_feature"] = {std::make_unique<PMC6960643_FeatureBasedEstimator>(), true, 0.2f, 0.0f};
    m_estimators["frontiers2019_deformable"] = {std::make_unique<Frontiers2019_DeformableShapeEstimator>(), true, 0.3f, 0.0f};
    m_estimators["pmc8018226_model"] = {std::make_unique<PMC8018226_ModelBasedEstimator>(), true, 0.2f, 0.0f};
    m_estimators["springer2024_ml"] = {std::make_unique<Springer2024_MLBasedEstimator>(), true, 0.3f, 0.0f};
    
    m_fusionMethod = "uncertainty";
  }

  float HybridScientificEstimator::EstimateOpenness(const EyeData& eye) {
    std::map<std::string, float> estimates;
    
    // Get estimates from all enabled estimators
    for (auto& pair : m_estimators) {
      if (pair.second.enabled && pair.second.estimator) {
        float estimate = pair.second.estimator->EstimateOpenness(eye);
        estimates[pair.first] = estimate;
        pair.second.confidence = pair.second.estimator->GetConfidence();
      }
    }
    
    // Fuse estimates
    return FuseEstimates(estimates);
  }

  void HybridScientificEstimator::UpdateLearning(const EyeData& eye, float groundTruth) {
    // Update all enabled estimators
    for (auto& pair : m_estimators) {
      if (pair.second.enabled && pair.second.estimator) {
        pair.second.estimator->UpdateLearning(eye, groundTruth);
      }
    }
  }

  bool HybridScientificEstimator::IsCalibrated() const {
    // Consider calibrated if at least one estimator is calibrated
    for (const auto& pair : m_estimators) {
      if (pair.second.enabled && pair.second.estimator && 
          pair.second.estimator->IsCalibrated()) {
        return true;
      }
    }
    return false;
  }

  void HybridScientificEstimator::Reset() {
    for (auto& pair : m_estimators) {
      if (pair.second.estimator) {
        pair.second.estimator->Reset();
      }
    }
  }

  std::string HybridScientificEstimator::GetName() const {
    return "HybridScientific";
  }

  float HybridScientificEstimator::GetConfidence() const {
    // Return average confidence of enabled estimators
    float totalConfidence = 0.0f;
    int count = 0;
    
    for (const auto& pair : m_estimators) {
      if (pair.second.enabled) {
        totalConfidence += pair.second.confidence;
        count++;
      }
    }
    
    return count > 0 ? totalConfidence / count : 0.0f;
  }

  void HybridScientificEstimator::EnableEstimator(const std::string& name, bool enabled) {
    auto it = m_estimators.find(name);
    if (it != m_estimators.end()) {
      it->second.enabled = enabled;
    }
  }

  void HybridScientificEstimator::SetEstimatorWeight(const std::string& name, float weight) {
    auto it = m_estimators.find(name);
    if (it != m_estimators.end()) {
      it->second.weight = std::clamp(weight, 0.0f, 1.0f);
    }
  }

  void HybridScientificEstimator::SetFusionMethod(const std::string& method) {
    if (method == "weighted" || method == "uncertainty" || method == "voting") {
      m_fusionMethod = method;
    }
  }

  float HybridScientificEstimator::FuseEstimates(const std::map<std::string, float>& estimates) {
    if (estimates.empty()) return 0.5f;
    
    if (m_fusionMethod == "weighted") {
      return WeightedFusion(estimates);
    } else if (m_fusionMethod == "uncertainty") {
      return UncertaintyFusion(estimates);
    } else if (m_fusionMethod == "voting") {
      return VotingFusion(estimates);
    }
    
    return WeightedFusion(estimates); // Default
  }

  float HybridScientificEstimator::WeightedFusion(const std::map<std::string, float>& estimates) {
    float weightedSum = 0.0f;
    float totalWeight = 0.0f;
    
    for (const auto& pair : estimates) {
      auto it = m_estimators.find(pair.first);
      if (it != m_estimators.end()) {
        float weight = it->second.weight;
        weightedSum += pair.second * weight;
        totalWeight += weight;
      }
    }
    
    return totalWeight > 0.0f ? weightedSum / totalWeight : 0.5f;
  }

  float HybridScientificEstimator::UncertaintyFusion(const std::map<std::string, float>& estimates) {
    float uncertaintyWeightedSum = 0.0f;
    float totalUncertaintyWeight = 0.0f;
    
    for (const auto& pair : estimates) {
      auto it = m_estimators.find(pair.first);
      if (it != m_estimators.end()) {
        float confidence = it->second.confidence;
        float uncertainty = 1.0f - confidence;
        float weight = uncertainty > 0.0f ? 1.0f / uncertainty : 1.0f;
        
        uncertaintyWeightedSum += pair.second * weight;
        totalUncertaintyWeight += weight;
      }
    }
    
    return totalUncertaintyWeight > 0.0f ? uncertaintyWeightedSum / totalUncertaintyWeight : 0.5f;
  }

  float HybridScientificEstimator::VotingFusion(const std::map<std::string, float>& estimates) {
    // Simple voting - return median estimate
    std::vector<float> values;
    for (const auto& pair : estimates) {
      values.push_back(pair.second);
    }
    
    if (values.empty()) return 0.5f;
    
    std::sort(values.begin(), values.end());
    int n = values.size();
    
    if (n % 2 == 0) {
      return (values[n/2 - 1] + values[n/2]) / 2.0f;
    } else {
      return values[n/2];
    }
  }
  */

  // Scientific Algorithm Manager Implementation
  ScientificAlgorithmManager::ScientificAlgorithmManager() {
    // Only enable the most feasible algorithm
    m_enabledMethods["pmc8018226_model"] = true;
    // m_enabledMethods["pmc6960643_feature"] = false;
    // m_enabledMethods["frontiers2019_deformable"] = false;
    // m_enabledMethods["springer2024_ml"] = false;
    // m_enabledMethods["hybrid_scientific"] = false;
    
    m_primaryMethod = "pmc8018226_model";
  }

  void ScientificAlgorithmManager::EnableAlgorithm(const std::string& name, bool enabled) {
    m_enabledMethods[name] = enabled;
  }

  void ScientificAlgorithmManager::SetAlgorithmWeight(const std::string& name, float weight) {
    // Only PMC8018226 is enabled, weight setting not applicable
    if (name == "pmc8018226_model") {
      // Weight is fixed at 1.0 for single algorithm
    }
  }

  void ScientificAlgorithmManager::SetFusionMethod(const std::string& method) {
    // Only PMC8018226 is enabled, fusion not applicable
  }

  float ScientificAlgorithmManager::EstimateOpenness(const EyeData& eye, const std::string& method) {
    if (method == "pmc8018226_model") {
      return m_pmc8018226_modelBased.EstimateOpenness(eye);
    }
    // if (method == "pmc6960643_feature") {
    //   return m_pmc6960643_featureBased.EstimateOpenness(eye);
    // } else if (method == "frontiers2019_deformable") {
    //   return m_frontiers2019_deformableShape.EstimateOpenness(eye);
    // } else if (method == "springer2024_ml") {
    //   return m_springer2024_mlBased.EstimateOpenness(eye);
    // } else if (method == "hybrid_scientific") {
    //   return m_hybridScientific.EstimateOpenness(eye);
    // }
    
    return m_pmc8018226_modelBased.EstimateOpenness(eye); // Default to model-based
  }

  void ScientificAlgorithmManager::UpdateLearning(const EyeData& eye, float groundTruth) {
    m_pmc8018226_modelBased.UpdateLearning(eye, groundTruth);
    // m_pmc6960643_featureBased.UpdateLearning(eye, groundTruth);
    // m_frontiers2019_deformableShape.UpdateLearning(eye, groundTruth);
    // m_springer2024_mlBased.UpdateLearning(eye, groundTruth);
    // m_hybridScientific.UpdateLearning(eye, groundTruth);
  }

  void ScientificAlgorithmManager::ResetAll() {
    m_pmc8018226_modelBased.Reset();
    // m_pmc6960643_featureBased.Reset();
    // m_frontiers2019_deformableShape.Reset();
    // m_springer2024_mlBased.Reset();
    // m_hybridScientific.Reset();
  }

  std::vector<std::string> ScientificAlgorithmManager::GetAvailableMethods() const {
    return {"pmc8018226_model"};
    // return {"pmc6960643_feature", "frontiers2019_deformable", "pmc8018226_model", "springer2024_ml", "hybrid_scientific"};
  }

  std::map<std::string, float> ScientificAlgorithmManager::GetMethodConfidences() const {
    std::map<std::string, float> confidences;
    confidences["pmc8018226_model"] = m_pmc8018226_modelBased.GetConfidence();
    // confidences["pmc6960643_feature"] = m_pmc6960643_featureBased.GetConfidence();
    // confidences["frontiers2019_deformable"] = m_frontiers2019_deformableShape.GetConfidence();
    // confidences["pmc8018226_model"] = m_pmc8018226_modelBased.GetConfidence();
    // confidences["springer2024_ml"] = m_springer2024_mlBased.GetConfidence();
    // confidences["hybrid_scientific"] = m_hybridScientific.GetConfidence();
    return confidences;
  }

  // Helper functions (simplified implementations)
  // COMMENTED OUT - Only keeping PMC8018226 helper functions
  /*
  float PMC6960643_FeatureBasedEstimator::CalculateConfidence(const EyeFeatureDetector::EyeFeatures& features, float edgeStrength) {
    return (features.irisVisibility + edgeStrength + features.eyelidContour) / 3.0f;
  }

  float Frontiers2019_DeformableShapeEstimator::CalculateShapeConfidence(const EyeShapeModel& model) {
    if (!model.isValid) return 0.0f;
    
    float confidence = 0.0f;
    int validLandmarks = 0;
    
    for (const auto& landmark : model.upperEyelidLandmarks) {
      if (landmark.isVisible) {
        confidence += landmark.confidence;
        validLandmarks++;
      }
    }
    
    return validLandmarks > 0 ? confidence / validLandmarks : 0.0f;
  }

  void Frontiers2019_DeformableShapeEstimator::UpdateShapeModel(EyeShapeModel& model, float groundTruth) {
    // Update shape model parameters based on ground truth
    // Simplified implementation
  }

  float Springer2024_MLBasedEstimator::CalculateMLConfidence(const EyeData& eye) {
    return 0.7f; // Simplified confidence calculation
  }

  void Springer2024_MLBasedEstimator::UpdateMLModels(const EyeData& eye, float groundTruth) {
    // Update ML models with ground truth data
    // Simplified implementation
  }
  */

}
