#include "modern_eyelid_estimator.h"
#include "headset_calibrator.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <cmath>

using namespace psvr2_toolkit;

// Test utilities
struct TestResult {
    std::string testName;
    bool passed;
    std::string message;
    float expectedOpenness;
    float actualOpenness;
    float tolerance;
};

std::vector<TestResult> testResults;

void PrintTestHeader(const std::string& header) {
    std::cout << "\n=================================================\n";
    std::cout << "  " << header << "\n";
    std::cout << "=================================================\n";
}

void PrintTestResult(const TestResult& result) {
    std::cout << (result.passed ? "[PASS]" : "[FAIL]") << " " << result.testName << "\n";
    if (!result.passed) {
        std::cout << "       Expected: " << result.expectedOpenness
                  << " (±" << result.tolerance << ")\n";
        std::cout << "       Actual: " << result.actualOpenness << "\n";
        std::cout << "       Message: " << result.message << "\n";
    }
}

void PrintSummary() {
    int passed = 0;
    int failed = 0;

    for (const auto& result : testResults) {
        if (result.passed) passed++;
        else failed++;
    }

    std::cout << "\n=================================================\n";
    std::cout << "  TEST SUMMARY\n";
    std::cout << "=================================================\n";
    std::cout << "Total Tests: " << testResults.size() << "\n";
    std::cout << "Passed: " << passed << " ("
              << (100.0f * passed / testResults.size()) << "%)\n";
    std::cout << "Failed: " << failed << "\n";
    std::cout << "=================================================\n";
}

// Helper to create synthetic eye data
EyeData CreateEyeData(float pupilDiaMm, float pupilPosY,
                      float gazeX, float gazeY, float gazeZ,
                      bool isBlink = false) {
    EyeData data;
    data.pupilDiaMm = pupilDiaMm;
    data.pupilPosY = pupilPosY;
    data.gazeDir = Vector3(gazeX, gazeY, gazeZ).Normalized();
    data.isBlink = isBlink;
    data.isValid = true;
    return data;
}

// Helper to create synthetic Hmd2GazeEye data
Hmd2GazeEye CreateHmd2GazeEye(float pupilDiaMm, float pupilPosY,
                               float gazeX, float gazeY, float gazeZ,
                               bool isBlink = false) {
    Hmd2GazeEye eye = {};
    eye.pupilDiaMm = pupilDiaMm;
    eye.isPupilDiaValid = HMD2_BOOL_TRUE;

    eye.pupilPosInSensor.x = 0.5f;
    eye.pupilPosInSensor.y = pupilPosY;
    eye.isPupilPosInSensorValid = HMD2_BOOL_TRUE;

    eye.gazeDirNorm.x = gazeX;
    eye.gazeDirNorm.y = gazeY;
    eye.gazeDirNorm.z = gazeZ;
    eye.isGazeDirValid = HMD2_BOOL_TRUE;

    eye.blink = isBlink ? HMD2_BOOL_TRUE : HMD2_BOOL_FALSE;
    eye.isBlinkValid = HMD2_BOOL_TRUE;

    return eye;
}

// Train estimator with baseline data
void TrainEstimator(ModernEyelidEstimator& estimator, int numFrames = 200) {
    std::cout << "Training estimator with " << numFrames << " baseline frames...\n";

    for (int i = 0; i < numFrames; i++) {
        // Simulate normal open eyes with slight variations
        float pupilDia = 3.5f + 0.5f * std::sin(i * 0.1f);
        float pupilPosY = 0.55f + 0.05f * std::cos(i * 0.15f);

        // Mix of gaze directions
        float gazeX = 0.1f * std::sin(i * 0.05f);
        float gazeY = 0.1f * std::cos(i * 0.07f);
        float gazeZ = 0.98f;

        // Occasional blinks (every 30 frames)
        bool blink = (i % 30 == 0);
        if (blink) {
            pupilDia = 2.0f;
            pupilPosY = 0.45f;
        }

        EyeData eye = CreateEyeData(pupilDia, pupilPosY, gazeX, gazeY, gazeZ, blink);
        EstimationResult result = estimator.Estimate(eye);

        // Debug output for first few frames
        if (i < 5 || i == numFrames - 1) {
            std::cout << "  Frame " << i << ": openness=" << result.openness
                      << " (blink=" << blink << ")\n";
        }
    }

    std::cout << "Training complete.\n";
}

// Train headset calibrator
void TrainHeadsetCalibrator(HeadsetCalibrator& calibrator,
                            float cameraTiltDeg = 0.0f,
                            float headsetHeightOffset = 0.0f,
                            int numFrames = 250) {
    std::cout << "Training headset calibrator (tilt=" << cameraTiltDeg
              << "deg, height=" << headsetHeightOffset << "mm)...\n";

    float cameraTiltRad = cameraTiltDeg * 3.14159f / 180.0f;

    for (int i = 0; i < numFrames; i++) {
        // Apply camera tilt to gaze direction
        float baseGazeY = 0.1f * std::sin(i * 0.05f) + cameraTiltRad;
        float baseGazeX = 0.1f * std::cos(i * 0.07f);
        float baseGazeZ = 0.98f;

        float pupilDia = 3.5f + 0.3f * std::sin(i * 0.1f);
        float pupilPosY = 0.55f + headsetHeightOffset + 0.03f * std::cos(i * 0.15f);

        Hmd2GazeEye leftEye = CreateHmd2GazeEye(pupilDia, pupilPosY,
                                                 baseGazeX, baseGazeY, baseGazeZ);
        Hmd2GazeEye rightEye = CreateHmd2GazeEye(pupilDia, pupilPosY,
                                                  baseGazeX, baseGazeY, baseGazeZ);

        calibrator.UpdateCalibration(leftEye, rightEye);
    }

    std::cout << "Headset calibration complete (stable="
              << calibrator.IsCalibrationStable() << ").\n";
}

// Test 1: Basic blink detection
void TestBlinkDetection() {
    PrintTestHeader("TEST 1: Basic Blink Detection");

    ModernEyelidEstimator estimator;
    TrainEstimator(estimator);

    // NOTE: Output is inverted! 0 = open, 1 = closed
    // Test open eyes
    EyeData openEye = CreateEyeData(3.5f, 0.55f, 0.0f, 0.0f, 1.0f, false);
    EstimationResult openResult = estimator.Estimate(openEye);

    testResults.push_back({
        "Open eyes should report low closedness (inverted output)",
        openResult.openness < 0.3f,
        "Eyes should be detected as open (0 = open)",
        0.15f, openResult.openness, 0.15f
    });

    // Test blink
    // NOTE: Blinks bypass inversion and always return 0.0 (closed) by design
    EyeData blinkEye = CreateEyeData(2.0f, 0.45f, 0.0f, 0.0f, 1.0f, true);
    EstimationResult blinkResult = estimator.Estimate(blinkEye);

    testResults.push_back({
        "Blink should report 0.0 (closed, bypasses inversion)",
        blinkResult.openness < 0.1f,
        "Eyes should be detected as closed during blink (0.0 = closed for blinks)",
        0.0f, blinkResult.openness, 0.1f
    });

    PrintTestResult(testResults[testResults.size() - 2]);
    PrintTestResult(testResults[testResults.size() - 1]);
}

// Test 2: Gaze-dependent compensation
void TestGazeDependentCompensation() {
    PrintTestHeader("TEST 2: Gaze-Dependent Compensation");

    ModernEyelidEstimator estimator;
    TrainEstimator(estimator);

    // NOTE: Output is inverted! 0 = open, 1 = closed
    // Test neutral gaze (forward)
    EyeData neutralGaze = CreateEyeData(3.5f, 0.55f, 0.0f, 0.0f, 1.0f);
    EstimationResult neutralResult = estimator.Estimate(neutralGaze);

    // Test upward gaze (pupil appears smaller due to foreshortening)
    EyeData upwardGaze = CreateEyeData(3.0f, 0.53f, 0.0f, 0.4f, 0.9f);
    EstimationResult upwardResult = estimator.Estimate(upwardGaze);

    // Test downward gaze
    EyeData downwardGaze = CreateEyeData(3.0f, 0.52f, 0.0f, -0.4f, 0.9f);
    EstimationResult downwardResult = estimator.Estimate(downwardGaze);

    testResults.push_back({
        "Neutral gaze should report consistent low closedness",
        neutralResult.openness < 0.4f && neutralResult.openness > 0.05f,
        "Neutral gaze closedness should be stable (inverted)",
        0.20f, neutralResult.openness, 0.2f
    });

    testResults.push_back({
        "Upward gaze compensation should maintain closedness",
        std::abs(upwardResult.openness - neutralResult.openness) < 0.3f,
        "Upward gaze should be compensated (inverted)",
        neutralResult.openness, upwardResult.openness, 0.3f
    });

    testResults.push_back({
        "Downward gaze compensation should maintain closedness",
        std::abs(downwardResult.openness - neutralResult.openness) < 0.3f,
        "Downward gaze should be compensated (inverted)",
        neutralResult.openness, downwardResult.openness, 0.3f
    });

    PrintTestResult(testResults[testResults.size() - 3]);
    PrintTestResult(testResults[testResults.size() - 2]);
    PrintTestResult(testResults[testResults.size() - 1]);
}

// Test 3: Eye shape profile adaptation
void TestEyeShapeAdaptation() {
    PrintTestHeader("TEST 3: Eye Shape Profile Adaptation");

    // Test with large pupils
    ModernEyelidEstimator largePupilEstimator;
    std::cout << "Training with large pupil sizes (4-6mm)...\n";
    for (int i = 0; i < 100; i++) {
        float pupilDia = (i % 20 == 0) ? 4.0f : 6.0f;  // Blink vs open
        float pupilPosY = (i % 20 == 0) ? 0.45f : 0.60f;
        bool blink = (i % 20 == 0);

        EyeData eye = CreateEyeData(pupilDia, pupilPosY, 0.0f, 0.0f, 1.0f, blink);
        largePupilEstimator.Estimate(eye);
    }

    // Test with small pupils
    ModernEyelidEstimator smallPupilEstimator;
    std::cout << "Training with small pupil sizes (2-3.5mm)...\n";
    for (int i = 0; i < 100; i++) {
        float pupilDia = (i % 20 == 0) ? 2.0f : 3.5f;  // Blink vs open
        float pupilPosY = (i % 20 == 0) ? 0.45f : 0.55f;
        bool blink = (i % 20 == 0);

        EyeData eye = CreateEyeData(pupilDia, pupilPosY, 0.0f, 0.0f, 1.0f, blink);
        smallPupilEstimator.Estimate(eye);
    }

    // Both should report similar openness for open eyes
    EyeData largePupilOpen = CreateEyeData(6.0f, 0.60f, 0.0f, 0.0f, 1.0f);
    EstimationResult largeResult = largePupilEstimator.Estimate(largePupilOpen);

    EyeData smallPupilOpen = CreateEyeData(3.5f, 0.55f, 0.0f, 0.0f, 1.0f);
    EstimationResult smallResult = smallPupilEstimator.Estimate(smallPupilOpen);

    // NOTE: Output is inverted! 0 = open, 1 = closed
    testResults.push_back({
        "Large pupil eyes should adapt to report low closedness",
        largeResult.openness < 0.4f,
        "Adaptation should work for large pupils (inverted)",
        0.20f, largeResult.openness, 0.2f
    });

    testResults.push_back({
        "Small pupil eyes should adapt to report low closedness",
        smallResult.openness < 0.4f,
        "Adaptation should work for small pupils (inverted)",
        0.20f, smallResult.openness, 0.2f
    });

    testResults.push_back({
        "Different pupil sizes should converge to similar closedness",
        std::abs(largeResult.openness - smallResult.openness) < 0.25f,
        "Adaptive profiling should normalize different eye shapes (inverted)",
        largeResult.openness, smallResult.openness, 0.25f
    });

    PrintTestResult(testResults[testResults.size() - 3]);
    PrintTestResult(testResults[testResults.size() - 2]);
    PrintTestResult(testResults[testResults.size() - 1]);
}

// Test 4: Headset mounting position compensation
void TestHeadsetMountingCompensation() {
    PrintTestHeader("TEST 4: Headset Mounting Position Compensation");

    // Test with headset mounted high
    ModernEyelidEstimator highMountEstimator;
    HeadsetCalibrator highCalibrator;
    TrainHeadsetCalibrator(highCalibrator, 5.0f, 0.1f);  // 5deg tilt, +0.1 height
    TrainEstimator(highMountEstimator);

    // Test with headset mounted low
    ModernEyelidEstimator lowMountEstimator;
    HeadsetCalibrator lowCalibrator;
    TrainHeadsetCalibrator(lowCalibrator, -5.0f, -0.1f);  // -5deg tilt, -0.1 height
    TrainEstimator(lowMountEstimator);

    // Test with neutral mounting
    ModernEyelidEstimator neutralEstimator;
    HeadsetCalibrator neutralCalibrator;
    TrainHeadsetCalibrator(neutralCalibrator, 0.0f, 0.0f);
    TrainEstimator(neutralEstimator);

    // Create test eye with calibrated data
    Hmd2GazeEye rawEye = CreateHmd2GazeEye(3.5f, 0.55f, 0.0f, 0.0f, 1.0f);

    CalibratedEyeData highCalibrated = highCalibrator.CalibrateEyeData(rawEye);
    EyeData highEye = CreateEyeData(
        highCalibrated.compensatedPupilDia,
        highCalibrated.compensatedPupilPos.y,
        highCalibrated.compensatedGazeDir.x,
        highCalibrated.compensatedGazeDir.y,
        highCalibrated.compensatedGazeDir.z
    );
    EstimationResult highResult = highMountEstimator.Estimate(highEye);

    CalibratedEyeData lowCalibrated = lowCalibrator.CalibrateEyeData(rawEye);
    EyeData lowEye = CreateEyeData(
        lowCalibrated.compensatedPupilDia,
        lowCalibrated.compensatedPupilPos.y,
        lowCalibrated.compensatedGazeDir.x,
        lowCalibrated.compensatedGazeDir.y,
        lowCalibrated.compensatedGazeDir.z
    );
    EstimationResult lowResult = lowMountEstimator.Estimate(lowEye);

    CalibratedEyeData neutralCalibrated = neutralCalibrator.CalibrateEyeData(rawEye);
    EyeData neutralEye = CreateEyeData(
        neutralCalibrated.compensatedPupilDia,
        neutralCalibrated.compensatedPupilPos.y,
        neutralCalibrated.compensatedGazeDir.x,
        neutralCalibrated.compensatedGazeDir.y,
        neutralCalibrated.compensatedGazeDir.z
    );
    EstimationResult neutralResult = neutralEstimator.Estimate(neutralEye);

    // NOTE: Output is inverted! 0 = open, 1 = closed
    testResults.push_back({
        "High-mounted headset should report reasonable closedness",
        highResult.openness > 0.05f && highResult.openness < 0.5f,
        "High mounting compensation should work (inverted)",
        0.25f, highResult.openness, 0.25f
    });

    testResults.push_back({
        "Low-mounted headset should report reasonable closedness",
        lowResult.openness > 0.05f && lowResult.openness < 0.5f,
        "Low mounting compensation should work (inverted)",
        0.25f, lowResult.openness, 0.25f
    });

    testResults.push_back({
        "Different mounting positions should converge after calibration",
        std::abs(highResult.openness - neutralResult.openness) < 0.3f &&
        std::abs(lowResult.openness - neutralResult.openness) < 0.3f,
        "Headset calibration should normalize mounting differences (inverted)",
        neutralResult.openness, (highResult.openness + lowResult.openness) / 2.0f, 0.3f
    });

    PrintTestResult(testResults[testResults.size() - 3]);
    PrintTestResult(testResults[testResults.size() - 2]);
    PrintTestResult(testResults[testResults.size() - 1]);
}

// Test 5: Fast re-learning on headset adjustment
void TestFastRelearning() {
    PrintTestHeader("TEST 5: Fast Re-Learning After Headset Adjustment");

    ModernEyelidEstimator estimator;
    TrainEstimator(estimator, 200);

    // Get baseline openness
    EyeData baselineEye = CreateEyeData(3.5f, 0.55f, 0.0f, 0.0f, 1.0f);
    EstimationResult baselineResult = estimator.Estimate(baselineEye);
    float baselineOpenness = baselineResult.openness;

    std::cout << "Baseline openness: " << baselineOpenness << "\n";

    // Simulate headset adjustment (sudden change in pupil position)
    std::cout << "Simulating headset adjustment...\n";
    for (int i = 0; i < 150; i++) {
        // New pupil position after adjustment
        float newPupilPosY = 0.48f + 0.03f * std::cos(i * 0.15f);  // Shifted down
        float pupilDia = 3.5f + 0.3f * std::sin(i * 0.1f);

        EyeData adjustedEye = CreateEyeData(pupilDia, newPupilPosY, 0.0f, 0.0f, 1.0f);
        estimator.Estimate(adjustedEye);
    }

    // Test new openness after re-learning
    EyeData newEye = CreateEyeData(3.5f, 0.48f, 0.0f, 0.0f, 1.0f);
    EstimationResult newResult = estimator.Estimate(newEye);

    std::cout << "After adjustment openness: " << newResult.openness << "\n";

    testResults.push_back({
        "Fast re-learning should adapt to new baseline within 150 frames",
        std::abs(newResult.openness - baselineOpenness) < 0.3f,
        "Re-learning should converge to similar openness",
        baselineOpenness, newResult.openness, 0.3f
    });

    PrintTestResult(testResults[testResults.size() - 1]);
}

// Test 6: Saccade sequence
void TestSaccadeSequence() {
    PrintTestHeader("TEST 6: Saccade Sequence (Realistic Eye Movements)");

    ModernEyelidEstimator estimator;
    TrainEstimator(estimator, 200);

    struct SaccadeTarget {
        std::string name;
        Vector3 gazeDir;
        float expectedOpennessMin;
        float expectedOpennessMax;
    };

    // NOTE: Output is inverted! 0 = open, 1 = closed
    // Values are now "closedness" not "openness"
    std::vector<SaccadeTarget> saccades = {
        {"Center", Vector3(0.0f, 0.0f, 1.0f), 0.05f, 0.4f},  // Low closedness = open
        {"Left", Vector3(-0.5f, 0.0f, 0.87f), 0.1f, 0.45f},
        {"Right", Vector3(0.5f, 0.0f, 0.87f), 0.1f, 0.45f},
        {"Up", Vector3(0.0f, 0.5f, 0.87f), 0.15f, 0.5f},
        {"Down", Vector3(0.0f, -0.5f, 0.87f), 0.15f, 0.5f},
        {"Up-Left", Vector3(-0.35f, 0.35f, 0.87f), 0.15f, 0.5f},
        {"Up-Right", Vector3(0.35f, 0.35f, 0.87f), 0.15f, 0.5f},
        {"Down-Left", Vector3(-0.35f, -0.35f, 0.87f), 0.15f, 0.5f},
        {"Down-Right", Vector3(0.35f, -0.35f, 0.87f), 0.15f, 0.5f},
    };

    for (const auto& saccade : saccades) {
        EyeData eye = CreateEyeData(3.5f, 0.55f,
                                     saccade.gazeDir.x,
                                     saccade.gazeDir.y,
                                     saccade.gazeDir.z);
        EstimationResult result = estimator.Estimate(eye);

        bool passed = result.openness >= saccade.expectedOpennessMin &&
                     result.openness <= saccade.expectedOpennessMax;

        testResults.push_back({
            "Saccade to " + saccade.name,
            passed,
            "Openness should remain stable during saccade",
            (saccade.expectedOpennessMin + saccade.expectedOpennessMax) / 2.0f,
            result.openness,
            (saccade.expectedOpennessMax - saccade.expectedOpennessMin) / 2.0f
        });

        std::cout << "  " << saccade.name << ": openness="
                  << std::fixed << std::setprecision(3) << result.openness
                  << " " << (passed ? "[PASS]" : "[FAIL]") << "\n";
    }
}

int main() {
    std::cout << "\n";
    std::cout << "###################################################\n";
    std::cout << "#  EYELID ESTIMATION CALIBRATION TEST SUITE      #\n";
    std::cout << "###################################################\n";

    TestBlinkDetection();
    TestGazeDependentCompensation();
    TestEyeShapeAdaptation();
    TestHeadsetMountingCompensation();
    TestFastRelearning();
    TestSaccadeSequence();

    PrintSummary();

    return (testResults.size() > 0 &&
            std::all_of(testResults.begin(), testResults.end(),
                       [](const TestResult& r) { return r.passed; })) ? 0 : 1;
}
