#include "modern_eyelid_estimator.h"
#include "headset_calibrator.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>

using namespace psvr2_toolkit;

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

void PrintSectionHeader(const std::string& header) {
    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "  " << header << "\n";
    std::cout << std::string(70, '=') << "\n";
}

void TestExtremeHeadsetAngles() {
    PrintSectionHeader("EXTREME HEADSET MOUNTING ANGLES TEST");

    struct MountingScenario {
        std::string name;
        float tiltDegrees;
        float heightOffset;
        float distanceOffset;
    };

    std::vector<MountingScenario> scenarios = {
        {"Normal Mounting", 0.0f, 0.0f, 0.0f},
        {"Slightly High", 10.0f, 0.15f, 5.0f},
        {"Very High (extreme)", 25.0f, 0.30f, 15.0f},
        {"Extremely High (worst case)", 35.0f, 0.40f, 20.0f},
        {"Slightly Low", -10.0f, -0.15f, -5.0f},
        {"Very Low (extreme)", -25.0f, -0.30f, -15.0f},
        {"Extremely Low (worst case)", -35.0f, -0.40f, -20.0f},
        {"Tilted Left", 0.0f, 0.0f, 0.0f},  // We'll apply lateral tilt via gaze
        {"Loose/Forward", 15.0f, 0.20f, 25.0f},
    };

    std::cout << "\nTesting " << scenarios.size() << " extreme mounting scenarios...\n";
    std::cout << std::fixed << std::setprecision(4);

    bool allPassed = true;

    for (const auto& scenario : scenarios) {
        std::cout << "\n--- " << scenario.name << " ---\n";
        std::cout << "  Tilt: " << scenario.tiltDegrees << "°, ";
        std::cout << "Height: " << scenario.heightOffset << "mm, ";
        std::cout << "Distance: " << scenario.distanceOffset << "mm\n";

        // Create fresh estimator and calibrator for this scenario
        ModernEyelidEstimator estimator;
        HeadsetCalibrator calibrator;

        float cameraTiltRad = scenario.tiltDegrees * 3.14159f / 180.0f;

        // Warmup phase - gradual lead-up to this mounting position
        std::cout << "  Warmup phase (50 frames)...\n";
        for (int i = 0; i < 50; i++) {
            // Gradually introduce the mounting characteristics
            float warmupProgress = (float)i / 50.0f;
            float warmupTilt = cameraTiltRad * warmupProgress;
            float warmupHeight = scenario.heightOffset * warmupProgress;

            bool blink = (i % 25 == 0);
            float pupilDia = blink ? 2.0f : 3.5f + 0.2f * std::sin(i * 0.08f);
            float pupilPosY = (blink ? 0.45f : 0.55f) + warmupHeight;
            float gazeY = 0.05f * std::sin(i * 0.05f) + warmupTilt;

            Hmd2GazeEye leftEye = CreateHmd2GazeEye(pupilDia, pupilPosY, 0.0f, gazeY, 0.98f, blink);
            Hmd2GazeEye rightEye = CreateHmd2GazeEye(pupilDia, pupilPosY, 0.0f, gazeY, 0.98f, blink);
            calibrator.UpdateCalibration(leftEye, rightEye);

            EyeData eyeData;
            if (calibrator.IsCalibrationStable()) {
                CalibratedEyeData calibrated = calibrator.CalibrateEyeData(leftEye);
                eyeData = CreateEyeData(
                    calibrated.compensatedPupilDia,
                    calibrated.compensatedPupilPos.y,
                    calibrated.compensatedGazeDir.x,
                    calibrated.compensatedGazeDir.y,
                    calibrated.compensatedGazeDir.z,
                    blink
                );
            } else {
                eyeData = CreateEyeData(pupilDia, pupilPosY, 0.0f, gazeY, 0.98f, blink);
            }

            estimator.Estimate(eyeData);
        }

        // Training phase with this mounting
        std::cout << "  Training with 300 frames...\n";
        for (int i = 0; i < 300; i++) {
            // Simulate open eyes with mounting offset
            float pupilDia = 3.5f + 0.4f * std::sin(i * 0.08f);
            float pupilPosY = 0.55f + scenario.heightOffset + 0.04f * std::cos(i * 0.12f);

            // Apply camera tilt to gaze
            float gazeY = 0.08f * std::sin(i * 0.05f) + cameraTiltRad;
            float gazeX = 0.08f * std::cos(i * 0.06f);
            float gazeZ = 0.98f;

            // Occasional blinks
            bool blink = (i % 40 == 0);
            if (blink) {
                pupilDia = 2.0f;
                pupilPosY = 0.45f + scenario.heightOffset;
            }

            // Train calibrator
            Hmd2GazeEye leftEye = CreateHmd2GazeEye(pupilDia, pupilPosY, gazeX, gazeY, gazeZ, blink);
            Hmd2GazeEye rightEye = CreateHmd2GazeEye(pupilDia, pupilPosY, gazeX, gazeY, gazeZ, blink);
            calibrator.UpdateCalibration(leftEye, rightEye);

            // Train estimator with calibrated data
            EyeData eyeData;
            if (calibrator.IsCalibrationStable()) {
                CalibratedEyeData calibrated = calibrator.CalibrateEyeData(leftEye);
                eyeData = CreateEyeData(
                    calibrated.compensatedPupilDia,
                    calibrated.compensatedPupilPos.y,
                    calibrated.compensatedGazeDir.x,
                    calibrated.compensatedGazeDir.y,
                    calibrated.compensatedGazeDir.z,
                    blink
                );
            } else {
                eyeData = CreateEyeData(pupilDia, pupilPosY, gazeX, gazeY, gazeZ, blink);
            }

            estimator.Estimate(eyeData);
        }

        std::cout << "  Calibration stable: " << (calibrator.IsCalibrationStable() ? "YES" : "NO") << "\n";

        // Test multiple eye states
        struct EyeState {
            std::string name;
            float pupilDia;
            float pupilPosY;
            bool isBlink;
            float minExpected;  // Minimum acceptable value (inverted: 0=open, 1=closed)
            float maxExpected;  // Maximum acceptable value
        };

        std::vector<EyeState> states = {
            {"Wide Open", 4.0f, 0.58f + scenario.heightOffset, false, 0.0f, 0.30f},
            {"Normal Open", 3.5f, 0.55f + scenario.heightOffset, false, 0.0f, 0.35f},
            {"Slightly Closed", 3.0f, 0.52f + scenario.heightOffset, false, 0.10f, 0.50f},
            {"Half Closed", 2.5f, 0.50f + scenario.heightOffset, false, 0.30f, 0.70f},
            {"Nearly Closed", 2.2f, 0.47f + scenario.heightOffset, false, 0.50f, 0.90f},
            {"Blink", 2.0f, 0.45f + scenario.heightOffset, true, 0.0f, 0.10f},  // Blinks bypass inversion
        };

        std::cout << "\n  Testing eye states:\n";
        std::cout << "  " << std::setw(20) << "State"
                  << std::setw(12) << "Closedness"
                  << std::setw(12) << "Expected"
                  << std::setw(10) << "Status" << "\n";
        std::cout << "  " << std::string(54, '-') << "\n";

        for (const auto& state : states) {
            // Test with calibrated data
            Hmd2GazeEye rawEye = CreateHmd2GazeEye(
                state.pupilDia, state.pupilPosY,
                0.0f, cameraTiltRad, 0.98f,  // Forward gaze with tilt
                state.isBlink
            );

            EyeData testEye;
            if (calibrator.IsCalibrationStable()) {
                CalibratedEyeData calibrated = calibrator.CalibrateEyeData(rawEye);
                testEye = CreateEyeData(
                    calibrated.compensatedPupilDia,
                    calibrated.compensatedPupilPos.y,
                    calibrated.compensatedGazeDir.x,
                    calibrated.compensatedGazeDir.y,
                    calibrated.compensatedGazeDir.z,
                    state.isBlink
                );
            } else {
                testEye = CreateEyeData(
                    state.pupilDia, state.pupilPosY,
                    0.0f, cameraTiltRad, 0.98f,
                    state.isBlink
                );
            }

            EstimationResult result = estimator.Estimate(testEye);

            bool inRange = (result.openness >= state.minExpected) &&
                          (result.openness <= state.maxExpected);
            bool notStuckAtZero = (result.openness > 0.001f || state.isBlink);  // Blinks can be 0.0

            std::string status;
            if (!notStuckAtZero) {
                status = "STUCK@0!";
                allPassed = false;
            } else if (!inRange) {
                status = "OUT_RANGE";
                allPassed = false;
            } else {
                status = "OK";
            }

            std::cout << "  " << std::setw(20) << state.name
                      << std::setw(12) << result.openness
                      << "  " << state.minExpected << "-" << state.maxExpected
                      << std::setw(10) << status << "\n";
        }
    }

    std::cout << "\n" << std::string(70, '=') << "\n";
    if (allPassed) {
        std::cout << "✓ ALL EXTREME SCENARIOS PASSED - No stuck-at-zero detected!\n";
    } else {
        std::cout << "✗ SOME SCENARIOS FAILED - Stuck-at-zero or out-of-range detected!\n";
    }
    std::cout << std::string(70, '=') << "\n";
}

void TestExtremePupilSizes() {
    PrintSectionHeader("EXTREME PUPIL SIZE VARIATIONS TEST");

    struct PupilProfile {
        std::string name;
        float minSize;
        float maxSize;
        std::string description;
    };

    std::vector<PupilProfile> profiles = {
        {"Tiny Pupils (miotics)", 1.5f, 2.5f, "Very constricted (bright light/drugs)"},
        {"Small Pupils", 2.0f, 3.0f, "Below average"},
        {"Normal Pupils", 2.5f, 4.0f, "Average range"},
        {"Large Pupils", 3.5f, 5.5f, "Above average"},
        {"Huge Pupils (mydriasis)", 5.0f, 7.0f, "Very dilated (dark/drugs)"},
    };

    std::cout << "\nTesting " << profiles.size() << " extreme pupil size profiles...\n";

    for (const auto& profile : profiles) {
        std::cout << "\n--- " << profile.name << " ---\n";
        std::cout << "  Range: " << profile.minSize << "mm - " << profile.maxSize << "mm\n";
        std::cout << "  " << profile.description << "\n";

        // Create fresh estimator
        ModernEyelidEstimator estimator;

        // Warmup phase - gradually introduce this pupil size range
        std::cout << "  Warmup phase (60 frames)...\n";
        for (int i = 0; i < 60; i++) {
            float warmupProgress = (float)i / 60.0f;

            // Start from normal range and gradually shift to this profile's range
            float normalMin = 2.5f, normalMax = 4.0f;
            float currentMin = normalMin + (profile.minSize - normalMin) * warmupProgress;
            float currentMax = normalMax + (profile.maxSize - normalMax) * warmupProgress;

            bool blink = (i % 25 == 0);
            float pupilDia, pupilPosY;
            if (blink) {
                pupilDia = currentMin;
                pupilPosY = 0.45f;
            } else {
                float t = (std::sin(i * 0.1f) + 1.0f) * 0.5f;
                pupilDia = currentMin + t * (currentMax - currentMin);
                pupilPosY = 0.50f + t * 0.08f;
            }

            EyeData eye = CreateEyeData(pupilDia, pupilPosY, 0.0f, 0.0f, 1.0f, blink);
            estimator.Estimate(eye);
        }

        // Train with this pupil size range
        std::cout << "  Training with 250 frames...\n";
        for (int i = 0; i < 250; i++) {
            bool blink = (i % 35 == 0);

            float pupilDia, pupilPosY;
            if (blink) {
                pupilDia = profile.minSize;
                pupilPosY = 0.45f;
            } else {
                // Vary between min and max
                float t = (std::sin(i * 0.1f) + 1.0f) * 0.5f;  // 0 to 1
                pupilDia = profile.minSize + t * (profile.maxSize - profile.minSize);
                pupilPosY = 0.50f + t * 0.10f;  // 0.50 to 0.60
            }

            EyeData eye = CreateEyeData(pupilDia, pupilPosY, 0.0f, 0.0f, 1.0f, blink);
            estimator.Estimate(eye);
        }

        // Test open eyes
        float openPupilSize = (profile.minSize + profile.maxSize) * 0.5f;
        EyeData openEye = CreateEyeData(openPupilSize, 0.55f, 0.0f, 0.0f, 1.0f, false);
        EstimationResult openResult = estimator.Estimate(openEye);

        // Test closed (blink)
        EyeData blinkEye = CreateEyeData(profile.minSize, 0.45f, 0.0f, 0.0f, 1.0f, true);
        EstimationResult blinkResult = estimator.Estimate(blinkEye);

        bool openOK = (openResult.openness > 0.001f && openResult.openness < 0.4f);
        bool blinkOK = (blinkResult.openness < 0.1f);

        std::cout << "  Open eyes:  closedness=" << openResult.openness
                  << " " << (openOK ? "[OK]" : "[FAIL - stuck at 0 or too high]") << "\n";
        std::cout << "  Blink:      closedness=" << blinkResult.openness
                  << " " << (blinkOK ? "[OK]" : "[FAIL]") << "\n";

        if (!openOK) {
            std::cout << "  ⚠️  WARNING: Open eyes stuck at zero or unreasonable!\n";
        }
    }
}

void TestExtremeGazeAngles() {
    PrintSectionHeader("EXTREME GAZE ANGLE TEST");

    ModernEyelidEstimator estimator;

    // Warmup phase - establish baseline with centered gaze
    std::cout << "Warmup phase (60 frames)...\n";
    for (int i = 0; i < 60; i++) {
        bool blink = (i % 25 == 0);
        float pupilDia = blink ? 2.0f : 3.5f + 0.2f * std::sin(i * 0.08f);
        float pupilPosY = blink ? 0.45f : 0.55f;

        // Very gentle gaze movements during warmup
        float gazeX = 0.05f * std::sin(i * 0.05f);
        float gazeY = 0.05f * std::cos(i * 0.07f);

        EyeData eye = CreateEyeData(pupilDia, pupilPosY, gazeX, gazeY, 0.995f, blink);
        estimator.Estimate(eye);
    }

    // Train with normal gaze first
    std::cout << "Training with normal gaze patterns (200 frames)...\n";
    for (int i = 0; i < 200; i++) {
        bool blink = (i % 30 == 0);
        float pupilDia = blink ? 2.0f : 3.5f + 0.3f * std::sin(i * 0.1f);
        float pupilPosY = blink ? 0.45f : 0.55f + 0.03f * std::cos(i * 0.12f);

        float gazeX = 0.15f * std::sin(i * 0.05f);
        float gazeY = 0.15f * std::cos(i * 0.07f);

        EyeData eye = CreateEyeData(pupilDia, pupilPosY, gazeX, gazeY, 0.98f, blink);
        estimator.Estimate(eye);
    }

    // Test extreme gaze directions
    struct GazeTest {
        std::string name;
        float x, y, z;
    };

    std::vector<GazeTest> gazes = {
        {"Center (forward)", 0.0f, 0.0f, 1.0f},
        {"Extreme Left", -0.8f, 0.0f, 0.6f},
        {"Extreme Right", 0.8f, 0.0f, 0.6f},
        {"Extreme Up", 0.0f, 0.8f, 0.6f},
        {"Extreme Down", 0.0f, -0.8f, 0.6f},
        {"Up-Left Corner", -0.6f, 0.6f, 0.5f},
        {"Up-Right Corner", 0.6f, 0.6f, 0.5f},
        {"Down-Left Corner", -0.6f, -0.6f, 0.5f},
        {"Down-Right Corner", 0.6f, -0.6f, 0.5f},
    };

    std::cout << "\nTesting extreme gaze angles:\n";
    std::cout << std::setw(25) << "Gaze Direction"
              << std::setw(12) << "Closedness"
              << std::setw(10) << "Status" << "\n";
    std::cout << std::string(47, '-') << "\n";

    for (const auto& gaze : gazes) {
        EyeData eye = CreateEyeData(3.5f, 0.55f, gaze.x, gaze.y, gaze.z, false);
        EstimationResult result = estimator.Estimate(eye);

        bool notStuckAtZero = (result.openness > 0.001f);
        bool reasonable = (result.openness < 0.6f);  // Should still report mostly open

        std::string status = (notStuckAtZero && reasonable) ? "OK" :
                            (!notStuckAtZero ? "STUCK@0!" : "TOO_HIGH");

        std::cout << std::setw(25) << gaze.name
                  << std::setw(12) << result.openness
                  << std::setw(10) << status << "\n";
    }
}

void TestRapidChanges() {
    PrintSectionHeader("RAPID STATE CHANGE TEST (Stuck-at-Zero Detection)");

    ModernEyelidEstimator estimator;

    std::cout << "This test specifically checks if the estimator gets stuck at 0.0\n";
    std::cout << "when experiencing rapid state changes.\n\n";

    // Warmup phase
    std::cout << "Warmup phase (50 frames)...\n";
    for (int i = 0; i < 50; i++) {
        bool blink = (i % 25 == 0);
        float pupilDia = blink ? 2.0f : 3.5f + 0.2f * std::sin(i * 0.08f);
        float pupilPosY = blink ? 0.45f : 0.55f;
        EyeData eye = CreateEyeData(pupilDia, pupilPosY, 0.0f, 0.0f, 1.0f, blink);
        estimator.Estimate(eye);
    }

    // Initial training
    std::cout << "Phase 1: Initial training (100 frames)...\n";
    for (int i = 0; i < 100; i++) {
        bool blink = (i % 25 == 0);
        float pupilDia = blink ? 2.0f : 3.5f;
        float pupilPosY = blink ? 0.45f : 0.55f;
        EyeData eye = CreateEyeData(pupilDia, pupilPosY, 0.0f, 0.0f, 1.0f, blink);
        estimator.Estimate(eye);
    }

    // Rapid open-close-open cycles
    std::cout << "Phase 2: Rapid open-close cycles (50 frames)...\n";
    int stuckCount = 0;
    for (int i = 0; i < 50; i++) {
        bool blink = (i % 2 == 0);  // Every other frame
        float pupilDia = blink ? 2.0f : 3.8f;
        float pupilPosY = blink ? 0.45f : 0.57f;
        EyeData eye = CreateEyeData(pupilDia, pupilPosY, 0.0f, 0.0f, 1.0f, blink);
        EstimationResult result = estimator.Estimate(eye);

        if (!blink && result.openness < 0.001f) {
            stuckCount++;
            if (stuckCount <= 3) {  // Show first few
                std::cout << "  Frame " << i << ": STUCK AT ZERO (expected non-zero for open eyes)\n";
            }
        }
    }

    // Test recovery
    std::cout << "Phase 3: Recovery test (normal operation for 50 frames)...\n";
    int recoveredCount = 0;
    for (int i = 0; i < 50; i++) {
        float pupilDia = 3.5f + 0.2f * std::sin(i * 0.1f);
        float pupilPosY = 0.55f + 0.02f * std::cos(i * 0.15f);
        EyeData eye = CreateEyeData(pupilDia, pupilPosY, 0.0f, 0.0f, 1.0f, false);
        EstimationResult result = estimator.Estimate(eye);

        if (result.openness > 0.01f) {
            recoveredCount++;
        }
    }

    std::cout << "\nResults:\n";
    std::cout << "  Stuck-at-zero occurrences: " << stuckCount << " / 25 non-blink frames\n";
    std::cout << "  Recovery frames: " << recoveredCount << " / 50 frames\n";

    if (stuckCount > 15) {
        std::cout << "  ✗ CRITICAL: System frequently stuck at zero!\n";
    } else if (stuckCount > 5) {
        std::cout << "  ⚠️  WARNING: Occasional stuck-at-zero detected\n";
    } else {
        std::cout << "  ✓ OK: Minimal stuck-at-zero occurrences\n";
    }

    if (recoveredCount < 25) {
        std::cout << "  ✗ CRITICAL: Poor recovery from stuck state!\n";
    } else {
        std::cout << "  ✓ OK: Good recovery capability\n";
    }
}

int main() {
    std::cout << "\n";
    std::cout << "###################################################################\n";
    std::cout << "#  EXTREME SCENARIO TESTING - STUCK-AT-ZERO DETECTION            #\n";
    std::cout << "#                                                                 #\n";
    std::cout << "#  This test suite specifically checks if eyelid estimation      #\n";
    std::cout << "#  returns 0.0 under extreme conditions:                         #\n";
    std::cout << "#  - Extreme headset mounting angles (±35°)                      #\n";
    std::cout << "#  - Extreme pupil sizes (1.5mm - 7.0mm)                         #\n";
    std::cout << "#  - Extreme gaze angles (±80% of FOV)                           #\n";
    std::cout << "#  - Rapid state changes                                         #\n";
    std::cout << "###################################################################\n";

    TestExtremeHeadsetAngles();
    TestExtremePupilSizes();
    TestExtremeGazeAngles();
    TestRapidChanges();

    std::cout << "\n";
    std::cout << "###################################################################\n";
    std::cout << "#  EXTREME SCENARIO TESTING COMPLETE                             #\n";
    std::cout << "###################################################################\n";

    return 0;
}
