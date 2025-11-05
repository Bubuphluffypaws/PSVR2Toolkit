#include "modern_eyelid_estimator.h"
#include <iostream>
#include <iomanip>

using namespace psvr2_toolkit;

EyeData CreateEye(float dia, float posY, float gX, float gY, float gZ, bool blink = false) {
    EyeData e;
    e.pupilDiaMm = dia;
    e.pupilPosY = posY;
    e.gazeDir = Vector3(gX, gY, gZ).Normalized();
    e.isBlink = blink;
    e.isValid = true;
    return e;
}

void WarmupEstimator(ModernEyelidEstimator& estimator, const std::string& scenario) {
    std::cout << "  Warmup phase (" << scenario << "): 50 frames...\n";

    // Simulate varied mounting positions during warmup
    float heightBias = 0.0f;
    float tiltBias = 0.0f;

    if (scenario == "high") {
        heightBias = 0.05f;  // Headset mounted higher
        tiltBias = 0.1f;
    } else if (scenario == "low") {
        heightBias = -0.05f;  // Headset mounted lower
        tiltBias = -0.1f;
    } else if (scenario == "loose") {
        heightBias = 0.03f;
        tiltBias = 0.15f;
    }

    for (int i = 0; i < 50; i++) {
        bool blink = (i % 25 == 0);
        float dia = blink ? 2.0f : 3.5f + 0.2f * std::sin(i * 0.08f);
        float posY = (blink ? 0.45f : 0.55f) + heightBias;
        float gazeY = tiltBias + 0.05f * std::sin(i * 0.05f);

        estimator.Estimate(CreateEye(dia, posY, 0, gazeY, 0.98f, blink));
    }
}

int main() {
    std::cout << "DEBUG: Checking if estimator produces non-zero values\n\n";
    std::cout << std::fixed << std::setprecision(4);

    ModernEyelidEstimator estimator;

    // Warmup phase
    WarmupEstimator(estimator, "normal");

    std::cout << "Training with 150 frames...\n";
    for (int i = 0; i < 150; i++) {
        bool blink = (i % 30 == 0);
        float dia = blink ? 2.0f : 3.5f + 0.3f * std::sin(i * 0.1f);
        float posY = blink ? 0.45f : 0.55f;

        auto result = estimator.Estimate(CreateEye(dia, posY, 0, 0, 1, blink));

        // Show detailed output for frames around where it gets stuck (frames 70-90)
        if ((i >= 70 && i <= 90) || i % 20 == 0) {
            std::cout << "  Frame " << std::setw(3) << i
                      << ": dia=" << std::setw(6) << dia
                      << ", posY=" << std::setw(6) << posY
                      << ", blink=" << blink
                      << " => openness=" << std::setw(7) << result.openness
                      << ", confidence=" << std::setw(6) << result.confidence
                      << ", primaryCue=" << result.primaryCue
                      << "\n";
        }
    }

    std::cout << "\nTesting final state with open eyes:\n";
    for (int i = 0; i < 5; i++) {
        auto result = estimator.Estimate(CreateEye(3.5f, 0.55f, 0, 0, 1));
        std::cout << "  Test " << i << ": openness=" << result.openness
                  << ", primaryCue=" << result.primaryCue
                  << ", confidence=" << result.confidence << "\n";
    }

    std::cout << "\nTesting with slightly different values:\n";
    auto r1 = estimator.Estimate(CreateEye(3.8f, 0.57f, 0, 0, 1));
    std::cout << "  Wide open (3.8mm, 0.57): " << r1.openness << "\n";

    auto r2 = estimator.Estimate(CreateEye(3.0f, 0.52f, 0, 0, 1));
    std::cout << "  Slightly closed (3.0mm, 0.52): " << r2.openness << "\n";

    auto r3 = estimator.Estimate(CreateEye(2.5f, 0.48f, 0, 0, 1));
    std::cout << "  More closed (2.5mm, 0.48): " << r3.openness << "\n";

    auto r4 = estimator.Estimate(CreateEye(2.0f, 0.45f, 0, 0, 1, true));
    std::cout << "  Blink (2.0mm, 0.45): " << r4.openness << "\n";

    bool allZero = (r1.openness < 0.001f && r2.openness < 0.001f && r3.openness < 0.001f);

    if (allZero) {
        std::cout << "\n✗ STUCK AT ZERO DETECTED!\n";
        return 1;
    } else {
        std::cout << "\n✓ Producing non-zero values\n";
        return 0;
    }
}
