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

int main() {
    std::cout << "ADAPTIVE RESPONSE TEST\n";
    std::cout << "Testing if system adapts to different eye states over time\n\n";
    std::cout << std::fixed << std::setprecision(4);

    ModernEyelidEstimator estimator;

    // Phase 1: Initial training with "open" eyes
    std::cout << "Phase 1: Training with open eyes (100 frames)...\n";
    for (int i = 0; i < 100; i++) {
        bool blink = (i % 30 == 0);
        float dia = blink ? 2.0f : 3.5f + 0.2f * std::sin(i * 0.1f);
        float posY = blink ? 0.45f : 0.55f;
        estimator.Estimate(CreateEye(dia, posY, 0, 0, 1, blink));
    }

    // Test open eyes
    auto r1 = estimator.Estimate(CreateEye(3.5f, 0.55f, 0, 0, 1));
    std::cout << "  Open eyes (3.5mm, 0.55): " << r1.openness << "\n";

    // Phase 2: Transition to VERY closed eyes (feed 50 frames) - use extreme values
    std::cout << "\nPhase 2: Transitioning to very closed (50 frames)...\n";
    for (int i = 0; i < 50; i++) {
        estimator.Estimate(CreateEye(1.5f, 0.40f, 0, 0, 1));  // Much smaller diameter, lower position
    }

    auto r2 = estimator.Estimate(CreateEye(1.5f, 0.40f, 0, 0, 1));
    std::cout << "  Very closed (1.5mm, 0.40): " << r2.openness << "\n";

    // Phase 3: Return to open eyes (feed 50 frames)
    std::cout << "\nPhase 3: Returning to open eyes (50 frames)...\n";
    for (int i = 0; i < 50; i++) {
        estimator.Estimate(CreateEye(3.5f, 0.55f, 0, 0, 1));
    }

    auto r3 = estimator.Estimate(CreateEye(3.5f, 0.55f, 0, 0, 1));
    std::cout << "  Open eyes (3.5mm, 0.55): " << r3.openness << "\n";

    // Phase 4: Test blink response
    std::cout << "\nPhase 4: Testing blink response...\n";
    auto r4 = estimator.Estimate(CreateEye(2.0f, 0.45f, 0, 0, 1, true));
    std::cout << "  Blink: " << r4.openness << "\n";

    // Validation
    std::cout << "\nRESULTS:\n";
    bool openCorrect = (r1.openness < 0.1f);  // Should be close to 0.0 (open)
    bool closedDifferent = (r2.openness > r1.openness + 0.05f);  // Should be more closed than open
    bool recovers = (r3.openness < r2.openness);  // Should return toward open
    bool blinkWorks = (r4.openness < 0.1f);  // Blinks should be 0.0

    std::cout << "  ✓ Open eyes recognized: " << (openCorrect ? "YES" : "NO") << "\n";
    std::cout << "  ✓ Differentiates closed: " << (closedDifferent ? "YES" : "NO") << "\n";
    std::cout << "  ✓ Recovers to open: " << (recovers ? "YES" : "NO") << "\n";
    std::cout << "  ✓ Blink response: " << (blinkWorks ? "YES" : "NO") << "\n";

    if (openCorrect && closedDifferent && recovers && blinkWorks) {
        std::cout << "\n✓ ALL TESTS PASSED - System adapts correctly\n";
        return 0;
    } else {
        std::cout << "\n✗ SOME TESTS FAILED\n";
        return 1;
    }
}
