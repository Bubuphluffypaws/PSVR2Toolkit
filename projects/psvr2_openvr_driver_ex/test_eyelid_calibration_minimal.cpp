#include "modern_eyelid_estimator.h"
#include <iostream>
using namespace psvr2_toolkit;

EyeData CreateEye(float dia, float posY, float gX, float gY, float gZ, bool blink = false) {
    EyeData e;
    e.pupilDiaMm = dia; e.pupilPosY = posY;
    e.gazeDir = Vector3(gX, gY, gZ).Normalized();
    e.isBlink = blink; e.isValid = true;
    return e;
}

void Train(ModernEyelidEstimator& est, int frames) {
    for (int i = 0; i < frames; i++) {
        bool blink = (i % 30 == 0);
        float dia = blink ? 2.0f : 3.5f + 0.3f * std::sin(i * 0.1f);
        float posY = blink ? 0.45f : 0.55f;
        est.Estimate(CreateEye(dia, posY, 0, 0, 1, blink));
    }
}

int main() {
    int pass = 0, fail = 0;
    
    // Test 1: Basic function
    ModernEyelidEstimator e1;
    Train(e1, 100);
    auto r1 = e1.Estimate(CreateEye(3.5f, 0.55f, 0, 0, 1));
    if (r1.openness > 0.001f && r1.openness < 0.3f) pass++; else fail++;
    
    // Test 2: Second instance (tests static bug)
    ModernEyelidEstimator e2;
    Train(e2, 100);
    auto r2 = e2.Estimate(CreateEye(3.5f, 0.55f, 0, 0, 1));
    if (r2.openness > 0.001f && r2.openness < 0.3f) pass++; else fail++;
    
    // Test 3: Third instance
    ModernEyelidEstimator e3;
    Train(e3, 100);
    auto r3 = e3.Estimate(CreateEye(3.5f, 0.55f, 0, 0, 1));
    if (r3.openness > 0.001f && r3.openness < 0.3f) pass++; else fail++;
    
    // Test 4: Extreme gaze
    auto r4 = e3.Estimate(CreateEye(3.5f, 0.55f, 0.7f, 0.7f, 0.3f));
    if (r4.openness > 0.001f && r4.openness < 0.6f) pass++; else fail++;
    
    std::cout << "RESULTS: " << pass << "/" << (pass+fail) << " passed";
    if (fail > 0) std::cout << " [" << fail << " FAILED]";
    std::cout << "\n";
    
    return (fail == 0) ? 0 : 1;
}
