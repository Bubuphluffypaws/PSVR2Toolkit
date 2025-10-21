#pragma once

#include "hmd2_gaze.h"

namespace psvr2_toolkit {

  class EyelidEstimator {
  public:
    EyelidEstimator();

    float Estimate(const Hmd2GazeEye &eye);

  private:
    float m_openDia;
    float m_closedDia;
    float m_sensorYOpen;
    float m_sensorYClosed;
    float m_lastOpenness;

    // Adaptive neutral gaze learning state
    Hmd2Vector3 m_learnedNeutralGaze; // Currently learned neutral gaze direction
    int m_gazeSampleCount; // Number of gaze samples collected for learning
    bool m_neutralGazeLearned; // Whether we have enough data to use learned neutral gaze

    bool IsNeutral(const Hmd2GazeEye &eye) const;
    void UpdateLearnedNeutralGaze(const Hmd2GazeEye &eye);
  };

}
