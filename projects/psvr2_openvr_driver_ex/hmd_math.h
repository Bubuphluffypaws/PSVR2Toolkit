#pragma once

#include <openvr_driver.h>

namespace psvr2_toolkit {

  class HmdMath {
  public:
    static vr::HmdQuaternion_t EulerToQuaternion(double yaw, double pitch, double roll) {
      double cy = cos(yaw * 0.5);
      double sy = sin(yaw * 0.5);
      double cp = cos(pitch * 0.5);
      double sp = sin(pitch * 0.5);
      double cr = cos(roll * 0.5);
      double sr = sin(roll * 0.5);
      return {
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
      };
    }

    static vr::HmdQuaternion_t QuaternionMultiply(const vr::HmdQuaternion_t &a, const vr::HmdQuaternion_t &b) {
      return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
      };
    }

    static vr::HmdQuaternion_t QuaternionInverse(const vr::HmdQuaternion_t &q) {
      double normSq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
      return { q.w / normSq, -q.x / normSq, -q.y / normSq, -q.z / normSq };
    }

    static vr::HmdVector3d_t RotateVectorByQuaternion(const vr::HmdVector3d_t &v, const vr::HmdQuaternion_t &q) {
      vr::HmdQuaternion_t vQuat = { 0.0, v.v[0], v.v[1], v.v[2] };
      vr::HmdQuaternion_t qv = QuaternionMultiply(q, vQuat);
      vr::HmdQuaternion_t qInv = QuaternionInverse(q);
      vr::HmdQuaternion_t result = QuaternionMultiply(qv, qInv);
      return { result.x, result.y, result.z };
    }

  };

}
