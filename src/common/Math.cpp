#include "common/Math.h"

RPY::RPY(Rotation3D r, RPYType rpy_type, ReferenceType ref_type)
    : _rpy_type(rpy_type), _ref_type(ref_type) {
  if ((_rpy_type == RPYType::XYZ && _ref_type == ReferenceType::EXTRINSIC) ||
      (_rpy_type == RPYType::ZYX && _ref_type == ReferenceType::INTRINSIC)) {
    double zer = 1e-8;
    double rx = 0;
    double ry = 0;
    double rz = 0;

    ry = atan2(-r(2, 0), sqrt(r(0, 0) * r(0, 0) + r(1, 0) * r(1, 0)));

    if (ry < M_PI / 2 + zer && ry > M_PI / 2 - zer) {
      ry = M_PI / 2;
      rx = atan2(r(0, 1), r(1, 1));
      rz = 0;
    } else if (ry > -M_PI / 2 - zer && ry < -M_PI / 2 + zer) {
      ry = -M_PI / 2;
      rx = -atan2(r(0, 1), r(1, 1));
      rz = 0;
    } else {
      rz = atan2(r(1, 0) / cos(ry), r(0, 0) / cos(ry));
      rx = atan2(r(2, 1) / cos(ry), r(2, 2) / cos(ry));
    }

    rx = rx / M_PI * 180;
    ry = ry / M_PI * 180;
    rz = rz / M_PI * 180;

    _v[0] = rx;
    _v[1] = ry;
    _v[2] = rz;
  } else if (_rpy_type == RPYType::ROT_VECTOR) {
    Eigen::AngleAxisd aa(r.eigen());
    Eigen::Vector3d rv = aa.axis() * aa.angle();
    _v[0] = rv[0]; // Axis-Angle/Rotation Vector remains in radians
    _v[1] = rv[1];
    _v[2] = rv[2];
  } else {
    std::cerr << "Unsupported RPYType & ReferenceType" << std::endl;
  }
}

const Rotation3D RPY::toRotation3D() const {
  if ((_rpy_type == RPYType::XYZ && _ref_type == ReferenceType::EXTRINSIC) ||
      (_rpy_type == RPYType::ZYX && _ref_type == ReferenceType::INTRINSIC)) {
    Rotation3D r;
    r(0, 0) = cos(_v[2] / 180 * M_PI) * cos(_v[1] / 180 * M_PI);
    r(0, 1) = cos(_v[2] / 180 * M_PI) * sin(_v[1] / 180 * M_PI) *
                  sin(_v[0] / 180 * M_PI) -
              sin(_v[2] / 180 * M_PI) * cos(_v[0] / 180 * M_PI);
    r(0, 2) = cos(_v[2] / 180 * M_PI) * sin(_v[1] / 180 * M_PI) *
                  cos(_v[0] / 180 * M_PI) +
              sin(_v[2] / 180 * M_PI) * sin(_v[0] / 180 * M_PI);
    r(1, 0) = sin(_v[2] / 180 * M_PI) * cos(_v[1] / 180 * M_PI);
    r(1, 1) = sin(_v[2] / 180 * M_PI) * sin(_v[1] / 180 * M_PI) *
                  sin(_v[0] / 180 * M_PI) +
              cos(_v[2] / 180 * M_PI) * cos(_v[0] / 180 * M_PI);
    r(1, 2) = sin(_v[2] / 180 * M_PI) * sin(_v[1] / 180 * M_PI) *
                  cos(_v[0] / 180 * M_PI) -
              cos(_v[2] / 180 * M_PI) * sin(_v[0] / 180 * M_PI);
    r(2, 0) = -sin(_v[1] / 180 * M_PI);
    r(2, 1) = cos(_v[1] / 180 * M_PI) * sin(_v[0] / 180 * M_PI);
    r(2, 2) = cos(_v[1] / 180 * M_PI) * cos(_v[0] / 180 * M_PI);
    return (r);
  } else if (_rpy_type == RPYType::ROT_VECTOR) {
    Eigen::Vector3d rv(_v[0], _v[1], _v[2]);
    double angle = rv.norm();
    if (angle < 1e-10)
      return Rotation3D::identity();
    return Rotation3D(Eigen::AngleAxisd(angle, rv / angle).toRotationMatrix());
  } else {
    std::cerr << "Unsupported RPYType & ReferenceType" << std::endl;
  }
  return Rotation3D();
}

Rotation3D RPY::fromRPY(RPY v, RPY::RPYType rpy_type,
                        RPY::ReferenceType ref_type) {
  double r1 = v.rx() * M_PI / 180;
  double r2 = v.ry() * M_PI / 180;
  double r3 = v.rz() * M_PI / 180;
  Matrix4d trans = Matrix4d::identity();
  Rotation3D rm_r1 = Rotation3D();
  Rotation3D rm_r2 = Rotation3D();
  Rotation3D rm_r3 = Rotation3D();
  Rotation3D m = Rotation3D();

  switch (rpy_type) {
  case RPY::RPYType::XYZ: {
    rm_r1 = Rotation3D::RotX(r1);
    rm_r2 = Rotation3D::RotY(r2);
    rm_r3 = Rotation3D::RotZ(r3);
    break;
  }
  case RPY::RPYType::XZY: {
    rm_r1 = Rotation3D::RotX(r1);
    rm_r2 = Rotation3D::RotZ(r2);
    rm_r3 = Rotation3D::RotY(r3);
    break;
  }
  case RPY::RPYType::YXZ: {
    rm_r1 = Rotation3D::RotY(r1);
    rm_r2 = Rotation3D::RotX(r2);
    rm_r3 = Rotation3D::RotZ(r3);
    break;
  }
  case RPY::RPYType::YZX: {
    rm_r1 = Rotation3D::RotY(r1);
    rm_r2 = Rotation3D::RotZ(r2);
    rm_r3 = Rotation3D::RotX(r3);
    break;
  }
  case RPY::RPYType::ZXY: {
    rm_r1 = Rotation3D::RotZ(r1);
    rm_r2 = Rotation3D::RotX(r2);
    rm_r3 = Rotation3D::RotY(r3);
    break;
  }
  case RPY::RPYType::ZYX: {
    rm_r1 = Rotation3D::RotZ(r1);
    rm_r2 = Rotation3D::RotY(r2);
    rm_r3 = Rotation3D::RotX(r3);
    break;
  }
  case RPY::RPYType::XYX: {
    rm_r1 = Rotation3D::RotX(r1);
    rm_r2 = Rotation3D::RotY(r2);
    rm_r3 = Rotation3D::RotX(r3);
    break;
  }
  case RPY::RPYType::XZX: {
    rm_r1 = Rotation3D::RotX(r1);
    rm_r2 = Rotation3D::RotZ(r2);
    rm_r3 = Rotation3D::RotX(r3);
    break;
  }
  case RPY::RPYType::YXY: {
    rm_r1 = Rotation3D::RotY(r1);
    rm_r2 = Rotation3D::RotX(r2);
    rm_r3 = Rotation3D::RotY(r3);
    break;
  }
  case RPY::RPYType::YZY: {
    rm_r1 = Rotation3D::RotY(r1);
    rm_r2 = Rotation3D::RotZ(r2);
    rm_r3 = Rotation3D::RotY(r3);
    break;
  }
  case RPY::RPYType::ZXZ: {
    rm_r1 = Rotation3D::RotZ(r1);
    rm_r2 = Rotation3D::RotX(r2);
    rm_r3 = Rotation3D::RotZ(r3);
    break;
  }
  case RPY::RPYType::ZYZ: {
    rm_r1 = Rotation3D::RotZ(r1);
    rm_r2 = Rotation3D::RotY(r2);
    rm_r3 = Rotation3D::RotZ(r3);
    break;
  }
  case RPY::RPYType::ROT_VECTOR: {
    // Rotation vector is already provided in the RPY object's v[0,1,2]
    break;
  }
  }

  switch (rpy_type) {
  case RPY::RPYType::XYZ:
  case RPY::RPYType::XZY:
  case RPY::RPYType::YXZ:
  case RPY::RPYType::YZX:
  case RPY::RPYType::ZXY:
  case RPY::RPYType::ZYX:
  case RPY::RPYType::XYX:
  case RPY::RPYType::XZX:
  case RPY::RPYType::YXY:
  case RPY::RPYType::YZY:
  case RPY::RPYType::ZXZ:
  case RPY::RPYType::ZYZ: {
    if (ref_type == RPY::ReferenceType::EXTRINSIC) {
      m = rm_r3.eigen() * rm_r2.eigen() * rm_r1.eigen();
    } else {
      m = rm_r1.eigen() * rm_r2.eigen() * rm_r3.eigen();
    }
    break;
  }
  case RPY::RPYType::ROT_VECTOR: {
    Eigen::Vector3d rv(v.rx(), v.ry(), v.rz());
    double angle = rv.norm();
    if (angle < 1e-10)
      return Rotation3D::identity();
    return Rotation3D(Eigen::AngleAxisd(angle, rv / angle).toRotationMatrix());
  }
  }
  return m;
}

RPY RPY::toRPY(Rotation3D r, RPY::RPYType rpy_type,
               RPY::ReferenceType ref_type) {
  double r1;
  double r2;
  double r3;
  double eps = 1e-9;

  switch (rpy_type) {
  case RPY::RPYType::XYZ:
  case RPY::RPYType::ZYX: {
    if ((rpy_type == RPY::RPYType::XYZ &&
         ref_type == RPY::ReferenceType::EXTRINSIC) ||
        (rpy_type == RPY::RPYType::ZYX &&
         ref_type == RPY::ReferenceType::INTRINSIC)) {
      r2 = atan2(-r(2, 0), sqrt(r(0, 0) * r(0, 0) + r(1, 0) * r(1, 0)));

      if (r2 < M_PI / 2 + eps && r2 > M_PI / 2 - eps) {
        r2 = M_PI / 2;
        r1 = atan2(-r(1, 2), r(1, 1));
        r3 = 0;
      } else if (r2 > (-M_PI / 2 - eps) && r2 < (-M_PI / 2 + eps)) {
        r2 = -M_PI / 2;
        r1 = atan2(-r(1, 2), r(1, 1));
        r3 = 0;
      } else {
        double c2 = cos(r2);
        r3 = atan2(r(1, 0) / c2, r(0, 0) / c2);
        r1 = atan2(r(2, 1) / c2, r(2, 2) / c2);
      }
    } else {
      r2 = atan2(r(0, 2), sqrt(r(1, 2) * r(1, 2) + r(2, 2) * r(2, 2)));

      if (r2 < M_PI / 2 + eps && r2 > M_PI / 2 - eps) {
        r2 = M_PI / 2;
        r1 = atan2(r(1, 0), r(1, 1));
        r3 = 0;
      } else if (r2 > (-M_PI / 2 - eps) && r2 < (-M_PI / 2 + eps)) {
        r2 = -M_PI / 2;
        r1 = atan2(r(1, 0), r(1, 1));
        r3 = 0;
      } else {
        double c2 = cos(r2);
        r3 = atan2(-r(1, 2) / c2, r(2, 2) / c2);
        r1 = atan2(-r(0, 1) / c2, r(0, 0) / c2);
      }
    }
    break;
  }
  case RPY::RPYType::XZY:
  case RPY::RPYType::YZX: {
    if ((rpy_type == RPY::RPYType::XZY &&
         ref_type == RPY::ReferenceType::EXTRINSIC) ||
        (rpy_type == RPY::RPYType::YZX &&
         ref_type == RPY::ReferenceType::INTRINSIC)) {
      r2 = atan2(r(1, 0), sqrt(r(1, 1) * r(1, 1) + r(1, 2) * r(1, 2)));

      if (r2 < M_PI / 2 + eps && r2 > M_PI / 2 - eps) {
        r2 = M_PI / 2;
        r1 = atan2(r(2, 1), r(2, 2));
        r3 = 0;
      } else if (r2 > (-M_PI / 2 - eps) && r2 < (-M_PI / 2 + eps)) {
        r2 = -M_PI / 2;
        r1 = atan2(r(2, 1), r(2, 2));
        r3 = 0;
      } else {
        double c2 = cos(r2);
        r3 = atan2(-r(2, 0) / c2, r(0, 0) / c2);
        r1 = atan2(-r(1, 2) / c2, r(1, 1) / c2);
      }
    } else {
      r2 = atan2(-r(0, 1), sqrt(r(0, 0) * r(0, 0) + r(0, 2) * r(0, 2)));
      if (r2 < M_PI / 2 + eps && r2 > M_PI / 2 - eps) {
        r2 = M_PI / 2;
        r1 = atan2(-r(2, 0), r(2, 2));
        r3 = 0;
      } else if (r2 > (-M_PI / 2 - eps) && r2 < (-M_PI / 2 + eps)) {
        r2 = -M_PI / 2;
        r1 = atan2(-r(2, 0), r(2, 2));
        r3 = 0;
      } else {
        double c2 = cos(r2);
        r3 = atan2(r(2, 1) / c2, r(1, 1) / c2);
        r1 = atan2(r(0, 2) / c2, r(0, 0) / c2);
      }
    }
    break;
  }
  case RPY::RPYType::YXZ:
  case RPY::RPYType::ZXY: {
    if ((rpy_type == RPY::RPYType::YXZ &&
         ref_type == RPY::ReferenceType::EXTRINSIC) ||
        (rpy_type == RPY::RPYType::ZXY &&
         ref_type == RPY::ReferenceType::INTRINSIC)) {
      r2 = atan2(r(2, 1), sqrt(r(2, 0) * r(2, 0) + r(2, 2) * r(2, 2)));

      if (r2 < M_PI / 2 + eps && r2 > M_PI / 2 - eps) {
        r2 = M_PI / 2;
        r1 = atan2(r(0, 2), r(0, 0));
        r3 = 0;
      } else if (r2 > (-M_PI / 2 - eps) && r2 < (-M_PI / 2 + eps)) {
        r2 = -M_PI / 2;
        r1 = atan2(r(0, 2), r(0, 0));
        r3 = 0;
      } else {
        double c2 = cos(r2);
        r1 = atan2(-r(2, 0) / c2, r(2, 2) / c2);
        r3 = atan2(-r(0, 1) / c2, r(1, 1) / c2);
      }
    } else {
      r2 = atan2(-r(1, 2), sqrt(r(0, 2) * r(0, 2) + r(2, 2) * r(2, 2)));

      if (r2 < M_PI / 2 + eps && r2 > M_PI / 2 - eps) {
        r2 = M_PI / 2;
        r1 = atan2(-r(0, 1), r(0, 0));
        r3 = 0;
      } else if (r2 > (-M_PI / 2 - eps) && r2 < (-M_PI / 2 + eps)) {
        r2 = -M_PI / 2;
        r1 = atan2(-r(0, 1), r(0, 0));
        r3 = 0;
      } else {
        double c2 = cos(r2);
        r3 = atan2(r(0, 2) / c2, r(2, 2) / c2);
        r1 = atan2(r(1, 0) / c2, r(1, 1) / c2);
      }
    }
    break;
  }
  case RPY::RPYType::XYX: {
    r2 = atan2(sqrt(r(0, 1) * r(0, 1) + r(0, 2) * r(0, 2)), r(0, 0));

    if (r2 < eps && r2 > -eps) {
      r2 = 0;
      r1 = atan2(-r(1, 2), r(1, 1));
      r3 = 0;
    } else {
      double s2 = sin(r2);
      r3 = atan2(r(1, 0), -r(2, 0));
      r1 = atan2(r(0, 1) / s2, r(0, 2) / s2);
    }
    break;
  }
  case RPY::RPYType::XZX: {
    r2 = atan2(sqrt(r(0, 1) * r(0, 1) + r(0, 2) * r(0, 2)), r(0, 0));

    if (r2 < eps && r2 > -eps) {
      r2 = 0;
      r1 = atan2(r(2, 1), r(2, 2));
      r3 = 0;
    } else {
      double s2 = sin(r2);
      r3 = atan2(r(2, 0) / s2, r(1, 0) / s2);
      r1 = atan2(r(0, 2) / s2, -r(0, 1) / s2);
    }
    break;
  }
  case RPY::RPYType::YXY: {
    r2 = atan2(sqrt(r(1, 0) * r(1, 0) + r(1, 2) * r(1, 2)), r(1, 1));

    if (r2 < eps && r2 > -eps) {
      r2 = 0;
      r1 = atan2(r(0, 2), r(0, 0));
      r3 = 0;
    } else {
      double s2 = sin(r2);
      r3 = atan2(r(0, 1) / s2, r(2, 1) / s2);
      r1 = atan2(r(1, 0) / s2, -r(1, 2) / s2);
    }
    break;
  }
  case RPY::RPYType::YZY: {
    r2 = atan2(sqrt(r(1, 0) * r(1, 0) + r(1, 2) * r(1, 2)), r(1, 1));

    if (r2 < eps && r2 > -eps) {
      r2 = 0;
      r1 = atan2(-r(2, 0), r(2, 2));
      r3 = 0;
    } else {
      double s2 = sin(r2);
      r3 = atan2(r(2, 1) / s2, -r(0, 1) / s2);
      r1 = atan2(r(1, 2) / s2, r(1, 0) / s2);
    }
    break;
  }
  case RPY::RPYType::ZXZ: {
    r2 = atan2(sqrt(r(2, 0) * r(2, 0) + r(2, 1) * r(2, 1)), r(2, 2));

    if (r2 < eps && r2 > -eps) {
      r2 = 0;
      r1 = atan2(-r(0, 1), r(0, 0));
      r3 = 0;
    } else {
      double s2 = sin(r2);
      r3 = atan2(r(0, 2) / s2, -r(1, 2) / s2);
      r1 = atan2(r(2, 0) / s2, r(2, 1) / s2);
    }
    break;
  }
  case RPY::RPYType::ZYZ: {
    r2 = atan2(sqrt(r(2, 0) * r(2, 0) + r(2, 1) * r(2, 1)), r(2, 2));

    if (r2 < eps && r2 > -eps) {
      r2 = 0;
      r1 = atan2(r(1, 0), r(1, 1));
      r3 = 0;
    } else {
      double s2 = sin(r2);
      r3 = atan2(r(1, 2) / s2, r(0, 2) / s2);
      r1 = atan2(r(2, 1) / s2, -r(2, 0) / s2);
    }
    break;
  }
  case RPY::RPYType::ROT_VECTOR: {
    Eigen::AngleAxisd aa(r.eigen());
    Eigen::Vector3d rv = aa.axis() * aa.angle();
    return RPY(rv[0], rv[1], rv[2], RPY::ROT_VECTOR);
  }
  }

  if (rpy_type == RPY::RPYType::XYZ || rpy_type == RPY::RPYType::XZY ||
      rpy_type == RPY::RPYType::YXZ || rpy_type == RPY::RPYType::YZX ||
      rpy_type == RPY::RPYType::ZXY || rpy_type == RPY::RPYType::ZYX ||
      rpy_type == RPY::RPYType::XYX || rpy_type == RPY::RPYType::XZX ||
      rpy_type == RPY::RPYType::YXY || rpy_type == RPY::RPYType::YZY ||
      rpy_type == RPY::RPYType::ZXZ || rpy_type == RPY::RPYType::ZYZ) {
    if (ref_type == RPY::ReferenceType::INTRINSIC) {
      std::swap(r1, r3);
    }
  }

  return RPY(r1 * 180 / M_PI, r2 * 180 / M_PI, r3 * 180 / M_PI);
}
