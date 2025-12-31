#pragma once

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

/**
 * @brief 向量模板类
 * 封装了 Eigen::Matrix<T, S, 1>，提供常用的向量运算。
 * @tparam T 数据类型
 * @tparam S 维度
 */
template <typename T, int S> class Vector {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Vector() { _v = Eigen::Matrix<T, S, 1>::Zero(); }

  // 单元素构造函数
  Vector(T v1) {
    if (1 != _v.rows()) {
      std::cerr << "Create vector with wrong constructor!" << std::endl;
    }
    this->operator[](0) = v1;
  }

  Vector(T v1, T v2) {
    if (2 != _v.rows()) {
      std::cerr << "Create vector with wrong constructor!" << std::endl;
    }
    this->operator[](0) = v1;
    this->operator[](1) = v2;
  }

  Vector(T v1, T v2, T v3) {
    if (3 != _v.rows()) {
      std::cerr << "Create vector with wrong constructor!" << std::endl;
    }
    this->operator[](0) = v1;
    this->operator[](1) = v2;
    this->operator[](2) = v3;
  }

  Vector(T v1, T v2, T v3, T v4) {
    if (4 != _v.rows()) {
      std::cerr << "Create vector with wrong constructor!" << std::endl;
    }
    this->operator[](0) = v1;
    this->operator[](1) = v2;
    this->operator[](2) = v3;
    this->operator[](3) = v4;
  }

  Vector(const Vector<T, S> &v) { _v = v.eigen(); }

  template <class ET> explicit Vector(const Eigen::MatrixBase<ET> &ev) {
    _v = ev;
  }

  /**
   * @brief 获取 Eigen 矩阵对象
   * @return Eigen 矩阵对象
   */
  const Eigen::Matrix<T, S, 1> eigen() const { return _v; }

  /**
   * @brief 获取向量维度
   * @return 向量维度
   */
  int size() const { return _v.rows(); }

  const T &operator()(int i) const { return _v(i); }
  T &operator()(int i) { return _v(i); }

  const T &operator[](int i) const { return _v(i); }
  T &operator[](int i) { return _v(i); }

  /**
   * @brief 获取 x 分量
   * @return x 分量引用
   */
  const T &x() const { return _v.x(); }

  /**
   * @brief 获取 y 分量
   * @return y 分量引用
   */
  const T &y() const { return _v.y(); }

  /**
   * @brief 获取 z 分量
   * @return z 分量引用
   */
  const T &z() const { return _v.z(); }

  /**
   * @brief 获取 w 分量
   * @return w 分量引用
   */
  const T &w() const { return _v.w(); }

  /**
   * @brief 类型转换
   * @tparam TC 目标类型
   * @return 转换后的向量
   */
  template <typename TC> Vector<TC, S> cast() const {
    return Vector<TC, S>(_v.template cast<TC>());
  }

  const Vector operator/(T s) const { return Vector(_v / s); }

  const Vector operator*(T s) const { return Vector(_v * s); }

  const Vector operator-(const Vector &b) const {
    return Vector(_v - b.eigen());
  }

  const Vector operator+(const Vector &b) const {
    return Vector(_v + b.eigen());
  }

  Vector &operator*=(T s) {
    _v *= s;
    return *this;
  }

  friend const Vector operator*(T s, const Vector &v) {
    return Vector(v.eigen() * s);
  }

  Vector &operator/=(T s) {
    _v /= s;
    return *this;
  }

  Vector &operator+=(const Vector &v) {
    _v += v.eigen();
    return *this;
  }

  Vector &operator-=(const Vector &v) {
    _v -= v.eigen();
    return *this;
  }

  const Vector operator-() const { return Vector(-_v); }

  /**
   * @brief 获取模长
   * @return 向量模长
   */
  T norm() const { return _v.norm(); }

  /**
   * @brief 归一化当前向量
   */
  void normalize() { _v.normalize(); }

  /**
   * @brief 获取归一化后的向量
   * @return 归一化后的向量
   */
  Vector normalized() const { return Vector(_v.normalized()); }

  /**
   * @brief 计算点积
   * @param v [in] 另一个向量
   * @return 点积结果
   */
  double dot(const Vector &v) const { return _v.dot(v.eigen()); }

  /**
   * @brief 计算叉积
   * @param v [in] 另一个向量
   * @return 叉积结果
   */
  Vector cross(const Vector &v) const { return Vector(_v.cross(v.eigen())); }

  /**
   * @brief 计算两个向量之间的夹角
   * @param v [in] 另一个向量
   * @return 夹角 (弧度)
   */
  double angle(const Vector &v) const {
    Vector nv1 = this->normalized();
    v.normalized();
    Vector nv2;
    Vector nn = this->cross(v).normalized();

    return atan2(nn.dot(nv1.cross(nv2)), nv1.dot(nv2));
  }

  static Vector zero() { return Vector(Eigen::Matrix<T, S, 1>::Zero()); }

  static Vector unitX() { return Vector(Eigen::Matrix<T, S, 1>::UnitX()); }

  static Vector unitY() { return Vector(Eigen::Matrix<T, S, 1>::UnitY()); }

  static Vector unitZ() { return Vector(Eigen::Matrix<T, S, 1>::UnitZ()); }

  static Vector unitW() { return Vector(Eigen::Matrix<T, S, 1>::UnitW()); }

protected:
  Eigen::Matrix<T, S, 1> _v; // 内部 Eigen 向量
};

using Vector1i = Vector<int, 1>;
using Vector1f = Vector<float, 1>;
using Vector1d = Vector<double, 1>;

using Vector2i = Vector<int, 2>;
using Vector2f = Vector<float, 2>;
using Vector2d = Vector<double, 2>;

using Vector3i = Vector<int, 3>;
using Vector3f = Vector<float, 3>;
using Vector3d = Vector<double, 3>;

using Vector4i = Vector<int, 4>;
using Vector4f = Vector<float, 4>;
using Vector4d = Vector<double, 4>;

using Vector2D = Vector2d;
using Vector3D = Vector3d;
using Vector4D = Vector4d;

using Vector2DList = std::vector<Vector2D, Eigen::aligned_allocator<Vector2D>>;
using Vector3DList = std::vector<Vector3D, Eigen::aligned_allocator<Vector3D>>;

/**
 * @brief 矩阵模板类
 * 封装了 Eigen::Matrix<T, R, C>
 * @tparam T 数据类型
 * @tparam R 行数
 * @tparam C 列数
 */
template <typename T, int R, int C> class Matrix {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Matrix() { _m = Eigen::Matrix<T, R, C>::Zero(); }

  Matrix(T m1, T m2, T m3, T m4) {
    if (R != 2 || C != 2) {
      std::cerr << "Create matrix with wrong constructor!" << std::endl;
    }
    _m << m1, m2, m3, m4;
  }

  Matrix(T m1, T m2, T m3, T m4, T m5, T m6, T m7, T m8, T m9) {
    if (R != 3 || C != 3) {
      std::cerr << "Create matrix with wrong constructor!" << std::endl;
    }
    _m << m1, m2, m3, m4, m5, m6, m7, m8, m9;
  }

  Matrix(T m1, T m2, T m3, T m4, T m5, T m6, T m7, T m8, T m9, T m10, T m11,
         T m12, T m13, T m14, T m15, T m16) {
    if (R != 4 || C != 4) {
      std::cerr << "Create matrix with wrong constructor!" << std::endl;
    }
    _m << m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11, m12, m13, m14, m15, m16;
  }

  Matrix(const Matrix &m) { _m = m.eigen(); }

  template <class ET> explicit Matrix(const Eigen::MatrixBase<ET> &m) {
    _m = m;
  }

  /**
   * @brief 获取 Eigen 矩阵对象
   * @return Eigen 矩阵对象
   */
  const Eigen::Matrix<T, R, C> eigen() const { return _m; }

  /**
   * @brief 获取行数
   * @return 行数
   */
  int rows() const { return _m.rows(); }

  /**
   * @brief 获取列数
   * @return 列数
   */
  int cols() const { return _m.cols(); }

  const T &operator()(int i, int j) const { return _m(i, j); }

  T &operator()(int i, int j) { return _m(i, j); }

  const T &operator[](int i) const { return _m[i]; }

  T &operator[](int i) { return _m[i]; }

  template <typename TC> Matrix<TC, R, C> cast() const {
    return Matrix<TC, R, C>(_m.template cast<TC>());
  }

  template <int MR, int MC> Matrix<T, MR, MC> block(int row, int col) const {
    return Matrix<T, MR, MC>(_m.template block<MR, MC>(row, col));
  }

  const Vector<T, C> row(int i) const {
    return Vector<T, C>(_m.row(i).transpose());
  }

  const Vector<T, R> col(int i) const { return Vector<T, R>(_m.col(i)); }

  // 运算符重载
  const Matrix operator/(T s) const { return Matrix(_m / s); }

  const Matrix operator*(T s) const { return Matrix(_m * s); }

  friend const Matrix operator*(T s, const Matrix &m) {
    return Matrix(m.eigen() * s);
  }

  const Matrix operator-(const Matrix &m) const {
    return Matrix(_m - m.eigen());
  }

  const Matrix operator+(const Matrix &m) const {
    return Matrix(_m + m.eigen());
  }

  const Matrix operator*(const Matrix &m) const {
    return Matrix(_m * m.eigen());
  }

  const Vector<T, R> operator*(const Vector<T, C> &v) const {
    return Vector<T, R>(_m * v.eigen());
  }

  Matrix &operator*=(T s) {
    _m *= s;
    return *this;
  }

  Matrix &operator/=(T s) {
    _m /= s;
    return *this;
  }

  Matrix &operator+=(const Matrix &m) {
    _m += m.eigen();
    return *this;
  }

  Matrix &operator-=(const Matrix &m) {
    _m -= m.eigen();
    return *this;
  }

  const Matrix operator-() const { return Matrix(-_m); }

  /**
   * @brief 矩阵求逆 (就地修改)
   */
  void inverse() { _m = _m.inverse().eval(); }

  /**
   * @brief 获取逆矩阵
   * @return 逆矩阵
   */
  Matrix inversed() const { return Matrix(_m.inverse()); }

  /**
   * @brief 矩阵转置 (就地修改)
   */
  void transpose() { _m.transposeInPlace(); }

  /**
   * @brief 获取转置矩阵
   * @return 转置矩阵
   */
  Matrix transposed() const { return Matrix(_m.transpose()); }

  static Matrix zero() { return Matrix(Eigen::Matrix<T, R, C>::Zero()); }

  static Matrix identity() {
    return Matrix(Eigen::Matrix<T, R, C>::Identity());
  }

protected:
  Eigen::Matrix<T, R, C> _m; // 内部 Eigen 矩阵
};

using Matrix2i = Matrix<int, 2, 2>;
using Matrix2f = Matrix<float, 2, 2>;
using Matrix2d = Matrix<double, 2, 2>;

using Matrix3i = Matrix<int, 3, 3>;
using Matrix3f = Matrix<float, 3, 3>;
using Matrix3d = Matrix<double, 3, 3>;

using Matrix4i = Matrix<int, 4, 4>;
using Matrix4f = Matrix<float, 4, 4>;
using Matrix4d = Matrix<double, 4, 4>;

/**
 * @brief 线性代数工具类
 * 提供正交性检查等静态方法。
 */
class LinearAlgebra {
public:
  template <class R>
  static inline bool
  isOrthonormal(const Eigen::MatrixBase<R> &r,
                typename R::Scalar precision =
                    std::numeric_limits<typename R::Scalar>::epsilon()) {
    return (r * r.transpose()).isIdentity(precision);
  }

  // 检查是否为正交矩阵 (行列式为 1)
  template <class R>
  static inline bool
  isProperOrthonormal(const Eigen::MatrixBase<R> &r,
                      typename R::Scalar precision =
                          std::numeric_limits<typename R::Scalar>::epsilon()) {
    return isOrthonormal(r, precision) &&
           fabs(r.determinant() - 1.0) <= precision;
  }

  // 检查是否属于特殊正交群 SO(n)
  template <class R> static inline bool isSO(const Eigen::MatrixBase<R> &M) {
    return M.cols() == M.rows() && isProperOrthonormal(M);
  }

  template <class R>
  static inline bool isSO(const Eigen::MatrixBase<R> &M,
                          typename R::Scalar precision) {
    return M.cols() == M.rows() && isProperOrthonormal(M, precision);
  }
};

/**
 * @brief 旋转矩阵类 (3x3)
 * 继承自 Matrix3d，表示 SO(3) 旋转。
 */
class Rotation3D : public Matrix3d {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Rotation3D() : Matrix3d(Matrix3d::identity()) {}

  Rotation3D(const Matrix3d &m) : Matrix3d(m) {}

  // 逐元素构造
  Rotation3D(double r11, double r12, double r13, double r21, double r22,
             double r23, double r31, double r32, double r33)
      : Matrix3d(r11, r12, r13, r21, r22, r23, r31, r32, r33) {}

  // 从三个列向量构造
  Rotation3D(const Vector3D &i, const Vector3D &j, const Vector3D &k) {
    _m.col(0) = i.eigen();
    _m.col(1) = j.eigen();
    _m.col(2) = k.eigen();
  }

  template <class T> Rotation3D(const Eigen::MatrixBase<T> &m) : Matrix3d(m) {}

  // 比较两个旋转矩阵是否相等
  bool
  equal(const Rotation3D &rot,
        const double precision = std::numeric_limits<double>::epsilon()) const {
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        if (fabs(_m(i, j) - rot(i, j)) > precision)
          return false;
    return true;
  }

  // 检查是否为合法的旋转矩阵
  bool isProperRotation() const { return LinearAlgebra::isSO(eigen()); }

  bool isProperRotation(double precision) const {
    return LinearAlgebra::isSO(eigen(), precision);
  }

  // 静态工厂函数：生成基本旋转矩阵
  /**
   * @brief 生成绕 X 轴旋转的旋转矩阵
   * @param angle [in] 旋转角度 (弧度)
   * @return 旋转矩阵
   */
  inline static Rotation3D RotX(const double angle) {
    Rotation3D tmp = Rotation3D();
    double ct = cos(angle);
    double st = sin(angle);
    tmp(1, 1) = ct;
    tmp(1, 2) = -st;
    tmp(2, 1) = st;
    tmp(2, 2) = ct;
    return tmp;
  }

  /**
   * @brief 生成绕 Y 轴旋转的旋转矩阵
   * @param angle [in] 旋转角度 (弧度)
   * @return 旋转矩阵
   */
  inline static Rotation3D RotY(const double angle) {
    Rotation3D tmp = Rotation3D();
    double ct = cos(angle);
    double st = sin(angle);
    tmp(0, 0) = ct;
    tmp(0, 2) = st;
    tmp(2, 0) = -st;
    tmp(2, 2) = ct;
    return tmp;
  }

  /**
   * @brief 生成绕 Z 轴旋转的旋转矩阵
   * @param angle [in] 旋转角度 (弧度)
   * @return 旋转矩阵
   */
  inline static Rotation3D RotZ(const double angle) {
    Rotation3D tmp = Rotation3D();
    double ct = cos(angle);
    double st = sin(angle);
    tmp(0, 0) = ct;
    tmp(0, 1) = -st;
    tmp(1, 0) = st;
    tmp(1, 1) = ct;
    return tmp;
  }
};

using Rotation3DList =
    std::vector<Rotation3D, Eigen::aligned_allocator<Rotation3D>>;

/**
 * @brief 齐次变换矩阵类 (4x4)
 * 表示 SE(3) 变换 [R t; 0 1]。
 */
class Transform3D : public Matrix4d {
public:
  Transform3D() : Matrix4d(Matrix4d::identity()) {}

  Transform3D(const Matrix4d &m) : Matrix4d(m) {}

  // 从平移向量和旋转矩阵构造
  Transform3D(const Vector3D &v, const Rotation3D &r)
      : Matrix4d(Matrix4d::identity()) {
    _m.block<3, 1>(0, 3) = v.eigen();
    _m.block<3, 3>(0, 0) = r.eigen();
  }

  explicit Transform3D(const Rotation3D &r) : Matrix4d(Matrix4d::identity()) {
    _m.block<3, 3>(0, 0) = r.eigen();
  }

  explicit Transform3D(const Vector3D &v) : Matrix4d(Matrix4d::identity()) {
    _m.block<3, 1>(0, 3) = v.eigen();
  }

  template <class ET> explicit Transform3D(const Eigen::MatrixBase<ET> &em) {
    _m = em;
  }

  // 提取旋转部分
  Rotation3D rotation() { return Rotation3D(_m.block<3, 3>(0, 0)); }
  const Rotation3D rotation() const { return Rotation3D(_m.block<3, 3>(0, 0)); }

  // 提取平移部分
  Vector3D translation() { return Vector3D(_m.block<3, 1>(0, 3)); }
  const Vector3D translation() const { return Vector3D(_m.block<3, 1>(0, 3)); }
};

using Transform3DList =
    std::vector<Transform3D, Eigen::aligned_allocator<Transform3D>>;

/**
 * @brief 欧拉角 (Roll-Pitch-Yaw) 类
 * 继承自 Vector3D，处理旋转顺序和参考系。
 */
class RPY : public Vector3D {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 参考系类型：内旋 vs 外旋
  typedef enum ReferenceType {
    INTRINSIC = 0, // 绕运动轴旋转
    EXTRINSIC = 1  // 绕固定轴旋转
  } ReferenceType;

  // 旋转顺序：支持全部 12 种类型
  typedef enum RPYType {
    XYZ = 0,
    XZY = 1,
    YXZ = 2,
    YZX = 3,
    ZXY = 4,
    ZYX = 5,
    XYX = 6,
    XZX = 7,
    YXY = 8,
    YZY = 9,
    ZXZ = 10,
    ZYZ = 11,
    ROT_VECTOR = 12 // UR Robot Axis-Angle (Rotation Vector)
  } RPYType;

  // 构造函数
  RPY(RPYType rpy_type = RPYType::XYZ,
      ReferenceType ref_type = ReferenceType::EXTRINSIC)
      : Vector3d(0.0, 0.0, 0.0), _rpy_type(rpy_type), _ref_type(ref_type) {}

  RPY(double roll, double pitch, double yaw, RPYType rpy_type = RPYType::XYZ,
      ReferenceType ref_type = ReferenceType::EXTRINSIC)
      : Vector3d(roll, pitch, yaw), _rpy_type(rpy_type), _ref_type(ref_type) {}

  // 从 Rotation3D 构造
  RPY(Rotation3D r, RPYType rpy_type = RPYType::XYZ,
      ReferenceType ref_type = ReferenceType::EXTRINSIC);

  /**
   * @brief 转换为 Rotation3D 旋转矩阵
   * @return 旋转矩阵
   */
  const Rotation3D toRotation3D() const;

  RPYType getRPYType() const { return _rpy_type; }

  ReferenceType getReferenceType() const { return _ref_type; }

  // 角度访问器 (度)
  double &rx() { return _v[0]; }
  double &ry() { return _v[1]; }
  double &rz() { return _v[2]; }

  void setRPYType(RPYType r) { _rpy_type = r; }
  void setReferenceType(ReferenceType r) { _ref_type = r; }

  /**
   * @brief 从 RPY 欧拉角生成旋转矩阵
   * @param v [in] RPY 欧拉角
   * @param rpy_type [in] 旋转顺序类型
   * @param ref_type [in] 参考系类型
   * @return 旋转矩阵
   */
  static Rotation3D fromRPY(RPY v, RPYType rpy_type, ReferenceType ref_type);

  /**
   * @brief 从旋转矩阵计算 RPY 欧拉角
   * @param r [in] 旋转矩阵
   * @param rpy_type [in] 旋转顺序类型
   * @param ref_type [in] 参考系类型
   * @return RPY 欧拉角
   */
  static RPY toRPY(Rotation3D r, RPYType rpy_type, ReferenceType ref_type);

private:
  RPYType _rpy_type;       // 旋转顺序类型
  ReferenceType _ref_type; // 参考系类型
};
