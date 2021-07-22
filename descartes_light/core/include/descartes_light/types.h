#pragma once

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
#include <Eigen/Geometry>
DESCARTES_IGNORE_WARNINGS_POP

namespace descartes_light
{
template <typename FloatType>
using Array = Eigen::Array<FloatType, Eigen::Dynamic, 1>;

/** @brief Definition of a state */
template <typename FloatType>
class State
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<State>;
  using ConstPtr = std::shared_ptr<const State>;

  State() = default;
  virtual ~State() = default;
  State(const State&) = delete;
  State& operator=(const State&) = delete;
  State(State&&) noexcept = delete;
  State& operator=(State&&) noexcept = delete;

  ////////////////////////
  // Eigen Constructors //
  ////////////////////////

  // This constructor allows you to construct the class from Eigen expressions
  template <typename OtherDerived>
  State(const Eigen::MatrixBase<OtherDerived>& other) : values(other)
  {
  }

  State(std::initializer_list<double> l)
  {
    values.resize(static_cast<Eigen::Index>(l.size()));
    Eigen::Index i = 0;
    for (auto& v : l)
      values(i++) = v;
  }

  Eigen::Matrix<FloatType, Eigen::Dynamic, 1> values;

  /////////////////////
  // Eigen Operators //
  /////////////////////

  // This method allows you to assign Eigen expressions to the class
  template <typename OtherDerived>
  inline State& operator=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    values = other;
    return *this;
  }

  template <typename OtherDerived>
  inline State& operator*=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    values = values * other;
    return *this;
  }

  inline State& operator=(std::initializer_list<double> l)
  {
    values.resize(static_cast<Eigen::Index>(l.size()));
    Eigen::Index i = 0;
    for (auto& v : l)
      values(i++) = v;

    return *this;
  }

  template <typename OtherDerived>
  inline Eigen::Matrix<FloatType, Eigen::Dynamic, 1> operator*(const Eigen::MatrixBase<OtherDerived>& other) const
  {
    Eigen::Matrix<FloatType, Eigen::Dynamic, 1> s = values;
    s = s * other;
    return s;
  }

  template <typename OtherDerived>
  inline Eigen::Matrix<FloatType, Eigen::Dynamic, 1> operator-(const Eigen::MatrixBase<OtherDerived>& other) const
  {
    Eigen::Matrix<FloatType, Eigen::Dynamic, 1> s = values;
    s = s - other;
    return s;
  }

  template <typename OtherDerived>
  inline Eigen::Matrix<FloatType, Eigen::Dynamic, 1> operator+(const Eigen::MatrixBase<OtherDerived>& other) const
  {
    Eigen::Matrix<FloatType, Eigen::Dynamic, 1> s = values;
    s = s + other;
    return s;
  }

  inline Eigen::Matrix<FloatType, Eigen::Dynamic, 1> operator-(const State& other) const
  {
    Eigen::Matrix<FloatType, Eigen::Dynamic, 1> s = values;
    s = s - other.values;
    return s;
  }

  template <typename OtherDerived>
  inline Eigen::Matrix<FloatType, Eigen::Dynamic, 1> operator+(const State& other) const
  {
    Eigen::Matrix<FloatType, Eigen::Dynamic, 1> s = values;
    s = s + other.values;
    return s;
  }

  inline Eigen::CommaInitializer<Eigen::Matrix<FloatType, Eigen::Dynamic, 1>> operator<<(const double& s)
  {
    return Eigen::CommaInitializer<Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>(values, s);
  }

  inline double& operator[](Eigen::Index i) { return values[i]; }
  inline double operator[](Eigen::Index i) const { return values[i]; }
  inline double& operator()(Eigen::Index i) { return values(i); }
  inline double operator()(Eigen::Index i) const { return values(i); }

  //////////////////////////
  // Implicit Conversions //
  //////////////////////////

  /** @return Implicit conversion to read-only Eigen::VectorX */
  inline operator const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>&() const { return values; }

  /** @return Implicit conversion to writable Eigen::VectorX */
  inline operator Eigen::Matrix<FloatType, Eigen::Dynamic, 1>&() { return values; }

  /** @return Implicit conversion to read-only Eigen::VectorX */
  inline operator Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>() const
  {
    return Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>(values);
  }

  /** @return Implicit conversion to writable Eigen::VectorX */
  inline operator Eigen::Ref<Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>()
  {
    return Eigen::Ref<Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>(values);
  }
};

template <typename FloatType>
struct StateSample
{
  StateSample() = default;
  // NOLINTNEXTLINE(modernize-pass-by-value)
  StateSample(typename State<FloatType>::ConstPtr state_, FloatType cost_) : state(std::move(state_)), cost(cost_) {}
  // NOLINTNEXTLINE(modernize-pass-by-value)
  StateSample(typename State<FloatType>::ConstPtr state_) : StateSample(std::move(state_), static_cast<FloatType>(0.0))
  {
  }

  /** @brief State values */
  typename State<FloatType>::ConstPtr state{ nullptr };
  /** @brief State cost */
  FloatType cost{ static_cast<FloatType>(0.0) };
};

}  // namespace descartes_light
