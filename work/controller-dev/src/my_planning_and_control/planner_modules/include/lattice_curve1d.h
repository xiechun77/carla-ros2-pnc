#pragma once

#include <array>
#include <memory>

namespace lattice {

/**
 * @brief 一维参数化曲线抽象基类（对标 Apollo Curve1d）
 *
 * 纵向轨迹以时间 t 为参数 s(t)，横向轨迹以纵向相对弧长 Δs 为参数 d(Δs)。
 */
class Curve1d
{
public:
    virtual ~Curve1d() = default;
    virtual double Evaluate(int order, double param) const = 0;
    virtual double ParamLength() const = 0;
};

/**
 * @brief 四次多项式曲线（纵向巡航采样：起点 s0, v0, a0 + 终点 v1, a1，终点位置自由）
 */
class QuarticPolynomialCurve1d : public Curve1d
{
public:
    QuarticPolynomialCurve1d(
        const std::array<double, 3>& start,
        const std::array<double, 2>& end_vel_acc,
        double param_length);

    double Evaluate(int order, double param) const override;
    double ParamLength() const override { return _param_length; }

private:
    std::array<double, 5> _coef{{0.0, 0.0, 0.0, 0.0, 0.0}};
    double _param_length = 0.0;
};

/**
 * @brief 五次多项式曲线（横向偏移采样 / 纵向跟车、超车、定点停车采样）
 */
class QuinticPolynomialCurve1d : public Curve1d
{
public:
    QuinticPolynomialCurve1d(
        const std::array<double, 3>& start,
        const std::array<double, 3>& end,
        double param_length);

    double Evaluate(int order, double param) const override;
    double ParamLength() const override { return _param_length; }

private:
    std::array<double, 6> _coef{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    double _param_length = 0.0;
};

/**
 * @brief 带末端匀速/保持外推的一维 Lattice 轨迹包装器（对标 Apollo LatticeTrajectory1d）
 */
class LatticeTrajectory1d : public Curve1d
{
public:
    explicit LatticeTrajectory1d(std::shared_ptr<Curve1d> base_curve);

    double Evaluate(int order, double param) const override;
    double ParamLength() const override;

    void set_target_position(double s) { _has_target_pos = true; _target_pos = s; }
    void set_target_velocity(double v) { _has_target_vel = true; _target_vel = v; }
    void set_target_time(double t) { _has_target_time = true; _target_time = t; }

    bool has_target_position() const { return _has_target_pos; }
    double target_position() const { return _target_pos; }
    double target_velocity() const { return _target_vel; }
    double target_time() const { return _target_time; }

private:
    std::shared_ptr<Curve1d> _base_curve;
    bool _has_target_pos = false;
    bool _has_target_vel = false;
    bool _has_target_time = false;
    double _target_pos = 0.0;
    double _target_vel = 0.0;
    double _target_time = 0.0;
};

}  // namespace lattice
