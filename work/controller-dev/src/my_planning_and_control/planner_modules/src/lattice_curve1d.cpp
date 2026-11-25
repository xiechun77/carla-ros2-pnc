#include "lattice_curve1d.h"

#include <algorithm>
#include <cmath>

namespace lattice {

QuarticPolynomialCurve1d::QuarticPolynomialCurve1d(
    const std::array<double, 3>& start,
    const std::array<double, 2>& end_vel_acc,
    double param_length)
    : _param_length(std::max(1e-3, param_length))
{
    const double T = _param_length;
    const double T2 = T * T;
    const double T3 = T2 * T;

    _coef[0] = start[0];
    _coef[1] = start[1];
    _coef[2] = 0.5 * start[2];

    const double dv = end_vel_acc[0] - _coef[1] - 2.0 * _coef[2] * T;
    const double da = end_vel_acc[1] - 2.0 * _coef[2];

    _coef[3] = (3.0 * dv - da * T) / (3.0 * T2);
    _coef[4] = (da * T - 2.0 * dv) / (4.0 * T3);
}

double QuarticPolynomialCurve1d::Evaluate(int order, double p) const
{
    switch (order) {
        case 0:
            return (((_coef[4] * p + _coef[3]) * p + _coef[2]) * p + _coef[1]) * p + _coef[0];
        case 1:
            return ((4.0 * _coef[4] * p + 3.0 * _coef[3]) * p + 2.0 * _coef[2]) * p + _coef[1];
        case 2:
            return (12.0 * _coef[4] * p + 6.0 * _coef[3]) * p + 2.0 * _coef[2];
        case 3:
            return 24.0 * _coef[4] * p + 6.0 * _coef[3];
        case 4:
            return 24.0 * _coef[4];
        default:
            return 0.0;
    }
}

QuinticPolynomialCurve1d::QuinticPolynomialCurve1d(
    const std::array<double, 3>& start,
    const std::array<double, 3>& end,
    double param_length)
    : _param_length(std::max(1e-3, param_length))
{
    const double T = _param_length;
    const double T2 = T * T;
    const double T3 = T2 * T;
    const double T4 = T3 * T;
    const double T5 = T4 * T;

    _coef[0] = start[0];
    _coef[1] = start[1];
    _coef[2] = 0.5 * start[2];

    const double dx = end[0] - _coef[0] - _coef[1] * T - _coef[2] * T2;
    const double dv = end[1] - _coef[1] - 2.0 * _coef[2] * T;
    const double da = end[2] - 2.0 * _coef[2];

    _coef[3] = (10.0 * dx - 4.0 * dv * T + 0.5 * da * T2) / T3;
    _coef[4] = (-15.0 * dx + 7.0 * dv * T - da * T2) / T4;
    _coef[5] = (6.0 * dx - 3.0 * dv * T + 0.5 * da * T2) / T5;
}

double QuinticPolynomialCurve1d::Evaluate(int order, double p) const
{
    switch (order) {
        case 0:
            return ((((_coef[5] * p + _coef[4]) * p + _coef[3]) * p + _coef[2]) * p + _coef[1]) * p + _coef[0];
        case 1:
            return (((5.0 * _coef[5] * p + 4.0 * _coef[4]) * p + 3.0 * _coef[3]) * p + 2.0 * _coef[2]) * p + _coef[1];
        case 2:
            return ((20.0 * _coef[5] * p + 12.0 * _coef[4]) * p + 6.0 * _coef[3]) * p + 2.0 * _coef[2];
        case 3:
            return (60.0 * _coef[5] * p + 24.0 * _coef[4]) * p + 6.0 * _coef[3];
        case 4:
            return 120.0 * _coef[5] * p + 24.0 * _coef[4];
        case 5:
            return 120.0 * _coef[5];
        default:
            return 0.0;
    }
}

LatticeTrajectory1d::LatticeTrajectory1d(std::shared_ptr<Curve1d> base_curve)
    : _base_curve(std::move(base_curve))
{
}

double LatticeTrajectory1d::ParamLength() const
{
    return _base_curve ? _base_curve->ParamLength() : 0.0;
}

double LatticeTrajectory1d::Evaluate(int order, double param) const
{
    if (!_base_curve) {
        return 0.0;
    }
    const double p_len = _base_curve->ParamLength();
    if (param <= p_len) {
        return _base_curve->Evaluate(order, std::max(0.0, param));
    }

    // 超出采样参数长度时，以末端一阶导数做平滑线性外推（与 Apollo LatticeTrajectory1d 一致）
    const double p0 = _base_curve->Evaluate(0, p_len);
    const double v0 = _base_curve->Evaluate(1, p_len);
    const double a0 = _base_curve->Evaluate(2, p_len);
    const double dp = param - p_len;

    switch (order) {
        case 0:
            return p0 + v0 * dp + 0.5 * a0 * dp * dp;
        case 1:
            return v0 + a0 * dp;
        case 2:
            return a0;
        default:
            return 0.0;
    }
}

}  // namespace lattice
