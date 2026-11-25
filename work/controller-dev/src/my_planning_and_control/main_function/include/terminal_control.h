#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace terminal_control {
inline double bound(double value, double low, double high) {
    return std::max(low, std::min(high, value));
}
struct RoutePosition {
    double remaining = std::numeric_limits<double>::infinity();
    double lateral = std::numeric_limits<double>::infinity();
    double goal_distance = std::numeric_limits<double>::infinity();
};
// Project onto the finite route, preserving arc length around corners.
inline RoutePosition locate(const std::vector<std::pair<double,double>>& route,
                            double x, double y) {
    RoutePosition result;
    if (route.size() < 2) return result;
    double total = 0., progress = 0., station = 0.;
    for (size_t i=1; i<route.size(); ++i)
        total += std::hypot(route[i].first-route[i-1].first,
                            route[i].second-route[i-1].second);
    for (size_t i=1; i<route.size(); ++i) {
        const double dx=route[i].first-route[i-1].first;
        const double dy=route[i].second-route[i-1].second;
        const double length=std::hypot(dx,dy);
        if (length < 1e-6) continue;
        const double fraction=bound(((x-route[i-1].first)*dx+
                                      (y-route[i-1].second)*dy)/(length*length),0.,1.);
        const double distance=std::hypot(x-route[i-1].first-fraction*dx,
                                         y-route[i-1].second-fraction*dy);
        if (distance < result.lateral) {
            result.lateral=distance;
            progress=station+fraction*length;
        }
        station+=length;
    }
    if (std::isfinite(result.lateral)) result.remaining=std::max(0.,total-progress);
    result.goal_distance=std::hypot(x-route.back().first,y-route.back().second);
    return result;
}
// Terminal speed envelope: 1.5 m/s^2 nominal deceleration, stop 1 m before end.
inline double target_speed(double remaining) {
    return std::sqrt(3.0*std::max(0.,remaining-1.0));
}
}
