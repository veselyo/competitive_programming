#include "projectile_path_simulator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace projectile_path_simulator {

// ------------------------------- Utilities ----------------------------------

static inline double vec_len(double x, double y) {
    return std::hypot(x, y);
}

static inline void normalize(double& x, double& y) {
    double L = vec_len(x, y);
    if (L == 0.0) return;
    x /= L; y /= L;
}

static inline double scale_for(double a, double b, double c, double d) {
    return std::max({1.0, std::fabs(a), std::fabs(b), std::fabs(c), std::fabs(d)});
}

static inline bool within(double v, double lo, double hi, double eps) {
    return (v >= lo - eps) && (v <= hi + eps);
}

struct SideHit {
    bool hit = false;
    double dist = 0.0;               // distance along ray (not param t) to impact
    double x = 0.0, y = 0.0;         // impact point
    bool vertical = false;           // hit a vertical face
    bool horizontal = false;         // hit a horizontal face
    WallBehavior behavior = WallBehavior::PASS_THROUGH;
};

static inline void maybe_add_vertical(double x_side,
                                      const Wall& w,
                                      double sx, double sy,
                                      double dx, double dy,
                                      double maxDist,
                                      double eps_dir, double eps_face, double eps_d,
                                      std::vector<SideHit>& out)
{
    if (std::fabs(dx) <= eps_dir) return;             // parallel -> no collide
    double s = (x_side - sx) / dx;                    // distance along ray
    if (s <= eps_d || s > maxDist + eps_d) return;    // only future, within this step
    double y = sy + dy * s;
    if (!within(y, w.y1, w.y2, eps_face)) return;     // not on segment span
    SideHit h; h.hit = true; h.dist = s; h.x = x_side; h.y = y;
    h.vertical = true; h.horizontal = false; h.behavior = w.behavior;
    out.push_back(h);
}

static inline void maybe_add_horizontal(double y_side,
                                        const Wall& w,
                                        double sx, double sy,
                                        double dx, double dy,
                                        double maxDist,
                                        double eps_dir, double eps_face, double eps_d,
                                        std::vector<SideHit>& out)
{
    if (std::fabs(dy) <= eps_dir) return;             // parallel -> no collide
    double s = (y_side - sy) / dy;
    if (s <= eps_d || s > maxDist + eps_d) return;
    double x = sx + dx * s;
    if (!within(x, w.x1, w.x2, eps_face)) return;
    SideHit h; h.hit = true; h.dist = s; h.x = x; h.y = y_side;
    h.vertical = false; h.horizontal = true; h.behavior = w.behavior;
    out.push_back(h);
}

// ------------------------------ Implementation ------------------------------

ProjectilePathSimulator::ProjectilePathSimulator(double speed, double distance_budget)
    : speed_(speed), distance_budget_(distance_budget) {
    if (speed <= 0) throw std::invalid_argument("Speed must be positive");
    if (distance_budget < 0) throw std::invalid_argument("Distance budget must be non-negative");
}

void ProjectilePathSimulator::add_wall(double x1, double y1, double x2, double y2, WallBehavior behavior) {
    // Ignore true zero-area (single point) "walls"
    if (std::abs(x1 - x2) < 1e-12 && std::abs(y1 - y2) < 1e-12) return;
    walls_.push_back({std::min(x1, x2), std::min(y1, y2), std::max(x1, x2), std::max(y1, y2), behavior});
}

std::vector<std::pair<double, double>> ProjectilePathSimulator::simulatePath(
    std::pair<double, double> start,
    std::pair<double, double> direction,
    double speed,
    double distance_budget,
    const std::vector<Wall>& walls)
{
    if (speed <= 0) throw std::invalid_argument("Speed must be positive");
    if (distance_budget < 0) throw std::invalid_argument("Distance budget must be non-negative");
    double L = vec_len(direction.first, direction.second);
    if (L == 0.0) throw std::invalid_argument("Direction vector must not be zero");
    direction.first /= L; direction.second /= L;

    ProjectilePathSimulator sim(speed, distance_budget);
    for (const auto& w : walls) {
        if (std::abs(w.x1 - w.x2) > 1e-12 || std::abs(w.y1 - w.y2) > 1e-12) {
            sim.add_wall(w.x1, w.y1, w.x2, w.y2, w.behavior);
        }
    }
    return sim.simulate(start.first, start.second, direction.first, direction.second);
}

std::vector<std::pair<double, double>> ProjectilePathSimulator::simulate(
    double start_x, double start_y, double direction_x, double direction_y)
{
    // Ensure direction is normalized even if user calls simulate directly.
    normalize(direction_x, direction_y);

    std::vector<std::pair<double, double>> path;
    path.emplace_back(start_x, start_y);

    double px = start_x, py = start_y;
    double dx = direction_x, dy = direction_y;

    double remaining_budget = distance_budget_;

    // Global numeric tolerances (scale-aware)
    auto scale_now = [&]() {
        return scale_for(px, py, px + dx * std::max(1.0, speed_), py + dy * std::max(1.0, speed_));
    };
    const double ulp = std::numeric_limits<double>::epsilon();

    // Iteration bounds: ticks + allowance for collisions per tick
    const int max_outer = static_cast<int>(std::ceil(distance_budget_ / std::max(1e-12, speed_))) + 2;
    const int max_inner_per_tick = 256; // generous allowance for many collisions in one tick

    for (int outer = 0; outer < max_outer && remaining_budget > 0.0; ++outer) {
        bool recorded_pass_before_nonpass = false;

        double remaining_in_tick = std::min(speed_, remaining_budget);

        for (int inner = 0; inner < max_inner_per_tick && remaining_in_tick > 0.0; ++inner) {
            double sscale = scale_now();
            const double eps_face = 64.0 * ulp * sscale;
            const double eps_dir  = 64.0 * ulp;                  // for direction components
            const double eps_d    = 64.0 * ulp * (1.0 + speed_); // min positive travel distance
            const double eps_tie  = 128.0 * ulp * (1.0 + speed_);
            const double eps_push = 1024.0 * ulp * (1.0 + speed_);

            // Gather all first-side hits among all walls within this tick distance.
            std::vector<SideHit> candidates;

            for (const auto& w : walls_) {
                // Vertical sides
                if (std::abs(w.x1 - w.x2) <= 1e-12) {
                    maybe_add_vertical(w.x1, w, px, py, dx, dy, remaining_in_tick,
                                       eps_dir, eps_face, eps_d, candidates);
                } else {
                    maybe_add_vertical(w.x1, w, px, py, dx, dy, remaining_in_tick,
                                       eps_dir, eps_face, eps_d, candidates);
                    maybe_add_vertical(w.x2, w, px, py, dx, dy, remaining_in_tick,
                                       eps_dir, eps_face, eps_d, candidates);
                }
                // Horizontal sides
                if (std::abs(w.y1 - w.y2) <= 1e-12) {
                    maybe_add_horizontal(w.y1, w, px, py, dx, dy, remaining_in_tick,
                                         eps_dir, eps_face, eps_d, candidates);
                } else {
                    maybe_add_horizontal(w.y1, w, px, py, dx, dy, remaining_in_tick,
                                         eps_dir, eps_face, eps_d, candidates);
                    maybe_add_horizontal(w.y2, w, px, py, dx, dy, remaining_in_tick,
                                         eps_dir, eps_face, eps_d, candidates);
                }
            }

            if (candidates.empty()) {
                // No collision in this tick: if the remainder is tiny, swallow it.
                if (remaining_in_tick <= eps_d) {
                    remaining_budget -= remaining_in_tick;
                    remaining_in_tick = 0.0;
                    break;
                }
                px += dx * remaining_in_tick;
                py += dy * remaining_in_tick;
                remaining_budget -= remaining_in_tick;
                remaining_in_tick = 0.0;
                break;
            }

            // Determine earliest event respecting pass-through + non-pass batching rules.
            double s_np_min = std::numeric_limits<double>::infinity(); // earliest REFLECT/STOP
            double s_p_min  = std::numeric_limits<double>::infinity(); // earliest PASS_THROUGH
            for (const auto& h : candidates) {
                if (h.behavior == WallBehavior::PASS_THROUGH) {
                    if (h.dist < s_p_min) s_p_min = h.dist;
                } else {
                    if (h.dist < s_np_min) s_np_min = h.dist;
                }
            }

            double s_min = std::numeric_limits<double>::infinity();
            if (std::isfinite(s_np_min)) {
                // There is a non-pass ahead in this tick. To avoid flooding with pass-throughs
                // before the important event, we allow at most one pass-through before it.
                if (!recorded_pass_before_nonpass && std::isfinite(s_p_min) && (s_p_min + eps_tie < s_np_min)) {
                    s_min = s_p_min;     // first PASS_THROUGH before the REFLECT/STOP
                } else {
                    s_min = s_np_min;    // jump to the REFLECT/STOP
                }
            } else {
                // No non-pass ahead: process pass-throughs normally
                s_min = s_p_min;
            }
// Collect all hits at the same earliest time (within eps_tie)
            std::vector<SideHit> hits;
            for (auto& h : candidates) {
                if (std::fabs(h.dist - s_min) <= eps_tie) hits.push_back(h);
            }

            // Impact point (averaging guards against tiny numerical spread)
            double ix = 0.0, iy = 0.0;
            for (auto& h : hits) { ix += h.x; iy += h.y; }
            ix /= static_cast<double>(hits.size());
            iy /= static_cast<double>(hits.size());

            // Consume distance
            double step_used = std::min(s_min, remaining_in_tick);
            px = ix; py = iy;
            remaining_in_tick -= step_used;
            remaining_budget  -= step_used;

            // Determine behavior precedence & axes at this instant
            bool anyStop = false, anyReflectV = false, anyReflectH = false;
            for (const auto& h : hits) {
                if (h.behavior == WallBehavior::STOP) anyStop = true;
                else if (h.behavior == WallBehavior::REFLECT) {
                    if (h.vertical)   anyReflectV = true;
                    if (h.horizontal) anyReflectH = true;
                }
            }

            // Record the vertex (collision / pass-through event)
            path.emplace_back(ix, iy);

            if (anyStop) return path;

            // Apply reflections (PASS_THROUGH implies no change)
            if (anyReflectV) dx = -dx;
            if (anyReflectH) dy = -dy;

                        // Update batching state: reset after any non-pass; otherwise we recorded a pass-through
            if (anyStop || anyReflectV || anyReflectH) {
                recorded_pass_before_nonpass = false;
            } else {
                recorded_pass_before_nonpass = true;
            }

// Only nudge if we truly continue (avoid tail micro-steps creating extra vertices)
            const bool will_continue = (remaining_in_tick > 10.0 * eps_d) && (remaining_budget > 10.0 * eps_d);
            if (will_continue) {
                px += dx * eps_push;
                py += dy * eps_push;
            }
        }
        // next tick if budget remains
    }

    // Avoid adding a near-duplicate final vertex (swallow tiny tail movement)
    const double sscale_final = scale_for(px, py, px, py);
    const double eps_push_final = 1024.0 * std::numeric_limits<double>::epsilon() * (1.0 + speed_);
    const double eps_out = std::max(64.0 * std::numeric_limits<double>::epsilon() * sscale_final,
                                    16.0 * eps_push_final);

    if (path.empty()
        || std::fabs(px - path.back().first)  > eps_out
        || std::fabs(py - path.back().second) > eps_out) {
        path.emplace_back(px, py);
    }
    return path;
}

} // namespace projectile_path_simulator
