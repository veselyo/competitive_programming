#define CATCH_CONFIG_MAIN
#include "catch2/catch.hpp"
#include "projectile_path_simulator.h"

#include <vector>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <algorithm>

using namespace std;
namespace sim = projectile_path_simulator;

/* -----------------------------------------------------------
   Helpers
 -----------------------------------------------------------*/
inline std::pair<double, double> extract(const std::pair<double, double>& p) {
    return p;
}
template <typename PointLike>
inline std::pair<double, double> extract(const PointLike& p) {
    return {p.x, p.y};
}
template <typename Path>
void comparePath(const Path& actual,
                 const std::vector<std::pair<double, double>>& expected,
                 double eps = 1e-6) {
    REQUIRE(actual.size() == expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        INFO("Vertex " << i << " mismatch");
        auto a = extract(actual[i]);
        REQUIRE(a.first  == Approx(expected[i].first ).epsilon(eps));
        REQUIRE(a.second == Approx(expected[i].second).epsilon(eps));
    }
}

inline sim::Wall makeWall(double x1, double y1,
                          double x2, double y2,
                          char behaviour) {
    sim::WallBehavior wb;
    switch (behaviour) {
        case 'R': wb = sim::WallBehavior::REFLECT; break;
        case 'P': wb = sim::WallBehavior::PASS_THROUGH; break;
        case 'S': wb = sim::WallBehavior::STOP; break;
        default: throw std::invalid_argument("Invalid behavior code");
    }
    return sim::Wall{x1, y1, x2, y2, wb};
}

/* -----------------------------------------------------------
   Basic / sanity
 -----------------------------------------------------------*/
TEST_CASE("Straight line – no walls", "[basic][no-walls]") {
    std::pair<double, double> start{0.0, 0.0};
    std::pair<double, double> dir{1.0, 0.0};
    double tick   = 1.0;
    double budget = 5.0;
    std::vector<sim::Wall> walls;

    auto path = sim::ProjectilePathSimulator::simulatePath(start, dir, tick, budget, walls);
    comparePath(path, {{0.0,0.0},{5.0,0.0}});
}

TEST_CASE("Bullet stops on terminating wall", "[terminate]") {
    std::pair<double, double> start{0.0, 0.0};
    std::pair<double, double> dir{1.0, 0.0};
    double tick   = 1.0;
    double budget = 10.0;
    std::vector<sim::Wall> walls{
        makeWall(2.0, -10.0, 2.0, 10.0, 'S')
    };
    auto path = sim::ProjectilePathSimulator::simulatePath(start, dir, tick, budget, walls);
    comparePath(path, {{0.0,0.0},{2.0,0.0}});
}

TEST_CASE("Distance budget exhausted mid-segment", "[budget][edge]") {
    std::pair<double,double> start{0.0, 0.0};
    std::pair<double,double> dir{1.0, 0.0};
    double tick   = 1.0;
    double budget = 2.3;
    std::vector<sim::Wall> walls;
    auto path = sim::ProjectilePathSimulator::simulatePath(start, dir, tick, budget, walls);
    comparePath(path, {{0.0,0.0},{2.3,0.0}});
}

TEST_CASE("Input validation and error handling", "[validation][error]") {
    std::pair<double,double> start{0.0, 0.0};
    std::pair<double,double> dir{1.0, 0.0};
    std::vector<sim::Wall> walls;

    SECTION("Zero tick length is invalid") {
        REQUIRE_THROWS_AS(sim::ProjectilePathSimulator::simulatePath(start, dir, 0.0, 1.0, walls),
                          std::invalid_argument);
    }
    SECTION("Negative tick length is invalid") {
        REQUIRE_THROWS_AS(sim::ProjectilePathSimulator::simulatePath(start, dir, -1.0, 1.0, walls),
                          std::invalid_argument);
    }
    SECTION("Zero budget returns only start vertex") {
        auto p = sim::ProjectilePathSimulator::simulatePath(start, dir, 1.0, 0.0, walls);
        REQUIRE(p.size() == 1);
        REQUIRE(extract(p[0]).first  == Approx(0.0));
        REQUIRE(extract(p[0]).second == Approx(0.0));
    }
    SECTION("Negative budget is invalid") {
        REQUIRE_THROWS_AS(sim::ProjectilePathSimulator::simulatePath(start, dir, 1.0, -0.001, walls),
                          std::invalid_argument);
    }
    SECTION("Direction vector with zero magnitude is invalid") {
        std::pair<double,double> badDir{0.0, 0.0};
        REQUIRE_THROWS_AS(sim::ProjectilePathSimulator::simulatePath(start, badDir, 1.0, 1.0, walls),
                          std::invalid_argument);
    }
}

/* -----------------------------------------------------------
   Budget vs tick boundaries
 -----------------------------------------------------------*/
TEST_CASE("Budget/tick boundaries", "[budget][boundary]") {
    std::pair<double,double> start{0.0, 0.0};
    std::pair<double,double> dir{0.6, 0.8};   // already unit
    std::vector<sim::Wall> walls;

    SECTION("Budget exactly one tick") {
        double tick = 3.0, budget = 3.0;
        auto p = sim::ProjectilePathSimulator::simulatePath(start, dir, tick, budget, walls);
        comparePath(p, {{0.0,0.0},{1.8,2.4}});
    }
    SECTION("Budget slightly less than one tick") {
        double tick = 3.0, budget = 2.999999;
        auto p = sim::ProjectilePathSimulator::simulatePath(start, dir, tick, budget, walls);
        comparePath(p, {{0.0,0.0},{dir.first*budget, dir.second*budget}}, 1e-9);
    }
    SECTION("Budget slightly more than N ticks") {
        double tick = 1.0, budget = 3.000001;
        auto p = sim::ProjectilePathSimulator::simulatePath(start, dir, tick, budget, walls);
        comparePath(p, {{0.0,0.0},{3.000001*0.6, 3.000001*0.8}}, 1e-6);
    }
}

/* -----------------------------------------------------------
   Floating precision & extremes
 -----------------------------------------------------------*/
TEST_CASE("Floating-point robustness small tick", "[floating][precision]") {
    std::pair<double,double> start{0.0, 0.0};
    std::pair<double,double> dir{0.70710678118, 0.70710678119};
    double tick   = 1e-9;
    double budget = 1e-6;
    std::vector<sim::Wall> walls;

    auto p = sim::ProjectilePathSimulator::simulatePath(start, dir, tick, budget, walls);
    auto end = extract(p.back());
    double dist = std::sqrt(end.first*end.first + end.second*end.second);
    REQUIRE(dist == Approx(budget).epsilon(1e-9));
}

TEST_CASE("Extreme coordinate values (scale-aware eps)", "[edge][limits]") {
    std::pair<double,double> start{1e9, 1e9};
    std::pair<double,double> dir{1.0, 0.0};
    double tick = 10.0;
    double budget = 4.0;
    std::vector<sim::Wall> walls{
        makeWall(1e9 + 1.0, 1e9 - 100.0, 1e9 + 1.0, 1e9 + 100.0, 'R')
    };
    auto path = sim::ProjectilePathSimulator::simulatePath(start, dir, tick, budget, walls);
    comparePath(path, {
        {1e9,       1e9},
        {1e9 + 1.0, 1e9},
        {1e9 - 2.0, 1e9}
    });
}

/* -----------------------------------------------------------
   Tangency (no grazing)
 -----------------------------------------------------------*/
TEST_CASE("Grazing/tangency parallel to wall does not collide", "[tangency][no-graze]") {
    std::pair<double,double> start{0.0, 1.0};
    std::pair<double,double> dir{1.0, 0.0};
    double tick = 10.0, budget = 5.0;
    // Horizontal wall exactly at y=1 (tangent). Expect no collision, just slide past.
    std::vector<sim::Wall> walls{
        makeWall(-100.0, 1.0, 100.0, 1.0, 'S')
    };
    auto path = sim::ProjectilePathSimulator::simulatePath(start, dir, tick, budget, walls);
    comparePath(path, {{0.0,1.0},{5.0,1.0}});
}

/* -----------------------------------------------------------
   Start inside a wall: exit-only collisions
 -----------------------------------------------------------*/
TEST_CASE("Start inside REFLECT wall: first event is exit then reflection", "[inside][reflect]") {
    std::pair<double,double> start{1.0, 1.0};
    std::pair<double,double> dir{1.0, 0.0};
    double tick = 5.0, budget = 3.0;
    std::vector<sim::Wall> walls{
        makeWall(0.0, 0.0, 2.0, 2.0, 'R')
    };
    auto path = sim::ProjectilePathSimulator::simulatePath(start, dir, tick, budget, walls);
    comparePath(path, {{1.0,1.0},{2.0,1.0},{0.0,1.0}});
}

TEST_CASE("Start inside STOP wall: exit point is where we stop", "[inside][stop]") {
    std::pair<double,double> start{1.0, 1.0};
    std::pair<double,double> dir{1.0, 0.0};
    double tick = 5.0, budget = 10.0;
    std::vector<sim::Wall> walls{
        makeWall(0.0, 0.0, 2.0, 2.0, 'S')
    };
    auto path = sim::ProjectilePathSimulator::simulatePath(start, dir, tick, budget, walls);
    comparePath(path, {{1.0,1.0},{2.0,1.0}});
}

TEST_CASE("Start inside PASS_THROUGH wall: record exit and continue", "[inside][pass]") {
    std::pair<double,double> start{1.0, 1.0};
    std::pair<double,double> dir{1.0, 0.0};
    double tick = 5.0, budget = 4.0;
    std::vector<sim::Wall> walls{
        makeWall(0.0, 0.0, 2.0, 2.0, 'P')
    };
    auto path = sim::ProjectilePathSimulator::simulatePath(start, dir, tick, budget, walls);
    comparePath(path, {{1.0,1.0},{2.0,1.0},{5.0,1.0}});
}

/* -----------------------------------------------------------
   Reflection basics (axis faces)
 -----------------------------------------------------------*/
TEST_CASE("Reflect on vertical wall flips X only", "[reflect][vertical]") {
    std::pair<double,double> start{0.0, 0.0};
    std::pair<double,double> dir{1.0, 0.0};
    double tick = 10.0, budget = 5.0;
    std::vector<sim::Wall> walls{
        makeWall(2.0, -10.0, 2.0, 10.0, 'R')
    };
    auto path = sim::ProjectilePathSimulator::simulatePath(start, dir, tick, budget, walls);
    comparePath(path, {{0.0,0.0},{2.0,0.0},{-1.0,0.0}});
}

TEST_CASE("Reflect on horizontal wall flips Y only", "[reflect][horizontal]") {
    std::pair<double,double> start{0.0, 0.0};
    std::pair<double,double> dir{0.0, 1.0};
    double tick = 10.0, budget = 5.0;
    std::vector<sim::Wall> walls{
        makeWall(-10.0, 2.0, 10.0, 2.0, 'R')
    };
    auto path = sim::ProjectilePathSimulator::simulatePath(start, dir, tick, budget, walls);
    comparePath(path, {{0.0,0.0},{0.0,2.0},{0.0,-1.0}});
}

TEST_CASE("Diagonal into vertical: reflect X only, Y keeps sign", "[reflect][diagonal]") {
    std::pair<double,double> start{0.0, 0.0};
    std::pair<double,double> dir{1.0, 1.0}; // will be normalized
    double tick = 10.0, budget = 3.0;
    std::vector<sim::Wall> walls{
        makeWall(1.0, -10.0, 1.0, 10.0, 'R')
    };
    auto path = sim::ProjectilePathSimulator::simulatePath(start, dir, tick, budget, walls);
    double rt2 = std::sqrt(2.0);
    std::vector<std::pair<double,double>> expected{
        {0.0, 0.0},
        {1.0, 1.0},
        {1.0 - (3.0-rt2)/rt2, 1.0 + (3.0-rt2)/rt2}
    };
    comparePath(path, expected, 1e-6);
}

/* -----------------------------------------------------------
   Corner collisions & simultaneous faces
 -----------------------------------------------------------*/
TEST_CASE("Corner reflection on a single rectangle flips both axes", "[corner][reflect]") {
    std::pair<double,double> start{0.0, 0.0};
    std::pair<double,double> dir{1.0, 1.0};
    double tick = 10.0, budget = 3.0;
    // Rectangle with bottom-left corner at (1,1)
    std::vector<sim::Wall> walls{
        makeWall(1.0, 1.0, 3.0, 3.0, 'R')
    };
    auto path = sim::ProjectilePathSimulator::simulatePath(start, dir, tick, budget, walls);
    double rt2 = std::sqrt(2.0);
    std::vector<std::pair<double,double>> expected{
        {0.0,0.0},
        {1.0,1.0},
        {1.0 - (3.0-rt2)/rt2, 1.0 - (3.0-rt2)/rt2}
    };
    comparePath(path, expected, 1e-6);
}

TEST_CASE("Overlapping walls at same hit time use deterministic precedence STOP > REFLECT > PASS", "[tie][overlap]") {
    std::pair<double,double> start{0.0, 0.0};
    std::pair<double,double> dir{1.0, 0.0};
    double tick = 10.0, budget = 5.0;
    std::vector<sim::Wall> walls{
        makeWall(1.0, -2.0, 1.0,  2.0, 'R'),
        makeWall(1.0, -2.0, 1.0,  2.0, 'S')  // STOP should win
    };
    auto path = sim::ProjectilePathSimulator::simulatePath(start, dir, tick, budget, walls);
    comparePath(path, {{0.0,0.0},{1.0,0.0}});
}

/* -----------------------------------------------------------
   Pass-through behavior
 -----------------------------------------------------------*/
TEST_CASE("Pass-through wall records the vertex and continues unchanged", "[pass]") {
    std::pair<double,double> start{0.0, 0.0};
    std::pair<double,double> dir{1.0, 0.0};
    double tick = 10.0, budget = 5.0;
    std::vector<sim::Wall> walls{
        makeWall(2.0, -10.0, 2.0, 10.0, 'P')
    };
    auto path = sim::ProjectilePathSimulator::simulatePath(start, dir, tick, budget, walls);
    comparePath(path, {{0.0,0.0},{2.0,0.0},{5.0,0.0}});
}

TEST_CASE("Chained pass-through walls", "[pass][chain]") {
    std::pair<double,double> start{0.0, 0.0};
    std::pair<double,double> dir{1.0, 0.0};
    double tick = 10.0, budget = 5.0;
    std::vector<sim::Wall> walls{
        makeWall(2.0, -10.0, 2.0, 10.0, 'P'),
        makeWall(4.0, -10.0, 4.0, 10.0, 'P')
    };
    auto path = sim::ProjectilePathSimulator::simulatePath(start, dir, tick, budget, walls);
    comparePath(path, {{0.0,0.0},{2.0,0.0},{4.0,0.0},{5.0,0.0}});
}

/* -----------------------------------------------------------
   Sequential reflections (multiple axes)
 -----------------------------------------------------------*/
TEST_CASE("Vertical then horizontal reflection in sequence", "[reflect][sequence]") {
    std::pair<double,double> start{0.0, 0.0};
    std::pair<double,double> dir{1.0, 1.0};
    double tick = 10.0, budget = 5.0;
    // Hit x=1 (vertical REFLECT), then y=2 (horizontal REFLECT)
    std::vector<sim::Wall> walls{
        makeWall(1.0,  -10.0, 1.0,  10.0, 'R'),
        makeWall(-10.0, 2.0,  10.0, 2.0,  'R')
    };
    auto path = sim::ProjectilePathSimulator::simulatePath(start, dir, tick, budget, walls);

    double rt2  = std::sqrt(2.0);
    double rem  = budget - 2.0*rt2;
    double endX = 0.0 - rem/rt2;
    double endY = 2.0 - rem/rt2;

    std::vector<std::pair<double,double>> expected{
        {0.0,0.0},
        {1.0,1.0},
        {0.0,2.0},
        {endX, endY}
    };
    comparePath(path, expected, 1e-6);
}

/* -----------------------------------------------------------
   High-density / performance sanity
 -----------------------------------------------------------*/
TEST_CASE("High-density field of pass-throughs is handled and deterministic", "[performance][density][pass]") {
    std::pair<double,double> start{0.0, 0.0};
    std::pair<double,double> dir{1.0, 0.0};
    double tick = 2.0, budget = 6.0;
    std::vector<sim::Wall> walls;
    for (int i = 1; i <= 5; ++i) {
        walls.push_back(makeWall(i, -100.0, i, 100.0, 'P'));
    }
    auto path = sim::ProjectilePathSimulator::simulatePath(start, dir, tick, budget, walls);
    std::vector<std::pair<double,double>> expected{
        {0.0,0.0},
        {1.0,0.0},
        {2.0,0.0},
        {3.0,0.0},
        {4.0,0.0},
        {5.0,0.0},
        {6.0,0.0}
    };
    comparePath(path, expected);
}

/* -----------------------------------------------------------
   Degenerate & exact boundary cases
 -----------------------------------------------------------*/
TEST_CASE("Zero-area wall is ignored", "[edge][degenerate]") {
    std::pair<double,double> start{0.0, 0.0};
    std::pair<double,double> dir{1.0, 0.0};
    double tick = 1.0, budget = 3.0;
    std::vector<sim::Wall> walls{
        makeWall(2.0, 0.0, 2.0, 0.0, 'S') // Zero-area wall
    };
    auto path = sim::ProjectilePathSimulator::simulatePath(start, dir, tick, budget, walls);
    comparePath(path, {{0.0,0.0},{3.0,0.0}});
}

TEST_CASE("Exact boundary movement hits STOP exactly and halts", "[edge][boundary]") {
    std::pair<double,double> start{0.0, 0.0};
    std::pair<double,double> dir{1.0, 0.0};
    double tick = 1.0, budget = 2.0;
    std::vector<sim::Wall> walls{
        makeWall(2.0, -1.0, 2.0, 1.0, 'S')
    };
    auto path = sim::ProjectilePathSimulator::simulatePath(start, dir, tick, budget, walls);
    comparePath(path, {{0.0,0.0},{2.0,0.0}});
}

// REFLECT vs PASS at the same face: REFLECT should win
TEST_CASE("Overlap tie precedence REFLECT over PASS at same face", "[tie][overlap]") {
    using namespace projectile_path_simulator;
    pair<double,double> start{0.0,0.0}, dir{1.0,0.0};
    double tick=10.0, budget=5.0;
    vector<Wall> walls{
        {1.0,-2.0,1.0, 2.0, WallBehavior::PASS_THROUGH},
        {1.0,-2.0,1.0, 2.0, WallBehavior::REFLECT}
    };
    auto path = ProjectilePathSimulator::simulatePath(start, dir, tick, budget, walls);
    comparePath(path, {{0.0,0.0},{1.0,0.0},{-3.0,0.0}});
}

// Corner tie with STOP present: STOP should win (halt at the corner)
TEST_CASE("Corner tie STOP wins over REFLECT/PASS", "[tie][corner]") {
    using namespace projectile_path_simulator;
    pair<double,double> start{0.0,0.0}, dir{1.0,1.0};
    double tick=10.0, budget=3.0;
    vector<Wall> walls{
        {1.0,1.0,3.0,3.0, WallBehavior::REFLECT},     // corner at (1,1)
        { -1.0,1.0, 2.0,1.0, WallBehavior::PASS_THROUGH}, // horizontal through y=1
        { 1.0, -1.0,1.0,2.0, WallBehavior::STOP}      // vertical through x=1 (STOP)
    };
    auto path = ProjectilePathSimulator::simulatePath(start, dir, tick, budget, walls);
    comparePath(path, {{0.0,0.0},{1.0,1.0}});
}

// Corner tie REFLECT vs PASS: reflect only on the REFLECT axis (X), not both
TEST_CASE("Corner tie REFLECT vs PASS flips only the REFLECT axis", "[tie][corner][mixed]") {
    using namespace projectile_path_simulator;
    std::pair<double,double> start{0.0,0.0}, dir{1.0,1.0};
    double tick = 10.0, budget = 3.0;

    // vertical REFLECT only, horizontal PASS only
    std::vector<Wall> walls{
        { 1.0,  -10.0, 1.0,  10.0, WallBehavior::REFLECT },
        {-10.0,  1.0,  10.0,  1.0, WallBehavior::PASS_THROUGH }
    };

    auto path = ProjectilePathSimulator::simulatePath(start, dir, tick, budget, walls);

    double rt2 = std::sqrt(2.0);
    double rem = budget - rt2;    // remaining after first hit
    std::vector<std::pair<double,double>> expected{
        {0.0,0.0},
        {1.0,1.0},
        {1.0 - rem/rt2, 1.0 + rem/rt2} // flip X only
    };
    comparePath(path, expected, 1e-6);
}

// Pass-through then reflect within the same tick
TEST_CASE("Pass-through then reflect in same tick", "[tick][mixed]") {
    using namespace projectile_path_simulator;
    std::pair<double,double> start{0.0,0.0}, dir{1.0,0.0};
    double tick=10.0, budget=5.0;
    std::vector<Wall> walls{
        {1.0,-2.0,1.0, 2.0, WallBehavior::PASS_THROUGH},
        {2.0,-2.0,2.0, 2.0, WallBehavior::PASS_THROUGH},
        {3.0,-2.0,3.0, 2.0, WallBehavior::REFLECT}
    };
    auto path = ProjectilePathSimulator::simulatePath(start, dir, tick, budget, walls);
    comparePath(path, {{0.0,0.0},{1.0,0.0},{3.0,0.0},{2.0,0.0},{1.0,0.0}});
}
