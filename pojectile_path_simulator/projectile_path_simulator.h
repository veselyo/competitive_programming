#ifndef PROJECTILE_PATH_SIMULATOR_H
#define PROJECTILE_PATH_SIMULATOR_H

#include <vector>
#include <utility>
#include <tuple>

namespace projectile_path_simulator {

enum class WallBehavior {
    REFLECT = 'R',
    PASS_THROUGH = 'P',
    STOP = 'S'
};

struct Wall {
    double x1, y1; 
    double x2, y2; 
    WallBehavior behavior;
};

class ProjectilePathSimulator {
public:
    ProjectilePathSimulator(double speed, double distance_budget);

    void add_wall(double x1, double y1, double x2, double y2, WallBehavior behavior);

    
    
    std::vector<std::pair<double, double>> simulate(double start_x, double start_y,
                                                    double direction_x, double direction_y);

    static std::vector<std::pair<double, double>> simulatePath(
        std::pair<double, double> start,
        std::pair<double, double> direction,   
        double speed,
        double distance_budget,
        const std::vector<Wall>& walls);

private:
    double speed_;
    double distance_budget_;
    std::vector<Wall> walls_;
};

} 

#endif 
