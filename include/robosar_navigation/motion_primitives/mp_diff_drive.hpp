// Created by Indraneel on 17/04/22

#ifndef MP_DIFF_DRIVE_HPP
#define MP_DIFF_DRIVE_HPP

#include <math.h>

class MPDiffDrive {

public:
    MPDiffDrive() : rotation_speed_rads(0.5), translation_speed_ms(0.2), translation_duration_s(1.0)
    {
        rotation_angles = {0.0, 45.0, 90.0, 135.0, 180.0, -45.0, -90.0, -135.0};
    }

    ~MPDiffDrive() {};

    std::vector<double> turn_in_place_action(std::vector<double> point, double angle) {

        point[2] += angle;
        // constrain angle to -180,180
        point[2] = remainder(angle,2.0*M_PI);
        point[3] += angle/rotation_speed_rads;

        return point;
    } 

    std::vector<double> forward_move_action(std::vector<double> point, double duration) {

        double move_distance = duration*translation_speed_ms;
        point[0] += move_distance*cos(point[2]);
        point[1] += move_distance*sin(point[2]);
        point[3] += duration;

        return point;
    } 

     std::vector<double> wait_in_place_action(std::vector<double> point, double duration) {

        point[3] += duration;

        return point;
    } 

    double toRadians(double angle_degrees)
    {
        return (angle_degrees*M_PI)/180.0;
    }

    std::vector<std::vector<double>> propogate_model(std::vector<double> point) {

        std::vector<std::vector<double>> next_states;
        for(auto angle:rotation_angles) {

            std::vector<double> next_state = point;
            next_state = turn_in_place_action(next_state,toRadians(angle));
            next_state = forward_move_action(next_state, translation_duration_s);
            next_states.push_back(next_state);
        }

        // Wait in place TODO

        return next_states;
    }

private:
    double rotation_speed_rads;
    double translation_speed_ms;
    double translation_duration_s;
    std::vector<double> rotation_angles;

};


#endif // MP_DIFF_DRIVE_HPP