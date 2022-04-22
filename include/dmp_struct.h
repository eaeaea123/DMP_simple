#ifndef DMP_STRUCT_H_
#define DMP_STRUCT_H_

#include <iostream>
#include <string>
#include <vector>
#include <map>

namespace dmp
{
    struct DMPPoint
    {
        std::vector<double> positions;
        std::vector<double> velocities;
    };

    struct DMPData
    {
        double k_gain;
        double d_gain;
        std::vector<double> weights;
        std::vector<double> f_domain;
        std::vector<double> f_targets;
    };

    struct DMPTraj
    {
        std::vector<DMPPoint> points;
        std::vector<double> times;
    };
        
}

#endif