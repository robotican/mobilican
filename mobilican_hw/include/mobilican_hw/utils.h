//
// Created by sub on 12/4/18.
//

#ifndef MOBILICAN_HW_UTILS_H
#define MOBILICAN_HW_UTILS_H

#include <ros/ros.h>

namespace Utils
{
    static void terminateNode(const char * msg)
    {
        ROS_FATAL("%s", msg);
        ros::shutdown();
        exit(EXIT_FAILURE);
    }

    static double ticksToRads(int ticks, uint ticks_per_round)
    {
        return ((double)ticks / (double)ticks_per_round) * 2.0 * M_PI;
    }
}


#endif //MOBILICAN_HW_UTILS_H
