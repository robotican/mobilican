//
// Created by sub on 12/5/18.
//

#ifndef MOBILICAN_HW_ROBOTBUILDER_H
#define MOBILICAN_HW_ROBOTBUILDER_H

#include "mobile_robot.h"
#include "mobilican_hw/robots_impl/lizi_2/lizi_2.h"

namespace RobotBuilder
{
    static MobileRobot* build(ros::NodeHandle &nh,
                                   uint16_t hardware_id,
                                    RicClient ric_client)
    {
        switch (hardware_id)
        {
            case (Lizi_2::getModelHardwareId()):
                return new Lizi_2(nh, ric_client);
        }
        return nullptr;
    }
}


#endif //MOBILICAN_HW_ROBOTBUILDER_H
