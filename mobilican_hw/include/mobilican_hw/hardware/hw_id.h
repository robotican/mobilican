//
// Created by sub on 12/19/18.
//

#ifndef MOBILICAN_HW_HW_ID_H
#define MOBILICAN_HW_HW_ID_H

#include <stdint.h>

// id_type represent the robot hardware id.
// 4 LSB in hardware id represent the robot model
// and version. e.g. : if the hardware id is 
// 0x68560102, then model number is 01, and version 
// is 02. This hw id belongs to Lizi_2 (lizi=model 01)
typedef uint32_t id_type;

#endif //MOBILICAN_HW_HW_ID_H
