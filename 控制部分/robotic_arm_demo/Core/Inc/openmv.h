#ifndef __OpenMV_H
#define	__OpenMV_H

#include "stm32f1xx.h"


void openmv_receive_data(int16_t com_data);
 
void coordinate_transformation(uint8_t center_x_temp, uint8_t center_y_temp);

#endif
