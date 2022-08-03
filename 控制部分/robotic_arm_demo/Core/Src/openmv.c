#include "openmv.h"
#include "stdio.h"
#include "usart.h"
/*四个变量用于存放目标物体的色彩种类以及中心坐标*/

uint8_t  center_x = 0, center_y = 0;
uint8_t color_type = 0;
double  center_x_cm = 0, center_y_cm = 0;
/*数据接收函数*/

void openmv_receive_data(int16_t com_data)
{
    /*循环体变量*/
    uint8_t i;
    /*计数变量*/
    static uint8_t rx_counter_1 = 0;//计数
    /*数据接收数组*/
    static uint16_t rx_buffer_1[10] = { 0 };
    /*数据传输状态位*/
    static uint8_t rx_state = 0;

    /*对数据进行校准，判断是否为有效数据*/
    if (rx_state == 0 && com_data == 0x2C)  //0x2c帧头
    {
        rx_state = 1;
        rx_buffer_1[rx_counter_1++] = com_data;

    }

    else if (rx_state == 1 && com_data == 0x12)  //0x12帧头
    {
        rx_state = 2;
        rx_buffer_1[rx_counter_1++] = com_data;

    }
    else if (rx_state == 2)
    {

        rx_buffer_1[rx_counter_1++] = com_data;
        if (rx_counter_1 >= 6 || com_data == 0xFF)   //RxBuffer1接受满了,接收数据结束
        //使用0xFF作为结束位的原因是，避免OpenMV回传的数据与结束位重合导致接收提前结束
        {
            rx_state = 3;
            color_type = rx_buffer_1[rx_counter_1 - 4];
            center_x = rx_buffer_1[rx_counter_1 - 3];
            center_y = rx_buffer_1[rx_counter_1 - 2];
            //    				for(k = 0;k < RxCounter1;k ++)
            //						{
            //							printf("%d ",RxBuffer1[k]);
            //						}
            //						printf("\r\n\n");
            printf("颜色种类: %c   ", color_type);
            printf("中心x坐标：%d   ", center_x);
            printf("中心y坐标：%d\r\n   ", center_y);

        }
    }

    else if (rx_state == 3)//检测是否接受到结束标志
    {
        if (rx_buffer_1[rx_counter_1 - 1] == 0x5B)
        {
            //RxFlag1 = 0;
            rx_counter_1 = 0;
            rx_state = 0;
        }
        else   //接收错误
        {
            rx_state = 0;
            rx_counter_1 = 0;
            for (i = 0; i < 10; i ++)
            {
                rx_buffer_1[i] = 0x00;      //将存放数据数组清零
            }
        }
    }

    else   //接收异常
    {
        rx_state = 0;
        rx_counter_1 = 0;
        for (i = 0; i < 10; i++)
        {
            rx_buffer_1[i] = 0x00;      //将存放数据数组清零
        }
    }
}


void coordinate_transformation(uint8_t center_x_temp, uint8_t center_y_temp)
{
    double ratio_x = 51.1 / 160;
    double ratio_y = 38.2 / 120;
    center_x_cm = (center_x - 80) * ratio_x;
    center_y_cm = (107 - center_y) * ratio_y;
}

