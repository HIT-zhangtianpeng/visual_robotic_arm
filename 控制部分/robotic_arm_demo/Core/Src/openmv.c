#include "openmv.h"
#include "stdio.h"
#include "usart.h"
/*�ĸ��������ڴ��Ŀ�������ɫ�������Լ���������*/

uint8_t  center_x = 0, center_y = 0;
uint8_t color_type = 0;
double  center_x_cm = 0, center_y_cm = 0;
/*���ݽ��պ���*/

void openmv_receive_data(int16_t com_data)
{
    /*ѭ�������*/
    uint8_t i;
    /*��������*/
    static uint8_t rx_counter_1 = 0;//����
    /*���ݽ�������*/
    static uint16_t rx_buffer_1[10] = { 0 };
    /*���ݴ���״̬λ*/
    static uint8_t rx_state = 0;

    /*�����ݽ���У׼���ж��Ƿ�Ϊ��Ч����*/
    if (rx_state == 0 && com_data == 0x2C)  //0x2c֡ͷ
    {
        rx_state = 1;
        rx_buffer_1[rx_counter_1++] = com_data;

    }

    else if (rx_state == 1 && com_data == 0x12)  //0x12֡ͷ
    {
        rx_state = 2;
        rx_buffer_1[rx_counter_1++] = com_data;

    }
    else if (rx_state == 2)
    {

        rx_buffer_1[rx_counter_1++] = com_data;
        if (rx_counter_1 >= 6 || com_data == 0xFF)   //RxBuffer1��������,�������ݽ���
        //ʹ��0xFF��Ϊ����λ��ԭ���ǣ�����OpenMV�ش������������λ�غϵ��½�����ǰ����
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
            printf("��ɫ����: %c   ", color_type);
            printf("����x���꣺%d   ", center_x);
            printf("����y���꣺%d\r\n   ", center_y);

        }
    }

    else if (rx_state == 3)//����Ƿ���ܵ�������־
    {
        if (rx_buffer_1[rx_counter_1 - 1] == 0x5B)
        {
            //RxFlag1 = 0;
            rx_counter_1 = 0;
            rx_state = 0;
        }
        else   //���մ���
        {
            rx_state = 0;
            rx_counter_1 = 0;
            for (i = 0; i < 10; i ++)
            {
                rx_buffer_1[i] = 0x00;      //�����������������
            }
        }
    }

    else   //�����쳣
    {
        rx_state = 0;
        rx_counter_1 = 0;
        for (i = 0; i < 10; i++)
        {
            rx_buffer_1[i] = 0x00;      //�����������������
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

