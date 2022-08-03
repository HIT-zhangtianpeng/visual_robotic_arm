/************************************************************************************************
* 程序版本：V1.0
* 程序日期：2022-7-2
* 程序作者：719飞行器实验室： 张天鹏、刘胜昔、赵艺超						      
************************************************************************************************/
#include "tim.h"
#include "servo.h"
#include "math.h"
#include "stdio.h"

extern double pulse_1;
extern double pulse_2;
extern double pulse_3;
extern double pulse_4;
extern double pulse_5;
extern double pulse_6;

extern double angle_1;
extern double angle_2;
extern double angle_3;
extern double angle_4;
extern double angle_5;
extern double angle_6;

extern double now_angle_1;
extern double now_angle_2;
extern double now_angle_3;
extern double now_angle_4;
extern double now_angle_5;
extern double now_angle_6;

extern double target_angle_1;
extern double target_angle_2;
extern double target_angle_3;
extern double target_angle_4;
extern double target_angle_5;
extern double target_angle_6;

extern double R;        //底座半径
extern double length_1; //底座高度
extern double length_2; //机械臂1长度
extern double length_3; //机械臂2长度
extern double length_4; //机械臂3长度

extern double j_all; //机械臂3长度


extern double l0;
extern double l1;
extern double l2;

extern float pi;

extern uint8_t center_x ,center_y ,color_type ;

extern double center_x_cm,center_y_cm;

double a1;
double a2;
double a3;
double a4;
double a5;
double a6;
    
double abs_angle_error_1;
double abs_angle_error_2;
double abs_angle_error_3;
double abs_angle_error_4;
double abs_angle_error_5;
double abs_angle_error_6;

/**************************************************************************
函数功能：开启PWM输出
入口参数：无
返回  值：无
备注    ：无
**************************************************************************/
void pwm_start(void)
{
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);//PWM_1
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);//PWM_2
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);//PWM_3
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);//PWM_4
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);//PWM_5
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);//PWM_6
}
/**************************************************************************
函数功能：将角度转换为脉宽，进而改变PWM输出占空比
入口参数：角度
返回  值：无
备注    ：角度范围为-90度到90度，当输入角度为-90度时，脉宽为0.5ms；当输入角度为0度时，脉宽为1.5ms；当输入角度为90度时，脉宽为2.5ms；
**************************************************************************/
//void translate_angle_to_pulse(double angle_1,double angle_2,double angle_3,double angle_4,double angle_5,double angle_6)
//{
//	pulse_1 = (((angle_1 + 90) / 90 ) + 0.5)*(20000/20);//参与姿态解算，控制旋转角度，范围（-90，90）
//	pulse_2 = (((angle_2 + 90) / 90 ) + 0.5)*(20000/20);//参与姿态解算，控制倾转角度，范围（-90，90）
//	pulse_3 = (((angle_3 + 90) / 90 ) + 0.5)*(20000/20);//参与姿态解算，控制倾转角度，范围（-90，90）
//	pulse_4 = (((angle_4 + 90) / 90 ) + 0.5)*(20000/20);//参与姿态解算，控制倾转角度，范围（-90，90）
//	pulse_5 = (((angle_5 + 90) / 90 ) + 0.5)*(20000/20);//控制末端控制器旋转角度，可以忽略，范围（-90，90）
//	pulse_6 = (((angle_6 + 90) / 90 ) + 0.5)*(20000/20);//控制末端控制器抓取，范围（-90，90）
//}


void translate_angle_to_pulse(double angle_1,double angle_2,double angle_3,double angle_4,double angle_5,double angle_6)
{
	pulse_1 = (((angle_1 + 70) / 90 ) + 0.5)*(20000/20);
	pulse_2 = (((angle_2 +90) / 90 ) + 0.5)*(20000/20);//正向
	pulse_3 = (((angle_3) / 90 ) + 0.5)*(20000/20);//反向
	pulse_4 = ((((-angle_4 + 90) / 270 * 180) / 90) + 0.5)*(20000/20);//正向
	pulse_5 = ((((angle_5 / 90 * 65) + 90) / 90 ) + 0.5)*(20000/20);
	pulse_6 = (((angle_6 ) / 90 ) + 0.5)*(20000/20);
}


/**************************************************************************
函数功能：底层PWM输出
入口参数：舵机目标角度
返回  值：无
备注    ：舵机控制频率为50Hz，周期为20ms，PSC = 72 - 1，ARR = 200 - 1，f = 72MHz /( PSC + 1 )( ARR +1 )
				占空比 = pulse / ARR
				当高电平时间为0.5ms时，舵机角度为0度
				当高电平时间为1.5ms时，舵机角度为90度
				当高电平时间为2.5ms时，舵机角度为180度
**************************************************************************/
void pwm_out(double angle_1, double angle_2, double angle_3, double angle_4, double angle_5,  double angle_6)
{
	translate_angle_to_pulse(angle_1,angle_2,angle_3,angle_4,angle_5,angle_6);
	
	if(pulse_1 != NULL)
	{
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, pulse_1);
	}
	if(pulse_2 != NULL)
	{
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, pulse_2);
	}
	if(pulse_3 != NULL)
	{
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, pulse_3);
	}
	if(pulse_4 != NULL)
	{
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pulse_4);
	}
	if(pulse_5 != NULL)
	{
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, pulse_5);
	}
	if(pulse_6 != NULL)
	{
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, pulse_6);
	}
}
/**************************************************************************
函数功能：根据目标位置，计算目标角度
入口参数：末端执行器姿态
返回  值：无
备注    ：target_x：目标距离原点距离；target_y：目标高度；
          alpha：末端控制器角度；target_angle：目标角度

侧视图                         俯视图

target_y                        90度
|                                |
|                                |
|______target_x      180度_______|________0度

**************************************************************************/
void servo_angle_calculate(float target_x, float target_y, float target_z)
{
    if (target_y >= 18)
        target_y = 18;
    else if(target_y <= 3)
        target_y = 3;
	float len_1, len_2, len_3, len_4;   //a1为底部圆台高度 剩下三个为三个机械臂长度 
	float j1,j2,j3,j4 ;   //四个姿态角
	float L, H, bottom_r;					//	L =	a2*sin(j2) + a3*sin(j2 + j3);H = a2*cos(j2) + a3*cos(j2 + j3); P为底部圆盘半径R
	float j_sum;			//j2,j3,j4之和
	float len, high;   //总长度,总高度
	float cos_j3, sin_j3; //用来存储cosj3,sinj3数值
	float cos_j2, sin_j2;
	float k1, k2;
	int i;
	float n, m;
	n = 0;
	m = 0;

	//输入初始值
	bottom_r = 14; 		//底部圆盘半径
	len_1 = 12; 	//底部圆盘高度
	//机械臂长度
	len_2 = 12;
	len_3 = 12;
	len_4 = 12;

	if (target_x == 0)
		j1 = 90;
	else
		j1 = 90 - atan(target_x / (target_y + bottom_r)) * (57.3);


	for (i = 0; i <= 180; i ++)
	{
		j_sum = 3.1415927 * i / 180;

		len = sqrt((target_y + bottom_r) * (target_y + bottom_r) + target_x * target_x);
		high = target_z;

		L = len - len_4 * sin(j_sum);
		H = high - len_4 * cos(j_sum) - len_1;

		cos_j3 = ((L * L) + (H * H) - ((len_2) * (len_2)) - ((len_3) * (len_3))) / (2 * (len_2) * (len_3));
		sin_j3 = (sqrt(1 - (cos_j3) * (cos_j3)));

		j3 = atan((sin_j3) / (cos_j3)) * (57.3);

		k2 = len_3 * sin(j3 / 57.3);
		k1 = len_2 + len_3 * cos(j3 / 57.3);

		cos_j2 = (k2 * L + k1 * H) / (k1 * k1 + k2 * k2);
		sin_j2 = (sqrt(1 - (cos_j2) * (cos_j2)));

		j2 = atan((sin_j2) / (cos_j2)) * 57.3;
		j4 = j_sum * 57.3 - j2 - j3;

		if (j2 >= 0 && j3 >= 0 && j4 >= -90 && j2 <= 180 && j3 <= 180 && j4 <= 90)
		{
			n ++;
		}
	}


	for (i = 0; i <= 180; i ++)
	{
		j_sum = 3.1415927 * i / 180;

		len = sqrt((target_y + bottom_r) * (target_y + bottom_r) + target_x * target_x);
		high = target_z;

		L = len - len_4 * sin(j_sum);
		H = high - len_4 * cos(j_sum) - len_1;

		cos_j3 = ((L * L) + (H * H) - ((len_2) * (len_2)) - ((len_3) * (len_3))) / (2 * (len_2) * (len_3));
		sin_j3 = (sqrt(1 - (cos_j3) * (cos_j3)));

		j3 = atan((sin_j3) / (cos_j3)) * (57.3);

		k2 = len_3 * sin(j3 / 57.3);
		k1 = len_2 + len_3 * cos(j3 / 57.3);

		cos_j2 = (k2 * L + k1 * H) / (k1 * k1 + k2 * k2);
		sin_j2 = (sqrt(1 - (cos_j2) * (cos_j2)));

		j2 = atan((sin_j2) / (cos_j2)) * 57.3;
		j4 = j_sum * 57.3 - j2 - j3;

		if (j2 >= 0 && j3 >= 0 && j4 >= -90 && j2 <= 180 && j3 <= 180 && j4 <= 90)
		{
			m ++;
			if (m == n / 2 || m == (n + 1) / 2)
				break;			
		}
	}
	target_angle_1 = j1;
	target_angle_2 = j2;
	target_angle_3 = j3;
	target_angle_4 = j4;

	printf("center_x: %f\r\n center_y: %f\r\n taret_angle_1: %f\r\n taret_angle_2: %f\r\n taret_angle_3: %f\r\n taret_angle_4: %f\r\n",target_x,target_y,j1,j2,j3,j4);
}




/**************************************************************************
函数功能：控制机械爪抓取目标
入口参数：无
返回  值：无
备注    ：无
**************************************************************************/
void servo_control(double temp_target_angle_1,double temp_target_angle_2,double temp_target_angle_3,double temp_target_angle_4,double temp_target_angle_5,double temp_target_angle_6)
{
	
//	 abs_angle_error_1 = fabs(temp_target_angle_1 - now_angle_1);   //误差绝对值
     abs_angle_error_2 = fabs(temp_target_angle_2 - now_angle_2);   //误差绝对值
     abs_angle_error_3 = fabs(temp_target_angle_3 - now_angle_3);   //误差绝对值
     abs_angle_error_4 = fabs(temp_target_angle_4 - now_angle_4);   //误差绝对值
     abs_angle_error_5 = fabs(temp_target_angle_5 - now_angle_5);   //误差绝对值
     abs_angle_error_6 = fabs(temp_target_angle_6 - now_angle_6);   //误差绝对值
    
//   a1 = abs_angle_error_1;
     a2 = abs_angle_error_2;
     a3 = abs_angle_error_3;
     a4 = abs_angle_error_4;
     a5 = abs_angle_error_5;
     a6 = abs_angle_error_6;  
    
//    if(temp_target_angle_1 != NULL)
//	{
//        for(;abs_angle_error_1 >= 3;abs_angle_error_1 --)
//        {
//            double pwm_angle_1 = ((temp_target_angle_1 > now_angle_1) ? (temp_target_angle_1 - abs_angle_error_1) : (temp_target_angle_1 + abs_angle_error_1));
//            

//            pwm_out(pwm_angle_1,now_angle_2,now_angle_3,now_angle_4,now_angle_5,now_angle_6);
//            
//            HAL_Delay(20);
//            now_angle_1 = pwm_angle_1;
////            printf("%2f %2f %2f %2f %2f %2f\n", now_angle_1,now_angle_2,now_angle_3,now_angle_4,now_angle_5,now_angle_6);
//        }
//        now_angle_1 = temp_target_angle_1;
//         pwm_out(temp_target_angle_1,now_angle_2,now_angle_3,now_angle_4,now_angle_5,now_angle_6);
//    }

//抓取部分舵机1不适用，因过多段抓取会导致机械臂无力

	if(temp_target_angle_2 != NULL)
	{
		for(;abs_angle_error_2 >= 3;abs_angle_error_2 --)
        {

            double pwm_angle_2 = (temp_target_angle_2 > now_angle_2 ? temp_target_angle_2 - abs_angle_error_2 : temp_target_angle_2 + abs_angle_error_2);

            pwm_out(temp_target_angle_1,pwm_angle_2,now_angle_3,now_angle_4,now_angle_5,now_angle_6);
            HAL_Delay(20);
            now_angle_2 = pwm_angle_2;
        }
        now_angle_2 = temp_target_angle_2;
        pwm_out(temp_target_angle_1,temp_target_angle_2,now_angle_3,now_angle_4,now_angle_5,now_angle_6);
	}
    
	if(temp_target_angle_3 != NULL)
	{
		for(;abs_angle_error_3 >= 3;abs_angle_error_3 --)
        {

            double pwm_angle_3 = (temp_target_angle_3 > now_angle_3 ? temp_target_angle_3 - abs_angle_error_3 : temp_target_angle_3 + abs_angle_error_3);
            pwm_out(temp_target_angle_1,temp_target_angle_2,pwm_angle_3,now_angle_4,now_angle_5,now_angle_6);
            HAL_Delay(20);
            now_angle_3 = pwm_angle_3;
        }
        now_angle_3 = temp_target_angle_3;
        pwm_out(temp_target_angle_1,temp_target_angle_2,temp_target_angle_3,now_angle_4,now_angle_5,now_angle_6);
	}
    
	if(temp_target_angle_4 != NULL)
	{
		for(;abs_angle_error_4 >= 3;abs_angle_error_4 --)
        {

            double pwm_angle_4 = (temp_target_angle_4 > now_angle_4 ? temp_target_angle_4 - abs_angle_error_4 : temp_target_angle_4 + abs_angle_error_4);
            pwm_out(temp_target_angle_1,temp_target_angle_2,temp_target_angle_3,pwm_angle_4,now_angle_5,now_angle_6);
            HAL_Delay(20);
            now_angle_4 = pwm_angle_4;
        }
        now_angle_4 = temp_target_angle_4;
        pwm_out(temp_target_angle_1,temp_target_angle_2,temp_target_angle_3,temp_target_angle_4,now_angle_5,now_angle_6);
	}
    
    if(temp_target_angle_6 != NULL)
	{
		for(;abs_angle_error_6 >= 3;abs_angle_error_6 --)
        {

            double pwm_angle_6 = (temp_target_angle_6 > now_angle_6 ? temp_target_angle_6 - abs_angle_error_6 : temp_target_angle_6 + abs_angle_error_6);
            pwm_out(temp_target_angle_1,temp_target_angle_2,temp_target_angle_3,temp_target_angle_4,now_angle_5,pwm_angle_6);
            HAL_Delay(20);
            now_angle_6 = pwm_angle_6;
        }
        now_angle_6 = temp_target_angle_6;
        pwm_out(temp_target_angle_1,temp_target_angle_2,temp_target_angle_3,temp_target_angle_4,now_angle_5,temp_target_angle_6);
	}    
   
	if(temp_target_angle_5 != NULL)
	{
		for(;abs_angle_error_5 >= 3;abs_angle_error_5 --)
        {

            double pwm_angle_5 = (temp_target_angle_5 > now_angle_5 ? temp_target_angle_5 - abs_angle_error_5 : temp_target_angle_5 + abs_angle_error_5);
            pwm_out(temp_target_angle_1,temp_target_angle_2,temp_target_angle_3,temp_target_angle_4,pwm_angle_5,temp_target_angle_6);
            HAL_Delay(20);
            now_angle_5 = pwm_angle_5;
        }
        now_angle_5 = temp_target_angle_5;
        pwm_out(temp_target_angle_1,temp_target_angle_2,temp_target_angle_3,temp_target_angle_4,temp_target_angle_5,temp_target_angle_6);
	}
    
    HAL_Delay(1000);    

    abs_angle_error_1  = a1 ;
    abs_angle_error_2  = a2 ;
    abs_angle_error_3  = a3 ;
    abs_angle_error_4  = a4 ;
    abs_angle_error_5  = a5 ;
    abs_angle_error_6  = a6 ;
   
}


void servo_reset_begin(void)
{
    pwm_out(0,0,0,0,0,90);
    HAL_Delay(1000);
    now_angle_1 = 0;
    now_angle_2 = 0;
    now_angle_3 = 0;
    now_angle_4 = 0;
    now_angle_5 = 0;
    now_angle_6 = 90;
    HAL_Delay(1000);
}

void servo_reset(void)//偷懒版
{
    double i1 = 0,i2 = 0,i3 = 0;
    now_angle_6 = 90;
    double b5 = now_angle_5;
    double b4 = now_angle_4;
    //double b3 = now_angle_3;
    double b2 = now_angle_2;
    for(i1 = 10 * b5;i1 >= 0;i1 -= b5)
    {
        pwm_out(0,now_angle_2,0,now_angle_4,0.1 * i1,90);
        HAL_Delay(30);
        now_angle_5 = 0.1 *i1;

    }
    pwm_out(0,now_angle_2,0,now_angle_4,0,90);

    for(i2 = 10 * b4;i2 >= 0;i2 -= b4)
    {
        pwm_out(0,now_angle_2,0,0.1 * i2,0,90);
        HAL_Delay(30);
        now_angle_4 = 0.1 * i2;

    }
    pwm_out(0,now_angle_2,0,0,0,90);

    for(i3 = 10 * b2;i3 >= 0;i3 -= b2)
    {
        pwm_out(0,0.1 * i3,0,0,0,90);
        HAL_Delay(30);
        now_angle_2 = 0.1 * i3;
    }    
    pwm_out(0,0,0,0,0,90);
    
    HAL_Delay(1000);
    now_angle_1 = 0;   
}

void servo_catch(double servo_target_angle_1,double servo_target_angle_2,double servo_target_angle_3,double servo_target_angle_4)
{
	servo_control(0,servo_target_angle_4,0,servo_target_angle_3,servo_target_angle_2,servo_target_angle_1);
    HAL_Delay(500);
    
    servo_control(-60,servo_target_angle_4,0,servo_target_angle_3,servo_target_angle_2,servo_target_angle_1);
    pwm_out(-65,target_angle_4,0,target_angle_3,target_angle_2,target_angle_1);
    HAL_Delay(500);
    pwm_out(-65,target_angle_4,0,target_angle_3,target_angle_2,target_angle_1);
    HAL_Delay(50);
    pwm_out(-65,target_angle_4,0,target_angle_3,target_angle_2,target_angle_1);
    HAL_Delay(50);
    pwm_out(-65,target_angle_4,0,target_angle_3,target_angle_2,target_angle_1);
    HAL_Delay(50);
    pwm_out(-65,target_angle_4,0,target_angle_3,target_angle_2,target_angle_1);
    HAL_Delay(50);
    pwm_out(-65,target_angle_4,0,target_angle_3,target_angle_2,target_angle_1);
    HAL_Delay(50);
    pwm_out(-65,target_angle_4,0,target_angle_3,target_angle_2,target_angle_1);
    HAL_Delay(50);
    
    now_angle_1 = -50;
}

void servo_lift(void)
{
    servo_control(-65,30,0,30,45,90);
    HAL_Delay(500);
}

void servo_lift_return(void)
{
    servo_control(-15,30,0,30,45,90);
    HAL_Delay(1000);
}

void servo_transfer_blue(void)
{
    servo_control(-65,30,0,30,45,160);
    HAL_Delay(1000);
}

void servo_transfer_yellow(void)
{
    servo_control(-65,30,0,30,45,20);
    HAL_Delay(1000);
}

/**************************************************************************
函数功能：控制机械爪释放目标
入口参数：无
返回  值：无
备注    ：无`
**************************************************************************/
void servo_release_blue(void)
{
    pwm_out(-15,30,0,30,45,160);
    HAL_Delay(300);
    pwm_out(-15,30,0,30,45,160);
    HAL_Delay(30);
    pwm_out(-15,30,0,30,45,160);
    HAL_Delay(30);
    pwm_out(-15,30,0,30,45,160);
    HAL_Delay(30);
    pwm_out(-15,30,0,30,45,160);
    HAL_Delay(30);
    pwm_out(-15,30,0,30,45,160);
    HAL_Delay(500);
    
    now_angle_1 = -15;
}

void servo_release_yellow(void)
{
    pwm_out(-15,30,0,30,45,20);
    HAL_Delay(300);
    pwm_out(-15,30,0,30,45,20);
    HAL_Delay(30);
    pwm_out(-15,30,0,30,45,20);
    HAL_Delay(30);
    pwm_out(-15,30,0,30,45,20);
    HAL_Delay(30);
    pwm_out(-15,30,0,30,45,20);
    HAL_Delay(30);
    pwm_out(-15,30,0,30,45,20);
    HAL_Delay(500);
    
    now_angle_1 = -15;
}

void servo_test(void)
{
	servo_control(0, 90, 0, 0, 0, 0);
	HAL_Delay(1000);
	servo_reset();
	HAL_Delay(1000);
	servo_control(0, -90, 0, 0, 0, 0);
	HAL_Delay(1000);
	servo_reset();
	HAL_Delay(2000);

	servo_control(0, 0, 0, 90, 0, 0);
	HAL_Delay(1000);
	servo_reset();
	HAL_Delay(1000);
	servo_control(0, 0, 0, -90, 0, 0);
	HAL_Delay(1000);
	servo_reset();
	HAL_Delay(2000);

	servo_control(0, 0, 0, 0, 90, 0);
	HAL_Delay(1000);
	servo_reset();
	HAL_Delay(1000);
	servo_control(0, 0, 0, 0, -90, 0);
	HAL_Delay(1000);
	servo_reset();
	HAL_Delay(2000);

	servo_control(0, 0, 0, 0, 0, 180);
	HAL_Delay(2000);
	servo_control(0, 0, 0, 0, 0, 90);
	HAL_Delay(1000);
	servo_reset();
	HAL_Delay(2000);


}

void blue_task(void)
{
    servo_angle_calculate(center_x_cm,center_y_cm + 1,2);  
    servo_catch(target_angle_1,target_angle_2,target_angle_3,target_angle_4);
    servo_lift();
    servo_transfer_blue();
    servo_release_blue();
    servo_lift_return();
    servo_reset();
    printf("颜色种类：%c   ", color_type);
    printf("中心x坐标：%d   ",   center_x);
    printf("中心y坐标：%d\n   ", center_y);   
}

void yellow_task(void)
{
    servo_angle_calculate(center_x_cm,center_y_cm + 1,2);  
    servo_catch(target_angle_1,target_angle_2,target_angle_3,target_angle_4);
    servo_lift();
    servo_transfer_yellow();
    servo_release_yellow();
    servo_lift_return();
    servo_reset();
    printf("颜色种类：%c   ", color_type);
    printf("中心x坐标：%d   ",   center_x);
    printf("中心y坐标：%d\n   ", center_y);     
}
