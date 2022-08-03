/************************************************************************************************
* 程序版本：V1.0
* 程序日期：2022-7-2
* 程序作者：719飞行器实验室： 张天鹏、刘胜昔、赵艺超						      
************************************************************************************************/
#ifndef   _SERVO_H_
#define   _SERVO_H_

#include "tim.h"

void pwm_start(void);
//开启PWM输出
void translate_angle_to_pulse(double angle_1,double angle_2,double angle_3,double angle_4,double angle_5,double angle_6);
//将角度转换为脉宽，进而改变PWM输出占空比
void pwm_out(double angle_1, double angle_2, double angle_3, double angle_4, double angle_5,  double angle_6);
//底层PWM输出
void servo_catch(double servo_target_angle_1,double servo_target_angle_2,double servo_target_angle_3,double servo_target_angle_4);
//机械爪收紧
void servo_test(void);
//对机械爪进行测试
void servo_reset_begin(void);
//程序开始时的机械爪初始化
void servo_reset(void);
//平缓地将机械爪各角度归零
void servo_release(void);
//机械爪松开
void servo_control(double target_angle_1, double target_angle_2, double target_angle_3, double target_angle_4, double target_angle_5,  double target_angle_6);
//控制机械臂平缓运动
void servo_angle_calculate(float target_x, float target_y, float target_z);
//解算舵机角度

void servo_lift(void);
//抓取动作结束后机械臂抬起
void servo_lift_return(void);
//释放物品后返回的中间阶段
void servo_transfer(void);
//机械臂左旋至固定角度

void blue_task(void);
//识别到蓝色物品后执行分拣任务
void yellow_task(void);
//识别到黄色物品后执行分拣任务
#endif
