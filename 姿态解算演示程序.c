#include "math.h"
#include <stdio.h>
int main()
{
	
	float a1,a2,a3,a4;                  //a1为底部圆台高度 剩下三个为三个机械臂长度 
	float j1,j4,j2,j3;                      //四个姿态角
	float L,H,P;	             //L =a2*sin(j2) + a3*sin(j2 + j3);H = a2*cos(j2) + a3*cos(j2 + j3); P为底部圆盘半径R
	float j_all;	                             //j2,j3,j4之和
	float len,high;                       //总长度,总高度
	float Cosj3,Sinj3;                   //用来存储cosj3,sinj3数值
	float Cosj2,Sinj2;
	float K1,K2;
	float X,Y,Z;  	             //输入 （X,Y,Z）坐标
	int i;
	float n,m,q;
	n = 0;
	m = 0;
	q = 0;
	//目标点坐标（X，Y，Z）
	X = 0;
	Y = 18;
	Z = 0;

	P = 14;     //底部圆盘半径
	a1 = 12; 	//底部圆盘高度	            
	a2 = 12;    //机械臂长度
	a3 = 12;
	a4 = 12;
	
	if (X == 0) 
	    j1=90;
	else 
	    j1 = atan((Y+P)/X)*(57.3);

	for(i=0;i<=180;i++)
	{	
		j_all = 3.1415927*i/180;

		len = sqrt((Y+P)*(Y+P)+X*X);
		high = Z;
			
		L = len	- a4*sin(j_all);
		H = high - a4*cos(j_all) - a1;
		
		Cosj3 = ((L*L)+(H*H)-((a2)*(a2))-((a3)*(a3)))/(2*(a2)*(a3));
		Sinj3 = (sqrt(1-(Cosj3)*(Cosj3)));
		
		j3 = atan((Sinj3)/(Cosj3))*(57.3);
		
		K2 = a3*sin(j3/57.3);
		K1 = a2+a3*cos(j3/57.3);
		
		Cosj2 = (K2*L+K1*H)/(K1*K1+K2*K2);
		Sinj2 = (sqrt(1-(Cosj2)*(Cosj2)));
		
		j2 = atan((Sinj2)/(Cosj2))*57.3;
		j4 = j_all*57.3- j2 - j3;
		
		if(j2>=0&&j3>=0&&j4>=-90&&j2<=180&&j3<=180&&j4<=90)
		{
			n=n+1;
		}
    } 
   
   
   	for(i=0;i<=180;i++)
	{
		j_all = 3.1415927*i/180;
		
		len = sqrt((Y+P)*(Y+P)+X*X);
		high = Z;

		L = len	- a4*sin(j_all);
		H = high - a4*cos(j_all) - a1;
		
		Cosj3 = ((L*L)+(H*H)-((a2)*(a2))-((a3)*(a3)))/(2*(a2)*(a3));
		Sinj3 = (sqrt(1-(Cosj3)*(Cosj3)));
		
		j3 = atan((Sinj3)/(Cosj3))*(57.3);
		
		K2 = a3*sin(j3/57.3);
		K1 = a2+a3*cos(j3/57.3);
		
		Cosj2 = (K2*L+K1*H)/(K1*K1+K2*K2);
		Sinj2 = (sqrt(1-(Cosj2)*(Cosj2)));
		
		j2 = atan((Sinj2)/(Cosj2))*57.3;
		j4 = j_all*57.3- j2 - j3;
		
	    if(j2>=0&&j3>=0&&j4>=-90&&j2<=180&&j3<=180&&j4<=90)
		{
			m=m+1;
			if(m==n/2||m==(n+1)/2)
			{	
				break;
			}
		}
    }
   
   	printf("j1:%f,j2:%f\nj3:%f,j4:%f\n",j1,j2,j3,j4);
}
