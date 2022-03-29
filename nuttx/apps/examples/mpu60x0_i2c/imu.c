/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include "imu_types.h"
#include "imu.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/
extern uint8_t IMU_Cal_EN;
extern struct Internal_PID Inter_PID;
/****************************************************************************
 * Private Data
 ****************************************************************************/
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;     // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
float ex_pre, ey_pre, ez_pre; //上一次误差

float accb[3],DCMgb[3][3];                //方向余弦阵（将 惯性坐标系 转化为 机体坐标系）

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**************************实现函数*********************************************************************
函  数：static float invSqrt(float x) 
功　能: 快速计算 1/Sqrt(x) 	
参  数：要计算的值
返回值：结果
备  注：比普通Sqrt()函数要快四倍See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
*********************************************************************************************************/
static float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/******************************************************************************
*函  数：uint8_t MPU_ZeroBias_Calib
*功  能：MPU零偏校准 
*参  数：value： MPU原始数据
*        offset：校准后的零偏值
*        sensivity：加速度计的灵敏度
*返回值：1校准完成 0校准未完成
*备  注：无
*******************************************************************************/
uint8_t MPU_ZeroBias_Calib(INT16_XYZ *value,INT16_XYZ *offset,uint16_t sensivity)
{
	static int32_t tempgx=0,tempgy=0,tempgz=0; 
	static uint16_t cnt_a=0;//使用static修饰的局部变量，表明次变量具有静态存储周期，也就是说该函数执行完后不释放内存

    if(cnt_a==0)
    {
        value->X=0;
        value->Y=0;
        value->Z=0;
        tempgx = 0;
        tempgy = 0;
        tempgz = 0;
        cnt_a = 1;
        sensivity = 0;
        offset->X = 0;
        offset->Y = 0;
        offset->Z = 0;
    }
    tempgx+= 	value->X;
    tempgy+= 	value->Y; 
    tempgz+= 	value->Z-sensivity ;//加速度计校准 sensivity 等于 MPU9250初始化时设置的灵敏度值（8196LSB/g）;陀螺仪校准 sensivity = 0；
    if(cnt_a==200)               //100个数值求平均
    {
        offset->X=tempgx/cnt_a;
        offset->Y=tempgy/cnt_a;
        offset->Z=tempgz/cnt_a;
        tempgx=0;
        tempgy=0;
        tempgz=0;
        //printf("MPU_ZeroBias_Calib : next will return  X=%d, Y=%d, Z=%d\r\n",Calib_ZeroBias_Data->X,Calib_ZeroBias_Data->Y,Calib_ZeroBias_Data->Z);
        cnt_a = 0;
        return 1;
    }
    //printf("MPU_ZeroBias_Calib : cnt_a=%d tempgx=%ld, tempgy=%ld, tempgz=%ld\r\n", cnt_a, tempgx, tempgy,tempgz);
    cnt_a++;
	return 0;
}	

/******************************************************************************
*函  数：void MPU_RAWDataProcess
*功  能：对MPU进行去零偏处理
*参  数：MPU_ACC_RAW MPU_GYRO_RAW 加速度、陀螺仪去零偏后的值，
        MPU_buff 从mpu读回的buf
*返回值：无
*备  注：无
*******************************************************************************/
void MPU_RAWDataProcess(INT16_XYZ *MPU_ACC_RAW, INT16_XYZ *MPU_GYRO_RAW, uint8_t MPU_buff[])
{ 
	MPU_ACC_RAW->X  =((((int16_t)MPU_buff[0]) << 8) | MPU_buff[1]);
	MPU_ACC_RAW->Y  =((((int16_t)MPU_buff[2]) << 8) | MPU_buff[3]);
	MPU_ACC_RAW->Z  =((((int16_t)MPU_buff[4]) << 8) | MPU_buff[5]);
    
	MPU_GYRO_RAW->X =((((int16_t)MPU_buff[8]) << 8) | MPU_buff[9]);
	MPU_GYRO_RAW->Y =((((int16_t)MPU_buff[10]) << 8) | MPU_buff[11]);
	MPU_GYRO_RAW->Z =((((int16_t)MPU_buff[12]) << 8) | MPU_buff[13]);

  //ANO_DT_Send_Senser(MPU_ACC_RAW->X,  MPU_ACC_RAW->Y,  MPU_ACC_RAW->Z,  \
  //                     MPU_GYRO_RAW->X, MPU_GYRO_RAW->Y, MPU_GYRO_RAW->Z, \
  //                     0,0,0, 0);

    //加速度去零偏AD值
    MPU_ACC_RAW->X -= ACC_OFFSET_RAW.X;
    MPU_ACC_RAW->Y -= ACC_OFFSET_RAW.Y;
    MPU_ACC_RAW->Z -= ACC_OFFSET_RAW.Z;
    //陀螺仪去零偏AD值 
    MPU_GYRO_RAW->X -= GYRO_OFFSET_RAW.X;
    MPU_GYRO_RAW->Y -= GYRO_OFFSET_RAW.Y;
    MPU_GYRO_RAW->Z -= GYRO_OFFSET_RAW.Z;
	
	if(GET_FLAG(GYRO_OFFSET)) //陀螺仪进行零偏校准
	{
		if(MPU_ZeroBias_Calib(MPU_GYRO_RAW, &GYRO_OFFSET_RAW, 0))
		{
			 SENSER_FLAG_RESET(GYRO_OFFSET);
			 //PID_WriteFlash(); //保存陀螺仪的零偏数据
	     SENSER_FLAG_SET(ACC_OFFSET);//校准加速度
    }
	}
	if(GET_FLAG(ACC_OFFSET)) //加速度计进行零偏校准 
	{
    if(MPU_ZeroBias_Calib(MPU_ACC_RAW, &ACC_OFFSET_RAW, 8196)) 
    {
        SENSER_FLAG_RESET(ACC_OFFSET);
        IMU_Cal_EN = 1;
        //PID_WriteFlash(); //保存加速度计的零偏数据
    }
	}
}



/*********************************************************************************************************
*函  数：void IMU_update(FLOAT_XYZ *Gyr_rad,FLOAT_XYZ *Acc_filt,FLOAT_ANGLE *Att_Angle)
*功　能：获取姿态角
*参  数： Gyr_rad    指向角速度的指针（注意单位必须是弧度）
*        Acc_filt   指向加速度的指针
*        Att_Angle  指向姿态角的指针
*返回值：无
*备  注：求解四元数和欧拉角都在此函数中完成
**********************************************************************************************************/	
void IMU_update(FLOAT_XYZ *Gyr_rad, FLOAT_XYZ *Acc_filt, FLOAT_ANGLE *Att_Angle)
{  
      float matrix[9] = {1.f,  0.0f,  0.0f, 0.0f,  1.f,  0.0f, 0.0f,  0.0f,  1.f };//初始化矩阵
      float ax = Acc_filt->X,ay = Acc_filt->Y,az = Acc_filt->Z;
      float gx = Gyr_rad->X, gy = Gyr_rad->Y, gz = Gyr_rad->Z;
      float vx, vy, vz;
      float ex, ey, ez;
      float norm;

      float q0q0 = q0*q0;
      float q0q1 = q0*q1;
      float q0q2 = q0*q2;
      float q0q3 = q0*q3;
      float q1q1 = q1*q1;
      float q1q2 = q1*q2;
      float q1q3 = q1*q3;
      float q2q2 = q2*q2;
      float q2q3 = q2*q3;
      float q3q3 = q3*q3;
      
      if(ax*ay*az==0)
        return;
      
      //加速度计测量的重力向量(机体坐标系) 
      norm = invSqrt(ax*ax + ay*ay + az*az); 
      ax = ax * norm;
      ay = ay * norm;
      az = az * norm;
    //	printf("ax=%0.2f ay=%0.2f az=%0.2f\r\n",ax,ay,az);
    
      //陀螺仪积分估计重力向量(机体坐标系) 
      vx = 2*(q1q3 - q0q2);												
      vy = 2*(q0q1 + q2q3);
      vz = q0q0 - q1q1 - q2q2 + q3q3 ;
    // printf("vx=%0.2f vy=%0.2f vz=%0.2f\r\n",vx,vy,vz); 
      
      //测量的重力向量与估算的重力向量差积求出向量间的误差 
      ex = (ay*vz - az*vy); //+ (my*wz - mz*wy);                     
      ey = (az*vx - ax*vz); //+ (mz*wx - mx*wz);
      ez = (ax*vy - ay*vx); //+ (mx*wy - my*wx);

      //用上面求出误差进行积分
      exInt = exInt + ex * Ki;								 
      eyInt = eyInt + ey * Ki;
      ezInt = ezInt + ez * Ki;

      //将误差PI后补偿到陀螺仪
      gx = gx + Kp*ex + exInt + Kd*(ex - ex_pre);					   		  	
      gy = gy + Kp*ey + eyInt + Kd*(ey - ey_pre);
      gz = gz + Kp*ez + ezInt + Kd*(ez - ez_pre);//这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减

      ex_pre = ex;
      ey_pre = ey;
      ez_pre = ez;

      //四元素的微分方程
      q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
      q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
      q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
      q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;
 
      //单位化四元数 
      norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
      q0 = q0 * norm;
      q1 = q1 * norm;
      q2 = q2 * norm;  
      q3 = q3 * norm;

/***************小无人机自带的求出欧拉角的方法***********************/      
/*     
      //矩阵R 将惯性坐标系(n)转换到机体坐标系(b) 
      matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;// 11(前列后行)
      matrix[1] = 2.f * (q1q2 + q0q3);	    // 12
      matrix[2] = 2.f * (q1q3 - q0q2);	    // 13
      matrix[3] = 2.f * (q1q2 - q0q3);	    // 21
      matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;// 22
      matrix[5] = 2.f * (q2q3 + q0q1);	    // 23
      matrix[6] = 2.f * (q1q3 + q0q2);	    // 31
      matrix[7] = 2.f * (q2q3 - q0q1);	    // 32
      matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;// 33
*/      

      //四元数转换成欧拉角(Z->Y->X) 
        Att_Angle->yaw += Gyr_rad->Z *RadtoDeg*0.01f;     
        //  Att_Angle->yaw = atan2(2.f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3)* 57.3f; // yaw
        Att_Angle->pit = -asin(2.f * (q1q3 - q0q2))* 57.3f;                                 // pitch(负号要注意) 
        Att_Angle->rol = atan2(2.f * q2q3 + 2.f * q0q1, q0q0 - q1q1 - q2q2 + q3q3)* 57.3f ; // roll
  
      /*for(i=0;i<9;i++)
      {
        *(&(DCMgb[0][0])+i) = matrix[i];      //用于高度融合
      }*/

    
}