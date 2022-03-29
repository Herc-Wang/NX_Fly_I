/****************************************************************************
 * apps/examples/mpu60x0_i2c/mpu60x0_i2c_main.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 * 
 * 该mpu60x0与匿名科创地面站v4.34进行通信
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <inttypes.h>

#include <nuttx/sensors/mpu60x0.h>
#include "imu_types.h"
#include "imu.h"
#include "imu_filter.h"
#include "ANO_DT_V4_34.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define M_PI        3.14159265358979323846 

#define MPU_DEV_PATH "/dev/imu0"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

INT16_XYZ	 GYRO_OFFSET_RAW={0,0,0},ACC_OFFSET_RAW={0,0,0};	  //零漂数据
uint8_t    SENSER_OFFSET_FLAG;                       //传感器校准标志位
struct Internal_PID Inter_PID;                          //串口传入pid值，仅用于调试
/****************************************************************************
 * Private Data
 ****************************************************************************/

FLOAT_ANGLE Att_Angle_Data;                   //飞机姿态数据
FLOAT_XYZ 	Gyr_rad,Gyr_radold;	              //把陀螺仪的各通道读出的数据，转换成弧度制
FLOAT_XYZ 	MPU_ACC_Filt, MPU_GRY_Filt, Acc_filtold, Gyr_filtold;	  //滤波后的各通道数据

static uint8_t MPU_buff[14];                  //加速度、温度、陀螺仪原始数据
uint8_t     IMU_Cal_EN = 0;

INT16_XYZ	 MPU_ACC_RAW,MPU_GYRO_RAW;	      //读取值原始数据   

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: main
 ****************************************************************************/

int main(int argc, char *argv[])
{
    FILE *filp_sensor;

    int ret; 
    static int IIR_mode = 1;
    
    Inter_PID.HALFT = atof(argv[1]);
    Inter_PID.KP = atof(argv[2]);
    Inter_PID.KI = atof(argv[3]);
    Inter_PID.KD = atof(argv[4]);
    printf("Inter_PID halft = %f, KP = %f, KI = %f, KD = %f \r\n", Inter_PID.HALFT, Inter_PID.KP, Inter_PID.KI, Inter_PID.KD);
  

    filp_sensor = open(MPU_DEV_PATH, O_RDWR);
    if(filp_sensor == NULL)
    {
        printf("Unable to create file\n");
        return -ENODEV;
    }

    SENSER_FLAG_SET(GYRO_OFFSET);//校准陀螺仪 加速度计
    //SENSER_FLAG_SET(ACC_OFFSET);//校准陀螺仪 加速度计

    for(; ; )
    {

        ret = read(filp_sensor, MPU_buff, sizeof(MPU_buff));
        if(ret < 0) 
            printf("READ failed %d \n",ret);

        MPU_RAWDataProcess(&MPU_ACC_RAW, &MPU_GYRO_RAW, MPU_buff);      //对MPU6050进行处理，减去零偏。如果没有计算零偏就计算零偏

        if(IMU_Cal_EN == 1)
        {
            if(0 == SortAver_FilterXYZ(&MPU_ACC_RAW, &MPU_ACC_Filt, Filter_Arr_N))        //对加速度原始数据进行去极值滑动窗口滤波
            {
                //加速度AD值 转换成 米/平方秒
                MPU_ACC_Filt.X = (float)MPU_ACC_Filt.X * ACC_GAIN * G;
                MPU_ACC_Filt.Y = (float)MPU_ACC_Filt.Y * ACC_GAIN * G;
                MPU_ACC_Filt.Z = (float)MPU_ACC_Filt.Z * ACC_GAIN * G;

                //陀螺仪AD值 转换成 弧度/秒
                MPU_GRY_Filt.X = (float)MPU_GYRO_RAW.X * GYRO_GR;
                MPU_GRY_Filt.Y = (float)MPU_GYRO_RAW.Y * GYRO_GR;
                MPU_GRY_Filt.Z = (float)MPU_GYRO_RAW.Z * GYRO_GR;

                if(IIR_mode)
                {
                    MPU_ACC_Filt.X = MPU_ACC_Filt.X * Kp_New + Acc_filtold.X * Kp_Old;
                    MPU_ACC_Filt.Y = MPU_ACC_Filt.Y * Kp_New + Acc_filtold.Y * Kp_Old;
                    MPU_ACC_Filt.Z = MPU_ACC_Filt.Z * Kp_New + Acc_filtold.Z * Kp_Old;

                    Acc_filtold.X = MPU_ACC_Filt.X;
                    Acc_filtold.Y = MPU_ACC_Filt.Y;
                    Acc_filtold.Z = MPU_ACC_Filt.Z;

                    MPU_GRY_Filt.X = MPU_GRY_Filt.X * Kp_New + Gyr_filtold.X * Kp_Old;
                    MPU_GRY_Filt.Y = MPU_GRY_Filt.Y * Kp_New + Gyr_filtold.Y * Kp_Old;
                    MPU_GRY_Filt.Z = MPU_GRY_Filt.Z * Kp_New + Gyr_filtold.Z * Kp_Old;

                    Gyr_filtold.X = MPU_GRY_Filt.X;
                    Gyr_filtold.Y = MPU_GRY_Filt.Y;
                    Gyr_filtold.Z = MPU_GRY_Filt.Z;
                }

                IMU_update(&MPU_GRY_Filt, &MPU_ACC_Filt, &Att_Angle_Data); //四元数姿态解算
                //上传加速度、陀螺仪数据到匿名上位机V4.34
                ANO_DT_Send_Senser((s16)(MPU_ACC_Filt.X*10),  (s16)(MPU_ACC_Filt.Y*10),  (s16)(MPU_ACC_Filt.Z*10),  \        
                                   MPU_GRY_Filt.X,  MPU_GRY_Filt.Y, MPU_GRY_Filt.Z, \
                                   0, 0 , 0, 0);
                //上传欧拉角数据到匿名上位机V4.34
                ANO_DT_Send_Status(Att_Angle_Data.rol, -Att_Angle_Data.pit, -Att_Angle_Data.yaw/3, 0x1234, 1, 1);   
            }
        }
    }

    fclose(filp_sensor);
    return EXIT_SUCCESS;
}
