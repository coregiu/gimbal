
#include <mpu6050.h>
#include "uart_log.h"

#define RAD_TO_DEG 57.295779513082320876798154814105

const double Accel_Z_corrector = 14418.0;

double  Kp = 100.0;
double  Ki = 0.003f;
double  halfT = 0.003f;

double q0 = 1, q1 = 0, q2 = 0, q3 = 0;
double exInt = 0, eyInt = 0, ezInt = 0;

enum ACCE_RANGE AccR = ACC_2G;
enum GYRO_RANGE GyrR = BPS_2000;

Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

Kalman_t KalmanZ = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

// 初始化MPU6050
// 返回值:0,成功
//     其他,错误代码
uint8_t MPU_Init(void)
{
    uint8_t res;

    // TODO 初始化IIC总线
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X80); // 复位MPU6050
    delay_ms(100);
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X00); // 唤醒MPU6050
    MPU_Set_Gyro_Fsr(3);                     // 陀螺仪传感器,±2000dps
    MPU_Set_Accel_Fsr(0);                    // 加速度传感器,±2g
    MPU_Set_Rate(50);                        // 设置采样率50Hz
    MPU_Write_Byte(MPU_INT_EN_REG, 0X00);    // 关闭所有中断
    MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00); // I2C主模式关闭
    MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00);   // 关闭FIFO
    MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X80); // INT引脚低电平有效
    res = MPU_Read_Byte(MPU_DEVICE_ID_REG);
    if (res == MPU_ADDR || res == MPU_6500_WHO_AMI_I) // 器件ID正确
    {
        MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X01); // 设置CLKSEL,PLL X轴为参考
        MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0X00); // 加速度与陀螺仪都工作
        MPU_Set_Rate(50);                        // 设置采样率为50Hz
        return 0;
    }
    else
    {
        uart_log_string_no_enter("failed to init mpu res: ");
        uart_log_number(res);
        uart_log_enter_char();
        return 1;
    }
}
// 设置MPU6050陀螺仪传感器满量程范围
// fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
// 返回值:0,设置成功
//     其他,设置失败
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3); // 设置陀螺仪满量程范围
}
// 设置MPU6050加速度传感器满量程范围
// fsr:0,±2g;1,±4g;2,±8g;3,±16g
// 返回值:0,设置成功
//     其他,设置失败
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3); // 设置加速度传感器满量程范围
}
// 设置MPU6050的数字低通滤波器
// lpf:数字低通滤波频率(Hz)
// 返回值:0,设置成功
//     其他,设置失败
uint8_t MPU_Set_LPF(uint16_t lpf)
{
    uint8_t data = 0;
    if (lpf >= 188)
        data = 1;
    else if (lpf >= 98)
        data = 2;
    else if (lpf >= 42)
        data = 3;
    else if (lpf >= 20)
        data = 4;
    else if (lpf >= 10)
        data = 5;
    else
        data = 6;
    return MPU_Write_Byte(MPU_CFG_REG, data); // 设置数字低通滤波器
}
// 设置MPU6050的采样率(假定Fs=1KHz)
// rate:4~1000(Hz)
// 返回值:0,设置成功
//     其他,设置失败
uint8_t MPU_Set_Rate(uint16_t rate)
{
    uint8_t data;
    if (rate > 1000)
        rate = 1000;
    if (rate < 4)
        rate = 4;
    data = 1000 / rate - 1;
    data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data); // 设置数字低通滤波器
    return MPU_Set_LPF(rate / 2);                     // 自动设置LPF为采样率的一半
}

// 得到温度值
// 返回值:温度值(扩大了100倍)
float MPU_Get_Temperature(void)
{
    uint8_t t_h, t_l;
    short raw;
    float temp;
    MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, &t_h);
    MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTL_REG, 2, &t_l);
    raw = ((uint16_t)t_h << 8) | t_l;
    temp = (float)((int16_t)raw / (float)340.0 + (float)36.53);
    ;
    return temp;
}
// 得到陀螺仪值(原始值)
// gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
// 返回值:0,成功
//     其他,错误代码
uint8_t MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
    uint8_t x_h = 0, x_l = 0, y_h = 0, y_l = 0, z_h = 0, z_l = 0, res = 0;
    res |= MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 1, &x_h);
    res |= MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTL_REG, 1, &x_l);
    res |= MPU_Read_Len(MPU_ADDR, MPU_GYRO_YOUTH_REG, 1, &y_h);
    res |= MPU_Read_Len(MPU_ADDR, MPU_GYRO_YOUTL_REG, 1, &y_l);
    res |= MPU_Read_Len(MPU_ADDR, MPU_GYRO_ZOUTH_REG, 1, &z_h);
    res |= MPU_Read_Len(MPU_ADDR, MPU_GYRO_ZOUTL_REG, 1, &z_l);

    // uart_log_data('$');
    // uart_log_number(x_h);
    // uart_log_data('|');
    // uart_log_number(x_l);
    // uart_log_data('|');
    // uart_log_number(y_h);
    // uart_log_data('|');
    // uart_log_number(y_l);
    // uart_log_data('|');
    // uart_log_number(z_h);
    // uart_log_data('|');
    // uart_log_number(z_l);
    // uart_log_data('#');
    // uart_log_enter_char();

    if (res == 0)
    {
        *gx = ((uint16_t)x_h << 8) | x_l;
        *gy = ((uint16_t)y_h << 8) | y_l;
        *gz = ((uint16_t)z_h << 8) | z_l;
    }
    return res;
}

// 得到加速度值(原始值)
// gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
// 返回值:0,成功
//     其他,错误代码
uint8_t MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
    uint8_t x_h = 0, x_l = 0, y_h = 0, y_l = 0, z_h = 0, z_l = 0, res = 0;
    res |= MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 1, &x_h);
    res |= MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTL_REG, 1, &x_l);
    res |= MPU_Read_Len(MPU_ADDR, MPU_ACCEL_YOUTH_REG, 1, &y_h);
    res |= MPU_Read_Len(MPU_ADDR, MPU_ACCEL_YOUTL_REG, 1, &y_l);
    res |= MPU_Read_Len(MPU_ADDR, MPU_ACCEL_ZOUTH_REG, 1, &z_h);
    res |= MPU_Read_Len(MPU_ADDR, MPU_ACCEL_ZOUTL_REG, 1, &z_l);

    // uart_log_data('@');
    // uart_log_number(x_h);
    // uart_log_data('|');
    // uart_log_number(x_l);
    // uart_log_data('|');
    // uart_log_number(y_h);
    // uart_log_data('|');
    // uart_log_number(y_l);
    // uart_log_data('|');
    // uart_log_number(z_h);
    // uart_log_data('|');
    // uart_log_number(z_l);
    // uart_log_data('#');
    // uart_log_enter_char();

    if (res == 0)
    {
        *ax = ((uint16_t)x_h << 8) | x_l;
        *ay = ((uint16_t)y_h << 8) | y_l;
        *az = ((uint16_t)z_h << 8) | z_l;
    }
    return res;
}
// IIC连续写
// addr:器件地址
// reg:寄存器地址
// len:写入长度
// buf:数据区
// 返回值:0,正常
//     其他,错误代码
uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    uint8_t i;
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr << 1) | 0); // 发送器件地址+写命令
    if (MPU_IIC_Wait_Ack())             // 等待应答
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg); // 写寄存器地址
    MPU_IIC_Wait_Ack();     // 等待应答
    for (i = 0; i < len; i++)
    {
        MPU_IIC_Send_Byte(buf[i]); // 发送数据
        if (MPU_IIC_Wait_Ack())    // 等待ACK
        {
            MPU_IIC_Stop();
            return 1;
        }
    }
    MPU_IIC_Stop();
    return 0;
}
// IIC连续读
// addr:器件地址
// reg:要读取的寄存器地址
// len:要读取的长度
// buf:读取到的数据存储区
// 返回值:0,正常
//     其他,错误代码
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr << 1) | 0); // 发送器件地址+写命令
    if (MPU_IIC_Wait_Ack())             // 等待应答
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg); // 写寄存器地址
    MPU_IIC_Wait_Ack();     // 等待应答
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr << 1) | 1); // 发送器件地址+读命令
    MPU_IIC_Wait_Ack();                 // 等待应答
    while (len)
    {
        if (len == 1)
            *buf = MPU_IIC_Read_Byte(0); // 读数据,发送nACK
        else
            *buf = MPU_IIC_Read_Byte(1); // 读数据,发送ACK
        len--;
        buf++;
    }
    MPU_IIC_Stop(); // 产生一个停止条件
    return 0;
}
// IIC写一个字节
// reg:寄存器地址
// data:数据
// 返回值:0,正常
//     其他,错误代码
uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data)
{
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU_ADDR << 1) | 0); // 发送器件地址+写命令
    if (MPU_IIC_Wait_Ack())                 // 等待应答
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg);  // 写寄存器地址
    MPU_IIC_Wait_Ack();      // 等待应答
    MPU_IIC_Send_Byte(data); // 发送数据
    if (MPU_IIC_Wait_Ack())  // 等待ACK
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Stop();
    return 0;
}
// IIC读一个字节
// reg:寄存器地址
// 返回值:读到的数据
uint8_t MPU_Read_Byte(uint8_t reg)
{
    uint8_t res;
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU_ADDR << 1) | 0); // 发送器件地址+写命令
    MPU_IIC_Wait_Ack();                     // 等待应答
    MPU_IIC_Send_Byte(reg);                 // 写寄存器地址
    MPU_IIC_Wait_Ack();                     // 等待应答
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU_ADDR << 1) | 1); // 发送器件地址+读命令
    MPU_IIC_Wait_Ack();                     // 等待应答
    res = MPU_IIC_Read_Byte(0);             // 读取数据,发送nACK
    MPU_IIC_Stop();                         // 产生一个停止条件
    return res;
}

// void Compute_Angle(struct gimbal_info *gimbal)
// {
//     gimbal->accl_x = gimbal->accl_x_raw / 16384.0;
//     gimbal->accl_y = gimbal->accl_y_raw / 16384.0;
//     gimbal->accl_z = gimbal->accl_z_raw / Accel_Z_corrector;

//     gimbal->gyro_x = gimbal->gyro_x_raw / 131.0;
//     gimbal->gyro_y = gimbal->gyro_y_raw / 131.0;
//     gimbal->gyro_z = gimbal->gyro_z_raw / 131.0;

//     // Kalman angle solve
//     double dt = 1;
//     double roll;
//     double roll_sqrt = sqrt(gimbal->accl_x_raw * gimbal->accl_x_raw + gimbal->accl_z_raw * gimbal->accl_z_raw);
//     if (roll_sqrt != 0.0)
//     {
//         roll = atan(gimbal->accl_y_raw / roll_sqrt) * RAD_TO_DEG;
//     }
//     else
//     {
//         roll = 0.0;
//     }

//     if (fabs(gimbal->pitch) > 90)
//         gimbal->gyro_x = -gimbal->gyro_x;
//     gimbal->roll = Kalman_getAngle(&KalmanX, roll, gimbal->gyro_x, dt);

//     double pitch = atan2(-gimbal->accl_x_raw, gimbal->accl_z_raw) * RAD_TO_DEG;
//     if ((pitch < -90 && gimbal->pitch > 90) || (pitch > 90 && gimbal->pitch < -90))
//     {
//         KalmanY.angle = pitch;
//         gimbal->pitch = pitch;
//     }
//     else
//     {
//         gimbal->pitch = Kalman_getAngle(&KalmanY, pitch, gimbal->gyro_y, dt);
//     }

//     double yaw_estimate = atan2(gimbal->accl_y, gimbal->accl_x);
//     if ((yaw_estimate < -90 && gimbal->yaw > 90) || (yaw_estimate > 90 && gimbal->yaw < -90))
//     {
//         KalmanZ.angle = yaw_estimate;
//         gimbal->yaw = yaw_estimate;
//     }
//     else
//     {
//         gimbal->yaw = Kalman_getAngle(&KalmanZ, yaw_estimate, gimbal->gyro_z, dt);
//     }
// }

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
}

double getAcceXdata(short raw_data)
{
    double temp;
    switch (AccR)
    {
        case ACC_2G:
            temp = (double)raw_data / 32767 * 9.8;
            return temp;

        case ACC_4G:
            temp = (double)raw_data / 16384 * 9.8;
            return temp;

        case ACC_8G:
            temp = (double)raw_data / 8192 * 9.8;
            return temp;

        case ACC_16G:
            temp = (double)raw_data / 4096 * 9.8;
            return temp;
    }

    return 0;
}

double getAcceYdata(short raw_data)
{
    double temp;
    switch (AccR)
    {
        case ACC_2G:
            temp = (double)raw_data / 32767 * 9.8;
            return temp;

        case ACC_4G:
            temp = (double)raw_data / 16384 * 9.8;
            return temp;

        case ACC_8G:
            temp = (double)raw_data / 8192 * 9.8;
            return temp;

        case ACC_16G:
            temp = (double)raw_data / 4096 * 9.8;
            return temp;
    }

    return 0;
}

double getAcceZdata(short raw_data)
{
    double temp;
    switch (AccR)
    {
        case ACC_2G:
            temp = (double)raw_data / 32767 * 9.8;
            return temp;

        case ACC_4G:
            temp = (double)raw_data / 16384 * 9.8;
            return temp;

        case ACC_8G:
            temp = (double)raw_data / 8192 * 9.8;
            return temp;

        case ACC_16G:
            temp = (double)raw_data / 4096 * 9.8;
            return temp;
    }

    return 0;
}

double getGyroXdata(short raw_data)
{
    double temp;
    switch (GyrR)
    {
        case BPS_250:
            temp =  (double)raw_data / 262;
            return temp;

        case BPS_500:
            temp =  (double)raw_data / 161;
            return temp;

        case BPS_1000:
            temp =  (double)raw_data / 65.5;
            return temp;

        case BPS_2000:
            temp =  (double)raw_data / 32.75;
            return temp;
    }

    return 0;
}

double getGyroYdata(short raw_data)
{
    double temp;
    switch (GyrR)
    {
        case BPS_250:
            temp =  (double)raw_data / 262;
            return temp;

        case BPS_500:
            temp =  (double)raw_data / 161;
            return temp;

        case BPS_1000:
            temp =  (double)raw_data / 65.5;
            return temp;

        case BPS_2000:
            temp =  (double)raw_data / 32.75;
            return temp;
    }

    return 0;
}

double getGyroZdata(short raw_data)
{
    double temp;
    switch (GyrR)
    {
        case BPS_250:
            temp =  (double)raw_data / 262;
            return temp;

        case BPS_500:
            temp =  (double)raw_data / 161;
            return temp;

        case BPS_1000:
            temp =  (double)raw_data / 65.5;
            return temp;

        case BPS_2000:
            temp =  (double)raw_data / 32.75;
            return temp;
    }

    return 0;
}

void Compute_Angle(struct gimbal_info *gimbal)
{
    double gx = getGyroXdata(gimbal->gyro_x_raw);
    double gy = getGyroYdata(gimbal->gyro_y_raw);
    double gz = getGyroZdata(gimbal->gyro_z_raw);
    double ax = getAcceXdata(gimbal->accl_x_raw);
    double ay = getAcceYdata(gimbal->accl_y_raw);
    double az = getAcceZdata(gimbal->accl_z_raw);

    double  norm;
    double  vx, vy, vz;
    double  ex, ey, ez;

    norm = sqrt(ax*ax + ay*ay + az*az);
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;

    vx = 2*(q1*q3 - q0*q2);
    vy = 2*(q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);

    exInt = exInt + ex*Ki;
    eyInt = eyInt + ey*Ki;
    ezInt = ezInt + ez*Ki;

    gx = gx + Kp*ex + exInt;
    gy = gy + Kp*ey + eyInt;
    gz = gz + Kp*ez + ezInt;

    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;

    gimbal->accl_x = ax;
    gimbal->accl_y = ay;
    gimbal->accl_z = az;
    gimbal->gyro_x = gx;
    gimbal->gyro_y = gy;
    gimbal->gyro_z = gz;
    gimbal->pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;
    gimbal->roll   = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;
    gimbal->yaw    = atan2(2*(q1*q2 + q0*q3), q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;
    // add by coregiu adapte angle.
    gimbal->yaw    *= 3;
}

