
#include <mpu6050.h>
#include "uart_log.h"

#define RAD_TO_DEG 57.295779513082320876798154814105

double q0 = 1, q1 = 0, q2 = 0, q3 = 0;
double exInt = 0, eyInt = 0, ezInt = 0;

enum MPU_TYPE mpu_type;

const short SAMPLEF_REQ = 500;
const double dt = 0.002f;
const double Kp = 100.0f;
const double Ki = 0.005f;
const double halfT = 0.001f;

short magoldx, magoldy, magoldz;
short gyro_offsetx = 0, gyro_offsety = 0, gyro_offsetz = 0;

float magoffsetx = 1.31454428611172, magoffsety = -1.21753632395713, magoffsetz = 1.6567777185719;
float B[6] = {0.980358187761106, -0.0105514731414606, 0.00754899338354401, 0.950648704823113, -0.0354995317649016, 1.07449478456729};

enum ACCE_RANGE AccR = ACC_2G;
enum GYRO_RANGE GyrR = BPS_2000;

// 初始化MPU6050
// 返回值:0,成功
//     其他,错误代码
uint8_t MPU_Init(void)
{
    uint8_t res;

    // TODO 初始化IIC总线
    MPU_Write_Byte(MPU_ADDR, MPU_PWR_MGMT1_REG, 0X80); // 复位MPU6050
    delay_ms(100);
    MPU_Write_Byte(MPU_ADDR, MPU_PWR_MGMT1_REG, 0X00); // 唤醒MPU6050
    MPU_Set_Gyro_Fsr(3);                     // 陀螺仪传感器,±2000dps
    MPU_Set_Accel_Fsr(0);                    // 加速度传感器,±2g
    MPU_Set_Rate(SAMPLEF_REQ);                        // 设置采样率50Hz
    MPU_Write_Byte(MPU_ADDR, MPU_INT_EN_REG, 0X00);    // 关闭所有中断
    MPU_Write_Byte(MPU_ADDR, MPU_USER_CTRL_REG, 0X00); // I2C主模式关闭
    MPU_Write_Byte(MPU_ADDR, MPU_FIFO_EN_REG, 0X00);   // 关闭FIFO
    MPU_Write_Byte(MPU_ADDR, MPU_INTBP_CFG_REG, 0X80); // INT引脚低电平有效
    res = MPU_Read_Byte(MPU_ADDR, MPU_DEVICE_ID_REG);
    switch (res)
    {
    case MPU_6050_WHO_AMI_I:
        mpu_type = MPU_6050;
        break;
    case MPU_6500_WHO_AMI_I:
        mpu_type = MPU_6500;
        break;
    case MPU_9250_WHO_AMI_I:
        mpu_type = MPU_9250;
        break;
    case MPU_9250_WHO_AMI_I_2:
        mpu_type = MPU_9250;
        break;

    default:
        mpu_type = ERROR_TYPE;
        break;
    }

    if (mpu_type == ERROR_TYPE) // 器件ID正确
    {
        uart_log_string_no_enter("failed to init mpu res: ");
        uart_log_number(res);
        uart_log_enter_char();
        return 1;                    // 设置采样率为50Hz
    }
    MPU_Write_Byte(MPU_ADDR, MPU_PWR_MGMT1_REG, 0X01); // 设置CLKSEL,PLL X轴为参考
    MPU_Write_Byte(MPU_ADDR, MPU_PWR_MGMT2_REG, 0X00); // 加速度与陀螺仪都工作
    MPU_Set_Rate(SAMPLEF_REQ);

    if (mpu_type == MPU_9250)
    {
        res = MPU_Read_Byte(AK8963_ADDR, MAG_WIA); // 读取AK8963 ID
        if (res == AK8963_ID)
        {
            MPU_Write_Byte(AK8963_ADDR, MAG_CNTL2, 0X01); // 复位AK8963
            delay_ms(50);
            MPU_Write_Byte(AK8963_ADDR, MAG_CNTL1, 0X11); // 设置AK8963为单次测量
        }
        else
        {
            uart_log_string_no_enter("failed to init ak8963 res: ");
            uart_log_number(res);
            uart_log_enter_char();
            return 1;
        }
    }

    calibrate();

    return 0;
}

// 设置MPU6050陀螺仪传感器满量程范围
// fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
// 返回值:0,设置成功
//     其他,设置失败
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU_ADDR, MPU_GYRO_CFG_REG, fsr << 3); // 设置陀螺仪满量程范围
}
// 设置MPU6050加速度传感器满量程范围
// fsr:0,±2g;1,±4g;2,±8g;3,±16g
// 返回值:0,设置成功
//     其他,设置失败
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU_ADDR, MPU_ACCEL_CFG_REG, fsr << 3); // 设置加速度传感器满量程范围
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
    return MPU_Write_Byte(MPU_ADDR, MPU_CFG_REG, data); // 设置数字低通滤波器
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
    data = MPU_Write_Byte(MPU_ADDR, MPU_SAMPLE_RATE_REG, data); // 设置数字低通滤波器
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
    temp = (float)((int16_t)raw / (float)340.0 + (float)33.53);

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

// 得到磁力计值(原始值)，平均滤波并调整方位
// mx,my,mz:磁力计x,y,z轴的原始读数(带符号)
// 返回值:0,成功
//     其他,错误代码
u8 MPU_Get_Magnetometer(short *mx, short *my, short *mz)
{
    u8 buf[6], res;
    res = MPU_Read_Len(AK8963_ADDR, MAG_XOUT_L, 6, buf);
    if (res == 0)
    {
        *mx = ((u16)buf[1] << 8) | buf[0];
        *my = ((u16)buf[3] << 8) | buf[2];
        *mz = ((u16)buf[5] << 8) | buf[4];
        *my = -*my;
        *mz = -*mz;
        *mx = (short)(magoldx * 0.5 + *mx * 0.5);
        *my = (short)(magoldy * 0.5 + *my * 0.5);
        *mz = (short)(magoldz * 0.5 + *mz * 0.5);
        magoldx = *mx;
        magoldy = *my;
        magoldz = *mz;
    }
    MPU_Write_Byte(AK8963_ADDR, MAG_CNTL1, 0X11); // AK8963每次读完以后都需要重新设置为单次测量模式
    return res;
}

/*//////////////////////////////////////////////////
*@功能：获得磁力计数据，单位高斯，并对磁力计进行补偿
*
*
///////////////////////////////////////////////////*/
void MPU_Get_Mag(short *imx, short *imy, short *imz, float *mx, float *my, float *mz)
{
    double tmp1 = (double)(*imx) * mag_scale - magoffsetx;
    double tmp2 = (double)(*imy) * mag_scale - magoffsety;
    double tmp3 = (double)(*imz) * mag_scale - magoffsetz;
    *mx = B[0] * tmp1 + B[1] * tmp2 + B[2] * tmp3;
    *my = B[1] * tmp1 + B[3] * tmp2 + B[4] * tmp3;
    *mz = B[2] * tmp1 + B[4] * tmp2 + B[5] * tmp3;
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
uint8_t MPU_Write_Byte(uint8_t addr, uint8_t reg, uint8_t data)
{
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr << 1) | 0); // 发送器件地址+写命令
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
uint8_t MPU_Read_Byte(uint8_t addr, uint8_t reg)
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

double getAccedata(short raw_data)
{
    double temp;
    switch (AccR)
    {
        case ACC_2G:
            temp = (double)raw_data / 16384 * 9.8;
            return temp;

        case ACC_4G:
            temp = (double)raw_data / 8192 * 9.8;
            return temp;

        case ACC_8G:
            temp = (double)raw_data / 4096 * 9.8;
            return temp;

        case ACC_16G:
            temp = (double)raw_data / 2048 * 9.8;
            return temp;
    }

    return 0;
}

double getGyrodata(short raw_data)
{
    double temp;
    switch (GyrR)
    {
        case BPS_250:
            temp =  (double)raw_data / 131.072;
            return temp;

        case BPS_500:
            temp =  (double)raw_data / 65.536;
            return temp;

        case BPS_1000:
            temp =  (double)raw_data / 32.768;
            return temp;

        case BPS_2000:
            temp =  (double)raw_data / 16.384;
            return temp;
    }

    return 0;
}

/**
 *yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
 *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2);
 *roll = atan2(2 * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
 */
void Compute_Angle(struct gimbal_info *gimbal)
{
    double gx = getGyrodata(gimbal->gyro_x_raw);
    double gy = getGyrodata(gimbal->gyro_y_raw);
    double gz = getGyrodata(gimbal->gyro_z_raw);

    double ax = getAccedata(gimbal->accl_x_raw);
    double ay = getAccedata(gimbal->accl_y_raw);
    double az = getAccedata(gimbal->accl_z_raw);

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

    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    gx = gx + Kp * ex + exInt;
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt;

    q0 = q0 + (-q1*gx - q2*gy - q3*gz) * halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)  * halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)  * halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)  * halfT;

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

    // uart_log_data('$');
    // uart_log_number(q0 * 10000);
    // uart_log_data('|');
    // uart_log_number(q1 * 10000);
    // uart_log_data('|');
    // uart_log_number(q2 * 10000);
    // uart_log_data('|');
    // uart_log_number(q3 * 10000);
    // uart_log_data('|');
    // uart_log_enter_char();

    // gimbal->pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;
    // gimbal->roll   = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;
    // gimbal->yaw    = atan2(2*(q1*q2 + q0*q3), q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;

    gimbal->pitch  = asin(2 * (q0 * q2 - q1 * q3)) * 57.3;
    gimbal->roll   = atan2(2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2)) * 57.3;
    gimbal->yaw    = atan2(2 * (q1 * q2 + q0 * q3), 1 - 2 * (q2 * q2 + q3 * q3)) * 57.3;
}

void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float *roll, float *pitch, float *yaw)
{
    double norm;               // 用于单位化
    double hx, hy, hz, bx, bz; //
    double vx, vy, vz, wx, wy, wz;
    double ex, ey, ez;

    // auxiliary variables to reduce number of repeated operations  辅助变量减少重复操作次数
    double q0q0 = q0 * q0;
    double q0q1 = q0 * q1;
    double q0q2 = q0 * q2;
    double q0q3 = q0 * q3;
    double q1q1 = q1 * q1;
    double q1q2 = q1 * q2;
    double q1q3 = q1 * q3;
    double q2q2 = q2 * q2;
    double q2q3 = q2 * q3;
    double q3q3 = q3 * q3;

    double k10, k11, k12, k13, k20, k21, k22, k23, k30, k31, k32, k33, k40, k41, k42, k43;

    // normalise the measurements  对加速度计和磁力计数据进行规范化
    norm = sqrt(ax * ax + ay * ay + az * az);
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;
    norm = sqrt(mx * mx + my * my + mz * mz);
    mx = mx / norm;
    my = my / norm;
    mz = mz / norm;

    // compute reference direction of magnetic field  计算磁场的参考方向
    // hx,hy,hz是mx,my,mz在参考坐标系的表示
    hx = 2 * mx * (0.5 - q2q2 - q3q3) + 2 * my * (q1q2 - q0q3) + 2 * mz * (q1q3 + q0q2);
    hy = 2 * mx * (q1q2 + q0q3) + 2 * my * (0.5 - q1q1 - q3q3) + 2 * mz * (q2q3 - q0q1);
    hz = 2 * mx * (q1q3 - q0q2) + 2 * my * (q2q3 + q0q1) + 2 * mz * (0.5 - q1q1 - q2q2);
    // bx,by,bz是地球磁场在参考坐标系的表示
    bx = sqrt((hx * hx) + (hy * hy));
    bz = hz;

    // estimated direction of gravity and magnetic field (v and w)  //估计重力和磁场的方向
    // vx,vy,vz是重力加速度在物体坐标系的表示
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    // wx,wy,wz是地磁场在物体坐标系的表示
    wx = 2 * bx * (0.5 - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2);
    wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3);
    wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5 - q1q1 - q2q2);

    // error is sum ofcross product between reference direction of fields and directionmeasured by sensors
    // ex,ey,ez是加速度计与磁力计测量出的方向与实际重力加速度与地磁场方向的误差，误差用叉积来表示，且加速度计与磁力计的权重是一样的
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

    // integral error scaled integral gain
    // 积分误差
    exInt = exInt + ex * Ki * (1.0f / SAMPLEF_REQ);
    eyInt = eyInt + ey * Ki * (1.0f / SAMPLEF_REQ);
    ezInt = ezInt + ez * Ki * (1.0f / SAMPLEF_REQ);
    // adjusted gyroscope measurements
    // PI调节陀螺仪数据
    gx = gx + Kp * ex + exInt;
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt;

    // integrate quaernion rate aafnd normalaizle
    // 欧拉法解微分方程
    //           q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    //           q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    //           q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    //           q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;
    // RUNGE_KUTTA 法解微分方程
    k10 = 0.5 * (-gx * q1 - gy * q2 - gz * q3);
    k11 = 0.5 * (gx * q0 + gz * q2 - gy * q3);
    k12 = 0.5 * (gy * q0 - gz * q1 + gx * q3);
    k13 = 0.5 * (gz * q0 + gy * q1 - gx * q2);

    k20 = 0.5 * (halfT * (q0 + halfT * k10) + (halfT - gx) * (q1 + halfT * k11) + (halfT - gy) * (q2 + halfT * k12) + (halfT - gz) * (q3 + halfT * k13));
    k21 = 0.5 * ((halfT + gx) * (q0 + halfT * k10) + halfT * (q1 + halfT * k11) + (halfT + gz) * (q2 + halfT * k12) + (halfT - gy) * (q3 + halfT * k13));
    k22 = 0.5 * ((halfT + gy) * (q0 + halfT * k10) + (halfT - gz) * (q1 + halfT * k11) + halfT * (q2 + halfT * k12) + (halfT + gx) * (q3 + halfT * k13));
    k23 = 0.5 * ((halfT + gz) * (q0 + halfT * k10) + (halfT + gy) * (q1 + halfT * k11) + (halfT - gx) * (q2 + halfT * k12) + halfT * (q3 + halfT * k13));

    k30 = 0.5 * (halfT * (q0 + halfT * k20) + (halfT - gx) * (q1 + halfT * k21) + (halfT - gy) * (q2 + halfT * k22) + (halfT - gz) * (q3 + halfT * k23));
    k31 = 0.5 * ((halfT + gx) * (q0 + halfT * k20) + halfT * (q1 + halfT * k21) + (halfT + gz) * (q2 + halfT * k22) + (halfT - gy) * (q3 + halfT * k23));
    k32 = 0.5 * ((halfT + gy) * (q0 + halfT * k20) + (halfT - gz) * (q1 + halfT * k21) + halfT * (q2 + halfT * k22) + (halfT + gx) * (q3 + halfT * k23));
    k33 = 0.5 * ((halfT + gz) * (q0 + halfT * k20) + (halfT + gy) * (q1 + halfT * k21) + (halfT - gx) * (q2 + halfT * k22) + halfT * (q3 + halfT * k23));

    k40 = 0.5 * (dt * (q0 + dt * k30) + (dt - gx) * (q1 + dt * k31) + (dt - gy) * (q2 + dt * k32) + (dt - gz) * (q3 + dt * k33));
    k41 = 0.5 * ((dt + gx) * (q0 + dt * k30) + dt * (q1 + dt * k31) + (dt + gz) * (q2 + dt * k32) + (dt - gy) * (q3 + dt * k33));
    k42 = 0.5 * ((dt + gy) * (q0 + dt * k30) + (dt - gz) * (q1 + dt * k31) + dt * (q2 + dt * k32) + (dt + gx) * (q3 + dt * k33));
    k43 = 0.5 * ((dt + gz) * (q0 + dt * k30) + (dt + gy) * (q1 + dt * k31) + (dt - gx) * (q2 + dt * k32) + dt * (q3 + dt * k33));

    q0 = q0 + dt / 6.0 * (k10 + 2 * k20 + 2 * k30 + k40);
    q1 = q1 + dt / 6.0 * (k11 + 2 * k21 + 2 * k31 + k41);
    q2 = q2 + dt / 6.0 * (k12 + 2 * k22 + 2 * k32 + k42);
    q3 = q3 + dt / 6.0 * (k13 + 2 * k23 + 2 * k33 + k43);

    // normalise quaternion
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;

    *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;                                    // pitch
    *roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;     // roll
    *yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3; // yaw
}

/*/////////////////////////////////////////////
*@功能：补偿陀螺仪漂移
*
*
/////////////////////////////////////////////*/
void calibrate(void)
{
    u8 t;
    short gx, gy, gz, sumx = 0, sumy = 0, sumz = 0;
    for (t = 0; t < 10; t++)
    {
        MPU_Get_Gyroscope(&gx, &gy, &gz);
        sumx = sumx + gx;
        sumy = sumy + gy;
        sumz = sumz + gz;
    }
    gyro_offsetx = -sumx / 10;
    gyro_offsety = -sumy / 10;
    gyro_offsetz = -sumz / 10;
}