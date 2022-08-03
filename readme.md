传感器：
imu使用mpu6500
imu是贴着镜头安装的
原始imu坐标系 右-上-后 满足右手定则 
转换到前左上
Acc和Gyro都是AD值
Gyro 量程 1000deg/s 灵敏度 32.8 LSB/d
Acc 量程 4g 灵敏度 8192 LSB/g
imu更新频率 2000hz
GimbalGyro_x 用于控制的角速度 deg/s

芯片外设：
使用串口3进行roll-yaw板和pitch板的通信
使用串口1进行发送调试数据
使用定时器2捕获编码器数据

控制
频率： 2000Hz 0.5ms 解算和控制频率都是2000Hz

pitch板：
1.串口通信
2.接收编码器数据
3.计算相机姿态
4.计算控制量
5.发送控制量给roll-yaw板(2000hz发送一次电机控制量)
6.控制pitch轴
7.打印调试数据

roll-yaw板：
1.串口通信
2.接收控制量
3.控制roll和yaw轴
4.发送编码器数据给pitch板

通信流程：
pitch板定时发送一次数据给roll板，roll板收到后，返回编码器的电机角度给pitch板，pitch板解析，等待下一个周期

坐标系：
相机坐标系 前左上
电机坐标系 前左上

频率：
2000hz的任务：估计+控制

飞控姿态更新频率 83hz 12ms
飞控姿态使用频率 83hz 12ms

编码器的更新是定时器中断捕获计算的，直接更新结构体的值
编码器角度采样频率 1000hz
编码器角度使用频率 2000hz

IMU的采样更新是传感器内部的，更新好了等主机读取
角速度采样频率 8000hz
角速度使用频率 2000hz
加速度采样频率 4000hz
加速度更新频率 2000hz

姿态更新频率 2000hz
控制频率 2000hz 为什么要2000hz？

示波器更新频率 180hz/250hz

电机校准
```c
Get_Encoder.Angle_P = 0.703125f * ((u16)(duty_data + 225)%512);
Get_Encoder.Angle_R = 0.703125f * ((u16)(duty_data + 158)%512);
Get_Encoder.Angle_Y = 0.703125f * ((u16)(duty_data_Y + 102)%512);
```

编码器零点
```c
float roll_encoder_offset = 0.0f,pitch_encoder_offset = -50.0f,yaw_encoder_offset = 45.0f;
```