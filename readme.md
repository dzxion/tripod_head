传感器：
imu使用mpu6500
imu是贴着镜头安装的
原始imu坐标系 右-上-后 满足右手定则 
转换到前左上
Acc和Gyro都是AD值
Gyro 量程 1000deg/s 灵敏度 32.8
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
5.发送控制量给roll-yaw板
6.控制pitch轴
7.打印调试数据

roll-yaw板：
1.串口通信
2.接收控制量
3.控制roll和yaw轴
4.发送编码器数据给pitch板

坐标系：
相机坐标系 前左上
电机坐标系 前左上

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