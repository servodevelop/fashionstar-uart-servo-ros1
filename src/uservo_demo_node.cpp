/* 
 * 舵机控制节点(Demo) 
 */
// 导入ROS依赖
#include "ros/ros.h"
#include "fashionstar_uart_servo_ros1/SetServoAngle.h"
#include "fashionstar_uart_servo_ros1/SetServoDamping.h"
#include "fashionstar_uart_servo_ros1/QueryServoAngle.h"

#include "CSerialPort/SerialPort.h"
#include "FashionStar/UServo/FashionStar_UartServoProtocol.h"
#include "FashionStar/UServo/FashionStar_UartServo.h"

using namespace fsuservo;
using namespace fashionstar_uart_servo_ros1;

// 参数定义
#define SERVO_PORT_NAME "/dev/ttyUSB0" 	// Linux下端口号名称 /dev/ttyUSB{}
#define SERVO_ID 0 				        // 舵机ID号

// 创建协议对象
FSUS_Protocol protocol(SERVO_PORT_NAME, FSUS_DEFAULT_BAUDRATE);
// 创建一个舵机对象
FSUS_Servo servo0(SERVO_ID, &protocol);

/* 舵机角度设置回调函数 */
void set_servo_angle_callback(const SetServoAngle& data){
    ROS_INFO("[RECV] Servo ID = %d Set Angle = %.1f", data.id, data.angle);
    // 设置舵机角度
    servo0.setRawAngle(data.angle, 0);
}

/* 舵机阻尼模式回调函数 */
void set_servo_damping_callback(const SetServoDamping& data){
    ROS_INFO("[RECV] Servo ID = %d, Set Damping Power = %d", data.id, data.power);
    servo0.setDamping(data.power);
}

/* 舵机角度查询 */
bool query_servo_angle_callback(QueryServoAngle::Request& req, QueryServoAngle::Response& res){
    float angle;
    // 通过SDK查询舵机角度 
    angle = servo0.queryRawAngle();
    // 填写返回数据包
    res.angle = angle;
    // 打印日志
    ROS_INFO("[RECV] Servo ID = %d  , Query Servo Angle = %.1f", req.id,  angle);
    // 注: C++版本的 服务回调函数，最后一定要返回一个bool值
    return true;
}

int main(int argc, char **argv)
{
    // 创建节点名称
    ros::init(argc, argv, "uservo_demo_node");
    // 创建NodeHandle
    ros::NodeHandle node_handle;

    // 创建接收者
    ros::Subscriber set_servo_angle_sub = node_handle.subscribe("set_servo_angle", 2, set_servo_angle_callback);
    ros::Subscriber set_servo_damping_sub = node_handle.subscribe("set_servo_damping", 2, set_servo_damping_callback);
    // 创建服务
    ros::ServiceServer query_servo_angle_srv = node_handle.advertiseService("query_servo_angle", query_servo_angle_callback);
    // 进入循环等待
    ros::spin();
}