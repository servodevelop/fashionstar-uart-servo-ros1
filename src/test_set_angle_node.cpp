/* 
 * 测试舵机角度控制节点 
 */

#include "ros/ros.h"
// 自定义消息
#include "fashionstar_uart_servo_ros1/SetServoAngle.h"

using namespace fashionstar_uart_servo_ros1;

#define SERVO_ID 0 				        // 舵机ID号

int main(int argc, char **argv)
{
    // 创建节点名称
    ros::init(argc, argv, "test_set_angle_node");
    // 创建NodeHandle
    ros::NodeHandle node_handle;
    // 创建发布者
    ros::Publisher set_servo_angle_pub = node_handle.advertise<SetServoAngle>("set_servo_angle", 2);
    // 循环频率0.2HZ
    ros::Rate loop_rate(0.2);
    // 创建Message
    SetServoAngle msg;

    float angle = 90.0;
    while(ros::ok()){
        // 修改目标角度
        angle *= -1.0;
        // 构建Message
        msg.id = SERVO_ID;
        msg.angle = angle;
        // 发布消息
        set_servo_angle_pub.publish(msg);
        // 输出日志
        ROS_INFO("Set Servo %d Angle = %.1f", msg.id, msg.angle);
        // 延时等待
        loop_rate.sleep();
    }
}
