/* 
 * 测试舵机角度查询节点 
 */
#include "ros/ros.h"
// 自定义消息
#include "fashionstar_uart_servo_ros1/SetServoDamping.h"
// 自定义服务
#include "fashionstar_uart_servo_ros1/QueryServoAngle.h"


using namespace fashionstar_uart_servo_ros1;

#define SERVO_ID 0 				        // 舵机ID号

int main(int argc, char **argv)
{
    // 创建节点名称
    ros::init(argc, argv, "test_set_angle_node");
    // 创建NodeHandle
    ros::NodeHandle node_handle;
    // 创建发布者
    ros::Publisher set_servo_damping_pub = node_handle.advertise<SetServoDamping>("set_servo_damping", 2);
    // 创建阻尼模式Message
    SetServoDamping damping_msg;

    // 角度查询服务客户端
    ros::ServiceClient query_angle_client = node_handle.serviceClient<QueryServoAngle>("query_servo_angle");
    // 创建服务对象
    QueryServoAngle query_angle_srv;

    // 等待角度查询服务开启
    query_angle_client.waitForExistence();

    // 设置舵机为阻尼模式
    damping_msg.id = SERVO_ID;
    damping_msg.power = 400;
    set_servo_damping_pub.publish(damping_msg);

    // 循环频率1HZ
    ros::Rate loop_rate(1);
    float angle = 90.0;
    while(ros::ok()){
        // 构造服务请求头
        query_angle_srv.request.id = SERVO_ID;
        // 发送服务请求
        if(query_angle_client.call(query_angle_srv)){
            // 服务成功
            ROS_INFO("[Send]Query Servo %d Angle %.1f", SERVO_ID, query_angle_srv.response.angle);
        }else{
            // 发送失败
            ROS_INFO("[Send]Query Servo %d Angle Failed", SERVO_ID);
        }
        // 延时等待
        loop_rate.sleep();
    }
}
