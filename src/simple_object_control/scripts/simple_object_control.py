#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
import math

class SimpleObjectControl:
    def __init__(self):
        rospy.init_node("simple_object_control")
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", geometry_msgs.msg.Twist, self.cmd_vel_callback)
        self.position = [0.0, 0.0, 0.0]  # x, y, z
        self.velocity = [0.0, 0.0, 0.0]  # linear.x, linear.y, angular.z
        self.yaw = 0.0  # 初始角度

    def cmd_vel_callback(self, msg):
        """
        回调函数，处理 /cmd_vel 消息
        """
        self.velocity[0] = msg.linear.x  # linear.x (前进速度)
        self.velocity[1] = msg.linear.y  # linear.y (侧向速度)
        self.velocity[2] = msg.angular.z  # angular.z (旋转速度)

    def update_position(self):
        """
        更新物体的位置和旋转
        """
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # 根据当前朝向 (yaw) 计算前进方向的速度分量
            dx = self.velocity[0] * math.cos(self.yaw)  # 沿 x 轴的速度分量
            dy = self.velocity[0] * math.sin(self.yaw)  # 沿 y 轴的速度分量

            # 更新位置
            self.position[0] += dx * 0.1  # 更新 x 坐标
            self.position[1] += dy * 0.1  # 更新 y 坐标

            # 更新旋转角度
            self.yaw += self.velocity[2] * 0.1

            # 创建变换消息
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "map"
            t.child_frame_id = "base_link"
            t.transform.translation.x = self.position[0]
            t.transform.translation.y = self.position[1]
            t.transform.translation.z = 0.0

            # 计算四元数
            quaternion = self.euler_to_quaternion(0, 0, self.yaw)
            t.transform.rotation.x = quaternion[0]
            t.transform.rotation.y = quaternion[1]
            t.transform.rotation.z = quaternion[2]
            t.transform.rotation.w = quaternion[3]

            # 发布变换
            self.tf_broadcaster.sendTransform(t)

            rate.sleep()

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        将欧拉角转换为四元数
        """
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]

if __name__ == "__main__":
    try:
        controller = SimpleObjectControl()
        controller.update_position()
    except rospy.ROSInterruptException:
        pass