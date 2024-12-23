#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
import tf2_ros
import tf2_geometry_msgs
import math
 
def move_to_goal(x, y):
    # Initialize node
    rospy.init_node('send_goal_node', anonymous=False)
 
    # Create a MoveBaseAction object
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
 
    # Wait for the action server to come up
    while not client.wait_for_server(rospy.Duration(5)):
        rospy.loginfo("Waiting for move_base action server...")
 
    # Define a goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0
 
    # Send the goal and wait for completion
    client.send_goal(goal)
    rospy.loginfo("Goal sent!")
    client.wait_for_result()
 
    # Print the result
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached!")
        return True
    else:
        rospy.loginfo("Failed to reach goal.")
        return False


# 定义绕圈的线速度和角速度
LINEAR_SPEED = 0.5  # 线速度 (m/s)
TURN_RADIUS = 1.5  # 转弯半径 (m)
STOP_TIME = 2.0

def start_circling():
    # 创建一个发布速度命令的 Publisher
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # 计算所需的角速度
    angular_speed = LINEAR_SPEED / TURN_RADIUS  # 角速度 = 线速度 / 转弯半径

    # 创建一个 Twist 消息
    twist_msg = Twist()
    twist_msg.linear.x = LINEAR_SPEED  # 线速度
    twist_msg.angular.z = angular_speed  # 角速度（逆时针）

    # 计算每转过 90 度所需的时间
    # time_per_90_degrees = (math.pi / 2) / angular_speed  # 90 度对应的时间
    # 计算绕一圈所需的时间
    # time_to_circle = (2 * math.pi) / angular_speed  # 一圈对应的时间

    # 初始化 TF 缓冲区和监听器
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    # 获取初始方向（yaw 角）
    try:
        # 获取 body 在 map 中的初始方向
        transform = tf_buffer.lookup_transform("map", "body", rospy.Time(0), rospy.Duration(5.0))
        initial_yaw = tf2_geometry_msgs.transform_to_pose(transform).orientation
        initial_yaw_angle = euler_from_quaternion([initial_yaw.x, initial_yaw.y, initial_yaw.z, initial_yaw.w])[2]
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr(f"TF lookup failed: {e}")
        return

    # 计算每转过 90 度所需的角度变化
    angle_per_90_degrees = math.pi / 2  # 90 度对应的角度变化




    # 发布速度命令，让机器人绕圈
    rate = rospy.Rate(10)  # 10 Hz
    rospy.loginfo("Starting to circle...")
    
    # 在最开始停 1 秒
    rospy.loginfo("Initial pause for 1 second...")
    current_yaw = initial_yaw_angle
    rospy.sleep(STOP_TIME)
    per_90_num = 0

    start_time = rospy.Time.now()  # 开始绕圈的初始时间
    last_90_degree_time = start_time  # 上一次转过 90 度的时间

    while not rospy.is_shutdown():
        try:
            # 获取当前 body 在 map 中的方向
            transform = tf_buffer.lookup_transform("map", "body", rospy.Time(0), rospy.Duration(1.0))
            current_yaw = euler_from_quaternion([transform.transform.rotation.x,
                                                 transform.transform.rotation.y,
                                                 transform.transform.rotation.z,
                                                 transform.transform.rotation.w])[2]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"TF lookup failed: {e}")
            continue

        # 计算当前方向与初始方向的差值
        angle_diff = current_yaw - initial_yaw_angle
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # 归一化到 [-pi, pi]




        # 检查是否转过 90 度
        if (rospy.Time.now() - last_90_degree_time).to_sec() >= time_per_90_degrees:
            rospy.loginfo("Reached 90 degrees. Stopping for 1 second...")
            stop_robot(cmd_vel_pub)  # 停止机器人
            rospy.sleep(STOP_TIME)  # 停 1 秒
            per_90_num = per_90_num + 1
            last_90_degree_time = rospy.Time.now()  # 更新上一次转过 90 度的时间
            rospy.loginfo("Resuming circling...")
            
            # 检查是否绕完一圈
            if per_90_num >= 4:
                rospy.loginfo("Finished circling one full loop.")
                break

        cmd_vel_pub.publish(twist_msg)
        rate.sleep()

    stop_robot(cmd_vel_pub)  # 停止机器人


def stop_robot(cmd_vel_pub):
    # 创建一个空的 Twist 消息，停止机器人
    stop_msg = Twist()
    cmd_vel_pub.publish(stop_msg)
    rospy.loginfo("Robot stopped.")

def wait_for_input():
    while not rospy.is_shutdown():
        # 提示用户输入（只打印一次）
        rospy.loginfo("Enter the goal coordinates (x y):")
        try:
            # 读取用户输入
            input_str = input("Input: ")
            # 将输入拆分为两个浮点数
            x, y = map(float, input_str.split())
            # 调用 move_to_goal 函数
            if move_to_goal(x, y):
                # 如果目标成功到达，开始绕圈
                start_circling()
            else:
                rospy.loginfo("Failed to reach goal. Please try again.")
        except ValueError:
            rospy.logerr("Invalid input! Please enter two numbers separated by a space.")
        except EOFError:
            rospy.loginfo("Exiting...")
            break

if __name__ == '__main__':
    rospy.sleep(3.0)
    try:
        wait_for_input()  # 等待用户输入
    except rospy.ROSInterruptException:
        pass