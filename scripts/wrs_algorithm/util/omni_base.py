# -*- coding: utf-8 -*-

import math
import rospy
import tf
import actionlib
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from wrs_algorithm.util.mathematics import quaternion_from_euler


# 速度指令のパブリッシャーを作成
base_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)

def cmd_vel(vx, vy, vw):
    u"""台車を速度制御する関数

    引数:
        vx (float): 直進方向の速度指令値 [m/s]（前進が正、後進が負）
        vy (float): 横方向の速度指令値 [m/s]（左が正、右が負）
        vw (float): 回転方向の速度指令値 [deg/s]（左回転が正、右回転が負）

    """

    # 速度指令値をセットします
    twist = Twist()
    twist.linear.x = vx
    twist.linear.y = vy
    twist.angular.z = vw / 180.0 * math.pi  # 「度」から「ラジアン」に変換します
    base_vel_pub.publish(twist)  # 速度指令をパブリッシュします


# 自律移動のゴールを送信するクライアントを作成
navclient = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

def go_abs(x, y, theta):
    u"""台車の自律移動のゴールを指定する関数

    引数：
        x (float): 目標のx値 [m]
        y (float): 目標のy値 [m]
        theta (float): 目標の回転角度 [deg]

    返り値:
        ゴールに到達したらTrue, そうでなければFalse

    """

    goal = MoveBaseGoal()

    # "map"座標を基準座標に指定
    goal.target_pose.header.frame_id = "map"

    # ゴールのx,y座標をセットします
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    # 角度はクオータニオンという形式で与えます。そのため、オイラー角からクオータニオンに変換します
    goal.target_pose.pose.orientation = quaternion_from_euler(0, 0, theta)

    # ゴールを送信
    navclient.send_goal(goal)
    navclient.wait_for_result()
    state = navclient.get_state()
    # 成功すると、3が返ってくる
    # http://docs.ros.org/fuerte/api/actionlib_msgs/html/msg/GoalStatus.html
    return True if state == 3 else False
