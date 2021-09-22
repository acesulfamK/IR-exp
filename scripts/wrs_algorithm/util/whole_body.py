# -*- coding: utf-8 -*-

from __future__ import unicode_literals, print_function, division, absolute_import
import actionlib
import moveit_commander
import rospy
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from wrs_algorithm.util.mathematics import quaternion_from_euler


# moveitでの制御対象として全身制御を指定
# whole_body_cmd = moveit_commander.MoveGroupCommander(str("whole_body_light"))
whole_body_cmd = moveit_commander.MoveGroupCommander(str("whole_body_weighted"))
whole_body_cmd.allow_replanning(True)
whole_body_cmd.set_workspace([-3.0, -3.0, 3.0, 3.0])


def move_end_effector_pose(x, y, z, yaw, pitch, roll):
    """
    ロボットを全身の逆運動学で制御する関数

    Parameters
    ----------
        x (float): エンドエフェクタの目標x値 [m]
        y (float): エンドエフェクタの目標y値 [m]
        z (float): エンドエフェクタの目標z値 [m]
        yaw (float): エンドエフェクタの目標yaw値 [deg]
        pitch (float): エンドエフェクタの目標pitch値 [deg]
        roll (float): エンドエフェクタの目標roll値 [deg]

    Return
    ------
        正しく動作すればTrue, そうでなければFalse

    """

    p = PoseStamped()

    # "map"座標を基準座標に指定
    p.header.frame_id = str("map")

    # エンドエフェクタの目標位置姿勢のx,y,z座標をセットします
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = z

    # オイラー角をクオータニオンに変換します
    p.pose.orientation = quaternion_from_euler(yaw, pitch, roll)

    # 目標位置姿勢をセット
    whole_body_cmd.set_pose_target(p)
    return whole_body_cmd.go()


# moveitでの制御対象としてアームを指定
arm_cmd = moveit_commander.MoveGroupCommander(str('arm'))


def move_to_neutral():
    """
    ロボットをニュートラルの姿勢に移動

    Return
    ------
        正しく動作すればTrue, そうでなければFalse

    """

    arm_cmd.set_named_target(str('neutral'))
    return arm_cmd.go()


def move_to_go():
    """
    ロボットを初期姿勢に移動

    Return
    ------
        正しく動作すればTrue, そうでなければFalse

    """

    arm_cmd.set_named_target(str('go'))
    return arm_cmd.go()


# moveitでの制御対象として頭部を指定
head_cmd = moveit_commander.MoveGroupCommander(str("head"))


def move_head_tilt(v):
    """
    ハンドを制御

    Parameters
    ----------
        v (float): 頭部の入力チルト角度 (マイナス:下向き、プラス:上向き)

    Return
    ------
        正しく動作すればTrue, そうでなければFalse

    """

    head_cmd.set_joint_value_target(str("head_tilt_joint"), v)
    return head_cmd.go()


def move_to_joint_positions(joints):
    """
    ハンドを制御

    Parameters
    ----------
        joints (dictionary): 各関節値

    Return
    ------
        正しく動作すればTrue, そうでなければFalse
    """
    ARM_JOINT_LIST = [
        'arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint',
        'wrist_flex_joint', 'wrist_roll_joint']
    HEAD_JOINT_LIST = ['head_pan_joint', 'head_tilt_joint']

    # arm関節の制御
    arm_traj, arm_cnt = _convert_to_ros_traj_msg(joints, ARM_JOINT_LIST)
    if arm_cnt > 0:
        arm_actionlib_name = '/hsrb/arm_trajectory_controller/follow_joint_trajectory'
        arm_client = actionlib.SimpleActionClient(arm_actionlib_name, FollowJointTrajectoryAction)
        arm_client.wait_for_server()
        arm_client.send_goal(FollowJointTrajectoryGoal(trajectory=arm_traj))

    # arm関節の制御
    head_traj, head_cnt = _convert_to_ros_traj_msg(joints, HEAD_JOINT_LIST)
    if head_cnt > 0:
        head_actionlib_name = '/hsrb/head_trajectory_controller/follow_joint_trajectory'
        head_client = actionlib.SimpleActionClient(head_actionlib_name, FollowJointTrajectoryAction)
        head_client.wait_for_server()
        head_client.send_goal(FollowJointTrajectoryGoal(trajectory=head_traj))

    # actionサーバの結果を待つ
    if arm_cnt > 0:
        arm_client.wait_for_result()
        arm_state = True if arm_client.get_result() == GoalStatus.SUCCEEDED else False
    else:
        arm_state = True

    if head_cnt > 0:
        head_client.wait_for_result()
        head_state = True if head_client.get_result() == GoalStatus.SUCCEEDED else False
    else:
        head_state = True

    return arm_state and head_state


def _convert_to_ros_traj_msg(joints, truth_list):
    """
    dictionary形式の関節値リストをROSメッセージに変換する
    """
    traj = JointTrajectory()
    point = JointTrajectoryPoint()
    for name, val in joints.items():
        if name in truth_list:
            traj.joint_names.append(name)
            point.positions.append(val)
            point.velocities.append(0)
    point.time_from_start = rospy.Time(1)
    traj.points = [point]

    return traj, len(traj.joint_names)
