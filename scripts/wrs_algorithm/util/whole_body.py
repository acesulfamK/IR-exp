# -*- coding: utf-8 -*-

from __future__ import unicode_literals, print_function, division, absolute_import
import moveit_commander
from geometry_msgs.msg import PoseStamped

from wrs_algorithm.util.mathematics import quaternion_from_euler


# moveitでの制御対象として全身制御を指定
whole_body = moveit_commander.MoveGroupCommander(str("whole_body_light"))
# whole_body = moveit_commander.MoveGroupCommander(str("whole_body_weighted"))
whole_body.allow_replanning(True)
whole_body.set_workspace([-3.0, -3.0, 3.0, 3.0])


def move_end_effector_pose(x, y, z, roll, pitch, yaw):
    """
    ロボットを全身の逆運動学で制御する関数

    Parameters
    ----------
        x (float): エンドエフェクタの目標x値 [m]
        y (float): エンドエフェクタの目標y値 [m]
        z (float): エンドエフェクタの目標z値 [m]
        roll (float): エンドエフェクタの目標roll値 [deg]
        pitch (float): エンドエフェクタの目標pitch値 [deg]
        yaw (float): エンドエフェクタの目標yaw値 [deg]

    Return
    ------
        正しく動作すればTrue, そうでなければFalse

    """

    p = PoseStamped()

    # "map"座標を基準座標に指定
    p.header.frame_id = str("/map")

    # エンドエフェクタの目標位置姿勢のx,y,z座標をセットします
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = z

    # オイラー角をクオータニオンに変換します
    p.pose.orientation = quaternion_from_euler(roll, pitch, yaw)

    # 目標位置姿勢をセット
    whole_body.set_pose_target(p)
    return whole_body.go()


# moveitでの制御対象としてアームを指定
arm = moveit_commander.MoveGroupCommander(str('arm'))


def move_to_neutral():
    """
    ロボットをニュートラルの姿勢に移動

    Return
    ------
        正しく動作すればTrue, そうでなければFalse

    """

    arm.set_named_target(str('neutral'))
    return arm.go()


def move_to_go():
    """
    ロボットを初期姿勢に移動

    Return
    ------
        正しく動作すればTrue, そうでなければFalse

    """

    arm.set_named_target(str('go'))
    return arm.go()


# moveitでの制御対象として頭部を指定
head = moveit_commander.MoveGroupCommander(str("head"))


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

    head.set_joint_value_target(str("head_tilt_joint"), v)
    return head.go()
