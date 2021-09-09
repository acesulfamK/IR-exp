# -*- coding: utf-8 -*-

import moveit_commander
from geometry_msgs.msg import PoseStamped

from wrs_algorithm.util.mathematics import quaternion_from_euler


# moveitでの制御対象として全身制御を指定
whole_body = moveit_commander.MoveGroupCommander("whole_body_light")
# whole_body = moveit_commander.MoveGroupCommander("whole_body_weighted")
whole_body.allow_replanning(True)
whole_body.set_workspace([-3.0, -3.0, 3.0, 3.0])


def move_end_effector_pose(x, y, z, roll, pitch, yaw):
    u"""ロボットを全身の逆運動学で制御する関数

    引数：
        x (float): エンドエフェクタの目標x値 [m]
        y (float): エンドエフェクタの目標y値 [m]
        z (float): エンドエフェクタの目標z値 [m]
        roll (float): エンドエフェクタの目標roll値 [deg]
        pitch (float): エンドエフェクタの目標pitch値 [deg]
        yaw (float): エンドエフェクタの目標yaw値 [deg]

    返り値:
        正しく動作すればTrue, そうでなければFalse

    """

    p = PoseStamped()

    # "map"座標を基準座標に指定
    p.header.frame_id = "/map"

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
arm = moveit_commander.MoveGroupCommander('arm')


def move_to_neutral():
    u"""ロボットをニュートラルの姿勢に移動

    返り値:
        正しく動作すればTrue, そうでなければFalse

    """

    arm.set_named_target('neutral')
    return arm.go()


def move_to_go():
    u"""ロボットを初期姿勢に移動

    返り値:
        正しく動作すればTrue, そうでなければFalse

    """

    arm.set_named_target('go')
    return arm.go()


# moveitでの制御対象として頭部を指定
head = moveit_commander.MoveGroupCommander("head")


def move_head_tilt(v):
    u"""ハンドを制御

    引数:
        v (float): 頭部の入力チルト角度 (マイナス:下向き、プラス:上向き)

    返り値:
        正しく動作すればTrue, そうでなければFalse

    """

    head.set_joint_value_target("head_tilt_joint", v)
    return head.go()
