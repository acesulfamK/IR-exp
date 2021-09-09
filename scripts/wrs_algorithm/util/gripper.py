# -*- coding: utf-8 -*-

import moveit_commander
import rospy


# moveitでの制御対象としてハンドを指定
gripper = moveit_commander.MoveGroupCommander("gripper")


def command(v):
    u"""ハンドを制御

    引数:
        v (float): ハンドの開き具合 (0：閉じる、1:開く)

    返り値:
        正しく動作すればTrue, そうでなければFalse

    """

    gripper.set_joint_value_target("hand_motor_joint", v)
    success = gripper.go()
    rospy.sleep(6)
    return success
