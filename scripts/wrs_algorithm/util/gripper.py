# -*- coding: utf-8 -*-

from __future__ import print_function, division, absolute_import
import moveit_commander
import rospy


# moveitでの制御対象としてハンドを指定
gripper = moveit_commander.MoveGroupCommander("gripper")


def command(v):
    """
    ハンドを制御

    Parameters
    ----------
        v (float): ハンドの開き具合 (0：閉じる、1:開く)

    Return
    ------
        正しく動作すればTrue, そうでなければFalse

    """

    gripper.set_joint_value_target("hand_motor_joint", v)
    success = gripper.go()
    rospy.sleep(6)
    return success
