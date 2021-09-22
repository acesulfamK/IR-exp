# -*- coding: utf-8 -*-

from __future__ import unicode_literals, print_function, division, absolute_import
import math
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import Quaternion, TransformStamped


def get_current_time_sec():
    """
    現在時刻を返す関数

    Return
    ------
        現在時刻 [s]

    """
    return rospy.Time.now().to_sec()


def quaternion_from_euler(yaw, pitch, roll):
    """
    オイラー角からクオータニオンに変換する関数

    Parameters
    ----------
        yaw (float): 入力yaw値 [deg]
        pitch (float): 入力pitch値 [deg]
        roll (float): 入力roll値 [deg]

    Return
    ------
        ロール、ピッチ、ヨーの順番で回転した場合のクオータニオン

    """

    # ロール、ピッチ、ヨーの順番で回転
    q = tf.transformations.quaternion_from_euler(yaw / 180.0 * math.pi,
                                                 pitch / 180.0 * math.pi,
                                                 roll / 180.0 * math.pi, 'rzyx')
    return Quaternion(q[0], q[1], q[2], q[3])
