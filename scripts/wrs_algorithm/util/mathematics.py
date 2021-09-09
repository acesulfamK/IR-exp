# -*- coding: utf-8 -*-

from __future__ import unicode_literals
from __future__ import print_function
from __future__ import division
from __future__ import absolute_import
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


def quaternion_from_euler(roll, pitch, yaw):
    """
    オイラー角からクオータニオンに変換する関数

    Parameters
    ----------
        roll (float): 入力roll値 [deg]
        pitch (float): 入力pitch値 [deg]
        yaw (float): 入力yaw値 [deg]

    Return
    ------
        ロール、ピッチ、ヨーの順番で回転した場合のクオータニオン

    """

    # ロール、ピッチ、ヨーの順番で回転
    q = tf.transformations.quaternion_from_euler(roll / 180.0 * math.pi,
                                                 pitch / 180.0 * math.pi,
                                                 yaw / 180.0 * math.pi, 'rxyz')
    return Quaternion(q[0], q[1], q[2], q[3])


def get_relative_coordinate(parent, child):
    """
    相対座標を取得する関数

    Parameters
    ----------
        parent (str): 親の座標系
        child (str): 子の座標系

    """

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    trans = TransformStamped()
    while not rospy.is_shutdown():
        try:
            # 4秒待機して各tfが存在すれば相対関係をセット
            trans = tfBuffer.lookup_transform(parent, child,
                                              rospy.Time().now(),
                                              rospy.Duration(4.0))
            break
        except (tf2_ros.ExtrapolationException):
            pass

    return trans.transform
