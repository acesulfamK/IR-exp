# -*- coding: utf-8 -*-

import os
import glob
import cv2
import numpy as np
import rospy
import ros_numpy
import tf
from sensor_msgs.msg import LaserScan, PointCloud2


class Laser():
    u"""レーザ情報を扱うクラス"""

    def __init__(self):
        # レーザースキャンのサブスクライバのコールバックに_laser_cbメソッドを登録
        self._laser_sub = rospy.Subscriber('/hsrb/base_scan',
                                           LaserScan, self._laser_cb)
        self._scan_data = None

    def _laser_cb(self, msg):
        # レーザスキャンのコールバック関数
        self._scan_data = msg

    def get_data(self):
        u"""レーザの値を取得する関数"""
        return self._scan_data


def get_object_dict():
    u"""Gazeboに出現させる物体の辞書を返す関数

    返り値:
        物体の辞書

    """

    object_dict = {}
    paths = glob.glob("/opt/ros/melodic/share/tmc_wrs_gazebo_worlds/models/ycb*")
    for path in paths:
        file = os.path.basename(path)
        object_dict[file[8:]] = file

    return object_dict


def get_object_list():
    u"""Gazeboに出現させる物体のリストを返す関数

    返り値:
        物体のリスト

    """

    object_list = get_object_dict().values()
    object_list.sort()
    for i in range(len(object_list)):
        object_list[i] = object_list[i][8:]

    return object_list


def put_object(name, x, y, z):
    u"""Gazeboに物体を出現させる関数"""

    cmd = "rosrun gazebo_ros spawn_model -database " \
          + str(get_object_dict()[name]) \
          + " -sdf -model " + str(name) \
          + " -x " + str(y - 2.1) + \
          " -y " + str(-x + 1.2) \
          + " -z " + str(z)
    subprocess.call(cmd.split())


def delete_object(name):
    u"""Gazeboの物体を消す関数

    引数:
        name (str): 物体の名前

    """

    cmd = ['rosservice', 'call', 'gazebo/delete_model',
           '{model_name: ' + str(name) + '}']
    subprocess.call(cmd)


class RGBD():
    u"""RGB-Dデータを扱うクラス"""

    def __init__(self):
        self._br = tf.TransformBroadcaster()
        # ポイントクラウドのサブスクライバのコールバックに_cloud_cbメソッドを登録
        self._cloud_sub = rospy.Subscriber(
            "/hsrb/head_rgbd_sensor/depth_registered/rectified_points",
            PointCloud2, self._cloud_cb)
        self._points_data = None
        self._image_data = None
        self._h_image = None
        self._region = None
        self._h_min = 0
        self._h_max = 0
        self._xyz = [0, 0, 0]
        self._frame_name = None

    def _cloud_cb(self, msg):
        # ポイントクラウドを取得する
        self._points_data = ros_numpy.numpify(msg)

        # 画像を取得する
        self._image_data = \
            self._points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]

        # 色相画像を作成する
        hsv_image = cv2.cvtColor(self._image_data, cv2.COLOR_RGB2HSV_FULL)
        self._h_image = hsv_image[..., 0]

        # 色相の閾値内の領域を抽出する
        self._region = \
            (self._h_image > self._h_min) & (self._h_image < self._h_max)

        # 領域がなければ処理を終える
        if not np.any(self._region):
            return

        # 領域からxyzを計算する
        (y_idx, x_idx) = np.where(self._region)
        x = np.average(self._points_data['x'][y_idx, x_idx])
        y = np.average(self._points_data['y'][y_idx, x_idx])
        z = np.average(self._points_data['z'][y_idx, x_idx])
        self._xyz = [x, y, z]

        # 座標の名前が設定されてなければ処理を終える
        if self._frame_name is None:
            return

        # tfを出力する
        self._br.sendTransform(
            (x, y, z), tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs),
            self._frame_name,
            msg.header.frame_id)

    def get_image(self):
        u"""画像を取得する関数"""
        return self._image_data

    def get_points(self):
        u"""ポイントクラウドを取得する関数"""
        return self._points_data

    def get_h_image(self):
        u"""色相画像を取得する関数"""
        return self._h_image

    def get_region(self):
        u"""抽出領域の画像を取得する関数"""
        return self._region

    def get_xyz(self):
        u"""抽出領域から計算されたxyzを取得する関数"""
        return self._xyz

    def set_h(self, h_min, h_max):
        u"""色相の閾値を設定する関数"""
        self._h_min = h_min
        self._h_max = h_max

    def set_coordinate_name(self, name):
        u"""座標の名前を設定する関数"""
        self._frame_name = name
