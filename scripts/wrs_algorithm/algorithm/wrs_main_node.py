#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import unicode_literals, print_function, division, absolute_import
import json
import os
import traceback
import rospy
import rospkg
import tf2_ros
from detector_msgs.msg import BBox, BBoxArray
from detector_msgs.srv import SetTransformFromBBox, SetTransformFromBBoxRequest
from wrs_algorithm.util import omni_base, whole_body, gripper

class WrsMainController():
    """
    WRSのシミュレーション環境内でタスクを実行するクラス
    """
    IGNORE_LIST = ["dining table", "bench", "tv", "bed", "laptop", "person", "chair", "umbrella"]
    GRASP_TF_NAME = "object_grasping"
    GRASP_READY = 0.2

    def __init__(self):
        # intialize variable
        self.detected_obj = None
        self.detected_obj_req_time = rospy.Time.now()

        # load config file
        self.coordinates = self.load_json(self.get_path(["config", "coordinates.json"]))

        # ROS通信関連の初期化
        tf_from_bbox_srv_name = "set_tf_from_bbox"
        rospy.wait_for_service(tf_from_bbox_srv_name)
        self.tf_from_bbox_clt = rospy.ServiceProxy(
            tf_from_bbox_srv_name, SetTfFromBBox)

        obj_detection_name = "detection/bbox"
        self.pcd_sub = rospy.Subscriber(
            obj_detection_name, BBoxArray, callback=self.object_detection_cb)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    @staticmethod
    def get_path(pathes=[], package="wrs_algorithm"):
        """
        ROSパッケージ名を指定して、ファイルを取得する
        """
        if len(pathes) == 0:
            rospy.logerr("Can NOT resolve file path.")
        pkg_path = rospkg.RosPack().get_path(package)
        path = os.path.join(*pathes)
        return os.path.join(pkg_path, path)

    @staticmethod
    def load_json(path):
        """
        jsonファイルを辞書型で読み込む
        """
        with open(path, "r") as json_file:
            return json.load(json_file)

    def get_relative_coordinate(self, parent, child):
        """
        tfで相対座標を取得する
        """
        try:
            # 4秒待機して各tfが存在すれば相対関係をセット
            trans = self.tf_buffer.lookup_transform(parent, child,
                                                rospy.Time.now(),
                                                rospy.Duration(4.0))
            return trans.transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            log_str = "failed to get transform between [{}] and [{}]\n".format(parent, child)
            log_str += traceback.format_exc()
            rospy.logerr(log_str)
            return None

    def goto(self, name):
        """
        waypoint名で指定された場所に移動する
        """
        if name in self.coordinates["positions"].keys():
            wp = self.coordinates["positions"][name]
            rospy.loginfo("go to [%s](%.2f, %.2f, %.2f)", name, wp[0], wp[1], wp[2])
            return omni_base.go_abs(wp[0], wp[1], wp[2])

        rospy.loginfo("unknown waypoint name [%s]", name)
        return False

    def check_positions(self):
        """
        読み込んだ座標ファイルの座標を巡回する
        """
        whole_body.move_to_go()
        for wp_name in self.coordinates["routes"]["test"]:
            self.goto(wp_name)
            rospy.sleep(1)

    def object_detection_cb(self, msg):
        """
        物体認識の結果を取得する
        """
        if self.detected_obj is None and msg.header.stamp > self.detected_obj_req_time:
            self.detected_obj = msg

    def wait_until_get_latest_detection(self):
        """
        最新の認識結果が到着するまで待つ
        """
        self.detected_obj_req_time = rospy.Time.now()
        self.detected_obj = None
        while self.detected_obj is None:
            rospy.sleep(0.1)

    @classmethod
    def get_most_graspable_bbox(cls, obj_list):
        """
        最も把持が行えそうなbboxを一つ返す。
        """
        # ignoreリストで除外しながら、つかむべきかのscoreを計算する。
        extracted = []
        extract_str = "detected object list\n"
        ignore_str = ""
        for obj in obj_list:
            info_str = "{:<15}({:.2%}, {:3d}, {:3d}, {:3d}, {:3d})\n".format(obj.label, obj.score, obj.x, obj.y, obj.w, obj.h)
            if obj.label not in cls.IGNORE_LIST:
                score = cls.calc_score_bbox(obj)
                extracted.append({"bbox": obj, "score": score})
                extract_str += "- extracted: {:07.3f} ".format(score) + info_str
            else:
                ignore_str += "- ignored  : " + info_str
        rospy.loginfo(extract_str + ignore_str)

        # つかむべきかのscoreが一番高い物体を返す
        for obj_info in sorted(extracted, key=lambda x: x["score"], reverse=True):
            obj = obj_info["bbox"]
            info_str = "{} ({:.2%}, {:3d}, {:3d}, {:3d}, {:3d})\n".format(obj.label, obj.score, obj.x, obj.y, obj.w, obj.h)
            rospy.loginfo("selected bbox: " + info_str)
            return obj

        # objが一つもない場合は、Noneを返す
        return None

    @staticmethod
    def calc_score_bbox(bbox):
        """
        detector_msgs/BBoxのスコアを計算する
        """
        gravity_x = bbox.x + bbox.w / 2
        gravity_y = bbox.y + bbox.h / 2
        xy_diff = abs(320 - gravity_x) / 320 + abs(240 - gravity_y) / 240
        square = (bbox.w * bbox.h) / (200*200)
        return 1 / xy_diff + square # + bbox.score

    def execute_task1(self):
        """
        task1を実行する
        """
        rospy.loginfo("#### start Task 1 ####")
        for _ in range(1):
            # 探索位置・姿勢に移動
            whole_body.move_to_go()
            whole_body.move_head_tilt(-0.8)
            gripper.command(0)
            self.goto("check_floor_r")

            # 物体検出結果から、把持するbboxを決定
            self.wait_until_get_latest_detection()
            grasp_bbox = self.get_most_graspable_bbox(self.detected_obj.bboxes)
            if grasp_bbox is None:
                rospy.logwarn("Cannot determine object to grasp. Aborted.")
                continue

            # tfを生成して、座標を取得
            self.tf_from_bbox_clt.call(
                SetTfFromBBoxRequest(bbox=grasp_bbox, frame=self.GRASP_TF_NAME))
            rospy.sleep(1.0) # tfが安定するのを待つ
            grasp_pos = self.get_relative_coordinate("map", self.GRASP_TF_NAME).translation
            rospy.loginfo("move hand to (%.2f, %.2f, %.2f)", grasp_pos.x, grasp_pos.y, grasp_pos.z)

            # 把持
            gripper.command(1)
            whole_body.move_end_effector_pose(grasp_pos.x, grasp_pos.y, grasp_pos.z + self.GRASP_READY, -90, 180, 0)
            whole_body.move_end_effector_pose(grasp_pos.x, grasp_pos.y, grasp_pos.z, -90, 180, 0)
            gripper.command(0)
            whole_body.move_end_effector_pose(grasp_pos.x, grasp_pos.y, grasp_pos.z + self.GRASP_READY, -90, 180, 0)

            # binに入れる
            whole_body.move_to_go()
            self.goto("bin_a")
            whole_body.move_to_neutral()
            whole_body.move_end_effector_pose(2.2, -0.6, 0.45, 90, -90, 0)
            gripper.command(1)
            rospy.sleep(5.0)
            whole_body.move_to_neutral()



    def execute_task2a(self):
        """
        task2aを実行する
        """
        rospy.loginfo("#### start Task 2a ####")
        whole_body.move_to_go()
        whole_body.move_head_tilt(-1.0)
        self.goto("standby_2a")

        # 本来はここで障害物を取り除く処理が必要

        self.goto("go_throw_2a")
        whole_body.move_to_go()

    def execute_task2b(self):
        """
        task2bを実行する
        """
        rospy.loginfo("#### start Task 2b ####")
        whole_body.move_head_tilt(-0.3)
        self.goto("shelf")


    def run(self):
        """
        全てのタスクを実行する
        """
        self.execute_task1()
        # self.execute_task2a()
        # self.execute_task2b()


def main():
    rospy.init_node('main_controller')
    try:
        ctrl = WrsMainController()
        rospy.loginfo("node initialized [%s]", rospy.get_name())

        # タスクの実行モードを確認する
        if rospy.get_param("~test_mode", default=False) is True:
            rospy.loginfo("#### start with TEST mode. ####")
            ctrl.check_positions()
        else:
            rospy.loginfo("#### start with NORMAL mode. ####")
            ctrl.run()

    except rospy.ROSInterruptException:
        pass


if __name__=='__main__':
    main()
