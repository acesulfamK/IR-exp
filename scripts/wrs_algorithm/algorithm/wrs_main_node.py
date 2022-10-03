#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
WRS環境内でロボットを動作させるためのメインプログラム
"""

from __future__ import unicode_literals, print_function, division, absolute_import
import json
import os
# from select import select
import traceback
from turtle import pos
import rospy
import rospkg
import tf2_ros
from std_msgs.msg import String
from detector_msgs.srv import (
    SetTransformFromBBox, SetTransformFromBBoxRequest,
    GetObjectDetection, GetObjectDetectionRequest)
from wrs_algorithm.util import omni_base, whole_body, gripper


class WrsMainController(object):
    """
    WRSのシミュレーション環境内でタスクを実行するクラス
    """
    GRASP_OBJECT_LIST = ["apple", "sports_ball", "cup"]
    IGNORE_LIST = ["dining table", "bench", "tv", "bed", "laptop", "person", "chair", "umbrella"]
    GRASP_TF_NAME = "object_grasping"
    GRASP_BACK_SAFE = {"z": 0.05, "xy": 0.3}
    GRASP_BACK = {"z": 0.05, "xy": 0.1}
    HAND_PALM_OFFSET = 0.06  # hand_palm_linkは指の付け根なので、把持のために少しずらす必要がある

    def __init__(self):
        # 変数の初期化
        self.instruction_list = []

        # configファイルの受信
        self.coordinates = self.load_json(self.get_path(["config", "coordinates.json"]))
        self.poses = self.load_json(self.get_path(["config", "poses.json"]))

        # ROS通信関連の初期化
        tf_from_bbox_srv_name = "set_tf_from_bbox"
        rospy.wait_for_service(tf_from_bbox_srv_name)
        self.tf_from_bbox_clt = rospy.ServiceProxy(
            tf_from_bbox_srv_name, SetTransformFromBBox)

        obj_detection_name = "detection/get_object_detection"
        rospy.wait_for_service(obj_detection_name)
        self.detection_clt = rospy.ServiceProxy(
            obj_detection_name, GetObjectDetection)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.instruction_sub = rospy.Subscriber(
            "/message", String, self.instruction_cb, queue_size=10)

    @staticmethod
    def get_path(pathes, package="wrs_algorithm"):
        """
        ROSパッケージ名とファイルまでのパスを指定して、ファイルのパスを取得する
        """
        if not pathes:  # check if the list is empty
            rospy.logerr("Can NOT resolve file path.")
            raise ValueError("You must specify the path to file.")
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

    def instruction_cb(self, msg):
        """
        指示分を受信する
        """
        rospy.loginfo("instruction received. [%s]", msg.data)
        self.instruction_list.append(msg.data)

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
            pos = self.coordinates["positions"][name]
            rospy.loginfo("go to [%s](%.2f, %.2f, %.2f)", name, pos[0], pos[1], pos[2])
            return omni_base.go_abs(pos[0], pos[1], pos[2])

        rospy.logerr("unknown waypoint name [%s]", name)
        return False

    def goto_raw(self, r_pos):
        """
        waypoint名で指定された場所に移動する
        """
        rospy.loginfo("go to [raw_pos](%.2f, %.2f, %.2f)", r_pos[0], r_pos[1], r_pos[2])
        return omni_base.go_abs(r_pos[0], r_pos[1], r_pos[2])

    def change_pose(self, name):
        """
        指定された姿勢名に遷移する
        """
        if name in self.poses.keys():
            rospy.loginfo("change pose to [%s]", name)
            return whole_body.move_to_joint_positions(self.poses[name])

        rospy.logerr("unknown pose name [%s]", name)
        return False

    def check_positions(self):
        """
        読み込んだ座標ファイルの座標を巡回する
        """
        whole_body.move_to_go()
        for wp_name in self.coordinates["routes"]["test"]:
            self.goto(wp_name)
            rospy.sleep(1)

    def get_latest_detection(self):
        """
        最新の認識結果が到着するまで待つ
        """
        res = self.detection_clt(GetObjectDetectionRequest())
        return res.bboxes

    def get_grasp_coordinate(self, bbox):
        """
        BBox情報から把持座標を取得する
        """
        # BBox情報からtfを生成して、座標を取得
        self.tf_from_bbox_clt.call(
            SetTransformFromBBoxRequest(bbox=bbox, frame=self.GRASP_TF_NAME))
        rospy.sleep(1.0)  # tfが安定するのを待つ
        return self.get_relative_coordinate("map", self.GRASP_TF_NAME).translation

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
            info_str = "{:<15}({:.2%}, {:3d}, {:3d}, {:3d}, {:3d})\n".format(
                obj.label, obj.score, obj.x, obj.y, obj.w, obj.h)
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
            info_str = "{} ({:.2%}, {:3d}, {:3d}, {:3d}, {:3d})\n".format(
                obj.label, obj.score, obj.x, obj.y, obj.w, obj.h)
            rospy.loginfo("selected bbox: " + info_str)
            return obj

        # objが一つもない場合は、Noneを返す
        return None

    @classmethod
    def get_most_graspable_obj(cls, obj_list):
        """把持すべきscoreが最も高い物体を返す。

        Args:
            obj_list (_type_): _description_

        Returns:
            _type_: _description_
        """
        extracted = []
        extract_str = "detected object list\n"
        ignore_str = ""
        for obj in obj_list:
            info_str = "{:<15}({:.2%}, {:3d}, {:3d}, {:3d}, {:3d})\n".format(
                obj.label, obj.score, obj.x, obj.y, obj.w, obj.h)
            if obj.label not in cls.IGNORE_LIST:
                score = cls.calc_score_bbox(obj)
                extracted.append({"bbox": obj, "score": score, "label": obj.label})
                extract_str += "- extracted: {:07.3f} ".format(score) + info_str
            else:
                ignore_str += "- ignored  : " + info_str
        rospy.loginfo(extract_str + ignore_str)

        # つかむべきかのscoreが一番高い物体を返す
        for obj_info in sorted(extracted, key=lambda x: x["score"], reverse=True):
            obj = obj_info["bbox"]
            info_str = "{} ({:.2%}, {:3d}, {:3d}, {:3d}, {:3d})\n".format(
                obj.label, obj.score, obj.x, obj.y, obj.w, obj.h)
            rospy.loginfo("selected bbox: " + info_str)
            return obj_info

        # objが一つもない場合は、Noneを返す
        return None

    @classmethod
    def calc_score_bbox(cls, bbox):
        """
        detector_msgs/BBoxのスコアを計算する
        """
        label_score = 1 if bbox.label in cls.GRASP_OBJECT_LIST else 0
        gravity_x = bbox.x + bbox.w / 2
        gravity_y = bbox.y + bbox.h / 2
        xy_diff = abs(320 - gravity_x) / 320 + abs(360 - gravity_y) / 240

        return 1 / xy_diff + 2 * label_score

    def grasp_from_side(self, pos_x, pos_y, pos_z, yaw, pitch, roll, preliminary="-y"):
        """
        把持の一連の動作を行う

        Note
        ----
        - tall_tableに対しての予備動作を生成するときはpreliminary="-y"と設定することになる。
        """
        if preliminary not in ["+y", "-y", "+x", "-x"]:
            raise RuntimeError("unnkown graps preliminary type [{}]".format(preliminary))

        rospy.loginfo("move hand to grasp (%.2f, %.2f, %.2f)", pos_x, pos_y, pos_z)

        grasp_back_safe = {"x": pos_x, "y": pos_y, "z": pos_z + self.GRASP_BACK["z"]}
        grasp_back = {"x": pos_x, "y": pos_y, "z": pos_z + self.GRASP_BACK["z"]}
        grasp_pos = {"x": pos_x, "y": pos_y, "z": pos_z}

        if "+" in preliminary:
            sign = 1
        elif "-" in preliminary:
            sign = -1

        if "x" in preliminary:
            grasp_back_safe["x"] += sign * self.GRASP_BACK_SAFE["xy"]
            grasp_back["x"] += sign * self.GRASP_BACK["xy"]
        elif "y" in preliminary:
            grasp_back_safe["y"] += sign * self.GRASP_BACK_SAFE["xy"]
            grasp_back["y"] += sign * self.GRASP_BACK["xy"]

        gripper.command(1)
        whole_body.move_end_effector_pose(
            grasp_back_safe["x"], grasp_back_safe["y"], grasp_back_safe["z"], yaw, pitch, roll)
        whole_body.move_end_effector_pose(
            grasp_back["x"], grasp_back["y"], grasp_back["z"], yaw, pitch, roll)
        whole_body.move_end_effector_pose(
            grasp_pos["x"], grasp_pos["y"], grasp_pos["z"], yaw, pitch, roll)
        gripper.command(0)
        whole_body.move_end_effector_pose(
            grasp_back_safe["x"], grasp_back_safe["y"], grasp_back_safe["z"], yaw, pitch, roll)

    def grasp_from_front_side(self, grasp_pos):
        """正面把持を行う
        ややアームを下に向けている
        """
        grasp_pos.y -= self.HAND_PALM_OFFSET
        rospy.loginfo("grasp_from_front_side (%.2f, %.2f, %.2f)", grasp_pos.x, grasp_pos.y, grasp_pos.z)
        self.grasp_from_side(grasp_pos.x, grasp_pos.y, grasp_pos.z, -90, -100, 0, "-y")

    def grasp_from_upper_side(self, grasp_pos):
        """上面から把持を行う
        オブジェクトに寄るときは、y軸から近づく上面からは近づかない
        """
        # grasp_pos.z += self.HAND_PALM_OFFSET
        grasp_pos.z += 0.075
        rospy.loginfo("grasp_from_upper_side (%.2f, %.2f, %.2f)", grasp_pos.x, grasp_pos.y, grasp_pos.z)
        self.grasp_from_side(grasp_pos.x, grasp_pos.y, grasp_pos.z, -90, -160, 0, "-y")

    def exe_graspable_method(self, grasp_pos, label=""):
        """posの位置によって把持方法を判定し実行する。task1a用
        把持可能後半の判定が優先される
        """
        method = None
        graspable_y = 1.85  # これ以上奥は把持できない
        desk_y = 1.5
        desk_z = 0.35

        # 把持禁止判定
        if (graspable_y < grasp_pos.y and desk_z > grasp_pos.z):
            return False

        # 机の下である条件を満たす場合
        if (desk_y < grasp_pos.y and desk_z > grasp_pos.z):
            method = self.grasp_from_front_side
        else:
            method = self.grasp_from_upper_side

        # bowlの張り付き対策
        if label in ["cup", "frisbee", "bowl"]:
            method = self.grasp_from_front_side

        method(grasp_pos)
        return True

    def put_in_place(self, place):
        """指定場所に入れる事前位置に戻すまでのタスク
        """
        self.change_pose("move_with_looking_floor")
        self.goto(place)
        self.change_pose("all_neutral")
        self.change_pose("put_in_bin")
        gripper.command(1)
        rospy.sleep(5.0)
        self.change_pose("all_neutral")

    def into_bin_a(self):
        """箱に入れる事前位置に戻すまでのタスク
        """
        self.put_in_place("bin_a_place")

    def into_bin_b(self):
        """箱に入れる事前位置に戻すまでのタスク
        """
        self.put_in_place("bin_b_place")
        
    def execute_task1(self):
        """task1でスコア200点を目指し、かつオブジェクトとの衝突などを発生しないように実施する
        """
        rospy.loginfo("#### start Task 1 ####")
        hsr_position = [
            ("check_floor_tall_table", "move_with_looking_floor"),
            # ("check_floor_l", "move_with_looking_floor"),
            # ("check_floor_c", "move_with_looking_floor"),
            ("check_floor_r", "move_with_looking_floor"),
            ("tall_table", "look_at_tall_table"),
            # ("long_table_l", "look_at_tall_table"),
            ("long_table_c", "look_at_tall_table"),
            # ("long_table_r", "look_at_tall_table"),
        ]

        detect_cnt = 2
        for plc, look_at in hsr_position:
            for cnt in range(detect_cnt):
                # 移動と視線指示
                self.goto(plc)
                self.change_pose(look_at)
                gripper.command(0)

                # 把持対象の有無チェック
                detected_objs = self.get_latest_detection()
                graspable_obj = self.get_most_graspable_obj(detected_objs.bboxes)
                if graspable_obj is None:
                    rospy.logwarn("Cannot determine object to grasp. Grasping is aborted.")
                    continue
                label = graspable_obj["label"]
                grasp_bbox = graspable_obj["bbox"]

                # 把持対象がある場合は把持関数実施
                grasp_pos = self.get_grasp_coordinate(grasp_bbox)
                self.change_pose("grasp_on_table")
                self.exe_graspable_method(grasp_pos, label)
                self.change_pose("all_neutral")

                # binに入れる
                if cnt % 2 == 0:
                    self.into_bin_a()
                else:
                    self.into_bin_b()

    def execute_task2a(self):
        """
        task2aを実行する
        """
        rospy.loginfo("#### start Task 2a ####")
        self.change_pose("move_with_looking_floor")
        gripper.command(0)
        self.change_pose("look_at_near_floor")
        self.goto("standby_2a")

        detected_objs = self.get_latest_detection()
        grasp_bbox = self.get_most_graspable_bbox(detected_objs.bboxes)
        if grasp_bbox is not None:
            # 本来はここで障害物を除去する
            pass
        else:
            rospy.logwarn("Cannot find object to grasp. Grasping in task2a is aborted.")

        self.execute_avoid_blocks()

        self.goto("go_throw_2a")
        whole_body.move_to_go()

    def execute_task2b(self):
        """
        task2bを実行する
        """
        rospy.loginfo("#### start Task 2b ####")

        for idx_trial in range(2):
            self.change_pose("move_with_looking_floor")
            self.goto("shelf")
            self.change_pose("look_at_shelf")

            # 物体検出結果から、把持するbboxを決定
            detected_objs = self.get_latest_detection()
            grasp_bbox = self.get_most_graspable_bbox(detected_objs.bboxes)
            if grasp_bbox is None:
                rospy.logwarn("Cannot find object to grasp. task2b is aborted.")
                return

            # BBoxの3次元座標を取得して、その座標で把持する
            grasp_pos = self.get_grasp_coordinate(grasp_bbox)
            grasp_pos.y -= self.HAND_PALM_OFFSET
            self.change_pose("grasp_on_shelf")
            self.grasp_from_side(grasp_pos.x, grasp_pos.y, grasp_pos.z, -90, -90, 0, "-y")
            self.change_pose("all_neutral")

            # 椅子の前に持っていく
            self.change_pose("move_with_looking_floor")
            if idx_trial == 0:
                self.goto("chair_a")
            elif idx_trial == 1:
                self.goto("chair_b")
            self.change_pose("deliver_to_human")
            rospy.sleep(10.0)
            gripper.command(1)
            self.change_pose("all_neutral")

    def execute_avoid_blocks(self):
        """blockを避ける
        """
        for i in range(3):
            detected_objs = self.get_latest_detection()
            # 取得した物体の座標を算出
            bboxes = detected_objs.bboxes
            pos_bboxes = [self.get_grasp_coordinate(bbox) for bbox in bboxes]
            waypoint = self.select_next_waypoint(i, pos_bboxes)
            rospy.loginfo(waypoint)
            self.goto_raw(waypoint)

    def select_next_waypoint(self, currentstp, pos_bboxes):
        """waypoints から近い場所にあるものを除外し、最適なwaypointを返す。
        x座標を原点に近い方からxa,xb,xcに分け、移動先を決定する(デフォルトは0.4間隔)
        pos_bboxesは get_grasp_coordinate() 済みであること
        """
        pos_xa = 1.7
        pos_xb = 2.1
        pos_xc = 2.5
        waypoints = {
            "xa": [[pos_xa, 2.5, 45], [pos_xa, 2.9, 45], [pos_xa, 3.3, 90]],
            "xb": [[pos_xb, 2.5, 90], [pos_xb, 2.9, 90], [pos_xb, 3.3, 90]],
            "xc": [[pos_xc, 2.5, 135], [pos_xc, 2.9, 135], [pos_xc, 3.3, 90]]
        }

        # 原点側からxa,xb,xcのラインに近い場合は候補から削除
        is_to_xa = True
        is_to_xb = True
        is_to_xc = True
        for bbox in pos_bboxes:
            pos_x = bbox.x
            rospy.loginfo(bbox.x)
            # NOTE ｙ座標次第で無視してよいオブジェクトもある。
            if pos_x < pos_xa + 0.2:
                is_to_xa = False
                rospy.loginfo("is_to_xa=False")
                continue
            elif pos_x < pos_xb + 0.2:
                is_to_xb = False
                rospy.loginfo("is_to_xb=False")
                continue
            elif pos_x < pos_xc + 0.2:
                is_to_xc = False
                rospy.loginfo("is_to_xc=False")
                continue

        x_line = None   # xa,xb,xcいずれかのリストが入る
        # NOTE 優先的にxcに行く
        if is_to_xc:
            x_line = waypoints["xc"]
            rospy.loginfo("select next waypoint_xc")
        elif is_to_xb:
            x_line = waypoints["xb"]
            rospy.loginfo("select next waypoint_xb")
        elif is_to_xa:
            x_line = waypoints["xa"]
            rospy.loginfo("select next waypoint_xa")
        else:
            # a,b,cいずれにも移動できない場合
            x_line = waypoints["xb"]
            rospy.loginfo("select default waypoint")

        return x_line[currentstp]

    def run(self):
        """
        全てのタスクを実行する
        """
        self.change_pose("all_neutral")
        self.execute_task1()
        self.execute_task2a()
        self.execute_task2b()


def main():
    """
    WRS環境内でタスクを実行するためのメインノードを起動する
    """
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


if __name__ == '__main__':
    main()
