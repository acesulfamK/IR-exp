#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import unicode_literals, print_function, division, absolute_import
import json
import os
from wrs_algorithm.util import omni_base, whole_body
import rospy
import rospkg

class WrsMainController():
    """
    WRSのシミュレーション環境内でタスクを実行するクラス
    """

    def __init__(self):
        # load config file
        self.coordinates = self.load_json(self.get_path(["config", "coordinates.json"]))

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

    def execute_task1(self):
        """
        task1を実行する
        """
        rospy.loginfo("#### start Task 1 ####")

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
        self.execute_task2a()
        self.execute_task2b()


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
