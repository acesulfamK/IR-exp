#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import unicode_literals, print_function, division, absolute_import
import sys
import traceback
import rospy

from wrs_algorithm.util import omni_base, whole_body, gripper, simulator, mathematics

if __name__=='__main__':

    rospy.init_node('move')
    try:
        # 視線を少し下げる
        whole_body.move_head_tilt(-0.7)
    except:
        rospy.logerr('fail to init')
        traceback.print_exc()
        sys.exit()

    try:
        # 移動姿勢
        whole_body.move_to_go()
        # 長テーブルの前に移動
        omni_base.go_abs(1, 0.5, 90)
    except:
        rospy.logerr('fail to move')
        traceback.print_exc()
        sys.exit()

    try:
        # 把持用初期姿勢に遷移
        whole_body.move_to_neutral()
        # 手を開く
        gripper.command(1)
        # 黄色のブロックに手を伸ばす（本来はブロックの座標は画像認識すべき）
        whole_body.move_end_effector_pose(0.9, 1.5, 0.2, -90, -180, 0)
        # 手を下げる
        whole_body.move_end_effector_pose(0.9, 1.5, 0.08, -90, -180, 0)
        # 手を閉じる
        gripper.command(0)
        # 把持用初期姿勢に遷移
        whole_body.move_to_neutral()
    except:
        rospy.logerr('fail to grasp')
        traceback.print_exc()
        sys.exit()

    try:
        # 移動姿勢
        whole_body.move_to_go()
        # トレーの前に移動
        omni_base.go_abs(1.8, -0.1, -90)
        # 手を前に動かす
        whole_body.move_to_neutral()
        # 手を開く
        gripper.command(1)
    except:
        rospy.logerr('fail to move')
        traceback.print_exc()
        sys.exit()
