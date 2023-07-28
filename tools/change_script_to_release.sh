#!/bin/bash -eu
#
# 【重要】受講者は実行しないこと【重要】
#
# 概要 　: リリース向けのソフトウェアに変更するためのソフトウェア
# 第1引数: 変更対象のスクリプトパス

script_path=$1

tmp_path=/tmp/change_to_release-$(basename $script_path)

cp $script_path $tmp_path

cat $tmp_path \
    | sed -E "s/^( *).*\# CHANGE1_ON_REL: (.*)$/\1\2/" \
    | awk '
        BEGIN {
            printing = "ON"
        }
        match($0 , "# *DEL_ON_REL_BEGIN") {
            printing = "OFF"
        }
        {
            if ( printing == "ON" && !match($0, "# *DEL1_ON_REL") ) {
                print $0
            }
        }
        match($0, "# *DEL_ON_REL_END") {
            printing = "ON"
        }
        ' \
    > $script_path
