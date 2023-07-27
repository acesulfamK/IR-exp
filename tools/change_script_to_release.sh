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
    | sed "/\# DEL1_ON_REL/d" \
    | sed -E "s/^( *).*\# CHANGE1_ON_REL: (.*)$/\1\2/" \
    | sed -E "s/^.*(\# DEL_ON_REL_BEGIN).*$/\1/" \
    | sed -E "s/^.*(\# DEL_ON_REL_END).*$/\1/" \
    | sed -z "s/\# DEL_ON_REL_BEGIN.*\# DEL_ON_REL_END\n//" \
    > $script_path
