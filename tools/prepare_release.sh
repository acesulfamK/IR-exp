#!/bin/bash -eu
#
# 【重要】受講者は実行しないこと【重要】
#
# 概要: リリース向けの作業を実行するスクリプト
# 引数: なし

root_path=$(readlink -f $(dirname $0)/..)

find $root_path/scripts/wrs_algorithm -name "*.py" | xargs -L1 $root_path/tools/change_script_to_release.sh
