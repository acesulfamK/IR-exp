#!/bin/bash -eu

script_dir=$(cd `dirname $0`; pwd)
$script_dir/start_py3.sh &
$script_dir/start_py2.sh &

wait
