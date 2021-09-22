# wrs_algorithm

## 環境構築方法
必要なリポジトリのクローンする。
```bash
cd /workspace/py2_ws/src
git clone git@github.com:keio-smilab21/wrs_algorithm21.git
vcs import /workspace/py2_ws/src < /workspace/py2_ws/src/wrs_algorithm21/py2_ws.rosinstall
vcs import /workspace/py3_ws/src < /workspace/py2_ws/src/wrs_algorithm21/py3_ws.rosinstall
```

それぞれのワークスペースでビルドを実行する。
```
cd /workspace/py2_ws
catkin build
cd /workspace/py3_ws
catkin build
```

## 開始方法
通常の起動
```bash
rviz -d /workspace/py2_ws/src/wrs_algorithm21/config/wrs_visualize.rviz
/workspace/py2_ws/src/wrs_algorithm21/scripts/start_all.sh
```

移動系のみ
```bash
/workspace/py2_ws/src/wrs_algorithm21/scripts/start_py2.sh
```

認識系のみ
```bash
/workspace/py2_ws/src/wrs_algorithm21/scripts/start_py3.sh
```
