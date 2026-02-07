# Gazebo simulator (Occlusion-CBF)
## Enter container
```bash
xhost +local:docker

docker run -itd \
--net=host \
-e DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
-v /home/renyayy/Dev/OccCBF/Gazebo_ws:/root/Gazebo_ws \
--name Gazebo_ws \
gazebo-ros2_nav2
```

## Build
```bash
cd ~/root/Gazebo_ws
colcon build --packages-select occlusion_sim --symlink-install
source install/setup.bash
```

## Execute
```bash
# Launch
ros2 launch occlusion_sim multi_obstacle_simulation.launch.py

# 個別実行
ros2 run occlusion_sim cbf_wrapper_node.py
```

## Select Controller 
`cbf_wrapper_node.py` 内でコメントアウトを切り替え:

```python
# Controller選択 (使用する Controller をコメント解除) 
self.controller = CBFQP(...)              # 基本CBF-QP
# self.controller = BackupCBFQP(...)      # バックアップCBF-QP (遮蔽対応)
# self.controller = MPCCBF(...)           # MPC-CBF
# self.controller = OptimalDecayCBFQP(...)# 最適減衰CBF-QP
# self.controller = OptimalDecayMPCCBF(...)# 最適減衰MPC-CBF
```

| コントローラー | 特徴 |
|--------------|------|
| CBFQP | 基本CBF-QP |
| BackupCBFQP | 遮蔽対応、動的障害物向け |
| MPCCBF | MPC+CBF |
| OptimalDecayCBFQP | 安全マージン動的調整 |
| OptimalDecayMPCCBF | MPC版最適減衰 |


