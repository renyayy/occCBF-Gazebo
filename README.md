# Occlusion Sim

## Build
```bash
cd ~/Dev/occlusion_sim_ws
colcon build --packages-select occlusion_sim
source install/setup.bash
```

## Execute
```bash
# Launch経由（推奨）
ros2 launch occlusion_sim mvp_simulation.launch.py

# または個別実行
ros2 run occlusion_sim cbf_wrapper_node.py
```

## Controller選択
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


