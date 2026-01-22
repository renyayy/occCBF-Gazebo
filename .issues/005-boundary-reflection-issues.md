---
title: "境界反射における振動・スタック問題の総合解析"
category: bug
severity: high
status: open
created: 2026-01-22
---

## 概要
multi_obstacle_controller.py の境界反射ロジックにより、境界付近で障害物が振動（スタック）する複合的な問題が発生しています。

## 該当箇所
- ファイル: `/src/occlusion_sim/scripts/multi_obstacle_controller.py`
- 行番号: 36 (timer), 53-67 (boundary reflection logic)

## 問題の詳細

### 1. 境界反射振動問題
**現在の実装:**
```python
# Y boundary reflection
if y >= self.Y_MAX:
    vy = -abs(vy)
    obs['theta'] = -obs['theta']
elif y <= self.Y_MIN:
    vy = abs(vy)
    obs['theta'] = -obs['theta']
```

**問題発生メカニズム:**
1. **めり込み**: 制御周期（0.05秒）で境界を超える（例：y = Y_MAX + 0.01）
2. **反射判定**: `y >= Y_MAX` で速度反転 → 内向き速度
3. **位置未補正**: 次のループでも `y > Y_MAX` のまま
4. **連続反射**: 再度反射判定が発生 → 速度が外向きに戻る
5. **振動**: 境界付近で震え発生

### 2. 制御周期による境界めり込み問題
**めり込み発生条件:**
- 障害物速度: v = 0.3 m/s
- 制御周期: dt = 0.05s  
- 1ステップ移動距離: 0.3 × 0.05 = **0.015m**
- 境界近くでは必然的にオーバーシュート

**例：境界オーバーシュート**
```
t=0.00: y = 9.99  (境界Y_MAX=10.0に接近)
t=0.05: y = 10.005 (境界を0.005m超過)
       ↓ 反射判定発生
t=0.10: 速度反転したが位置は未補正のまま
```

### 3. 反射状態管理の欠如
**現在の問題:**
```python
# 毎ステップ独立した判定
if y >= self.Y_MAX:
    vy = -abs(vy)           # Step N: 内向き
    obs['theta'] = -obs['theta']

# 次のステップでまだ y >= Y_MAX なら
if y >= self.Y_MAX:         # Step N+1: 再度判定
    vy = -abs(vy)           # 内向きのまま（問題なし）
    obs['theta'] = -obs['theta']  # しかし角度は再反転（問題）
```

**連続反射パターン:**
```
Step 1: y=10.01 → 反射 → theta=45° から theta=-45°
Step 2: y=10.01 → 再反射 → theta=-45° から theta=45° (元に戻る)
Step 3: y=10.01 → 再々反射 → 無限ループ
```

## 影響
- 障害物が境界付近で不自然に振動
- シミュレーション品質の低下
- 他のロボットとの相互作用への悪影響
- 予測不可能な障害物挙動

## 改善提案

### 推奨解決策: safe_control/dynamic_env方式の採用
```python
def control_loop(self):
    for name, obs in self.obstacles.items():
        if obs['state'] is None:
            continue
        
        x = obs['state'].pose.pose.position.x
        y = obs['state'].pose.pose.position.y
        theta = obs['theta']
        v_max = obs['v_max']
        
        # Random walk
        if obs['mode'] == 1 and random.random() < 0.05:
            theta += random.gauss(0.0, 0.2)
            obs['theta'] = theta
        
        # 現在の速度計算
        vx = v_max * math.cos(theta)
        vy = v_max * math.sin(theta)
        
        # 次ステップ位置予測
        x_new = x + vx * 0.05
        y_new = y + vy * 0.05
        
        # Y境界反射（位置クランプ + 速度反転）
        if y_new >= self.Y_MAX:
            y_new = self.Y_MAX
            vy = -abs(vy)
            obs['theta'] = -obs['theta']
            vx = v_max * math.cos(obs['theta'])
            vy = v_max * math.sin(obs['theta'])
        elif y_new <= self.Y_MIN:
            y_new = self.Y_MIN
            vy = abs(vy)
            obs['theta'] = -obs['theta']
            vx = v_max * math.cos(obs['theta'])
            vy = v_max * math.sin(obs['theta'])
            
        # X境界反射
        if x_new >= self.X_MAX:
            x_new = self.X_MAX
            vx = -abs(vx)
            obs['theta'] = math.pi - obs['theta']
            vx = v_max * math.cos(obs['theta'])
            vy = v_max * math.sin(obs['theta'])
        elif x_new <= self.X_MIN:
            x_new = self.X_MIN
            vx = abs(vx)
            obs['theta'] = math.pi - obs['theta']
            vx = v_max * math.cos(obs['theta'])
            vy = v_max * math.sin(obs['theta'])
        
        # 角度正規化
        obs['theta'] = math.atan2(math.sin(obs['theta']), math.cos(obs['theta']))
        
        # 速度指令送信
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        self.cmd_pubs[name].publish(cmd)
```

### 核心改善点
1. **予測制御**: 次ステップの位置を事前計算
2. **位置クランプ**: 境界を超えた位置を境界値に修正
3. **一回限り反射**: 位置更新前の予測で反射判定
