# `kf1.py` Explanation

対象ファイル: [section_kalman_filter/kf1.py](../section_kalman_filter/kf1.py)

## Overview

`kf1.py` は、ランドマーク付きの2次元環境でロボットを動かしながら、`Mcl` を使って自己位置推定を行うシミュレーションです。

## What The Code Does

`trial()` では次の処理を順に行います。

1. `World` を作る
2. `Map` に3つの `Landmark` を置く
3. ロボットの初期姿勢 `initial_pose = [x, y, theta]` を決める
4. `Mcl` を生成して、自己位置推定器として使う
5. `EstimationAgent` を作り、ロボットの制御と推定更新を担当させる
6. `Robot` に `Camera` と `EstimationAgent` を載せる
7. `world.draw()` でシミュレーションを描画する

## Import Structure

`kf1.py` は次のコードで `scripts` ディレクトリを import path に追加しています。

```python
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent / "scripts"))
from mcl import *
```

この `mcl.py` はさらに `robot.py` を読み、`robot.py` は `ideal_robot.py` を読みます。結果として `kf1.py` では以下の主要クラスが使えるようになります。

- `World`
- `Map`
- `Landmark`
- `Robot`
- `Camera`
- `Mcl`
- `EstimationAgent`

## Main Objects

### `World`

シミュレーション全体を管理します。時間を進めながら、地図やロボットを描画します。

### `Map` / `Landmark`

ロボットが観測するランドマークを保持します。MCL は、観測したランドマークの位置と地図上の正解位置を比較して自己位置を絞り込みます。

### `Robot`

移動する本体です。`Camera` でランドマークを観測し、`EstimationAgent` から速度指令を受けて動きます。

### `Camera`

ロボットから見たランドマークの距離と方向を返します。この観測値が MCL の更新に使われます。

### `Mcl`

Monte Carlo Localization の本体です。複数のパーティクルを持ち、各時刻で以下を行います。

- 運動に応じてパーティクルを動かす
- 観測と地図を比較して重みを更新する
- 重みに基づいてリサンプリングする

### `EstimationAgent`

ロボットに速度 `nu`, 角速度 `omega` を与えると同時に、`Mcl` の `motion_update` と `observation_update` を呼び出します。

## Flow In `trial()`

`trial()` の中心部分は次の通りです。

```python
initial_pose = np.array([0, 0, 0]).T
estimator = Mcl(m, initial_pose, 100)
a = EstimationAgent(time_interval, 0.2, 10.0/180*math.pi, estimator)
r = Robot(initial_pose, sensor=Camera(m), agent=a, color="red")
world.append(r)
world.draw()
```

ここでの意味は次の通りです。

- `initial_pose`: ロボットの初期位置と向き
- `Mcl(..., 100)`: 100個のパーティクルで自己位置推定を行う
- `EstimationAgent(...)`: ロボットを前進・回転させつつ推定器を更新する
- `Robot(...)`: センサと推定器付きのロボットを生成する
- `world.draw()`: アニメーションを開始する

## Related Files

- [section_kalman_filter/kf1.py](../section_kalman_filter/kf1.py)
- [scripts/mcl.py](../scripts/mcl.py)
- [scripts/robot.py](../scripts/robot.py)
- [scripts/ideal_robot.py](../scripts/ideal_robot.py)
