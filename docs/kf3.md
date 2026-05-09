# `kf3.py` Explanation

対象ファイル: [section_kalman_filter/kf3.py](../section_kalman_filter/kf3.py)

## Overview

`kf3.py` は、`kf2.py` で用意したカルマンフィルタの枠組みに対して、運動モデルによる予測ステップ `motion_update` を実装したファイルです。

目的は引き続き、ロボットが地図中を動くときに「自分はいまどこにいるか」を推定することです。ただし、この段階ではまだ観測更新 `observation_update` は `pass` のままです。

そのため `kf3.py` は、

- カルマンフィルタの信念をガウス分布で持つ
- ロボットを動かしたときに平均と共分散を更新する
- 動き方によって不確かさの広がり方がどう変わるかを見る

ためのファイルと捉えるのが正確です。

## First Mental Model

このファイルでは、次の3点を押さえると全体が追いやすくなります。

1. ロボットは速度指令に従って移動する
2. カルマンフィルタは、その移動量から「次にどこにいるはずか」を予測する
3. 予測にはノイズがあるので、不確かさも一緒に広がる

ここで大事なのは、`kf3.py` では観測で不確かさを縮めるのではなく、運動によって不確かさがどう増えるかを見ているという点です。

## Main Characters

- `World`
  シミュレーション全体を管理し、時間を進めながら描画します。
- `Map`
  ランドマークを持つ地図です。
- `Landmark`
  ロボットが観測する目印です。
- `Robot`
  実際に動くロボット本体です。
- `Camera`
  ランドマークまでの距離と方向を観測するセンサです。
- `KalmanFilter`
  自己位置推定器です。平均と共分散で信念を持ち、`motion_update` で予測します。
- `EstimationAgent`
  ロボットに速度指令を出しつつ、推定器の更新も呼び出します。

## Overall Flow

ファイル末尾では、3台のロボットを同じ初期位置から動かしています。

1. シミュレーション世界を作る
2. 地図を作り、ランドマークを置く
3. 初期姿勢を決める
4. ロボットごとに `KalmanFilter` を作る
5. それぞれ異なる速度指令を与える
6. 時間を少しずつ進めながら、運動更新を繰り返す
7. 推定位置と不確かさを描画する

中心部分は次のコードです。

```python
initial_pose = np.array([0, 0, 0]).T

kf = KalmanFilter(m, initial_pose)
circling = EstimationAgent(time_interval, 0.2, 10.0/180*math.pi, kf)
world.append(Robot(initial_pose, sensor=Camera(m), agent=circling, color="red"))

kf = KalmanFilter(m, initial_pose)
linear = EstimationAgent(time_interval, 0.1, 0.0, kf)
world.append(Robot(initial_pose, sensor=Camera(m), agent=linear, color="red"))

kf = KalmanFilter(m, initial_pose)
right = EstimationAgent(time_interval, 0.1, -3.0/180*math.pi, kf)
world.append(Robot(initial_pose, sensor=Camera(m), agent=right, color="red"))
```

意味は次の通りです。

- 1台目
  前進しながら左に曲がるロボットです。
- 2台目
  直進するロボットです。
- 3台目
  前進しながら右に曲がるロボットです。

これにより、動き方の違いで共分散の広がり方がどう変わるかを見比べられます。

## What Happens In One Step

1回の時間更新では、概念的には次の処理が起きます。

1. ロボットが現在位置からランドマークを観測する
2. `EstimationAgent` が推定器に前回の移動量を渡す
3. `KalmanFilter.motion_update(...)` が予測ステップを行う
4. 平均位置を運動モデルに従って次時刻へ進める
5. 共分散をヤコビアンと運動ノイズで更新する
6. `observation_update(...)` はまだ未実装なので何もしない
7. ロボット本体も速度指令に従って少し移動する
8. 推定位置と不確かさを画面に描画する

ここで重要なのは、`kf3.py` では「予測だけが入っている」ことです。  
観測による補正がないため、不確かさは基本的に減らず、動くほど広がっていきます。

## What `KalmanFilter` Stores

`KalmanFilter` の中心は `belief` です。

```python
self.belief = multivariate_normal(
    mean=np.array([0.0, 0.0, 0.0]),
    cov=np.diag([1e-10, 1e-10, 1e-10])
)
```

ここで持っている情報は次の2つです。

- `mean`
  現在もっとも信じている姿勢です。`[x, y, theta]` です。
- `cov`
  その姿勢にどれくらい不確かさがあるかです。

初期共分散が非常に小さいので、開始時点では「最初の位置はほぼ分かっている」という前提になっています。  
そこから運動ノイズによって、少しずつ不確かさが増えていきます。

## What `motion_update` Does

`kf3.py` の本体は `motion_update` です。

```python
M = matM(nu, omega, time, self.motion_noise_stds)
A = matA(nu, omega, time, self.belief.mean[2])
F = matF(nu, omega, time, self.belief.mean[2])
cov = F.dot(self.belief.cov).dot(F.T) + A.dot(M).dot(A.T)
mean = IdealRobot.state_transition(nu, omega, time, self.belief.mean)
self.belief = multivariate_normal(mean=mean, cov=cov)
```

大きく分けると、次の2つをしています。

1. 平均 `mean` を運動モデルで進める
2. 共分散 `cov` を線形化したモデルと運動ノイズで更新する

つまり、ロボットが「これだけ動いたはず」という予測を平均に反映し、その予測がどれくらいあいまいかを共分散に反映しています。

## What `matM`, `matA`, `matF` Mean

この3つの関数は、運動ノイズを共分散へ伝えるために使われます。

### `matM`

```python
def matM(nu, omega, time, stds):
    return np.diag([
        stds["nn"]**2*abs(nu)/time + stds["no"]**2*abs(omega)/time,
        stds["on"]**2*abs(nu)/time + stds["oo"]**2*abs(omega)/time
    ])
```

これは制御入力側のノイズ共分散です。  
前進速度 `nu` と角速度 `omega` にどれくらい誤差が乗るかを表します。

### `matA`

`A` は、制御入力のノイズが姿勢 `(x, y, theta)` にどう影響するかを表すヤコビアンです。  
「速度のずれがあったら、位置と向きがどれだけ変わるか」を姿勢空間へ写します。

### `matF`

`F` は、ひとつ前の姿勢の不確かさが次の姿勢へどう伝わるかを表すヤコビアンです。  
つまり「いまの姿勢誤差が、移動後にどう残るか」を表しています。

この2つを使うことで、

- もともと持っていた不確かさ
- 今回の移動で新しく加わる不確かさ

を合成できます。

## Why `omega` Is Clamped

`motion_update` には次の処理があります。

```python
if abs(omega) < 1e-5:
    omega = 1e-5
```

これは、`matA` や `matF` の式に `omega` で割る項があるためです。  
直進時は本来 `omega = 0` ですが、そのままだとゼロ除算になります。

そのため、計算上だけごく小さい値を入れて近似しています。

## What `draw()` Is Showing

`draw()` は `kf2.py` と同様に、推定結果の不確かさを可視化しています。

1. `x-y` 平面の不確かさを楕円で描く
2. 向き `theta` の不確かさを2本線で描く

楕円は位置の共分散 `cov[0:2, 0:2]` を3シグマ範囲で描いたものです。  
向きの線は `cov[2,2]` から角度方向のばらつきを取り出して示しています。

この表示によって、

- 直進ではどちら方向に不確かさが伸びやすいか
- 回転しながら進むと楕円の向きがどう変わるか
- 向きの不確かさが位置の不確かさへどう結びつくか

を視覚的に確認できます。

## Difference From `kf2.py`

`kf2.py` との一番大きな違いは、予測ステップが入ったことです。

- `kf2.py`
  `motion_update` も `observation_update` も未実装
- `kf3.py`
  `motion_update` は実装済み、`observation_update` は未実装

したがって `kf3.py` は、「カルマンフィルタの運動モデルだけを先に入れて、不確かさ伝播を見る段階」と言えます。

## What Is Still Missing

現時点で未実装なのはここです。

```python
def observation_update(self, observation):
    pass
```

つまり、ランドマーク観測を使って推定を修正する処理はまだありません。

そのため現状では、

- 動いたぶんだけ予測する
- 予測誤差は増えていく
- 観測で補正して絞り込むことはしない

という振る舞いになります。

## How The Files Are Connected

`kf3.py` は次のコードで `scripts` ディレクトリを import path に追加しています。

```python
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent / "scripts"))
from mcl import *
```

依存関係は次の通りです。

- `kf3.py` が `mcl.py` を読む
- `mcl.py` が `robot.py` を読む
- `robot.py` が `ideal_robot.py` を読む

そのため、`kf3.py` でも `World`, `Map`, `Landmark`, `Robot`, `Camera`, `EstimationAgent`, `IdealRobot` などを使えます。

## Related Files

- [section_kalman_filter/kf3.py](../section_kalman_filter/kf3.py)
- [section_kalman_filter/kf2.py](../section_kalman_filter/kf2.py)
- [section_kalman_filter/kf1.py](../section_kalman_filter/kf1.py)
- [scripts/mcl.py](../scripts/mcl.py)
- [scripts/robot.py](../scripts/robot.py)
- [scripts/ideal_robot.py](../scripts/ideal_robot.py)
