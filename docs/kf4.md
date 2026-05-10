# `kf4.py` Explanation

対象ファイル: [section_kalman_filter/kf4.py](../section_kalman_filter/kf4.py)

## Overview

`kf4.py` は、`kf3.py` で実装した運動モデルによる予測ステップに加えて、ランドマーク観測を使う観測更新 `observation_update` を実装したファイルです。

ここでようやく、カルマンフィルタが

- 動いたぶんだけ予測し
- センサ観測でその予測を補正する

という基本形になります。

`kf3.py` では運動するほど不確かさが広がるだけでしたが、`kf4.py` ではランドマークが見えたときに推定位置と共分散を修正できます。

## First Mental Model

このファイルでは、次の3点を押さえると全体が追いやすくなります。

1. ロボットは速度指令に従って移動する
2. カルマンフィルタは移動量から次の姿勢を予測する
3. 実際の観測と予測した観測を比べて、ずれを打ち消すように補正する

つまり `kf4.py` は、「予測だけ」の `kf3.py` から一歩進んで、「予測して、見えた情報で直す」段階です。

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
  自己位置推定器です。平均と共分散で信念を持ち、予測と観測補正を行います。
- `EstimationAgent`
  ロボットに速度指令を出しつつ、推定器の更新も呼び出します。

## Overall Flow

ファイル末尾では `kf3.py` と同じく、3台のロボットを同じ初期位置から動かしています。

1. シミュレーション世界を作る
2. 地図を作り、ランドマークを置く
3. 初期姿勢を決める
4. ロボットごとに `KalmanFilter` を作る
5. それぞれ異なる速度指令を与える
6. 時間を少しずつ進めながら、予測と観測補正を繰り返す
7. 推定位置と不確かさを描画する

中心部分は次のコードです。

```python
initial_pose = np.array([0, 0, 0]).T

kf = KalmanFilter(m, initial_pose)
circling = EstimationAgent(time_interval, 0.2, 10.0/180*math.pi, kf)
r = Robot(initial_pose, sensor=Camera(m), agent=circling, color="red")
world.append(r)

kf = KalmanFilter(m, initial_pose)
linear = EstimationAgent(time_interval, 0.1, 0.0, kf)
r = Robot(initial_pose, sensor=Camera(m), agent=linear, color="red")
world.append(r)

kf = KalmanFilter(m, initial_pose)
right = EstimationAgent(time_interval, 0.1, -3.0/180*math.pi, kf)
r = Robot(initial_pose, sensor=Camera(m), agent=right, color="red")
world.append(r)
```

3台とも別々の `KalmanFilter` を持つので、それぞれが自分の観測に基づいて独立に推定を更新します。

## What Happens In One Step

1回の時間更新では、概念的には次の処理が起きます。

1. ロボットが現在位置からランドマークを観測する
2. `EstimationAgent` が推定器に前回の移動量を渡す
3. `KalmanFilter.motion_update(...)` が予測ステップを行う
4. 平均位置を運動モデルで次時刻へ進める
5. 共分散をヤコビアンと運動ノイズで更新する
6. `KalmanFilter.observation_update(...)` が観測ごとに補正ステップを行う
7. 実際の観測 `z` と、現在の平均位置から予測される観測 `estimated_z` を比べる
8. その差をカルマンゲイン `K` で姿勢空間へ戻して、平均と共分散を更新する
9. ロボット本体も速度指令に従って少し移動する
10. 推定位置と不確かさを画面に描画する

ここで重要なのは、`kf4.py` では「動いたので不確かになる」と「見えたので不確かさを減らす」の両方が入っていることです。

## What `KalmanFilter` Stores

`KalmanFilter` の中心は `belief` です。

```python
self.belief = multivariate_normal(
    mean=init_pose,
    cov=np.diag([1e-10, 1e-10, 1e-10])
)
```

ここで持っている情報は次の2つです。

- `mean`
  現在もっとも信じている姿勢です。`[x, y, theta]` です。
- `cov`
  その姿勢にどれくらい不確かさがあるかです。

`kf3.py` と違って、初期平均は `init_pose` をそのまま使っています。  
つまり開始時点では「ロボットの初期姿勢はここだ」として推定を始めています。

また、このクラスは観測更新のために次も保持します。

- `self.map`
  ランドマーク位置を参照するための地図
- `self.distance_dev_rate`
  距離観測の標準偏差の係数
- `self.direction_dev`
  方向観測の標準偏差

## What `motion_update` Does

`motion_update` は `kf3.py` と同じです。

```python
M = matM(nu, omega, time, self.motion_noise_stds)
A = matA(nu, omega, time, self.belief.mean[2])
F = matF(nu, omega, time, self.belief.mean[2])
cov = F.dot(self.belief.cov).dot(F.T) + A.dot(M).dot(A.T)
mean = IdealRobot.state_transition(nu, omega, time, self.belief.mean)
self.belief = multivariate_normal(mean=mean, cov=cov)
```

ここでは

1. 平均 `mean` を運動モデルで進める
2. 共分散 `cov` を線形化したモデルと運動ノイズで更新する

という予測ステップを行います。

つまり、「このくらい動いたはず」という仮説を先に作る処理です。

## What `observation_update` Does

`kf4.py` の本体は `observation_update` です。

```python
H = matH(mean, self.map.landmarks[obs_id].pos)
estimated_z = IdealCamera.observation_function(mean, self.map.landmarks[obs_id].pos)
Q = matQ(estimated_z[0]*self.distance_dev_rate, self.direction_dev)
K = cov.dot(H.T).dot(np.linalg.inv(Q + H.dot(cov).dot(H.T)))
mean += K.dot(z - estimated_z)
cov = (np.eye(3) - K.dot(H)).dot(cov)
```

意味を順に整理するとこうです。

1. いまの推定姿勢 `mean` から見たとき、ランドマークがどう見えるはずかを計算する
2. 実際の観測 `z` と比べて、どれだけずれているかを見る
3. そのずれをカルマンゲイン `K` で姿勢の修正量へ変換する
4. 平均 `mean` を補正する
5. 観測したぶん、共分散 `cov` を小さくする

要するに、「予測した見え方」と「本当に見えたもの」の差を使って、自分の位置と向きを直しています。

## What `matH` Means

`matH` は観測モデルのヤコビアンです。

```python
def matH(pose, landmark_pos):
    mx, my = landmark_pos
    mux, muy, mut = pose
    q = (mux - mx)**2 + (muy - my)**2
    return np.array([
        [(mux - mx)/np.sqrt(q), (muy - my)/np.sqrt(q), 0.0],
        [(my - muy)/q, (mux - mx)/q, -1.0]
    ])
```

ここで観測は

- ランドマークまでの距離
- ランドマークの方向

の2つです。  
`H` は「姿勢 `(x, y, theta)` が少し変わったとき、観測値がどれだけ変わるか」を表します。

つまり、観測空間で見つかったずれを、姿勢空間へ戻すために必要な係数です。

## What `matQ` Means

`matQ` は観測ノイズの共分散です。

```python
def matQ(distance_dev, direction_dev):
    return np.diag(np.array([distance_dev**2, direction_dev**2]))
```

これは、

- 距離観測にはどれくらい誤差があるか
- 方向観測にはどれくらい誤差があるか

を表します。

`kf4.py` では距離誤差を

```python
estimated_z[0] * self.distance_dev_rate
```

で作っているので、遠いランドマークほど距離誤差が大きくなる前提です。

## What The Kalman Gain Is Doing

カルマンゲイン `K` は、「観測をどのくらい信じるか」を調整する係数です。

- 観測ノイズが小さい
  観測を強く信じるので、大きめに補正する
- 観測ノイズが大きい
  観測をあまり信じないので、小さめに補正する

そのため `K` は、予測と観測のバランスを取る役だと考えると分かりやすいです。

## What `draw()` Is Showing

`draw()` は `kf2.py` と `kf3.py` と同様に、推定結果の不確かさを可視化しています。

1. `x-y` 平面の不確かさを楕円で描く
2. 向き `theta` の不確かさを2本線で描く

意味は次の通りです。

- 青い楕円
  位置 `(x, y)` がどれくらいあいまいか
- 2本線
  向き `theta` がどれくらいあいまいか

`kf3.py` と違って、`kf4.py` では観測が入るたびにこの楕円や角度のばらつきが小さくなる様子を見られます。

## Difference From `kf3.py`

`kf3.py` との一番大きな違いは、観測更新が入ったことです。

- `kf3.py`
  運動更新だけを行う
- `kf4.py`
  運動更新したあと、ランドマーク観測で補正する

そのため振る舞いも変わります。

- `kf3.py`
  動くほど不確かさが広がりやすい
- `kf4.py`
  動いて不確かになっても、観測が取れれば再び絞り込める

これがカルマンフィルタらしい振る舞いです。

## One Important Detail

`observation_update` では、観測を1個ずつ順番に使って更新しています。

```python
for d in observation:
    ...
    mean += K.dot(z - estimated_z)
    cov = (np.eye(3) - K.dot(H)).dot(cov)
```

つまり、同じ時刻に複数のランドマークが見えていても、

1. 1つ目で更新
2. 更新後の平均・共分散を使って2つ目で更新
3. さらに更新後の状態で3つ目を反映

という流れです。

## How The Files Are Connected

`kf4.py` は次のコードで `scripts` ディレクトリを import path に追加しています。

```python
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent / "scripts"))
from mcl import *
```

依存関係は次の通りです。

- `kf4.py` が `mcl.py` を読む
- `mcl.py` が `robot.py` を読む
- `robot.py` が `ideal_robot.py` を読む

そのため、`kf4.py` でも `World`, `Map`, `Landmark`, `Robot`, `Camera`, `EstimationAgent`, `IdealRobot` などを使えます。

## Related Files

- [section_kalman_filter/kf4.py](../section_kalman_filter/kf4.py)
- [section_kalman_filter/kf3.py](../section_kalman_filter/kf3.py)
- [section_kalman_filter/kf2.py](../section_kalman_filter/kf2.py)
- [section_kalman_filter/kf1.py](../section_kalman_filter/kf1.py)
- [scripts/mcl.py](../scripts/mcl.py)
- [scripts/robot.py](../scripts/robot.py)
- [scripts/ideal_robot.py](../scripts/ideal_robot.py)
