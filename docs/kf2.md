# `kf2.py` Explanation

対象ファイル: [section_kalman_filter/kf2.py](../section_kalman_filter/kf2.py)

## Overview

`kf2.py` は、`kf1.py` と同じロボットシミュレーションの枠組みを使いながら、自己位置推定器を `Mcl` から `KalmanFilter` に置き換えようとしているファイルです。

ロボットがランドマークを観測し、それを使って「自分はいまどこにいるか」を推定する、という大きな目的は `kf1.py` と同じです。違いは、位置の持ち方です。

- `kf1.py`: たくさんのパーティクルで位置候補を持つ
- `kf2.py`: 平均と共分散で「信念」を持つ

ただし、現在の `kf2.py` では `motion_update` と `observation_update` がまだ `pass` のままなので、カルマンフィルタの更新式そのものは未実装です。現時点では「カルマンフィルタ用の器を作って、推定結果の不確かさを可視化する段階」にあります。

## First Mental Model

このファイルを初めて見るときは、次の3点だけ押さえると全体を追いやすくなります。

1. 世界の中でロボットが動く
2. ロボットはランドマークを観測する
3. 推定器は、その観測を使って「自分の位置の信念」を更新する

`kf2.py` では、その「信念」を1個の点ではなく、次の2つで表そうとしています。

- 平均 `mean`
  今もっとも信じている位置
- 共分散 `cov`
  その位置にどれくらい不確かさがあるか

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
  自己位置推定器です。位置の信念を平均と共分散で持ちます。
- `EstimationAgent`
  ロボットに速度指令を出しつつ、推定器の更新も呼び出します。

## Overall Flow

`trial()` の流れは `kf1.py` とほぼ同じです。

1. シミュレーション世界を作る
2. 地図を作り、ランドマークを置く
3. ロボットの初期位置を決める
4. `KalmanFilter` を作る
5. ロボットにセンサと推定器を載せる
6. 時間を少しずつ進めながら、
   ロボットを動かし、目印を観測し、推定器を更新する
7. 推定結果を描画する

中心部分は次のコードです。

```python
initial_pose = np.array([0, 0, 0]).T
kf = KalmanFilter(m, initial_pose)
circling = EstimationAgent(time_interval, 0.2, 10.0/180*math.pi, kf)
r = Robot(initial_pose, sensor=Camera(m), agent=circling, color="red")
world.append(r)
world.draw()
```

意味は次の通りです。

- `initial_pose`
  ロボットの本当の初期姿勢です。`[x, y, theta]` で表します。
- `KalmanFilter(m, initial_pose)`
  自己位置推定器を生成します。ここでは内部に `belief` を持ちます。
- `EstimationAgent(...)`
  ロボットを動かしつつ、推定器の `motion_update` と `observation_update` を呼びます。
- `Robot(...)`
  本体、センサ、推定器を組み合わせたロボットを作ります。
- `world.draw()`
  シミュレーションを開始します。

## What Happens In One Step

1回の時間更新では、概念的には次の処理が起きます。

1. ロボットが現在位置からランドマークを見る
2. `Camera` がランドマークまでの距離と方向を観測する
3. `EstimationAgent` が推定器に前回の移動量を伝える
4. `KalmanFilter.motion_update(...)` が予測ステップを行う
5. `KalmanFilter.observation_update(...)` が観測で修正する
6. ロボット本体も速度指令に従って少し移動する
7. 現在の推定結果と不確かさを画面に描画する

ただし、今のコードでは 4 と 5 はまだ未実装です。  
そのため現状の `kf2.py` では、「本来ここで予測と更新が入る」という構造だけが作られています。

## What `KalmanFilter` Stores

`KalmanFilter` クラスの中心は `belief` です。

```python
self.belief = multivariate_normal(
    mean=np.array([0.0, 0.0, math.pi/4]),
    cov=np.diag([0.1, 0.2, 0.01])
)
self.pose = self.belief.mean
```

ここで持っている情報は次の2つです。

- `mean`
  今の推定位置です。`[x, y, theta]` です。
- `cov`
  その推定の不確かさです。

共分散は、「x方向にどれくらいずれているか」「y方向にどれくらいずれているか」「向きがどれくらい不確かか」を表します。

## Difference From MCL

`kf1.py` の MCL では、位置候補をたくさん持っていました。  
`kf2.py` のカルマンフィルタでは、候補を大量に持つ代わりに、推定を1つのガウス分布で表そうとしています。

イメージとしては次の違いです。

- MCL
  「自分はここかもしれない、ここかもしれない」を多数持つ
- Kalman Filter
  「自分の位置はこのあたりで、不確かさはこのくらい」とまとめて持つ

このため、`kf2.py` ではパーティクル群ではなく、楕円で不確かさを描いています。

## What `sigma_ellipse` Does

`sigma_ellipse(p, cov, n)` は、2次元位置の不確かさを楕円で描くための関数です。

```python
def sigma_ellipse(p, cov, n):
    eig_vals, eig_vec = np.linalg.eig(cov)
    ang = math.atan2(eig_vec[:,0][1], eig_vec[:,0][0])/math.pi*180
    return Ellipse(
        p,
        width=2*n*math.sqrt(eig_vals[0]),
        height=2*n*math.sqrt(eig_vals[1]),
        angle=ang,
        fill=False,
        color="blue",
        alpha=0.5
    )
```

意味は次の通りです。

- `p`
  楕円の中心です。推定位置です。
- `cov`
  位置の不確かさです。
- 固有値 `eig_vals`
  楕円の長さに対応します。
- 固有ベクトル `eig_vec`
  楕円の向きに対応します。
- `n`
  何シグマ範囲を描くかを決めます。

このコードでは `n=3` を使っているので、3シグマ楕円を描いています。

## What `draw()` Is Showing

`KalmanFilter.draw()` は、推定結果の不確かさを可視化しています。

1. `x-y` 平面の不確かさを楕円で描く
2. 向き `theta` の不確かさを扇形のような線で描く

コードの前半では位置の不確かさを描いています。

```python
e = sigma_ellipse(self.belief.mean[0:2], self.belief.cov[0:2, 0:2], 3)
elems.append(ax.add_patch(e))
```

後半では向きの不確かさを描いています。

```python
x, y, c = self.belief.mean
sigma3 = math.sqrt(self.belief.cov[2, 2])*3
xs = [x + math.cos(c-sigma3), x, x + math.cos(c+sigma3)]
ys = [y + math.sin(c-sigma3), y, y + math.sin(c+sigma3)]
elems += ax.plot(xs, ys, color="blue", alpha=0.5)
```

これは「向きはこの角度を中心に、このくらいぶれている」という範囲を示しています。

## What Is Missing Right Now

現時点の `kf2.py` で未実装なのはここです。

```python
def motion_update(self, nu, omega, time):
    pass

def observation_update(self, observation):
    pass
```

つまり、カルマンフィルタの本体である

- 予測ステップ
- 観測による更新ステップ

がまだ入っていません。

そのため、今の `kf2.py` は「カルマンフィルタを使うための枠組みと描画まではあるが、更新式はこれから実装する段階」と説明するのが正確です。

## How The Files Are Connected

`kf2.py` は次のコードで `scripts` ディレクトリを import path に追加しています。

```python
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent / "scripts"))
from mcl import *
```

依存関係は次の通りです。

- `kf2.py` が `mcl.py` を読む
- `mcl.py` が `robot.py` を読む
- `robot.py` が `ideal_robot.py` を読む

そのため、`kf2.py` でも `World`, `Map`, `Landmark`, `Robot`, `Camera`, `EstimationAgent` を使えます。

## Related Files

- [section_kalman_filter/kf2.py](../section_kalman_filter/kf2.py)
- [section_kalman_filter/kf1.py](../section_kalman_filter/kf1.py)
- [scripts/mcl.py](../scripts/mcl.py)
- [scripts/robot.py](../scripts/robot.py)
- [scripts/ideal_robot.py](../scripts/ideal_robot.py)
