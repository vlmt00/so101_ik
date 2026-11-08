# SO-101 IK/FK Simulation

Hugging Face LeRobot SO-101 を対象とする Python のURDFベース運動学・シミュレーション設計。

**状態:** 基本設計。実装・実機URDFでの検証は未実施。

## 目的と要件

- Robotics Toolbox for Python（RTB-P）でSO-101のURDFを読み込む。
- グリッパ可動爪の先端をTCPとし、第1〜第6軸すべてをFK・IKの対象にする。
- TCPのXYZ座標（ベース座標系、単位m）を操作し、爪先方向を鉛直下向きに維持する。
- IK結果をFKで検証し、衝突判定に合格した姿勢のみ採用する。
- Swiftで3D表示し、デバッグ段階ではXYZスライダーで目標を変更する。
- 将来は第6軸のモーター・関節構造を維持し、可動爪の先の部品のみ交換する。

## ソフトウェアの5クラス構成

| クラス | 責務 |
| --- | --- |
| `RobotModel` | URDF読込、RTB-Pモデル、関節制限、TCP、衝突形状の管理 |
| `Kinematics` | FK、下向き拘束付きIK、ヤコビアン |
| `CollisionChecker` | 自己衝突・障害物との干渉判定 |
| `SimulationController` | 計算順序・IK候補の採否判断、状態管理 |
| `SwiftViewer` | 3D描画・目標XYZの操作 |

URDFは`RobotModel`だけが解析する。`Kinematics`と`CollisionChecker`は生成済みロボットモデルを参照する。

## 処理フロー

```text
目標XYZ（SwiftViewer）
  -> IK（Kinematics）
  -> FKでTCP位置・向きの誤差と関節限界を確認
  -> 衝突判定（CollisionChecker）
  -> Controllerが採否を判断
  -> SwiftViewerに反映
```

当面は終点姿勢で判定。実際の移動では経路中の衝突も別途確認する。

## TCPと姿勢

TCPのローカル+Zを爪先方向と定義し、ベースの+Zが上なら鉛直下向きは`[0,0,-1]`。拘束はTCP位置3成分＋方向2成分の計5成分で、爪先軸周りの回転は自由。

```python
# 構想API（未実装）
q_target = kin.ik(
    xyz=[0.20, 0.10, 0.15],
    direction=[0, 0, -1],
    q0=q_current,
)
```

RTB-Pの`mask=[1,1,1,1,1,0]`は一般には「爪先の軸方向だけ固定」と数学的に同一ではないため、方向ベクトル誤差に基づく拘束付きIKを検討する。

## URDF変更予定

- 第1〜第6軸のモーターと関節機構は変更しない。
- 第6軸で動く可動爪リンクの`visual`と`collision`を交換部品に合わせて変更。
- 可動爪リンクの子として固定`tcp_joint`と`tcp_link`を設置。
- 新部品のSTL原点は第6軸の回転中心とする方針。URDFリンク原点との対応、STL軸方向、単位は要確認。
- 必要なら`inertial`を更新。`joint`の`origin`/`axis`/`limit`は機構が不変なら維持。

詳細: [docs/architecture.md](docs/architecture.md)
