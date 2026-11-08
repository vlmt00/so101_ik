# SO-101 IK/FK simulator

Hugging Face LeRobot SO-101用の **URDFベース IK/FK・衝突判定・Swift可視化の実装プロトタイプ**です。

**実装済み:** URDFからRTB-Pモデルを生成、moving jaw末端へのTCP固定リンクの追加、下向き方向拘束付き数値IK（SciPy）、FKでの位置・方向検証、関節角制限、衝突判定の呼び出し、コントローラ、Swift表示とXYZスライダー。

**未検証:** この環境ではRobotics Toolbox本体と実際のSO-101 URDFアセットが利用できないため、実URDFの統合試験およびSwift描画試験は未実施。現段階では実機へ接続しないでください。

## セットアップ

Python 3.10以上を使用してください。

```bash
python -m venv .venv
source .venv/bin/activate
pip install -e '.[simulation,test]'
```

SO-101 followerのURDFと同じディレクトリ構造のSTLメッシュ一式を用意してください（例: Hugging Face `lerobot/robot-urdfs` の `so101/so101_new_calib.urdf` と `so101/assets/`）。XML内の `package://` のメッシュパスは事前に相対パスへ変換してください。

## TCPの設定

`moving_jaw_so101_v1_link` の子に固定 `tcp_link` / `tcp_joint` を追加します。**実際の爪先までのTCPオフセットは未計測**です。 `--tcp-xyz` はmoving jawの**リンク座標系から測った爪先までの位置（m）**を指定してください。STL原点が第6軸中心でも、URDFのリンク原点が同じとは限りません。

`--tcp-rpy` はTCPリンク座標系の方向、`--tip-axis` はTCPリンクでの爪先方向（既定値 `0 0 1`）です。下向き目標方向はベース座標系で `0 0 -1` に固定しています。ベース+Zが上向きであることを確認してください。

## 実行

```bash
# FKと拘束付きIKを実行（TCPオフセットは実測値へ置換すること）
so101-ik --urdf /path/to/so101_new_calib.urdf \
  --tcp-xyz 0 -0.03 0.05 \
  --target 0.20 0.10 0.15 \
  --allow-unchecked-collision

# Swift 3D描画＋別ウィンドウのMatplotlib XYZスライダー
so101-ik --urdf /path/to/so101_new_calib.urdf \
  --tcp-xyz 0 -0.03 0.05 \
  --viewer --allow-unchecked-collision
```

上記 `--tcp-xyz` は**仮の説明用数値**でありSO-101の実寸を保証しません。

**安全上の制限:** 衝突判定が使えない場合はデフォルトで姿勢更新を拒否します。`--allow-unchecked-collision` はデバッグのための明示的な上書きであり、実機制御には使わないでください。また現状は最終姿勢のみ検査し、移動途中の経路衝突は未検査です。衝突判定が実装されていてもURDFの衝突形状が欠落・不完全な場合、判定の妥当性は保証されません。

## コンポーネント

| クラス | 役割 |
| --- | --- |
| `RobotModel` | URDFロード・TCP固定ジョイント追加・関節制限取得 |
| `Kinematics` | 6関節FK・下向き拘束付きIK |
| `CollisionChecker` | RTB-Pの自己衝突・障害物衝突API |
| `SimulationController` | IK/FK検証・衝突判定・採否判断 |
| `SwiftViewer` | Swift 3D表示・XYZスライダー |

詳細: [docs/architecture.md](docs/architecture.md)

## テスト

```bash
pip install -e '.[test]'
pytest -q
```

`tests/test_core.py` はモックモデルを使うユニットテストです。実SO-101のURDF、ライブラリ実API、Swift、メッシュ干渉の結合テストは別途必要です。
