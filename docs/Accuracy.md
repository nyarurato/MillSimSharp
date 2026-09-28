# 精度モデル（Accuracy）

MillSimSharp は離散ボクセル / SDF と pose サンプリングに基づく**近似**シミュレータです。
本ドキュメントは「どの誤差がどこから来るか」を分離して説明します。ここに書かれた値は
**最終面・残し量を保証するものではありません**。

## 1. ボクセル解像度（material 表現）

- ボクセルは**中心点サンプリング**で inside / outside を判定します（`signedDistance < 0` の厳密規約）。
- 表面は解像度 1 ボクセルの階段状近似です。解像度を半分にすると表面誤差も概ね半減します。
- ボクセルは部分占有率を持ちません。工具がボクセルの一部だけを削る場合、そのボクセル中心が
  工具内部にあるときだけ除去されます。

## 2. pose サンプリング（工具掃引）

- `MaxLinearStep` / `MaxAngularStep`: 直進・回転の最大サンプリング間隔です。
- `EnableAdaptiveSampling` / `MaxChordError`: 回転時に cutting-center が描く円弧の弦逸脱
  （サジッタ）を `MaxChordError` 以下に抑えるよう、追加の分割を計算します。
- シミュレーション結果は「サンプルした pose の工具形状の union」です。サンプル間で工具が通る
  領域は取り残される可能性があり、誤差はサンプル間隔と回転半径に比例します。
- `MaxLinearStep` / `MaxAngularStep` / `MaxChordError` は**姿勢経路の離散化誤差の指標**であり、
  最終面の形状や残し量を保証する値ではありません（例: 直径 6mm のボール工具で 15° 間隔の
  回転サンプリングを行うと、ボール中心の軌道はサンプル間で最大約 0.39mm ずれます）。

## 3. メッシュ再構成

- Voxel: 表面抽出（1 ボクセルの階段、頂点法線は面法線の平均）。STL 出力の facet normal は
  三角形幾何から計算されます。
- SDF: Dual Contouring。narrow band 幅は表面近傍で正確な距離を保持する範囲を決めます。
- narrow band 外の SDF 値は `±narrowBandWidth` にクランプされます。距離クエリ・勾配・メッシュは
  表面近傍でのみ正確で、遠方の距離は「空気 / 材料」の符号以上の意味を持ちません。

## 4. 衝突判定

`ToolCollisionDetector.IntersectsMaterial` の保証範囲:

- **単一 pose** の判定です。移動中（連続軌跡）に工具が通る領域は判定しません。
- 呼び出し側が渡した**1 つの工具形状のみ**を判定します（既定は切れ刃）。shank は
  `Tool.GetShankGeometry()` が既定 `null` のため含まれず、渡した場合のみ判定します。
  holder / fixture / target part は未実装です。
- ストックは**ボクセル中心**でサンプリングされます。中心に届かない接触（サブ解像度の工具が
  ボクセル角を通る場合など）は検出されません。解像度依存です。
- 「ガウジ」（目標形状への食い込み）判定には target part が必要で、別問題です。

## 5. 関連 API

| 項目 | API / 設定 |
|---|---|
| 解像度 | `VoxelGrid.Resolution` / `SDFGrid.Resolution` |
| pose サンプリング | `SimulationSettings.MaxLinearStep` / `MaxAngularStep` / `MinimumSteps` |
| 適応サンプリング | `SimulationSettings.EnableAdaptiveSampling` / `MaxChordError` |
| narrow band | `SDFGrid` コンストラクタの `narrowBandWidth` |
| 衝突判定 | `ToolCollisionDetector.IntersectsMaterial` |

検証テスト:

- `tests/Simulation/CuttingCorrectnessTest.cs`: 解像度・適応サンプリングの精度モデル
- `tests/Simulation/ToolCollisionTest.cs`: 衝突判定の center-sampling / 解像度依存
- `tests/Geometry/EffectiveBoundsTest.cs`: effective bounds とボクセル中心の対応
