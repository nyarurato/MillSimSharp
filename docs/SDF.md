# SDF (Signed Distance Field)

このドキュメントは、MillSimSharp における SDF（Signed Distance Field）の実装と設計に関する詳細情報をまとめたものです。開発者向けの内部挙動、アーキテクチャ、精度特性を中心に説明します。

---

## 概要

- **SDF** はボクセル格子（VoxelGrid）から算出される距離場で、ゼロレベルセット（distance = 0）が表面を表します
- **符号規則（標準SDF）**: 負の値 = マテリアル（残っている部分 / ソリッド内部）、正の値 = 空領域（削除された部分 / 空気）
- **単位**: 保存値・公開API・narrow band はすべて **ワールド座標のミリメートル（mm）** で統一
- **サンプル位置**: 距離値はボクセル中心に存在し、補間はその半ボクセルオフセットを考慮します
- SDF から **Dual Contouring** アルゴリズムを使用して三角形メッシュを生成します
- **増分更新**: ボクセル変更時に影響範囲のみを再計算します

---

## アーキテクチャ

### 公開API: `SDFGrid`

ユーザーが直接使用するクラスです:

```csharp
// VoxelGridからSDFを生成
var sdfGrid = SDFGrid.FromVoxelGrid(
    voxelGrid,
    narrowBandWidth: 2     // Narrow band幅（ボクセル単位）
);

// メッシュ生成
var mesh = MeshConverter.ConvertToMeshFromSDF(sdfGrid);

// VoxelGridとバインドして増分更新を有効化
sdfGrid.BindToVoxelGrid(voxelGrid);

// 距離クエリ（ワールド座標、半ボクセルオフセットを考慮）
float d = sdfGrid.GetDistance(worldPos);

// 勾配（マテリアル → 空気方向の法線）
Vector3 g = sdfGrid.GetGradient(worldPos);
```

SDF だけを直接操作するワークフロー（`VoxelGrid` なし）も可能です:

```csharp
var sdfGrid = new SDFGrid(workArea, resolution: 0.5f, narrowBandWidth: 10);
sdfGrid.RemoveSphere(center, radius);
sdfGrid.RemoveCapsule(start, end, radius);
sdfGrid.RemoveFiniteCylinder(start, end, radius);
```

### 内部実装

#### 1. `SignedDistanceFieldBuilder` (internal)

- **役割**: ボクセル占有状態から正しいユークリッド距離場を計算
- **アルゴリズム**: Felzenszwalb & Huttenlocher の **Exact Euclidean Distance Transform**（O(N)、決定論的）
- **処理（2パス）**:
  1. 空ボクセルをシード（距離0）として EDT → マテリアルボクセルの距離を決定
  2. マテリアルボクセルをシードとして EDT → 空ボクセルの距離を決定
- **半ボクセル補正**: 最近傍の反対状態ボクセル中心距離から `0.5 voxel` を引くことで、ゼロレベルセットが隣接ボクセル中心の間に来るようにする
- **境界**: グリッド外は空気として扱う（外周1ボクセルのパディング）
- **Narrow band**: 計算後に `±narrowBand` へクランプ

#### 2. `DualContouring` (internal)

- **役割**: SDFから高品質なメッシュを生成
- **符号**: `cornerVal < 0` をマテリアルとして扱う（標準SDF）
- **処理**:
  1. 各セルのエッジで符号変化を検出し、エッジ交点と勾配法線を収集
  2. **マス点まわりの正則化 QEF**（二次誤差関数、λ = trace × 1e-3）を解いてセル頂点を決定し、セル境界内へクランプ（0.45×res のインセットで非多様体化を防止）
  3. 頂点を**ゼロ等値面へ射影**（ニュートン法、最大 0.5×res）して表面へ引き戻す
  4. **符号変化のあるグリッドエッジごとに**、そのエッジを共有する4セルの頂点からクワッド（2三角形）を生成（冗長クワッドを排除）
  5. セル角の符号（マテリアル / 空気）から外向きワインディングを決定し、法線は生成後の三角形から再計算
  6. 生成後に頂点をマージし、watertight なメッシュを維持（`tests/Geometry/DualContouringTest.cs` で検証）

---

## 主要機能

### 1. Narrow Band最適化

- **目的**: narrow band 外の距離計算を打ち切り、計算量を抑制
- **設定**: `narrowBandWidth` パラメータ（ボクセル単位、推奨値: 2〜10）
- **効果**: 小さい値ほど高速。メッシュ精度に必要な最小値は形状の局所曲率に依存

### 2. 増分更新

ボクセル変更時に全体を再計算せず、影響範囲のみを更新します:

```csharp
// VoxelGridとバインド
sdfGrid.BindToVoxelGrid(voxelGrid);

// ボクセル変更時、自動的にSDF更新がトリガーされる
voxelGrid.RemoveVoxelsInSphere(position, radius);
// → VoxelsChanged イベント経由で SDF 更新がスケジュールされる
// → SignedDistanceFieldBuilder.ComputeRegion が該当領域を再計算
```

**更新プロセス**:
1. 変更領域を narrow band 分拡張（この範囲外の距離は変化しない）
2. 計算ウィンドウ = 書き込み領域 + narrow band + 境界パディング1
3. 2パス EDT で該当領域のSDFを再計算
4. ウィンドウ外は変更しない

増分更新の結果は、同じボクセル状態から全再構築した結果と一致します（`tests/Geometry/SDFAccuracyTest.cs` で検証）。

### 3. マテリアル除去（CSG）

`SDFGrid` の除去APIは標準SDFの CSG difference として実装されています:

```text
dResult = max(dCurrent, -dTool)
```

| API | 形状 | 備考 |
|---|---|---|
| `RemoveSphere(center, r)` | 球 | |
| `RemoveCapsule(start, end, r)` | 線分＋球（カプセル） | 球エンドミルの掃引 |
| `RemoveFiniteCylinder(start, end, r)` | 平底の有限円柱 | フラットエンドミルの掃引。カプセルとは端面形状が異なる |

書き込みは表面バンド（`narrowBandWidth` 分）をマージンとして拡張した領域のみに行い、書き込み後は
影響範囲の距離を再計算します。工具 AABB が広い（シャンクが長い）場合でも、セル単位の並列書き込みで高速化されています。

CSG 直後は 2 つの表面が交差する領域で距離の整合性が崩れやすいため、表面バンド内のセルに対して
空気側の距離を EDT で再計算し、マテリアル側はツール形状の解析距離を優先するハイブリッド補正
（`RepairDistances`）を行います。これによりクロスカット（交差部）のメッシュ破綻を抑制しています。

---

## 精度特性

- **軸平行な平面**: ボクセル中心基準であるため**厳密**（テストで `±0.05mm` を検証）
- **曲面・斜め形状**: 離散化誤差は `O(resolution)`。テストでは解析解に対する **RMS誤差 ≤ 2 × resolution** を検証
- **ボクセル中心**: `GetDistance(worldPos)` は半ボクセルオフセットを考慮するため、`GetDistance(voxelCenter) == GetDistance(x, y, z)` が成立
- **グリッド外**: 空気として正の距離を返す（narrow band でクランプ）

---

## デバッグとトラブルシューティング

### メッシュに穴が開く場合

1. **Narrow Band幅を増やす**: `narrowBandWidth: 5` 以上に設定
2. **符号規則を確認**: 負 = マテリアル、正 = 空（標準SDF）
3. **解像度を上げる**: 薄い形状はボクセル解像度に依存する
4. **SDF値の確認**: `GetDistance()` が NaN / Infinity でないこと

### パフォーマンスが遅い場合

1. **Narrow Band幅を減らす**: `narrowBandWidth: 2` に設定
2. **解像度を下げる**: `resolution` を大きくする

### メモリ不足の場合

1. **解像度を見直す**: 密配列が確保されるため `resolution` / バウンディングボックスで調整する
2. **Narrow Band幅を最小化**: `narrowBandWidth: 2`
3. **グリッドサイズを分割**: 複数の小さいグリッドに分割して処理

---

## 参考コード箇所

### コア実装
- **公開API**: [`SDFGrid.cs`](../src/MillSimSharp/Geometry/SDFGrid.cs)
- **EDTビルダー**: [`SignedDistanceFieldBuilder.cs`](../src/MillSimSharp/Geometry/SignedDistanceFieldBuilder.cs) (internal)
- **Dual Contouring**: [`DualContouring.cs`](../src/MillSimSharp/Geometry/DualContouring.cs) (internal)

### テスト
- **SDF基本テスト**: `tests/Geometry/SDFGridTest.cs`
- **解析解・精度テスト**: `tests/Geometry/SDFAccuracyTest.cs`
- **メッシュ変換テスト**: `tests/Geometry/MeshConverterTest.cs`
- **Watertight / ワインディング**: `tests/Geometry/DualContouringTest.cs`

### サンプル
- **Viewer**: [`VoxelViewerWindow.cs`](../src/MillSimSharp.Viewer/VoxelViewerWindow.cs) (デモアプリ)
- **5軸加工**: [`samples/05-FiveAxisMachining`](../samples/05-FiveAxisMachining/Program.cs)

---

## 技術的詳細

### 標準SDF規約を採用する理由

MillSimSharp は以下の用途へ拡張するため、一般的な「負 = ソリッド内部、正 = 外部」規約を採用しています:

- ToolGeometry との CSG（`dResult = max(dStock, -dTool)`）
- holder collision / gouge detection
- remaining stock / deviation analysis

### 半ボクセル補正

距離サンプルはボクセル中心に存在します。隣接するマテリアル／空ボクセル中心の間が実際の表面であるため、
「反対状態ボクセル中心までの距離 − 0.5 voxel」が中心からの表面距離の良い近似になります。
軸平行な平面ではこの式が厳密に一致します。

### 決定論性

EDT と CSG はセル単位で独立に計算できるため並列化されていますが、各セルの結果は入力のみで決まるため、
実行結果はスレッドスケジューリングに依存しません。VoxelGrid の除去も dirty bounds を集約して直列に実行されます。

---

このドキュメントは、MillSimSharp の SDF 実装の全体像を把握するためのガイドです。詳細な実装は上記のコードファイルを参照してください。
