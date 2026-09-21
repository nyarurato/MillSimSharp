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
    narrowBandWidth: 2,    // Narrow band幅（ボクセル単位）
    useSparse: false,      // ※未実装（予約）。密配列が常に確保される
    fastMode: false        // ※無視される（互換用パラメータ）
);

// メッシュ生成
var mesh = MeshConverter.ConvertToMeshFromSDF(sdfGrid);

// VoxelGridとバインドして増分更新を有効化
sdfGrid.BindToVoxelGrid(voxelGrid);
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
  1. 各セルのエッジで符号変化を検出
  2. エッジ交点の平均（質量点）をセル頂点とする
  3. 隣接セル間でクワッド（2つの三角形）を生成
  4. 法線に基づいてワインディングを調整（法線はマテリアルから空気へ向く）

---

## 主要機能

### 1. Narrow Band最適化

- **目的**: narrow band 外の距離計算を打ち切り、計算量を抑制
- **設定**: `narrowBandWidth` パラメータ（ボクセル単位、推奨値: 2〜10）
- **効果**: 小さい値ほど高速。メッシュ精度に必要な最小値は形状の局所曲率に依存

### 2. スパースストレージ（未実装・予約）

- **現状**: `useSparse: true` を指定しても密配列（`float[,,]`）が確保され、メモリ削減効果はない
- **予定**: `ConcurrentDictionary` ベースの実装は PR6 で再検討
- **注意**: 現時点で `useSparse` による挙動・メモリ使用量の差はない

### 3. 増分更新

ボクセル変更時に全体を再計算せず、影響範囲のみを更新します:

```csharp
// VoxelGridとバインド
sdfGrid.BindToVoxelGrid(voxelGrid);

// ボクセル変更時、自動的にSDF更新がトリガーされる
voxelGrid.RemoveVoxelsInSphere(position, radius);
// → SDFGrid.OnVoxelGridChanged が呼ばれる
// → SignedDistanceFieldBuilder.ComputeRegion が該当領域を再計算
```

**更新プロセス**:
1. 変更領域を narrow band 分拡張（この範囲外の距離は変化しない）
2. 計算ウィンドウ = 書き込み領域 + narrow band + 境界パディング1
3. 2パス EDT で該当領域のSDFを再計算
4. ウィンドウ外は変更しない

増分更新の結果は、同じボクセル状態から全再構築した結果と一致します（`tests/Geometry/SDFAccuracyTest.cs` で検証）。

### 4. マテリアル除去（CSG）

`SDFGrid` の除去APIは標準SDFの CSG difference として実装されています:

```text
dResult = max(dCurrent, -dTool)
```

| API | 形状 | 備考 |
|---|---|---|
| `RemoveSphere(center, r)` | 球 | |
| `RemoveCapsule(start, end, r)` | 線分＋球（カプセル） | 球エンドミルの掃引 |
| `RemoveFiniteCylinder(start, end, r)` | 平底の有限円柱 | フラットエンドミルの掃引。カプセルとは端面形状が異なる |

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
3. **スパースストレージ（未実装）**: 現在は指定してもメモリ・速度は変わらない（PR6 で再検討）

### メモリ不足の場合

1. **現状の注意**: スパースストレージは未実装。密配列が確保されるため `resolution` / バウンディングボックスで調整する
2. **Narrow Band幅を最小化**: `narrowBandWidth: 2`
3. **グリッドサイズを分割**: 複数の小さいグリッドに分割して処理

---

## 参考コード箇所

### コア実装
- **公開API**: [`SDFGrid.cs`](file:///d:/workspace/projects/MillSimSharp/src/MillSimSharp/Geometry/SDFGrid.cs)
- **EDTビルダー**: [`SignedDistanceFieldBuilder.cs`](file:///d:/workspace/projects/MillSimSharp/src/MillSimSharp/Geometry/SignedDistanceFieldBuilder.cs) (internal)
- **Dual Contouring**: [`DualContouring.cs`](file:///d:/workspace/projects/MillSimSharp/src/MillSimSharp/Geometry/DualContouring.cs) (internal)

### テスト
- **SDF基本テスト**: `tests/Geometry/SDFGridTest.cs`
- **解析解・精度テスト**: `tests/Geometry/SDFAccuracyTest.cs`
- **メッシュ変換テスト**: `tests/Geometry/MeshConverterTest.cs`

### サンプル
- **Viewer**: [`VoxelViewerWindow.cs`](file:///d:/workspace/projects/MillSimSharp/src/MillSimSharp.Viewer/VoxelViewerWindow.cs) (デモアプリ)

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

SDF計算は EDT によりシングルスレッドで決定的に実行されます。
VoxelGrid の除去も、SVO のスレッド安全性のため直列実行に統一されています（結果はスケジューリングに依存しません）。

---

このドキュメントは、MillSimSharp の SDF 実装の全体像を把握するためのガイドです。詳細な実装は上記のコードファイルを参照してください。
