# SDF (Signed Distance Field)

このドキュメントは、MillSimSharp における SDF（Signed Distance Field）の実装と設計に関する詳細情報をまとめたものです。開発者向けの内部挙動、アーキテクチャ、最適化手法を中心に説明します。

---

## 概要

- **SDF** はボクセル格子（VoxelGrid）から算出される距離場で、ゼロレベルセット（distance=0）が表面を表します
- **符号規則**: 負の値 = 空領域内部（削除された部分）、正の値 = マテリアル内部（残っている部分）
- SDF から **Dual Contouring** アルゴリズムを使用して高品質な三角形メッシュを生成します
- **増分更新**: ボクセル変更時に影響範囲のみを再計算する最適化機構を実装

---

## アーキテクチャ

### 公開API: `SDFGrid`

ユーザーが直接使用するクラスです:

```csharp
// VoxelGridからSDFを生成
var sdfGrid = SDFGrid.FromVoxelGrid(
    voxelGrid, 
    narrowBandWidth: 2,    // Narrow band幅（ボクセル単位）
    useSparse: true,       // スパースストレージを使用（大規模グリッド向け）
    fastMode: false        // 高速モード（精度とのトレードオフ）
);

// メッシュ生成
var mesh = sdfGrid.GenerateMesh();

// VoxelGridとバインドして増分更新を有効化
sdfGrid.BindToVoxelGrid(voxelGrid);
```

### 内部実装

#### 1. `OctreeSDF` (internal)

- **役割**: SDF値を階層的に格納し、高速なクエリを実現
- **最適化**: 
  - Fast Sweeping Algorithm で事前計算された密なSDF配列を使用
  - Octree構造で空間を分割し、均一な領域を圧縮
  - サンプルキャッシュによる重複計算の削減

#### 2. `FastSweepingSDF` (internal)

- **役割**: O(N) の計算量で高速にSDFを計算
- **アルゴリズム**:
  1. 表面ボクセルを検出し、初期距離を設定
  2. 8方向（±X, ±Y, ±Z の組み合わせ）にスイープを実行
  3. 各ボクセルで隣接ボクセルからの距離を伝播
  4. 2イテレーションで収束
- **並列化**: Z軸方向は順次処理、XY平面内は並列処理

#### 3. `DualContouring` (internal)

- **役割**: SDFから高品質なメッシュを生成
- **利点**: Marching Cubesより鋭いエッジを保持
- **処理**:
  1. 各セルのエッジで符号変化を検出
  2. QEF（Quadratic Error Function）を使用して最適な頂点位置を計算
  3. 隣接セル間でクワッド（2つの三角形）を生成
  4. 法線方向に基づいてワインディングオーダーを調整

---

## 主要機能

### 1. Narrow Band最適化

- **目的**: 表面付近のみ正確な距離を計算し、メモリと計算時間を削減
- **設定**: `narrowBandWidth` パラメータで制御（推奨値: 2-10ボクセル）
- **効果**: 
  - 小さい値（2）: 高速だが粗いメッシュ
  - 大きい値（10）: 高品質だが低速

### 2. スパースストレージ

- **対象**: 100万ボクセル以上の大規模グリッド
- **実装**: `ConcurrentDictionary` を使用して非ゼロ値のみ保存
- **効果**: メモリ使用量を大幅に削減

### 3. 増分更新

ボクセル変更時に全体を再計算せず、影響範囲のみを更新:

```csharp
// VoxelGridとバインド
sdfGrid.BindToVoxelGrid(voxelGrid);

// ボクセル変更時、自動的にSDF更新がトリガーされる
voxelGrid.RemoveVoxelsInSphere(position, radius);
// → SDFGrid.OnVoxelGridChanged が呼ばれる
// → OctreeSDF.UpdateRegionWithFastSweeping が実行される
```

**更新プロセス**:
1. 変更領域を narrow band 分拡張
2. Fast Sweeping で拡張領域のSDFを再計算
3. Octree の該当ノードを再構築

### 4. Fast Mode

- **用途**: テストやプレビュー時の高速化
- **トレードオフ**: 精度が低下するが、計算時間が大幅に短縮
- **実装**: 探索半径を制限し、軸方向のみスキャン
- **環境変数**: `MILLSIM_FAST_TESTS=1` で強制的に有効化可能

---

## デバッグとトラブルシューティング

### メッシュに穴が開く場合

1. **Narrow Band幅を増やす**: `narrowBandWidth: 5` 以上に設定
2. **Fast Modeを無効化**: `fastMode: false` で精度を優先
3. **境界処理を確認**: グリッド境界付近で問題が発生しやすい
4. **符号規則を確認**: 負=空、正=マテリアルの規則が正しいか

### パフォーマンスが遅い場合

1. **Narrow Band幅を減らす**: `narrowBandWidth: 2` に設定
2. **スパースストレージを有効化**: `useSparse: true`
3. **Fast Modeを使用**: プレビュー時は `fastMode: true`
4. **解像度を下げる**: VoxelGrid の `resolution` を大きくする

### メモリ不足の場合

1. **スパースストレージを有効化**: 必須
2. **Narrow Band幅を最小化**: `narrowBandWidth: 2`
3. **グリッドサイズを分割**: 複数の小さいグリッドに分割して処理

---

## 参考コード箇所

### コア実装
- **公開API**: [`SDFGrid.cs`](file:///d:/workspace/projects/MillSimSharp/src/MillSimSharp/Geometry/SDFGrid.cs)
- **Octree**: [`OctreeSDF.cs`](file:///d:/workspace/projects/MillSimSharp/src/MillSimSharp/Geometry/OctreeSDF.cs) (internal)
- **Fast Sweeping**: [`FastSweepingSDF.cs`](file:///d:/workspace/projects/MillSimSharp/src/MillSimSharp/Geometry/FastSweepingSDF.cs) (internal)
- **Dual Contouring**: [`DualContouring.cs`](file:///d:/workspace/projects/MillSimSharp/src/MillSimSharp/Geometry/DualContouring.cs) (internal)

### テスト
- **SDFテスト**: `tests/Geometry/SDFGridTest.cs`
- **メッシュ変換テスト**: `tests/Geometry/MeshConverterTest.cs`

### サンプル
- **Viewer**: [`VoxelViewerWindow.cs`](file:///d:/workspace/projects/MillSimSharp/src/MillSimSharp.Viewer/VoxelViewerWindow.cs) (デモアプリ)

---

## 技術的詳細

### 符号規則の理由

標準的なSDFでは「負=内部、正=外部」ですが、MillSimSharpでは削り出しシミュレーションのため:

- **負の値**: 空領域（削除された部分）= 「彫られた内部」
- **正の値**: マテリアル（残っている部分）= 「固体の内部」

この規則により、削り出し後の形状を直感的に表現できます。

### Octree の役割

Fast Sweeping で密なSDF配列を計算後、Octreeは:

1. **空間圧縮**: 均一な領域を単一ノードで表現
2. **高速クエリ**: O(log N) でSDF値を取得
3. **増分更新**: 変更領域のみノードを再構築

実際には、Octree は主に構造的な役割で、SDF値は事前計算された配列から直接読み取ります。

---

このドキュメントは、MillSimSharp の SDF 実装の全体像を把握するためのガイドです。詳細な実装は上記のコードファイルを参照してください。
