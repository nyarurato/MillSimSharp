# MillSimSharp Samples

このディレクトリには、MillSimSharpライブラリの使い方を示すサンプルプロジェクトが含まれています。

## サンプル一覧

### 01-BasicToolpath
基本的なツールパスシミュレーションのサンプルです。

**内容**:
- VoxelGridの作成
- EndMillツールの定義
- 単純なツールパス（四角形のポケット加工）の実行
- STLエクスポート

**出力**: `output/basic_toolpath.stl`

**実行方法**:
```bash
cd 01-BasicToolpath
dotnet run
```

---

### 02-SDFMeshGeneration
SDFを使用した高品質メッシュ生成のサンプルです。

**内容**:
- 複雑な形状の作成（球と円柱の組み合わせ）
- SDFGridの生成（Narrow band最適化）
- Dual Contouringによる高品質メッシュ生成
- STLエクスポート

**出力**: `output/sdf_mesh.stl`

**実行方法**:
```bash
cd 02-SDFMeshGeneration
dotnet run
```

**ポイント**: 直接VoxelGridからエクスポートした場合と比較して、表面の滑らかさの違いを確認できます。

---

### 03-CustomShapes
カスタム形状の削り出しサンプルです。

**内容**:
- 複数ツールの使用（荒加工用と仕上げ用）
- レイヤーごとの円形ポケット加工
- スパイラル仕上げパス
- STLエクスポート

**出力**: `output/custom_shape.stl`

**実行方法**:
```bash
cd 03-CustomShapes
dotnet run
```

**ポイント**: 荒加工と仕上げ加工の2段階プロセスを実演しています。

---

### 04-StepByStep
ステップバイステップ実行のサンプルです。

**内容**:
- ToolpathExecutorのステップバイステップ実行
- 各ステップでの中間結果の保存
- 複数のSTLファイル出力

**出力**: `output/step_01.stl`, `output/step_02.stl`, ..., `output/step_final.stl`

**実行方法**:
```bash
cd 04-StepByStep
dotnet run
```

**ポイント**: 生成されたSTLファイルを順番に読み込むことで、加工プロセスをアニメーションのように確認できます。

---

## すべてのサンプルを実行

すべてのサンプルを一度に実行するには、samplesディレクトリから：

```bash
# Windowsの場合
foreach ($dir in Get-ChildItem -Directory) { 
    Push-Location $dir.FullName
    dotnet run
    Pop-Location
}

# Linux/macOSの場合
for dir in */; do
    (cd "$dir" && dotnet run)
done
```

---

## 出力ファイルの確認

各サンプルは `output/` フォルダにSTLファイルを生成します。以下のような3Dビューアーで確認できます：

- **Windows**: 3D Builder（標準搭載）
- **クロスプラットフォーム**: Blender, MeshLab, FreeCAD
- **オンライン**: [3D Viewer Online](https://3dviewer.net/)

---

## 要件

- .NET 8.0 SDK
- MillSimSharpライブラリ（自動的にプロジェクト参照されます）

---

## カスタマイズ

各サンプルのソースコードは自由に編集できます。以下のパラメータを変更して実験してみてください：

- **解像度**: `VoxelGrid`の`resolution`パラメータ
- **ワークエリアサイズ**: `BoundingBox.FromCenterAndSize()`の引数
- **ツール径**: `EndMill`の`diameter`パラメータ
- **送り速度**: `G1Move`の第2引数
- **Narrow Band幅**: `SDFGrid.FromVoxelGrid()`の`narrowBandWidth`パラメータ

---

## トラブルシューティング

### メモリ不足エラー
- VoxelGridの解像度を大きくする（例: 1.0mm → 2.0mm）
- ワークエリアサイズを小さくする
- SDFを使用する場合は`useSparse: true`を設定

### 実行が遅い
- 解像度を粗くする
- `fastMode: true`を使用（精度は低下）
- Narrow Band幅を小さくする（例: 2）

### STLファイルが開けない
- ファイルサイズを確認（0バイトでないか）
- 別の3Dビューアーで試す
- エラーメッセージを確認

---

詳細なドキュメントは[README.md](../README.md)と[docs/SDF.md](../docs/SDF.md)を参照してください。
