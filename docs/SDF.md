# SDF (Signed Distance Field)

このドキュメントは、MillSimSharp における SDF（Signed Distance Field）生成と Marching Cubes ベースのメッシュ変換の実装と設計に関する詳細情報をまとめたものです。開発者向けの内部挙動やデバッグのヒント、`fastMode` の利点・トレードオフを中心に説明します。

---

## 概要

- SDF はボクセル格子（VoxelGrid）から算出される距離場で、ゼロレベルセット（distance=0）が表面を表します。
- SDF をサンプリングして Marching Cubes によって三角形メッシュを生成します。
- メッシュ生成は `MeshConverter` によって実行され、`ConvertToMeshFromSDF` と `ConvertToMeshViaSDF` といった API から呼ばれます。

---

## 実装上の主要点

### 1) fastMode（高速モード）

- `SDFGrid` には `fastMode`（bool）があり、小さいグリッドやテスト目的で SDF 計算を省略/近似するために利用できます。
- `fastMode` の動作:
  - 高速なスキャンベースの近似アルゴリズム（ComputeSDFFast）を使う
  - 精度と引き換えに計算時間を削減
  - 大きなボクセルグリッドや CI テストの時間短縮に有用
- 注意: 精度を要求するユニットテスト（例: SDF の勾配テスト、境界の正確性テスト）は `fastMode=false` を使う必要があります。

### 2) 境界のサンプリングとクラッピング

- 以前はグリッド外のサンプリングが単純に `±narrowBandWidth` を返す実装であり、これが境界セルで全てのコーナーの符号が同じになってしまい、キューブインデックスが 0 や 255 になって三角形が生成されないことがありました。
- 改善点:
  - グリッド外のサンプルは、グリッド境界までの距離を計算してクランプして返す（つまり格子の外側でも距離が正しく推定される）。
  - これにより隣接するセルでの補間がより安定になります。

### 3) ノード（コーナー）でのサンプリング

- Marching Cubes の安定性を高めるため、センターではなくノード（角）で SDF を評価するようにしました。
- これにより境界セルにおける符号変化の検出が正しく行われ、欠けた面の生成を防ぎます。

### 4) スレッドローカル SDF キャッシュ

- SDF の GetDistance 呼び出しは頻繁に行われるため、`MeshConverter` ではスレッドローカルの簡易キャッシュ(`SdfCache`)を導入しました。
- キーは `x,y,z` と narrow-band offset の組合せに基づく固定インデックスで、キャッシュミスを最小化します。

### 5) Marching Cubes の範囲

- Marching Cubes を 0..(size-1) のループで実行すると、グリッドの最外殻にあるキューブを処理できない場合があります。
- 修正: ループを -1 から size-1 の範囲に拡張して境界まで計算するようにしました。

### 6) トライアングルのワインディングと退化除外

- 面の向き（ワインディング）は SDF 勾配を使い、必要に応じて三角形の反転処理を行います。
- 面積がほぼゼロの三角形（退化した頂点）をスキップし、レンダリング時のアーティファクトを軽減します。

---

## SDF を使ったメッシュ生成でのデバッグ手順（穴が現れる場合）

1. Viewer（サンプルアプリケーション）で SDF を確認することはできますが、Viewer は本リポジトリのメイン機能ではなく動作確認用のサンプルです。Viewer の GUI やキー操作はサンプル実装に留まり、詳細なドキュメントは含めていません。
3. 小さいグリッド（低分解能）で再現できるか確認する。大きなグリッドでは近似の影響が見られやすい。
4. 精度が必要なテストは `fastMode=false` を指定して SDF を再計算する。
5. `MeshConverter` の `SdfCache` が正しく初期化されていることを確認する（viewer で NullReference が発生した場合は SdfCache 初期化をチェック）。

---

## 性能上のアドバイス

- ビューアーのデフォルトは `fastMode=false`（精度優先）です。大きなグリッドは時間がかかるため、プレビューやテストでは `fastMode=true` を使うと良いです。
- さらに高速で正確な SDF を目指す場合は、EDT（Euclidean Distance Transform）を検討してください（今後の改善案）。

---

## その他の改善案

- Viewer に `fastMode` の UI 切り替えを追加して、一時的に高速モードで SDF を作成したいときに切り替えられるようにする。
- SDF 生成の進捗表示・キャンセル機能を Viewer に追加する。

---

## 参考コード箇所

- SDF 計算: `src/MillSimSharp/Geometry/SDFGrid.cs`
- Mesh 変換: `src/MillSimSharp/Geometry/MeshConverter.cs`
`VoxelViewerWindow`（Viewer はサンプルアプリ）: `src/MillSimSharp.Viewer/VoxelViewerWindow.cs`
- Unit tests:
  - `tests/Geometry/SDFGridTest.cs`
  - `tests/Geometry/MeshConverterTest.cs`

---

このドキュメントは SDF 周りの最近の修正と実装に関する要点をまとめたものです。直接的なコード変更や最適化を行う際には、上記のコード箇所を参照してください。
