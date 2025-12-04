# DEVELOPER Guide

このドキュメントは開発者向けのチェックリストと手順をまとめたものです。特に SDF / Marching Cubes 関連の開発やテストに焦点を当てています。Viewer はサンプルアプリであり、主に動作確認やデモに利用します。

---

## 開発環境のセットアップ

- 必要: .NET 8.0
- 一般的なビルド / テスト:

  ```powershell
  cd d:\workspace\projects\MillSimSharp
  dotnet build
  dotnet test
  ```

- Viewer（サンプル）の実行（必要に応じて）:

  ```powershell
  dotnet run --project src\MillSimSharp.Viewer
  ```

---

## 重要なコード箇所

- `SDFGrid` - SDF の計算・取得・オンデマンド再計算
  - `src/MillSimSharp/Geometry/SDFGrid.cs`
- `MeshConverter` - Marching Cubes を使ったメッシュ生成
  - `src/MillSimSharp/Geometry/MeshConverter.cs`
- Viewer（サンプル）の場所（実装例）
  - `src/MillSimSharp.Viewer/VoxelViewerWindow.cs`

---

## ローカルのビルドとデバッグのヒント

- SDF 関連のバグ（境界面が消える、面が反転する等）は、次の順に再現・確認することをお勧めします:
  1. 小さなボクセルグリッド（低分解能）にして検証する
  2. `fastMode` を無効にして（SDF の精度を上げる）再実行する
   3. 「Viewer（サンプル）」を使って可視化できることを確認する（例: カリングのオン/オフで見え方を確かめるなど）。
  4. SDF は境界で distances がクランプされる設計になっているか確認する（境界サンプリングの挙動）

> Note: Viewer はサンプルアプリケーションです。詳細なキー操作や GUI は本ドキュメントの中心ではなく、必要に応じてソースコード（`VoxelViewerWindow.cs`）内の実装を参照してください。

---

## テストのベストプラクティス

- SDF に対する数値精度を確認するユニットテスト（例: 階差や境界勾配検査）は `fastMode=false` を使うこと。
- テスト実行時間を短くしたい場合: 重いメッシュ生成テストは `fastMode=true` を使うと良いです。
- テスト修正時:
  - `tests/Geometry/SDFGridTest.cs` を確認し、`ComputeSDFFast` の挙動がどのテストで使われているかを把握すること
  - `MeshConverterTest.cs` で `fastMode` を切り替えたテストを分けておくと、失敗原因の調査が容易になります。

---

## デバッグ・レアケース

- 若干の面が消えるとき:
  - 通常は SDF の境界クラップ処理が不足しているため、グリッド外のサンプルが ± narrowBandWidth の固定値を返しているケース。
  - `SDFGrid.GetDistance(int x, int y, int z)` の実装と、`GetDistance` の浮動小数点補間処理を確認してください。

- Mesh のワインディングが反転しているとき:
  - `MeshConverter` の三角形生成で法線（SDF 勾配）を使い、ワインディングの適正化を行っている箇所が正しく動作しているかを確認してください。

---

## 将来の改善点（リスト）

- フル EDT ベースの SDF 実装（精度と速度の両方の改善を目指す）
- Viewer 側の `fastMode` UI スイッチを追加（ユーザーが GUI で切り替えられるようにする）
- SDF 生成の進捗表示とキャンセルの実装

---

## PR とコード変更

- SDF の設計変更を行う際は、必ず:
  1. ユニットテストの追加または修正（境界テストやワインディング確認など）
   2. Viewer（サンプル）での再現手順を参考にする
  3. パフォーマンスに影響がある場合は `fastMode` を導入して互換性を保持

---

これらの手順に沿って作業すれば、SDF と Marching Cubes 複合の問題（外殻が表示されない etc.）に対して再発を抑止しつつ、開発者とユーザーの両方にとって分かりやすい挙動に保てます。
