# PyLoN本体への貢献

このセクションは、PyLoNのKSP MOD、ROS2 bridge、共通制御器、メッセージ定義、ドキュメントを変更する人向けです。

## 開発環境

[Getting Started](../guide/getting-started.md)に従ってKSPとROS2 Jazzyを準備します。実装の責務と依存方向は[アーキテクチャ](architecture.md)、機能ごとのファイルは[ソース案内](source-map.md)を参照してください。

作業前にリポジトリの`AGENTS.md`を確認してください。ローカルに`Development/AGENTS.md`がある場合、調査やKSP実装の変更前にそちらも確認します。実験コード、検証記録、モデルの編集元はGit管理外の`Development/`で管理します。本体のビルドは公開ソースだけで完結させます。

## ビルドと同期

KSPを終了し、リポジトリのルートで実行します。

```bash
./sync.sh
```

KSPプラグインのビルドと配置、ROS2パッケージの同期とcolconビルドを行います。インストール先を変える場合は環境変数で指定します。

```bash
KSPDIR="/path/to/Kerbal Space Program" ROS2_WS="$HOME/ros2_ws" ./sync.sh
```

KSP側だけ、またはROS2側だけを変更した場合は範囲を指定できます。

```bash
# KSP側
./sync.sh --skip-ros2-sync --skip-ros2-build

# ROS2側
./sync.sh --skip-ksp-build --skip-ksp-sync
```

ローカルに`./dev`がある環境では、同じ引数で`./dev sync`を使用できます。追跡対象のエントリーポイントは`./sync.sh`です。ROS2の既定環境は`/opt/ros/jazzy/setup.bash`で、`ROS_SETUP`で変更できます。

## 変更の検証とコミット

1. 変更した機能のビルドと関連するテストを実行します。[設計と検証の観点](design.md)も参照してください。
2. MODやbridgeに関係するコード・設定・アセット・ドキュメントを変更したら、上記の同期コマンドを実行します。
3. KSPとbridgeを再起動し、対象Topicの型、値、単位、座標系、開始・終了時の動作を確認します。
4. APIを変更したら、メッセージ定義と利用者向けの説明・実行例を合わせて更新します。
5. コミットは目的ごとにまとめ、変更内容、利用者への影響、実施した検証を説明します。同期を実行できなかった場合は、理由と再実行するコマンドを記載します。

生成物やローカルの検証記録はコミット対象から外します。

## ドキュメントの編集

Node.jsとpnpmを用意し、リポジトリのルートから実行します。

```bash
cd docs
pnpm install --frozen-lockfile
pnpm run docs:dev
```

静的ビルドとプレビューは次のとおりです。

```bash
pnpm run docs:build
pnpm run docs:preview
```

ビルド時にリンク切れを確認し、プレビューでナビゲーションとコード例の表示を確認します。公開設定は`docs/`にあり、VercelのRoot DirectoryとCLIの作業ディレクトリも`docs`です。

利用者向けのページには導入・アプリケーション開発・APIの使い方を記載します。本体への貢献手順と内部設計は、この末尾セクションにまとめます。
