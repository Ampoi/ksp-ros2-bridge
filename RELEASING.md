# PyLoNのリリース

MODはKSPをインストールしたローカル環境でビルドし、GitHub Releasesへ手動でアップロードします。CI用runnerや参照DLLのアップロードは不要です。

## ソースと生成物

- `Source/PyLoN`: C#ソース。
- `Assets/PyLoN`: 配布用CFG・モデル・画像の原本。Gitで管理します。
- `GameData/PyLoN`: ビルド時に原本とDLLから毎回作り直す配置用フォルダ。Git管理外です。
- `dist/PyLoN-vX.Y.Z.zip`と`.zip.sha256`: 配布ZIPとSHA-256チェックサム。Git管理外です。

`PyLoN.csproj`はKSPインストール内の`Assembly-CSharp.dll`とUnityのDLLをコンパイル時に参照します。これらは`Private=false`であり、配布ZIPにも入りません。パッケージ処理は`Assets/PyLoN`のCFG・MU・PNGとビルドした`PyLoN.dll`、ルートの`LICENSE`を格納します。`Development/`、`Tools/`、PDB、ROS2ワークスペースは含めません。

## 配布ZIPを作る

Linuxで.NET 8 SDK、Python 3、KSP 1.xを用意し、リリース対象のソースを確認してから実行します。

```bash
./package.sh v1.0.0 --ksp-dir "$HOME/.local/share/Steam/steamapps/common/Kerbal Space Program"
```

バージョンは例です。`v1.0.0-rc.1`形式も使えます。`KSPDIR`環境変数も指定可能です。`package.sh`はRelease構成でビルドしてからZIPを作り、KSPへのインストールは行いません。Windowsでの通常のビルドは引き続き`build.ps1`を使えます。

```bash
cd dist
sha256sum --check PyLoN-v1.0.0.zip.sha256
unzip -l PyLoN-v1.0.0.zip
```

ZIP直下は`GameData/PyLoN/`で、`LICENSE`、`Plugins/PyLoN.dll`、`Config/`、`Models/`、`Parts/`を含みます。利用者はこの`PyLoN`フォルダをKSPの`GameData`へ配置します。

## GitHubへ登録する

1. 変更をコミット・pushし、ビルドに使ったコミットをリリース対象にします。
2. [Releases](https://github.com/Ampoi/KSP_ROS2/releases)の「Draft a new release」で同じバージョンのタグを選択または作成し、対象コミットを確認します。
3. `dist/`のZIPと`.zip.sha256`を添付し、変更内容を記載して公開します。候補版はpre-releaseにします。

GitHubが自動生成する「Source code (zip/tar.gz)」はMOD配布物ではありません。利用者には添付した`PyLoN-vX.Y.Z.zip`を案内してください。
