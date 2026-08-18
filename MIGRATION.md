# Formulator Pen migration

この開発リポジトリは `Renato-D/formulator_pen` の履歴を引き継ぎます。

## Remotes

- `upstream`: 同僚のリポジトリ。取得専用で、pushは無効化する
- `origin`: 自分のGitHubリポジトリ。作成後に追加する

## Local source

従来版は次に変更せず保全します。

`C:\sdl_dev\formulator-pen-project\legacy\formulator_pen-main-20260727`

従来版の装置固有設定や校正値を最新版へ単純上書きせず、元コミットを特定してから
移行ブランチ上で機能単位に取り込みます。

## Legacy baseline identified

2026-08-18の照合で、保全した従来版17ファイルは上流コミット
`96b90343a528591a76db0e1fe4feaade7b276024`（2026-07-24、`deleted unused codes`）と
全ファイル完全一致しました。追跡不能なローカル差分はありません。

最新版との差分に含まれるIPアドレス、USBデバイスパス、流体プロファイル、校正係数は、
旧環境で必要だった値かを実機構成と照合し、将来的にはGit対象外のローカル設定へ分離します。
