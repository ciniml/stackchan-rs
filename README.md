# stackchan-rs

Rust 版ｽﾀｯｸﾁｬﾝ ([Stack-chan](https://github.com/stack-chan/stack-chan)) ファームウェアです。
C++ 版 [stackchan-idf](https://github.com/ciniml/stackchan-idf) を参照実装として、
esp-hal (no_std) + embassy 上にフルスクラッチで実装しています。

開発の経緯・設計判断・技術的知見は [RECORD.md](RECORD.md) にまとめています。

## 対応ハードウェア

| ボード | 状態 |
|---|---|
| M5Stack CoreS3 / CoreS3 SE | 実機確認済み(描画は async DMA) |
| M5Stack Core2 | ビルド可(実機未確認、描画はブロッキング SPI) |

首(サーボ)は Feetech SCS0009 × 2(pan=id1, tilt=id2)を UART バスで制御します。
未接続でもアバター単体で動作します。

## 主な機能

- **アバター描画** — stackchan-idf 準拠のレンダラ + `avatar_vm` バイトコード
  インタプリタ(AVDS v1)。顔は HTTP でホットスワップ&フラッシュ永続化
- **デュアルコア** — コア0: 描画/入力/音声/ネットワーク、コア1: サーボ制御専用
  (台形速度プロファイル、20ms tick)
- **タッチ** — タップで表情切替+ブリップ音、ホールドで視線と首がタッチ追従
- **音声** — 音符単位合成の async I2S DMA 再生、エンベロープ連動リップシンク
- **吹き出し** — 日本語対応(u8g2 b16/b12 japanese3 フォント)、マーキースクロール
- **Wi-Fi + HTTP API** — 表情・首・音・音量・吹き出し・顔の遠隔操作
- **セットアップ AP** — 未設定時は WPA2 AP + キャプティブポータルで起動し、
  スマホから Wi-Fi 設定(C++ 版 captive_portal 相当)

## リポジトリ構成

```
stackchan-rs/            首制御など HW 非依存ロジック (PathGenerator)
├── boards/m5stack/      M5Stack 向けファームウェア本体
├── m5stack-avatar-rs/   [submodule] アバター描画 + avatar_vm インタプリタ
├── m5drivers-rs/        [submodule] AXP192/2101, ILI9341/9342(DMA), FT6336, AW88298 等
└── scs-servo-rs/        [submodule] SCS0009 サーボプロトコル
```

## ビルドと書き込み

espup でインストールした Xtensa Rust ツールチェイン(チャネル `esp`)が必要です。
リンカは espup 同梱の GCC を使います(**古い `~/.espressif` の GCC 8.4 は不可**。
壊れたバイナリを静かに生成します — 詳細は RECORD.md)。

```bash
git clone --recurse-submodules https://github.com/ciniml/stackchan-rs.git
cd stackchan-rs
source export-esp.sh          # リンカを PATH に(必須)
cd boards/m5stack

# CoreS3
cargo build --release --no-default-features --features cores3 \
    --target xtensa-esp32s3-none-elf
espflash flash --monitor --port /dev/ttyACM0 \
    target/xtensa-esp32s3-none-elf/release/stackchan-m5stack

# Core2
cargo build --release --target xtensa-esp32-none-elf
```

## Wi-Fi 設定

初回起動時(認証情報未設定)は自動でセットアップ AP モードになります。

1. 画面の吹き出しに表示される AP(`Stackchan-XXXXXX` / パスワード `sc-xxxxxxxx`)に
   スマホで接続
2. キャプティブポータル(または `http://192.168.4.1/`)の設定ページで SSID と
   パスワードを入力
3. 保存すると再起動し、設定した Wi-Fi に接続します

ビルド時に `WIFI_SSID=... WIFI_PASS=...` 環境変数で埋め込むことも、接続後に
`/api/wifi/<ssid>/<pass>` + `/api/reboot` で変更することもできます。
`/api/wifi/clear` + `/api/reboot` で AP モードに戻ります(設定初期化)。

## HTTP API(ポート 80)

```
GET  /                              Wi-Fi 設定ページ
GET  /api/status                    表情/首/口/音量
POST /api/expression/<name>         neutral|happy|angry|sad|doubt|sleepy
POST /api/head/<pan>/<tilt>         度指定 (0-180)
POST /api/sound/<arpeggio|blip>
POST /api/volume/<0-100>            再起動時に永続化
POST /api/wifi/<ssid>/<pass>        再起動時に永続化・適用
POST /api/wifi/clear                認証情報クリア(再起動で AP モードへ)
POST /api/balloon/<text>            UTF-8(日本語可、%エンコード)/ balloon/clear
POST /api/face                      AVDS v1 バイナリボディ(再起動時に永続化)/ face/reset
POST /api/reboot                    設定保存 + リセット
```

顔バイトコードは stackchan-idf の DSL コンパイラで作成します:

```bash
node <stackchan-idf>/tools/avatar_dsl/cli.mjs face.avdsl face.avbc
curl --data-binary @face.avbc -X POST http://<ip>/api/face
```

## ライセンス

MIT または Apache-2.0 のデュアルライセンスです([LICENSE-MIT](LICENSE-MIT) /
[LICENSE-APACHE](LICENSE-APACHE))。

吹き出しの日本語フォントは [u8g2](https://github.com/olikraus/u8g2) プロジェクト
由来(/efont/ unicode bitmap fonts — 主にパブリックドメインの東雲・jiskan フォント)
です。
