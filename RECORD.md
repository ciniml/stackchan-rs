# 作業記録: boards/m5stack — Rust版ｽﾀｯｸﾁｬﾝファームウェア

期間: 2026-07-30 〜 2026-08-03
対象: `boards/m5stack`(CoreS3 / Core2)、`m5stack-avatar-rs`、`m5drivers-rs`、`scs-servo-rs`
参考実装: C++版 `~/repos/stackchan-idf`

## 最終アーキテクチャ

```
コア0 (executor)                       コア1 (専用 executor)
├─ render : 30fps アバター描画          └─ servo : 20ms tick 首制御
│    avatar_vm バイトコード実行               PathGenerator 台形速度プロファイル
│    → RAMアリーナに合成                      SCS0009 応答なし書き込み
│    → async DMA で転送
├─ idle   : ランダムポーズ生成 (1.5〜4s)
├─ input  : FT6336 タッチ 30Hz
│    タップ=表情切替+ブリップ音
│    ホールド=視線+首がタッチ追従
├─ audio  : 音符単位合成 → async I2S DMA
│    エンベロープ→口開度 (リップシンク)
└─ net    : Wi-Fi STA (esp-radio) + HTTP API
     設定はフラッシュ保存 (再起動時に書き込み)

タスク間通信: shared_state::STATE (atomic 中心、C++版 SharedState の写像)
```

## 主要クレート構成

| 層 | 内容 |
|---|---|
| `stackchan-rs`(ルート) | HW非依存ロジック(PathGenerator) |
| `m5stack-avatar-rs::stackchan` | stackchan-idf 準拠レンダラ + avatar_vm インタプリタ |
| `m5drivers-rs` | AXP192/2101, AW9523, ILI9341/9342, **Ili9342Dma**, FT6336, AW88298, PY32 |
| `scs-servo` | SCS0009 プロトコル(応答なし書き込み対応を追加) |
| `boards/m5stack` | board(BSP)/ config / head / shared_state / tasks |

ランタイム: esp-hal 1.1 + **esp-rtos 0.3**(embassy executor 0.10)+ esp-radio 0.18 + embassy-net 0.9

## 作業内容(時系列)

### 1. マルチタスク化 (07-30)
- 単一ループの `cores3.rs`/`core2.rs` を BSP層(`board/`)+共通ポーズ空間(`head.rs`、1/1024度固定小数)+共通ループに分割
- esp-rtos + embassy 化。**esp-hal-embassy は esp-hal 1.1 と非互換**(要求する内部 feature が 1.0.0-rc.0 にしか無い)ため esp-rtos を採用
- `SharedState`(atomic群)導入、render/servo/idle をタスク分離

### 2. デュアルコア化 (07-30)
- `esp_rtos::start_second_core` + コア1専用 executor に servo タスクを固定
- 非 `Send` な UART/RefCell 共有は「コア1上で構築」する設計(`HeadParts` → `make_head`)で回避

### 3. タッチ入力 (07-31)
- FT6336 ドライバ新規作成(Core2/CoreS3 共通、I2C 0x38)
- Core2 の I2C を共有バス化(AXP192 とタッチの同居)
- タップ=表情サイクル、ホールド=首追従、アイドル抑制(3秒)

### 4. 音声出力 (08-01)
- AW88298 ドライバ(M5Unified のレジスタ列を移植)、Core2 は NS4168(AXP GPIO2 でイネーブル)
- 24kHz サイン波合成(起動アルペジオ/タップ音)、エンベロープ→口開度
- 循環DMAはリング一周で `DmaError(Late)` から復帰不能と判明し、**音符単位の一括 `write_dma_async`** に変更(音符端はゼロ振幅なので継ぎ目が聞こえない)

### 5. Wi-Fi + HTTP API (08-01)
- esp-radio 0.18 + embassy-net 0.9(バージョン整合: embassy-time 0.5 / embedded-io-async 0.7)
- 自前 HTTP サーバ(ポート80、Content-Length ボディ対応)

### 6. 設定永続化 (08-01)
- espflash 標準パーティションの `nvs` 領域(0x9000)に独自レコード(Wi-Fi認証・音量)
- esp-storage 0.9 `multicore_auto_park`(書き込み中コア1を自動停止)
- **Wi-Fi 稼働中のフラッシュ書き込みはデバイスごと落ちる**ため、保存は `/api/reboot` 直前に一本化

### 7. 実機ブリングアップと重大バグ修正 (08-01)
- espflash 4.x 対応: `esp_bootloader_esp_idf::esp_app_desc!()` 追加
- **【最重要】2021年製 xtensa GCC 8.4 リンカが現行 esp-hal と非互換**: 壊れたバイナリを静かに生成(rodata/data 化け → `Uart::new` での野良ポインタ例外、ログの NUL 洪水、見かけ上のハング)。`export-esp.sh` を espup 同梱 GCC 15.2 に更新して解決
- USB-Serial-JTAG はリセット直後の約64バイト以降を喪失(再列挙+esp-println の破棄ラッチ)→ 起動直後ログは信用できない
- SCS サーボが response level 0(書き込みに無応答)で毎コマンド 200ms タイムアウト→ scs-servo に**応答なし書き込み**を追加、プローブは読み取りコマンド化
- サーボ制御を 20ms tick + 到達時間40ms のオーバーラップ指定に(C++版と同じ連続速度化)

### 8. Wi-Fi 実地テスト (08-01)
- 接続・全API・認証情報のフラッシュ保存→env なしバイナリでの自動接続まで確認
- 視線のタッチ追従(avatar-rs に `gaze_override` 追加、サッカードは外部目標に加算)

### 9. stackchan-idf 準拠レンダラ (08-02)
- C++版 avatar(`default_face.avdsl` がピクセル単位仕様)を移植: 顔ジオメトリ・表情エフェクト・アニメータ(呼吸/サッカード/瞬時まばたき、XorShift32)・バルーン(マーキー)
- 描画は direct 方式: 背景は全再描画時のみ、要素ごとにスクラッチスプライト合成→一括転送

### 10. avatar_vm 移植 (08-02)
- `AVDS` v1 デコーダ + スタックマシン(全オペコード、スタック64/ローカル256/深度16)を完全移植
- デフォルト顔はC++ファームと同一の 1630B バイトコードを埋め込み
- `POST /api/face`(バイナリボディ、検証付き)で **顔のホットスワップ** / `POST /api/face/reset`
- 実機で aokko_face.avbc の差し替え・復帰・不正拒否を確認

### 11. 描画の DMA 化 (08-02)
- `Ili9342Dma`: SPIバス直結ドライバ。ブロッキング init → `map_bus(into_async)`、4KBチャンクの DMA 転送(転送中 CPU は他タスクへ)
- `tick_async`: VM実行→RAMアリーナに全グループ合成→async 転送、バルーンはアリーナ再利用の2パス
- 従来フレームあたり約20msあったコア0のSPIブロッキングを解消

### 12. ちらつき解消 (08-02)
- 原因1: エフェクトグループ(x≥236)が右目・右眉グループと重なり、後からのクリアで右端が毎フレーム消えていた → DSL で **effect() を最初に描画**(stackchan-idf 側の asset にも同修正)
- 原因2: 重なりグループ間の転送タイムラグ → **転送前マージ**(後のグループのピクセルを先のグループのバッファへコピー、全画素が最初の転送で最終内容になる)で構造的に解消

### 13. セットアップ AP + キャプティブポータル (08-03)
- 認証情報が無いとき WPA2 AP(`Stackchan-XXXXXX` / `sc-xxxxxxxx`、C++版と同じ MAC 由来)+ 192.168.4.1 で起動
- 最小 DHCP サーバ(MAC 由来の決定的リース)+ 全応答 DNS + API 以外 302 リダイレクトでスマホのログインシートを自動表示
- `GET /` の設定フォーム(STA モードでも可)→ 保存 → 再起動で STA 接続。`/api/wifi/clear` で AP モードに戻せる
- **スタック知見**: コア0メインスタックは「RAM の残り」(静的領域を増やすと黙って縮む)。さらに embassy タスク future はスポーン時に一時スタック構築されるため、serve×2(各12KB)を抱えた net future がスタックを溢れさせた → ヒープ 200→176KB + `Box::pin(serve(...))` で解決

## 重要な技術的知見

1. **リンカ**: `~/.espressif` の GCC 8.4(2021)は現行 esp-hal のリンカスクリプトと非互換。espup 同梱 GCC 15.2 を使う。リンカ変更後は `touch src/main.rs` で再リンク必須
2. **ヒープ**: esp-rtos の esp-alloc 連携で複数ヒープ領域が登録され、`HEAP.free()` は合算値(連続性を保証しない)。Wi-Fi の長命確保で断片化すると 50KB 確保が「200KB空き」でも失敗する → 大きな長命バッファ(フレームアリーナ64KB)は **net タスク起動前に一括確保**
3. **フラッシュ書き込み**: Wi-Fi 稼働中は不可(キャッシュ無効化と衝突)。リセット直前に集約
4. **`#[esp_rtos::main]`**: executor を先に起動するため、`esp_rtos::start` を main 内で呼ぶ構成とは併用不可 → `#[esp_hal::main]` + 手動 executor
5. **ESP32(無印)の DRAM**: esp-radio 込みでヒープ 112KB が上限(超えるとリンク時 "cannot move location counter backwards")

## HTTP API 一覧(ポート80)

```
GET  /api/status                    表情/首/口/音量
POST /api/expression/<name>         neutral|happy|angry|sad|doubt|sleepy
POST /api/head/<pan>/<tilt>         度指定 (0-180)
POST /api/sound/<arpeggio|blip>
POST /api/volume/<0-100>            (再起動時に永続化)
POST /api/wifi/<ssid>/<pass>        (再起動時に永続化・適用)
POST /api/wifi/clear                認証情報クリア(再起動で AP モードへ)
GET  /                              Wi-Fi 設定ページ / POST /setup (フォーム)
POST /api/balloon/<text>            UTF-8(日本語可、%エンコード、_→スペース)/ balloon/clear
POST /api/face                      AVDS v1 バイナリボディ(再起動時に永続化、最大8KB)/ face/reset
POST /api/reboot                    設定保存 + リセット
```

顔バイトコードの作成: `node ~/repos/stackchan-idf/tools/avatar_dsl/cli.mjs face.avdsl face.avbc`

## ビルド・書き込み

```bash
source export-esp.sh   # GCC 15.2 を PATH に (必須)
cd boards/m5stack
# CoreS3
cargo build --release --no-default-features --features cores3 --target xtensa-esp32s3-none-elf
espflash flash --monitor --port /dev/ttyACM0 target/xtensa-esp32s3-none-elf/release/stackchan-m5stack
# Core2
cargo build --release --target xtensa-esp32-none-elf
```

Wi-Fi 初回設定: `WIFI_SSID=... WIFI_PASS=...` でビルドするか、接続後に `/api/wifi` + `/api/reboot`。

## 残タスク

- [x] 顔バイトコードの永続化(nvs 領域 0xA000 に独自レコード、最大8KB。保存は reboot 経路のみ)
- [x] バルーンの日本語フォント(u8g2-fonts の b16/b12 japanese3 = JIS第1+2水準。u8g2 に 24px カットが無いため大パネルは 16px)
- [ ] Core2 実機での動作確認(ビルドは通っている。描画はブロッキング経路のまま)
- [ ] チルトサーボ(id=2)接続時の2軸動作確認
- [ ] stackchan-idf 側 `assets/default_face.avdsl` の effect 先行描画修正のコミット(適用済み・未コミット)

## コミット一覧

- stackchan-rs: `5978103`(マルチタスク/デュアルコア化)→ `e40fd9e`(gaze/サーボ/保存方式)→ `0b98dc8`(stackchan-idf レンダラ + avatar_vm)→ `95e127c`(DMA描画)
- m5stack-avatar-rs: `58d0e9d`(gaze override)→ `70a826e`(stackchan レンダラ + VM)→ `7f4995e`(async パイプライン + ちらつき解消)
- m5drivers-rs: `1747e08`(PY32/FT6336/AW88298)→ `e16aa0e`(Ili9342Dma)
- scs-servo-rs: `a287b4e`(応答なし書き込み)
