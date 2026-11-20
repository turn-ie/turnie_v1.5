#include "Config.h"
#include "Display.h"
#include "Storage.h"
#include "Animations.h"
#include "Content.h"
#include "Inbox.h"
#include "BleStack.h"
#include "BleGatt.h"
#include "BleBroadcast.h"
#include "OTA_Handler.h"
#include "Power.h"
#include "Fluid.h"
#include "Settings.h"
#include <ArduinoJson.h>

// ============================================================
// Config.h extern 変数の実体定義
//   BLE "settings" JSON で書き換え可能
// ============================================================
uint16_t MOTION_HUE       = 104; // 色相 0–360（度）。turnie4=104 (黄)
uint8_t MOTION_BRIGHTNESS = DEFAULT_MOTION_BRIGHTNESS;  // 内部はハードウェア値 0–255（setting.json の 0–100 から変換）
uint8_t MOTION_ANIM       = 0;   // 0=Ripple / 1=DiagonalWave
char    DEVICE_NAME[DEVICE_NAME_MAX] = {};  // 起動時に BLE_DEVICE_NAME で初期化
char    HOMETOWN[HOMETOWN_MAX]       = {};  // 出身地（setting.json の hometown）

// 受信メッセージの受け渡し（BT タスクでは固定長コピーだけを行う）
static constexpr size_t RX_BUFFER_BYTES = 200;
static uint8_t       g_rxBuffer[RX_BUFFER_BYTES];
static size_t        g_rxLength            = 0;
static portMUX_TYPE  g_rxMux                = portMUX_INITIALIZER_UNLOCKED;
static unsigned long g_lastRxTime         = 0;
static String        g_lastSavedJson      = ""; // 直前に inbox へ保存した内容（重複保存ガード用）
static unsigned long g_lastSavedTime      = 0; // 直前に inbox へ保存した時刻（millis()）
static volatile bool newMessageFlag        = false;
static volatile bool g_needReply           = false;
static Content::Data parsedContent;
static bool          showContentAfterAnim = false;

// 受信履歴の閲覧モード（USER長押しで開始/終了、短押しで次へ、無操作BROWSE_TIMEOUT_MSで通常へ）
static bool          g_browseMode        = false;
static int           g_browseIndex       = 0;     // Inbox の index（0=最古, size-1=最新）
static unsigned long g_browseLastActive  = 0;     // 最後にボタン操作した時刻

// ピクチャ確認モード（USER短押しで自分のピクチャ表示。再短押し or タイムアウトで流体へ）
static const unsigned long PREVIEW_TIMEOUT_MS = 10000;  // 10 秒で流体へ自動復帰
static bool          g_previewMode       = false;
static unsigned long g_previewStartedAt  = 0;

// ---------------------------------------------------------
// コンテンツをマトリクスに表示する（テキスト・画像共通）
// ---------------------------------------------------------
void showContent(const Content::Data& data, bool isReceived = false) {
    if (data.kind == Content::Kind::Text) {
        // 自分のテキストなら無限ループ、受信したテキストなら1回のみ
        Display::showText(data.text.c_str(), TEXT_SCROLL_DELAY_MS, !isReceived);
    } else if (data.kind == Content::Kind::Image) {
        // 受信したデータのときのみアニメーション（1ピクセルずつ表示）を有効にする
        // 自分のデータのときは無期限表示 (ULONG_MAX) にする
        unsigned long duration = isReceived ? data.durationMs : ULONG_MAX;
        Display::showImage(data.rgb, sizeof(data.rgb), duration, isReceived);
    }
}

// ---------------------------------------------------------
// 閲覧モード：現在の g_browseIndex の受信履歴を表示する。
//   パースできない履歴は古い方へ読み飛ばす。
//   履歴が空（または全て無効）なら "no data" を表示する。
// ---------------------------------------------------------
void showBrowseItem() {
    int n = Inbox::size();
    if (n <= 0) {
        Display::showText("no data", TEXT_SCROLL_DELAY_MS, true, Animations::motionHueColor());
        return;
    }

    // 有効な履歴が見つかるまで最大 n 件、古い方へ読み飛ばす
    for (int tries = 0; tries < n; tries++) {
        Inbox::Item item;
        Content::Data data;
        if (Inbox::get(g_browseIndex, item) && Content::parse(item.json, data)) {
            if (data.kind == Content::Kind::Text) {
                // 次の操作までループスクロール
                Display::showText(data.text.c_str(), TEXT_SCROLL_DELAY_MS, true);
            } else if (data.kind == Content::Kind::Image) {
                // 1ピクセルずつ表示し、次の操作まで保持（ULONG_MAX）
                Display::showImage(data.rgb, sizeof(data.rgb), ULONG_MAX, true);
            }
            return;
        }
        // 無効な履歴 → 1つ古い方へ（末尾は最新へループ）
        g_browseIndex = (g_browseIndex - 1 + n) % n;
    }

    // 有効な履歴が1件も無かった
    Display::showText("no data", TEXT_SCROLL_DELAY_MS, true, Animations::motionHueColor());
}

// 閲覧モードを開始する（最新の履歴から表示）
void enterBrowseMode() {
    Animations::startDiagonalWave();   // 遷移演出（ブロッキング）
    g_browseMode       = true;
    g_browseLastActive = millis();
    int n = Inbox::size();
    g_browseIndex = (n > 0) ? (n - 1) : 0;   // 最新
    showBrowseItem();
    Serial.println("  [Browse] enter");
}

// 閲覧モードを終了して通常フローへ戻す
void exitBrowseMode() {
    Animations::startDiagonalWave();   // 遷移演出（ブロッキング）
    g_browseMode = false;
    Display::clear();
    Serial.println("  [Browse] exit");
}

// ピクチャ確認モードの開始/終了をトグルする（USER ボタン短押し）
//   開始：自分のピクチャを表示し、10秒のタイマーを張る
//   終了：流体表示へ戻す
void togglePreview() {
    if (!g_previewMode) {
        g_previewMode  = true;
        g_previewStartedAt = millis();
        Display::clear();
        const Content::Data& own = Content::own();
        if (own.kind == Content::Kind::Image) {
            Display::showImageFadeIn(own.rgb, sizeof(own.rgb), ULONG_MAX);
        } else {
            showContent(own);   // テキストなどは従来どおり表示
        }
        Serial.println("  [Preview] show own picture");
    } else {
        g_previewMode = false;
        Display::clear();              // 流体表示は loop の待機処理で再開する
        Serial.println("  [Preview] back to fluid");
    }
}

// 1つ古い履歴へ進む（最古の次は最新へループ）
void browseNext() {
    int n = Inbox::size();
    if (n <= 0) return;
    g_browseIndex = (g_browseIndex - 1 + n) % n;
    g_browseLastActive = millis();
    showBrowseItem();
}

// ---------------------------------------------------------
// 他のデバイスからメッセージを受信したときに呼ばれる（BT タスク）
//   ここでは固定長バッファへのコピーとフラグ設定だけを行う。
//   JSON 化・パース・保存・返信は loop() 内でメインタスクが処理する。
// ---------------------------------------------------------
void onMessageReceived(const uint8_t* data, size_t len) {
    if (data == nullptr || len == 0 || len > sizeof(g_rxBuffer)) return;

    portENTER_CRITICAL(&g_rxMux);
    // loop() が未処理の 1 件を保持している間は上書きしない。
    if (!newMessageFlag) {
        memcpy(g_rxBuffer, data, len);
        g_rxLength = len;
        newMessageFlag = true;
    }
    portEXIT_CRITICAL(&g_rxMux);
}

// BT タスクから渡された 1 件をメインタスクへ取り出す。
static bool takePendingMessage(uint8_t* out, size_t& outLen) {
    bool ready = false;
    portENTER_CRITICAL(&g_rxMux);
    if (newMessageFlag) {
        outLen = g_rxLength;
        memcpy(out, g_rxBuffer, outLen);
        newMessageFlag = false;
        ready = true;
    }
    portEXIT_CRITICAL(&g_rxMux);
    return ready;
}

// 受信データの重い処理はすべて loop() の文脈で行う。
static bool processPendingMessage() {
    uint8_t raw[RX_BUFFER_BYTES];
    size_t len = 0;
    if (!takePendingMessage(raw, len)) return false;

    // 安価なクールダウン判定を String 生成・画像 JSON 化・JSON パースより先に行う。
    unsigned long now = millis();
    if (g_lastRxTime != 0 && (now - g_lastRxTime < RX_IGNORE_MS)) {
        return false;
    }

    String json;
    if (raw[0] == 'I') {
        if (len < 1 + MATRIX_RGB_BYTES) return false;
        Content::Data imgData;
        imgData.kind = Content::Kind::Image;
        memcpy(imgData.rgb, raw + 1, MATRIX_RGB_BYTES);
        imgData.durationMs = RX_DISPLAY_HOLD_MS;
        json = Content::serialize(imgData);
        Serial.println("  [RX] received image binary -> mapped to json");
    } else if (raw[0] == 'T') {
        json = String((const char*)(raw + 1), len - 1);
        Serial.println("  [RX] received text: " + json);
    } else {
        json = String((const char*)raw, len);
        Serial.println("  [RX] received unknown: " + json);
    }

    // 空の tune は表示も保存もせず無視する。
    //   tune 未設定のデバイスは名前だけのパケット（{"name":"..."}）を
    //   広告し続けるため、Content::parse に失敗する。これを弾かないと
    //   Inbox に溜まり、流体待機画面にモーション色の余計な粒が増えてしまう。
    Content::Data rxCheck;
    if (!Content::parse(json, rxCheck)) {
        Serial.println("  [RX] empty/invalid tune -> ignored");
        return false;
    }

    // 無効データではクールダウンを開始しない（従来の挙動を維持）。
    g_lastRxTime = now;

    // 同一内容の 5 分リジェクト：直前と同じ内容が RX_SAME_REJECT_MS 以内なら
    //   完全に拒否（表示も保存も返信もしない）。超えていれば改めて受け付ける。
    if (json == g_lastSavedJson
        && g_lastSavedTime != 0
        && (millis() - g_lastSavedTime < RX_SAME_REJECT_MS)) {
        Serial.println("  [RX] same content within 5min -> rejected (no display/save/reply)");
        return false;
    }

    Inbox::append(json);
    g_lastSavedJson = json;
    g_lastSavedTime = millis();

    parsedContent       = rxCheck;
    showContentAfterAnim = true;
    g_needReply          = true;
    return true;
}

// ---------------------------------------------------------
// 初期化（factory reset）：全データと設定を消して既定値に戻す
//   v1.4 までは BOOT 長押し。v1.5r2 では BOOT が押せないので、USB のときのボタン長押しでも呼ぶ。
// ---------------------------------------------------------
void factoryReset() {
    // 紫で2秒点灯
    Display::fillColor(128, 0, 128); // 紫 (R=128, G=0, B=128)
    delay(2000);
    Display::clear();

    // JSONデータをリセット（LittleFS上のファイル内容を空化）
    Storage::clearAll();

    // インメモリの変数も初期化
    MOTION_HUE        = 104; // デフォルト (黄, 色相0–360)
    MOTION_BRIGHTNESS = DEFAULT_MOTION_BRIGHTNESS;
    Display::setBrightness(MOTION_BRIGHTNESS); // HWマトリクスへ即時反映
    MOTION_ANIM       = 0;
    strncpy(DEVICE_NAME, BLE_DEVICE_NAME, DEVICE_NAME_MAX - 1);
    DEVICE_NAME[DEVICE_NAME_MAX - 1] = '\0';
    HOMETOWN[0]       = '\0'; // 出身地もクリア
    g_lastSavedJson   = ""; // 重複保存ガードのキャッシュもクリア
    g_lastSavedTime   = 0;

    // 各種データのインメモリ状態もリロード/初期化
    Content::init();
    Inbox::init();
    Fluid::syncFromInbox();  // 受信件数 0 に合わせて粒を作り直す

    // 接続デバイス名や初期アドバタイズデータを更新・再送信
    BleBroadcast::updateGattName();
    {
        const Content::Data& own = Content::own();
        if (own.kind == Content::Kind::Image) {
            BleBroadcast::setInitialImageRgb(own.rgb, sizeof(own.rgb));
        } else if (own.kind == Content::Kind::Text) {
            BleBroadcast::setInitialData(Content::serialize(own));
        }
    }

    Serial.println("  [Reset] All JSON data and settings reset completed.");
}

// 電源を切る直前（SW2 OFF・電池切れ・長押し）と、充電だけの状態へ移る前に呼ばれる
static void beforePowerOff() {
    Inbox::flush();     // 3 秒待ってから保存する仕組みなので、未保存の分をここで書く
    Display::clear();   // WS2812 は電源が残る限り最後の色を保つので、消してから切る
}

// ---------------------------------------------------------
// setup: 起動時に一度だけ実行される
// ---------------------------------------------------------
void setup() {
    // 電池で起動したときは、ボタンを離してから約 1 秒で電源が切れる。ほかの初期化より先に保持を決める。
    const Power::Mode pm = Power::begin();

    Serial.begin(115200);
    delay(200);
    Serial.println("\n--- turnie_device_v1.5r2 ---");
    Serial.printf("  [Power] %s, USB %s, VBAT %.2f V, VIN %.2f V\n",
                  pm == Power::Mode::Normal ? "normal" : "charge only",
                  Power::usbPresent() ? "yes" : "no", Power::vbat(), Power::vin());

    pinMode(PIN_BTN_BOOT, INPUT_PULLUP);

    Display::init();

    if (pm == Power::Mode::ChargeOnly) {
        // USB あり・SW2 OFF：充電だけ。マトリクスを消し、BLE は起動しない。
        Display::clear();
        return;
    }
    Power::setBeforeOffHook(beforePowerOff);
    if (!Storage::init()) {
        Serial.println("  [Setup] Storage init failed.");
    }
    Content::init();
    Inbox::init();

    // setting.json から前回の設定（hue, brightness, motion, name）をロード
    strncpy(DEVICE_NAME, BLE_DEVICE_NAME, DEVICE_NAME_MAX - 1); // デフォルト名で初期化
    Settings::loadFromStorage(false);

    BleStack::init();
    BleGatt::init();

    BleBroadcast::setOnReceive(onMessageReceived);
    // コンテンツの種類に応じて初期アドバタイズデータを設定
    // 画像はJSONに変換すると664バイトに肥大化してエラーになるため、バイナリで設定する
    {
        const Content::Data& own = Content::own();
        if (own.kind == Content::Kind::Image) {
            BleBroadcast::setInitialImageRgb(own.rgb, sizeof(own.rgb));
        } else if (own.kind == Content::Kind::Text) {
            BleBroadcast::setInitialData(Content::serialize(own));
        }
        // Kind::None の場合はデバイス名のみの初期値が使われる
    }
    BleBroadcast::init();

    // 待機画面の流体シミュレーション（QMI8658 + 受信データの粒）を初期化
    Fluid::init();

    Animations::startMotion();
}

// ---------------------------------------------------------
// loop: 繰り返し実行される
// ---------------------------------------------------------
void loop() {
    // USB あり・SW2 OFF（充電だけ）：表示も BLE も動かさない。SW2 が ON になったら再起動する。
    if (Power::mode() == Power::Mode::ChargeOnly) {
        Power::chargeOnlyLoop();
        return;
    }
    // SW2・USB・電池電圧の監視（SW2 OFF や電池切れなら、ここで保存して電源を切る）
    Power::tick();

    // 受信が落ち着いた後に inbox.json をまとめて永続化する。
    Inbox::tick();

    // BOOT ボタンのハンドリング（v1.5r2 では組み立て後は押せない。モジュール単体で試すとき用に残す）
    static unsigned long bootPressedAt = 0;
    static bool resetTriggered = false;

    if (digitalRead(PIN_BTN_BOOT) == LOW) {
        if (bootPressedAt == 0) {
            bootPressedAt = millis();
            resetTriggered = false;
        } else if (!resetTriggered && (millis() - bootPressedAt >= BOOT_RESET_LONG_PRESS_MS)) {
            // BOOT長押し：すべてのJSONデータをリセット
            resetTriggered = true;
            factoryReset();
        }
    } else {
        if (bootPressedAt != 0) {
            unsigned long pressDuration = millis() - bootPressedAt;
            bootPressedAt = 0;
            // 5秒未満かつ200ms以上の押下で OTA モードへ移行
            if (!resetTriggered && pressDuration > 200 && !isDebugMode()) {
                startDebugMode();
            }
        }
    }

    // USER ボタン（SW1、IO7）のハンドリング
    //   一回押し：通常時はピクチャ確認、閲覧モード中は次の履歴へ
    //   0.8 秒：閲覧モードの開始/終了（DiagonalWave 演出付き。押している最中に発火）
    //   電池のとき 3 秒：電源を切る
    //   USB のとき 5 秒押して離す：OTA モード（緑）／10 秒：初期化（紫）。BOOT ボタンの代わり
    static unsigned long userPressedAt   = 0;
    static bool          userLongFired   = false;
    static bool          userWaitRelease = true;   // 電源を入れたときの押下は、離すまで操作として扱わない
    static bool          otaArmed        = false;
    static bool          resetFired      = false;

    const bool userDown = (digitalRead(PIN_BTN_USER) == LOW);
    if (userWaitRelease) {
        if (!userDown) userWaitRelease = false;
    } else if (!isDebugMode()) {
        if (userDown) {
            if (userPressedAt == 0) {
                userPressedAt = millis();
                userLongFired = false;
                otaArmed      = false;
                resetFired    = false;
                Power::noteActivity();
            }
            const unsigned long held = millis() - userPressedAt;
            if (!userLongFired && held >= BROWSE_LONG_PRESS_MS) {
                // 長押し確定（押している最中に1回だけ発火）
                userLongFired = true;
                if (!g_browseMode) {
                    enterBrowseMode();
                } else {
                    exitBrowseMode();
                }
            }
            if (!Power::usbPresent()) {
                if (held >= POWER_OFF_HOLD_MS) {
                    Display::fillColor(60, 0, 0);   // 暗い赤で知らせてから切る
                    delay(400);
                    Power::powerOff("button hold");  // 離すのを待ってから眠る
                }
            } else {
                if (!resetFired && held >= FACTORY_RESET_HOLD_MS) {
                    resetFired    = true;
                    g_browseMode  = false;
                    g_previewMode = false;
                    factoryReset();
                } else if (!resetFired && held >= OTA_HOLD_MS) {
                    // ここで離すと OTA モード。押している間は緑で知らせる（流体の描画で上書きされないよう、ここで返す）
                    otaArmed      = true;
                    g_browseMode  = false;
                    g_previewMode = false;
                    Display::fillColor(0, 80, 0);
                    delay(LOOP_TICK_MS);
                    return;
                }
            }
        } else {
            if (userPressedAt != 0) {
                unsigned long pressDuration = millis() - userPressedAt;
                userPressedAt = 0;
                if (resetFired) {
                    // 初期化は済んでいる
                } else if (otaArmed) {
                    Display::clear();
                    startDebugMode();
                } else if (!userLongFired && pressDuration > BUTTON_DEBOUNCE_MS) {
                    // 長押しでなく、チャタリング閾値を超えた離しなら「一回押し」
                    if (g_browseMode) {
                        browseNext();          // 閲覧モード中：次の履歴へ
                    } else {
                        togglePreview();        // 通常時：ピクチャ確認 ↔ 流体
                    }
                }
                userLongFired = false;
                otaArmed      = false;
                resetFired    = false;
            }
        }
    }

    // OTA モード中は通常処理をスキップ
    if (isDebugMode()) {
        handleDebugMode();
        return;
    }

    // 閲覧モード中：表示更新だけ行い、通常の受信/自分のコンテンツ表示は止める。
    //   無操作が続いたら自動的に通常フローへ戻る。
    if (g_browseMode) {
        if (millis() - g_browseLastActive >= BROWSE_TIMEOUT_MS) {
            exitBrowseMode();
        } else {
            Display::tick();
        }
        delay(LOOP_TICK_MS);
        return;
    }

    // アニメーション終了直後：受信コンテンツを表示する
    if (showContentAfterAnim) {
        showContentAfterAnim = false;
        showContent(parsedContent, true); // 受信データとして表示（アニメーション有効）
    }

    // 受信メッセージを処理する（全てメインタスクで実行）
    if (processPendingMessage()) {
        Power::noteActivity();
        g_previewMode = false;    // 受信コンテンツを優先（ピクチャ確認中なら解除）
        Fluid::syncFromInbox();   // 受信件数が増えた → 粒の数と色を更新
        // 受信したら即座に返信（isShowing() を待たない）
        if (g_needReply) {
            g_needReply = false;
            Content::Data ownData = Content::own();
            if (ownData.kind == Content::Kind::Text) {
                BleBroadcast::send(Content::serialize(ownData));
            } else if (ownData.kind == Content::Kind::Image) {
                BleBroadcast::sendImageRgb(ownData.rgb, sizeof(ownData.rgb));
            }
            Serial.println("  [TX] Sent own content back to partner");
        }
        Animations::startRipple();
        delay(LOOP_TICK_MS);
        return;
    }

    Display::tick();
    BleGatt::tick();

    // ピクチャ確認モード：自分のピクチャを表示し、10秒で流体へ自動復帰。
    //   （もう一度短押しした場合は togglePreview() 側で即座に解除される）
    if (g_previewMode) {
        if (millis() - g_previewStartedAt >= PREVIEW_TIMEOUT_MS) {
            g_previewMode = false;
            Display::clear();
            // 下の流体表示へフォールスルー
        } else {
            delay(LOOP_TICK_MS);
            return;   // ピクチャ表示中（Display が保持）
        }
    }

    // 待機画面：流体シミュレーションを表示する。
    //   受信コンテンツ等を表示中（isShowing）はそれを優先し、流体は描かない。
    if (!Display::isShowing()) {
        Fluid::tick();
    }

    delay(LOOP_TICK_MS);
}
