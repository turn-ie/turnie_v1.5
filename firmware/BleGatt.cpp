#include "BleGatt.h"
#include "BleBroadcast.h"
#include "Config.h"
#include "Content.h"
#include "Display.h"
#include "Storage.h"
#include "Animations.h"
#include "Settings.h"
#include "Fluid.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>

// ============================================================
// 内部状態
// ============================================================
static BLECharacteristic* g_pTxChar     = nullptr;
static constexpr size_t GATT_RX_BUFFER_BYTES = 2048;
static char               g_pendingData[GATT_RX_BUFFER_BYTES + 1];
static volatile size_t    g_pendingLength = 0;
static volatile bool      g_pendingReady  = false;
static volatile unsigned long g_pendingLastWriteAt = 0;
static portMUX_TYPE       g_pendingMux = portMUX_INITIALIZER_UNLOCKED;
static constexpr unsigned long GATT_RX_TIMEOUT_MS = 3000;
static constexpr unsigned long PHONE_WRITE_PREVIEW_MS = 3000;
static constexpr size_t GATT_NOTIFY_CHUNK_BYTES = 20;

// スマホ探索用 GATT 広告（handle 1）の自己回復状態。
// 切断を検知した場合だけ再開し、失敗時は間隔を空けて再試行する。
static volatile bool      g_phoneConnected = false;
static volatile bool      g_advRestartNeeded = false;
static constexpr unsigned long GATT_ADV_WATCHDOG_MS = 3000;

static bool notifyText(const String& message) {
    if (g_pTxChar == nullptr || message.isEmpty()) return false;

    for (size_t offset = 0; offset < message.length(); offset += GATT_NOTIFY_CHUNK_BYTES) {
        String chunk = message.substring(offset, offset + GATT_NOTIFY_CHUNK_BYTES);
        g_pTxChar->setValue(chunk.c_str());
        g_pTxChar->notify();
        delay(10);
    }
    return true;
}

static int brightnessPercentFromHardware(uint8_t brightness) {
    return ((int)brightness * 100 + 127) / 255;
}

static String buildSettingsJson() {
    String settingsJson = Storage::loadJson(SD_SETTING_JSON);
    if (!settingsJson.isEmpty()) {
        JsonDocument doc;
        if (deserializeJson(doc, settingsJson) == DeserializationError::Ok) {
            return settingsJson;
        }
        Serial.println("  [BleGatt] setting.json is invalid, sending current settings");
    }

    JsonDocument doc;
    doc["flag"]       = "settings";
    doc["hue"]        = MOTION_HUE;
    doc["brightness"] = brightnessPercentFromHardware(MOTION_BRIGHTNESS);
    doc["motion"]     = Animations::motionEnumToString(MOTION_ANIM);
    doc["name"]       = DEVICE_NAME[0] ? DEVICE_NAME : BLE_DEVICE_NAME;
    doc["hometown"]   = HOMETOWN;

    String out;
    serializeJson(doc, out);
    return out;
}

static bool sendSettingsToPhone() {
    String settingsJson = buildSettingsJson();
    Serial.println("  [BleGatt] sending settings: " + settingsJson);
    return notifyText(settingsJson);
}

// ============================================================
// 受信コールバック
//   GATT で送られてきた JSON データを受け取る
// ============================================================
class WriteCallback : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pChar) override {
        std::string received = pChar->getValue();
        if (received.empty()) return;

        portENTER_CRITICAL(&g_pendingMux);
        if (!g_pendingReady) {
            unsigned long now = millis();
            if (g_pendingLength > 0
                && now - g_pendingLastWriteAt >= GATT_RX_TIMEOUT_MS) {
                g_pendingLength = 0;
                g_pendingLastWriteAt = 0;
            }
            // 新しい蓄積は必ず '{' から始める。不正な先頭断片は即座に破棄する。
            if (g_pendingLength == 0 && received[0] != '{') {
                g_pendingLastWriteAt = 0;
            } else if (g_pendingLength > 0 && g_pendingData[0] != '{') {
                g_pendingLength = 0;
                g_pendingLastWriteAt = 0;
            } else if (received.size() > GATT_RX_BUFFER_BYTES - g_pendingLength) {
                // 上限超過した未完成データは、以後の受信を塞がないよう破棄する。
                g_pendingLength = 0;
                g_pendingLastWriteAt = 0;
            } else {
                memcpy(g_pendingData + g_pendingLength, received.data(), received.size());
                g_pendingLength += received.size();
                g_pendingData[g_pendingLength] = '\0';
                g_pendingLastWriteAt = now;
                if (g_pendingData[g_pendingLength - 1] == '}') {
                    g_pendingReady = true;
                }
            }
        }
        portEXIT_CRITICAL(&g_pendingMux);
    }
};

// ============================================================
// 接続コールバック
// ============================================================
class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        g_phoneConnected = true;
        g_advRestartNeeded = false;
        Serial.println("  [BleGatt] phone connected");
    }

    void onDisconnect(BLEServer* pServer) override {
        // コールバックでは状態だけを記録し、広告の再開は loop() 側で行う。
        g_phoneConnected = false;
        g_advRestartNeeded = true;
        Serial.println("  [BleGatt] phone disconnected");
    }
};

// ============================================================
// Public API
// ============================================================
namespace BleGatt {

bool init() {
    BLEServer* server = BLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());

    BLEService* service = server->createService(GATT_SVC_UUID);

    BLECharacteristic* rxChar = service->createCharacteristic(
        GATT_RX_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    rxChar->setCallbacks(new WriteCallback());

    g_pTxChar = service->createCharacteristic(
        GATT_TX_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    g_pTxChar->addDescriptor(new BLE2902());

    service->start();

    // GATT 広告は BleBroadcast の handle 1（legacy 互換 connectable）で発信される。
    // ここでは GATT サービスを登録するだけ。
    Serial.println("  [BleGatt] service ready");
    return true;
}

void tick() {
    // --- GATT 広告ウォッチドッグ ---
    // 切断で広告停止が見込まれる場合だけ再開を要求する。失敗時のみ間隔を空けて再試行する。
    {
        static unsigned long lastAdvAttempt = 0;
        unsigned long now = millis();
        if (!g_phoneConnected && g_advRestartNeeded
            && now - lastAdvAttempt >= GATT_ADV_WATCHDOG_MS) {
            lastAdvAttempt = now;
            if (BleBroadcast::restartGattAdv()) {
                g_advRestartNeeded = false;
            }
        }
    }

    // 未完成の断片が一定時間更新されなければ破棄し、次の JSON を受信可能に戻す。
    bool timedOut = false;
    bool ready = false;
    static char processingData[GATT_RX_BUFFER_BYTES + 1];
    size_t processingLength = 0;
    unsigned long now = millis();

    portENTER_CRITICAL(&g_pendingMux);
    if (!g_pendingReady && g_pendingLength > 0
        && now - g_pendingLastWriteAt >= GATT_RX_TIMEOUT_MS) {
        g_pendingLength = 0;
        g_pendingLastWriteAt = 0;
        timedOut = true;
    }
    if (g_pendingReady) {
        processingLength = g_pendingLength;
        memcpy(processingData, g_pendingData, processingLength);
        processingData[processingLength] = '\0';
        g_pendingLength = 0;
        g_pendingLastWriteAt = 0;
        g_pendingReady = false;
        ready = true;
    }
    portEXIT_CRITICAL(&g_pendingMux);

    if (timedOut) {
        Serial.println("  [BleGatt] incomplete JSON timed out and was discarded");
    }
    if (!ready) return;

    String data(processingData, processingLength);
    Serial.println("  [BleGatt] JSON received: " + data);

    // ----------------------------------------------------------
    // flag: "settings" の場合は設定変数のみを更新して終わり
    // ----------------------------------------------------------
    {
        JsonDocument settingsDoc;
        if (deserializeJson(settingsDoc, data) == DeserializationError::Ok) {
            String flag = settingsDoc["flag"] | "";
            if (flag == "get_settings") {
                sendSettingsToPhone();
                return;
            }
            if (flag == "settings") {
                // スマホから送られてきた settings の内容を丸ごと SD (setting.json) に保存する。
                // hue / brightness / name / hometown など、含まれる項目をそのまま保持する。
                {
                    String settingsJson;
                    serializeJson(settingsDoc, settingsJson);
                    if (Storage::saveJson(SD_SETTING_JSON, settingsJson)) {
                        Serial.println("  [BleGatt] saved settings to SD: " + settingsJson);
                        // ripple の前に設定を反映 → 新しい設定色で ripple が再生される
                        Settings::loadFromStorage(true);
                        Animations::startRipple();
                        Display::clear();
                        notifyText("saved");
                    }
                }

                return; // settings 処理完了。コンテンツは保存しない
            }
        }
    }

    // コンテンツをパースして保存
    Content::Data content;
    if (!Content::parse(data, content)) return;

    // 自分のコンテンツとして更新（メモリ + SD）
    Content::saveOwn(content);
    Serial.println("  [BleGatt] saved as own content");
    notifyText("saved");

    // 待機（流体）の自分の粒(index 0)の色を更新（戻ったとき古い色にならないように）
    Fluid::syncFromInbox();

    // プレビュー：画像・文字とも PHONE_WRITE_PREVIEW_MS だけ表示し、自動で待機へ戻る
    if (content.kind == Content::Kind::Text) {
        Display::showText(content.text.c_str(), TEXT_SCROLL_DELAY_MS, true,
                          0xFFFF, PHONE_WRITE_PREVIEW_MS);
    } else if (content.kind == Content::Kind::Image) {
        Display::showImage(content.rgb, sizeof(content.rgb), PHONE_WRITE_PREVIEW_MS, false);
    }

    // 他のデバイスへブロードキャストする
    if (content.kind == Content::Kind::Image) {
        BleBroadcast::sendImageRgb(content.rgb, sizeof(content.rgb));
    } else {
        String json = Content::serialize(content);
        Serial.println("  [TX] " + json);
        BleBroadcast::send(json);
    }
}

} // namespace BleGatt
