#include "Power.h"
#include "Config.h"
#include <driver/gpio.h>
#include <driver/rtc_io.h>
#include <esp_sleep.h>
#include <esp_system.h>

namespace {

Power::Mode   g_mode        = Power::Mode::Normal;
bool          g_usb         = false;
float         g_vbat        = 0.0f;
float         g_vin         = 0.0f;
unsigned long g_lastSw2Poll = 0;
unsigned long g_lastAdcPoll = 0;
int           g_sw2OffCount = 0;
int           g_sw2OnCount  = 0;
int           g_lowCount    = 0;
unsigned long g_lastActive  = 0;
void (*g_beforeOff)()       = nullptr;

bool sw2On() { return digitalRead(PIN_SW_SENSE) == LOW; }
bool buttonDown() { return digitalRead(PIN_BTN_USER) == LOW; }

void setHold(bool on) {
    gpio_hold_dis((gpio_num_t)PIN_PWR_HOLD);
    pinMode(PIN_PWR_HOLD, OUTPUT);
    digitalWrite(PIN_PWR_HOLD, on ? HIGH : LOW);
}

// IO1・IO5 はどちらも 100k/100k の分圧なので、読み値の 2 倍が元の電圧。
void readAdc() {
    uint32_t mvB = 0, mvV = 0;
    for (int i = 0; i < 4; i++) {
        mvB += analogReadMilliVolts(PIN_VBAT_SENSE);
        mvV += analogReadMilliVolts(PIN_USB_SENSE);
    }
    g_vbat = (mvB / 4) * 2 / 1000.0f;
    g_vin  = (mvV / 4) * 2 / 1000.0f;
    // USB あり：VIN ≒ 4.55〜4.6 V（モジュール上のダイオードの分）、電池だけ：VIN ≒ 電池 − 0.3 V（D3 の分）。
    // ラッチが切れていると IO1 はほぼ 0 V になるが、この判定はそのまま正しく働く。
    g_usb = g_vin > g_vbat + USB_DETECT_MARGIN_V;
}

}  // namespace

namespace Power {

Mode begin() {
    // ディープスリープ中に L で固定していた IO2 を解放してから、すぐに保持するかを決める。
    gpio_hold_dis((gpio_num_t)PIN_PWR_HOLD);
    gpio_deep_sleep_hold_dis();
    pinMode(PIN_SW_SENSE, INPUT_PULLUP);
    pinMode(PIN_BTN_USER, INPUT_PULLUP);
    delayMicroseconds(300);  // プルアップで電圧が上がるのを待つ

    const bool on = sw2On();
    const esp_reset_reason_t rr = esp_reset_reason();
    const bool brownout = (rr == ESP_RST_BROWNOUT);

    // 電圧低下でのリセットのときは、ここで保持しない。電源を切る途中で再起動して
    // 保持し直してしまうと、電源が切れなくなるため（回路図確認 3 章 1）。
    setHold(on && !brownout);

    analogSetPinAttenuation(PIN_VBAT_SENSE, ADC_11db);
    analogSetPinAttenuation(PIN_USB_SENSE, ADC_11db);
    readAdc();

    if (on) {
        if (brownout && !g_usb) powerOff("brownout reset on battery");
        if (brownout) setHold(true);  // USB があるときは通常どおり動かす
        g_mode = Mode::Normal;
    } else if (g_usb) {
        // USB あり・SW2 OFF：充電だけ。IO2 は L のまま（H にすると USB を抜いても電池で動き続ける）。
        g_mode = Mode::ChargeOnly;
    } else {
        // SW2 OFF で USB もない：C4 の残りで一瞬動いているだけなので、そのまま切る。
        powerOff("sw2 off without usb");
    }
    g_lastActive = millis();
    return g_mode;
}

Mode mode() { return g_mode; }
bool usbPresent() { return g_usb; }
float vbat() { return g_vbat; }
float vin() { return g_vin; }
void noteActivity() { g_lastActive = millis(); }
void setBeforeOffHook(void (*fn)()) { g_beforeOff = fn; }

void tick() {
    if (g_mode != Mode::Normal) return;
    const unsigned long now = millis();

    // SW2：100 ms ごとに読み、2 回続けて OFF なら確定。
    if (now - g_lastSw2Poll >= 100) {
        g_lastSw2Poll = now;
        g_sw2OffCount = sw2On() ? 0 : g_sw2OffCount + 1;
        if (g_sw2OffCount >= 2) {
            readAdc();
            if (g_usb) {
                // USB で動いていて SW2 を OFF にした：充電だけの状態へ。IO2 を L にして再起動する。
                Serial.println("  [Power] SW2 OFF on USB -> charge only (restart)");
                if (g_beforeOff) g_beforeOff();
                setHold(false);
                Serial.flush();
                delay(20);
                esp_restart();
            }
            // 電池で動いていて SW2 を OFF にした：ラッチの戻り線が切れ、数秒以内に電源が落ちる。先に保存して切る。
            powerOff("sw2 off on battery");
        }
    }

    // 電池電圧と USB の有無：1 秒ごと。
    if (now - g_lastAdcPoll >= 1000) {
        g_lastAdcPoll = now;
        readAdc();
        if (!g_usb && g_vbat < LOW_BATT_OFF_V) {
            if (++g_lowCount >= LOW_BATT_CONFIRM_COUNT) powerOff("low battery");
        } else {
            g_lowCount = 0;
        }
        if (AUTO_OFF_MS > 0 && !g_usb && now - g_lastActive >= AUTO_OFF_MS) {
            powerOff("auto off (no activity)");
        }
    }
}

void chargeOnlyLoop() {
    // 表示も BLE も使わない。ディープスリープに入らず起きたままにしておくのは、
    // モジュールの BOOT ボタンが基板側を向いていて押せないため。起きていれば USB から書き込める。
    const unsigned long now = millis();
    if (now - g_lastSw2Poll >= 100) {
        g_lastSw2Poll = now;
        g_sw2OnCount = sw2On() ? g_sw2OnCount + 1 : 0;
        if (g_sw2OnCount >= 2) {
            // SW2 を ON にすると C4 を通してラッチが入る。再起動して Normal で IO2 を H にする。
            Serial.println("  [Power] SW2 ON -> restart into normal mode");
            Serial.flush();
            delay(20);
            esp_restart();
        }
    }
    delay(20);
}

void powerOff(const char* reason) {
    Serial.printf("  [Power] off: %s (VBAT %.2f V, VIN %.2f V)\n", reason, g_vbat, g_vin);
    if (g_beforeOff) g_beforeOff();
    Serial.flush();

    setHold(false);
    gpio_hold_en((gpio_num_t)PIN_PWR_HOLD);
    gpio_deep_sleep_hold_en();

    const bool swOn = sw2On();
    gpio_num_t wakePin;
    if (swOn) {
        // ボタンを押している間は D4 を通してラッチが保たれる。離すのを待ってから眠る（押したまま眠るとすぐ目覚めるため）。
        const unsigned long t0 = millis();
        while (buttonDown() && millis() - t0 < 10000) delay(10);
        delay(50);
        wakePin = (gpio_num_t)PIN_BTN_USER;
    } else {
        // SW2 OFF ではボタンが読めない（戻り線が SW2 を通る）。SW2 が ON に戻ったら目覚める。
        wakePin = (gpio_num_t)PIN_SW_SENSE;
    }
    rtc_gpio_pullup_en(wakePin);
    rtc_gpio_pulldown_dis(wakePin);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    esp_sleep_enable_ext0_wakeup(wakePin, 0);
    // ふつうは、ここで眠っている間に C4 が放電してラッチが外れ、電源そのものが切れる。
    esp_deep_sleep_start();
}

}  // namespace Power
