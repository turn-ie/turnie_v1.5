#pragma once
#include <Arduino.h>

// ============================================================
// v1.5r2 シールド基板の電源制御
//   IO2 = 電源の保持（H で保持）、IO4 = SW2（L = ON）、
//   IO5 = VIN / 2、IO1 = 電池電圧（VBAT_SW）/ 2、IO7 = ボタン。
//   動きは turnie_v1.5 の作業記録 12.4 節の表に従う。
// ============================================================
namespace Power {

enum class Mode : uint8_t {
    Normal,      // SW2 ON。電池でも USB でも通常どおり動く（IO2 = H）
    ChargeOnly,  // USB あり・SW2 OFF。表示も BLE も使わず、USB の書き込みだけ受け付ける（IO2 = L）
};

// setup() の最初に呼ぶ。電池で起動したときは、ボタンを離してから約 1 秒以内に
// IO2 を H にしないと電源が切れるので、ほかの初期化より先に保持を決める。
// 電源を保持しない場合（電圧低下でのリセット、SW2 OFF で USB なし）はここで powerOff() に入り、戻らない。
Mode begin();

Mode mode();

// loop() の最初に毎回呼ぶ。SW2 と USB と電池電圧を見て、必要なら再起動や電源断を行う。
void tick();

// ChargeOnly のときの loop() の中身。SW2 が ON になったら再起動して Normal に入る。
void chargeOnlyLoop();

bool  usbPresent();   // 最後に読んだ値
float vbat();         // 電池電圧 [V]（ラッチが切れているときはほぼ 0）
float vin();          // VIN [V]

// 操作や受信があったことを知らせる（AUTO_OFF_MS が 0 でなければ、自動で切るまでの時間を延ばす）
void noteActivity();

// 電源を切る前に呼ぶ処理（受信履歴の保存など）を登録する
void setBeforeOffHook(void (*fn)());

// 電源を切る。IO2 を L にして、ディープスリープに入る（戻らない）。
//   SW2 が ON のまま切るとき：ボタンを離すのを待ってから眠る。電源が切れずに残った場合はボタンで目覚める。
//   SW2 が OFF で切るとき　：SW2 が ON に戻ったら目覚める。
[[noreturn]] void powerOff(const char* reason);

}  // namespace Power
