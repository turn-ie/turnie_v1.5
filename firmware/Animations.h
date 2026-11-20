#pragma once
#include <Arduino.h>

// =============================================================
// Animations
//   8x8 マトリクス用の演出アニメーション。
//   Display::Internal::matrix() で直接ピクセルを操作する。
//   Ripple / DiagonalWave は start*() 内で完了するブロッキング実装。
// =============================================================
namespace Animations {

    // MOTION_ANIM の値に従ってアニメーションを選択して開始する
    //   0 = Ripple / 1 = DiagonalWave
    void startMotion();

    // 受信演出：中心から外へ広がるリップル（ブロッキング）
    void startRipple();

    // すれ違い演出：左右端から中央へ収束する波（ブロッキング）
    void startDiagonalWave();

    // モーション名文字列を内部のモーションID (0=Ripple, 1=DiagonalWave) に変換する
    uint8_t motionStringToEnum(const String& motionStr);

    // 内部のモーションIDをモーション名文字列に変換する
    String motionEnumToString(uint8_t animType);

    // 現在の MOTION_HUE をマトリクス用の 16bit カラーで返す（リップル等と同じ色味）
    uint16_t motionHueColor();

}
