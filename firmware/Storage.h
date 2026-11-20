#pragma once
#include <Arduino.h>

// =============================================================
// Storage
//   内部フラッシュ（LittleFS）への JSON 読み書き。
//   上位レイヤー（Content / Inbox）はこのモジュール経由で
//   ファイルシステムにアクセスする。
// =============================================================
namespace Storage {

    // LittleFS を初期化する。失敗時は false を返す。
    bool init();

    // path のファイルを JSON 文字列として返す。失敗時は空文字列。
    String loadJson(const char* path);

    // json を path に上書き保存する。成功時 true。
    bool saveJson(const char* path, const String& json);

    // 全ての保存ファイルの内容を空にする
    void clearAll();

}
