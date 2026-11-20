#pragma once
#include <Arduino.h>

// =============================================================
// Inbox
//   受信したメッセージを RAM 上のリングバッファに保存する。
//   古いものから順に INBOX_CAPACITY 件まで保持し、
//   それを超えると最も古いものから上書きする。
//   更新が数秒落ち着いた後、SD の JSON ファイルへまとめて保存する。
// =============================================================
namespace Inbox {

    // 1 件のメッセージ
    struct Item {
        unsigned long receivedAt;  // 受信時刻（millis()）
        String        json;        // 受信した JSON 文字列
    };

    // SD から保存されたメッセージをロードする。
    bool init();

    // メッセージを RAM に追加し、遅延保存を予約する。
    void append(const String& json);

    // loop() から呼び、デバウンス期間を過ぎた変更を SD に保存する。
    void tick();
    // 未保存の受信履歴をすぐに保存する（電源を切る前に呼ぶ）
    void flush();

    // 現在保存されているメッセージの件数
    int size();

    // index 番目のメッセージを取得する（0 = 最も古い、size()-1 = 最新）
    // 取得できた場合は true を返す
    bool get(int index, Item& out);

    // 全メッセージを削除する
    void clear();

}
