#pragma once

#include <PacketListener.h>
#include <messages.pb.h>

void setupMessaging();
void updateMessaging();
void debug(const char* text);
void debug(const char* text, size_t n);
void sendLogBundle(const LogEntry* entries, size_t n);
void sendLog(const LogEntry& entry);

//! stores a handler for each message type.
template<typename T>
struct messageHandlers {
    typedef packetio::LambdaPointer<void (const T&)> type;
    static type handler;
};

//! Attach a listener for a given message type
template<typename T> void onMessage(typename messageHandlers<T>::type handler) {
    messageHandlers<T>::handler = handler;
}
