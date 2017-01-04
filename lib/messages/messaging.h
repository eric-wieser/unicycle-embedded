#pragma once

#include <PacketListener.h>
#include <messages.pb.h>

void setupMessaging();
void updateMessaging();
void debug(const char* text);
void debug(const char* text, size_t n);
void sendLogs(const LogEntry* entries, size_t n);

//! stores a handler for each message type.
template<typename T>
struct messageHandlers {
    typedef packetio::LambdaPointer<void (T&)> type;
    static type handler;
};

template struct messageHandlers<Go>;
template struct messageHandlers<Stop>;
template struct messageHandlers<SetController>;

//! Attach a listener for a given message type
template<typename T> void onMessage(typename messageHandlers<T>::type handler) {
    messageHandlers<T>::handler = handler;
}
