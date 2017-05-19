#pragma once

#include <PacketListener.h>
#include <messages.pb.h>

void setupMessaging();
void updateMessaging();

namespace logging {
	void log(DebugLevel level, const char* text);
	void log(DebugLevel level, const char* text, size_t n);

    inline void debug(const char* text)           { log(DebugLevel_DEBUG, text); }
    inline void debug(const char* text, size_t n) { log(DebugLevel_DEBUG, text, n); }
    inline void info(const char* text)            { log(DebugLevel_INFO,  text); }
    inline void info(const char* text, size_t n)  { log(DebugLevel_INFO,  text, n); }
    inline void warn(const char* text)            { log(DebugLevel_WARN,  text); }
    inline void warn(const char* text, size_t n)  { log(DebugLevel_WARN,  text, n); }
    inline void error(const char* text)           { log(DebugLevel_ERROR, text); }
    inline void error(const char* text, size_t n) { log(DebugLevel_ERROR, text, n); }
}

void sendLogBundle(const LogEntry* entries, size_t n);
void sendLog(const LogEntry& entry);

//! stores a handler for each message type.
template<typename T>
struct messageHandlers {
    typedef packetio::LambdaPointer<void (const T&)> type;
    static type handler;
};

//! Attach a listener for a given message type
template<typename T>
void onMessage(typename messageHandlers<T>::type handler) {
    // if this gives an undefined reference, then you haven't followed the
    // instructions in dispatch.h
    messageHandlers<T>::handler = handler;
}
