#include <wiring.h>          // workaround for https://github.com/chipKIT32/chipKIT-core/pull/324
#include <Board_Defs.h>
#include <HardwareSerial.h>  // for Serial

#include <cobs/Print.h>
#include <cobs/Stream.h>
#include <PacketListener.h>
#include <pb_arduino.h>  // for as_pb_ostream

#include <messaging.h>

#include "nanopb_helpers.h"
#include "dispatch_impl.h"

template <typename T> typename messageHandlers<T>::type messageHandlers<T>::handler;

namespace {
    //! our cobs packetizer
    packetio::COBSPrint cobs_out(Serial);
    packetio::COBSStream cobs_in(Serial);

    packetio::PacketListener listener(cobs_in);

    //! Send a message object over serial, using protobuf and cobs
    void sendMessage(RobotMessage& message) {
        // Create stream
        pb_ostream_t pb_stream = as_pb_ostream(cobs_out);

        // serialize the message
        bool status = pb_encode(&pb_stream, RobotMessage_fields, &message);

        /* Then just check for any errors.. */
        if (!status)
        {
            printf("Encoding failed: %s\n", PB_GET_ERROR(&pb_stream));
            cobs_out.abort();
        }
        else{
            // finish the packet
            cobs_out.end();
        }

        Serial.flush();
    }

    template<typename T>
    inline bool try_handlers(const PCMessage &message) {
        if (message.which_msg == field_info<T>::tag) {
            auto handler = messageHandlers<T>::handler;
            if (handler) {
                handler(message.msg.*(field_info<T>::value));
            }
            return true;
        }
        else {
            return false;
        }
    }

    //! base packet listener
    void handlePacket(uint8_t* data, size_t n) {
        pb_istream_t pb_stream = pb_istream_from_buffer(data, n);

        PCMessage message = PCMessage_init_zero;
        bool status = pb_decode(&pb_stream, PCMessage_fields, &message);

        if(!status) {
            logging::error("Message was corrupt");
            logging::error(reinterpret_cast<char*>(data), n);
            return;
        }

        // dispatch to the appropriate handler
        if(!try_handlers(message))
        {
            logging::error("Message type unknown");
            logging::error(reinterpret_cast<char*>(data), n);
        }
    }

    using PacketError = packetio::PacketListener::Error;

    void handleError(uint8_t* data, size_t n, PacketError e) {
        if(e == PacketError::Overflow)
            logging::error("Overflow error");
        else if(e == PacketError::Framing)
            logging::error("Framing error");
        else
            logging::error("Unknown error");

        logging::error(reinterpret_cast<char*>(data), n);
    }

}

//! do any setup required for messaging
void setupMessaging() {
    Serial.begin(57600);
    listener.onMessage(handlePacket);
    listener.onError(handleError);
}

void updateMessaging() {
    listener.update();
}

namespace logging {
    //! send a debug string
    void log(DebugLevel level, const char* text, size_t n) {
        nanopb_helpers::array_handle<const char> arr = {text, n};

        // fill out the message
        RobotMessage message = RobotMessage_init_zero;
        message.which_msg = RobotMessage_debug_tag;
        message.msg.debug.s.funcs.encode = nanopb_helpers::write_string;
        message.msg.debug.s.arg = &arr;
        message.msg.debug.level = level;

        sendMessage(message);
    }
    void log(DebugLevel level, const char* text) {
        log(level, text, strlen(text));
    }
}

//! send log messages
void sendLogBundle(const LogEntry* entries, size_t n) {
    nanopb_helpers::array_handle<const LogEntry> arr = {entries, n};

    // fill out the message
    RobotMessage message = RobotMessage_init_zero;
    message.which_msg = RobotMessage_log_bundle_tag;
    message.msg.log_bundle.entry.funcs.encode
        = &nanopb_helpers::write_array<const LogEntry, LogEntry_fields>;
    message.msg.log_bundle.entry.arg = &arr;

    sendMessage(message);
}

//! send log messages
void sendLog(const LogEntry& entry) {

    // fill out the message
    RobotMessage message = RobotMessage_init_zero;
    message.which_msg = RobotMessage_single_log_tag;
    message.msg.single_log = entry;
    sendMessage(message);
}
