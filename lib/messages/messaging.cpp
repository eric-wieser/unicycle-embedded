#include <wiring.h>          // workaround for https://github.com/chipKIT32/chipKIT-core/pull/324
#include <Board_Defs.h>
#include <HardwareSerial.h>  // for Serial

#include <cobs/Print.h>
#include <cobs/Stream.h>
#include <PacketListener.h>
#include <pb_arduino.h>  // for as_pb_ostream

#include <messaging.h>


template <typename T> typename messageHandlers<T>::type messageHandlers<T>::handler;


template struct messageHandlers<Go>;
template struct messageHandlers<Stop>;
template struct messageHandlers<Controller>;
template struct messageHandlers<GetLogs>;
template struct messageHandlers<CalibrateGyro>;

namespace {
    //! our cobs packetizer
    packetio::COBSPrint cobs_out(Serial);
    packetio::COBSStream cobs_in(Serial);

    packetio::PacketListener listener(cobs_in);

    //! simple wrapper type to pass array length into write_array
    template <typename T>
    struct array_handle {
        T* ptr;
        size_t len;
    };

    //! nanopb callback for writing a string
    bool write_string(pb_ostream_t *stream, const pb_field_t *field, void * const *arg)
    {
        auto argval = *arg;
        auto handle = *reinterpret_cast<array_handle<const uint8_t>*>(argval);

        if (!pb_encode_tag_for_field(stream, field))
            return false;

        return pb_encode_string(stream, handle.ptr, handle.len);
    }

    //! nanopb callback for writing an array
    template<typename T, const pb_field_t* fields>
    bool write_array(pb_ostream_t * stream, const pb_field_t *field, void * const *arg)
    {
        auto argval = *arg;
        auto handle = *reinterpret_cast<array_handle<T>*>(argval);

        for(size_t i = 0; i < handle.len; i++) {
            if (!pb_encode_tag_for_field(stream, field))
                return false;
            T* curr = handle.ptr + i;
            if (!pb_encode_submessage(stream, fields, curr))
                return false;
        }
        return true;
    }

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

    template<typename T, typename U>
    inline void run_handler(T t, U arg) {
        if (t) {
            t(arg);
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
        switch(message.which_msg) {
            case PCMessage_controller_tag:
                run_handler(
                    messageHandlers<Controller>::handler,
                    message.msg.controller);
                return;
            case PCMessage_go_tag:
                run_handler(
                    messageHandlers<Go>::handler,
                    message.msg.go);
                return;
            case PCMessage_stop_tag:
                run_handler(
                    messageHandlers<Stop>::handler,
                    message.msg.stop);
                return;
            case PCMessage_get_logs_tag:
                run_handler(
                    messageHandlers<GetLogs>::handler,
                    message.msg.get_logs);
                return;
            case PCMessage_calibrate_tag:
                run_handler(
                    messageHandlers<CalibrateGyro>::handler,
                    message.msg.calibrate);
                return;
            default:
                logging::error("Message type unknown");
                logging::error(reinterpret_cast<char*>(data), n);
                return;
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
        array_handle<const char> arr = {text, n};

        // fill out the message
        RobotMessage message = RobotMessage_init_zero;
        message.which_msg = RobotMessage_debug_tag;
        message.msg.debug.s.funcs.encode = write_string;
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
    array_handle<const LogEntry> arr = {entries, n};

    // fill out the message
    RobotMessage message = RobotMessage_init_zero;
    message.which_msg = RobotMessage_log_bundle_tag;
    message.msg.log_bundle.entry.funcs.encode = &write_array<const LogEntry, LogEntry_fields>;
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
