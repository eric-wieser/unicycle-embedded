#include <PacketSerial.h>
#include <cobs/Print.h>

#include <messages.pb.h>
#include <pb_arduino.h>

//! our cobs packetizer
packetio::COBSPrint cobs_serial(Serial);

//! nanopb callback for writing a string
static bool write_string(pb_ostream_t *stream, const pb_field_t *field, void * const *arg)
{
    auto argval = *arg;
    auto str = *reinterpret_cast<const char**>(argval);
    if (!pb_encode_tag_for_field(stream, field))
        return false;

    return pb_encode_string(stream, (const uint8_t*) str, strlen(str));
}

//! simple wrapper type to pass array length into write_array
template <typename T>
struct array_handle {
    T* ptr;
    size_t len;
};

//! nanopb callback for writing an array
template<typename T, const pb_field_t* fields>
static bool write_array(pb_ostream_t * stream, const pb_field_t *field, void * const *arg)
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

//! do any setup required for messaging
void setupMessaging() {
    Serial.begin(57600);
}

//! Send a message object over serial, using protobuf and cobs
static void sendMessage(RobotMessage& message) {
    // Create stream
    pb_ostream_t pb_stream = as_pb_ostream(cobs_serial);

    // serialize the message
    bool status = pb_encode(&pb_stream, RobotMessage_fields, &message);

    /* Then just check for any errors.. */
    if (!status)
    {
        printf("Encoding failed: %s\n", PB_GET_ERROR(&pb_stream));
        cobs_serial.abort();
    }
    else{
        // finish the packet
        cobs_serial.end();
    }
}

//! send a debug string
void debug(const char* text) {

    // fill out the message
    RobotMessage message = RobotMessage_init_zero;
    message.which_msg = RobotMessage_debug_tag;
    message.msg.debug.s.funcs.encode = write_string;
    message.msg.debug.s.arg = &text;

    sendMessage(message);
}

//! send log messages
void sendLogs(const LogEntry* entries, size_t n) {
    array_handle<const LogEntry> arr = {entries, n};

    // fill out the message
    RobotMessage message = RobotMessage_init_zero;
    message.which_msg = RobotMessage_log_tag;
    message.msg.log.entries.funcs.encode = &write_array<const LogEntry, LogEntry_fields>;
    message.msg.log.entries.arg = &arr;

    sendMessage(message);
}
