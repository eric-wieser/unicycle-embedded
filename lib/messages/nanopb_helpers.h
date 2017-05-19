#pragma once

#include <pb_encode.h>

namespace nanopb_helpers {

//! simple wrapper type to pass array length into write_array
template <typename T>
struct array_handle {
    T* ptr;
    size_t len;
};

//! nanopb callback for writing a string
bool write_string(pb_ostream_t *stream, const pb_field_t *field, void * const *arg);

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

}
