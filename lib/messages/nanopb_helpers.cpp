#include "nanopb_helpers.h"

namespace nanopb_helpers {

//! nanopb callback for writing a string
bool write_string(pb_ostream_t *stream, const pb_field_t *field, void * const *arg)
{
    auto argval = *arg;
    auto handle = *reinterpret_cast<array_handle<const uint8_t>*>(argval);

    if (!pb_encode_tag_for_field(stream, field))
        return false;

    return pb_encode_string(stream, handle.ptr, handle.len);
}

}
