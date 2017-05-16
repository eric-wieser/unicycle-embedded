"""
Matlab has no easy-to-use protobuf driver.

Instead of writing one, we just have python convert to mat files
"""
from pathlib import Path
from datetime import datetime

import scipy.io
import numpy as np
from google.protobuf.descriptor import FieldDescriptor

import messages_pb2

# mapping of protobuf cpp types to numpy types
np_types = {
    FieldDescriptor.CPPTYPE_BOOL:    np.bool,
    FieldDescriptor.CPPTYPE_DOUBLE:  np.float64,
    FieldDescriptor.CPPTYPE_ENUM:    np.int64,
    FieldDescriptor.CPPTYPE_FLOAT:   np.float32,
    FieldDescriptor.CPPTYPE_INT32:   np.int32,
    FieldDescriptor.CPPTYPE_INT64:   np.int64,
    FieldDescriptor.CPPTYPE_MESSAGE: np.object,
    FieldDescriptor.CPPTYPE_STRING:  np.object,
    FieldDescriptor.CPPTYPE_UINT32:  np.uint32,
    FieldDescriptor.CPPTYPE_UINT64:  np.uint64
}

def dtype_for(msg_type):
    """ Get the dtype corresponding to the message type """
    return np.dtype([
        (f.name, np_types[f.cpp_type])
        for f in msg_type.DESCRIPTOR.fields
    ])

def field_values(msg):
    return tuple(getattr(msg, f.name) for f in msg.DESCRIPTOR.fields)

def repeated_submessage_to_np(type, msg):
    return np.core.records.fromrecords([
        field_values(e)
        for e in msg
    ], dtype=dtype_for(type))

base = Path('..')

class LogSaver:
    def __init__(self):
        self.logs_dir = base / 'logs' / (
            '{:%Y-%m-%d %H:%M:%S}'
                .format(datetime.now())
                .replace(':', '\uA789')
        )
        self.log_count = 0

    def save(self, logs):
        fpath = (self.logs_dir / '{}.mat'.format(self.log_count))
        log = repeated_submessage_to_np(messages_pb2.LogEntry, logs)

        if self.log_count == 0:
            self.logs_dir.mkdir()
        with fpath.open('wb') as f:
            scipy.io.savemat(f, dict(msg=log, tstamp=datetime.now().isoformat()))

        self.log_count += 1
        return fpath


def _apply_to_msg(msg, np_array):
    """
    Take a message, and a numpy array loaded from a .mat file, and update the
    fields in the message.

    The fieldnames should match exactly, and the file must have been loaded with
    squeeze_me=True.

    Only works for scalar structs right now - no arrays
    """
    for field_name in np_array.dtype.fields:
        field_val = np_array[field_name]
        field_val = field_val[()]  # squeeze_me doesn't squeeze 0d to scalar

        # recurse nested fields
        if isinstance(field_val, np.ndarray):
            _apply_to_msg(getattr(msg, field_name), field_val)
        # directly set other ones
        else:
            setattr(msg, field_name, field_val)
    return msg

def load_policy(mat_file_name):
    msg = scipy.io.loadmat(mat_file_name, squeeze_me=True)['msg']
    return _apply_to_msg(messages_pb2.Controller(), msg)
