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

def log_to_np(log):
    return np.core.records.fromrecords([
        field_values(e)
        for e in log.entry
    ], dtype=dtype_for(messages_pb2.LogEntry))

base = Path('..')

class LogSaver:
    def __init__(self):
        self.logs_dir = base / 'logs' / (
            '{:%Y-%m-%d %H:%M:%S}'
                .format(datetime.now())
                .replace(':', '\uA789')
        )
        self.log_count = 0

    def save(self, log):
        if self.log_count == 0:
            self.logs_dir.mkdir()

        fpath = (self.logs_dir / '{}.mat'.format(self.log_count))
        log = log_to_np(log)
        with fpath.open('wb') as f:
            scipy.io.savemat(f, dict(log=log, tstamp=datetime.now().isoformat()))

        self.log_count += 1
        return fpath