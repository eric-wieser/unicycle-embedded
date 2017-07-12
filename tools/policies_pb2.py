# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: policies.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='policies.proto',
  package='',
  syntax='proto3',
  serialized_pb=_b('\n\x0epolicies.proto\"\xbe\x01\n\x0cLinearPolicy\x12\x0f\n\x07k_droll\x18\x01 \x01(\x02\x12\x0e\n\x06k_dyaw\x18\x02 \x01(\x02\x12\x11\n\tk_dAngleW\x18\x03 \x01(\x02\x12\x10\n\x08k_dpitch\x18\x04 \x01(\x02\x12\x12\n\nk_dAngleTT\x18\x05 \x01(\x02\x12\x11\n\tk_xOrigin\x18\x06 \x01(\x02\x12\x11\n\tk_yOrigin\x18\x07 \x01(\x02\x12\x0e\n\x06k_roll\x18\x08 \x01(\x02\x12\r\n\x05k_yaw\x18\t \x01(\x02\x12\x0f\n\x07k_pitch\x18\n \x01(\x02\"\xdb\x02\n\x13PureQuadraticPolicy\x12\x1e\n\x07k_droll\x18\x01 \x01(\x0b\x32\r.LinearPolicy\x12\x1d\n\x06k_dyaw\x18\x02 \x01(\x0b\x32\r.LinearPolicy\x12 \n\tk_dAngleW\x18\x03 \x01(\x0b\x32\r.LinearPolicy\x12\x1f\n\x08k_dpitch\x18\x04 \x01(\x0b\x32\r.LinearPolicy\x12!\n\nk_dAngleTT\x18\x05 \x01(\x0b\x32\r.LinearPolicy\x12 \n\tk_xOrigin\x18\x06 \x01(\x0b\x32\r.LinearPolicy\x12 \n\tk_yOrigin\x18\x07 \x01(\x0b\x32\r.LinearPolicy\x12\x1d\n\x06k_roll\x18\x08 \x01(\x0b\x32\r.LinearPolicy\x12\x1c\n\x05k_yaw\x18\t \x01(\x0b\x32\r.LinearPolicy\x12\x1e\n\x07k_pitch\x18\n \x01(\x0b\x32\r.LinearPolicy\"<\n\x0c\x41\x66\x66inePolicy\x12\x0e\n\x06k_bias\x18\x01 \x01(\x02\x12\x1c\n\x05k_lin\x18\x02 \x01(\x0b\x32\r.LinearPolicy\"e\n\x0fQuadraticPolicy\x12\x0e\n\x06k_bias\x18\x01 \x01(\x02\x12\x1c\n\x05k_lin\x18\x02 \x01(\x0b\x32\r.LinearPolicy\x12$\n\x06k_quad\x18\x03 \x01(\x0b\x32\x14.PureQuadraticPolicy\"p\n\x06Policy\x12\x1c\n\x03lin\x18\x01 \x01(\x0b\x32\r.LinearPolicyH\x00\x12\x1f\n\x06\x61\x66\x66ine\x18\x02 \x01(\x0b\x32\r.AffinePolicyH\x00\x12 \n\x04quad\x18\x03 \x01(\x0b\x32\x10.QuadraticPolicyH\x00\x42\x05\n\x03msgb\x06proto3')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_LINEARPOLICY = _descriptor.Descriptor(
  name='LinearPolicy',
  full_name='LinearPolicy',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='k_droll', full_name='LinearPolicy.k_droll', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='k_dyaw', full_name='LinearPolicy.k_dyaw', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='k_dAngleW', full_name='LinearPolicy.k_dAngleW', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='k_dpitch', full_name='LinearPolicy.k_dpitch', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='k_dAngleTT', full_name='LinearPolicy.k_dAngleTT', index=4,
      number=5, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='k_xOrigin', full_name='LinearPolicy.k_xOrigin', index=5,
      number=6, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='k_yOrigin', full_name='LinearPolicy.k_yOrigin', index=6,
      number=7, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='k_roll', full_name='LinearPolicy.k_roll', index=7,
      number=8, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='k_yaw', full_name='LinearPolicy.k_yaw', index=8,
      number=9, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='k_pitch', full_name='LinearPolicy.k_pitch', index=9,
      number=10, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=19,
  serialized_end=209,
)


_PUREQUADRATICPOLICY = _descriptor.Descriptor(
  name='PureQuadraticPolicy',
  full_name='PureQuadraticPolicy',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='k_droll', full_name='PureQuadraticPolicy.k_droll', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='k_dyaw', full_name='PureQuadraticPolicy.k_dyaw', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='k_dAngleW', full_name='PureQuadraticPolicy.k_dAngleW', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='k_dpitch', full_name='PureQuadraticPolicy.k_dpitch', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='k_dAngleTT', full_name='PureQuadraticPolicy.k_dAngleTT', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='k_xOrigin', full_name='PureQuadraticPolicy.k_xOrigin', index=5,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='k_yOrigin', full_name='PureQuadraticPolicy.k_yOrigin', index=6,
      number=7, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='k_roll', full_name='PureQuadraticPolicy.k_roll', index=7,
      number=8, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='k_yaw', full_name='PureQuadraticPolicy.k_yaw', index=8,
      number=9, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='k_pitch', full_name='PureQuadraticPolicy.k_pitch', index=9,
      number=10, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=212,
  serialized_end=559,
)


_AFFINEPOLICY = _descriptor.Descriptor(
  name='AffinePolicy',
  full_name='AffinePolicy',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='k_bias', full_name='AffinePolicy.k_bias', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='k_lin', full_name='AffinePolicy.k_lin', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=561,
  serialized_end=621,
)


_QUADRATICPOLICY = _descriptor.Descriptor(
  name='QuadraticPolicy',
  full_name='QuadraticPolicy',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='k_bias', full_name='QuadraticPolicy.k_bias', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='k_lin', full_name='QuadraticPolicy.k_lin', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='k_quad', full_name='QuadraticPolicy.k_quad', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=623,
  serialized_end=724,
)


_POLICY = _descriptor.Descriptor(
  name='Policy',
  full_name='Policy',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='lin', full_name='Policy.lin', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='affine', full_name='Policy.affine', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='quad', full_name='Policy.quad', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
    _descriptor.OneofDescriptor(
      name='msg', full_name='Policy.msg',
      index=0, containing_type=None, fields=[]),
  ],
  serialized_start=726,
  serialized_end=838,
)

_PUREQUADRATICPOLICY.fields_by_name['k_droll'].message_type = _LINEARPOLICY
_PUREQUADRATICPOLICY.fields_by_name['k_dyaw'].message_type = _LINEARPOLICY
_PUREQUADRATICPOLICY.fields_by_name['k_dAngleW'].message_type = _LINEARPOLICY
_PUREQUADRATICPOLICY.fields_by_name['k_dpitch'].message_type = _LINEARPOLICY
_PUREQUADRATICPOLICY.fields_by_name['k_dAngleTT'].message_type = _LINEARPOLICY
_PUREQUADRATICPOLICY.fields_by_name['k_xOrigin'].message_type = _LINEARPOLICY
_PUREQUADRATICPOLICY.fields_by_name['k_yOrigin'].message_type = _LINEARPOLICY
_PUREQUADRATICPOLICY.fields_by_name['k_roll'].message_type = _LINEARPOLICY
_PUREQUADRATICPOLICY.fields_by_name['k_yaw'].message_type = _LINEARPOLICY
_PUREQUADRATICPOLICY.fields_by_name['k_pitch'].message_type = _LINEARPOLICY
_AFFINEPOLICY.fields_by_name['k_lin'].message_type = _LINEARPOLICY
_QUADRATICPOLICY.fields_by_name['k_lin'].message_type = _LINEARPOLICY
_QUADRATICPOLICY.fields_by_name['k_quad'].message_type = _PUREQUADRATICPOLICY
_POLICY.fields_by_name['lin'].message_type = _LINEARPOLICY
_POLICY.fields_by_name['affine'].message_type = _AFFINEPOLICY
_POLICY.fields_by_name['quad'].message_type = _QUADRATICPOLICY
_POLICY.oneofs_by_name['msg'].fields.append(
  _POLICY.fields_by_name['lin'])
_POLICY.fields_by_name['lin'].containing_oneof = _POLICY.oneofs_by_name['msg']
_POLICY.oneofs_by_name['msg'].fields.append(
  _POLICY.fields_by_name['affine'])
_POLICY.fields_by_name['affine'].containing_oneof = _POLICY.oneofs_by_name['msg']
_POLICY.oneofs_by_name['msg'].fields.append(
  _POLICY.fields_by_name['quad'])
_POLICY.fields_by_name['quad'].containing_oneof = _POLICY.oneofs_by_name['msg']
DESCRIPTOR.message_types_by_name['LinearPolicy'] = _LINEARPOLICY
DESCRIPTOR.message_types_by_name['PureQuadraticPolicy'] = _PUREQUADRATICPOLICY
DESCRIPTOR.message_types_by_name['AffinePolicy'] = _AFFINEPOLICY
DESCRIPTOR.message_types_by_name['QuadraticPolicy'] = _QUADRATICPOLICY
DESCRIPTOR.message_types_by_name['Policy'] = _POLICY

LinearPolicy = _reflection.GeneratedProtocolMessageType('LinearPolicy', (_message.Message,), dict(
  DESCRIPTOR = _LINEARPOLICY,
  __module__ = 'policies_pb2'
  # @@protoc_insertion_point(class_scope:LinearPolicy)
  ))
_sym_db.RegisterMessage(LinearPolicy)

PureQuadraticPolicy = _reflection.GeneratedProtocolMessageType('PureQuadraticPolicy', (_message.Message,), dict(
  DESCRIPTOR = _PUREQUADRATICPOLICY,
  __module__ = 'policies_pb2'
  # @@protoc_insertion_point(class_scope:PureQuadraticPolicy)
  ))
_sym_db.RegisterMessage(PureQuadraticPolicy)

AffinePolicy = _reflection.GeneratedProtocolMessageType('AffinePolicy', (_message.Message,), dict(
  DESCRIPTOR = _AFFINEPOLICY,
  __module__ = 'policies_pb2'
  # @@protoc_insertion_point(class_scope:AffinePolicy)
  ))
_sym_db.RegisterMessage(AffinePolicy)

QuadraticPolicy = _reflection.GeneratedProtocolMessageType('QuadraticPolicy', (_message.Message,), dict(
  DESCRIPTOR = _QUADRATICPOLICY,
  __module__ = 'policies_pb2'
  # @@protoc_insertion_point(class_scope:QuadraticPolicy)
  ))
_sym_db.RegisterMessage(QuadraticPolicy)

Policy = _reflection.GeneratedProtocolMessageType('Policy', (_message.Message,), dict(
  DESCRIPTOR = _POLICY,
  __module__ = 'policies_pb2'
  # @@protoc_insertion_point(class_scope:Policy)
  ))
_sym_db.RegisterMessage(Policy)


# @@protoc_insertion_point(module_scope)