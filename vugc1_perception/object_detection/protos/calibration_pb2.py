# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: object_detection/protos/calibration.proto

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
  name='object_detection/protos/calibration.proto',
  package='object_detection.protos',
  syntax='proto2',
  serialized_pb=_b('\n)object_detection/protos/calibration.proto\x12\x17object_detection.protos\"\xf7\x02\n\x11\x43\x61librationConfig\x12P\n\x16\x66unction_approximation\x18\x01 \x01(\x0b\x32..object_detection.protos.FunctionApproximationH\x00\x12]\n\x1dlabel_function_approximations\x18\x02 \x01(\x0b\x32\x34.object_detection.protos.LabelFunctionApproximationsH\x00\x12J\n\x13sigmoid_calibration\x18\x03 \x01(\x0b\x32+.object_detection.protos.SigmoidCalibrationH\x00\x12W\n\x1alabel_sigmoid_calibrations\x18\x04 \x01(\x0b\x32\x31.object_detection.protos.LabelSigmoidCalibrationsH\x00\x42\x0c\n\ncalibrator\"L\n\x15\x46unctionApproximation\x12\x33\n\tx_y_pairs\x18\x01 \x01(\x0b\x32 .object_detection.protos.XYPairs\"\xf6\x01\n\x1bLabelFunctionApproximations\x12\x65\n\x12label_xy_pairs_map\x18\x01 \x03(\x0b\x32I.object_detection.protos.LabelFunctionApproximations.LabelXyPairsMapEntry\x12\x16\n\x0elabel_map_path\x18\x02 \x01(\t\x1aX\n\x14LabelXyPairsMapEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12/\n\x05value\x18\x02 \x01(\x0b\x32 .object_detection.protos.XYPairs:\x02\x38\x01\"\\\n\x12SigmoidCalibration\x12\x46\n\x12sigmoid_parameters\x18\x01 \x01(\x0b\x32*.object_detection.protos.SigmoidParameters\"\x98\x02\n\x18LabelSigmoidCalibrations\x12v\n\x1clabel_sigmoid_parameters_map\x18\x01 \x03(\x0b\x32P.object_detection.protos.LabelSigmoidCalibrations.LabelSigmoidParametersMapEntry\x12\x16\n\x0elabel_map_path\x18\x02 \x01(\t\x1al\n\x1eLabelSigmoidParametersMapEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12\x39\n\x05value\x18\x02 \x01(\x0b\x32*.object_detection.protos.SigmoidParameters:\x02\x38\x01\"d\n\x07XYPairs\x12\x39\n\x08x_y_pair\x18\x01 \x03(\x0b\x32\'.object_detection.protos.XYPairs.XYPair\x1a\x1e\n\x06XYPair\x12\t\n\x01x\x18\x01 \x01(\x02\x12\t\n\x01y\x18\x02 \x01(\x02\"0\n\x11SigmoidParameters\x12\r\n\x01\x61\x18\x01 \x01(\x02:\x02-1\x12\x0c\n\x01\x62\x18\x02 \x01(\x02:\x01\x30')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_CALIBRATIONCONFIG = _descriptor.Descriptor(
  name='CalibrationConfig',
  full_name='object_detection.protos.CalibrationConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='function_approximation', full_name='object_detection.protos.CalibrationConfig.function_approximation', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='label_function_approximations', full_name='object_detection.protos.CalibrationConfig.label_function_approximations', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='sigmoid_calibration', full_name='object_detection.protos.CalibrationConfig.sigmoid_calibration', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='label_sigmoid_calibrations', full_name='object_detection.protos.CalibrationConfig.label_sigmoid_calibrations', index=3,
      number=4, type=11, cpp_type=10, label=1,
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
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
    _descriptor.OneofDescriptor(
      name='calibrator', full_name='object_detection.protos.CalibrationConfig.calibrator',
      index=0, containing_type=None, fields=[]),
  ],
  serialized_start=71,
  serialized_end=446,
)


_FUNCTIONAPPROXIMATION = _descriptor.Descriptor(
  name='FunctionApproximation',
  full_name='object_detection.protos.FunctionApproximation',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='x_y_pairs', full_name='object_detection.protos.FunctionApproximation.x_y_pairs', index=0,
      number=1, type=11, cpp_type=10, label=1,
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
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=448,
  serialized_end=524,
)


_LABELFUNCTIONAPPROXIMATIONS_LABELXYPAIRSMAPENTRY = _descriptor.Descriptor(
  name='LabelXyPairsMapEntry',
  full_name='object_detection.protos.LabelFunctionApproximations.LabelXyPairsMapEntry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='object_detection.protos.LabelFunctionApproximations.LabelXyPairsMapEntry.key', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='value', full_name='object_detection.protos.LabelFunctionApproximations.LabelXyPairsMapEntry.value', index=1,
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
  options=_descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001')),
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=685,
  serialized_end=773,
)

_LABELFUNCTIONAPPROXIMATIONS = _descriptor.Descriptor(
  name='LabelFunctionApproximations',
  full_name='object_detection.protos.LabelFunctionApproximations',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='label_xy_pairs_map', full_name='object_detection.protos.LabelFunctionApproximations.label_xy_pairs_map', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='label_map_path', full_name='object_detection.protos.LabelFunctionApproximations.label_map_path', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[_LABELFUNCTIONAPPROXIMATIONS_LABELXYPAIRSMAPENTRY, ],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=527,
  serialized_end=773,
)


_SIGMOIDCALIBRATION = _descriptor.Descriptor(
  name='SigmoidCalibration',
  full_name='object_detection.protos.SigmoidCalibration',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='sigmoid_parameters', full_name='object_detection.protos.SigmoidCalibration.sigmoid_parameters', index=0,
      number=1, type=11, cpp_type=10, label=1,
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
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=775,
  serialized_end=867,
)


_LABELSIGMOIDCALIBRATIONS_LABELSIGMOIDPARAMETERSMAPENTRY = _descriptor.Descriptor(
  name='LabelSigmoidParametersMapEntry',
  full_name='object_detection.protos.LabelSigmoidCalibrations.LabelSigmoidParametersMapEntry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='object_detection.protos.LabelSigmoidCalibrations.LabelSigmoidParametersMapEntry.key', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='value', full_name='object_detection.protos.LabelSigmoidCalibrations.LabelSigmoidParametersMapEntry.value', index=1,
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
  options=_descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001')),
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1042,
  serialized_end=1150,
)

_LABELSIGMOIDCALIBRATIONS = _descriptor.Descriptor(
  name='LabelSigmoidCalibrations',
  full_name='object_detection.protos.LabelSigmoidCalibrations',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='label_sigmoid_parameters_map', full_name='object_detection.protos.LabelSigmoidCalibrations.label_sigmoid_parameters_map', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='label_map_path', full_name='object_detection.protos.LabelSigmoidCalibrations.label_map_path', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[_LABELSIGMOIDCALIBRATIONS_LABELSIGMOIDPARAMETERSMAPENTRY, ],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=870,
  serialized_end=1150,
)


_XYPAIRS_XYPAIR = _descriptor.Descriptor(
  name='XYPair',
  full_name='object_detection.protos.XYPairs.XYPair',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='x', full_name='object_detection.protos.XYPairs.XYPair.x', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='y', full_name='object_detection.protos.XYPairs.XYPair.y', index=1,
      number=2, type=2, cpp_type=6, label=1,
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
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1222,
  serialized_end=1252,
)

_XYPAIRS = _descriptor.Descriptor(
  name='XYPairs',
  full_name='object_detection.protos.XYPairs',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='x_y_pair', full_name='object_detection.protos.XYPairs.x_y_pair', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[_XYPAIRS_XYPAIR, ],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1152,
  serialized_end=1252,
)


_SIGMOIDPARAMETERS = _descriptor.Descriptor(
  name='SigmoidParameters',
  full_name='object_detection.protos.SigmoidParameters',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='a', full_name='object_detection.protos.SigmoidParameters.a', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=True, default_value=float(-1),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='b', full_name='object_detection.protos.SigmoidParameters.b', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=True, default_value=float(0),
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
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1254,
  serialized_end=1302,
)

_CALIBRATIONCONFIG.fields_by_name['function_approximation'].message_type = _FUNCTIONAPPROXIMATION
_CALIBRATIONCONFIG.fields_by_name['label_function_approximations'].message_type = _LABELFUNCTIONAPPROXIMATIONS
_CALIBRATIONCONFIG.fields_by_name['sigmoid_calibration'].message_type = _SIGMOIDCALIBRATION
_CALIBRATIONCONFIG.fields_by_name['label_sigmoid_calibrations'].message_type = _LABELSIGMOIDCALIBRATIONS
_CALIBRATIONCONFIG.oneofs_by_name['calibrator'].fields.append(
  _CALIBRATIONCONFIG.fields_by_name['function_approximation'])
_CALIBRATIONCONFIG.fields_by_name['function_approximation'].containing_oneof = _CALIBRATIONCONFIG.oneofs_by_name['calibrator']
_CALIBRATIONCONFIG.oneofs_by_name['calibrator'].fields.append(
  _CALIBRATIONCONFIG.fields_by_name['label_function_approximations'])
_CALIBRATIONCONFIG.fields_by_name['label_function_approximations'].containing_oneof = _CALIBRATIONCONFIG.oneofs_by_name['calibrator']
_CALIBRATIONCONFIG.oneofs_by_name['calibrator'].fields.append(
  _CALIBRATIONCONFIG.fields_by_name['sigmoid_calibration'])
_CALIBRATIONCONFIG.fields_by_name['sigmoid_calibration'].containing_oneof = _CALIBRATIONCONFIG.oneofs_by_name['calibrator']
_CALIBRATIONCONFIG.oneofs_by_name['calibrator'].fields.append(
  _CALIBRATIONCONFIG.fields_by_name['label_sigmoid_calibrations'])
_CALIBRATIONCONFIG.fields_by_name['label_sigmoid_calibrations'].containing_oneof = _CALIBRATIONCONFIG.oneofs_by_name['calibrator']
_FUNCTIONAPPROXIMATION.fields_by_name['x_y_pairs'].message_type = _XYPAIRS
_LABELFUNCTIONAPPROXIMATIONS_LABELXYPAIRSMAPENTRY.fields_by_name['value'].message_type = _XYPAIRS
_LABELFUNCTIONAPPROXIMATIONS_LABELXYPAIRSMAPENTRY.containing_type = _LABELFUNCTIONAPPROXIMATIONS
_LABELFUNCTIONAPPROXIMATIONS.fields_by_name['label_xy_pairs_map'].message_type = _LABELFUNCTIONAPPROXIMATIONS_LABELXYPAIRSMAPENTRY
_SIGMOIDCALIBRATION.fields_by_name['sigmoid_parameters'].message_type = _SIGMOIDPARAMETERS
_LABELSIGMOIDCALIBRATIONS_LABELSIGMOIDPARAMETERSMAPENTRY.fields_by_name['value'].message_type = _SIGMOIDPARAMETERS
_LABELSIGMOIDCALIBRATIONS_LABELSIGMOIDPARAMETERSMAPENTRY.containing_type = _LABELSIGMOIDCALIBRATIONS
_LABELSIGMOIDCALIBRATIONS.fields_by_name['label_sigmoid_parameters_map'].message_type = _LABELSIGMOIDCALIBRATIONS_LABELSIGMOIDPARAMETERSMAPENTRY
_XYPAIRS_XYPAIR.containing_type = _XYPAIRS
_XYPAIRS.fields_by_name['x_y_pair'].message_type = _XYPAIRS_XYPAIR
DESCRIPTOR.message_types_by_name['CalibrationConfig'] = _CALIBRATIONCONFIG
DESCRIPTOR.message_types_by_name['FunctionApproximation'] = _FUNCTIONAPPROXIMATION
DESCRIPTOR.message_types_by_name['LabelFunctionApproximations'] = _LABELFUNCTIONAPPROXIMATIONS
DESCRIPTOR.message_types_by_name['SigmoidCalibration'] = _SIGMOIDCALIBRATION
DESCRIPTOR.message_types_by_name['LabelSigmoidCalibrations'] = _LABELSIGMOIDCALIBRATIONS
DESCRIPTOR.message_types_by_name['XYPairs'] = _XYPAIRS
DESCRIPTOR.message_types_by_name['SigmoidParameters'] = _SIGMOIDPARAMETERS

CalibrationConfig = _reflection.GeneratedProtocolMessageType('CalibrationConfig', (_message.Message,), dict(
  DESCRIPTOR = _CALIBRATIONCONFIG,
  __module__ = 'object_detection.protos.calibration_pb2'
  # @@protoc_insertion_point(class_scope:object_detection.protos.CalibrationConfig)
  ))
_sym_db.RegisterMessage(CalibrationConfig)

FunctionApproximation = _reflection.GeneratedProtocolMessageType('FunctionApproximation', (_message.Message,), dict(
  DESCRIPTOR = _FUNCTIONAPPROXIMATION,
  __module__ = 'object_detection.protos.calibration_pb2'
  # @@protoc_insertion_point(class_scope:object_detection.protos.FunctionApproximation)
  ))
_sym_db.RegisterMessage(FunctionApproximation)

LabelFunctionApproximations = _reflection.GeneratedProtocolMessageType('LabelFunctionApproximations', (_message.Message,), dict(

  LabelXyPairsMapEntry = _reflection.GeneratedProtocolMessageType('LabelXyPairsMapEntry', (_message.Message,), dict(
    DESCRIPTOR = _LABELFUNCTIONAPPROXIMATIONS_LABELXYPAIRSMAPENTRY,
    __module__ = 'object_detection.protos.calibration_pb2'
    # @@protoc_insertion_point(class_scope:object_detection.protos.LabelFunctionApproximations.LabelXyPairsMapEntry)
    ))
  ,
  DESCRIPTOR = _LABELFUNCTIONAPPROXIMATIONS,
  __module__ = 'object_detection.protos.calibration_pb2'
  # @@protoc_insertion_point(class_scope:object_detection.protos.LabelFunctionApproximations)
  ))
_sym_db.RegisterMessage(LabelFunctionApproximations)
_sym_db.RegisterMessage(LabelFunctionApproximations.LabelXyPairsMapEntry)

SigmoidCalibration = _reflection.GeneratedProtocolMessageType('SigmoidCalibration', (_message.Message,), dict(
  DESCRIPTOR = _SIGMOIDCALIBRATION,
  __module__ = 'object_detection.protos.calibration_pb2'
  # @@protoc_insertion_point(class_scope:object_detection.protos.SigmoidCalibration)
  ))
_sym_db.RegisterMessage(SigmoidCalibration)

LabelSigmoidCalibrations = _reflection.GeneratedProtocolMessageType('LabelSigmoidCalibrations', (_message.Message,), dict(

  LabelSigmoidParametersMapEntry = _reflection.GeneratedProtocolMessageType('LabelSigmoidParametersMapEntry', (_message.Message,), dict(
    DESCRIPTOR = _LABELSIGMOIDCALIBRATIONS_LABELSIGMOIDPARAMETERSMAPENTRY,
    __module__ = 'object_detection.protos.calibration_pb2'
    # @@protoc_insertion_point(class_scope:object_detection.protos.LabelSigmoidCalibrations.LabelSigmoidParametersMapEntry)
    ))
  ,
  DESCRIPTOR = _LABELSIGMOIDCALIBRATIONS,
  __module__ = 'object_detection.protos.calibration_pb2'
  # @@protoc_insertion_point(class_scope:object_detection.protos.LabelSigmoidCalibrations)
  ))
_sym_db.RegisterMessage(LabelSigmoidCalibrations)
_sym_db.RegisterMessage(LabelSigmoidCalibrations.LabelSigmoidParametersMapEntry)

XYPairs = _reflection.GeneratedProtocolMessageType('XYPairs', (_message.Message,), dict(

  XYPair = _reflection.GeneratedProtocolMessageType('XYPair', (_message.Message,), dict(
    DESCRIPTOR = _XYPAIRS_XYPAIR,
    __module__ = 'object_detection.protos.calibration_pb2'
    # @@protoc_insertion_point(class_scope:object_detection.protos.XYPairs.XYPair)
    ))
  ,
  DESCRIPTOR = _XYPAIRS,
  __module__ = 'object_detection.protos.calibration_pb2'
  # @@protoc_insertion_point(class_scope:object_detection.protos.XYPairs)
  ))
_sym_db.RegisterMessage(XYPairs)
_sym_db.RegisterMessage(XYPairs.XYPair)

SigmoidParameters = _reflection.GeneratedProtocolMessageType('SigmoidParameters', (_message.Message,), dict(
  DESCRIPTOR = _SIGMOIDPARAMETERS,
  __module__ = 'object_detection.protos.calibration_pb2'
  # @@protoc_insertion_point(class_scope:object_detection.protos.SigmoidParameters)
  ))
_sym_db.RegisterMessage(SigmoidParameters)


_LABELFUNCTIONAPPROXIMATIONS_LABELXYPAIRSMAPENTRY.has_options = True
_LABELFUNCTIONAPPROXIMATIONS_LABELXYPAIRSMAPENTRY._options = _descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001'))
_LABELSIGMOIDCALIBRATIONS_LABELSIGMOIDPARAMETERSMAPENTRY.has_options = True
_LABELSIGMOIDCALIBRATIONS_LABELSIGMOIDPARAMETERSMAPENTRY._options = _descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001'))
# @@protoc_insertion_point(module_scope)
