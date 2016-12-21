from SCons.Script import DefaultEnvironment
import os

env = DefaultEnvironment()

def _detect():
    """Try to find the Protoc compiler"""
    try:
        return env['PROTOC']
    except KeyError:
        pass

    protoc = env.WhereIs('protoc')
    if protoc:
        return protoc

    else:
        # this is where it is on my machine
        protoc = r'C:\software\nanopb-0.3.7-windows-x86\generator-bin\protoc.exe'
        if os.path.exists(protoc):
            return protoc

env['PROTOC'] = _detect()

# rebuild protobuf files
env.Command(
    target=['messages.pb.c', 'messages.pb.h'],
    source='messages.proto',
    action='$PROTOC --nanopb_out=. $SOURCES'
)
