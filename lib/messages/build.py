"""
This file is loaded by PlatformIO to do a custom build of this directory.
The main library being used here is Scons

We use it to compile the protobuf definition files using nanopb.

Running `pio run -t python` will rebuild the python protobuf files
and dump them in :/tools
"""

from SCons.Script import DefaultEnvironment, Main

# This line forces scons to discover our site_scons/site_tools/protoc.py file
# We shouldn't have to do this - it's normally automatic
# This is probably a bug in platformio
Main.test_load_all_site_scons_dirs('.')

env = DefaultEnvironment().Clone(tools=['protoc'])

# Add search paths for the protoc tool. This needs to be the one built with nanopb.
# Try to only add lines here, so that it works on all previous editor's PCS
env.AppendENVPath('PATH', R'C:\software\nanopb-0.3.7-windows-x86\generator-bin')

for f in ['messages', 'policies']:
    # configure build for arduino protobuf files
    target_nanopb = env.Protoc(
        target=['{}.pb.h'.format(f)],
        source='{}.proto'.format(f),
        PROTOCNANOOUTDIR='${SOURCE.dir}'
    )

    # configure build for python protobuf files
    target_python = env.Protoc(
        target=[],
        source='{}.proto'.format(f),
        PROTOCPYTHONOUTDIR='tools'
    )

env.Alias('python', target_python)
