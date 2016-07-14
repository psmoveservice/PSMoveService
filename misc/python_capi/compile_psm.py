import os
import platform
import struct
import re
from cffi import FFI
from shutil import copyfile

client_path = os.path.abspath(os.path.join('..', '..', 'src', 'psmoveclient'))

with open(os.path.join(client_path, 'PSMoveClient_CAPI.h'), 'r') as myfile:
    header_dat=myfile.read()

# Get rid of the leading #ifdefs and includes
cut_ix = [ss.start() for ss in re.finditer(r"\n", header_dat)][4]
header_dat = header_dat[cut_ix:]
cut_ix = [ss.start() for ss in re.finditer(r"\n", header_dat)][-2]
header_dat = header_dat[:cut_ix]

pattern = r"PSM_PUBLIC_FUNCTION\((.*?)\)"
header_dat = re.sub(pattern, r"\1", header_dat)

#TODO: Read constants from ClientConstants.h and parse them to do the following replacements.
header_dat = re.sub(r"PSMOVESERVICE_MAX_CONTROLLER_COUNT", r"5", header_dat)
header_dat = re.sub(r"PSMOVESERVICE_MAX_TRACKER_COUNT", r"4", header_dat)

os_name = platform.system()
bitness = 8 * struct.calcsize("P")
if bitness == 32:
    lib_suffix = ''
else:
    lib_suffix = ''
if os_name in ['Windows', 'Microsoft']:
    lib_ext = 'dll'
    lib_prefix = ''
elif os_name == 'Darwin':
    lib_ext = 'dylib'
    lib_prefix = 'lib'
elif os_name == 'Linux':
    lib_ext = '.so'
    lib_prefix = 'lib'
else:
    raise RuntimeError("unrecognized operating system:", os_name)
lib_path = os.path.abspath(os.path.join('.', '..', '..', 'build', 'src', 'psmoveclient', 'Debug'))
libname = lib_prefix + 'PSMoveClient_CAPI' + lib_suffix + '.' + lib_ext

ffi = FFI()
ffi.set_source("_psmoveclient",
    """
    #include "PSMoveClient_CAPI.h"
    """,
    include_dirs=[client_path],
    libraries=['PSMoveClient_CAPI'], library_dirs=[os.path.abspath('.')])  #lib_path, os.path.abspath('.')
ffi.cdef(header_dat)

libpath = os.path.abspath(os.path.join(lib_path, libname))
if not os.path.isfile(libpath):
    libpath = util.find_library(libname)
if not libpath:
    raise RuntimeError("library " + libname + " was not found - make sure "
                       "that it is on the search path (e.g., in the same "
                       "folder as _psmoveclient.py).")

# If we wanted to use the cffi ABI way, we would load the library as follows
# lib = ffi.dlopen(libpath)
# To use the cffi API way, which is more robust, we execute this script to compile a module.
# See test_psmoveclient.py for an example of how to use the compiled module.

if __name__ == "__main__":
    copyfile(libpath, os.path.abspath(os.path.join('.', libname)))
    ffi.compile(verbose=True)
    if platform.system() == "Darwin":
        from subprocess import call
        call(['install_name_tool', '-change', '@rpath/libPSMoveClient_CAPI.dylib', '@loader_path/libPSMoveClient_CAPI.dylib', '_psmoveclient.cpython-35m-darwin.so'])