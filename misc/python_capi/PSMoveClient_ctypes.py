import os
import platform
import struct
from ctypes import CDLL, util, byref, c_char_p, c_void_p, c_double, c_int, \
    c_long, c_float, c_short, c_byte, c_longlong
    
def Initialize(host='localhost', port='9512'):
    return lib.PSM_Initialize(c_char_p(str.encode(host)), c_char_p(str.encode(port)))

def Shutdown():
    return lib.PSM_Shutdown()

# find and load library
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
    
client_path = os.path.abspath(os.path.join('.', '..', '..', 'src', 'psmoveclient'))
lib_path = os.path.abspath(os.path.join('.', '..', '..', 'build', 'src', 'psmoveclient', 'Debug'))

libname = lib_prefix + 'PSMoveClient_CAPI' + lib_suffix + '.' + lib_ext
libpath = os.path.abspath(os.path.join(lib_path, libname))
if not os.path.isfile(libpath):
    libpath = util.find_library(libname)
if not libpath:
    raise RuntimeError("library " + libname + " was not found - make sure "
                       "that it is on the search path (e.g., in the same "
                       "folder as pylsl.py).")
lib = CDLL(libpath)

lib.PSM_Initialize.restype = c_int
lib.PSM_Initialize.argtypes = [c_char_p, c_char_p]
lib.PSM_Shutdown.restype = c_int