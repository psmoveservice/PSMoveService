import os
import platform
import struct
from ctypes import util
import re
from cffi import FFI
from shutil import copyfile

def get_libpath():
    # find the library
    os_name = platform.system()
    bitness = 8 * struct.calcsize("P")
    if bitness == 32:
        lib_suffix = ''
    else:
        lib_suffix = ''
    if os_name in ['Windows', 'Microsoft']:
        lib_ext = '.dll'
        lib_prefix = ''
    elif os_name == 'Darwin':
        lib_ext = '.dylib'
        lib_prefix = 'lib'
    elif os_name == 'Linux':
        lib_ext = '.so'
        lib_prefix = 'lib'
    else:
        raise RuntimeError("unrecognized operating system:", os_name)

    libname = lib_prefix + 'PSMoveClient_CAPI' + lib_suffix + lib_ext
    libpath = os.path.abspath(os.path.join('.', '..', '..', 'build', 'src', 'psmoveclient', 'Debug', libname))
    if not os.path.isfile(libpath):
        libpath = util.find_library(libname)
    if not libpath:
        raise RuntimeError("library " + libname + " was not found"
                           " - make sure that it is on the search path.")

    # TODO: Search ../pypsmmove for the library. If not there, copy from expected build location into that dir.
    copyfile(libpath, os.path.abspath(os.path.join('.', 'pypsmove', libname)))

    return libpath

def get_headerpath():
    # TODO: Search for header in ../pypsmove folder. If not there, copy from source into that dir.
    return os.path.abspath(os.path.join('..', '..', 'src', 'psmoveclient'))

def get_cleaned_header():
    with open(os.path.join(get_headerpath(), 'PSMoveClient_CAPI.h'), 'r') as myfile:
        header_dat=myfile.read()
    # Cut the #defines and #ifdefs from the beginning and end
    cut_ix = [ss.start() for ss in re.finditer(r"//cut_before", header_dat)][0]
    header_dat = header_dat[cut_ix + 13:]
    cut_ix = [ss.start() for ss in re.finditer(r"\n//cut_after", header_dat)][0]
    header_dat = header_dat[:cut_ix]
    #Replace macro wrapper with only wrapper contents.
    pattern = r"PSM_PUBLIC_FUNCTION\((.*?)\)"
    header_dat = re.sub(pattern, r"\1", header_dat)
    #Replace defines with their actual values
    #TODO: Read constants from ClientConstants.h and parse them to do the following replacements.
    header_dat = re.sub(r"PSMOVESERVICE_MAX_CONTROLLER_COUNT", r"5", header_dat)
    header_dat = re.sub(r"PSMOVESERVICE_MAX_TRACKER_COUNT", r"4", header_dat)
    header_dat = re.sub(r"PSMOVESERVICE_MAX_HMD_COUNT", r"1", header_dat)
    return header_dat

libpath = get_libpath()
lib_dir, lib_name = os.path.split(libpath)
ffi = FFI()
ffi.set_source("pypsmove._psmoveclient",
    """
    #include "PSMoveClient_CAPI.h"
    """,
    include_dirs=[get_headerpath()],
    libraries=['PSMoveClient_CAPI'], library_dirs=[lib_dir])  #lib_path, os.path.abspath('.')
ffi.cdef(get_cleaned_header())


# If we wanted to use the cffi ABI way, we would load the library as follows
# lib = ffi.dlopen(libpath); 
# To use the cffi API way, which is more robust, we execute this script to compile a module.
# See psmoveclient.py for an example of how to use the compiled module.

#install_name_tool -change @rpath/libPSMoveClient_CAPI.dylib @loader_path/libPSMoveClient_CAPI.dylib _psmoveclient.cpython-35m-darwin.so

if __name__ == "__main__":
    ffi.compile(verbose=True)
    if platform.system() == "Darwin":
        from subprocess import call
        call(['install_name_tool', '-change', '@rpath/libPSMoveClient_CAPI.dylib', '@loader_path/libPSMoveClient_CAPI.dylib', '_psmoveclient.cpython-35m-darwin.so'])