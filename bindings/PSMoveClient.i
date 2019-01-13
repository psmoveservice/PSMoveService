%module PSMoveClient
%include "typemaps.i"

%begin %{
  #ifdef _MSC_VER
  #define SWIG_PYTHON_INTERPRETER_NO_DEBUG
  #endif
%}

%{
  #include "SharedConstants.h"
  #include "ClientConstants.h"
  #include "ClientGeometry_CAPI.h"
  #include "PSMoveClient_CAPI.h"
%}

#ifdef _MSC_VER
#define PSM_CDECL __cdecl
#define PSM_STDCALL __stdcall
#else
#define PSM_CDECL
#define PSM_STDCALL
#endif
#define PSM_PUBLIC_FUNCTION(rval) rval
#define PSM_PUBLIC_CLASS
#define PSM_PUBLIC_VARIABLE_DECL(rval) rval

#if defined(SWIGPYTHON)
#endif  /* SWIGPYTHON */

#if defined(SWIGCSHARP)
%include "PSMoveClientCSharpTypeMaps.i"  
%include "PSMoveClientCSharpCallbacks.i"
#endif  /* SWIGCSHARP */

%include "../src/psmoveprotocol/SharedConstants.h"
%include "../src/psmoveclient/ClientConstants.h"
%include "../src/psmoveclient/ClientGeometry_CAPI.h"

// Make all variables read-only in exposed structs in PSMoveClient_CAPI.h
%immutable;
%include "../src/psmoveclient/PSMoveClient_CAPI.h"