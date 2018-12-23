%module PSMoveClient

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

#define PSM_PUBLIC_FUNCTION(rval) rval
#define PSM_PUBLIC_CLASS
#define PSM_PUBLIC_VARIABLE_DECL(rval) rval

%include "../src/psmoveprotocol/SharedConstants.h"
%include "../src/psmoveclient/ClientConstants.h"
%include "../src/psmoveclient/ClientGeometry_CAPI.h"
%include "../src/psmoveclient/PSMoveClient_CAPI.h"