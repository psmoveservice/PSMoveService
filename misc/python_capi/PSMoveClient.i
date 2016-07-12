/* File : PSMoveClient.i */
/* swig -python -outdir pyfiles -o cppfiles/example_wrap.c -I../../src/psmoveclient PSMoveClient.i */
/* gcc -c cppfiles/example_wrap.c `pkg-config --cflags python3` -I../../src/psmoveclient */
%module PSMoveClient
%{
#include "PSMoveClient_CAPI.h"
%}

%include "PSMoveClient_export.h"
%include "ClientConstants.h"
%include "PSMoveClient_CAPI.h"
