#ifndef CLIENT_CONFIG_H
#define CLIENT_CONFIG_H

#ifdef _WIN32
#  define CLIENTPSMOVECALL __cdecl
#  if defined(BUILDING_SHARED_PSMOVECLIENT_LIBRARY)
#    define CLIENTPSMOVEAPI __declspec(dllexport)
#  else
#    define CLIENTPSMOVEAPI __declspec(dllimport)
#  endif
#else
#  define CLIENTPSMOVEAPI
#  define CLIENTPSMOVECALL
#endif

#endif // CLIENT_CONFIG_H