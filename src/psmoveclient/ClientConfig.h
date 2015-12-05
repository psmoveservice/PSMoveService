#ifndef CLIENT_CONFIG_H
#define CLIENT_CONFIG_H

#ifdef _WIN32
#  if defined(BUILDING_SHARED_LIBRARY)
#    define CLIENTPSMOVEAPI __declspec(dllexport)
#  else
#    define CLIENTPSMOVEAPI __declspec(dllimport)
#  endif
#else
#  define CLIENTPSMOVEAPI
#endif

#endif // CLIENT_CONFIG_H