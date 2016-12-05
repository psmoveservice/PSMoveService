#ifndef PSMOVECLIENT_EXPORT_H
#define PSMOVECLIENT_EXPORT_H

#ifndef PSM_CALL
    #if defined _WIN32 || defined __CYGWIN__
        #define PSM_CALL __cdecl
    #else
        #define PSM_CALL
    #endif
#endif

#ifndef PSM_EXTERN_C
    #ifdef __cplusplus
        #define PSM_EXTERN_C extern "C"
    #else
        #define PSM_EXTERN_C
    #endif
#endif

#ifndef PSM_PUBLIC_FUNCTION
    #if defined(PSMoveClient_EXPORTS) || defined(PSMoveClient_CAPI_EXPORTS)  // CMake-defined when creating shared library
        #if defined _WIN32 || defined __CYGWIN__
            #define PSM_PUBLIC_FUNCTION(rval)       PSM_EXTERN_C    __declspec(dllexport)                   rval    PSM_CALL
            #define PSM_PUBLIC_CLASS                                __declspec(dllexport)
            #define PSM_PRIVATE_FUNCTION(rval)                                                              rval    PSM_CALL
            #define PSM_PRIVATE_CLASS
        #else  // Not Windows
            #if __GNUC__ >= 4
                #define PSM_PUBLIC_FUNCTION(rval)   PSM_EXTERN_C    __attribute__((visibility("default")))  rval    PSM_CALL
                #define PSM_PUBLIC_CLASS                            __attribute__((visibility("default")))
            #else
                #define PSM_PUBLIC_FUNCTION(rval)   PSM_EXTERN_C rval PSM_CALL
                #define PSM_PUBLIC_CLASS
            #endif
            #define PSM_PRIVATE_FUNCTION(rval)                      __attribute__((visibility("hidden")))   rval    PSM_CALL
            #define PSM_PRIVATE_CLASS                               __attribute__((visibility("hidden")))
        #endif  //defined _WIN32 || defined __CYGWIN__
    #elif defined(PSMoveClient_STATIC) || defined(PSMoveClient_CAPI_STATIC)  // Building static lib
        #define PSM_PUBLIC_FUNCTION(rval)           PSM_EXTERN_C                                            rval    PSM_CALL
        #define PSM_PUBLIC_CLASS
        #define PSM_PRIVATE_FUNCTION(rval)                                                                  rval    PSM_CALL
        #define PSM_PRIVATE_CLASS
    #else //This DLL/so/dylib is being imported
        #if defined _WIN32 || defined __CYGWIN__
            #define PSM_PUBLIC_FUNCTION(rval)       PSM_EXTERN_C    __declspec(dllimport)                   rval    PSM_CALL
            #define PSM_PUBLIC_CLASS                                __declspec(dllimport)
        #else  // Not Windows
            #define PSM_PUBLIC_FUNCTION(rval)       PSM_EXTERN_C                                            rval    PSM_CALL
            #define PSM_PUBLIC_CLASS
        #endif  //defined _WIN32 || defined __CYGWIN__
        #define PSM_PRIVATE_FUNCTION(rval)                                                                  rval    PSM_CALL
        #define PSM_PRIVATE_CLASS
    #endif //PSMoveClient_EXPORTS
#endif //!defined(PSM_PUBLIC_FUNCTION)

/*
 We also have a CPP API that is still in use.
 We want those classes/functions to NOT be exported/imported when building/using
 the C API, but we do want them exported/imported when building/using the C++ API.
 */

#if !defined(PSM_CPP_PUBLIC_FUNCTION) && defined(PSMOVECLIENT_CPP_API)
    #define PSM_CPP_PUBLIC_FUNCTION(rval) PSM_PUBLIC_FUNCTION(rval)
    #define PSM_CPP_PUBLIC_CLASS PSM_PUBLIC_CLASS
    #define PSM_CPP_PRIVATE_FUNCTION(rval) PSM_PRIVATE_FUNCTION(rval)
    #define PSM_CPP_PRIVATE_CLASS PSM_PRIVATE_CLASS
#else
    #define PSM_CPP_PUBLIC_FUNCTION(rval) PSM_PRIVATE_FUNCTION(rval)
    #define PSM_CPP_PUBLIC_CLASS PSM_PRIVATE_CLASS
    #define PSM_CPP_PRIVATE_FUNCTION(rval) PSM_PRIVATE_FUNCTION(rval)
    #define PSM_CPP_PRIVATE_CLASS PSM_PRIVATE_CLASS
#endif


#endif // PSMOVECLIENT_EXPORT_H