#ifndef PSMOVECLIENT_EXPORT_H
#define PSMOVECLIENT_EXPORT_H

/** \def PSM_CALL
 * \ingroup psm_client_misc
 * PSMoveService's Windows calling convention.
 *
 * Under Windows, the selection of available compilers and configurations
 * means that, unlike other platforms, there is not <em>one true calling
 * convention</em> (calling convention: the manner in which parameters are
 * passed to functions in the generated assembly code).
 *
 * Matching the Windows API itself, PSMoveService's client API uses the 
 * __cdecl convention and guarantees that the library is compiled in this way. 
 * The public header file also includes appropriate annotations so that 
 * your own software will use the right convention, even if another convention 
 * is being used by default within your codebase.
 *
 * The one consideration that you must apply in your software is to mark
 * all functions which you use as PSMResponseCallbacks with this PSM_CALL
 * annotation, so that they too get compiled for the correct calling
 * convention.
 *
 * On non-Windows operating systems, this macro is defined as nothing. This
 * means that you can apply it to your code without worrying about
 * cross-platform compatibility.
 *
 * When exposing C-Style global variables from the client dll make sure to use:
 *  - "PSM_PUBLIC_VARIABLE_DECL(variable_type) variable_name" in the header file 
 *  - "PSM_PUBLIC_VARIABLE_DEF(variable_type) variable_name" in the cpp/c file
 * https://stackoverflow.com/questions/3097548/exporting-global-variables-from-dll
 */
#ifndef PSM_CALL
    #if (defined _WIN32 || defined __CYGWIN__)
        #define PSM_CALL __cdecl
    #else
        #define PSM_CALL
    #endif
#endif

#ifndef PSM_EXTERN_C
    #if defined(__cplusplus)
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
            #define PSM_PUBLIC_VARIABLE_DECL(rval)  PSM_EXTERN_C    __declspec(dllexport)            extern rval
            #define PSM_PUBLIC_VARIABLE_DEF(rval)   PSM_EXTERN_C    __declspec(dllexport)                   rval
            #define PSM_PRIVATE_FUNCTION(rval)                                                              rval    PSM_CALL
            #define PSM_PRIVATE_CLASS
        #else  // Not Windows
            #if __GNUC__ >= 4
                #define PSM_PUBLIC_FUNCTION(rval)       PSM_EXTERN_C    __attribute__((visibility("default")))         rval    PSM_CALL
                #define PSM_PUBLIC_CLASS                                __attribute__((visibility("default")))
                #define PSM_PUBLIC_VARIABLE_DECL(rval)  PSM_EXTERN_C    __attribute__((visibility("default")))  extern rval
                #define PSM_PUBLIC_VARIABLE_DEF(rval)   PSM_EXTERN_C    __attribute__((visibility("default")))         rval
            #else
                #define PSM_PUBLIC_FUNCTION(rval)       PSM_EXTERN_C           rval PSM_CALL
                #define PSM_PUBLIC_CLASS
                #define PSM_PUBLIC_VARIABLE_DECL(rval)  PSM_EXTERN_C    extern rval
                #define PSM_PUBLIC_VARIABLE_DEF(rval)   PSM_EXTERN_C           rval
            #endif
            #define PSM_PRIVATE_FUNCTION(rval)                      __attribute__((visibility("hidden")))   rval    PSM_CALL
            #define PSM_PRIVATE_CLASS                               __attribute__((visibility("hidden")))
        #endif  //defined _WIN32 || defined __CYGWIN__
    #elif defined(PSMoveClient_STATIC) || defined(PSMoveClient_CAPI_STATIC)  // Building static lib
        #define PSM_PUBLIC_FUNCTION(rval)           PSM_EXTERN_C                                            rval    PSM_CALL
        #define PSM_PUBLIC_CLASS
        #define PSM_PUBLIC_VARIABLE_DECL(rval)      PSM_EXTERN_C                                     extern rval
        #define PSM_PUBLIC_VARIABLE_DEF(rval)       PSM_EXTERN_C                                            rval
        #define PSM_PRIVATE_FUNCTION(rval)                                                                  rval    PSM_CALL
        #define PSM_PRIVATE_CLASS
    #else //This DLL/so/dylib is being imported
        #if defined _WIN32 || defined __CYGWIN__
            #define PSM_PUBLIC_FUNCTION(rval)       PSM_EXTERN_C    __declspec(dllimport)                   rval    PSM_CALL
            #define PSM_PUBLIC_CLASS                                __declspec(dllimport)
            #define PSM_PUBLIC_VARIABLE_DECL(rval)  PSM_EXTERN_C    __declspec(dllimport)            extern rval
            #define PSM_PUBLIC_VARIABLE_DEF(rval)   PSM_EXTERN_C    __declspec(dllimport)                   rval
        #else  // Not Windows
            #define PSM_PUBLIC_FUNCTION(rval)       PSM_EXTERN_C                                            rval    PSM_CALL
            #define PSM_PUBLIC_CLASS
            #define PSM_PUBLIC_VARIABLE_DECL(rval)  PSM_EXTERN_C                                     extern rval
            #define PSM_PUBLIC_VARIABLE_DEF(rval)   PSM_EXTERN_C                                            rval
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