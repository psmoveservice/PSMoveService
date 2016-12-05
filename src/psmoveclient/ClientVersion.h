#ifndef CLIENT_VERSION_H
#define CLIENT_VERSION_H

/// Conventional string-ification macro.
// From: http://stackoverflow.com/questions/5256313/c-c-macro-string-concatenation
#if !defined(PSM_STRINGIZE)
    #define PSM_STRINGIZEIMPL(x) #x
    #define PSM_STRINGIZE(x)     PSM_STRINGIZEIMPL(x)
#endif

#define PSM_PRODUCT_VERSION 0
#define PSM_MAJOR_VERSION   9
#define PSM_PHASE           alpha
#define PSM_MINOR_VERSION   5
#define PSM_PATCH_VERSION   5
#define PSM_BUILD_NUMBER    8

/// "Product.Major-Phase Minor.Patch"
#if !defined(PSM_VERSION_STRING)
    #define PSM_VERSION_STRING  PSM_STRINGIZE(PSM_PRODUCT_VERSION.PSM_MAJOR_VERSION-PSM_PHASE PSM_MINOR_VERSION.PSM_PATCH_VERSION)
#endif

/// "Product.Major-Phase Minor.Patch.Build"
#if !defined(PSM_DETAILED_VERSION_STRING)
    #define PSM_DETAILED_VERSION_STRING PSM_STRINGIZE(PSM_PRODUCT_VERSION.PSM_MAJOR_VERSION-PSM_PHASE PSM_MINOR_VERSION.PSM_PATCH_VERSION.PSM_BUILD_NUMBER)
#endif

#endif // CLIENT_VERSION_H
