#ifndef PROTOCOL_VERSION_H
#define PROTOCOL_VERSION_H

/// Conventional string-ification macro.
// From: http://stackoverflow.com/questions/5256313/c-c-macro-string-concatenation
#if !defined(PSM_STRINGIZE)
    #define PSM_STRINGIZEIMPL(x) #x
    #define PSM_STRINGIZE(x)     PSM_STRINGIZEIMPL(x)
#endif

#define PSM_VERSION_PRODUCT 0
#define PSM_VERSION_MAJOR   9
#define PSM_VERSION_PHASE   alpha
#define PSM_VERSION_MINOR   8
#define PSM_VERSION_RELEASE 5
#define PSM_VERSION_HOTFIX  0

/// "Product.Major-Phase Minor.Release"
#if !defined(PSM_VERSION_STRING)
    #define PSM_VERSION_STRING  PSM_STRINGIZE(PSM_VERSION_PRODUCT.PSM_VERSION_MAJOR-PSM_VERSION_PHASE PSM_VERSION_MINOR.PSM_VERSION_RELEASE)
#endif

/// "Product.Major-Phase Minor.Release.Hotfix"
#if !defined(PSM_DETAILED_VERSION_STRING)
    #define PSM_DETAILED_VERSION_STRING PSM_STRINGIZE(PSM_VERSION_PRODUCT.PSM_VERSION_MAJOR-PSM_VERSION_PHASE PSM_VERSION_MINOR.PSM_VERSION_RELEASE.PSM_VERSION_HOTFIX)
#endif

#endif // PROTOCOL_VERSION_H
