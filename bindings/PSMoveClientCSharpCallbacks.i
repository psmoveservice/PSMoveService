// Define a typemap macro used to bind a C# delegate to a C STDCALL callback function type
%define %cs_callback(TYPE, CSTYPE)
    %typemap(ctype) TYPE, TYPE& "void *"
    %typemap(in) TYPE  %{ $1 = ($1_type)$input; %}
    %typemap(in) TYPE& %{ $1 = ($1_type)&$input; %}
    %typemap(imtype, out="IntPtr") TYPE, TYPE& "CSTYPE"
    %typemap(cstype, out="IntPtr") TYPE, TYPE& "CSTYPE"
    %typemap(csin) TYPE, TYPE& "$csinput"
%enddef

// Insert the C# delegate type into the PSMoveClient class
%pragma(csharp) modulecode=%{
  public delegate void PSMResponseCallback(PSMResponseMessage message, System.IntPtr callback_userdata); 
%}

// Ignore PSM_RegisterCallback since it relies on a CDECL, which C# can't use
%ignore PSM_RegisterCallback;

// Rename PSM_RegisterSTDCALLCallback -> PSM_RegisterDelegate
%rename(PSM_RegisterSTDCALLCallback) PSM_RegisterDelegate;

// Register function pointer types in the typemap
%cs_callback(PSMResponseCallback_STDCALL, PSMoveClient.PSMResponseCallback)