// Define a typemap macro used to bind a C# delegate to a C STDCALL callback function type
%define %cs_callback(TYPE, CSTYPE)
    %typemap(ctype) TYPE, TYPE& "void *"
    %typemap(in) TYPE  %{ $1 = ($1_type)$input; %}
    %typemap(in) TYPE& %{ $1 = ($1_type)&$input; %}
    %typemap(imtype, inattributes="[global::System.Runtime.InteropServices.MarshalAs(global::System.Runtime.InteropServices.UnmanagedType.FunctionPtr)]", out="IntPtr") TYPE, TYPE& "CSTYPE"
    %typemap(cstype, out="IntPtr") TYPE, TYPE& "CSTYPE"
    %typemap(csin) TYPE, TYPE& "$csinput"
%enddef

// Insert the C# delegate type into the PSMoveClient class
%pragma(csharp) modulecode=%{
  public delegate void PSMResponseDelegate(PSMResponseMessage message);   

  [global::System.Runtime.InteropServices.UnmanagedFunctionPointer(global::System.Runtime.InteropServices.CallingConvention.Cdecl)]
  public delegate void PSMResponseCallback(System.IntPtr message_ptr, System.IntPtr callback_userdata); 
  
  public static PSMResult PSM_RegisterDelegate(int request_id, PSMoveClient.PSMResponseDelegate response_delegate) {
    PSMResponseCallback callback = (message_ptr, callback_userdata) => {
      response_delegate(new PSMResponseMessage(message_ptr, false));
    };
   
    PSMResult ret = (PSMResult)PSMoveClientPINVOKE.PSM_RegisterCallback(request_id, callback, System.IntPtr.Zero);
    return ret;
  }  
%}

// Register function pointer types in the typemap
//%cs_callback(PSMResponseCallback_STDCALL, PSMoveClient.PSMResponseCallback)
%cs_callback(PSMResponseCallback_CDECL, PSMoveClient.PSMResponseCallback)