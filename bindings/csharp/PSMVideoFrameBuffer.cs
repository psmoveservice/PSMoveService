//------------------------------------------------------------------------------
// <auto-generated />
//
// This file was automatically generated by SWIG (http://www.swig.org).
// Version 3.0.12
//
// Do not make changes to this file unless you know what you are doing--modify
// the SWIG interface file instead.
//------------------------------------------------------------------------------

namespace PSMoveService {

public class PSMVideoFrameBuffer : global::System.IDisposable {
  private global::System.Runtime.InteropServices.HandleRef swigCPtr;
  protected bool swigCMemOwn;

  internal PSMVideoFrameBuffer(global::System.IntPtr cPtr, bool cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = new global::System.Runtime.InteropServices.HandleRef(this, cPtr);
  }

  internal static global::System.Runtime.InteropServices.HandleRef getCPtr(PSMVideoFrameBuffer obj) {
    return (obj == null) ? new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero) : obj.swigCPtr;
  }

  ~PSMVideoFrameBuffer() {
    Dispose();
  }

  public virtual void Dispose() {
    lock(this) {
      if (swigCPtr.Handle != global::System.IntPtr.Zero) {
        if (swigCMemOwn) {
          swigCMemOwn = false;
          PSMoveClientPINVOKE.delete_PSMVideoFrameBuffer(swigCPtr);
        }
        swigCPtr = new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero);
      }
      global::System.GC.SuppressFinalize(this);
    }
  }

  public System.IntPtr rgb_buffer { 
    get {
        System.IntPtr cPtr = PSMoveClientPINVOKE.PSMVideoFrameBuffer_rgb_buffer_get(swigCPtr); 
        return cPtr; 
   } 

  }

  public uint buffer_size_bytes {
    get {
      uint ret = PSMoveClientPINVOKE.PSMVideoFrameBuffer_buffer_size_bytes_get(swigCPtr);
      return ret;
    } 
  }

  public uint width {
    get {
      uint ret = PSMoveClientPINVOKE.PSMVideoFrameBuffer_width_get(swigCPtr);
      return ret;
    } 
  }

  public uint height {
    get {
      uint ret = PSMoveClientPINVOKE.PSMVideoFrameBuffer_height_get(swigCPtr);
      return ret;
    } 
  }

  public uint stride {
    get {
      uint ret = PSMoveClientPINVOKE.PSMVideoFrameBuffer_stride_get(swigCPtr);
      return ret;
    } 
  }

  public uint frame_index {
    get {
      uint ret = PSMoveClientPINVOKE.PSMVideoFrameBuffer_frame_index_get(swigCPtr);
      return ret;
    } 
  }

  public PSMVideoFrameBuffer() : this(PSMoveClientPINVOKE.new_PSMVideoFrameBuffer(), true) {
  }

}

}
