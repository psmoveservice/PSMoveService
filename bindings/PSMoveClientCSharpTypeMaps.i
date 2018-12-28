%include "arrays_csharp.i"
%include "typemaps.i"

//------------------------------TypeMaps--------------------------
//Swig typemaps help to either convert a C++ type into a C# type or tell it which types are the same
//Swig typemaps also can modify the postprocessed C++ file as well as the postprocessed C# files
//For example the cscode typemap is used extensively to write C# code into the postprocessed file
//----------------------------------------------------------------

//------------------------------Simple Typemaps--------------------------
// Remap C-style char * -> C# string
CSHARP_ARRAYS(char *,string)

%typemap(ctype)  void* "void *"
%typemap(imtype) void* "System.IntPtr"
%typemap(cstype) void* "System.IntPtr"
%typemap(csin)   void* "$csinput"
%typemap(in)     void* %{ $1 = $input; %}
%typemap(out)    void* %{ $result = $1; %}
%typemap(csout, excode=SWIGEXCODE)  void* { 
    System.IntPtr cPtr = $imcall;$excode
    return cPtr;
    }
%typemap(csvarout, excode=SWIGEXCODE2) void* %{ 
    get {
        System.IntPtr cPtr = $imcall;$excode 
        return cPtr; 
   } 
%} 

%apply bool *OUTPUT { bool *out_is_stable };
%apply bool *OUTPUT { bool *out_is_tracking };
%apply int *OUTPUT { int *out_tracker_id };
%apply int *OUTPUT { int *out_request_id };
%apply float *OUTPUT { float *out_length };
%apply float *OUTPUT { float *out_rumble_fraction };

//------------------------------Complex Typemaps--------------------------

/*
  Macro used to create a sizeof helper for the given structure.
  These helper functions are needed by CUSTOM_READONLY_STRUCT_ARRAY_PROPERTY.
  
  STRUCT_TYPE - The C# structure type
*/
%define MAKE_STRUCT_SIZEOF_ACCESSOR(STRUCT_TYPE)
%{
size_t STRUCT_TYPE ## _getSize()
{
  return sizeof(STRUCT_TYPE);
}
%}
size_t STRUCT_TYPE ## _getSize();
%enddef

/*
  Macro to marshall a primitive type array property from C to C#.
  NOTE: This makes a new copy of the managed copy of the primitive type array every time this accessor is called.
  
  PROPERTY_NAME - The array property name we want exposed
  C_TYPE - The C primitive array type
  CS_TYPE - The corresponding C# primitive array type
  IN_DATA_GET_FUNCTION - The swig-generated p/invoke call that fetches the array pointer (ex: PSMVirtualController_axisStates_get)
  LENGTH_EXPRESSION - Expression used to determine the length of the array
*/
%define CUSTOM_READONLY_PRIMITIVE_TYPE_ARRAY_PROPERTY(PROPERTY_NAME,C_TYPE,CS_TYPE,IN_DATA_GET_FUNCTION,LENGTH_EXPRESSION)
  %typemap(cstype, out="CS_TYPE[]") C_TYPE PROPERTY_NAME[ANY] "CS_TYPE[]"
  %typemap(csvarout, excode=SWIGEXCODE2) C_TYPE PROPERTY_NAME[ANY]
  %{
    get {
      System.IntPtr cPtr = PSMoveClientPINVOKE.IN_DATA_GET_FUNCTION(swigCPtr);
      int len = (int)LENGTH_EXPRESSION;
      if (len<=0)
      {
        return null;
      }
      CS_TYPE[] returnArray = new CS_TYPE[len];
      System.Runtime.InteropServices.Marshal.Copy(cPtr, returnArray, 0, len);
       
      return returnArray;
    }
  %}
%enddef

/*
  Macro to marshall a enum type array property from C to C#.
  NOTE: This makes a new copy of the managed copy of the enum type array every time this accessor is called.
  
  PROPERTY_NAME - The array property name we want exposed
  ENUM_TYPE - The corresponding C# primitive array type
  IN_DATA_GET_FUNCTION - The swig-generated p/invoke call that fetches the array pointer (ex: PSMVirtualController_axisStates_get)
  LENGTH_EXPRESSION - Expression used to determine the length of the array
*/
%define CUSTOM_READONLY_ENUM_TYPE_ARRAY_PROPERTY(PROPERTY_NAME,ENUM_TYPE,IN_DATA_GET_FUNCTION,LENGTH_EXPRESSION)
  %typemap(cstype, out="ENUM_TYPE[]") ENUM_TYPE PROPERTY_NAME[ANY] "ENUM_TYPE[]"
  %typemap(csvarout, excode=SWIGEXCODE2) ENUM_TYPE PROPERTY_NAME[ANY]
  %{
    get {
      System.IntPtr cPtr = PSMoveClientPINVOKE.IN_DATA_GET_FUNCTION(swigCPtr);
      int len = (int)LENGTH_EXPRESSION;
      if (len<=0)
      {
        return null;
      }
      ENUM_TYPE[] returnArray = new ENUM_TYPE[len];
      byte[] intermediateArray = new byte[len];
      System.Runtime.InteropServices.Marshal.Copy(cPtr, intermediateArray, 0, len);
      for (int i = 0; i < len; ++i)
      {
        returnArray[i] = (ENUM_TYPE)intermediateArray[i];
      }
       
      return returnArray;
    }
  %}
%enddef

/*
  Macro to marshall a primitive type buffer property from C to C#.
  NOTE: This makes a new copy of the managed copy of the primitive type buffer every time this accessor is called.
  
  PROPERTY_NAME - The array property name we want exposed
  C_TYPE - The C primitive array type
  CS_TYPE - The corresponding C# primitive array type
  IN_DATA_GET_FUNCTION - The swig-generated p/invoke call that fetches the array pointer (ex: PSMVirtualController_axisStates_get)
  LENGTH_EXPRESSION - Expression used to determine the length of the array
*/
%define CUSTOM_READONLY_PRIMITIVE_TYPE_BUFFER_PROPERTY(PROPERTY_NAME,C_TYPE,CS_TYPE,IN_DATA_GET_FUNCTION,LENGTH_EXPRESSION)
  %typemap(cstype, out="CS_TYPE[]") C_TYPE* PROPERTY_NAME "CS_TYPE[]"
  %typemap(csvarout, excode=SWIGEXCODE2) C_TYPE* PROPERTY_NAME
  %{
    get {
      System.IntPtr cPtr = PSMoveClientPINVOKE.IN_DATA_GET_FUNCTION(swigCPtr);
      int len = (int)LENGTH_EXPRESSION;
      if (len<=0)
      {
        return null;
      }
      CS_TYPE[] returnArray = new CS_TYPE[len];
      System.Runtime.InteropServices.Marshal.Copy(cPtr, returnArray, 0, len);
       
      return returnArray;
    }
  %}
%enddef

/*
  Macro to marshall a struct array property from C to C#.
  NOTE: This makes a new copy of the managed copy of the struct array every time this accessor is called.
  
  PROPERTY_NAME - The array property name we want exposed
  STRUCT_TYPE - The C# structure type
  IN_DATA_GET_FUNCTION - The swig-generated p/invoke call that fetches the array pointer (ex: PSMTrackerList_trackers_get)
  LENGTH_EXPRESSION - Expression used to determine the length of the array
*/
%define CUSTOM_READONLY_STRUCT_ARRAY_PROPERTY(PROPERTY_NAME,STRUCT_TYPE,IN_DATA_GET_FUNCTION,LENGTH_EXPRESSION) 
  %typemap(cstype, out="STRUCT_TYPE[]") STRUCT_TYPE PROPERTY_NAME[ANY] "STRUCT_TYPE[]"
  %typemap(csvarout, excode=SWIGEXCODE2) STRUCT_TYPE PROPERTY_NAME[ANY]
  %{
    get {
      STRUCT_TYPE[] returnArray;
      
      int structSize = (int)PSMoveClientPINVOKE. ## STRUCT_TYPE ## _getSize();
      System.IntPtr cPtr = PSMoveClientPINVOKE.IN_DATA_GET_FUNCTION(swigCPtr);
      int len = LENGTH_EXPRESSION;
      if (len<=0)
      {
        return null;
      }
      returnArray = new STRUCT_TYPE[len];
      for (int i = 0; i < len; ++i)
      {
          System.IntPtr data = new System.IntPtr(cPtr.ToInt64() + structSize * i);
          returnArray[i] = new STRUCT_TYPE(data, false);        
      }

      return returnArray;
    }
  %}
%enddef

// Make sizeof() helpers for the array accessors
MAKE_STRUCT_SIZEOF_ACCESSOR(PSMClientControllerInfo);
MAKE_STRUCT_SIZEOF_ACCESSOR(PSMClientHMDInfo);
MAKE_STRUCT_SIZEOF_ACCESSOR(PSMClientTrackerInfo);
MAKE_STRUCT_SIZEOF_ACCESSOR(PSMVector2f);

// Marshall the video buffer on PSMVideoFrameBuffer from unmanaged to managed memory
CUSTOM_READONLY_PRIMITIVE_TYPE_BUFFER_PROPERTY(
  rgb_buffer, 
  unsigned char,
  byte, 
  PSMVideoFrameBuffer_rgb_buffer_get,
  this.buffer_size_bytes);

// Marshall the axis state array on PSMVirtualController from unmanaged to managed memory
CUSTOM_READONLY_PRIMITIVE_TYPE_ARRAY_PROPERTY(
  axisStates, 
  unsigned char,
  byte, 
  PSMVirtualController_axisStates_get,
  this.numAxes);
  
// Marshall the button state array on PSMVirtualController from unmanaged to managed memory
CUSTOM_READONLY_ENUM_TYPE_ARRAY_PROPERTY(
  buttonStates, 
  PSMButtonState,
  PSMVirtualController_buttonStates_get,
  this.numButtons);

// Marshall the controller list on PSMControllerList from unmanaged to managed memory
CUSTOM_READONLY_STRUCT_ARRAY_PROPERTY(
  controllers,
  PSMClientControllerInfo,
  PSMControllerList_controllers_get,
  this.count);
  
// Marshall the hmd list on PSMHmdList from unmanaged to managed memory
CUSTOM_READONLY_STRUCT_ARRAY_PROPERTY(
  hmds,
  PSMClientHMDInfo,
  PSMHmdList_hmds_get,
  this.count);
  
// Marshall the tracker list on PSMTrackerList from unmanaged to managed memory
CUSTOM_READONLY_STRUCT_ARRAY_PROPERTY(
  trackers,
  PSMClientTrackerInfo,
  PSMTrackerList_trackers_get,
  this.count);

// Marshall the triangle list on PSMTrackingProjection_shape_lightbar from unmanaged to managed memory
CUSTOM_READONLY_STRUCT_ARRAY_PROPERTY(
  triangle,
  PSMVector2f,
  PSMTrackingProjection_shape_lightbar_triangle_get,
  3);
  
// Marshall the quad list on PSMTrackingProjection_shape_lightbar from unmanaged to managed memory
CUSTOM_READONLY_STRUCT_ARRAY_PROPERTY(
  quad,
  PSMVector2f,
  PSMTrackingProjection_shape_lightbar_quad_get,
  4);
  
// Marshall the point list on PSMTrackingProjection_shape_pointcloud from unmanaged to managed memory
CUSTOM_READONLY_STRUCT_ARRAY_PROPERTY(
  points,
  PSMVector2f,
  PSMTrackingProjection_shape_pointcloud_points_get,
  this.point_count); 