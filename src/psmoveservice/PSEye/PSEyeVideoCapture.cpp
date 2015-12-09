#include "PSEyeVideoCapture.h"
#ifdef HAVE_CLEYE
#include "libusb.h"
#include "DeviceInterfaceWin32.h"
const uint16_t VENDOR_ID = 0x1415;
const uint16_t PRODUCT_ID = 0x2000;
const char *CLEYE_DRIVER_PROVIDER_NAME= "Code Laboratories, Inc.";
#endif

#ifdef HAVE_PS3EYE
/**
 * Taken from the PS3EYEDriver OpenFrameworks example
 * written by Eugene Zatepyakin, MIT license
 **/

static const int ITUR_BT_601_CY = 1220542;
static const int ITUR_BT_601_CUB = 2116026;
static const int ITUR_BT_601_CUG = -409993;
static const int ITUR_BT_601_CVG = -852492;
static const int ITUR_BT_601_CVR = 1673527;
static const int ITUR_BT_601_SHIFT = 20;

static void
yuv422_to_bgr(const uint8_t *yuv_src, const int stride, uint8_t *dst, const int width, const int height)
{
    const int bIdx = 2;
    const int uIdx = 0;
    const int yIdx = 0;
    
    const int uidx = 1 - yIdx + uIdx * 2;
    const int vidx = (2 + uidx) % 4;
    int j, i;
    
#define _max(a, b) (((a) > (b)) ? (a) : (b))
#define _saturate(v) (uint8_t)((uint32_t)(v) <= 0xff ? v : v > 0 ? 0xff : 0)
    
    for (j = 0; j < height; j++, yuv_src += stride)
    {
        uint8_t* row = dst + (width * 3) * j;
        
        for (i = 0; i < 2 * width; i += 4, row += 6)
        {
            int u = (int)(yuv_src[i + uidx]) - 128;
            int v = (int)(yuv_src[i + vidx]) - 128;
            
            int ruv = (1 << (ITUR_BT_601_SHIFT - 1)) + ITUR_BT_601_CVR * v;
            int guv = (1 << (ITUR_BT_601_SHIFT - 1)) + ITUR_BT_601_CVG * v + ITUR_BT_601_CUG * u;
            int buv = (1 << (ITUR_BT_601_SHIFT - 1)) + ITUR_BT_601_CUB * u;
            
            int y00 = _max(0, (int)(yuv_src[i + yIdx]) - 16) * ITUR_BT_601_CY;
            row[2-bIdx] = _saturate((y00 + buv) >> ITUR_BT_601_SHIFT);
            row[1]      = _saturate((y00 + guv) >> ITUR_BT_601_SHIFT);
            row[bIdx]   = _saturate((y00 + ruv) >> ITUR_BT_601_SHIFT);
            
            int y01 = _max(0, (int)(yuv_src[i + yIdx + 2]) - 16) * ITUR_BT_601_CY;
            row[5-bIdx] = _saturate((y01 + buv) >> ITUR_BT_601_SHIFT);
            row[4]      = _saturate((y01 + guv) >> ITUR_BT_601_SHIFT);
            row[3+bIdx] = _saturate((y01 + ruv) >> ITUR_BT_601_SHIFT);
        }
    }
}
#undef _max
#undef _saturate
#endif //HAVE_PS3EYE

bool PSEyeVideoCapture::open(int index)
{
    if (isOpened()) release();

	// Try to open a PS3EYE-specific camera capture

#if !defined(CV_VERSION_EPOCH) || (CV_VERSION_EPOCH > 2)
    /* TODO: Consider using IVideoCapture
    icap = IVideoCapture_create(index);
    if (!icap.empty())
        return true;
    */
    cap.reset(pseyeCreateCameraCapture(index));
#else  // OpenCV2
    cap = pseyeCreateCameraCapture(index);
#endif
    
	// PS3EYE-specific camera capture if available, else default OpenCV object
    if (isOpened())
    {
        return isOpened();
    }
    else
    {
        std::cout << "Attempting cv::VideoCapture::open(index)" << std::endl;
        return cv::VideoCapture::open(index);
    }
}

CvCapture* PSEyeVideoCapture::pseyeCreateCameraCapture(int index)
{
    int pref = (index / 100) * 100;
    index -= pref;
    CvCapture *capture = 0; // no match
    
    switch (pref)
    {
        default:
            if (pref)
            {
                // Passed in a non-zero preference
                // that didn't match any switch cases
                break;
            }
            
#ifdef HAVE_CLEYE
        case PSEYE_CAP_CLMULTI:
            if (!capture)
            {
                std::cout << "Attempting pseyeCreateCameraCapture_CLMULTI" << std::endl;
                capture = pseyeCreateCameraCapture_CLMULTI(index);
            }
            if (pref)
            {
                break;
            }

        case PSEYE_CAP_CLEYE:
            if (!capture)
            {
                capture = pseyeCreateCameraCapture_CLEYE(index);
            }
            if (pref)
            {
                break;
            }

#endif //HAVE_CLEYE
#ifdef HAVE_PS3EYE
        case PSEYE_CAP_PS3EYE:
            if (!capture)
            {
                std::cout << "Attempting pseyeCreateCameraCapture_PS3EYE" << std::endl;
                capture = pseyeCreateCameraCapture_PS3EYE(index);
            }
            if (pref)
            {
                break;
            }
#endif //HAVE_PS3EYE
    }
    
    return capture;
}

#ifdef HAVE_CLEYE
CvCapture* PSEyeVideoCapture::pseyeCreateCameraCapture_CLMULTI(int index)
{
    PSEEYECaptureCAM_CLMULTI* capture = new PSEEYECaptureCAM_CLMULTI;
    try
    {
        if (capture->open(index))
        {
            return (CvCapture*)capture;
        }
    }
    catch (...)
    {
        delete capture;
        throw;
    }
    delete capture;
    return 0;
}

CvCapture* PSEyeVideoCapture::pseyeCreateCameraCapture_CLEYE(int index)
{
    PSEEYECaptureCAM_CLEYE* capture = new PSEEYECaptureCAM_CLEYE;
    try
    {
        if (capture->open(index))
        {
            return (CvCapture*)capture;
        }
            
    }
    catch(...)
    {
        delete capture;
        throw;
    }
    delete capture;
    return 0;
}

#endif
#ifdef HAVE_PS3EYE
CvCapture* PSEyeVideoCapture::pseyeCreateCameraCapture_PS3EYE(int index)
{
    PSEEYECaptureCAM_PS3EYE* capture = new PSEEYECaptureCAM_PS3EYE;
    try
    {
        if (capture->open(index))
        {
            return (CvCapture*)capture;
        }
            
    }
    catch (...)
    {
        delete capture;
        throw;
    }
    delete capture;
    return 0;
}
#endif



#ifdef HAVE_CLEYE
/*
* PSEEYECaptureCAM_CLMULTI
* Uses the CLEyeMulticam.dll
* Either uses the one that comes with this source and the user has their camera activated.
* Or the user has the CL Eye Platform SDK developer binaries installed and they delete
* the DLL that comes with this source.
*/
PSEEYECaptureCAM_CLMULTI::PSEEYECaptureCAM_CLMULTI() :
frame(NULL), frame4ch(NULL)
{
    
}

PSEEYECaptureCAM_CLMULTI::~PSEEYECaptureCAM_CLMULTI()
{
    close();
}

void PSEEYECaptureCAM_CLMULTI::close()
{
    if (openOK)
    {
        CLEyeCameraStop(eye);
        CLEyeDestroyCamera(eye);
    }
    cvReleaseImage(&frame);
    cvReleaseImage(&frame4ch);
}

// Initialize camera input
bool PSEEYECaptureCAM_CLMULTI::open(int _index)
{
    openOK = false;
    int cams = CLEyeGetCameraCount();
    std::cout << "CLEyeGetCameraCount() found " << cams << " devices." << std::endl;
    if (_index < cams)
    {
        GUID guid = CLEyeGetCameraUUID(_index);
        eye = CLEyeCreateCamera(guid, CLEYE_COLOR_PROCESSED, CLEYE_VGA, 75);
        CLEyeCameraGetFrameDimensions(eye, width, height);

        frame4ch = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 4);
        frame = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);

        CLEyeCameraStart(eye);
        CLEyeSetCameraParameter(eye, CLEYE_AUTO_EXPOSURE, false);
        CLEyeSetCameraParameter(eye, CLEYE_AUTO_GAIN, false);
        openOK = true;
    }
    return openOK;
}

bool PSEEYECaptureCAM_CLMULTI::grabFrame()
{
    cvGetRawData(frame4ch, &pCapBuffer, 0, 0);
    return true;
}

IplImage* PSEEYECaptureCAM_CLMULTI::retrieveFrame(int)
{
    CLEyeCameraGetFrame(eye, pCapBuffer, 2000);
    const int from_to[] = { 0, 0, 1, 1, 2, 2 };
    const CvArr** src = (const CvArr**)&frame4ch;
    CvArr** dst = (CvArr**)&frame;
    cvMixChannels(src, 1, dst, 1, from_to, 3);
    return frame;
}

double PSEEYECaptureCAM_CLMULTI::getProperty(int property_id) const
{
    int _width, _height;
    switch (property_id)
    {
    case CV_CAP_PROP_BRIGHTNESS:
        return (double)(CLEyeGetCameraParameter(eye, CLEYE_LENSBRIGHTNESS)); // [-500, 500]
    case CV_CAP_PROP_CONTRAST:
        return false;
    case CV_CAP_PROP_EXPOSURE:
        return (double)(CLEyeGetCameraParameter(eye, CLEYE_EXPOSURE)); // [0, 511]
    case CV_CAP_PROP_FPS:
        return (double)(60);
    case CV_CAP_PROP_FRAME_HEIGHT:
        CLEyeCameraGetFrameDimensions(eye, _width, _height);
        return (double)(_height);
    case CV_CAP_PROP_FRAME_WIDTH:
        CLEyeCameraGetFrameDimensions(eye, _width, _height);
        return (double)(_width);
    case CV_CAP_PROP_GAIN:
        return (double)(CLEyeGetCameraParameter(eye, CLEYE_GAIN)); // [0, 79]
    case CV_CAP_PROP_HUE:
        //return (double)(CLEyeGetCameraParameter(eye, CLEYE_EXPOSURE)); // [0 511]
        return false;
    case CV_CAP_PROP_SHARPNESS:
        //return (double)(CLEyeGetCameraParameter(eye, CLEYE_EXPOSURE)); // [0 511]
        return false;

    }
    return 0;
}
bool PSEEYECaptureCAM_CLMULTI::setProperty(int property_id, double value)
{
    if (!eye)
    {
        return false;
    }
    switch (property_id)
    {
    case CV_CAP_PROP_BRIGHTNESS:
        // [-500, 500]
        CLEyeSetCameraParameter(eye, CLEYE_LENSBRIGHTNESS, (int)value);
    case CV_CAP_PROP_CONTRAST:
        return false;
    case CV_CAP_PROP_EXPOSURE:
        CLEyeSetCameraParameter(eye, CLEYE_AUTO_EXPOSURE, value <= 0);
        if (value > 0)
        {
            //[0, 511]
            CLEyeSetCameraParameter(eye, CLEYE_EXPOSURE, (int)round(value));// (511 * value) / 0xFFFF));
        }
    case CV_CAP_PROP_FPS:
        return false; //TODO: Modifying FPS probably requires resetting the camera
    case CV_CAP_PROP_FRAME_HEIGHT:
        return false; //TODO: Modifying frame size probably requires resetting the camera
    case CV_CAP_PROP_FRAME_WIDTH:
        return false; //TODO: Modifying frame size probably requires resetting the camera
    case CV_CAP_PROP_GAIN:
        CLEyeSetCameraParameter(eye, CLEYE_AUTO_GAIN, value <= 0);
        if (value > 0)
        {
            //[0, 79]
            CLEyeSetCameraParameter(eye, CLEYE_GAIN, (int)round(value));// (79 * value) / 0xFFFF));
        }
    case CV_CAP_PROP_HUE:
        return false;
    case CV_CAP_PROP_SHARPNESS:
        return false; // TODO: Using OpenCV interface, sharpness appears to work
    }
    return true;
}

/*
* PSEEYECaptureCAM_CLEYE
* PS3EYE with CL Eye driver, not Platform SDK.
* Uses OpenCV for video, and registry for parameters.
*/
PSEEYECaptureCAM_CLEYE::PSEEYECaptureCAM_CLEYE():
frame(NULL)
{

}

PSEEYECaptureCAM_CLEYE::~PSEEYECaptureCAM_CLEYE()
{
    close();
}

void PSEEYECaptureCAM_CLEYE::close()
{
    cvReleaseImage(&frame);
}

// Initialize camera input
bool PSEEYECaptureCAM_CLEYE::open( int _index )
{
    // TODO: Use libusb to iterate through the devices
    // and see if one matches the VID and PID, and
    // also see that it is using the CL Eye Driver.

    bool cleyedriver_found = false;

    char provider_name[128];

    if (DeviceInterface::fetch_driver_reg_property_for_usb_device(
            DeviceInterface::Camera,
            VENDOR_ID, 
            PRODUCT_ID, 
            DeviceInterface::k_reg_property_provider_name, 
            provider_name, 
            sizeof(provider_name)))
    {
        cleyedriver_found= strcmp(provider_name, CLEYE_DRIVER_PROVIDER_NAME) == 0;
    }

    #if 0
    libusb_context *context = NULL;
    libusb_device **devs = NULL;
    int rc = 0;
    ssize_t count = 0;

    rc = libusb_init(&context);

    count = libusb_get_device_list(context, &devs);

    int idx = 0;
    for (idx = 0; idx < count; ++idx)
    {
        libusb_device *device = devs[idx];
        libusb_device_descriptor desc = { 0 };

        rc = libusb_get_device_descriptor(device, &desc);
        assert(rc == 0);

        if (desc.idVendor == VENDOR_ID && desc.idProduct == PRODUCT_ID)
        {
            // TODO: Check for cleye driver
            // Description will be "PS3Eye Camera"
            // Also note that the class:subclass is
            // camera -LIBUSB_CLASS_VENDOR_SPEC:LIBUSB_CLASS_PER_INTERFACE
            // audio - LIBUSB_CLASS_AUDIO:LIBUSB_CLASS_AUDIO
            // composite device - LIBUSB_CLASS_PER_INTERFACE:LIBUSB_CLASS_PER_INTERFACE
            //desc.iManufacturer
            libusb_device_handle *devhandle;
            int err = libusb_open(device, &devhandle);
            if (err == 0)
            {
                libusb_close(devhandle);
                //cleyedriver_found = true;
            }   
        }
    }
    libusb_free_device_list(devs, 1);
    #endif

    //return cv::VideoCapture::open(index);
    return cleyedriver_found;
}

bool PSEEYECaptureCAM_CLEYE::grabFrame()
{
    return false;
}

IplImage* PSEEYECaptureCAM_CLEYE::retrieveFrame(int)
{
    return frame;
}

double PSEEYECaptureCAM_CLEYE::getProperty( int property_id ) const
{
    return 0;
}
bool PSEEYECaptureCAM_CLEYE::setProperty( int property_id, double value )
{
    return false;
}

#endif

/*
 * PSEEYECaptureCAM_PS3EYE
 */
#ifdef HAVE_PS3EYE
PSEEYECaptureCAM_PS3EYE::PSEEYECaptureCAM_PS3EYE():
frame(NULL)
{
    //CoInitialize(NULL);
    
    // Enumerate libusb devices
    devices = ps3eye::PS3EYECam::getDevices();
    std::cout << "ps3eye::PS3EYECam::getDevices() found " << devices.size() << " devices." << std::endl;
}


PSEEYECaptureCAM_PS3EYE::~PSEEYECaptureCAM_PS3EYE()
{
    close();
    
    // eye is a smart pointer to a class.
    // When it goes out of scope, it should trigger the class destructor.
    
    //CoUninitialize();
}

void PSEEYECaptureCAM_PS3EYE::close()
{
    cvReleaseImage( &frame );
}

// Initialize camera input
bool PSEEYECaptureCAM_PS3EYE::open( int _index )
{
    if (devices.size() > _index) {

        eye = devices[_index];

        if (eye && eye->init(640, 480, 75))
        {
            // Change any default settings here

            frame = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);

            eye->start();

            eye->setAutogain(false);
            eye->setAutoWhiteBalance(false);

            return true;
        }
    }
    else {
        return false;
    }
}

bool PSEEYECaptureCAM_PS3EYE::grabFrame()
{
    ps3eye::PS3EYECam::updateDevices();
    return eye->isNewFrame();
}

IplImage* PSEEYECaptureCAM_PS3EYE::retrieveFrame(int)
{
    const unsigned char *new_pixels = eye->getLastFramePointer();
    
    widthStep = eye->getRowBytes();
    width = eye->getWidth();
    height = eye->getHeight();
    size = widthStep * height;
    
    // Convert pixels from camera to OpenCV BGR image
    unsigned char *cvpixels;
    cvGetRawData(frame, &cvpixels, 0, 0); // cvpixels becomes low-level access to frame
    if (new_pixels != NULL)
    {
        // Simultaneously copy from new_pixels to cvpixels and convert colorspace.
        yuv422_to_bgr(new_pixels, widthStep, cvpixels, width, height);
        
        // This seems like an opportunity for optimization.
        // Maybe we can use OpenCV tools to convert the memory format
        // (see cvInitImageHeader and cvSetData)
        // Then use OpenCV tools to convert the colorspace. e.g.,
        // cvtColor(src,dst,CV_YUV2BGR_YUY2);
    }
    return frame;
}

double PSEEYECaptureCAM_PS3EYE::getProperty( int property_id ) const
{
    switch( property_id )
    {
        case CV_CAP_PROP_BRIGHTNESS:
            return (double)(eye->getBrightness());
        case CV_CAP_PROP_CONTRAST:
            return (double)(eye->getContrast());
        case CV_CAP_PROP_EXPOSURE:
            // Default 120
            return (double)(eye->getExposure());
        case CV_CAP_PROP_FPS:
            return (double)(eye->getFrameRate());
        case CV_CAP_PROP_FRAME_HEIGHT:
            return (double)(eye->getHeight());
        case CV_CAP_PROP_FRAME_WIDTH:
            return (double)(eye->getWidth());
        case CV_CAP_PROP_GAIN:
            return (double)(eye->getGain());
        case CV_CAP_PROP_HUE:
            return (double)(eye->getHue());
        case CV_CAP_PROP_SHARPNESS:
            return (double)(eye->getSharpness());
        
    }
    return 0;
}
bool PSEEYECaptureCAM_PS3EYE::setProperty( int property_id, double value )
{
    if (!eye)
    {
        return false;
    }
    switch (property_id)
    {
        case CV_CAP_PROP_BRIGHTNESS:
            // 0 <-> 255 [20]
            eye->setBrightness((int)round(value));
        case CV_CAP_PROP_CONTRAST:
            // 0 <-> 255 [37]
            eye->setContrast((int)round(value));
        case CV_CAP_PROP_EXPOSURE:
            // 0 <-> 255 [120]
            eye->setExposure((int)round(value));
        case CV_CAP_PROP_FPS:
            return false; //TODO: Modifying FPS probably requires resetting the camera
        case CV_CAP_PROP_FRAME_HEIGHT:
            return false; //TODO: Modifying frame size probably requires resetting the camera
        case CV_CAP_PROP_FRAME_WIDTH:
            return false;
        case CV_CAP_PROP_GAIN:
            // 0 <-> 63 [20]
            eye->setGain((int)round(value));
        case CV_CAP_PROP_HUE:
           // 0 <-> 255 [143]
            eye->setHue((int)round(value));
        case CV_CAP_PROP_SHARPNESS:
            // 0 <-> 63 [0]
            eye->setSharpness((int)round(value));
    }
    return true;
}
#endif