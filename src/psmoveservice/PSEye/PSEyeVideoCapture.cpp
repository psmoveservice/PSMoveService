#include "PSEyeVideoCapture.h"
#ifdef HAVE_PS3EYE
#include "ps3eye.h"
#endif
#ifdef HAVE_CLEYE
#include "CLEyeMulticam.h"
#include "DeviceInterfaceWin32.h"
const uint16_t VENDOR_ID = 0x1415;
const uint16_t PRODUCT_ID = 0x2000;
const char *CLEYE_DRIVER_PROVIDER_NAME = "Code Laboratories, Inc.";
const char *CL_DRIVER_REG_PATH = "Software\\PS3EyeCamera\\Settings";  // [HKCU]
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

/*
We need a base class 
Note: See definition of CvCapture here:
https://github.com/Itseez/opencv/blob/09e6c82190b558e74e2e6a53df09844665443d6d/modules/videoio/src/precomp.hpp#L84-L94
This is very convenient as a base for custom capture devices.
Unfortunately, it does not have a public interface, so we redefine it here.

cv:VideoCapture has a member variable cv::Ptr<CvCapture> that we are hijacking with one of our custom implementations below.
*/
struct CvCapture
{
    virtual ~CvCapture() {}
    virtual double getProperty(int) const { return 0; }
    virtual bool setProperty(int, double) { return 0; }
    virtual bool grabFrame() { return true; }
    virtual IplImage* retrieveFrame(int) { return 0; }
    virtual int getCaptureDomain() { return CV_CAP_ANY; } // Return the type of the capture object: CV_CAP_VFW, etc...
};

// TODO: Convert below to sub-classes of cv::IVideoCapture
/*
// https://github.com/Itseez/opencv/blob/09e6c82190b558e74e2e6a53df09844665443d6d/modules/videoio/src/precomp.hpp#L164-L176
This is very convenient as a base for custom capture devices.
Unfortunately, it does not have a public interface, so we redefine it here.
*/

class cv::IVideoCapture
{
public:
    virtual ~IVideoCapture() {}
    virtual double getProperty(int) const { return 0; }
    virtual bool setProperty(int, double) { return false; }
    virtual bool grabFrame() = 0;
    virtual bool retrieveFrame(int, OutputArray) = 0;
    virtual bool isOpened() const = 0;
    virtual int getCaptureDomain() { return CAP_ANY; } // Return the type of the capture object: CAP_VFW, etc...
};

/*
-- Camera-specific implementations of CvCapture --
*/

#ifdef HAVE_CLEYE
/// Implementation of CvCapture when using CLEyeMulticam.dll
/**
Either uses the DLL that comes with PSMoveService and the user has their camera activated.
Or the user has the CL Eye Platform SDK developer binaries installed and they delete
the DLL that comes with PSMoveService.
*/
class PSEYECaptureCAM_CLMULTI : public cv::IVideoCapture
{
public:
    PSEYECaptureCAM_CLMULTI() :
        frame(NULL), frame4ch(NULL)
    {
    }

    ~PSEYECaptureCAM_CLMULTI()
    {
        close();
    }

    bool open(int _index) {
        openOK = false;
        int cams = CLEyeGetCameraCount();
        std::cout << "CLEyeGetCameraCount() found " << cams << " devices." << std::endl;
        if (_index < cams)
        {
            std::cout << "Attempting to open camera " << _index << " of " << cams << "." << std::endl;
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

    void close()
    {
        if (openOK)
        {
            CLEyeCameraStop(eye);
            CLEyeDestroyCamera(eye);
        }
        cvReleaseImage(&frame);
        cvReleaseImage(&frame4ch);
    }

    double getProperty(int property_id) const
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

    bool setProperty(int property_id, double value)
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

    bool grabFrame()
    {
        cvGetRawData(frame4ch, &pCapBuffer, 0, 0);
        return true;
    }

    IplImage* retrieveFrame(int)
    {
        CLEyeCameraGetFrame(eye, pCapBuffer, 2000);
        const int from_to[] = { 0, 0, 1, 1, 2, 2 };
        const CvArr** src = (const CvArr**)&frame4ch;
        CvArr** dst = (CvArr**)&frame;
        cvMixChannels(src, 1, dst, 1, from_to, 3);
        return frame;
    }

    int getCaptureDomain() { return PSEYE_CAP_CLMULTI; }

protected:
    void init();
    int index, width, height, fourcc;
    bool openOK;
    PBYTE pCapBuffer;
    IplImage* frame;
    IplImage* frame4ch;
    CLEyeCameraInstance eye;
};

/// Implementation of CvCapture when using CL Eye Driver
/*
* Uses OpenCV for video, and registry for parameters.
*/
class PSEYECaptureCAM_CLEYE : public CvCapture
{
public:

    PSEYECaptureCAM_CLEYE(){}

    ~PSEYECaptureCAM_CLEYE() { close(); }

    /// Ape cap.reset(cvCreateCameraCapture(index));
    bool open(int _index)
    {
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
            cleyedriver_found = strcmp(provider_name, CLEYE_DRIVER_PROVIDER_NAME) == 0;
        }

        if (cleyedriver_found)
        {
            CvCapture* tmp = cvCreateCameraCapture(_index);
            _cap.reset(tmp);
            index = _index;
            return true;
        }
        else
        {
            return false;
        }
    }

    void close() {  }

    double getProperty(int property_id) const
    {
        HKEY hKey;
        DWORD l = sizeof(DWORD);
        DWORD result = 0;

        int err = RegOpenKeyExA(HKEY_CURRENT_USER, CL_DRIVER_REG_PATH, 0, KEY_ALL_ACCESS, &hKey);
        if (err != ERROR_SUCCESS) {
            printf("Error: %d Unable to open reg-key:  [HKCU]\\%s!\n", err, CL_DRIVER_REG_PATH);
            printf("CL-Eye Test must be run at least once for each Windows user.\n");
            return false;
        }
        
        switch (property_id)
        {
        case CV_CAP_PROP_BRIGHTNESS:
            return false;
        case CV_CAP_PROP_CONTRAST:
            return false;
        case CV_CAP_PROP_EXPOSURE:
            RegQueryValueExA(hKey, "AutoAEC", NULL, NULL, (LPBYTE)&result, &l);
            if (result < 1)
            {
                RegQueryValueExA(hKey, "Exposure", NULL, NULL, (LPBYTE)&result, &l);
                return (double)(result);
            }
            else
            {
                return (double)0;
            }

        case CV_CAP_PROP_FPS:
            return (double)(60);
        case CV_CAP_PROP_FRAME_HEIGHT:
            return (double)480;
        case CV_CAP_PROP_FRAME_WIDTH:
            return (double)640;
        case CV_CAP_PROP_GAIN:
            RegQueryValueExA(hKey, "AutoAGC", NULL, NULL, (LPBYTE)&result, &l);
            if (result < 1)
            {
                RegQueryValueExA(hKey, "Gain", NULL, NULL, (LPBYTE)&result, &l);
                return (double)(result);
            }
            else
            {
                return (double)0;
            }
        default:
            return _cap->getProperty(property_id);
        }
        return 0;
    }

    bool setProperty(int property_id, double value)
    {
        bool param_set = false;

        int val;
        HKEY hKey;
        DWORD l = sizeof(DWORD);
        int err = RegOpenKeyExA(HKEY_CURRENT_USER, CL_DRIVER_REG_PATH, 0, KEY_ALL_ACCESS, &hKey);
        if (err != ERROR_SUCCESS) {
            printf("Error: %d Unable to open reg-key:  [HKCU]\\%s!\n", err, CL_DRIVER_REG_PATH);
            printf("CL-Eye Test must be run at least once for each Windows user.\n");
            return false;
        }

        switch (property_id)
        {
            case CV_CAP_PROP_EXPOSURE:
                val = value > 0;
                RegSetValueExA(hKey, "AutoAEC", 0, REG_DWORD, (CONST BYTE*)&val, l);
                val = (int)round((511 * value) / 0xFFFF);
                RegSetValueExA(hKey, "Exposure", 0, REG_DWORD, (CONST BYTE*)&val, l);
                param_set = true;
            case CV_CAP_PROP_FPS:
                return false;
            case CV_CAP_PROP_GAIN:
                val = value > 0;
                RegSetValueExA(hKey, "AutoAGC", 0, REG_DWORD, (CONST BYTE*)&val, l);
                val = (int)round((79 * value) / 0xFFFF);
                RegSetValueExA(hKey, "Gain", 0, REG_DWORD, (CONST BYTE*)&val, l);
            case CV_CAP_PROP_HUE:
#if 0
                val = value > 0;
                RegSetValueExA(hKey, "AutoAWB", 0, REG_DWORD, (CONST BYTE*)&val, l);
                val = 1234;
                RegSetValueExA(hKey, "ColorBar", 0, REG_DWORD, (CONST BYTE*)&val, l);
                val = (int)round((255 * value) / 0xFFFF);
                RegSetValueExA(hKey, "WhiteBalanceR", 0, REG_DWORD, (CONST BYTE*)&val, l);
                val = (int)round((255 * value) / 0xFFFF);
                RegSetValueExA(hKey, "WhiteBalanceG", 0, REG_DWORD, (CONST BYTE*)&val, l);
                val = (int)round((255 * value) / 0xFFFF);
                RegSetValueExA(hKey, "WhiteBalanceB", 0, REG_DWORD, (CONST BYTE*)&val, l);
#endif
                return false;
            default:
                return cvSetCaptureProperty(_cap, property_id, value);
        }

        // restart the camera capture
        if (param_set && _cap) {
            _cap.reset(cvCreateCameraCapture(index));
        }

        return param_set;
    }

    bool grabFrame()
    {
        return (_cap && _cap->grabFrame());
    }

    IplImage* retrieveFrame(int _idx)
    {
        return _cap->retrieveFrame(_idx);
    }

    int getCaptureDomain() { return PSEYE_CAP_CLEYE; }

protected:
    void init();
    int index, width, height;
    cv::Ptr<CvCapture> _cap;
};
#endif

#ifdef HAVE_PS3EYE
// Implementation of PS3EyeCapture when using PS3EYEDriver
class PSEYECaptureCAM_PS3EYE : public CvCapture
{
public:
    PSEYECaptureCAM_PS3EYE() : frame(NULL)
    {
        //CoInitialize(NULL);

        // Enumerate libusb devices
        devices = ps3eye::PS3EYECam::getDevices();
        std::cout << "ps3eye::PS3EYECam::getDevices() found " << devices.size() << " devices." << std::endl;
    }

    ~PSEYECaptureCAM_PS3EYE() { close(); }

    bool open(int _index)
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
        return false;
    }

    virtual void close() { cvReleaseImage(&frame); }

    double getProperty(int property_id) const
    {
        switch (property_id)
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

    virtual bool setProperty(int property_id, double value)
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

    bool grabFrame()
    {
        ps3eye::PS3EYECam::updateDevices();
        return eye->isNewFrame();
    }

    IplImage* retrieveFrame(int)
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

    int getCaptureDomain() { return PSEYE_CAP_PS3EYE; }

protected:
    void init();
    std::vector<ps3eye::PS3EYECam::PS3EYERef> devices;
    int index, width, height, widthStep;
    IplImage* frame;
    ps3eye::PS3EYECam::PS3EYERef eye;
    size_t size;
};

#endif


/*
-- Definitions for PSEyeVideoCapture --
*/
bool PSEyeVideoCapture::open(int index)
{
    if (isOpened())
    {
        release();
    }

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
    
	// PS3EYE-specific camera capture if available, else call base OpenCV open()
    if (!isOpened())
    {
        std::cout << "Attempting cv::VideoCapture::open(index)" << std::endl;
        //cap.reset(cvCreateCameraCapture(index));
        return cv::VideoCapture::open(index);
    }
    return isOpened();
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
                std::cout << "Attempting pseyeCreateCameraCapture_CLEYE" << std::endl;
                capture = cvCreateCameraCapture(index);
                //capture = pseyeCreateCameraCapture_CLEYE(index);
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
                //capture = pseyeCreateCameraCapture_PS3EYE(index);
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
    PSEYECaptureCAM_CLMULTI* capture = new PSEYECaptureCAM_CLMULTI;
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
    PSEYECaptureCAM_CLEYE* capture = new PSEYECaptureCAM_CLEYE;
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
    PSEYECaptureCAM_PS3EYE* capture = new PSEYECaptureCAM_PS3EYE;
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