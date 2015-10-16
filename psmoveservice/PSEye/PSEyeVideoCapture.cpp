#include "PSEyeVideoCapture.h"

bool PSEyeVideoCapture::open(int index)
{
    if (isOpened()) release();
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
    
    return isOpened() ? isOpened() : cv::VideoCapture::open(index);
}

CvCapture* PSEyeVideoCapture::pseyeCreateCameraCapture(int index)
{
    int pref = (index / 100) * 100;
    index -= pref;
    CvCapture *capture = 0;
    
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

/*
CvCapture* PSEyeVideoCapture::pseyeCreateCameraCapture_CLMULTI(int index)
{
    PSEEYECaptureCAM_CLMULTI* capture = new PSEEYECaptureCAM_CLMULTI;
    try
    {
        if( capture->open( index ))
            return (CvCapture*)capture;
    }
    catch(...)
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
        if( capture->open( index ))
            return (CvCapture*)capture;
    }
    catch(...)
    {
        delete capture;
        throw;
    }
    delete capture;
    return 0;
}
*/

CvCapture* PSEyeVideoCapture::pseyeCreateCameraCapture_PS3EYE(int index)
{
    PSEEYECaptureCAM_PS3EYE* capture = new PSEEYECaptureCAM_PS3EYE;
    try
    {
        if( capture->open( index ))
            return (CvCapture*)capture;
    }
    catch(...)
    {
        delete capture;
        throw;
    }
    delete capture;
    return 0;
}


/*
 * PSEEYECaptureCAM_CLMULTI
 */
/*
PSEEYECaptureCAM_CLMULTI::PSEEYECaptureCAM_CLMULTI():
frame(NULL)
{

}

PSEEYECaptureCAM_CLMULTI::~PSEEYECaptureCAM_CLMULTI()
{
    close();
}

void PSEEYECaptureCAM_CLMULTI::close()
{
}

// Initialize camera input
bool PSEEYECaptureCAM_CLMULTI::open( int _index )
{
    return false;
}

bool PSEEYECaptureCAM_CLMULTI::grabFrame()
{
    return false;
}

IplImage* PSEEYECaptureCAM_CLMULTI::retrieveFrame(int)
{
    return frame;
}

double PSEEYECaptureCAM_CLMULTI::getProperty( int property_id ) const
{
    return 0;
}
bool PSEEYECaptureCAM_CLMULTI::setProperty( int property_id, double value )
{
    return false;
}
*/

/*
 * PSEEYECaptureCAM_CLEYE
 */

/*
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
}

// Initialize camera input
bool PSEEYECaptureCAM_CLEYE::open( int _index )
{
    return false;
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
*/

/*
 * PSEEYECaptureCAM_PS3EYE
 */
PSEEYECaptureCAM_PS3EYE::PSEEYECaptureCAM_PS3EYE():
frame(NULL)
{
    
}

/*
 PSEEYECaptureCAM_PS3EYE::~PSEEYECaptureCAM_PS3EYE()
 {
 close();
 }
 */

void PSEEYECaptureCAM_PS3EYE::close()
{
}

// Initialize camera input
bool PSEEYECaptureCAM_PS3EYE::open( int _index )
{
    return false;
}

bool PSEEYECaptureCAM_PS3EYE::grabFrame()
{
    return false;
}

IplImage* PSEEYECaptureCAM_PS3EYE::retrieveFrame(int)
{
    return frame;
}

double PSEEYECaptureCAM_PS3EYE::getProperty( int property_id ) const
{
    return 0;
}
bool PSEEYECaptureCAM_PS3EYE::setProperty( int property_id, double value )
{
    return false;
}