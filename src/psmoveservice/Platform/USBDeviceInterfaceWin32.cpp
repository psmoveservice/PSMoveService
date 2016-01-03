#include "USBDeviceInterfaceWin32.h"

#include <windows.h>  // Required for data types
#include <guiddef.h>
#include <setupapi.h> // Device setup APIs
#include <assert.h>
#include <strsafe.h>
#include <winreg.h>

//-- constants -----
namespace USBDeviceInterface
{
    const char *k_reg_property_driver_desc= "DriverDesc";
    const char *k_reg_property_driver_version= "DriverVersion";
    const char *k_reg_property_matching_device_id= "MatchingDeviceId";
    const char *k_reg_property_provider_name= "ProviderName";
    const char *k_reg_property_vendor= "Vendor";
}

DEFINE_GUID( GUID_DEVCLASS_IMAGE, 0x6bdd1fc6L, 0x810f, 0x11d0, 0xbe, 0xc7, 0x08, 0x00, 0x2b, 0xe2, 0x09, 0x2f );

//-- private definitions -----
class DeviceInfoIterator
{
public:
    DeviceInfoIterator(const GUID &deviceClassGUID)
        : m_DeviceClassGUID(deviceClassGUID)
        , m_DeviceInfoSetHandle(INVALID_HANDLE_VALUE)
        , m_MemberIndex(-1)
        , m_bNoMoreItems(false)
    {
        m_DeviceInfoSetHandle = SetupDiGetClassDevs((LPGUID) &GUID_DEVCLASS_IMAGE, 0, 0, DIGCF_PRESENT);    
        m_DeviceInfoData.cbSize = sizeof(SP_DEVINFO_DATA);

        if (isValid())
        {
            next();
        }
    }

    virtual ~DeviceInfoIterator()
    {
        if (m_DeviceInfoSetHandle != INVALID_HANDLE_VALUE)
        {
            SetupDiDestroyDeviceInfoList(m_DeviceInfoSetHandle);
        }
    }

    bool isValid() const
    {
        return m_DeviceInfoSetHandle != INVALID_HANDLE_VALUE && !m_bNoMoreItems;
    }

    void next()
    {
        if (isValid()) 
        {   
            ++m_MemberIndex;

            if (SetupDiEnumDeviceInfo(m_DeviceInfoSetHandle, m_MemberIndex, &m_DeviceInfoData) == FALSE)
            {
                m_bNoMoreItems= true;
            }
        }  
    }

    inline HDEVINFO getDeviceInfoSetHandle() const
    { return m_DeviceInfoSetHandle; }

    inline SP_DEVINFO_DATA &getDeviceInfo()
    { return m_DeviceInfoData; }

private:
    const GUID &m_DeviceClassGUID;
    HDEVINFO m_DeviceInfoSetHandle;
    SP_DEVINFO_DATA m_DeviceInfoData;
    int m_MemberIndex;
    bool m_bNoMoreItems;
};

//-- private prototypes -----
static bool fetch_property_string(HDEVINFO devInfoSetHandle, SP_DEVINFO_DATA &devInfo, const DWORD propertyType, 
    char *buffer, const int bufferSize);
static bool fetch_driver_registry_property(const char *driver_reg_path, const char *property_name, 
    char *property_buffer, const int buffer_size);

//-- public methods -----
namespace USBDeviceInterface
{
    bool fetch_driver_reg_property_for_usb_device(
        const DeviceClass deviceClass,
        const int vendor_id, 
        const int product_id, 
        const char *property_name, 
        char *buffer, 
        const int buffer_size)
    {
        bool success= false;
        const GUID *deviceClassGUID= NULL;

        switch (deviceClass)
        {
        case USBDeviceInterface::Camera:
            deviceClassGUID= &GUID_DEVCLASS_IMAGE;
            break;
        default:
            assert(0 && "Unhandled device class type");
        }

        if (deviceClassGUID != NULL)
        {
            char expected_hardware_id[128];
            size_t expected_length;

            StringCchPrintfA(expected_hardware_id, sizeof(expected_hardware_id), "USB\\VID_%X&PID_%X", vendor_id, product_id);
            StringCchLengthA(expected_hardware_id, sizeof(expected_hardware_id), &expected_length);

            for (DeviceInfoIterator iter(*deviceClassGUID); iter.isValid(); iter.next())
            {
                char hardware_id_property[128]; // ex: "USB\\VID_1415&PID_2000&REV_0200&MI_00"

                if (fetch_property_string(
                        iter.getDeviceInfoSetHandle(), 
                        iter.getDeviceInfo(), 
                        SPDRP_HARDWAREID,
                        hardware_id_property, 
                        sizeof(hardware_id_property)))
                {
                    if (strncmp(expected_hardware_id, hardware_id_property, expected_length) == 0)
                    {
                        char driver_reg_path[128]; // ex: "{6bdd1fc6-810f-11d0-bec7-08002be2092f}\\0007"

                        if (fetch_property_string(
                                iter.getDeviceInfoSetHandle(), 
                                iter.getDeviceInfo(), 
                                SPDRP_DRIVER,
                                driver_reg_path, 
                                sizeof(driver_reg_path)))
                        {
                            if (fetch_driver_registry_property(
                                    driver_reg_path,
                                    property_name,
                                    buffer,
                                    buffer_size))
                            {
                                success= true;
                                break;
                            }
                        }
                    }
                }
            }
        }

        return success;
    }
};

//-- private helper methods -----
static bool fetch_property_string(
    HDEVINFO devInfoSetHandle, 
    SP_DEVINFO_DATA &devInfo, 
    const DWORD propertyType,
    char *buffer, 
    const int bufferSize)
{
    DWORD propertyDataType;
    DWORD requiredBufferSize= 0;

    BOOL success=
        SetupDiGetDeviceRegistryPropertyA(
            devInfoSetHandle, 
            &devInfo, 
            propertyType,
            &propertyDataType,
            reinterpret_cast<PBYTE>(buffer),
            static_cast<DWORD>(bufferSize),
            &requiredBufferSize);
    assert(bufferSize >= static_cast<int>(requiredBufferSize));

    return success == TRUE;
}

static bool fetch_driver_registry_property(
    const char *driver_reg_path,
    const char *property_name,
    char *property_buffer,
    const int buffer_size)
{
    bool success= false;
    char full_driver_reg_path[512]; // ex: "{6bdd1fc6-810f-11d0-bec7-08002be2092f}\\0007"

    StringCchPrintfA(
        full_driver_reg_path, 
        sizeof(full_driver_reg_path), 
        "SYSTEM\\CurrentControlSet\\Control\\Class\\%s",
        driver_reg_path);

    HKEY hKey;
    int err = RegOpenKeyExA(HKEY_LOCAL_MACHINE, full_driver_reg_path, 0, KEY_READ, &hKey);
    if (err == ERROR_SUCCESS) 
    {
        DWORD inout_buffer_size= static_cast<DWORD>(buffer_size);

        err= RegQueryValueExA(
            hKey, 
            property_name, 
            NULL, 
            NULL, 
            reinterpret_cast<LPBYTE>(property_buffer), 
            &inout_buffer_size);

        if (err == ERROR_SUCCESS)
        {
            success= true;
        }

        RegCloseKey(hKey);
    }

    return success;
}