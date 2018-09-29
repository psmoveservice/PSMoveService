// -- include -----
#include "PlatformDeviceAPIWin32.h"
#include "ServerLog.h"

#define ANSI
#define WIN32_LEAN_AND_MEAN

#include <windows.h>  // Required for data types
#include <winuser.h>
#include <bthsdpdef.h>
#include <bluetoothapis.h>
#include <Dbt.h>
#include <guiddef.h>
#include <setupapi.h> // Device setup APIs
#include <assert.h>
#include <strsafe.h>
#include <winreg.h>
#include <Shellapi.h>

#include <string>
#include <vector>
#include <iostream>
#include <iomanip>

//-- constants -----
const char *k_reg_property_driver_desc = "DriverDesc";
const char *k_reg_property_driver_version = "DriverVersion";
const char *k_reg_property_matching_device_id = "MatchingDeviceId";
const char *k_reg_property_provider_name = "ProviderName";
const char *k_reg_property_vendor = "Vendor";

GUID GUID_DEVCLASS_IMAGE = { 0x6bdd1fc6, 0x810f, 0x11d0, 0xbe, 0xc7, 0x08, 0x00, 0x2b, 0xe2, 0x09, 0x2f };
GUID GUID_DEVCLASS_HID = { 0x4d1e55b2, 0xf16f, 0x11cf, 0x88, 0xcb, 0x00, 0x11, 0x11, 0x00, 0x00, 0x30 };
GUID GUID_DEVCLASS_USB_RAW = { 0xa5dcbf10, 0x6530, 0x11d2, 0x90, 0x1f, 0x00, 0xc0, 0x4f, 0xb9, 0x51, 0xed };


#define CLS_NAME "DEVICE_LISTENER_CLASS"
#define HWND_MESSAGE     ((HWND)-3)

// -- globals ----
IDeviceHotplugListener *g_hotplug_broadcaster = nullptr;
HDEVNOTIFY g_hImageDeviceNotify = nullptr;
HDEVNOTIFY g_hHIDDeviceNotify = nullptr;
HDEVNOTIFY g_hGenericUSBDeviceNotify = nullptr;
std::vector<HDEVNOTIFY> g_BluetoothDeviceNotify;
HWND g_hWnd = nullptr;

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
		m_DeviceInfoSetHandle = SetupDiGetClassDevs((LPGUID)&GUID_DEVCLASS_IMAGE, 0, 0, DIGCF_PRESENT);
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
				m_bNoMoreItems = true;
			}
		}
	}

	inline HDEVINFO getDeviceInfoSetHandle() const
	{
		return m_DeviceInfoSetHandle;
	}

	inline SP_DEVINFO_DATA &getDeviceInfo()
	{
		return m_DeviceInfoData;
	}

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
static std::string bluetooth_address_to_string(const BTH_ADDR* bt_address);

static void register_all_bluetooth_connection_notifications(HWND__* hwnd);
static HDEVNOTIFY register_device_class_notification(HWND__* hwnd, const GUID &guid);
static LRESULT message_handler(HWND__* hwnd, UINT uint, WPARAM wparam, LPARAM lparam);

// -- definitions -----
PlatformDeviceAPIWin32::PlatformDeviceAPIWin32()
{

}

PlatformDeviceAPIWin32::~PlatformDeviceAPIWin32()
{

}

// System
bool PlatformDeviceAPIWin32::startup(IDeviceHotplugListener *broadcaster)
{
	bool bSuccess = true;

	if (g_hWnd == nullptr)
	{
		WNDCLASSEX wx;
		ZeroMemory(&wx, sizeof(wx));

		wx.cbSize = sizeof(WNDCLASSEX);
		wx.lpfnWndProc = reinterpret_cast<WNDPROC>(message_handler);
		wx.hInstance = reinterpret_cast<HINSTANCE>(GetModuleHandle(0));
		wx.style = CS_HREDRAW | CS_VREDRAW;
		wx.hInstance = GetModuleHandle(0);
		wx.hbrBackground = (HBRUSH)(COLOR_WINDOW);
		wx.lpszClassName = CLS_NAME;

		if (RegisterClassEx(&wx))
		{
			g_hWnd = CreateWindow(
				CLS_NAME, "DevNotifWnd", WS_ICONIC,
				0, 0, CW_USEDEFAULT, 0, HWND_MESSAGE,
				NULL, GetModuleHandle(0), nullptr);
		}

		if (g_hWnd != nullptr)
		{
			g_hotplug_broadcaster = broadcaster;
		}
		else
		{
			SERVER_LOG_ERROR("DeviceHotplugAPIWin32::startup") << "Could not create message window!";
			bSuccess = false;
		}
	}
	else
	{
		SERVER_LOG_WARNING("DeviceHotplugAPIWin32::startup") << "Message handler window already created";
	}

	return bSuccess;
}

void PlatformDeviceAPIWin32::poll()
{
	MSG msg;
	while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE) > 0)
	{
		TranslateMessage(&msg);
		DispatchMessage(&msg);
	}
}

void PlatformDeviceAPIWin32::shutdown()
{
	for (HDEVNOTIFY hNotify : g_BluetoothDeviceNotify)
	{
		UnregisterDeviceNotification(hNotify);
	}
	g_BluetoothDeviceNotify.clear();

	if (g_hImageDeviceNotify != nullptr)
	{
		UnregisterDeviceNotification(g_hImageDeviceNotify);
		g_hImageDeviceNotify = nullptr;
	}

	if (g_hHIDDeviceNotify != nullptr)
	{
		UnregisterDeviceNotification(g_hHIDDeviceNotify);
		g_hHIDDeviceNotify = nullptr;
	}

	if (g_hGenericUSBDeviceNotify != nullptr)
	{
		UnregisterDeviceNotification(g_hGenericUSBDeviceNotify);
		g_hGenericUSBDeviceNotify = nullptr;
	}

	if (g_hWnd != nullptr)
	{
		DestroyWindow(g_hWnd);
		g_hWnd = nullptr;
	}
}

// Events
void PlatformDeviceAPIWin32::handle_bluetooth_request_started()
{
	for (HDEVNOTIFY hNotify : g_BluetoothDeviceNotify)
	{
		UnregisterDeviceNotification(hNotify);
	}
	g_BluetoothDeviceNotify.clear();

	if (g_hHIDDeviceNotify != nullptr)
	{
		UnregisterDeviceNotification(g_hHIDDeviceNotify);
		g_hHIDDeviceNotify = nullptr;
	}

	if (g_hGenericUSBDeviceNotify != nullptr)
	{
		UnregisterDeviceNotification(g_hGenericUSBDeviceNotify);
		g_hGenericUSBDeviceNotify = nullptr;
	}
}

void PlatformDeviceAPIWin32::handle_bluetooth_request_finished()
{
	if (g_BluetoothDeviceNotify.empty())
	{
		register_all_bluetooth_connection_notifications(g_hWnd);
	}

	if (g_hGenericUSBDeviceNotify == nullptr)
	{
		g_hGenericUSBDeviceNotify = register_device_class_notification(g_hWnd, GUID_DEVCLASS_USB_RAW);
	}

	if (g_hHIDDeviceNotify == nullptr)
	{
		g_hHIDDeviceNotify = register_device_class_notification(g_hWnd, GUID_DEVCLASS_IMAGE);
	}
}

// Queries
bool PlatformDeviceAPIWin32::get_device_property(
	const DeviceClass deviceClass,
	const int vendor_id,
	const int product_id,
	const char *property_name,
	char *buffer,
	const int buffer_size)
{
	bool success = false;
	const GUID *deviceClassGUID = NULL;

	switch (deviceClass)
	{
	case DeviceClass::DeviceClass_Camera:
		deviceClassGUID = &GUID_DEVCLASS_IMAGE;
		break;
	case DeviceClass::DeviceClass_HID:
		deviceClassGUID = &GUID_DEVCLASS_HID;
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
							success = true;
							break;
						}
					}
				}
			}
		}
	}

	return success;
}

//-- private helper methods -----
LRESULT message_handler(HWND__* hwnd, UINT msg_type, WPARAM wparam, LPARAM lparam)
{
	switch (msg_type)
	{
	case WM_NCCREATE:
		return true;
		break;

	case WM_CREATE: 
		{
			g_hImageDeviceNotify = register_device_class_notification(hwnd, GUID_DEVCLASS_HID);
			g_hGenericUSBDeviceNotify = register_device_class_notification(hwnd, GUID_DEVCLASS_USB_RAW);
			g_hHIDDeviceNotify = register_device_class_notification(hwnd, GUID_DEVCLASS_IMAGE);
			register_all_bluetooth_connection_notifications(hwnd);
			break;
		}

	case WM_DEVICECHANGE:
		{
			PDEV_BROADCAST_HDR lpdb = (PDEV_BROADCAST_HDR)lparam;

			if (lpdb->dbch_devicetype == DBT_DEVTYP_DEVICEINTERFACE)
			{
				PDEV_BROADCAST_DEVICEINTERFACE lpdbv = (PDEV_BROADCAST_DEVICEINTERFACE)lpdb;
				DeviceClass device_class = DeviceClass::DeviceClass_INVALID;
				std::string path = std::string(lpdbv->dbcc_name);

				if (IsEqualCLSID(lpdbv->dbcc_classguid, GUID_DEVCLASS_IMAGE))
				{
					device_class = DeviceClass::DeviceClass_Camera;
				}
				else if (IsEqualCLSID(lpdbv->dbcc_classguid, GUID_DEVCLASS_HID) || 
						IsEqualCLSID(lpdbv->dbcc_classguid, GUID_DEVCLASS_USB_RAW))
				{
					// Only consider this event if we want to listen for the event
					if (g_hGenericUSBDeviceNotify != nullptr ||
						g_hHIDDeviceNotify != nullptr)
					{
						device_class = DeviceClass::DeviceClass_HID;
					}
				}

				if (device_class != DeviceClass_INVALID)
				{
					switch (wparam)
					{
					case DBT_DEVICEARRIVAL:
						g_hotplug_broadcaster->handle_device_connected(device_class, path);
						break;

					case DBT_DEVICEREMOVECOMPLETE:
						g_hotplug_broadcaster->handle_device_disconnected(device_class, path);
						break;
					}
				}
			}
			else if (lpdb->dbch_devicetype == DBT_DEVTYP_HANDLE)
			{
				PDEV_BROADCAST_HANDLE lpdbv = (PDEV_BROADCAST_HANDLE)lpdb;

				if (std::find(
					g_BluetoothDeviceNotify.begin(),
					g_BluetoothDeviceNotify.end(),
					lpdbv->dbch_hdevnotify) != g_BluetoothDeviceNotify.end())
				{
					if (IsEqualGUID(lpdbv->dbch_eventguid, GUID_BLUETOOTH_HCI_EVENT))
					{
						const BTH_HCI_EVENT_INFO *eventInfo= (BTH_HCI_EVENT_INFO *)lpdbv->dbch_data;
						std::string address= bluetooth_address_to_string(&eventInfo->bthAddress);

						if (eventInfo->connectionType == HCI_CONNECTION_TYPE_ACL)
						{
							if (eventInfo->connected)
							{
								g_hotplug_broadcaster->handle_device_connected(DeviceClass::DeviceClass_HID, address);
							}
							else
							{
								g_hotplug_broadcaster->handle_device_disconnected(DeviceClass::DeviceClass_HID, address);
							}
						}
					}
				}
			}
			break;
		}
	}
	return 0L;
}

static void register_all_bluetooth_connection_notifications(HWND__* hwnd)
{
    BLUETOOTH_FIND_RADIO_PARAMS radio_params;
    radio_params.dwSize = sizeof(BLUETOOTH_FIND_RADIO_PARAMS);

	HANDLE hRadio;
    HBLUETOOTH_RADIO_FIND hFind = BluetoothFindFirstRadio(&radio_params, &hRadio);
    if (hFind != INVALID_HANDLE_VALUE) 
    {
		do 
		{
			DEV_BROADCAST_HANDLE NotificationFilter;
			ZeroMemory(&NotificationFilter, sizeof(NotificationFilter));
			NotificationFilter.dbch_size = sizeof(DEV_BROADCAST_HANDLE);
			NotificationFilter.dbch_devicetype = DBT_DEVTYP_HANDLE;
			NotificationFilter.dbch_handle= hRadio;
			memcpy(&NotificationFilter.dbch_eventguid, &GUID_BLUETOOTH_HCI_EVENT, sizeof(NotificationFilter.dbch_eventguid));
			HDEVNOTIFY dev_notify = RegisterDeviceNotification(hwnd, &NotificationFilter, DEVICE_NOTIFY_WINDOW_HANDLE);	

			if (dev_notify != nullptr)
			{
				g_BluetoothDeviceNotify.push_back(dev_notify);
			}
		} while (BluetoothFindNextRadio(hFind, &hRadio));

		BluetoothFindRadioClose(hFind);
    }
}

static HDEVNOTIFY register_device_class_notification(HWND__* hwnd, const GUID &guid)
{
	DEV_BROADCAST_DEVICEINTERFACE NotificationFilter;
	ZeroMemory(&NotificationFilter, sizeof(NotificationFilter));
	NotificationFilter.dbcc_size = sizeof(DEV_BROADCAST_DEVICEINTERFACE);
	NotificationFilter.dbcc_devicetype = DBT_DEVTYP_DEVICEINTERFACE; // DEVICE_NOTIFY_ALL_INTERFACE_CLASSES;
	NotificationFilter.dbcc_classguid = guid;
	HDEVNOTIFY dev_notify = RegisterDeviceNotification(hwnd, &NotificationFilter, DEVICE_NOTIFY_WINDOW_HANDLE);	

	if (dev_notify == nullptr)
	{
		SERVER_LOG_ERROR("RegisterDeviceClassNotification") << "Could not register for device notifications!";
	}

	return dev_notify;
}

static bool fetch_property_string(
	HDEVINFO devInfoSetHandle,
	SP_DEVINFO_DATA &devInfo,
	const DWORD propertyType,
	char *buffer,
	const int bufferSize)
{
	DWORD propertyDataType;
	DWORD requiredBufferSize = 0;

	BOOL success =
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
	bool success = false;
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
		DWORD inout_buffer_size = static_cast<DWORD>(buffer_size);

		err = RegQueryValueExA(
			hKey,
			property_name,
			NULL,
			NULL,
			reinterpret_cast<LPBYTE>(property_buffer),
			&inout_buffer_size);

		if (err == ERROR_SUCCESS)
		{
			success = true;
		}

		RegCloseKey(hKey);
	}

	return success;
}

static std::string
bluetooth_address_to_string(const BTH_ADDR* bt_address)
{
    std::ostringstream stream;

	BYTE  *raw_bytes= (BYTE *)(&bt_address);
    for (int buff_ind = 5; buff_ind >= 0; buff_ind--)
    {
        stream << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(raw_bytes[buff_ind]);

        if (buff_ind > 0)
        {
            stream << ":";
        }
    }

    return stream.str();
}