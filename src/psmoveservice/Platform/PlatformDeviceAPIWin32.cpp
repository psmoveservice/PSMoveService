// -- include -----
#include "PlatformDeviceAPIWin32.h"
#include "ServerLog.h"

#define ANSI
#define WIN32_LEAN_AND_MEAN

#include <windows.h>  // Required for data types
#include <winuser.h>
#include <Dbt.h>
#include <guiddef.h>
#include <setupapi.h> // Device setup APIs
#include <assert.h>
#include <strsafe.h>
#include <winreg.h>
#include <Shellapi.h>

#include <string>
#include <iostream>

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

class RegistryWriteAccess {
public:

	static void    DoSetHiveRights(HKEY hKey)
	{
		SECURITY_DESCRIPTOR         sd;
		PSID                        psidWorldSid;
		SID_IDENTIFIER_AUTHORITY    siaWorldSidAuthority = SECURITY_WORLD_SID_AUTHORITY;

		psidWorldSid = (PSID)LocalAlloc(LPTR,
			GetSidLengthRequired(1)
		);

		InitializeSid(psidWorldSid, &siaWorldSidAuthority, 1);

		*(GetSidSubAuthority(psidWorldSid, 0)) = SECURITY_WORLD_RID;

		RegSetHiveSecurity(hKey,
			psidWorldSid,
			OWNER_SECURITY_INFORMATION,
			&sd
		);
	}


	static  DWORD   RegSetHiveSecurity(HKEY                    hKey,
		PSID                    psid,
		SECURITY_INFORMATION    si,
		PSECURITY_DESCRIPTOR    psd
	)
	{
		int     nIdx = 0;
		HKEY    hSubKey;
		char    acSubKey[MAX_PATH + 1];
		DWORD   dwRes = ERROR_SUCCESS;


		if (!AddAccessRights(hKey, psid, GENERIC_ALL))
			return  (GetLastError());

		for (;;)
		{
			dwRes = RegEnumKey(hKey,
				nIdx,
				acSubKey,
				MAX_PATH + 1
			);

			if (ERROR_NO_MORE_ITEMS == dwRes)
			{
				break;
			}

			if (ERROR_SUCCESS != dwRes)
			{
				break;
			}

			nIdx++;

			printf("found '%s'\n", acSubKey);

			dwRes = RegOpenKeyEx(hKey,
				acSubKey,
				0,
				KEY_ALL_ACCESS,
				&hSubKey
			);

			if (ERROR_SUCCESS != dwRes)
			{
				printf("ERROR opening '%s', reason == %d\n", acSubKey, dwRes);
				continue;
			}

			dwRes = RegSetHiveSecurity(hSubKey,
				psid,
				si,
				psd
			);

			RegCloseKey(hSubKey);

			if (ERROR_NO_MORE_ITEMS != dwRes)
			{
				break;
			}

			printf("SUCCEEDED for '%s'\n", acSubKey);
		}

		return  (dwRes);
	}

	static  DWORD AddToRegKeySD(PSECURITY_DESCRIPTOR pRelSD, PSID pGroupSID,
		DWORD dwAccessMask, HKEY    hSecurityRegKey)
	{
		PSECURITY_DESCRIPTOR pAbsSD = NULL;

		PACL  pDACL;

		DWORD  dwSDLength = 0;
		DWORD  dwSDRevision;
		DWORD  dwDACLLength = 0;

		SECURITY_DESCRIPTOR_CONTROL sdcSDControl;

		PACL  pNewDACL = NULL;
		DWORD  dwAddDACLLength = 0;

		BOOL  fAceFound = 0;

		BOOL  fHasDACL = FALSE;
		BOOL  fDACLDefaulted = FALSE;

		ACCESS_ALLOWED_ACE  *pDACLAce;

		DWORD  dwError = 0;

		DWORD  i;


		// get SD control bits
		if (!GetSecurityDescriptorControl(pRelSD,
			(PSECURITY_DESCRIPTOR_CONTROL)&sdcSDControl,
			(LPDWORD)&dwSDRevision))
			return (GetLastError());

		// check if DACL is present
		if (SE_DACL_PRESENT & sdcSDControl)
		{
			// get dacl
			if (!GetSecurityDescriptorDacl(pRelSD, (LPBOOL)&fHasDACL,
				(PACL *)&pDACL,
				(LPBOOL)&fDACLDefaulted))
				return (GetLastError());

			// get dacl length
			dwDACLLength = pDACL->AclSize;

			// now check if SID's ACE is there
			for (i = 0; i < pDACL->AceCount; i++)
			{
				if (!GetAce(pDACL, i, (LPVOID *)&pDACLAce))
					return (GetLastError());

				// check if group sid is already there
				if (EqualSid((PSID) &(pDACLAce->SidStart), pGroupSID))
					break;
			}




			// exit if found (means already has been set)
			if (i < pDACL->AceCount)
			{
				dwError = ERROR_GROUP_EXISTS;

				return (dwError);
			}

			// get length of new DACL
			dwAddDACLLength = sizeof(ACCESS_ALLOWED_ACE) -
				sizeof(DWORD) + GetLengthSid(pGroupSID);
		}
		else
			// get length of new DACL
			dwAddDACLLength = sizeof(ACL) + sizeof(ACCESS_ALLOWED_ACE) -
			sizeof(DWORD) + GetLengthSid(pGroupSID);

		// get memory needed for new DACL
		if (!(pNewDACL = (PACL)malloc(dwDACLLength + dwAddDACLLength)))
			return (GetLastError());

		// get the sd length
		dwSDLength = GetSecurityDescriptorLength(pRelSD);

		// get memory for new SD
		if (!(pAbsSD = (PSECURITY_DESCRIPTOR)
			malloc(dwSDLength + dwAddDACLLength)))
		{
			dwError = GetLastError();

			goto ErrorExit;
		}

		// change self-relative SD to absolute by making new SD
		if (!InitializeSecurityDescriptor(pAbsSD,
			SECURITY_DESCRIPTOR_REVISION))
		{
			dwError = GetLastError();

			goto ErrorExit;
		}

		// init new DACL
		if (!InitializeAcl(pNewDACL, dwDACLLength + dwAddDACLLength,
			ACL_REVISION))
		{
			dwError = GetLastError();

			goto ErrorExit;
		}

		// now add in all of the ACEs into the new DACL (if org DACL is there)
		if (SE_DACL_PRESENT & sdcSDControl)
		{
			for (i = 0; i < pDACL->AceCount; i++)
			{
				// get ace from original dacl
				if (!GetAce(pDACL, i, (LPVOID *)&pDACLAce))
				{
					dwError = GetLastError();

					goto ErrorExit;
				}

				// now add ace to new dacl
				if (!AddAccessAllowedAce(pNewDACL,
					ACL_REVISION,
					pDACLAce->Mask,
					(PSID) &(pDACLAce->SidStart)))
				{
					dwError = GetLastError();

					goto ErrorExit;
				}
			}
		}

		// now add new ACE to new DACL
		if (!AddAccessAllowedAce(pNewDACL, ACL_REVISION, dwAccessMask,
			pGroupSID))
		{
			dwError = GetLastError();

			goto ErrorExit;
		}

		// check if everything went ok
		if (!IsValidAcl(pNewDACL))
		{
			dwError = GetLastError();

			goto ErrorExit;
		}

		// now set security descriptor DACL
		if (!SetSecurityDescriptorDacl(pAbsSD, TRUE, pNewDACL,
			fDACLDefaulted))
		{
			dwError = GetLastError();

			goto ErrorExit;
		}

		// check if everything went ok
		if (!IsValidSecurityDescriptor(pAbsSD))
		{
			dwError = GetLastError();

			goto ErrorExit;
		}


		// now set the reg key security (this will overwrite any existing security)
		dwError = RegSetKeySecurity(
			hSecurityRegKey,
			(SECURITY_INFORMATION)(DACL_SECURITY_INFORMATION),
			pAbsSD);


	ErrorExit:

		// free memory
		if (pAbsSD)
			free((VOID *)pAbsSD);
		if (pNewDACL)
			free((VOID *)pNewDACL);

		return (dwError);
	}
	/* eof - AddToRegKeySD */

	DWORD GetRegKeySecurity(HKEY  hRegKey, PSECURITY_DESCRIPTOR* ppRegKeySD) {
#define PERR(szApi,lError) printf ( "\n%s: Error %d from %s on line %d", \
 __FILE__, lError, szApi, __LINE__); //HKEY  hRegKey;   // handle for register key
		LONG  lError = 0L;  // reg errors
							// (GetLastError won't work with registry calls)
		CHAR  szClassName[MAX_PATH] = ""; // Buffer for class name.
		DWORD dwcClassLen = MAX_PATH;  // Length of class string.
		DWORD dwcSubKeys;     // Number of sub keys.
		DWORD dwcMaxSubKey;    // Longest sub key size.
		DWORD dwcMaxClass;    // Longest class string.
		DWORD dwcValues;     // Number of values for this key.
		DWORD dwcMaxValueName;   // Longest Value name.
		DWORD dwcMaxValueData;   // Longest Value data.
		DWORD dwcSDLength;    // Security descriptor length
		FILETIME ftLastWriteTime;   // Last write time.

		if ((lError = RegQueryInfoKey(hRegKey, szClassName, &dwcClassLen,
			NULL, &dwcSubKeys, &dwcMaxSubKey, &dwcMaxClass,
			&dwcValues, &dwcMaxValueName, &dwcMaxValueData,
			&dwcSDLength, &ftLastWriteTime))) {
			PERR("RegQueryInfoKey", lError);  goto CleanUp;
		}

		*ppRegKeySD = (PSECURITY_DESCRIPTOR)LocalAlloc(LPTR, (UINT)dwcSDLength);
		// now get SD
		if ((lError = RegGetKeySecurity(hRegKey,
			(SECURITY_INFORMATION)(OWNER_SECURITY_INFORMATION
				| GROUP_SECURITY_INFORMATION
				| DACL_SECURITY_INFORMATION),
			*ppRegKeySD, &dwcSDLength))) {
			PERR("RegGetKeySecurity", lError);  goto CleanUp;
		} // check if SD is good
		if (!IsValidSecurityDescriptor(*ppRegKeySD)) {
			lError = GetLastError();
			PERR("IsValidSecurityDescriptor", lError);
		}
	CleanUp: // must close reg key
			 //RegCloseKey ( hRegKey);
		return (lError);
	}/* eof - GetRegKeySecurity */


	static SECURITY_DESCRIPTOR GetWorldSD()
	{
		SID_IDENTIFIER_AUTHORITY siaWorld = SECURITY_WORLD_SID_AUTHORITY;
		PSID psidEveryone = NULL;
		int nSidSize;
		int nAclSize;
		PACL paclNewDacl = NULL;
		SECURITY_DESCRIPTOR sd;

		__try {
			// Create the everyone sid
			if (!AllocateAndInitializeSid(&siaWorld, 1, SECURITY_WORLD_RID, 0,
				0, 0, 0, 0, 0, 0, &psidEveryone))
			{
				psidEveryone = NULL;
				__leave;
			}

			nSidSize = GetLengthSid(psidEveryone);
			nAclSize = nSidSize * 2 + sizeof(ACCESS_ALLOWED_ACE) + sizeof(ACCESS_DENIED_ACE) + sizeof(ACL);
			paclNewDacl = (PACL)LocalAlloc(LPTR, nAclSize);
			if (!paclNewDacl)
				__leave;
			if (!InitializeAcl(paclNewDacl, nAclSize, ACL_REVISION))
				__leave;
			if (!AddAccessDeniedAce(paclNewDacl, ACL_REVISION, WRITE_DAC | WRITE_OWNER, psidEveryone))
				__leave;
			// I am using GENERIC_ALL here so that this very code can be applied to
			// other objects.  Specific access should be applied when possible.
			if (!AddAccessAllowedAce(paclNewDacl, ACL_REVISION, GENERIC_ALL, psidEveryone))
				__leave;
			if (!InitializeSecurityDescriptor(&sd, SECURITY_DESCRIPTOR_REVISION))
				__leave;
			if (!SetSecurityDescriptorDacl(&sd, TRUE, paclNewDacl, FALSE))
				__leave;
		}
		__finally {

			if (!paclNewDacl)
				LocalFree(paclNewDacl);
			if (!psidEveryone)
				FreeSid(psidEveryone);

		}

		return sd;
	}

#define SD_SIZE (65536 + SECURITY_DESCRIPTOR_MIN_LENGTH)

	static BOOL AddAccessRights(HKEY hKey, PSID pSID, DWORD dwAcessMask)
	{

		//  SD variables.

		UCHAR          ucSDbuf[SD_SIZE];
		PSECURITY_DESCRIPTOR pSD = (PSECURITY_DESCRIPTOR)ucSDbuf;
		DWORD          dwSDLengthNeeded = SD_SIZE;

		// ACL variables.

		PACL           pACL;
		BOOL           bDaclPresent;
		BOOL           bDaclDefaulted;
		ACL_SIZE_INFORMATION AclInfo;

		// New ACL variables.

		PACL           pNewACL;
		DWORD          dwNewACLSize;

		// New SD variables.

		UCHAR                NewSD[SECURITY_DESCRIPTOR_MIN_LENGTH];
		PSECURITY_DESCRIPTOR psdNewSD = (PSECURITY_DESCRIPTOR)NewSD;

		// Temporary ACE.

		PVOID          pTempAce;
		UINT           CurrentAceIndex;

		// STEP 2: Get SID (parameter).

		// STEP 3: Get security descriptor (SD) for key.

		if (ERROR_SUCCESS != RegGetKeySecurity(hKey,
			(SECURITY_INFORMATION)(DACL_SECURITY_INFORMATION),
			pSD,
			&dwSDLengthNeeded))
		{
			printf("Error %d:RegGetKeySecurity\n", GetLastError());
			return(FALSE);
		}

		// STEP 4: Initialize new SD.

		if (!InitializeSecurityDescriptor
		(psdNewSD, SECURITY_DESCRIPTOR_REVISION))
		{
			printf("Error %d:InitializeSecurityDescriptor\n", GetLastError());
			return(FALSE);
		}

		// STEP 5: Get DACL from SD.

		if (!GetSecurityDescriptorDacl(pSD,
			&bDaclPresent,
			&pACL,
			&bDaclDefaulted))
		{
			printf("Error %d:GetSecurityDescriptorDacl\n", GetLastError());
			return(FALSE);
		}

		// STEP 6: Get key ACL size information.

		if (!GetAclInformation(pACL, &AclInfo, sizeof(ACL_SIZE_INFORMATION),
			AclSizeInformation))
		{
			printf("Error %d:GetAclInformation\n", GetLastError());
			return(FALSE);
		}

		// STEP 7: Compute size needed for the new ACL.

		dwNewACLSize = AclInfo.AclBytesInUse +
			sizeof(ACCESS_ALLOWED_ACE) +
			GetLengthSid(pSID) - sizeof(DWORD);

		// STEP 8: Allocate memory for new ACL.

		pNewACL = (PACL)LocalAlloc(LPTR, dwNewACLSize);

		// STEP 9: Initialize the new ACL.

		if (!InitializeAcl(pNewACL, dwNewACLSize, ACL_REVISION2))
		{
			printf("Error %d:InitializeAcl\n", GetLastError());
			LocalFree((HLOCAL)pNewACL);
			return(FALSE);
		}

		// STEP 10: If DACL is present, copy it to a new DACL.

		if (bDaclPresent)  // Only copy if DACL was present.
		{
			// STEP 11: Copy the file's ACEs to our new ACL.

			if (AclInfo.AceCount)
			{

				for (CurrentAceIndex = 0; CurrentAceIndex < AclInfo.AceCount;
					CurrentAceIndex++)
				{
					// STEP 12: Get an ACE.

					if (!GetAce(pACL, CurrentAceIndex, &pTempAce))
					{
						printf("Error %d: GetAce\n", GetLastError());
						LocalFree((HLOCAL)pNewACL);
						return(FALSE);
					}

					// STEP 13: Add the ACE to the new ACL.

					if (!AddAce(pNewACL, ACL_REVISION, MAXDWORD, pTempAce,
						((PACE_HEADER)pTempAce)->AceSize))
					{
						printf("Error %d:AddAce\n", GetLastError());
						LocalFree((HLOCAL)pNewACL);
						return(FALSE);
					}

				}
			}
		}

		// STEP 14: Add the access-allowed ACE to the new DACL.

		if (!AddAccessAllowedAce(pNewACL, ACL_REVISION, dwAcessMask, pSID))
		{
			printf("Error %d:AddAccessAllowedAce", GetLastError());
			LocalFree((HLOCAL)pNewACL);
			return(FALSE);
		}

		// STEP 15: Set our new DACL to the file SD.

		if (!SetSecurityDescriptorDacl(psdNewSD,
			TRUE,
			pNewACL,
			FALSE))
		{
			printf("Error %d:SetSecurityDescriptorDacl", GetLastError());
			LocalFree((HLOCAL)pNewACL);
			return(FALSE);
		}

		// STEP 16: Set the SD to the key.

		if (ERROR_SUCCESS != RegSetKeySecurity(hKey, DACL_SECURITY_INFORMATION, psdNewSD))
		{
			printf("Error %d:RegSetKeySecurity\n", GetLastError());
			LocalFree((HLOCAL)pNewACL);
			return(FALSE);
		}

		// STEP 17: Free the memory allocated for the new ACL.

		LocalFree((HLOCAL)pNewACL);
		return(TRUE);
	}

	static BOOL IsElevated() {
		BOOL fRet = FALSE;
		HANDLE hToken = NULL;
		if (OpenProcessToken(GetCurrentProcess(), TOKEN_QUERY, &hToken)) {
			TOKEN_ELEVATION Elevation;
			DWORD cbSize = sizeof(TOKEN_ELEVATION);
			if (GetTokenInformation(hToken, TokenElevation, &Elevation, sizeof(Elevation), &cbSize)) {
				fRet = Elevation.TokenIsElevated;
			}
		}
		if (hToken) {
			CloseHandle(hToken);
		}
		return fRet;
	}
};

//-- private prototypes -----
static bool fetch_property_string(HDEVINFO devInfoSetHandle, SP_DEVINFO_DATA &devInfo, const DWORD propertyType,
	char *buffer, const int bufferSize);
static bool fetch_driver_registry_property(const char *driver_reg_path, const char *property_name,
	char *property_buffer, const int buffer_size);

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

	if (bSuccess) {

		BOOL admin = RegistryWriteAccess::IsElevated();
		HKEY hKey;
		LONG errAccess = RegOpenKeyEx(HKEY_LOCAL_MACHINE,
			("SYSTEM\\CurrentControlSet\\Services\\HidBth\\Parameters\\Devices"),
			0, KEY_READ | KEY_QUERY_VALUE | KEY_WOW64_64KEY | KEY_ALL_ACCESS, &hKey);

		if (!admin && errAccess != ERROR_SUCCESS) {
			char moduleFileName[MAXCHAR];
			char moduleDrive[MAXCHAR];
			char moduleDir[MAXCHAR];

			SERVER_LOG_INFO("DeviceHotplugAPIWin32::startup") << "Failed to access registry... starting in Admin mode.";

			GetModuleFileName(NULL, moduleFileName, sizeof(moduleFileName));
			_splitpath_s(moduleFileName, moduleDrive, MAXCHAR, moduleDir, MAXCHAR, NULL, 0, NULL, 0);

			std::string adminPath = moduleDrive;
			adminPath = adminPath + moduleDir;
			adminPath = adminPath + "PSMoveServiceAdmin.exe";

			SHELLEXECUTEINFO shExInfo = { 0 };
			shExInfo.cbSize = sizeof(shExInfo);
			shExInfo.fMask = SEE_MASK_NOCLOSEPROCESS;
			shExInfo.hwnd = 0;
			shExInfo.lpVerb = ("runas");                // Operation to perform
			shExInfo.lpFile = adminPath.c_str(),       // Application to start    
				shExInfo.lpParameters = "";                  // Additional parameters
			shExInfo.lpDirectory = 0;
			shExInfo.nShow = SW_SHOW;
			shExInfo.hInstApp = 0;

			if (ShellExecuteEx(&shExInfo))
			{
				CloseHandle(shExInfo.hProcess);
			};

			bSuccess = false;
		}
		else if (admin && errAccess == ERROR_SUCCESS)
		{
			SERVER_LOG_INFO("DeviceHotplugAPIWin32::startup") << "Running in Admin mode... updating registry write access.";

			RegistryWriteAccess::DoSetHiveRights(hKey);

			RegCloseKey(hKey);
		}
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
		break;
	}

	case WM_DEVICECHANGE:
	{
		PDEV_BROADCAST_HDR lpdb = (PDEV_BROADCAST_HDR)lparam;
		PDEV_BROADCAST_DEVICEINTERFACE lpdbv = (PDEV_BROADCAST_DEVICEINTERFACE)lpdb;
		std::string path;

		if (lpdb->dbch_devicetype == DBT_DEVTYP_DEVICEINTERFACE)
		{
			DeviceClass device_class = DeviceClass::DeviceClass_INVALID;
			path = std::string(lpdbv->dbcc_name);

			if (IsEqualCLSID(lpdbv->dbcc_classguid, GUID_DEVCLASS_IMAGE))
			{
				device_class = DeviceClass::DeviceClass_Camera;
			}
			else if (IsEqualCLSID(lpdbv->dbcc_classguid, GUID_DEVCLASS_HID) ||
				IsEqualCLSID(lpdbv->dbcc_classguid, GUID_DEVCLASS_USB_RAW))
			{
				device_class = DeviceClass::DeviceClass_HID;
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
		break;
	}
	}
	return 0L;
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