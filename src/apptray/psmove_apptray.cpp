// QuickCodePage.cpp : Defines the entry point for the application.
//

//-- includes -----
#include <windows.h>
#include <tchar.h>
#include <shellapi.h>
#pragma comment (lib, "shell32.lib")

#include "resource.h"

//-- macros -----
//#define _countof(x) (sizeof(x) / sizeof((x)[0]))

//-- constants -----
const UINT WM_TRAY = WM_USER + 1;
HINSTANCE g_hInstance = NULL;
HICON g_hLargeIcon = NULL;
HICON g_hSmallIcon = NULL;

//-- macros -----
#define MESSAGE_MAP(MessageParam, MessageCallback)\
    if(message == MessageParam)\
        return MessageCallback(hWnd, message, wParam, lParam)
#define MESSAGE_MAP_WITH_LPARAM(MessageParam, lParamValue, MessageCallback)\
    if(message == MessageParam && (lParam == lParamValue))\
        return MessageCallback(hWnd, message, wParam, lParam)
#define MESSAGE_MAP_COMMAND_WITH_LOWORD(Identifier, MessageCallback)\
    if((message == WM_COMMAND) && (LOWORD(wParam) == Identifier))\
    return MessageCallback(hWnd, message, wParam, lParam);


//-- structures -----

//-- globals -----

//-- prototypes -----
LRESULT CALLBACK MainHiddenWndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
LRESULT OnMainHiddenWindowInit(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
LRESULT OnMainHiddenWindowDestroy(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
LRESULT OnMainHiddenWindowTrayDoubleClick(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
LRESULT OnMainHiddenWindowTrayRightClick(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
LRESULT OnMainHiddenWindowShowCommand(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
LRESULT OnMainHiddenWindowQuitCommand(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

INT_PTR CALLBACK ConfigurationDialogProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
LRESULT OnConfigurationDialogInit(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
LRESULT OnConfigurationDialogDestroy(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
LRESULT OnConfigurationDialogClose(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
LRESULT OnConfigurationDialogSize(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
LRESULT OnConfigurationDialogRefresh(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
LRESULT OnMenuServiceConfigure(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
LRESULT OnMenuServiceStart(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
LRESULT OnMenuServiceStop(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
LRESULT OnMenuHelpAbout(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
LRESULT OnMenuClose(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

INT_PTR CALLBACK ConfigServiceDialogProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
LRESULT OnConfigServiceDialogInit(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
LRESULT OnConfigServiceOk(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
LRESULT OnConfigServiceCancel(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

INT_PTR CALLBACK AboutDialogProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
LRESULT OnAboutDialogInit(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
LRESULT OnAboutDialogDone(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

void LoadStringSafe(UINT nStrID, LPTSTR szBuf, UINT nBufLen);

//-- Entry Point -----
int WINAPI WinMain(HINSTANCE hInstance,
                     HINSTANCE hPrevInstance,
                     LPSTR     lpCmdLine,
                     int       nCmdShow)
{
	TCHAR szTxt[0x100];
	wsprintf(szTxt, _T("hInstance=%08x\n"), hInstance);
	OutputDebugString(szTxt);

	WNDCLASS stWC;
	ZeroMemory(&stWC, sizeof(stWC));
	stWC.lpszClassName = _T("PSMove AppTray");

	HWND hHiddenWnd = FindWindow(stWC.lpszClassName, NULL);
	if (hHiddenWnd)
	{
		PostMessage(hHiddenWnd, WM_TRAY, 0, WM_LBUTTONDBLCLK);
	}
	else
	{
		stWC.hInstance = hInstance;
		stWC.lpfnWndProc = MainHiddenWndProc;

		ATOM aClass = RegisterClass(&stWC);
		if (aClass)
		{
			g_hInstance = hInstance;
			if (hHiddenWnd = CreateWindow((LPCTSTR) aClass, _T(""), 0, 0, 0, 0, 0, NULL, NULL, hInstance, NULL))
			{
				MSG stMsg;
				while (GetMessage(&stMsg, NULL, 0, 0) > 0)
				{
					TranslateMessage(&stMsg);
					DispatchMessage(&stMsg);
				}

				if (IsWindow(hHiddenWnd))
				{
					DestroyWindow(hHiddenWnd);
				}
			}

			UnregisterClass((LPCTSTR) aClass, g_hInstance);
		}
	}

	return 0;
}

//-- Implementation ----

//-- Main Hidden Window --------------
LRESULT CALLBACK MainHiddenWndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	MESSAGE_MAP(WM_CREATE, OnMainHiddenWindowInit);
	MESSAGE_MAP(WM_DESTROY, OnMainHiddenWindowDestroy);
	MESSAGE_MAP_WITH_LPARAM(WM_TRAY, WM_LBUTTONDBLCLK, OnMainHiddenWindowTrayDoubleClick);
	MESSAGE_MAP_WITH_LPARAM(WM_TRAY, WM_RBUTTONDOWN, OnMainHiddenWindowTrayRightClick);
	MESSAGE_MAP_COMMAND_WITH_LOWORD(ID_SHOW, OnMainHiddenWindowShowCommand);
	MESSAGE_MAP_COMMAND_WITH_LOWORD(ID_QUIT, OnMainHiddenWindowQuitCommand);

	return DefWindowProc(hWnd, message, wParam, lParam);
}

LRESULT OnMainHiddenWindowInit(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	LRESULT result= true;
	NOTIFYICONDATA stData;

	g_hLargeIcon = reinterpret_cast<HICON>(LoadImage(
		::GetModuleHandle(0),
		MAKEINTRESOURCE(IDI_TRAYICON),
		IMAGE_ICON,
		::GetSystemMetrics(SM_CXICON),
		::GetSystemMetrics(SM_CYICON),
		0));
	g_hSmallIcon = reinterpret_cast<HICON>(LoadImage(
		::GetModuleHandle(0),
		MAKEINTRESOURCE(IDI_TRAYICON),
		IMAGE_ICON,
		::GetSystemMetrics(SM_CXSMICON),
		::GetSystemMetrics(SM_CYSMICON),
		0));

	ZeroMemory(&stData, sizeof(stData));
	stData.cbSize = sizeof(stData);
	stData.hWnd = hWnd;
	stData.uFlags = NIF_ICON | NIF_MESSAGE | NIF_TIP;
	stData.uCallbackMessage = WM_TRAY;
	stData.hIcon = g_hSmallIcon;
	LoadStringSafe(IDS_TIP, stData.szTip, _countof(stData.szTip));

	result = Shell_NotifyIcon(NIM_ADD, &stData);

	return result;
}

LRESULT OnMainHiddenWindowDestroy(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	NOTIFYICONDATA stData;

	ZeroMemory(&stData, sizeof(stData));
	stData.cbSize = sizeof(stData);
	stData.hWnd = hWnd;
	Shell_NotifyIcon(NIM_DELETE, &stData);

	return FALSE;
}

LRESULT OnMainHiddenWindowTrayDoubleClick(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	SendMessage(hWnd, WM_COMMAND, ID_SHOW, 0);
	return TRUE;
}

LRESULT OnMainHiddenWindowTrayRightClick(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	HMENU hMenu = LoadMenu(g_hInstance, MAKEINTRESOURCE(IDR_POPUP));

	if (hMenu)
	{
		HMENU hSubMenu = GetSubMenu(hMenu, 0);

		if (hSubMenu)
		{
			POINT stPoint;

			GetCursorPos(&stPoint);
			TrackPopupMenu(hSubMenu, TPM_LEFTALIGN | TPM_BOTTOMALIGN | TPM_RIGHTBUTTON, stPoint.x, stPoint.y, 0, hWnd, NULL);
		}

		DestroyMenu(hMenu);
	}

	return TRUE;
}

LRESULT OnMainHiddenWindowShowCommand(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	DialogBox(g_hInstance, MAKEINTRESOURCE(IDD_CONFIGURATION), NULL, ConfigurationDialogProc);

	return TRUE;
}

LRESULT OnMainHiddenWindowQuitCommand(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	PostQuitMessage(0);

	return TRUE;
}

//-- Configuration Panel --------------
INT_PTR CALLBACK ConfigurationDialogProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	MESSAGE_MAP(WM_INITDIALOG, OnConfigurationDialogInit);
	MESSAGE_MAP(WM_DESTROY, OnConfigurationDialogDestroy);
	MESSAGE_MAP(WM_CLOSE, OnConfigurationDialogClose);
	MESSAGE_MAP(WM_SIZE, OnConfigurationDialogSize);
	MESSAGE_MAP_COMMAND_WITH_LOWORD(IDC_REFRESH, OnConfigurationDialogRefresh)
	MESSAGE_MAP_COMMAND_WITH_LOWORD(ID_SERVICE_CONFIGURE, OnMenuServiceConfigure)
	MESSAGE_MAP_COMMAND_WITH_LOWORD(ID_SERVICE_STARTRUNTIME, OnMenuServiceStart)
	MESSAGE_MAP_COMMAND_WITH_LOWORD(ID_SERVICE_STOPRUNTIME, OnMenuServiceStop)
	MESSAGE_MAP_COMMAND_WITH_LOWORD(ID_HELP_ABOUT, OnMenuHelpAbout)
	MESSAGE_MAP_COMMAND_WITH_LOWORD(ID_FILE_CLOSE_CONFIGURATION, OnMenuClose)

	return FALSE;
}

LRESULT OnConfigurationDialogInit(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	HMENU hMenu = LoadMenu(g_hInstance, MAKEINTRESOURCE(IDR_CONFIGURATION_MENU));
	SetMenu(hWnd, hMenu);

	if (g_hLargeIcon)
	{
		SendMessage(hWnd, WM_SETICON, ICON_BIG, (LPARAM)g_hLargeIcon);
	}

	if (g_hSmallIcon)
	{
		SendMessage(hWnd, WM_SETICON, ICON_SMALL, (LPARAM)g_hSmallIcon);
	}

	SendMessage(hWnd, WM_COMMAND, MAKELONG(0, CBN_SELENDOK), 0);

	return TRUE;
}

LRESULT OnConfigurationDialogDestroy(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	// TODO: Save state in registry
	return FALSE;
}

LRESULT OnConfigurationDialogClose(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	EndDialog(hWnd, IDCANCEL);
	return TRUE;
}

LRESULT OnConfigurationDialogSize(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	if (SIZE_MINIMIZED == wParam)
	{
		DestroyWindow(hWnd); // no need to hide it
	}

	return FALSE;
}

LRESULT OnConfigurationDialogRefresh(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	SendMessage(hWnd, WM_COMMAND, MAKELONG(0, CBN_SELENDOK), 0);
	return FALSE;
}

LRESULT OnMenuServiceConfigure(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	DialogBox(g_hInstance, MAKEINTRESOURCE(IDD_CONFIG_SERVICE), NULL, ConfigServiceDialogProc);

	return TRUE;
}

LRESULT OnMenuServiceStart(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	MessageBox(hWnd, "Starting the PSMove Service...", "Service Start", MB_OK);

	return TRUE;
}

LRESULT OnMenuServiceStop(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	if (MessageBox(hWnd, "Are you sure?", "Confirm Service Stop", MB_YESNO) == IDYES)
	{
		//TODO: stop the psmove windows service
	}

	return TRUE;
}

LRESULT OnMenuHelpAbout(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	DialogBox(g_hInstance, MAKEINTRESOURCE(IDD_ABOUT), NULL, AboutDialogProc);
	
	return TRUE;
}

LRESULT OnMenuClose(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	DestroyWindow(hWnd);
	return TRUE;
}

//-- ConfigService Dialog -----
INT_PTR CALLBACK ConfigServiceDialogProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	MESSAGE_MAP(WM_INITDIALOG, OnConfigServiceDialogInit);
	MESSAGE_MAP_COMMAND_WITH_LOWORD(IDOK, OnConfigServiceOk)
	MESSAGE_MAP_COMMAND_WITH_LOWORD(IDCANCEL, OnConfigServiceCancel)

	return FALSE;
}

LRESULT OnConfigServiceDialogInit(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	return TRUE;
}

LRESULT OnConfigServiceOk(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	DestroyWindow(hWnd);
	return TRUE;
}

LRESULT OnConfigServiceCancel(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	DestroyWindow(hWnd);
	return TRUE;
}

//-- About Dialog -----
INT_PTR CALLBACK AboutDialogProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	MESSAGE_MAP(WM_INITDIALOG, OnAboutDialogInit);
	MESSAGE_MAP_COMMAND_WITH_LOWORD(IDCANCEL, OnAboutDialogDone)
	MESSAGE_MAP_COMMAND_WITH_LOWORD(IDOK, OnAboutDialogDone)

	return FALSE;
}

LRESULT OnAboutDialogInit(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	return TRUE;
}

LRESULT OnAboutDialogDone(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	DestroyWindow(hWnd);
	return TRUE;
}

//-- Utilities -----
void LoadStringSafe(UINT nStrID, LPTSTR szBuf, UINT nBufLen)
{
	UINT nLen = LoadString(g_hInstance, nStrID, szBuf, nBufLen);
	if (nLen >= nBufLen)
		nLen = nBufLen - 1;
	szBuf[nLen] = 0;
}

