#include "PSMoveController.h"
#include "ServerLog.h"
#include <stdio.h>

// For sleep
#ifdef _WIN32
#include <cstdlib>
#include <windows.h>
#else
#include <unistd.h>
#endif

// https://stackoverflow.com/questions/26423537/how-to-position-the-input-text-cursor-in-c
// https://stackoverflow.com/questions/6486289/how-can-i-clear-console
// http://www.mailsend-online.com/blog/setting-windows-console-text-colors-in-c.html
// https://gist.github.com/anonymous/8f1e6c22b5213faf8170dcfc2b0f5b93
#ifdef _WIN32
#define CONSOLE_BACKGROUND_BLACK 0x07
#define CONSOLE_BACKGROUND_BLUE 0x97
#define CONSOLE_BACKGROUND_RED 0x47
#define CONSOLE_BACKGROUND_GREEN 0x27

void clear() {
    COORD topLeft  = { 0, 0 };
    HANDLE console = GetStdHandle(STD_OUTPUT_HANDLE);
    CONSOLE_SCREEN_BUFFER_INFO screen;
    DWORD written;

    GetConsoleScreenBufferInfo(console, &screen);
    FillConsoleOutputCharacterA(
        console, ' ', screen.dwSize.X * screen.dwSize.Y, topLeft, &written
    );
    FillConsoleOutputAttribute(
        console, FOREGROUND_GREEN | FOREGROUND_RED | FOREGROUND_BLUE,
        screen.dwSize.X * screen.dwSize.Y, topLeft, &written
    );
    SetConsoleCursorPosition(console, topLeft);
}
void gotoxy(int x, int y) { 
    COORD pos = {(short)x, (short)y};
    HANDLE output = GetStdHandle(STD_OUTPUT_HANDLE);
    SetConsoleCursorPosition(output, pos);
}
void set_background_color(unsigned char color_code)
{
	HANDLE output = GetStdHandle(STD_OUTPUT_HANDLE);
	SetConsoleTextAttribute(output, color_code);
}
#else
#define CONSOLE_BACKGROUND_BLACK 0
#define CONSOLE_BACKGROUND_BLUE 16
#define CONSOLE_BACKGROUND_RED 64
#define CONSOLE_BACKGROUND_GREEN 32
#define set_background_color(color_code) printf("\e[?16;5;%d;c", color_code)
#define clear() printf("\x1B[2J\x1B[H")
#define gotoxy(x,y) printf("\033[%d;%dH", (x), (y))
#endif

void print_bytes(const unsigned char *bytes, const unsigned char *byte_colors, int byte_count, int num_cols)
{	
	int byte_index = 0;
	while (byte_index < byte_count)
	{
		for (int col = 0; col < num_cols; ++col)
		{
			if (byte_index < byte_count)
			{
				set_background_color(byte_colors[byte_index]);
				printf("%2.2X ", bytes[byte_index]);
				++byte_index;
			}
			else
			{
				break;
			}
		}
		printf("\n");
	}
}

int main()
{
    log_init("info");

    if (hid_init() == -1)
    {
        printf("Failed to initialize hidapi\n");
        return -1;
    }

    PSMoveController psmove;

	printf("Opening PSMoveController...\n");
	if (psmove.open())
	{
        const PSMoveControllerInputState *psmstate= nullptr;

        psmove.poll();
        psmstate= static_cast<const PSMoveControllerInputState *>(psmove.getState());

		unsigned char byte_colors[64];
		memset(byte_colors, CONSOLE_BACKGROUND_BLACK, sizeof(byte_colors));

		for (int byte_index= 13; byte_index <= 24; ++byte_index)
			byte_colors[byte_index]= CONSOLE_BACKGROUND_RED; // Accelerometer

		for (int byte_index= 25; byte_index <= 36; ++byte_index)
			byte_colors[byte_index]= CONSOLE_BACKGROUND_GREEN; // Gyroscope

		if (!psmove.getIsPS4Controller())
		{
			for (int byte_index= 38; byte_index <= 42; ++byte_index)
				byte_colors[byte_index]= CONSOLE_BACKGROUND_BLUE; // Magnetometer
		}

		clear();

		while (psmove.getIsBluetooth() && psmstate->Cross != CommonControllerState::Button_DOWN)
		{
			psmove.poll();
            psmstate= static_cast<const PSMoveControllerInputState *>(psmove.getState());

			if (psmstate->Move != CommonControllerState::Button_DOWN)
			{
				gotoxy(0,0);
				print_bytes(psmstate->RawPacket, byte_colors, psmstate->RawPacketSize, 16);

				printf("\n");

				if (!psmove.getIsPS4Controller())
				{
					set_background_color(CONSOLE_BACKGROUND_RED);
					printf("Accelerometer(frame 0):\n");
					set_background_color(CONSOLE_BACKGROUND_BLACK);
					printf("  Raw=<%04x,%04x,%04x>\n",
						(unsigned short)psmstate->RawAccel[0][0], (unsigned short)psmstate->RawAccel[0][1], (unsigned short)psmstate->RawAccel[0][2]);
					printf("  Calibrated=<%8.3f,%8.3f,%8.3f>\n",
						psmstate->CalibratedAccel[0][0], psmstate->CalibratedAccel[0][1], psmstate->CalibratedAccel[0][2]);
					set_background_color(CONSOLE_BACKGROUND_RED);
					printf("Accelerometer(frame 1):\n");
					set_background_color(CONSOLE_BACKGROUND_BLACK);
					printf("  Raw=<%04x,%04x,%04x>\n",
						(unsigned short)psmstate->RawAccel[1][0], (unsigned short)psmstate->RawAccel[1][1], (unsigned short)psmstate->RawAccel[1][2]);
					printf("  Calibrated=<%8.3f,%8.3f,%8.3f>\n",
						psmstate->CalibratedAccel[1][0], psmstate->CalibratedAccel[1][1], psmstate->CalibratedAccel[1][2]);

					printf("\n");

					set_background_color(CONSOLE_BACKGROUND_GREEN);
					printf("Gyroscope(frame 0):\n");
					set_background_color(CONSOLE_BACKGROUND_BLACK);
					printf("  Raw=<%04x,%04x,%04x>\n",
						(unsigned short)psmstate->RawGyro[0][0], (unsigned short)psmstate->RawGyro[0][1], (unsigned short)psmstate->RawGyro[0][2]);
					printf("  Calibrated=<%8.3f,%8.3f,%8.3f>\n",
						psmstate->CalibratedGyro[0][0], psmstate->CalibratedGyro[0][1], psmstate->CalibratedGyro[0][2]);
					set_background_color(CONSOLE_BACKGROUND_GREEN);
					printf("Gyroscope(frame 1):\n");
					set_background_color(CONSOLE_BACKGROUND_BLACK);
					printf("  Raw=<%04x,%04x,%04x>\n",
						(unsigned short)psmstate->RawGyro[1][0], (unsigned short)psmstate->RawGyro[1][1], (unsigned short)psmstate->RawGyro[1][2]);
					printf("  Calibrated=<%8.3f,%8.3f,%8.3f>\n",
						psmstate->CalibratedGyro[1][0], psmstate->CalibratedGyro[1][1], psmstate->CalibratedGyro[1][2]);
					printf("\n");

					set_background_color(CONSOLE_BACKGROUND_BLUE);
					printf("Magnetometer:\n");
					set_background_color(CONSOLE_BACKGROUND_BLACK);
					printf("  Raw=<%04x,%04x,%04x>\n",
						(unsigned short)psmstate->RawMag[0], (unsigned short)psmstate->RawMag[1], (unsigned short)psmstate->RawMag[2]);
					printf("  Calibrated=<%8.3f,%8.3f,%8.3f>\n",
						psmstate->CalibratedMag[0], psmstate->CalibratedMag[1], psmstate->CalibratedMag[2]);
				}
				else
				{
					set_background_color(CONSOLE_BACKGROUND_RED);
					printf("Accelerometer:\n");
					set_background_color(CONSOLE_BACKGROUND_BLACK);
					printf("  Raw=<%04x,%04x,%04x>\n",
						(unsigned short)psmstate->RawAccel[0][0], (unsigned short)psmstate->RawAccel[0][1], (unsigned short)psmstate->RawAccel[0][2]);
					printf("  Calibrated=<%8.3f,%8.3f,%8.3f>\n",
						psmstate->CalibratedAccel[0][0], psmstate->CalibratedAccel[0][1], psmstate->CalibratedAccel[0][2]);

					printf("\n");

					set_background_color(CONSOLE_BACKGROUND_GREEN);
					printf("Gyroscope:\n");
					set_background_color(CONSOLE_BACKGROUND_BLACK);
					printf("  Raw=<%04x,%04x,%04x>\n",
						(unsigned short)psmstate->RawGyro[0][0], (unsigned short)psmstate->RawGyro[0][1], (unsigned short)psmstate->RawGyro[0][2]);
					printf("  Calibrated=<%8.3f,%8.3f,%8.3f>\n",
						psmstate->CalibratedGyro[0][0], psmstate->CalibratedGyro[0][1], psmstate->CalibratedGyro[0][2]);
				}
			}

#ifdef _WIN32
			Sleep(5); // 1 msec
#else
            usleep(5000);
#endif
		}

        psmove.close();
	}
    
    // Tear-down hid api
    hid_exit();

	log_dispose();
    
    return 0;
}