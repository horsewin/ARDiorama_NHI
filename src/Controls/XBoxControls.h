#ifndef XBOX_CONTROLS_H
#define XBOX_CONTROLS_H

#include <windows.h>
#include <commdlg.h>
#include <XInput.h> // XInput API

#pragma comment(lib, "xinput.lib")

#define MAX_CONTROLLERS 2

class XboxController: public Controller {
public:
	XboxController(bt_ARMM_world *m_world):Controller(m_world) {
		// Default to 24% of the +/- 32767 range.   This is a reasonable default value but can be altered if needed.
		INPUT_DEADZONE  = 0.24f * FLOAT(0x7FFF);
		g_bDeadZoneOn = true;
	};

	void check_input() {

		for( DWORD i = 0; i < MAX_CONTROLLERS; i++ )
		{
			// Simply get the state of the controller from XInput.
			if (XInputGetState( i, &g_Controllers[i] ) == ERROR_SUCCESS) {

				WORD wButtons = g_Controllers[i].Gamepad.wButtons;

				if( g_bDeadZoneOn )
				{
					// Zero value if thumbsticks are within the dead zone 
					if( ( g_Controllers[i].Gamepad.sThumbLX < INPUT_DEADZONE && g_Controllers[i].Gamepad.sThumbLX > -INPUT_DEADZONE ) &&
						( g_Controllers[i].Gamepad.sThumbLY < INPUT_DEADZONE && g_Controllers[i].Gamepad.sThumbLY > -INPUT_DEADZONE ) )
					{
						g_Controllers[i].Gamepad.sThumbLX = 0;
						g_Controllers[i].Gamepad.sThumbLY = 0;
					}

					if( ( g_Controllers[i].Gamepad.sThumbRX < INPUT_DEADZONE && g_Controllers[i].Gamepad.sThumbRX > -INPUT_DEADZONE ) &&
						( g_Controllers[i].Gamepad.sThumbRY < INPUT_DEADZONE && g_Controllers[i].Gamepad.sThumbRY > -INPUT_DEADZONE ) )
					{
						g_Controllers[i].Gamepad.sThumbRX = 0;
						g_Controllers[i].Gamepad.sThumbRY = 0;
					}
				}

				if (g_Controllers[i].Gamepad.bRightTrigger > 0 || g_Controllers[i].Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_UP || g_Controllers[i].Gamepad.wButtons & XINPUT_GAMEPAD_A) {
					world->accelerateEngine(i);
				} else if ( g_Controllers[i].Gamepad.bLeftTrigger > 0 || g_Controllers[i].Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_DOWN || g_Controllers[i].Gamepad.wButtons & XINPUT_GAMEPAD_X) {
					world->decelerateEngine(i);
				} else if (g_Controllers[i].Gamepad.sThumbLY > 10000) {
					world->accelerateEngine(i);
				} else if (g_Controllers[i].Gamepad.sThumbLY < -10000) {
					world->decelerateEngine(i);
				} else {
					world->resetEngineForce(i);
				}

				if ( g_Controllers[i].Gamepad.sThumbLX < 0 || g_Controllers[i].Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_LEFT) {
					world->turnEngineLeft(i);
				} else if ( g_Controllers[i].Gamepad.sThumbLX > 0 || g_Controllers[i].Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_RIGHT) {
					world->turnEngineRight(i);
				} else {
					world->turnReset(i);
				}

				if ( g_Controllers[i].Gamepad.wButtons & XINPUT_GAMEPAD_Y) world->resetCarScene(i);
			}
		}
	}
				
/* 
	( wButtons & XINPUT_GAMEPAD_DPAD_UP ) ? L"DPAD_UP " : L"",
	( wButtons & XINPUT_GAMEPAD_DPAD_DOWN ) ? L"DPAD_DOWN " : L"",
	( wButtons & XINPUT_GAMEPAD_DPAD_LEFT ) ? L"DPAD_LEFT " : L"",
	( wButtons & XINPUT_GAMEPAD_DPAD_RIGHT ) ? L"DPAD_RIGHT " : L"",
	( wButtons & XINPUT_GAMEPAD_START ) ? L"START " : L"",
	( wButtons & XINPUT_GAMEPAD_BACK ) ? L"BACK " : L"",
	( wButtons & XINPUT_GAMEPAD_LEFT_THUMB ) ? L"LEFT_THUMB " : L"",
	( wButtons & XINPUT_GAMEPAD_RIGHT_THUMB ) ? L"RIGHT_THUMB " : L"",
	( wButtons & XINPUT_GAMEPAD_LEFT_SHOULDER ) ? L"LEFT_SHOULDER " : L"",
	( wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER ) ? L"RIGHT_SHOULDER " : L"",
	( wButtons & XINPUT_GAMEPAD_A ) ? L"A " : L"",
	( wButtons & XINPUT_GAMEPAD_B ) ? L"B " : L"",
	( wButtons & XINPUT_GAMEPAD_X ) ? L"X " : L"",
	( wButtons & XINPUT_GAMEPAD_Y ) ? L"Y " : L"",
	g_Controllers[i].state.Gamepad.bLeftTrigger,
	g_Controllers[i].state.Gamepad.bRightTrigger,
	g_Controllers[i].state.Gamepad.sThumbLX,
	g_Controllers[i].state.Gamepad.sThumbLY,
	g_Controllers[i].state.Gamepad.sThumbRX,
	g_Controllers[i].state.Gamepad.sThumbRY );
*/

private:
	XINPUT_STATE g_Controllers[MAX_CONTROLLERS];

	float INPUT_DEADZONE;
	bool g_bDeadZoneOn;

};

#endif