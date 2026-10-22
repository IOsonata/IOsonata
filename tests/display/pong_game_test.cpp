/**-------------------------------------------------------------------------
@file	pong_game_test.cpp

@brief	Host model test of the joystick Pong game

@author	Hoang Nguyen Hoan
@date	Oct. 20, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/

#define PONG_GAME_TEST
#include "joystick_display_fixture.h"

static void Advance(uint32_t Ms)
{
	ticks += uint64_t(Ms) * 1000;
	PollJoystick();
}

int main()
{
	assert(HardwareInit() && displayOn && !selected);
	assert(s_State == GAME_WAIT && s_PlayerScore == 0 && s_AiScore == 0);

	// A select held through startup is ignored until the joystick is released.
	Sample(0, 20);
	assert(s_State == GAME_WAIT);
	Sample(3.3f);
	Press(0);
	assert(s_State == GAME_PLAY);

	int y = s_PlayerY;
	Sample(2.01f);
	Advance(s_FrameMs);
	assert(s_PlayerY < y);
	Sample(3.3f);
	y = s_PlayerY;
	Sample(1.32f);
	Advance(s_FrameMs);
	assert(s_PlayerY > y);
	Sample(3.3f);

	// Top-wall and player-paddle collisions reverse the expected velocity.
	s_BallX = 60;
	s_BallY = s_FieldTop;
	s_BallVx = 2;
	s_BallVy = -1;
	Advance(s_FrameMs);
	assert(s_BallY == s_FieldTop && s_BallVy > 0);

	s_BallX = s_PlayerX + s_PaddleWidth + 1;
	s_BallY = s_PlayerY + 4;
	s_BallVx = -2;
	s_BallVy = 1;
	Advance(s_FrameMs);
	assert(s_BallVx > 0 && s_BallX == s_PlayerX + s_PaddleWidth);

	Press(0);
	assert(s_State == GAME_PAUSED);
	int x = s_BallX;
	Advance(200);
	assert(s_BallX == x);
	Press(0);
	assert(s_State == GAME_PLAY);

	// A joystick read failure stops paddle input, not the running game.
	s_BallX = 60;
	s_BallY = 30;
	s_BallVx = 2;
	s_BallVy = 1;
	readOk = false;
	x = s_BallX;
	Advance(s_FrameMs);
	assert(s_InputError && s_BallX != x && s_State == GAME_PLAY);
	readOk = true;
	Sample(3.3f);
	assert(!s_InputError);

	// A ninth point ends the match; SELECT starts a fresh match.
	s_PlayerScore = 8;
	s_AiScore = 0;
	s_AiY = s_PaddleMinY;
	s_BallX = DISPLAY_WIDTH - s_BallSize - 1;
	s_BallY = s_BallMaxY;
	s_BallVx = 2;
	s_BallVy = -1;
	Advance(s_FrameMs);
	assert(s_State == GAME_OVER && s_PlayerScore == s_WinScore);
	Press(0);
	assert(s_State == GAME_PLAY && s_PlayerScore == 0 && s_AiScore == 0);
	assert(!selected);
	std::puts("Pong: startup gate, held paddle, wall/paddle bounce, pause, ADC failure, score and restart PASS");
}
