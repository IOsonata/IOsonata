/**-------------------------------------------------------------------------
@file	icli.cpp

@brief	Command Line Interpreter

@author	Hoang Nguyen Hoan
@date	Dec. 15, 2021

@license

MIT License

Copyright (c) 2021, I-SYST, all rights reserved

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
#include <stdio.h>
#include <string.h>

#include "cli/icli.h"

#define CLI_CH_BS		0x08	// backspace
#define CLI_CH_LF		0x0A	// line feed
#define CLI_CH_CR		0x0D	// carriage return
#define CLI_CH_DEL		0x7F	// delete

static const char s_CrLf[] = "\r\n";
static const char s_BsErase[] = "\b \b";	// move back, overwrite with space, move back
static const char s_Unknown[] = "unknown command: ";
static const char s_HelpSep[] = " : ";

static inline void CliWrite(CliDev_t * const pCli, const char *pStr, int Len)
{
	if (pCli->pIntrf != 0 && Len > 0)
	{
		DeviceIntrfTx(pCli->pIntrf, pCli->IntrfAddr, (const uint8_t *)pStr, Len);
	}
}

bool CliInit(CliDev_t * const pCli, const CliCfg_t *pCfg)
{
	if (pCli == 0 || pCfg == 0)
	{
		return false;
	}

	if (pCfg->pLineBuf == 0 || pCfg->LineBufSize < 2)
	{
		return false;
	}

	if (pCfg->pArgv == 0 || pCfg->ArgvMax < 1)
	{
		return false;
	}

	if (pCfg->pIntrf == 0)
	{
		return false;
	}

	pCli->pIntrf = pCfg->pIntrf;
	pCli->IntrfAddr = pCfg->IntrfAddr;
	pCli->BaseSet.pCmd = pCfg->pCmd;
	pCli->BaseSet.NbCmd = pCfg->NbCmd;
	pCli->BaseSet.pNext = 0;
	pCli->pLineBuf = pCfg->pLineBuf;
	pCli->LineBufSize = pCfg->LineBufSize;
	pCli->LineLen = 0;
	pCli->pArgv = pCfg->pArgv;
	pCli->ArgvMax = pCfg->ArgvMax;
	pCli->pPrompt = pCfg->pPrompt;
	pCli->bEcho = pCfg->bEcho;
	pCli->LastCh = 0;

	return true;
}

void CliCmdRegister(CliDev_t * const pCli, CliCmdSet_t * const pSet, const CliCmd_t *pCmd, int NbCmd)
{
	CliCmdSet_t *p = &pCli->BaseSet;

	pSet->pCmd = pCmd;
	pSet->NbCmd = NbCmd;
	pSet->pNext = 0;

	// append to the tail of the chain so base commands keep priority
	while (p->pNext != 0)
	{
		p = p->pNext;
	}

	p->pNext = pSet;
}

void CliPrompt(CliDev_t * const pCli)
{
	if (pCli->pPrompt != 0)
	{
		CliWrite(pCli, pCli->pPrompt, (int)strlen(pCli->pPrompt));
	}
}

int CliPuts(CliDev_t * const pCli, const char *pStr)
{
	int len = (int)strlen(pStr);

	CliWrite(pCli, pStr, len);

	return len;
}

int CliCmdHelp(CliDev_t * const pCli, int argc, char *argv[])
{
	(void)argc;
	(void)argv;

	for (CliCmdSet_t *pSet = &pCli->BaseSet; pSet != 0; pSet = pSet->pNext)
	{
		for (int i = 0; i < pSet->NbCmd; i++)
		{
			const CliCmd_t *pCmd = &pSet->pCmd[i];

			CliWrite(pCli, pCmd->pName, (int)strlen(pCmd->pName));

			if (pCmd->pHelp != 0)
			{
				CliWrite(pCli, s_HelpSep, (int)(sizeof(s_HelpSep) - 1));
				CliWrite(pCli, pCmd->pHelp, (int)strlen(pCmd->pHelp));
			}

			CliWrite(pCli, s_CrLf, 2);
		}
	}

	return 0;
}

// Split the line buffer in place into argv tokens. Returns argc.
static int CliTokenize(CliDev_t * const pCli)
{
	char *p = pCli->pLineBuf;
	char *end = pCli->pLineBuf + pCli->LineLen;
	int argc = 0;

	while (p < end && argc < pCli->ArgvMax)
	{
		// skip leading separators
		while (p < end && (*p == ' ' || *p == '\t'))
		{
			*p = 0;
			p++;
		}

		if (p >= end)
		{
			break;
		}

		pCli->pArgv[argc] = p;
		argc++;

		// advance to next separator
		while (p < end && *p != ' ' && *p != '\t')
		{
			p++;
		}
	}

	return argc;
}

// Match argv[0] across every set and run the handler. First match wins.
static int CliDispatch(CliDev_t * const pCli, int argc, char *argv[])
{
	if (argc < 1)
	{
		return 0;
	}

	for (CliCmdSet_t *pSet = &pCli->BaseSet; pSet != 0; pSet = pSet->pNext)
	{
		for (int i = 0; i < pSet->NbCmd; i++)
		{
			if (strcmp(argv[0], pSet->pCmd[i].pName) == 0)
			{
				return pSet->pCmd[i].Handler(pCli, argc, argv);
			}
		}
	}

	CliWrite(pCli, s_Unknown, (int)(sizeof(s_Unknown) - 1));
	CliWrite(pCli, argv[0], (int)strlen(argv[0]));
	CliWrite(pCli, s_CrLf, 2);

	return -1;
}

int CliInput(CliDev_t * const pCli, uint8_t Ch)
{
	int dispatched = 0;

	if (Ch == CLI_CH_CR || Ch == CLI_CH_LF)
	{
		// fold a CR LF or LF CR pair into a single line end
		if ((Ch == CLI_CH_LF && pCli->LastCh == CLI_CH_CR) ||
			(Ch == CLI_CH_CR && pCli->LastCh == CLI_CH_LF))
		{
			pCli->LastCh = Ch;
			return 0;
		}

		if (pCli->bEcho)
		{
			CliWrite(pCli, s_CrLf, 2);
		}

		pCli->pLineBuf[pCli->LineLen] = 0;

		if (pCli->LineLen > 0)
		{
			int argc = CliTokenize(pCli);

			if (argc > 0)
			{
				CliDispatch(pCli, argc, pCli->pArgv);
				dispatched = 1;
			}
		}

		pCli->LineLen = 0;
		CliPrompt(pCli);
		pCli->LastCh = Ch;

		return dispatched;
	}

	if (Ch == CLI_CH_BS || Ch == CLI_CH_DEL)
	{
		if (pCli->LineLen > 0)
		{
			pCli->LineLen--;

			if (pCli->bEcho)
			{
				CliWrite(pCli, s_BsErase, 3);
			}
		}

		pCli->LastCh = Ch;

		return 0;
	}

	// store printable bytes, drop everything else
	if (Ch >= 0x20 && Ch < CLI_CH_DEL)
	{
		if (pCli->LineLen < pCli->LineBufSize - 1)
		{
			pCli->pLineBuf[pCli->LineLen] = (char)Ch;
			pCli->LineLen++;

			if (pCli->bEcho)
			{
				CliWrite(pCli, (const char *)&Ch, 1);
			}
		}
	}

	pCli->LastCh = Ch;

	return 0;
}

int CliProcess(CliDev_t * const pCli)
{
	uint8_t buf[CLI_RX_CHUNK];
	int total = 0;
	int n;

	if (pCli->pIntrf == 0)
	{
		return 0;
	}

	while ((n = DeviceIntrfRx(pCli->pIntrf, pCli->IntrfAddr, buf, sizeof(buf))) > 0)
	{
		for (int i = 0; i < n; i++)
		{
			CliInput(pCli, buf[i]);
		}

		total += n;

		if (n < (int)sizeof(buf))
		{
			break;
		}
	}

	return total;
}
