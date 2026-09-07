/**-------------------------------------------------------------------------
@file	usb_iso_loopback_libusb.c

@brief	Asynchronous libusb hardware test for UsbIsoIntrf.

Target firmware: exemples/usb/usb_iso_loopback.cpp
Default device: 1209:0003, interface 0, bidirectional endpoint 8.

The test submits one asynchronous ISO IN transfer and one asynchronous ISO OUT
transfer at the same time. IN is resubmitted across empty service intervals
until the device echoes the OUT frame. Alternate setting 0 is selected between
every active alternate so endpoint close/reopen transitions are exercised.

Use --manual-suspend-wake for the final suspend/resume phase. With alt 6 open
and OUT armed, put the host into real USB/system suspend, wake it, then press
Enter. The test requires the existing libusb handle to keep working; it does
not hide a failed resume by reopening the device.

@author	Hoang Nguyen Hoan
@date	Sep. 7, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved
----------------------------------------------------------------------------*/
#include <errno.h>
#include <libusb.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#define DEFAULT_VID		0x1209U
#define DEFAULT_PID		0x0003U
#define DEFAULT_INTERFACE	0
#define DEFAULT_EP		8U
#define DEFAULT_ROUNDS		32U
#define ROUND_TIMEOUT_MS	3000U
#define XFER_TIMEOUT_MS		1000U
#define MAX_MPS			63U

static const uint16_t s_Mps[6] = { 9U, 17U, 25U, 33U, 49U, 63U };

typedef struct __Iso_Round IsoRound_t;

typedef struct __Iso_Xfer_Context {
	IsoRound_t *pRound;
	bool In;
} IsoXferContext_t;

struct __Iso_Round {
	libusb_device_handle *Handle;
	libusb_transfer *In;
	libusb_transfer *Out;
	IsoXferContext_t InContext;
	IsoXferContext_t OutContext;
	uint8_t InBuffer[MAX_MPS];
	uint8_t OutBuffer[MAX_MPS];
	uint16_t Length;
	bool InDone;
	bool OutDone;
	bool InSubmitted;
	bool OutSubmitted;
	int Error;
	int InActual;
	unsigned EmptyIn;
};

typedef struct __Options {
	uint16_t Vid;
	uint16_t Pid;
	int Interface;
	uint8_t Ep;
	unsigned Rounds;
	bool ManualSuspendWake;
} Options_t;

static uint64_t TimeMs(void)
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return ((uint64_t)tv.tv_sec * 1000ULL) + (uint64_t)(tv.tv_usec / 1000);
}

static void PrintUsage(const char *pName)
{
	fprintf(stderr,
		"Usage: %s [--vid N] [--pid N] [--interface N] [--ep N] "
		"[--rounds N] [--manual-suspend-wake]\n", pName);
}

static bool ParseUnsigned(const char *pText, unsigned long Max,
						 unsigned long *pValue)
{
	char *pEnd = NULL;
	errno = 0;
	unsigned long value = strtoul(pText, &pEnd, 0);
	if (errno != 0 || pEnd == pText || *pEnd != '\0' || value > Max)
	{
		return false;
	}
	*pValue = value;
	return true;
}

static bool ParseArgs(int argc, char **argv, Options_t *pOpt)
{
	pOpt->Vid = DEFAULT_VID;
	pOpt->Pid = DEFAULT_PID;
	pOpt->Interface = DEFAULT_INTERFACE;
	pOpt->Ep = DEFAULT_EP;
	pOpt->Rounds = DEFAULT_ROUNDS;
	pOpt->ManualSuspendWake = false;

	for (int i = 1; i < argc; i++)
	{
		if (strcmp(argv[i], "--manual-suspend-wake") == 0)
		{
			pOpt->ManualSuspendWake = true;
			continue;
		}
		if (i + 1 >= argc)
		{
			return false;
		}

		unsigned long value = 0;
		if (strcmp(argv[i], "--vid") == 0)
		{
			if (!ParseUnsigned(argv[++i], 0xFFFFUL, &value)) return false;
			pOpt->Vid = (uint16_t)value;
		}
		else if (strcmp(argv[i], "--pid") == 0)
		{
			if (!ParseUnsigned(argv[++i], 0xFFFFUL, &value)) return false;
			pOpt->Pid = (uint16_t)value;
		}
		else if (strcmp(argv[i], "--interface") == 0)
		{
			if (!ParseUnsigned(argv[++i], 255UL, &value)) return false;
			pOpt->Interface = (int)value;
		}
		else if (strcmp(argv[i], "--ep") == 0)
		{
			if (!ParseUnsigned(argv[++i], 15UL, &value) || value == 0UL)
				return false;
			pOpt->Ep = (uint8_t)value;
		}
		else if (strcmp(argv[i], "--rounds") == 0)
		{
			if (!ParseUnsigned(argv[++i], 1000000UL, &value) || value == 0UL)
				return false;
			pOpt->Rounds = (unsigned)value;
		}
		else
		{
			return false;
		}
	}
	return true;
}

static void SetError(IsoRound_t *pRound, int Error)
{
	if (pRound->Error == 0)
	{
		pRound->Error = Error != 0 ? Error : LIBUSB_ERROR_OTHER;
	}
}

static bool PacketComplete(const struct libusb_transfer *pTransfer,
							  int *pActual)
{
	if (pTransfer->status != LIBUSB_TRANSFER_COMPLETED ||
		pTransfer->num_iso_packets != 1)
	{
		return false;
	}
	const struct libusb_iso_packet_descriptor *p =
		&pTransfer->iso_packet_desc[0];
	if (p->status != LIBUSB_TRANSFER_COMPLETED)
	{
		return false;
	}
	*pActual = (int)p->actual_length;
	return true;
}

static void LIBUSB_CALL IsoComplete(struct libusb_transfer *pTransfer)
{
	IsoXferContext_t *pContext = (IsoXferContext_t *)pTransfer->user_data;
	IsoRound_t *pRound = pContext->pRound;
	int actual = 0;

	if (pContext->In)
	{
		pRound->InSubmitted = false;
		if (!PacketComplete(pTransfer, &actual))
		{
			SetError(pRound, LIBUSB_ERROR_IO);
			pRound->InDone = true;
			return;
		}
		if (actual == 0 && pRound->Error == 0)
		{
			pRound->EmptyIn++;
			int rc = libusb_submit_transfer(pTransfer);
			if (rc == LIBUSB_SUCCESS)
			{
				pRound->InSubmitted = true;
				return;
			}
			SetError(pRound, rc);
		}
		pRound->InActual = actual;
		pRound->InDone = true;
		return;
	}

	pRound->OutSubmitted = false;
	if (!PacketComplete(pTransfer, &actual) || actual != pRound->Length)
	{
		SetError(pRound, LIBUSB_ERROR_IO);
	}
	pRound->OutDone = true;
}

static void PumpCancel(libusb_context *pUsb, IsoRound_t *pRound)
{
	if (pRound->InSubmitted)
	{
		(void)libusb_cancel_transfer(pRound->In);
	}
	if (pRound->OutSubmitted)
	{
		(void)libusb_cancel_transfer(pRound->Out);
	}

	const uint64_t deadline = TimeMs() + 500U;
	while ((pRound->InSubmitted || pRound->OutSubmitted) && TimeMs() < deadline)
	{
		struct timeval tv = { 0, 10000 };
		(void)libusb_handle_events_timeout(pUsb, &tv);
	}
}

static int RunRound(libusb_context *pUsb, libusb_device_handle *pHandle,
					uint8_t Ep, uint8_t Alt, uint16_t Mps, unsigned Seq)
{
	IsoRound_t round;
	memset(&round, 0, sizeof(round));
	round.Handle = pHandle;
	round.Length = (Seq % 3U) == 0U ? Mps :
		(Seq % 3U) == 1U ? (uint16_t)(Mps > 1U ? Mps - 1U : Mps) : 1U;

	for (uint16_t i = 0U; i < round.Length; i++)
	{
		round.OutBuffer[i] = (uint8_t)(0xA5U ^ Alt ^ (uint8_t)Seq ^ (uint8_t)i);
	}
	if (round.Length >= 4U)
	{
		round.OutBuffer[0] = Alt;
		round.OutBuffer[1] = (uint8_t)Seq;
		round.OutBuffer[2] = (uint8_t)(Seq >> 8);
		round.OutBuffer[3] = (uint8_t)(Seq >> 16);
	}

	round.In = libusb_alloc_transfer(1);
	round.Out = libusb_alloc_transfer(1);
	if (round.In == NULL || round.Out == NULL)
	{
		libusb_free_transfer(round.In);
		libusb_free_transfer(round.Out);
		return LIBUSB_ERROR_NO_MEM;
	}

	round.InContext.pRound = &round;
	round.InContext.In = true;
	round.OutContext.pRound = &round;
	round.OutContext.In = false;

	libusb_fill_iso_transfer(round.In, pHandle, (unsigned char)(0x80U | Ep),
		round.InBuffer, Mps, 1, IsoComplete, &round.InContext, XFER_TIMEOUT_MS);
	libusb_set_iso_packet_lengths(round.In, Mps);
	libusb_fill_iso_transfer(round.Out, pHandle, Ep,
		round.OutBuffer, round.Length, 1, IsoComplete, &round.OutContext,
		XFER_TIMEOUT_MS);
	libusb_set_iso_packet_lengths(round.Out, round.Length);

	// IN is submitted first so it is definitely pending when OUT is submitted.
	// That makes each round an actual simultaneous bidirectional ISO test.
	int rc = libusb_submit_transfer(round.In);
	if (rc == LIBUSB_SUCCESS)
	{
		round.InSubmitted = true;
		rc = libusb_submit_transfer(round.Out);
		if (rc == LIBUSB_SUCCESS)
		{
			round.OutSubmitted = true;
		}
	}
	if (rc != LIBUSB_SUCCESS)
	{
		SetError(&round, rc);
	}

	const uint64_t deadline = TimeMs() + ROUND_TIMEOUT_MS;
	while (round.Error == 0 && (!round.InDone || !round.OutDone) &&
		TimeMs() < deadline)
	{
		struct timeval tv = { 0, 20000 };
		rc = libusb_handle_events_timeout(pUsb, &tv);
		if (rc != LIBUSB_SUCCESS && rc != LIBUSB_ERROR_INTERRUPTED)
		{
			SetError(&round, rc);
		}
	}
	if (round.Error == 0 && (!round.InDone || !round.OutDone))
	{
		SetError(&round, LIBUSB_ERROR_TIMEOUT);
	}

	if (round.Error == 0 &&
		(round.InActual != round.Length ||
		 memcmp(round.InBuffer, round.OutBuffer, round.Length) != 0))
	{
		SetError(&round, LIBUSB_ERROR_IO);
	}

	if (round.Error != 0)
	{
		PumpCancel(pUsb, &round);
	}
	libusb_free_transfer(round.In);
	libusb_free_transfer(round.Out);
	return round.Error;
}

static int RunAlt(libusb_context *pUsb, libusb_device_handle *pHandle,
				  int Interface, uint8_t Ep, uint8_t Alt, uint16_t Mps,
				  unsigned Rounds)
{
	int rc = libusb_set_interface_alt_setting(pHandle, Interface, 0);
	if (rc != LIBUSB_SUCCESS)
	{
		fprintf(stderr, "alt 0 before alt %u failed: %s\n",
			Alt, libusb_error_name(rc));
		return rc;
	}
	rc = libusb_set_interface_alt_setting(pHandle, Interface, Alt);
	if (rc != LIBUSB_SUCCESS)
	{
		fprintf(stderr, "alt %u failed: %s\n", Alt, libusb_error_name(rc));
		return rc;
	}

	unsigned empty = 0U;
	for (unsigned seq = 0U; seq < Rounds; seq++)
	{
		rc = RunRound(pUsb, pHandle, Ep, Alt, Mps, seq);
		if (rc != 0)
		{
			fprintf(stderr,
				"FAIL alt %u MPS %u round %u: %s\n",
				Alt, Mps, seq, libusb_error_name(rc));
			return rc;
		}
		(void)empty;
	}
	printf("PASS alt %u MPS %u: %u simultaneous IN/OUT rounds\n",
		Alt, Mps, Rounds);
	return LIBUSB_SUCCESS;
}

static int ManualSuspendWake(libusb_context *pUsb,
							 libusb_device_handle *pHandle,
							 const Options_t *pOpt)
{
	int rc = libusb_set_interface_alt_setting(pHandle, pOpt->Interface, 0);
	if (rc == LIBUSB_SUCCESS)
	{
		rc = libusb_set_interface_alt_setting(pHandle, pOpt->Interface, 6);
	}
	if (rc != LIBUSB_SUCCESS)
	{
		return rc;
	}

	printf("\nSuspend/wake phase: alt 6 is open and EP%u OUT is armed.\n", pOpt->Ep);
	printf("Put the host into real USB/system suspend now. After wake, press Enter.\n");
	fflush(stdout);
	int ch;
	do { ch = getchar(); } while (ch != '\n' && ch != EOF);

	// Do not reopen the device here. Success therefore proves the existing USB
	// function resumed and the generic interface restored service correctly.
	rc = RunRound(pUsb, pHandle, pOpt->Ep, 6U, s_Mps[5], 0xA55AU);
	if (rc != LIBUSB_SUCCESS)
	{
		fprintf(stderr, "FAIL suspend/wake resume: %s\n", libusb_error_name(rc));
		return rc;
	}
	printf("PASS suspend/wake: existing handle and alt-6 ISO path resumed\n");
	return LIBUSB_SUCCESS;
}

int main(int argc, char **argv)
{
	Options_t opt;
	if (!ParseArgs(argc, argv, &opt))
	{
		PrintUsage(argv[0]);
		return 2;
	}

	libusb_context *usb = NULL;
	int rc = libusb_init(&usb);
	if (rc != LIBUSB_SUCCESS)
	{
		fprintf(stderr, "libusb_init: %s\n", libusb_error_name(rc));
		return 1;
	}

	libusb_device_handle *handle =
		libusb_open_device_with_vid_pid(usb, opt.Vid, opt.Pid);
	if (handle == NULL)
	{
		fprintf(stderr, "device %04x:%04x not found\n", opt.Vid, opt.Pid);
		libusb_exit(usb);
		return 1;
	}

	(void)libusb_set_auto_detach_kernel_driver(handle, 1);
	rc = libusb_claim_interface(handle, opt.Interface);
	if (rc != LIBUSB_SUCCESS)
	{
		fprintf(stderr, "claim interface %d: %s\n",
			opt.Interface, libusb_error_name(rc));
		libusb_close(handle);
		libusb_exit(usb);
		return 1;
	}

	printf("USB ISO loopback %04x:%04x interface %d EP%u\n",
		opt.Vid, opt.Pid, opt.Interface, opt.Ep);
	for (uint8_t alt = 1U; alt <= 6U; alt++)
	{
		rc = RunAlt(usb, handle, opt.Interface, opt.Ep,
			alt, s_Mps[alt - 1U], opt.Rounds);
		if (rc != LIBUSB_SUCCESS)
		{
			break;
		}
	}

	if (rc == LIBUSB_SUCCESS && opt.ManualSuspendWake)
	{
		rc = ManualSuspendWake(usb, handle, &opt);
	}
	else if (rc == LIBUSB_SUCCESS)
	{
		printf("Suspend/wake not run; use --manual-suspend-wake for full hardware validation.\n");
	}

	(void)libusb_set_interface_alt_setting(handle, opt.Interface, 0);
	libusb_release_interface(handle, opt.Interface);
	libusb_close(handle);
	libusb_exit(usb);

	if (rc != LIBUSB_SUCCESS)
	{
		return 1;
	}
	printf("USB ISO hardware test: PASS%s\n",
		opt.ManualSuspendWake ? " including suspend/wake" : " (no suspend/wake)");
	return 0;
}
