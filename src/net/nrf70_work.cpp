/**-------------------------------------------------------------------------
@file nrf70_work.cpp
@brief Deferred nRF70 interrupt processing for bare-metal IOsonata.
@license MIT License
Copyright (c) 2026 I-SYST inc.
-------------------------------------------------------------------------*/
#include <atomic>
#include "net/nrf70_work.h"
#include "app_evt_handler.h"

static std::atomic<bool> s_Active(false);
static std::atomic<bool> s_Pending(false);
static std::atomic<bool> s_Queued(false);
static Nrf70WorkHandler_t s_Handler;
static void *s_pContext;

static void Nrf70WorkEvt(uint32_t Evt, void *pContext)
{
	(void)Evt;
	(void)pContext;
	s_Queued.store(false, std::memory_order_release);
	Nrf70WorkProcess();
	// A new interrupt can arrive during the callback.
	Nrf70WorkCheckStatus();
}

extern "C" void Nrf70WorkCheckStatus(void)
{
	bool expected;

	if (!s_Active.load(std::memory_order_acquire) ||
		!s_Pending.load(std::memory_order_acquire))
		return;
	expected = false;
	if (!s_Queued.compare_exchange_strong(expected, true,
		std::memory_order_acq_rel))
		return;
	if (!AppEvtHandlerQue(0, nullptr, Nrf70WorkEvt))
		s_Queued.store(false, std::memory_order_release);
}

extern "C" void Nrf70WorkInit(Nrf70WorkHandler_t Handler, void *pContext)
{
	s_Active.store(false, std::memory_order_release);
	s_Pending.store(false, std::memory_order_release);
	s_Queued.store(false, std::memory_order_release);
	s_Handler = Handler;
	s_pContext = pContext;
	s_Active.store(Handler != nullptr, std::memory_order_release);
}

extern "C" void Nrf70WorkIrq(void)
{
	if (s_Active.load(std::memory_order_acquire))
	{
		s_Pending.store(true, std::memory_order_release);
		Nrf70WorkCheckStatus();
	}
}

extern "C" bool Nrf70WorkPending(void)
{
	return s_Pending.load(std::memory_order_acquire);
}

extern "C" void Nrf70WorkProcess(void)
{
	if (!s_Active.load(std::memory_order_acquire))
		return;
	if (!s_Pending.exchange(false, std::memory_order_acq_rel))
		return;
	if (s_Handler != nullptr)
		s_Handler(s_pContext);
}

extern "C" void Nrf70WorkClose(void)
{
	// Caller must first disable the GPIO interrupt and ensure that no
	// foreground callback is in progress before closing the device.
	s_Active.store(false, std::memory_order_release);
	s_Pending.store(false, std::memory_order_release);
	s_Queued.store(false, std::memory_order_release);
	s_Handler = nullptr;
	s_pContext = nullptr;
}
