/**-------------------------------------------------------------------------
@file	pei_matrix.h

@brief	Small C++ wrapper around the C PeiDev_t matrix

The matrix implementation lives in src/coredev/pei.c. C and C++ clients
share the same routing state and lifecycle; neither implementation forwards
hardware events through interrupts or requires dynamic allocation.

----------------------------------------------------------------------------*/
#ifndef __PEI_MATRIX_H__
#define __PEI_MATRIX_H__

class Pei {
public:
	explicit Pei(int DevNo = 0)
	{
		PeiInit(&vPei, DevNo);
	}

	~Pei()
	{
		PeiDeinit(&vPei);
	}

	Pei(const Pei &) = delete;
	Pei &operator=(const Pei &) = delete;

	operator PeiDev_t *()
	{
		return &vPei;
	}

	PeiDev_t *Dev()
	{
		return &vPei;
	}

	bool BindEvent(PeiEvent_t &Evt, PeiEndpoint_t Endpoint)
	{
		return PeiBindEvent(&vPei, &Evt, Endpoint);
	}

	bool UnbindEvent(PeiEvent_t Evt, PeiEndpoint_t Endpoint)
	{
		return PeiUnbindEvent(&vPei, Evt, Endpoint);
	}

	bool Connect(PeiEvent_t Evt, PeiEndpoint_t Task)
	{
		return PeiConnectTask(&vPei, Evt, Task);
	}

	bool Disconnect(PeiEvent_t Evt, PeiEndpoint_t Task)
	{
		return PeiDisconnectTask(&vPei, Evt, Task);
	}

	bool Enable(PeiEvent_t Evt)
	{
		return PeiEnableEvent(&vPei, Evt);
	}

	bool Disable(PeiEvent_t Evt)
	{
		return PeiDisableEvent(&vPei, Evt);
	}

	bool ReleaseEvent(PeiEvent_t &Evt)
	{
		return PeiReleaseEvent(&vPei, &Evt);
	}

	PeiEndpoint_t EventEndpoint(PeiEvent_t Evt) const
	{
		return PeiGetEventEndpoint(&vPei, Evt);
	}

private:
	PeiDev_t vPei;
};

#endif // __PEI_MATRIX_H__
