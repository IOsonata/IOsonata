// Algorithm mock: exercise units, timing, failure handling and FuelGauge lifecycle.
// The calls follow the sdk-nrfxlib v3.4.0 API: an error code is returned and
// values come back through pointers, which are left untouched on error.
static float s_Soc = 52.34f, s_Dt, s_V, s_I, s_T, s_IdleCurrent;
static float s_ChargeLimit, s_TermCurrent;
static float s_Tte = 1234.0f, s_Ttf = 567.0f;
static int s_InitResult, s_ExtResult, s_LowCount, s_ProcessCount;
static int s_ProcessResult, s_TimeResult, s_IdleResult;
static bool s_Vbus;
static nrf_fuel_gauge_charge_state s_ChargeState;
static const void *s_State;
static size_t s_StateSize;
const size_t nrf_fuel_gauge_state_size = 16;

int nrf_fuel_gauge_init(const nrf_fuel_gauge_init_parameters *p, float *)
{
	assert(p->model != NULL && p->opt_params == NULL && p->lock_func == NULL);
	s_V = p->v0; s_I = p->i0; s_T = p->t0;
	s_State = p->state; s_StateSize = p->state_size;
	return s_InitResult;
}
int nrf_fuel_gauge_process(float v, float i, float t, float dt, float *soc,
	nrf_fuel_gauge_state_info *)
{
	assert(soc != NULL);
	s_V = v; s_I = i; s_T = t; s_Dt = dt; ++s_ProcessCount;
	if (s_ProcessResult != 0)
		return s_ProcessResult;
	*soc = s_Soc;
	return 0;
}
int nrf_fuel_gauge_tte_get(float *p)
{
	assert(p != NULL);
	if (s_TimeResult != 0)
		return s_TimeResult;
	*p = s_Tte;
	return 0;
}
int nrf_fuel_gauge_ttf_get(float *p)
{
	assert(p != NULL);
	if (s_TimeResult != 0)
		return s_TimeResult;
	*p = s_Ttf;
	return 0;
}
int nrf_fuel_gauge_idle_set(float, float, float i)
{
	s_IdleCurrent = i;
	return s_IdleResult;
}
int nrf_fuel_gauge_state_get(void *p, size_t n)
{
	assert(n >= nrf_fuel_gauge_state_size);
	memset(p, 0xAB, nrf_fuel_gauge_state_size);
	return 0;
}
int nrf_fuel_gauge_ext_state_update(nrf_fuel_gauge_ext_state_info_type type,
	nrf_fuel_gauge_ext_state_info_data *p)
{
	switch (type)
	{
	case NRF_FUEL_GAUGE_EXT_STATE_INFO_VBUS_CONNECTED: s_Vbus = true; break;
	case NRF_FUEL_GAUGE_EXT_STATE_INFO_VBUS_DISCONNECTED: s_Vbus = false; break;
	case NRF_FUEL_GAUGE_EXT_STATE_INFO_CHARGE_CURRENT_LIMIT:
		s_ChargeLimit = p->charge_current_limit; break;
	case NRF_FUEL_GAUGE_EXT_STATE_INFO_TERM_CURRENT:
		s_TermCurrent = p->charge_term_current; break;
	case NRF_FUEL_GAUGE_EXT_STATE_INFO_CHARGE_STATE_CHANGE:
		s_ChargeState = p->charge_state; break;
	default: assert(false);
	}
	return s_ExtResult;
}
static PowerMgnt *s_ExpectedPower;
static void FuelLow(PowerMgnt *pm) { assert(pm == s_ExpectedPower); ++s_LowCount; }

static void TestFuelGauge()
{
	MockIntrf bus;
	PmnPM1300FuelGauge pm, other;
	FuelGauge &gauge = pm;
	PowerMgnt &power = pm;
	Device &device = gauge;
	BatProfile_t profile = BatteryProfile(3435, 10);
	FuelGaugeCfg_t cfg = { NPM1300_I2C_7BITS_ADDR, profile, FuelLow };
	battery_model model = {};
	Npm1300BatterySample_t sample = {};
	uint8_t state[16];
	int calls;
	s_ExpectedPower = &power;

	assert(gauge.Level() == UINT16_MAX);
	assert(!pm.StartFuelGauge(model, 0));
	assert(gauge.Init(cfg, &bus, &power));
	assert(device.Valid());
	bus.SetAdcVbat(768); bus.SetAdcNtc(512);
	bus.vReg[0x510] = 4;
	bus.vReg[0x30A] = 207; bus.vReg[0x30B] = 1;
	bus.vReg[0x518] = 128;
	assert(power.SetCharge(PWRMGNT_CHARGE_TYPE_NORMAL, 4200, 400) == 400);
	assert(pm.ReadBatterySample(sample));
	assert(sample.VoltageMv == 3750 && sample.CurrentUa == 560547);
	assert(sample.TemperatureDeciC == 250);
	assert(gauge.Voltage() == 3750 && gauge.Temperature() == 250);
	assert(!pm.StartFuelGauge(model, 0, state, 15));
	assert(!pm.StartFuelGauge(model, 0, NULL, 16));
	s_InitResult = -1;
	assert(!pm.StartFuelGauge(model, 0));
	s_InitResult = 0;
	s_ExtResult = -1;
	assert(!pm.StartFuelGauge(model, 0));
	s_ExtResult = 0;
	s_ProcessResult = -22;
	assert(!pm.StartFuelGauge(model, 0) && gauge.Level() == UINT16_MAX);
	s_ProcessResult = 0;
	assert(pm.StartFuelGauge(model, 100));
	assert(s_State == NULL && s_StateSize == 0);
	assert(gauge.Level() == 523 && s_Dt == 0.0f);
	assert(s_V == 3.75f && fabsf(s_I - 0.560547f) < 0.000001f && s_T == 25.0f);
	assert(fabsf(s_ChargeLimit - 0.4f) < 0.00001f);
	assert(fabsf(s_TermCurrent - 0.04f) < 0.00001f);
	assert(!s_Vbus && s_ChargeState == NRF_FUEL_GAUGE_CHARGE_STATE_IDLE);
	assert(other.Init(cfg, &bus, &other));
	assert(!other.StartFuelGauge(model, 0));
	assert(gauge.Level() == 523);
	assert(pm.UpdateFuelGauge(1600) && s_Dt == 1.5f);
	assert(pm.TimeToEmpty() == 1234 && pm.TimeToFull() == 567);
	s_Tte = s_Ttf = NAN;
	assert(isnan(pm.TimeToEmpty()) && isnan(pm.TimeToFull()));
	s_Tte = 1234; s_Ttf = 567;
	s_TimeResult = -61;
	assert(isnan(pm.TimeToEmpty()) && isnan(pm.TimeToFull()));
	s_TimeResult = 0;
	// A refused sample invalidates the level and is not counted in the next
	// time delta.
	calls = s_ProcessCount;
	s_ProcessResult = -22;
	assert(!pm.UpdateFuelGauge(1650) && gauge.Level() == UINT16_MAX);
	assert(s_ProcessCount == calls + 1 && isnan(pm.TimeToEmpty()));
	s_ProcessResult = 0;
	bus.vReg[NPM1300_BCHARGER_BCHG_CHARGE_STATUS_REG] = 1U << 3;
	assert(pm.UpdateFuelGauge(1700) && fabsf(s_Dt - 0.1f) < 0.00001f);
	assert(gauge.Level() == 523);
	assert(s_ChargeState == NRF_FUEL_GAUGE_CHARGE_STATE_CC);
	bus.vReg[NPM1300_BCHARGER_BCHG_CHARGE_STATUS_REG] = 0;
	assert(pm.FuelGaugeStateSize() == 16);
	assert(!pm.SaveFuelGaugeState(NULL, 16));
	assert(!pm.SaveFuelGaugeState(state, 15));
	assert(pm.SaveFuelGaugeState(state, 16) && state[15] == 0xAB);
	assert(pm.FuelGaugeIdle(100) && fabsf(s_IdleCurrent - 0.0001f) < 0.000001f);
	assert(!pm.FuelGaugeIdle(-1));
	s_IdleResult = -22;
	assert(!pm.FuelGaugeIdle(100));
	s_IdleResult = 0;
	calls = s_ProcessCount;
	bus.vFailReadReg = NPM1300_ADC_ADCVBAT_RESULTMSB_REG;
	sample.VoltageMv = 123;
	assert(!pm.ReadBatterySample(sample) && sample.VoltageMv == 123);
	assert(!pm.UpdateFuelGauge(2600) && gauge.Level() == UINT16_MAX);
	assert(s_ProcessCount == calls && isnan(pm.TimeToEmpty()));
	bus.vFailReadReg = 0xFFFF;
	assert(pm.UpdateFuelGauge(3600) && fabsf(s_Dt - 1.9f) < 0.00001f);
	assert(!pm.UpdateFuelGauge(3600));
	assert(!pm.UpdateFuelGauge(3500));
	s_Soc = NAN;
	assert(!pm.UpdateFuelGauge(4600) && gauge.Level() == UINT16_MAX);
	s_Soc = 101;
	assert(!pm.UpdateFuelGauge(5600));
	s_Soc = 10;
	assert(pm.UpdateFuelGauge(6600) && gauge.Level() == 100 && s_LowCount == 1);
	assert(pm.UpdateFuelGauge(7600) && s_LowCount == 1);
	s_Soc = 15;
	assert(pm.UpdateFuelGauge(8600));
	s_Soc = 9;
	assert(pm.UpdateFuelGauge(9600) && s_LowCount == 2);
	s_Soc = 50;
	// State saved by another library version: the library refuses it
	s_InitResult = -EBADF;
	assert(!pm.StartFuelGauge(model, UINT32_MAX - 499, state, sizeof(state)));
	assert(gauge.Level() == UINT16_MAX);
	s_InitResult = 0;
	assert(pm.StartFuelGauge(model, UINT32_MAX - 499, state, sizeof(state)));
	assert(s_State == state && s_StateSize == sizeof(state));
	assert(pm.UpdateFuelGauge(500) && s_Dt == 1.0f);
	bus.vReg[NPM1300_VBUSIN_VBUSIN_STATUS_REG] = 1;
	bus.vReg[NPM1300_BCHARGER_BCHG_CHARGE_STATUS_REG] = 1U << 4;
	assert(pm.UpdateFuelGauge(1500) && s_Vbus);
	assert(s_ChargeState == NRF_FUEL_GAUGE_CHARGE_STATE_CV);
	assert(!pm.EnterShipMode() && gauge.Level() == 500);
	bus.vReg[NPM1300_VBUSIN_VBUSIN_STATUS_REG] = 0;
	assert(pm.EnterShipMode() && gauge.Level() == UINT16_MAX);
	assert(other.StartFuelGauge(model, 0));
	Device &otherDevice = other;
	otherDevice.Reset();
	assert(other.Level() == UINT16_MAX);
}
