#include "syslog.h"

alignas(4) static uint8_t s_Memory[SYSLOG_MEMSIZE(2, 32)];
static const SysLogCfg_t s_Config = {s_Memory, sizeof(s_Memory), 32, false};

int main()
{
	SysLog *log = SysLogGetInstance();
	log->Init(s_Config);
	uint32_t code = 0;
	while (true)
	{
		log->Log(SYSSTATUS_TYPE_ERR | (++code & 0xffff), "status");
	}
}
