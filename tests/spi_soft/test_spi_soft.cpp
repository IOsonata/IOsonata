// SPDX-License-Identifier: MIT
// Host wire model for the real driver, not a replacement transfer engine.
#include <cassert>
#include <cstdio>
#include <vector>
#include "coredev/spi_soft.h"

static bool level[16];
static IOPINDIR direction[16];
static unsigned edges, assertions, releases, reads, sample, bits;
static bool idle, phase, lsb, shared;
static uint16_t reply, observed;
static std::vector<uint16_t> sent;
static void output(int pin, bool high)
{
	if(pin==0 && level[pin]!=high) ++edges;
	if(pin>=3 && pin<=4 && level[pin]!=high) high ? ++releases : ++assertions;
	level[pin]=high;
}
void IOPinSet(int,int pin) {output(pin,true);}
void IOPinClear(int,int pin) {output(pin,false);}
void IOPinSetDir(int,int pin,IOPINDIR dir) {direction[pin]=dir;}
extern "C" void IOPinConfig(int,int pin,int,IOPINDIR dir,IOPINRES,IOPINTYPE) {direction[pin]=dir;}
extern "C" void IOPinDisable(int,int pin) {direction[pin]=IOPINDIR_INPUT;}
void usDelay(uint32_t us) {assert(us>0);}
int IOPinRead(int,int pin)
{
	assert(pin==(shared?2:1));
	assert(level[0]==(phase?idle:!idle)); // configured sampling edge
	assert(edges==sample*2+(phase?2:1));
	if(shared) assert(direction[2]==IOPINDIR_INPUT);
	unsigned shift=lsb ? sample%bits : bits-1-sample%bits;
	if(level[2]) observed |= 1U<<shift;
	int value=(reply>>shift)&1;
	++sample; ++reads;
	if(sample%bits==0) {sent.push_back(observed); observed=0;}
	return value;
}
static void wire(unsigned n,bool pol,bool pha,bool order,bool three=false)
{
	bits=n;idle=pol;phase=pha;lsb=order;shared=three;
	edges=assertions=releases=reads=sample=observed=0;sent.clear();reply=0x9639;
}
static IOPinCfg_t pins[]={
	{0,0,IOPINOP_GPIO,IOPINDIR_OUTPUT,IOPINRES_NONE,IOPINTYPE_NORMAL},
	{0,1,IOPINOP_GPIO,IOPINDIR_INPUT,IOPINRES_NONE,IOPINTYPE_NORMAL},
	{0,2,IOPINOP_GPIO,IOPINDIR_OUTPUT,IOPINRES_NONE,IOPINTYPE_NORMAL},
	{0,3,IOPINOP_GPIO,IOPINDIR_OUTPUT,IOPINRES_NONE,IOPINTYPE_NORMAL},
	{0,4,IOPINOP_GPIO,IOPINDIR_OUTPUT,IOPINRES_NONE,IOPINTYPE_NORMAL}
};
int main()
{
	SPICfg_t cfg{};cfg.Mode=SPIMODE_MASTER;cfg.Phy=SPIPHY_NORMAL;
	cfg.pIOPinMap=pins;cfg.NbIOPins=5;cfg.Rate=100000;cfg.DataSize=8;
	cfg.ChipSel=SPICSEL_AUTO;cfg.DummyByte=0xA5;
	SPISoft dev;
	unsigned cases=0;
	for(unsigned n=4;n<=16;++n) for(unsigned mode=0;mode<4;++mode) for(unsigned order=0;order<2;++order)
	{
		cfg.DataSize=n;cfg.ClkPol=mode&2?SPICLKPOL_LOW:SPICLKPOL_HIGH;
		cfg.DataPhase=mode&1?SPIDATAPHASE_SECOND_CLK:SPIDATAPHASE_FIRST_CLK;
		cfg.BitOrder=order?SPIDATABIT_LSB:SPIDATABIT_MSB;
		assert(dev.Init(cfg)); wire(n,mode&2,mode&1,order);
		uint8_t data[]={0x65,0xB2,0xE9,0x37};uint8_t rx[4]{};
		int step=n>8?2:1;uint16_t mask=(1U<<n)-1;
		assert(dev.Transfer(1,data,rx,4)==4);
		assert(assertions==1 && releases==1 && level[4] && level[0]==idle);
		assert(edges==2*n*(4/step));
		for(int i=0;i<4;i+=step)
		{
			uint16_t tx=data[i]|(step==2?data[i+1]<<8:0);
			uint16_t got=rx[i]|(step==2?rx[i+1]<<8:0);
			assert(sent[i/step]==(tx&mask));assert(got==(reply&mask));
		}
		wire(n,mode&2,mode&1,order);
		assert(dev.Transfer(0,data,data,4)==4); // in place
		assert(data[0]==(reply&mask&255));
		++cases;
	}
	cfg.DataSize=8;cfg.ClkPol=SPICLKPOL_HIGH;cfg.DataPhase=SPIDATAPHASE_FIRST_CLK;cfg.BitOrder=SPIDATABIT_MSB;
	assert(dev.Init(cfg));wire(8,false,false,false);
	uint8_t cmd[]={0x81,0x03},rx[4]{};
	DeviceIntrf *base=&dev;
	assert(base->Read(0,cmd,2,rx,4)==4); // CS spans command + receive
	assert(assertions==1 && releases==1 && sent.size()==6);
	assert(sent[0]==0x81 && sent[1]==3);
	for(unsigned i=2;i<6;++i)assert(sent[i]==0xA5);
	wire(8,false,false,false);
	assert(base->Write(0,cmd,2,cmd,2)==2);
	assert(sent.size()==4 && sent[0]==0x81 && sent[2]==0x81);
	wire(8,false,false,false);
	assert(base->StartTx(0));assert(!base->StartRx(1));assert(dev.Rate(50000)==0);
	base->Reset(); // Reset retires the selected session and its framework lock.
	assert(base->StartTx(1));base->StopTx();
	dev.Disable();assert(dev.Transfer(0,cmd,rx,2)==0);dev.Enable();
	assert(dev.Rate(120000)==100000);assert(dev.Rate(1000000)==500000);assert(dev.Rate(0)==0);
	assert(dev.Transfer(99,cmd,rx,2)==0);assert(dev.Transfer(0,nullptr,nullptr,2)==0);
	cfg.DataSize=9;assert(dev.Init(cfg));wire(9,false,false,false);
	assert(dev.Transfer(0,cmd,rx,1)==0 && edges==0 && assertions==0);
	cfg.bDmaEn=true;assert(!dev.Init(cfg));cfg.bDmaEn=false;
	cfg.bIntEn=true;assert(!dev.Init(cfg));cfg.bIntEn=false;
	cfg.Mode=SPIMODE_SLAVE;assert(!dev.Init(cfg));cfg.Mode=SPIMODE_MASTER;
	// Optional pins, shared direction turnaround, and manual CS ownership.
	cfg.DataSize=8;cfg.Phy=SPIPHY_3WIRE;pins[1].PortNo=pins[1].PinNo=-1;
	assert(dev.Init(cfg));wire(8,false,false,false,true);
	assert(base->Read(0,nullptr,0,rx,2)==2 && direction[2]==IOPINDIR_INPUT);
	assert(dev.Transfer(0,cmd,rx,2)==0);
	assert(base->Tx(0,cmd,2)==2 && direction[2]==IOPINDIR_OUTPUT);
	cfg.Phy=SPIPHY_NORMAL;cfg.ChipSel=SPICSEL_MAN;
	assert(dev.Init(cfg));wire(8,false,false,false);
	assert(base->Tx(0,cmd,2)==2 && assertions==0 && releases==0);
	dev.PowerOff();assert(dev.Transfer(0,cmd,nullptr,2)==0);dev.Enable();
	wire(8,false,false,false);assert(base->Tx(0,cmd,2)==2);
	// Independent device storage, no hardware SPIInit symbol required.
	SPISoft second;assert(second.Init(cfg));assert(second.Rate(25000)==25000);
	assert(dev.Rate()!=second.Rate());
	printf("%u mode/width/order cases; generic APIs, CS, lifecycle and validation PASS\n",cases);
}
