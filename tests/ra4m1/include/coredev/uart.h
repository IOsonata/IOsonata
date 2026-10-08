/* REDUCED UART/DevIntrf test contract copied from d35dbeb's production API.
 * No std::atomic, C++ wrapper or newlib dependency is tested by this shim.
 */
#ifndef TEST_UART_H
#define TEST_UART_H
#include <stdint.h>
#include <stddef.h>
#include "cfifo.h"
typedef enum {UART_PARITY_NONE=-1, UART_PARITY_ODD=0, UART_PARITY_EVEN=1,
 UART_PARITY_MARK=2,UART_PARITY_SPACE=3} UART_PARITY;
typedef enum {UART_FLWCTRL_NONE,UART_FLWCTRL_XONXOFF,UART_FLWCTRL_HW} UART_FLWCTRL;
typedef enum {UART_MODE_UART,UART_MODE_USART,UART_MODE_NET} UART_MODE;
typedef enum {UART_DUPLEX_FULL,UART_DUPLEX_HALF} UART_DUPLEX;
typedef enum {UART_EVT_RXTIMEOUT,UART_EVT_RXDATA,UART_EVT_TXREADY,UART_EVT_LINESTATE} UART_EVT;
#define UART_LINESTATE_BRK (1U<<2)
#define UART_LINESTATE_FRMERR (1U<<4)
#define UART_LINESTATE_PARERR (1U<<5)
#define UART_LINESTATE_OVR (1U<<6)
#define UART_RETRY_MAX 100
#define DEVINTRF_TYPE_UART 8
struct DevIntrf_t;
typedef int (*DevIntrfEvtHandler_t)(DevIntrf_t*,int,uint8_t*,int);
struct UARTDEV;
typedef UARTDEV UARTDev_t;
typedef int (*UARTEvtHandler_t)(UARTDEV*,UART_EVT,uint8_t*,int);
struct TestAtomicFlag {bool Value;};
static inline void atomic_flag_clear(TestAtomicFlag *f) {f->Value=false;}
#pragma pack(push,4)
struct DevIntrf_t {
 void *pDevData; int IntPrio; DevIntrfEvtHandler_t EvtCB; TestAtomicFlag bBusy;
 int MaxRetry, EnCnt, Type; bool bDma,bIntEn,bTxReady,bNoStop;
 void (*Disable)(DevIntrf_t*); void (*Enable)(DevIntrf_t*);
 uint32_t (*GetRate)(DevIntrf_t*); uint32_t (*SetRate)(DevIntrf_t*,uint32_t);
 bool (*StartRx)(DevIntrf_t*,uint32_t); int (*RxData)(DevIntrf_t*,uint8_t*,int);
 void (*StopRx)(DevIntrf_t*); bool (*StartTx)(DevIntrf_t*,uint32_t);
 int (*TxData)(DevIntrf_t*,const uint8_t*,int);
 int (*TxSrData)(DevIntrf_t*,const uint8_t*,int);
 void (*StopTx)(DevIntrf_t*); void (*Reset)(DevIntrf_t*); void (*PowerOff)(DevIntrf_t*);
 void *(*GetHandle)(DevIntrf_t*);
};
struct UARTCfg_t {
 int DevNo; const void *pIOPinMap; int NbIOPins,Rate,DataBits; UART_PARITY Parity;
 int StopBits; UART_FLWCTRL FlowControl; bool bIntMode; int IntPrio;
 UARTEvtHandler_t EvtCallback; bool bFifoBlocking; int RxMemSize; uint8_t *pRxMem;
 int TxMemSize; uint8_t *pTxMem; bool bDMAMode,bIrDAMode,bIrDAInvert,bIrDAFixPulse;
 int IrDAPulseDiv; UART_DUPLEX Duplex; UART_MODE Mode;
};
typedef UARTCfg_t UARTCFG;
struct UARTDEV {
 UART_MODE Mode; UART_DUPLEX Duplex; int Rate,DataBits; UART_PARITY Parity;
 int StopBits; UART_FLWCTRL FlowControl; bool bIrDAMode,bIrDAInvert,bIrDAFixPulse;
 int IrDAPulseDiv; DevIntrf_t DevIntrf; UARTEvtHandler_t EvtCallback; void *pObj;
 hCFifo_t hRxFifo,hTxFifo; uint32_t LineState; int hStdIn,hStdOut;
 uint32_t RxOvrErrCnt,ParErrCnt,FramErrCnt,RxDropCnt,TxDropCnt;
 volatile bool bRxReady,bTxReady;
};
#pragma pack(pop)
extern "C" {
bool UARTInit(UARTDEV*, const UARTCFG*);
void UARTSetCtrlLineState(UARTDEV*,uint32_t);
UARTDEV const *UARTGetInstance(int);
}
#endif
