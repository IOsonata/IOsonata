from pathlib import Path

path = Path('exemples/bluetooth/uart_ble.cpp')
text = path.read_text()
old = '''    BtAppInit(&s_BleAppCfg);
    UartBleOobInit();

    BtAppRun();

\treturn 0;
'''
new = '''    if (!BtAppInit(&s_BleAppCfg))
    {
        g_Uart.printf("BtAppInit failed\\r\\n");
        while (true)
        {
            __NOP();
        }
    }

    UartBleOobInit();
    BtAppRun();

    // BtAppRun is not expected to return. Keep embedded startup from falling
    // through newlib exit if a target implementation does return.
    g_Uart.printf("BtAppRun returned\\r\\n");
    while (true)
    {
        __NOP();
    }
'''
if old not in text:
    raise SystemExit('main init block not found')
path.write_text(text.replace(old, new, 1))
