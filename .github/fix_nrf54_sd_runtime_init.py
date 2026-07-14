from pathlib import Path

sdh_path = Path('ARM/Nordic/nRF54/src/nrf_sdh.c')
system_path = Path('ARM/Nordic/nRF54/src/system_nrf54l.c')

sdh = sdh_path.read_text()
system = system_path.read_text()

old = '''void NrfSdhEarlyInit(void)
{
\tif (softdevice_reset_done)
\t{
\t\treturn;
\t}

\t/* The reset entry initializes the SoftDevice static and dynamic RAM.
\t * Clear that reserved range before calling it. Clearing the range later,
\t * immediately before the first SVC, destroys the state just initialized by
\t * the reset entry. */
\tconst uintptr_t app_ram_start = (uintptr_t)SystemRamStart();
\tif (app_ram_start > 0x20000000UL)
\t{
\t\tmemset((void *)0x20000000UL, 0,
\t\t\t   app_ram_start - 0x20000000UL);
\t}

\tsoftdevice_vector_forward_address = FIXED_PARTITION_OFFSET(softdevice_partition);
#ifdef CONFIG_BOOTLOADER_MCUBOOT
\tsoftdevice_vector_forward_address += CONFIG_ROM_START_OFFSET;
#endif

\tCallSoftDeviceResetHandler();
\tsoftdevice_reset_done = true;
}
'''
new = '''void NrfSdhEarlyInit(void)
{
\tif (softdevice_reset_done)
\t{
\t\treturn;
\t}

\t/* The reset entry initializes the SoftDevice static and dynamic RAM.
\t * Clear that reserved range before calling it. This function runs from an
\t * early constructor, after the C runtime initialized .data/.bss but before
\t * main() initializes UART, timers, or other application peripherals. */
\tconst uintptr_t app_ram_start = (uintptr_t)SystemRamStart();
\tif (app_ram_start > 0x20000000UL)
\t{
\t\tmemset((void *)0x20000000UL, 0,
\t\t\t   app_ram_start - 0x20000000UL);
\t}

\tsoftdevice_vector_forward_address = FIXED_PARTITION_OFFSET(softdevice_partition);
#ifdef CONFIG_BOOTLOADER_MCUBOOT
\tsoftdevice_vector_forward_address += CONFIG_ROM_START_OFFSET;
#endif

\tCallSoftDeviceResetHandler();
\tsoftdevice_reset_done = true;
}

#if defined(__GNUC__)
/* ResetEntry calls the C runtime before main(). Constructor priority 101 runs
 * before normal C++ static constructors and before application HardwareInit().
 * SystemInit is too early because the following .bss initialization clears the
 * vector address and softdevice_reset_done flag. */
static void NrfSdhRuntimeInit(void) __attribute__((constructor(101)));
static void NrfSdhRuntimeInit(void)
{
\tNrfSdhEarlyInit();
}
#endif
'''
if old not in sdh:
    raise SystemExit('nrf_sdh early init block not found')
sdh = sdh.replace(old, new, 1)

old_decl = '''#if defined(S145)
extern void NrfSdhEarlyInit(void);
#endif

'''
if old_decl not in system:
    raise SystemExit('system early init declaration not found')
system = system.replace(old_decl, '', 1)

old_call = '''    #if defined(S145)
        /* Initialize the SoftDevice image before clocks, UART, timers or any
         * other application peripheral. The reset entry may change peripheral
         * and clock state, so it must not be called later from BtAppInit. */
        NrfSdhEarlyInit();
    #endif

'''
if old_call not in system:
    raise SystemExit('SystemInit early reset call not found')
system = system.replace(old_call, '', 1)

sdh_path.write_text(sdh)
system_path.write_text(system)
