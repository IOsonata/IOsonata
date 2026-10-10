/* STM32WBA65 G memory variant.
 * Application linker selection, independent of the MCU library build.
 * Nonsecure/secure boot, option bytes and reserved radio regions must match
 * the target provisioning configuration before programming.
 */
SEARCH_DIR(.)
SEARCH_DIR("../../../../../../ARM/ldscript")
GROUP(-lstdc++ -lgcc -lc)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 0x0F8000
  NVM0 (rw) : ORIGIN = 0x080F8000, LENGTH = 0x4000
  NVM1 (rw) : ORIGIN = 0x080FC000, LENGTH = 0x4000
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x40000
}

/* The application NVM partitions are excluded from the image. */
SECTIONS
{
  nvm0 (NOLOAD) :
  {
    __start_nvm0 = .;
    KEEP(*(nvm0))
    . = ORIGIN(NVM0) + LENGTH(NVM0);
    __stop_nvm0 = .;
  } > NVM0
  nvm1 (NOLOAD) :
  {
    __start_nvm1 = .;
    KEEP(*(nvm1))
    . = ORIGIN(NVM1) + LENGTH(NVM1);
    __stop_nvm1 = .;
  } > NVM1
}
INCLUDE "gcc_arm_flash.ld"
