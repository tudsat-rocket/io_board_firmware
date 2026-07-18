/* The application links into the ACTIVE partition. FLASH's ORIGIN/LENGTH here
 * must match __bootloader_active_start/_end in the shared partitions.x; the
 * STATE and DFU offsets that FirmwareUpdater uses come from that file. */

MEMORY
{
  FLASH       : ORIGIN = 0x08006800, LENGTH = 114K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
}

INCLUDE partitions.x;
