/* The bootloader links into the BOOTLOADER region. The A/B partition offsets
 * (state/active/dfu) that embassy-boot uses come from the shared ../partitions.x.
 * FLASH's LENGTH here (24K) defines where ACTIVE begins, so it must match
 * __bootloader_active_start in partitions.x. */
MEMORY
{
  FLASH       : ORIGIN = 0x08000000, LENGTH = 24K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
}

INCLUDE partitions.x;
