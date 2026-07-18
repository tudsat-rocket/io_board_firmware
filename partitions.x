/* Shared partition map for the cancan bootloader and the application.
 *
 * Sized for the STM32F105RC (256K flash, 2K pages). Values are offsets from
 * the flash base (0x08000000); embassy-boot indexes the flash bank by these.
 *
 *   0x00000000  +----------------------+
 *               | BOOTLOADER    24 KB  |
 *   0x00006000  +----------------------+
 *               | STATE          2 KB  |  swap state + power-fail progress
 *   0x00006800  +----------------------+
 *               | ACTIVE       114 KB  |  the running application
 *   0x00023000  +----------------------+
 *               | DFU          116 KB  |  staged update (ACTIVE + 1 scratch page)
 *   0x00040000  +----------------------+
 *
 * DFU is one page (2 KB) larger than ACTIVE, as the swap algorithm requires.
 * Each binary's memory.x FLASH region must agree with these boundaries. */

__bootloader_state_start  = 0x00006000;
__bootloader_state_end    = 0x00006800;

__bootloader_active_start = 0x00006800;
__bootloader_active_end   = 0x00023000;

__bootloader_dfu_start    = 0x00023000;
__bootloader_dfu_end      = 0x00040000;
