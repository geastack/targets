/*
 * STM32F746G-Discovery memory layout.
 *
 *   FLASH  internal, boot + runtime + the engine's per-frame code
 *   QSPI   memory-mapped N25Q128A/W25Q128J, the generated app, the engine's
 *          cold code, fonts and assets
 *   DTCM   stack and DMA buffers (uncached, no cache maintenance needed)
 *   SRAM   .data/.bss
 *   SDRAM  framebuffers, then the malloc heap
 *
 * QSPI and SDRAM are brought up in SystemInit, before .data/.bss and static
 * constructors, so everything below may use them from the first C++ line.
 *
 * Code goes to QSPI by archive: every member of an archive named *_qspi.a
 * (the generated app, the framework's cold code) lands there; `.qspi.*`
 * sections hold data only.
 *
 * QSPI and FLASH are ~2 GB apart, beyond the +-1 GB reach of the prel31
 * offsets in ARM unwind tables. So each region carries its own table
 * (unwind_tables.cpp picks one by program counter), and QSPI archives name a
 * personality thunk that lives in QSPI (qspi_personality.S).
 */

MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 1024K
  DTCM  (rw) : ORIGIN = 0x20000000, LENGTH = 64K
  SRAM  (rw) : ORIGIN = 0x20010000, LENGTH = 256K
  QSPI  (rx) : ORIGIN = 0x90000000, LENGTH = 16M
  SDRAM (rw) : ORIGIN = 0xC0000000, LENGTH = 8M
}

ENTRY(Reset_Handler)

_estack = ORIGIN(DTCM) + LENGTH(DTCM);
__stack_size = 32K;
__framebuffer_bytes = 512K;

SECTIONS
{
  .isr_vector : ALIGN(512)
  {
    KEEP(*(.isr_vector))
  } > FLASH

  /* Claimed before the generic rules below: ld places an input section in the
     first output section whose pattern matches it. */
  .qspi : ALIGN(32)
  {
    *_qspi.a:*(.text .text.* .rodata .rodata.* .gcc_except_table .gcc_except_table.*)
    *gea_embedded_*_generated.cpp.obj(.rodata .rodata.*)
    *(.qspi .qspi.*)
  } > QSPI

  .qspi_extab :
  {
    *_qspi.a:*(.ARM.extab .ARM.extab.* .gnu.linkonce.armextab.*)
  } > QSPI

  .qspi_exidx :
  {
    __qspi_exidx_start = .;
    *_qspi.a:*(.ARM.exidx .ARM.exidx.* .gnu.linkonce.armexidx.*)
    __qspi_exidx_end = .;
  } > QSPI

  .text :
  {
    *(.text .text.*)
    *(.glue_7 .glue_7t .vfp11_veneer .v4_bx)
    KEEP(*(.init))
    KEEP(*(.fini))
  } > FLASH

  .rodata : ALIGN(4)
  {
    *(.rodata .rodata.*)
    *(.gcc_except_table .gcc_except_table.*)
  } > FLASH

  .ARM.extab :
  {
    *(.ARM.extab .ARM.extab.* .gnu.linkonce.armextab.*)
  } > FLASH

  .ARM.exidx :
  {
    __exidx_start = .;
    *(.ARM.exidx .ARM.exidx.* .gnu.linkonce.armexidx.*)
    __exidx_end = .;
  } > FLASH

  .preinit_array (READONLY) :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } > FLASH

  .init_array (READONLY) :
  {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
  } > FLASH

  .fini_array (READONLY) :
  {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } > FLASH

  .data : ALIGN(4)
  {
    _sdata = .;
    *(.data .data.*)
    . = ALIGN(4);
    _edata = .;
  } > SRAM AT > FLASH
  _sidata = LOADADDR(.data);

  /* One thread, one TLS block: the live block is the linked one, set up by
     thread_local_storage.cpp before any constructor runs. ld does not move
     the location counter past .tbss, so .tbss_space reserves its room. */
  .tdata : ALIGN(8)
  {
    __tdata_start = .;
    *(.tdata .tdata.* .gnu.linkonce.td.*)
    __tdata_end = .;
  } > SRAM AT > FLASH
  __tdata_load = LOADADDR(.tdata);

  .tbss (NOLOAD) :
  {
    __tbss_start = .;
    *(.tbss .tbss.* .gnu.linkonce.tb.*)
    *(.tcommon)
    __tbss_end = .;
  } > SRAM

  .tbss_space (NOLOAD) :
  {
    . = ADDR(.tbss) + SIZEOF(.tbss);
  } > SRAM

  .bss (NOLOAD) : ALIGN(4)
  {
    _sbss = .;
    *(.bss .bss.*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > SRAM

  /* Never zeroed: DMA rings and scratch the owning driver initialises. */
  .dtcm (NOLOAD) : ALIGN(32)
  {
    *(.dtcm .dtcm.*)
  } > DTCM

  .sdram (NOLOAD) : ALIGN(32)
  {
    __framebuffer_start = .;
    . += __framebuffer_bytes;
    *(.sdram .sdram.*)
    . = ALIGN(32);
    __heap_start = .;
  } > SDRAM
  __heap_end = ORIGIN(SDRAM) + LENGTH(SDRAM);

  ASSERT(ADDR(.dtcm) + SIZEOF(.dtcm) <= _estack - __stack_size, "DTCM buffers leave less than __stack_size for the stack")
  ASSERT(ALIGNOF(.tdata) <= 8 && ALIGNOF(.tbss) <= 8, "thread_local data aligned past 8 bytes needs a thread pointer offset")
}
