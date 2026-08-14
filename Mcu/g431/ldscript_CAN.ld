/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM) + LENGTH(RAM); /* end of "RAM" Ram type memory */

_Min_Heap_Size = 0x200; /* required amount of heap */
_Min_Stack_Size = 0x400; /* required amount of stack */

/* Memories definition */
MEMORY
{
 RAM          (xrw)  : ORIGIN = 0x20000000, LENGTH = 32K
 FLASH1        (rx)  : ORIGIN = 0x08004000, LENGTH = 1024
 FILE_NAME     (rx)  : ORIGIN = ORIGIN(FLASH1)+LENGTH(FLASH1), LENGTH = 32
 EEPROM        (rx)  : ORIGIN = 0x0801f800, LENGTH = 2K
 FLASH         (rx)  : ORIGIN = ORIGIN(FILE_NAME)+LENGTH(FILE_NAME), LENGTH = ORIGIN(EEPROM)-(ORIGIN(FILE_NAME)+LENGTH(FILE_NAME))
}

/* Sections */
SECTIONS
{
  /* The startup code into "FLASH" Rom type memory */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
    KEEP(*(.app_signature))
    . = ALIGN(4);
  } > FLASH1

  /* PX4 SD-card update scans the first 1 KiB for APDesc00 (8-byte aligned). */
  .px4_app_descriptor :
  {
    . = ALIGN(8);
    KEEP(*(.px4_app_descriptor))
    . = ALIGN(4);
  } > FLASH1

  .fill_flash1 :
  {
        FILL(0x00);
        . = ORIGIN(FLASH1) + LENGTH(FLASH1) - 1;
        BYTE(0x00)
  } > FLASH1

  /* The program code and other data into "FLASH" Rom type memory */
  .text :
  {
    FILL(0x00);
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH

  /* Constant data into "FLASH" Rom type memory */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >FLASH

  .ARM.extab   : {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >FLASH

  .ARM : {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } >FLASH

  .preinit_array     :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .init_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .fini_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .flash_fill :
  {
        . = ALIGN(4);
        BYTE(0x00);
  } > FLASH

  /* Bootloader image embedded for the app-side bootloader update (Src/bl_image.S).
     Empty unless the build defines EMBED_BOOTLOADER. Own section so the 16 KiB
     blob stays attributable in the map and the size asserts have symbols.
     Packed after code, not at the end of the FLASH region, so objcopy -O
     binary does not pad tens of KiB of unused flash into the app image. */
  .bl_image :
  {
    . = ALIGN(4);
    _bl_image_start = .;
    KEEP (*(.bl_image))
    . = ALIGN(4);
    _bl_image_end = .;
  } > FLASH

  /* The file name */
  .file_name :
  {
    FILL(0x00);
    . = ALIGN(4);
    KEEP (*(.file_name))
    BYTE(0x00)
  } >FILE_NAME

  .filename_fill :
  {
        BYTE(0x00);
  } > FILE_NAME

  /* Used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections into "RAM" Ram type memory */
  .data :
  {
    FILL(0x00);
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */
    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */

  } >RAM AT> FLASH

  /* Uninitialized data section into "RAM" Ram type memory */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM

  /* Survives NVIC_SystemReset (not zeroed by startup); used for post-reset boot sound */
  .noinit (NOLOAD) :
  {
    . = ALIGN(4);
    *(.noinit)
    *(.noinit*)
    . = ALIGN(4);
  } >RAM

  /* User_heap_stack section, used to check that there is enough "RAM" Ram  type memory left */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /* Remove information from the compiler libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}

/* The embedded bootloader image is written over the bootloader region, which
   runs from the start of flash up to the app at ORIGIN(FLASH1). Size 0 is the
   non-embedding build. A future MEMORY edit that puts the blob inside the
   region it erases, or changes the region size, must fail the link. */
_bl_region_start = 0x08000000;
_bl_region_size  = ORIGIN(FLASH1) - _bl_region_start;

ASSERT(_bl_image_end - _bl_image_start == 0 ||
       _bl_image_end - _bl_image_start == _bl_region_size,
       "embedded bootloader image must be exactly the size of the bootloader region")

ASSERT(_bl_image_start >= ORIGIN(FLASH1),
       "embedded bootloader image must not live inside the region it erases")
