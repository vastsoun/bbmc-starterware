/*
 * ld script for SitaraWare
 */

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)

SECTIONS
{
	. = BOOT_START_ADDR;
	. = ALIGN(4);

        .rsthand       :
        {
               *bl_init.o         (.text)
        }
 
	. = ALIGN(4);
	.text      :
	{
		
		*(.text)
	}

	. = ALIGN(4);

	.data : 
	{ 
		*(.data) 
	}
	
	. = ALIGN(4);

	_bss_start = .;
	.bss : 
	{ 
		
		*(.bss) 
	}
        . = ALIGN(4);

	_bss_end = .;	
	
	.stack ALIGN(256) :
	{
		. += 0x800;
	} 
	_stack = .;
}



	
