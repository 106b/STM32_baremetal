
int main(void){
	return 0; 
}

// startup code
// defines the boot function
__attribute__((naked, noreturn)) void _reset(void){

	// initilize .bss to 0, copy .data from flash to RAM
	// addresses of variables defined in linker.ld
	extern long _sdata, _edata, _sbss, _ebss, _sidata;
	for(long *dst = &_sbss; dst < &_ebss; dst++) *dst = 0;
	for(long *dst = &_sdata, *src = &_sidata; dst < &_edata;) *dst++ = *src++;

	main();
	for(;;) (void) 0; //infinite loop

}

extern void _estack(void); // Defined in link.ld
						   // This is the initial stack pointer 
						   // interrupt handler
						   
// 16 standard and 91 STM32-specific handlers
// this function sets up vector table and defines an area of memory ".vectors"// the first two addresses point to the _estack and _reset interrupt handlers
__attribute__((section(".vectors"))) void (*const tab[16 + 91])(void) = {
	_estack, _reset
};
