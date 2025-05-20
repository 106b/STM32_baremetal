#include <inttypes.h>
#include <stdbool.h>

#define BIT(x) (1UL << (x)) // convenience macro to set bit at pos x
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num)) // set bank in upper byte, pin in lower byte 
#define PINNO(pin) (pin & 255) // isolate pin number
#define PINBANK(pin) (pin >> 8) // isolate bank

struct gpio // define gpio registers 
{ 
	volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
};
#define GPIO(bank) ((struct gpio *) (0x40020000 + 0x400 * (bank))) 

enum // define MODE register types
{ 
	GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG
};

static inline void gpio_set_mode(uint16_t pin, uint8_t mode)
{
	struct gpio *gpio = GPIO(PINBANK(pin)); // set bank
	int n = PINNO(pin);					// get pin number
	gpio->MODER &= ~(3U << (n * 2));		// clear exisiting bits
	gpio->MODER |= (mode & 3) << (n * 2);	// set bits according to mode
}

static inline void gpio_write(uint16_t pin, bool val)
{
	// set pin in bank high or low based on current val
	// val = 1 sets pin high
	// val = 0 resets pin
	// set/reset functionality are 16 bits away from eachother for each pin
	struct gpio *gpio = GPIO(PINBANK(pin)); // set address of specified bank
	gpio->BSRR = (1U << PINNO(pin)) << (val ? 0 : 16); 
}

// delay function
static inline void spin(volatile uint32_t count)
{
	while(count--) (void) 0;
}


struct rcc {
  volatile uint32_t CR, PLLCFGR, CFGR, CIR, AHB1RSTR, AHB2RSTR, AHB3RSTR,
      RESERVED0, APB1RSTR, APB2RSTR, RESERVED1[2], AHB1ENR, AHB2ENR, AHB3ENR,
      RESERVED2, APB1ENR, APB2ENR, RESERVED3[2], AHB1LPENR, AHB2LPENR,
      AHB3LPENR, RESERVED4, APB1LPENR, APB2LPENR, RESERVED5[2], BDCR, CSR,
      RESERVED6[2], SSCGR, PLLI2SCFGR;
};
#define RCC ((struct rcc *) 0x40023800)

int main(void){
	uint16_t led = PIN('B', 0);
	RCC->AHB1ENR |= BIT(PINBANK(led));
	gpio_set_mode(led, GPIO_MODE_OUTPUT);
	for (;;)
	{
		gpio_write(led, true);
		spin(999999);
		gpio_write(led, false);
		spin(999999);
	}
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
