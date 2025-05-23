// Blinking LED with Systick interrupt

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>	

#define BIT(x) (1UL << (x)) // convenience macro to set bit at pos x
#define FREQ 16000000 // CPU frequency, 16Mhz

static inline void spin(volatile uint32_t count) {
  while (count--) (void) 0;
}

///// RCC /////
struct rcc {
  volatile uint32_t CR, PLLCFGR, CFGR, CIR, AHB1RSTR, AHB2RSTR, AHB3RSTR,
      RESERVED0, APB1RSTR, APB2RSTR, RESERVED1[2], AHB1ENR, AHB2ENR, AHB3ENR,
      RESERVED2, APB1ENR, APB2ENR, RESERVED3[2], AHB1LPENR, AHB2LPENR,
      AHB3LPENR, RESERVED4, APB1LPENR, APB2LPENR, RESERVED5[2], BDCR, CSR,
      RESERVED6[2], SSCGR, PLLI2SCFGR;
};
#define RCC ((struct rcc *) 0x40023800)
/////////////////////////////////////////////////////////////////////////////////

///// GPIO /////

// Macros to access GPIO pins and GPIO banks
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
	int n = PINNO(pin);						// get pin number
	RCC->AHB1ENR |= BIT(PINBANK(pin));		// start clocking GPIO bank
	gpio->MODER &= ~(3U << (n * 2));		// clear exisiting bits
	gpio->MODER |= (mode & 3) << (n * 2);	// set bits according to mode
}

static inline void gpio_set_af(uint16_t pin, uint8_t af_num)
{
	struct gpio *gpio = GPIO(PINBANK(pin)); // set bank
	int n = PINNO(pin);						// get pin number
	gpio->AFR[n >> 3] &= ~(15UL << ((n & 7) * 4)); // clear 4 bits at pin number
												   // pin is located either in AFRL = AFR[0] or AFRH = AFR[1], 8 pins per register
												   // each pin has 16 possible alternate functions, so 4 bits per pin
	gpio->AFR[n >> 3] |= ((uint32_t) af_num) << ((n & 7) * 4);
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
/////////////////////////////////////////////////////////////////////////////////

///// Systick /////
struct systick
{
	volatile uint32_t CTRL, LOAD, VAL, CALIB;
};
#define SYSTICK ((struct systick *) 0xe000e010)

static inline void systick_init(uint32_t ticks)
{
	if ((ticks - 1) > 0xffffff) return; // Error Check. Systick is 24-bit value
	SYSTICK->LOAD = ticks - 1; // Set ticks value
	SYSTICK->VAL = 0; // Init val
	SYSTICK->CTRL = BIT(0) | BIT(1) | BIT(2); // Enable Systick using CTRL reg
	RCC->APB2ENR |= BIT(14); // Start clocking Systick (Enable SYSCFG)
}

// Interrupt Handler for Systick
static volatile uint32_t s_ticks; // Volatile is important
void SysTick_Handler(void)
{
	s_ticks++;
}

// Create Arbitrary Periodic Timer
// t: expiration time, prd: period, now: current time. Return true if expired
bool timer_expired(uint32_t *t, uint32_t prd, uint32_t now) 
{
  if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
  if (*t == 0) *t = now + prd;                   // First poll? Set expiration
  if (*t > now) return false;                    // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
  return true;                                   // Expired, return true
}
/////////////////////////////////////////////////////////////////////////////////

///// UART /////
struct uart {
  volatile uint32_t SR, DR, BRR, CR1, CR2, CR3, GTPR;
};
#define UART1 ((struct uart *) 0x40011000)
#define UART2 ((struct uart *) 0x40004400)
#define UART3 ((struct uart *) 0x40004800)

static inline void uart_init(struct uart *uart, unsigned long baud)
{
	uint8_t af = 7; // Alternate Function for UART1/2/3, specified in datasheet
	uint16_t rx = 0, tx = 0;

	if (uart == UART1) RCC->APB2ENR |= BIT(4);
  	if (uart == UART2) RCC->APB1ENR |= BIT(17);
  	if (uart == UART3) RCC->APB1ENR |= BIT(18);

  	if (uart == UART1) tx = PIN('A', 9), rx = PIN('A', 10);
  	if (uart == UART2) tx = PIN('A', 2), rx = PIN('A', 3);
  	if (uart == UART3) tx = PIN('D', 8), rx = PIN('D', 9);

	gpio_set_mode(tx, GPIO_MODE_AF);
	gpio_set_af(tx, af);
	gpio_set_mode(rx, GPIO_MODE_AF);
	gpio_set_af(rx, af);
	uart->CR1 = 0;					// disable this uart
	uart->BRR = FREQ / baud;			//
	uart->CR1 |= BIT(13) | BIT(2) | BIT(3); // set UE, RE, TE
										   // enable uart, rx, tx
}

static inline int uart_read_ready(struct uart *uart) {
  return uart->SR & BIT(5);  // If RXNE bit is set, data is ready
}

static inline uint8_t uart_read_byte(struct uart *uart) {
  return (uint8_t) (uart->DR & 255);
}

static inline void uart_write_byte(struct uart *uart, uint8_t byte) {
  uart->DR = byte;
  while ((uart->SR & BIT(7)) == 0) spin(1);
}

static inline void uart_write_buf(struct uart *uart, char *buf, size_t len) {
  while (len-- > 0) uart_write_byte(uart, *(uint8_t *) buf++);
}

/////////////////////////////////////////////////////////////////////////////////
int main(void){
	uint16_t led = PIN('B', 7);
	gpio_set_mode(led, GPIO_MODE_OUTPUT);
	uart_init(UART3, 115200);
	systick_init(16000000 / 1000); // Init Systick to start ticking every 1ms
	uint32_t timer, period = 100; // Declare timer and 500ms period
	for (;;)
	{
		if (timer_expired(&timer, period, s_ticks))
		{
			static bool on;		// This block is executed
			gpio_write(led, on);// every 'period' milliseconds
			on = !on;			// Toggle LED state
			uart_write_buf(UART3, "hi\r\n", 4);
		}

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
    _estack, _reset, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, SysTick_Handler};
