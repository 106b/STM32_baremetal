//program to blink light on STM32F249 board
#include <inttypes.h>

struct gpio {
	volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, 
			 BSRR, LCKR, AFR[2];
};

//modes for MODER
enum {GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG};

#define GPIO(bank) ((struct gpio *) (0x40020000 + 0x400 * (bank)))

//define bank in upper byte, pin number in lower byte
#define PIN(bank, num) (((bank) - 'A') << 8 | (num))

//these next two defines are used to split up the bytes above 
//clears upper byte and keeps pin number to assign to variable
#define PINNO(pin) (pin & 255)
//needed to set bank of pin in GPIO macro 
#define PINBANK(pin) (pin >> 8)


//2 bytes of pin argument encode bank in upper byte and pin # in lower byte
static inline void gpio_set_mode(uint16_t pin, uint8_t mode){
	struct gpio *gpio = GPIO(PINBANK(pin)); //set GPIO bank
	uint8_t n = PINNO(pin);					//get pin number
	gpio->MODER &= ~(3U << (n * 2));		//clear pin number of GPIO bank
	gpio->MODER |= (mode & 3) << (n * 2);	//set mode of GPIO bank at pin #

}

int main(){
	return 0;
}
