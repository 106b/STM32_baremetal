// Blinking LED using Systick interrupt
#include <inttype.h>

#define BIT(x) (1UL << (x))

struct systick 
{
	volatile uint32_t CTRL, LOAD, VAL, CALIB; 
};
#define SYSTICK ((struct systick *) 0xe000e010)

struct rcc 
{
  volatile uint32_t CR, PLLCFGR, CFGR, CIR, AHB1RSTR, AHB2RSTR, AHB3RSTR,
      RESERVED0, APB1RSTR, APB2RSTR, RESERVED1[2], AHB1ENR, AHB2ENR, AHB3ENR,
      RESERVED2, APB1ENR, APB2ENR, RESERVED3[2], AHB1LPENR, AHB2LPENR,
      AHB3LPENR, RESERVED4, APB1LPENR, APB2LPENR, RESERVED5[2], BDCR, CSR,
      RESERVED6[2], SSCGR, PLLI2SCFGR;
};
#define RCC ((struct rcc *) 0x40023800)

static inline void systick_init(unit32_t ticks)
{
	if((ticks - 1) > 0xffffff) return; // Error check. Systick is 24 bits max
	SYSTICK->LOAD = ticks - 1;	// Load counter value
	SYSTICK->VAL = 0;			// Init current val
	SYSTICK->CTRL = BIT(0) | BIT(1) | BIT(2); // Enable Systick
	RCC->APB2ENR |= BIT(14); // Start clocking Systick
}

static volatile uint32_t s_ticks;
void sysTick_Handler(void)
{
	s_ticks++; // Increase counter every tick
}

// t: expiration time, prd: period, now: current time. Return true if expired
bool timer_expired(uint32_t *t, uint32_t prd, uint32_t now) {
  if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
  if (*t == 0) *t = now + prd;                   // First poll? Set expiration
  if (*t > now) return false;                    // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
  return true;                                   // Expired, return true
}


