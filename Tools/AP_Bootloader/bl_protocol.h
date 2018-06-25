void jump_to_app(void);
void bootloader(unsigned timeout);

static enum led_state {LED_BLINK, LED_ON, LED_OFF} led_state;
/*
  millisecond timer array
 */
#define NTIMERS		    2
#define TIMER_BL_WAIT	0
#define TIMER_LED	    1

extern volatile unsigned timer[NTIMERS];
#define BL_WAIT_MAGIC	0x19710317		/* magic number in PWR regs to wait in bootloader */

/*****************************************************************************
 * Chip/board functions.
 */

/* LEDs */

#ifdef BOOT_DELAY_ADDRESS
# define BOOT_DELAY_SIGNATURE1	0x92c2ecff
# define BOOT_DELAY_SIGNATURE2	0xc5057d5d
# define BOOT_DELAY_MAX		30
#endif

#define MAX_DES_LENGTH 20

#define arraySize(a) (sizeof((a))/sizeof(((a)[0])))
