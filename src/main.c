#define SRAM_SIZE		((uint32_t) 0x00005000)
#define SRAM_BASE		((uint32_t) 0x20000000)
#define STACKINIT		((interrupt_t)(SRAM_BASE+SRAM_SIZE))

typedef int			   int32_t;
typedef short		   int16_t;
typedef char		   int8_t;
typedef unsigned int   uint32_t;
typedef unsigned short uint16_t;
typedef unsigned char  uint8_t;

typedef void(*interrupt_t)(void);

typedef union {
	uint8_t  byte[4];
	uint16_t hword[2];
	uint32_t word;
} word_t;

typedef word_t page[0x400/sizeof(uint32_t)];

// Memory map

enum {TIM2	= 0, TIM3  = 1, TIM4  = 2 };
enum {GPIOA = 0, GPIOB = 1, GPIOC = 2, GPIOD = 3, GPIOE = 4, GPIOF = 5 };
enum {DMA1	= 0 };
enum {CHN1	= 0, CHN2  = 1, CHN3  = 2, CHN4  = 3, CHN5	= 4, CHN6  = 5, CHN7 = 6, CHN8 = 7 };
enum {ADC1	= 0 };
struct {
	union {
		struct {
			uint32_t CR1;
			uint32_t CR2;
			uint32_t SMCR;
			uint32_t DIER;
			uint32_t SR;
			uint32_t EGR;
			uint32_t CCMR1;
			uint32_t CCMR2;
			uint32_t CCER;
			uint32_t CNT;
			uint32_t PSC;
			uint32_t ARR;
			uint32_t RES1;
			uint32_t CCR1;
			uint32_t CCR2;
			uint32_t CCR3;
			uint32_t CCR4;
			uint32_t BDTR;
			uint32_t DCR;
			uint32_t DMAR;
		} REGs;
		page reserved;
	} TIMs[3];

	word_t reserved1[(0x40002800-0x40000c00)/sizeof(word_t)];
	page RTC;
	page WWDG;
	page IWDG;
	word_t reserved2[(0x40003800-0x40003400)/sizeof(word_t)];
	page SPI2;
	word_t reserved3[(0x40004400-0x40003c00)/sizeof(word_t)];
	page USART[2];
	word_t reserved4[(0x40005400-0x40004c00)/sizeof(word_t)];
	page I2C[2];
	page USB;
	page USBCAN_SRAM;
	page bxCAN;
	word_t reserved5[(0x40006c00-0x40006800)/sizeof(word_t)];
	page BKP;
	page PWR;
	word_t reserved6[(0x40010000-0x40007400)/sizeof(word_t)];

	page AFIO;
	page EXTI;

	union {
		struct {
			uint32_t CRL;
			uint32_t CRH;
			uint32_t IDR;
			uint32_t ODR;
			uint32_t BSRR;
			uint32_t BRR;
			uint32_t LCKR;
		} REGs;
		page reserved;
	} GPIOs[5];
	word_t reserved7[(0x40012400-0x40011C00)/sizeof(word_t)];
	union {
		struct {
			uint32_t SR;
			uint32_t CR1;
			uint32_t CR2;
			uint32_t SMPR1;
			uint32_t SMPR2;
			uint32_t JOFR;
			uint32_t HTR;
			uint32_t LTR;
			uint32_t SQR1;
			uint32_t SQR2;
			uint32_t SQR3;
			uint32_t JSQR;
			uint32_t JDR;
			uint32_t DR;
		} REGs;
		page reserved;
	} ADC[2];
	page TIM1;
	page SPI1;
	word_t reserved8[(0x40013800-0x40013400)/sizeof(word_t)];
	union  {
		struct {
			uint32_t SR;
			uint32_t DR;
			uint32_t BRR;
			uint32_t CR1;
			uint32_t CR2;
			uint32_t CR3;
			uint32_t GTPR;
		} REGs;
		page reserved;
	} USART1;
	word_t reserved9[(0x40020000-0x40013C00)/sizeof(word_t)];
	union {
		struct {
			uint32_t ISR;
			uint32_t IFCR;
			struct {
				uint32_t CCR;
				uint32_t CNDTR;
				uint32_t CPAR;
				uint32_t CMAR;
				uint32_t RESERVED;
			} CHN[8];
		} REGs;
		page reserved;
	} DMAs[1];
	word_t reservedA[(0x40021000-0x40020400)/sizeof(word_t)];

	union {
		struct {
			uint32_t CR;
			uint32_t CFGR;
			uint32_t CIR;
			uint32_t APB2RSTR;
			uint32_t APB1RSTR;
			uint32_t AHBENR;
			uint32_t APB2ENR;
			uint32_t APB1ENR;
			uint32_t BDCR;
			uint32_t CSR;
			uint32_t AHBRSTR;
			uint32_t CFGR2;
		} REGs;
		page reserved;
	} RCC;
	word_t reservedB[(0x40022000-0x40021400)/sizeof(word_t)];

	union {
		struct {
			uint32_t ACR;
			uint32_t KEYR;
			uint32_t OPTKEYR;
			uint32_t SR;
			uint32_t CR;
			uint32_t AR;
			uint32_t reserved;
			uint32_t OBR;
			uint32_t WRPR;
		} REGs;
		page reserved;
	} FLASH;
} volatile *const DEVMAP = (void *) 0x40000000;

#define ENA_IRQ(IRQ) {CTX->NVIC.REGs.ISER[((uint32_t)(IRQ) >> 5)] = (1 << ((uint32_t)(IRQ) & 0x1F));}
#define DIS_IRQ(IRQ) {CTX->NVIC.REGs.ICER[((uint32_t)(IRQ) >> 5)] = (1 << ((uint32_t)(IRQ) & 0x1F));}
#define CLR_IRQ(IRQ) {CTX->NVIC.REGs.ICPR[((uint32_t)(IRQ) >> 5)] = (1 << ((uint32_t)(IRQ) & 0x1F));}

struct {
	word_t reversed0[(0xe000e010-0xe0000000)/sizeof(word_t)];
	union {
		struct {
			uint32_t CSR;
			uint32_t RVR;
			uint32_t CVR;
			uint32_t CALIB;
		} REGs;
	} SYSTICK;
	word_t reversed1[(0xe000e100-0xe000e020)/sizeof(word_t)];
	union {
		struct {
			uint32_t ISER[8];
			uint32_t RES0[24];
			uint32_t ICER[8];
			uint32_t RES1[24];
			uint32_t ISPR[8];
			uint32_t RES2[24];
			uint32_t ICPR[8];
			uint32_t RES3[24];
			uint32_t IABR[8];
			uint32_t RES4[56];
			uint8_t  IPR[240];
			uint32_t RES5[644];
			uint32_t STIR;
		} REGs;
	} NVIC;
} volatile *const CTX = ((void *) 0xE0000000);

enum IRQs {
	IRQ_DMA1CHN2  = 12,
	IRQ_ADC1_2	  = 18,
	IRQ_TIM2	  = 28,
	IRQ_USART1	  = 37,
	IRQ_EXTI15_10 = 40,
};

int  main(void);

const interrupt_t vector_table[] __attribute__ ((section(".vtab"))) = {
	STACKINIT,												// 0x0000_0000 Stack Pointer
	(interrupt_t) main,										// 0x0000_0004 Reset
};


int main(void)
{
	// ========================================
	// Configuración de reloj: 72MHz vía PLL
	// ========================================
	DEVMAP->RCC.REGs.CR |= (1 << 16);              // Habilitar HSE (8MHz)
	while (!(DEVMAP->RCC.REGs.CR & (1 << 17)));    // Esperar HSE ready
	
	DEVMAP->RCC.REGs.CR &= ~(1 << 24);             // Deshabilitar PLL
	DEVMAP->RCC.REGs.CFGR |= (0b0111 << 18);       // PLLMUL = 9 (8MHz * 9 = 72MHz)
	DEVMAP->RCC.REGs.CFGR |= (1 << 16);            // PLLSRC = HSE
	DEVMAP->RCC.REGs.CR |= (1 << 24);              // Habilitar PLL
	while (!(DEVMAP->RCC.REGs.CR & (1 << 25)));    // Esperar PLL ready
	
	DEVMAP->FLASH.REGs.ACR |= (0b010 << 0);        // FLASH: 2 wait states
	DEVMAP->RCC.REGs.CFGR |= (0b100 << 8);         // APB1: /2 (36MHz)
	
	DEVMAP->RCC.REGs.CFGR |= (0b10 << 0);          // SW = PLL
	while (!(DEVMAP->RCC.REGs.CFGR & (0b10 << 2))); // Esperar conmutación
	
	// ========================================
	// Habilitar relojes periféricos
	// ========================================
	DEVMAP->RCC.REGs.APB2ENR |= (1 << 2);  // GPIOA clock enable
	DEVMAP->RCC.REGs.APB2ENR |= (1 << 0);  // AFIO clock enable
	
	// ========================================
	// Deshabilitar JTAG para liberar PA15
	// ========================================
	// AFIO->MAPR: SWJ_CFG = 010 (JTAG-DP Disabled, SW-DP Enabled)
	// Esto libera PA15, PB3, PB4 pero mantiene PA13 (SWDIO) y PA14 (SWCLK)
	uint32_t *AFIO_MAPR = (uint32_t*)(0x40010000 + 0x04);  // AFIO base + offset MAPR
	*AFIO_MAPR &= ~(0b111 << 24);  // Limpiar bits SWJ_CFG[26:24]
	*AFIO_MAPR |=  (0b010 << 24);  // SWJ_CFG = 010 (Deshabilitar JTAG, mantener SWD)
	
	// ========================================
	// Configurar PA[15:0] como salidas 50MHz push-pull
	// ========================================
	// CRL configura PA[7:0]:   MODE=11 (50MHz), CNF=00 (Push-pull) = 0x3 por pin
	// CRH configura PA[15:8]:  MODE=11 (50MHz), CNF=00 (Push-pull) = 0x3 por pin
	DEVMAP->GPIOs[GPIOA].REGs.CRL = 0x33333333;  // PA[7:0]  como salidas
	DEVMAP->GPIOs[GPIOA].REGs.CRH = 0x33333333;  // PA[15:8] como salidas
	
	// ========================================
	// Bucle principal: encender LEDs en secuencia
	// ========================================
	// LEDs en PA0-PA12 y PA15 (14 LEDs totales)
	const uint16_t led_pins[14] = {
		7, 6, 5, 4, 3, 2, 1, 0, 8, 9, 10, 11, 12, 15
	};
	
	uint8_t led_index = 0;
	
	for(;;) {
		// Apagar todos los LEDs
		DEVMAP->GPIOs[GPIOA].REGs.ODR = 0x0000;
		
		// Encender LED actual
		DEVMAP->GPIOs[GPIOA].REGs.ODR |= (1 << led_pins[led_index]);
		
		// Delay simple (~100ms a 72MHz)
		for (volatile uint32_t i = 0; i < 720000; i++);
		
		// Avanzar al siguiente LED (cíclico)
		led_index++;
		if (led_index >= 14) {
			led_index = 0;
		}
	}
	
	return 0;
}
