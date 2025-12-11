#define SRAM_SIZE		((uint32_t) 0x00005000)
#define SRAM_BASE		((uint32_t) 0x20000000)
#define STACKINIT		((interrupt_t)(SRAM_BASE+SRAM_SIZE))

typedef int			   int32_t;
typedef short		   int16_t;
typedef char		   int8_t;
typedef unsigned int   uint32_t;
typedef unsigned short uint16_t;
typedef unsigned char  uint8_t;
typedef unsigned long long uint64_t;

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
        uint32_t JOFR[4];   // JOFR1..4
        uint32_t HTR;
        uint32_t LTR;
        uint32_t SQR1;
        uint32_t SQR2;
        uint32_t SQR3;
        uint32_t JSQR;
        uint32_t JDR[4];    // JDR1..4
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

#define LED_PATTERN_LENGTH            180U
#define ADC_THRESHOLD_LOW_RAW         1365U  // ~1100mV
#define ADC_THRESHOLD_HIGH_RAW        1870U  // ~1500mV
#define ADC_CHANNEL_B1                9U
#define ADC_SAMPLE_TIME_CYCLES        0b111
#define GPIO_OUTPUT_MODE_PUSH_PULL    0x33333333
#define TIM2_PRESCALER_30KHZ          (2400U - 1U)
#define TIM2_AUTO_RELOAD_MAX_16BIT    0xFFFFU

static volatile uint8_t low_detected = 0;
static volatile uint32_t delta_ticks = 1;
static volatile uint16_t laps = 0;

static const uint16_t led_pattern[180] = {
        0x8080, 0x8080, 0x8080, 0x8080, 0x9FFF, 0x90C0, 0x90C0, 0x90C0,
        0x90C0, 0x88A0, 0x879F, 0x8080, 0x8080, 0x9FEF, 0x8080, 0x8080,
        0x879F, 0x88A0, 0x90C0, 0x91C0, 0x91C0, 0x89C0, 0x87A0, 0x8080,
        0x8080, 0x9FEF, 0x8080, 0x8080, 0x80C0, 0x80C0, 0x80C0, 0x9FFF,
        0x80C0, 0x80C0, 0x80C0, 0x8080, 0x8080, 0x9C80, 0x8381, 0x829E,
        0x82E0, 0x829E, 0x8381, 0x9C80, 0x8080, 0x8080, 0x9FFF, 0x9080,
        0x9080, 0x9080, 0x9080, 0x9080, 0x8080, 0x8080, 0x9FFF, 0x90C2,
        0x90C2, 0x90C0, 0x90C0, 0x90C0, 0x8080, 0x8080, 0x889C, 0x90A2,
        0x90C2, 0x90C1, 0x90C1, 0x90C1, 0x8FA0, 0x8080, 0x8080, 0x8080,
        0x90C0, 0x90C0, 0x9FFF, 0x90C0, 0x90C0, 0x8080, 0x8080, 0x90C0,
        0x90C0, 0x9FFF, 0x90C0, 0x90C0, 0x8080, 0x8080, 0x8080, 0x0000,
        0x0000, 0x0000, 0x8082, 0x8082, 0x8085, 0x8085, 0x8188, 0x8188,
        0x8290, 0x8290, 0x84A0, 0x84A0, 0x98C0, 0x98C0, 0x84A0, 0x84A0,
        0x8290, 0x8290, 0x8188, 0x8188, 0x8085, 0x8085, 0x8082, 0x8082,
        0x0000, 0x0000, 0x8080, 0x8080, 0x8080, 0x9FFF, 0x80A0, 0x8090,
        0x8088, 0x8090, 0x80A0, 0x9FFF, 0x8080, 0x8080, 0x9FEF, 0x8080,
        0x8080, 0x9FFF, 0x80C1, 0x81C1, 0x82C1, 0x84C2, 0x98BC, 0x8080,
        0x8080, 0x9C80, 0x8381, 0x828E, 0x82B8, 0x82C0, 0x82B8, 0x828E,
        0x8381, 0x9C80, 0x8080, 0x8080, 0x9FFF, 0x80C1, 0x81C1, 0x82C1,
        0x84C2, 0x98BC, 0x8080, 0x8080, 0x879F, 0x88A0, 0x90C0, 0x90C0,
        0x90C0, 0x90C0, 0x8080, 0x8080, 0x9FFF, 0x8180, 0x8180, 0x8180,
        0x8180, 0x8180, 0x9FFF, 0x8080, 0x8080, 0x9FEF, 0x8080, 0x8080,
        0x8080, 0x8080, 0x8080, 0x8080
};


void ADC1_2_IRQHandler(void);
int  main(void);

const interrupt_t vector_table[] __attribute__ ((section(".vtab"))) = {
	STACKINIT,												// 0x0000_0000 Stack Pointer
	(interrupt_t) main,										// 0x0000_0004 Reset
	0,														// 0x0000_0008
	0,														// 0x0000_000C
	0,														// 0x0000_0010
	0,														// 0x0000_0014
	0,														// 0x0000_0018
	0,														// 0x0000_001C
	0,														// 0x0000_0020
	0,														// 0x0000_0024
	0,														// 0x0000_0028
	0,														// 0x0000_002C
	0,														// 0x0000_0030
	0,														// 0x0000_0034
	0,														// 0x0000_0038
	0,														// 0x0000_003C SYSTICK
	0,														// 0x0000_0040
	0,														// 0x0000_0044
	0,														// 0x0000_0048
	0,														// 0x0000_004C
	0,														// 0x0000_0050
	0,														// 0x0000_0054
	0,														// 0x0000_0058
	0,														// 0x0000_005C
	0,														// 0x0000_0060
	0,														// 0x0000_0064
	0,														// 0x0000_0068
	0,														// 0x0000_006C
	0,														// 0x0000_0070
	0,														// 0x0000_0074
	0,														// 0x0000_0078
	0,														// 0x0000_007C
	0,														// 0x0000_0080
	0,														// 0x0000_0084
	ADC1_2_IRQHandler,										// 0x0000_0088
	0,														// 0x0000_008C
	0,														// 0x0000_0090
	0,														// 0x0000_0094
	0,														// 0x0000_0098
	0,														// 0x0000_009C
	0,														// 0x0000_00A0
	0,														// 0x0000_00A4
	0,														// 0x0000_00A8
	0,														// 0x0000_00AC
	0,														// 0x0000_00B0 TIM2
	0,														// 0x0000_00B4
	0,														// 0x0000_00B8
	0,														// 0x0000_00BC
	0,														// 0x0000_00C0
	0,														// 0x0000_00C4
	0,														// 0x0000_00C8
	0,														// 0x0000_00CC
	0,														// 0x0000_00D0
	0,														// 0x0000_00D4 USART1
	0,														// 0x0000_00D8
	0,														// 0x0000_00DC
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
        DEVMAP->RCC.REGs.APB1ENR |= (1 << 0);  // TIM2 clock enable
        DEVMAP->RCC.REGs.APB2ENR |= (1 << 2);  // GPIOA clock enable
        DEVMAP->RCC.REGs.APB2ENR |= (1 << 3);  // GPIOB clock enable
        DEVMAP->RCC.REGs.APB2ENR |= (1 << 9);  // ADC1 clock enable
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
        // Configurar PB1 como entrada analógica (ADC1_IN9)
        // ========================================
        DEVMAP->GPIOs[GPIOB].REGs.CRL &= ~((uint32_t)0xF << 4);

        // ========================================
        // Configurar TIM2 como contador libre para time-stamping
        // ========================================
        DEVMAP->TIMs[TIM2].REGs.CR1 = 0;
        DEVMAP->TIMs[TIM2].REGs.PSC = TIM2_PRESCALER_30KHZ;            // 72MHz / 2400 = 30kHz -> 33.33us por tick
        DEVMAP->TIMs[TIM2].REGs.ARR = TIM2_AUTO_RELOAD_MAX_16BIT;      // Conteo libre de 16 bits
        DEVMAP->TIMs[TIM2].REGs.CNT = 0;
        DEVMAP->TIMs[TIM2].REGs.CR1 |= (1 << 0);             // CEN: habilitar contador

        // ========================================
        // Configurar ADC1 para lecturas continuas en canal 9 (PB1)
        // ========================================
        // El ADC1 necesita un reloj máximo de 14MHz, así que dividimos PCLK2 (72MHz) / 8 = 9MHz
        DEVMAP->RCC.REGs.CFGR &= ~(0b11 << 14);
        DEVMAP->RCC.REGs.CFGR |=  (0b11 << 14);             // ADCPRE = 10: PCLK2/8

        // Configurar ADC primero, ANTES de encender
        DEVMAP->ADC[ADC1].REGs.SMPR2 &= ~(0b111 << (3 * ADC_CHANNEL_B1));
        DEVMAP->ADC[ADC1].REGs.SMPR2 |=  (ADC_SAMPLE_TIME_CYCLES << (3 * ADC_CHANNEL_B1)); // Sample time = 239.5 ciclos
        DEVMAP->ADC[ADC1].REGs.SQR1 &= ~(0b1111 << 20);      // Longitud de secuencia = 1
        DEVMAP->ADC[ADC1].REGs.SQR3  = ADC_CHANNEL_B1;       // Canal 9 (PB1) como primera conversión
        DEVMAP->ADC[ADC1].REGs.CR2  |= (1 << 1);             // Modo continuo habilitado
        
        // Ahora encender y calibrar
        DEVMAP->ADC[ADC1].REGs.CR2  |= (1 << 0);             // ADON: Encender ADC (primera vez)
        for(volatile int i=0; i<1000; i++);                  // Delay para estabilización
        
        DEVMAP->ADC[ADC1].REGs.CR2  |= (1 << 3);             // RSTCAL: Reset calibration
        while (DEVMAP->ADC[ADC1].REGs.CR2 & (1 << 3));       // Esperar que se complete reset
        
        DEVMAP->ADC[ADC1].REGs.CR2  |= (1 << 2);             // CAL: Iniciar calibración
        while (DEVMAP->ADC[ADC1].REGs.CR2 & (1 << 2));       // Esperar que se complete calibración
        
        // Arranque determinista de conversión continua por software
        DEVMAP->ADC[ADC1].REGs.CR2 &= ~(0b111 << 17);        // EXTSEL = 111 (SWSTART)
        DEVMAP->ADC[ADC1].REGs.CR2 |=  (0b111 << 17);
        DEVMAP->ADC[ADC1].REGs.CR2 |= (1 << 20);             // EXTTRIG habilitado

        // Segunda escritura a ADON inicia la conversión en modo continuo
        DEVMAP->ADC[ADC1].REGs.CR2  |= (1 << 0);             // ADON: Iniciar conversión continua
        DEVMAP->ADC[ADC1].REGs.CR2 |= (1 << 22);             // SWSTART: disparar conversión
        // ========================================
        // Configurar PA[15:0] como salidas 50MHz push-pull
        // ========================================
		// CRL configura PA[7:0]:   MODE=11 (50MHz), CNF=00 (Push-pull) = 0x3 por pin
		// CRH configura PA[15:8]:  MODE=11 (50MHz), CNF=00 (Push-pull) = 0x3 por pin
        DEVMAP->GPIOs[GPIOA].REGs.CRL = GPIO_OUTPUT_MODE_PUSH_PULL;  // PA[7:0]  como salidas
        DEVMAP->GPIOs[GPIOA].REGs.CRH = GPIO_OUTPUT_MODE_PUSH_PULL;  // PA[15:8] como salidas
	
        // ========================================
        // Bucle principal: encender LEDs en secuencia
        // ========================================
        // Preparar estado inicial del comparador de umbral
        // Descartar la primera muestra tras el arranque para evitar lecturas iniciales incorrectas
        while (!(DEVMAP->ADC[ADC1].REGs.SR & (1 << 1)));
        (void)(DEVMAP->ADC[ADC1].REGs.DR & 0xFFFF);

        // Preparar estado inicial del comparador de umbral con la siguiente conversión válida
        while (!(DEVMAP->ADC[ADC1].REGs.SR & (1 << 1)));
        uint16_t initial_sample = (uint16_t)(DEVMAP->ADC[ADC1].REGs.DR & 0xFFFF);
        low_detected = (initial_sample < ADC_THRESHOLD_LOW_RAW);


        // Habilitar interrupción de fin de conversión
        DEVMAP->ADC[ADC1].REGs.CR1 |= (1 << 5);             // EOCIE
        ENA_IRQ(IRQ_ADC1_2);


        for(;;) {
            DEVMAP->GPIOs[GPIOA].REGs.ODR = led_pattern[((DEVMAP->TIMs[TIM2].REGs.CNT  / delta_ticks)+(laps)) % LED_PATTERN_LENGTH];
        }

        return 0;
}

void ADC1_2_IRQHandler(void)
{
        // Leer DR (esto automáticamente limpia EOC en modo continuo)
    uint16_t sample = (uint16_t)(DEVMAP->ADC[ADC1].REGs.DR & 0xFFFF);

    if (!low_detected && (sample < ADC_THRESHOLD_LOW_RAW)) {
        low_detected = 1;
    }

    if (low_detected && (sample > ADC_THRESHOLD_HIGH_RAW)) {
            uint32_t ticks_one_lap = DEVMAP->TIMs[TIM2].REGs.CNT;
			DEVMAP->TIMs[TIM2].REGs.CNT = 0;
            delta_ticks = ticks_one_lap / LED_PATTERN_LENGTH; // Ventana de 2 grados
            if (delta_ticks <= 0) {
                    delta_ticks = 1;
            }
            low_detected = 0;
			laps = (laps + 1) % 180;
    }
	DEVMAP->ADC[ADC1].REGs.SR &= ~(1 << 1);					// Clear EOC bit
	CLR_IRQ(IRQ_ADC1_2);
}
