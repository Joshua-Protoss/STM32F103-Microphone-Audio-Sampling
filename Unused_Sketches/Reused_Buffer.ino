/* Single ADC channel audio recording DMA transfer SPI SD Card
 * 
 * 
 */
// Library Used
#include <libmaple/adc.h>
#include <libmaple/dma.h>
#include <SPI.h>
#include <SdFat.h>

//SD card chip select pin
#define CHIP_SELECT   PA4
#define FILE_NAME     ("RawWrite.txt")
#define LED_Pin       PC13

//ADC trigger timer frequency : should be configurable
// Just use macro 44100 Hz
uint16_t recording_time = 1000; //milliseconds
uint32_t sampling_frequency = 44100; //Hz
uint16_t samples_per_sequence = 1;

//Timer related constant
#define TIMER_PRESCALER      4                                        // to divide the system clock, appropriate for frequencies <1kHz
#define TIMER_FREQUENCY     (72000000/TIMER_PRESCALER)
uint32_t timer_reload_value = (TIMER_FREQUENCY/sampling_frequency)-1; //Used to make the timer frequency equal to sampling frequency


// FIFO Buffer and blocks record if needed
uint8_t sdcard_buffer[64]; // or uint8_t buffer[128] ? 512 or 1024?
uint16_t dma_buffer

// Pin PA0 for analog input for example
#define ADC_IN0 PA0

//DMA and Overrun variables
// set by HW when a complete DMA transfer was finished.
volatile uint8_t dma_irq_full_complete;
// set by HW when a DMA transfer is at its half.
volatile uint8_t dma_irq_half_complete;
// set by SW when overrun occurs
volatile uint8_t overrun;
//volatile uint32_t dma_irq_counter;
volatile uint32_t dma_isr;  // must not be global, make it local later
// signalling for lower and upper buffer status
volatile uint8_t buff0_stored, buff1_stored, buff_index;

void configure_modules(){
  configure_timer();
  configure_adc();
  configure_dma();
  
}

void setup() {

  configure_modules();

  pinMode(ADC_IN0, INPUT_ANALOG);

}

void loop() {

  //sprintf(buff, "ADC: %d\r\n", dma_adc_sample);

}

void Blink(){
  //Blinking function
}

void BlinkError(){
  //Blinking error function
}

//Timer Setup
// TRG0 and stuff
void configure_timer(){
  timer_init(TIMER3);
  // set timer 3 in up-counter mode with auto-reload.
  // as this mode is not supported by the core lib, we have to set up the registers manually.
  (TIMER3->regs).adv->CR1 = 0;
  (TIMER3->regs).adv->CR2 = ( TIMER_CR2_MMS_UPDATE );
  (TIMER3->regs).adv->SMCR = 0;
  (TIMER3->regs).adv->DIER = 0;
  (TIMER3->regs).adv->SR = 0;
  (TIMER3->regs).adv->EGR = 0;      // bit TIMER_EGR_UG can be set to generate event by SW !
  (TIMER3->regs).adv->CCMR1 = 0;
  (TIMER3->regs).adv->CCMR2 = 0;
  (TIMER3->regs).adv->CCER = 0;
  (TIMER3->regs).adv->CNT = 0;  // set it only in down-counting more
  (TIMER3->regs).adv->PSC = TIMER_PRESCALER-1;  
  (TIMER3->regs).adv->ARR = timer_reload_value;
  (TIMER3->regs).adv->CCR1 = 0;
  (TIMER3->regs).adv->CCR2 = 0;
  (TIMER3->regs).adv->CCR3 = 0;
  (TIMER3->regs).adv->CCR4 = 0;
  (TIMER3->regs).adv->DCR = 0;      // don't use DMA
  (TIMER3->regs).adv->DMAR = 0;
  // don't forget to set the ADC trigger source to TIMER3->TRG0. Do it before enabling the ADC !!!
}
//ADC setup
// Single channel single conversion
void configure_adc(){
  //set prescaler

  adc_set_prescaler(ADC_PRE_PCLK2_DIV_2);
  
  //initialize adc1 rcc clock enable
  adc_init(ADC1);
  
  //set sample rate
  adc_set_sample_rate(ADC1, ADC_SMPR_7_5);
  
  //set CR2 register to enable ext trigger and put TRG0 as trigger source
  uint32_t cr2 = ( ADC_CR2_EXTTRIG | ADC_EXT_EV_TIM3_TRGO | ADC_CR2_DMA | ADC_CR2_TSVREFE ); // ADC_CR2_TSEREFE for test only
  ADC1->regs->CR2 = cr2;
  
  //Enable ADC1
  adc_enable(ADC1);
  
  //Calibrate ADC1 (optional)
  adc_calibrate(ADC1);
  
  //set sequence length to be converted in SQR1 register
  adc_set_reg_seqlen(ADC1, samples_per_sequence); //The number of channels to be converted.
  //need to read on scan mode in CR1 register ---> SCAN mode is not needed for single channel

  //void ADC_set_channel_sequences(void)-> SQR3 and SQRL registers
  //  MODIFY_REG(ADC1->SQR3, 0x0000001FU, 0x00); alternative
  ADC1->regs->SQR3 = 0;
}


//DMA Setup
// 1. DMA Init
// 2. DMA RX Irq
// 3. DMA_Setup
// 4. uint8_t check DMA Status

void configure_dma(){
  //DMA init needed?
  //IRQ handler inside this (void DMA_Rx_Irq)
}

void DMA_Init(void){
  
  dma_irq_full_complete = 0;
  dma_irq_half_complete = 0;
  overrun = 0;
  //dma_irq_counter = 0;
  dma_isr = 0;
  buff0_stored = buff1_stored = 1;  // avoid overrun detection
  buff_index = 0;
  dma_clear_isr_bits(DMA1, DMA_CH1);

}

void DMA_Rx_irq(void)
{
// Used to store DMA interrupt status register (ISR) bits. This helps explain what's going on
  dma_isr = dma_get_isr_bits(DMA1, DMA_CH1);
  if (dma_isr&DMA_ISR_HTIF1) {
    dma_irq_half_complete = 1;
    buff0_stored = 0; // reset storage flag to detect overrun
    //if ( buff1_stored==0 )  overrun++;  // upper buffer half being written before was stored
  }
  if (dma_isr&DMA_ISR_TCIF1) {
    dma_irq_full_complete = 1;
    buff1_stored = 0; // reset storage flag to detect overrun
    //if ( buff0_stored==0 )  overrun++;  // lower buffer half being written before was stored
  }
  dma_clear_isr_bits(DMA1, DMA_CH1);
}
