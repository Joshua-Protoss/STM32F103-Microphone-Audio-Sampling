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
#define CHIP_SELECT PA4
#define FILE_NAME     ("RawWrite.txt")
#define LED_Pin       PC13

//ADC trigger timer frequency : should be configurable
// Just use macro 44100 Hz
uint16_t recording_time = 1000; //milliseconds
uint32_t sampling_frequency = 44100; //Hz

//Timer related constant
#define TIMER_PRESCALER      4 // to divide the system clock, appropriate for frequencies <1kHz
#define TIMER_FREQUENCY     (72000000/TIMER_PRESCALER)
uint32_t timer_reload_value = (TIMER_FREQUENCY/sampling_frequency)-1;


// FIFO Buffer and blocks record if needed
uint8_t adc_buffer[64]; // or uint8_t buffer[128] ? 512 or 1024?

// Pin PA0 for analog input for example
#define ADC_IN0 PA0


void configre_modules(){
  configure_timer();
  configure_adc();
  configure_dma();
  
}

void setup() {

  configre_modules();

}

void loop() {

  

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
  //initialize adc1 rcc clock enable
  //set sample rate
  //set CR2 register to enable ext trigger and put TRG0 as trigger source
  //Enable ADC1
  //Calibrate ADC1 (optional)
  //set sequence length to be converted in SQR1 register

  //need to read on scan mode in CR1 register

  //void ADC_set_channel_sequences(void)-> SQR3 and SQRL registers
}


//DMA Setup
// 1. DMA Init
// 2. DMA RX Irq
// 3. DMA_Setup
// 4. uint8_t check DMA Status

void configure_dma(){
  //DMA init needed?
  //IRQ handler inside this
}
