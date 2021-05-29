/*
  =================================================================================
  ADC conversion of single channels in single regular conversion mode.
  =================================================================================
  Each conversion sequence is started by timer 3 update event (up-counting mode).
  The result of each sample (16bits) is stored into the ADC buffer via DMA transfer.
  The recording time and sampling frequency is configurable.
  ---------------------------------------------------------------------------------
*/
#include <libmaple/adc.h>
#include <libmaple/dma.h>
#include <SPI.h>
#include <SdFat.h>

//SD Fat file system
SdFat sd;
SdFile file;
#define error(s) sd.errorHalt(F(s))

// Variables for SD Card initialization
uint8_t * pCache;
uint32_t bgnBlock, endBlock;

// serial output steam
ArduinoOutStream cout(Serial);
// store error strings in flash
#define sdErrorMsg(msg) sd.errorPrint(F(msg));

// global for card size
uint32_t cardSize;
// global for card erase size
uint32_t eraseSize;

/********************************************************************/
// Configuration
/********************************************************************/

// setup recording parameters
uint16_t max_recording_time = 1000;       // in milliseconds, unused for now, enable it for time based recording
uint16_t rate_kb_per_sec    = 66;         //kB/sec
uint32_t sampling_frequency = 44100;      // in Hz

//SD Card related definition and LED_PIN
#define CHIP_SELECT     PB12
#define FILE_NAME       ("RawWrite.txt")
#define LED_PIN         PC13 
#define BLOCK_SIZE      512UL
#define BLOCK_COUNT     (rate_kb_per_sec*max_recording_time)

// Some definitions of data blocks later needed to implement time based recording
#define ADC_BUFFER_SIZE       32
#define SAMPLES_PER_BUFFER    32
//The main buffer to store ADC sample values
uint16_t adc_buffer[ADC_BUFFER_SIZE];

// Configurable timing related constants : adjusted to run TIMER3 at 44100 Hz
#define TIMER_PRESCALER       4
#define TIMER_FREQUENCY       (72000000/TIMER_PRESCALER)
uint32_t timer_reload_value = (TIMER_FREQUENCY/sampling_frequency)-1; // For ARR register

// Setup analog input pin for ADC Channel
#define ADC_IN0 PA0

/*****************************************************************************/


/********************************************************************/
//DMA and Overrun variables
/********************************************************************/
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
/********************************************************************/

/********************************************************************/
// TIMER, ADC, AND DMA Configuration Functions
/********************************************************************/
// Configure all modules
void configure_Modules(void){
  configure_TIMER();
  configure_DMA();
  configure_ADC();
  
}

//TIMER3 Configuration
void configure_TIMER(void){
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
  (TIMER3->regs).adv->PSC = TIMER_PRESCALER-1;      // since PSC value is 4 so the TIMER3 frequency becomes 72 MHz / 4 = 18 MHz
  (TIMER3->regs).adv->ARR = timer_reload_value;     // Hence the clock will be ticking frequency : 18 MHz / ARR = 18000000/(timer_reload_value) = 44100 Hz
  (TIMER3->regs).adv->CCR1 = 0;
  (TIMER3->regs).adv->CCR2 = 0;
  (TIMER3->regs).adv->CCR3 = 0;
  (TIMER3->regs).adv->CCR4 = 0;
  (TIMER3->regs).adv->DCR = 0;      // don't use DMA
  (TIMER3->regs).adv->DMAR = 0;
  // don't forget to set the ADC trigger source to TIMER3->TRG0. Do it before enabling the ADC !!!
}

// DMA Configuration
// Function to count every completed DMA transfers via the ISR bits
void DMA_CNT_var(void)
{
  dma_irq_full_complete = 0;
  dma_irq_half_complete = 0;
  overrun = 0;
  //dma_irq_counter = 0;
  dma_isr = 0;
  buff0_stored = buff1_stored = 1;  // avoid overrun detection
  buff_index = 0;
  dma_clear_isr_bits(DMA1, DMA_CH1);
}

void configure_DMA(void){
  Serial.print("Configuring DMA...");
  DMA_CNT_var();
  
  // DMA tube configuration
dma_tube_config my_tube_cfg = {
  &ADC1->regs->DR,  // data source address
  DMA_SIZE_16BITS,  // source transfer size
  &adc_buffer,    // data destination address 
  DMA_SIZE_16BITS,  // destination transfer size
  SAMPLES_PER_BUFFER, // nr. of data to transfer
  // tube flags: auto increment dest addr, circular buffer, set half/full IRQ, very high prio:
  ( DMA_CFG_DST_INC | DMA_CFG_CIRC | DMA_CFG_HALF_CMPLT_IE | DMA_CFG_CMPLT_IE | DMA_CCR_PL_VERY_HIGH ),
  0,  // unused
  DMA_REQ_SRC_ADC1, // Hardware DMA request source
};
  // configure DMA channel
  int ret = dma_tube_cfg(DMA1, DMA_CH1, &my_tube_cfg);
  if ( ret>0 ) {
    Serial.print(F("DMA configuration error: ")); Serial.println(ret,HEX);
    Serial.print(F("It is not safe to continue!!!"));
    while ( Serial.read()<=0 ); // wait for a key stroke
  }
  // attach an interrupt handler.
  dma_attach_interrupt(DMA1, DMA_CH1, DMA_Rx_irq);
  // Turn on the DMA tube. It will now begin serving requests.
  dma_enable(DMA1, DMA_CH1);

  Serial.println("done.");
}

// DMA interrupt Callback Function (interrupt Handler)
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

//ADC Configuration
void configure_ADC(void){
      Serial.print("Configuring ADC Setup...");
      
      adc_set_prescaler(ADC_PRE_PCLK2_DIV_2);
      adc_init(ADC1); //rcc_clk_enable(ADC1->clk_id), Must be the first adc command!
      adc_set_sample_rate(ADC1, ADC_SMPR_7_5); //up to ADC_SMPR_1_5

      uint32_t cr2 = ( ADC_CR2_EXTTRIG | ADC_EXT_EV_TIM3_TRGO | ADC_CR2_DMA | ADC_CR2_TSVREFE ); // ADC_CR2_TSEREFE for test only
      ADC1->regs->CR2 = cr2;

      adc_enable(ADC1);
      adc_calibrate(ADC1);
      adc_set_reg_seqlen(ADC1, 1);
      ADC1->regs->SQR3 = 0;               //Select channel 0 at SQR3 register
      //ADC1->regs->CR1 = ( ADC_CR1_SCAN ); // b0110: Regular simultaneous mode only, enable SCAN

      Serial.println("done.");
}

/********************************************************************/

/********************************************************************/
// SD Card Initialization Functions
/********************************************************************/
void SD_Init(void){
    if (sd.exists(FILE_NAME)){
        Serial.print("deleting existing raw file...");
        if (!sd.remove(FILE_NAME)){
            error("Can't remove raw file!");
        }
        Serial.println("done.");

        file.close();
    }
    Serial.println("Creating New Contiguous file...");

    // Create a contiguous file
    if(!file.createContiguous(sd.vwd(), FILE_NAME, (BLOCK_SIZE*BLOCK_COUNT))){
        error("createContiguous failed!");
    }

    // Get the location of the file's blocks
    if(!file.contiguousRange(&bgnBlock, &endBlock)){
        error("contiguousRange failed!");
    }

    // Clear the cache and use it as a 512 byte buffer
    pCache = (uint8_t*)sd.vol()->cacheClear();
    //tell card to setup for multiple block write with pre-erase
    if (!sd.card()->writeStart(bgnBlock, BLOCK_COUNT)){
        error("writeStart failed");
    }

    // Fill cache with 64 bytes 0 data
    memset(pCache, 0, BLOCK_SIZE);
}   

/********************************************************************/


/********************************************************************/
// SD Card Info Functions
/********************************************************************/
uint8_t cidDmp() {
  cid_t cid;
  if (!sd.card()->readCID(&cid)) {
    sdErrorMsg("readCID failed");
    return false;
  }
  cout << F("\nManufacturer ID: ");
  cout << hex << int(cid.mid) << dec << endl;
  cout << F("OEM ID: ") << cid.oid[0] << cid.oid[1] << endl;
  cout << F("Product: ");
  for (uint8_t i = 0; i < 5; i++) {
    cout << cid.pnm[i];
  }
  cout << F("\nVersion: ");
  cout << int(cid.prv_n) << '.' << int(cid.prv_m) << endl;
  cout << F("Serial number: ") << hex << cid.psn << dec << endl;
  cout << F("Manufacturing date: ");
  cout << int(cid.mdt_month) << '/';
  cout << (2000 + cid.mdt_year_low + 10 * cid.mdt_year_high) << endl;
  cout << endl;
  return true;
}
uint8_t csdDmp() {
  csd_t csd;
  uint8_t eraseSingleBlock;
  if (!sd.card()->readCSD(&csd)) {
    sdErrorMsg("readCSD failed");
    return false;
  }
  if (csd.v1.csd_ver == 0) {
    eraseSingleBlock = csd.v1.erase_blk_en;
    eraseSize = (csd.v1.sector_size_high << 1) | csd.v1.sector_size_low;
  } else if (csd.v2.csd_ver == 1) {
    eraseSingleBlock = csd.v2.erase_blk_en;
    eraseSize = (csd.v2.sector_size_high << 1) | csd.v2.sector_size_low;
  } else {
    cout << F("csd version error\n");
    return false;
  }
  eraseSize++;
  cout << F("cardSize: ") << 0.000512*cardSize;
  cout << F(" MB (MB = 1,000,000 bytes)\n");

  cout << F("flashEraseSize: ") << int(eraseSize) << F(" blocks\n");
  cout << F("eraseSingleBlock: ");
  if (eraseSingleBlock) {
    cout << F("true\n");
  } else {
    cout << F("false\n");
  }
  return true;
}

void SD_Info(void){
  
  SPI.setModule(2); // remove for the new SD Fat beta

  uint32_t t = millis();
  cout << ("initializing the SD card...");
  // initialize the SD card
  //if ( !sd.begin(CHIP_SELECT, SPISettings(18000000)) ) { // SPI clock value
  if ( !sd.begin(CHIP_SELECT, SPI_CLOCK_DIV2) ) {
    //sd.initErrorHalt("card begin failed!"); // ignore FAT record error
  }
  cout << ("done.") << endl;
  t = millis() - t;
  
  cardSize = sd.card()->cardSize();
  if (cardSize == 0) {
    sd.errorPrint("cardSize failed");
  }
  cout << ("\ninit time: ") << t << " ms" << endl;
  cout << ("\nCard type: ");
  switch (sd.card()->type())
  {
    case SD_CARD_TYPE_SD1:      cout << F("SD1\n"); break;
    case SD_CARD_TYPE_SD2:      cout << F("SD2\n"); break;
    case SD_CARD_TYPE_SDHC:
      if (cardSize < 70000000)  cout << F("SDHC\n");
      else            cout << F("SDXC\n");
      break;
    default:            cout << F("Unknown\n"); break;
  }
  cidDmp();
  csdDmp();
}

/********************************************************************/
void setup() {
  Serial.begin(500000);
  //Serial.begin(57600);
  //Serial.begin(9600);
  
  pinMode(LED_PIN, OUTPUT);
  while ( !Serial.isConnected() ) Blink(500,500);
  
  cout << F("\n**** Zackie Stethoscope single regular mode acquisition ***\n");

  pinMode(ADC_IN0, INPUT_ANALOG);
  
  SD_Info(); //comment if not needed, this takes a lot of SRAM Memory!!!
  
  configure_Modules();
  timer_pause(TIMER3);
  SD_Init();

  Serial.println(F("Type any character to Start."));
  while (!Serial.available()) {
    SysCall::yield();
  }

  // Read any Serial data.
  do {
    delay(10);
  } while (Serial.available() && Serial.read() >= 0);
  
  Serial.print(F("-> sampling started..."));
  Serial.println(F("Type any character to Stop."));

}

void loop() {
  //SD_Init();

  timer_resume(TIMER3);
  
  if (dma_irq_full_complete){
      //pCache = (uint8_t*) adc_buffer;
      if (!sd.card()->writeData((uint8_t*)adc_buffer)){
            error("writeData failed");
      }
      dma_irq_full_complete = 0;
  }

   if (Serial.available()) {
    timer_pause(TIMER3);
    if (!sd.card()->writeStop()){
    Serial.println(F("\nERROR: sd.card->writeStop failed!"));
    error("writeStop failed");
    }  
    // Close file and stop.
    file.close();
    Serial.println(F("Sampling done..."));
    SysCall::halt();
  }
   


}

/********************************************************************/
// Blink Functions
/********************************************************************/
void Blink(int ms_on, int ms_off)
{
  digitalWrite(LED_PIN,0);
  delay(ms_on);
  digitalWrite(LED_PIN,1);
  delay(ms_off);
}

void BlinkError()
{
  for (int i=0;i<5;i++) Blink(100,100);
}

/********************************************************************/
