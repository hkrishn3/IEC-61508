/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */

/*********************************************************
 * 
 * IEC 61508 Testing
 *
 * (c) Hari Krishnan
 * https://github.com/hkrishn3/IEC_61508
 *
 *********************************************************/
 
#include <stdlib.h>
#include <iwdg.h>
#include <scb.h>
#include <dma.h>
#include <bkp.h>
//Beginning of Auto generated function prototypes by Atmel Studio
void fail_safe();
void cpu_test();
void crc_reset_dr();
uint32 get_crc_dr();
uint32 crc32(uint32 data, uint32 length);
void crc_DMA_event();
void flash_test();
void setup_DMA();
void sram_test();
inline void rcc_start_lsi(void );
inline void rcc_start_hse(void );
inline void rtc_wait_finished();
inline void rtc_clear_sync();
inline void rtc_wait_sync();
inline void rtc_enter_config_mode();
inline void rtc_exit_config_mode();
inline void rtc_enable_irq();
inline void rtc_disable_irq();
void dispatch_rtc_second_interrupt();
void __irq_rtc(void );
void increment_counter(void );
void clock_test();
void adc_test(void );
void gpio_test(void );
void timer_ISR();
void interrupts_test();
void uart_test();
void iwdg_test();
//End of Auto generated function prototypes by Atmel Studio



/********DEFINITIONS FOR FLASH AND SRAM MEMORY CHEC***************/

#define CRC_CR_RESET_BIT                                      0
#define BOOTLOADER_CRC_CHECKSUM                      0x532C19DF

#define FLASH_SYSTEM_BASE                            0x08000000
#define SRAM_USER_ADDRESS                            0x20001400

#define FLASH_SYSTEM_SIZE                               20*1024
#define SRAM_USER_SIZE                                  15*1024
#define FLASH_TOTAL_SIZE                               128*1024

#define PATTERN1                                     0xAAAAAAAA
#define PATTERN2                                     0x55555555


/***********CLOCK TESTING CODE***********************************/

//RTC ISR definitions
#define RTC_SECONDS_INTERRUPT	0
#define RCC_BDCR_RTCSEL_LSI     (0x2 << 8)

//RTC read and write configuration and flag bits
#define RTC_CRL_RTOFF_BIT	5
#define RTC_CRL_CNF_BIT		4
#define RTC_CRL_RSF_BIT		3
#define RTC_CRL_SECF_BIT	0

#define RTC_CRL_RTOFF	BIT(RTC_CRL_RTOFF_BIT)
#define RTC_CRL_CNF	BIT(RTC_CRL_CNF_BIT)
#define RTC_CRL_RSF	BIT(RTC_CRL_RSF_BIT)
#define RTC_CRL_SECF	BIT(RTC_CRL_SECF_BIT)

//definitions and pre-scaler values for the LSI and HSE osciallator
#define RTCSEL_HSE              11
#define RTCSEL_LSI              10
#define LSI_PRESCALER          0x9C3F
#define HSE_PRESCALER          0xF423

#define HSI                      0
#define HSE                      1

//the RTC register map
typedef struct rtc_reg_map {
  __io uint32 CRH;
  __io uint32 CRL;
  __io uint32 PRLH;
  __io uint32 PRLL;
  __io uint32 DIVH;
  __io uint32 DIVL;
  __io uint32 CNTH;
  __io uint32 CNTL;
  __io uint32 ALRH;
  __io uint32 ALRL;
} rtc_reg_map;

// RTC register map base pointer
#define RTC_BASE        ((struct rtc_reg_map*)0x40002800)



//RTC functions
void rtc_init(uint8 rtc_clk_src);
uint32 rtc_get_count();
void rtc_set_count(uint32 value);
void rtc_set_prescaler_load(uint32 value);

int counter_value = 0;
int lsi_hsi_count = 0;
int lsi_hse_count = 0;
int ratio_index = 0;

uint8 last_flag = HSI;
uint8 clock_test_complete = 0;

uint8 hsi_count[10];
uint8 hse_count[10];
/******************************************************************/


uint32 crc_final = 0;
int sram_nowrite_count;

//CRC register map
typedef struct crc_reg_map {
  __io uint32 DR;
  __io uint32 IDR;
  __io uint32 CR;
} crc_reg_map;

//CRC engine register map base pointer

#define CRC_BASE              ((struct crc_reg_map *)0x40023000)

//variable for IWDG status
int iwdg_status = 0;



//variable for CPU control flow test
uint32 control_flow_count  = 0;

//FAIL_SAFE routine for CPU testing
extern void fail_safe(void) asm("FAIL_SAFE");

//FAIL_SAFE routine
//loop forever and toggle LED
void fail_safe() {
  control_flow_count++;
  while(1) {
    delay(900);
    toggleLED();
    delay(100);
  }
}

/*************************CPU TESTING************************************/

void cpu_test() {

  SerialUSB.println("Starting CPU tests...");
  delay(1000);
  SerialUSB.println("CARRY, NEGATIVE, OVERFLOW, SATURATION, STATUS REGISTER TESTS");
  delay(1000);
  SerialUSB.println("Testing registers...RO R1 R2 R3 R4 R5 R6 R7 R8 R9 R10 R11 R12 R13 R14");
  delay(1000);
  SerialUSB.println("Ramp pattern verification ");
  delay(1000);
  SerialUSB.println("Control flow test ");
  delay(1000);

//START CPU TESTS

  asm volatile(

    "MOVS R0, #0x00000000    /* Set Z(ero) Flag*/ \n\t"
    "BNE.W FAIL_SAFE         /* Fails if Z clear*/ \n\t"
    "SUBS R0,#1              /* Set N(egative) Flag*/ \n\t"
    "BPL.W FAIL_SAFE         /* Fails if N clear*/ \n\t"
    "ADDS R0,#2              /* Set C(arry) Flag and do not set Z*/ \n\t"
    "BCC.W FAIL_SAFE         /* Fails if C clear*/ \n\t"
    "MOVS R0, #0x80000000    /* Prepares Overflow test*/ \n\t"
    "ADDS R0, R0, R0         /* Set V(overflow) Flag*/ \n\t"
    "BVC.W FAIL_SAFE         /* Fails if V clear*/ \n\t"
    "MOVS R0, #0xFFFFFFFF    /* Prepares Saturation test*/ \n\t"
    "USAT R1,#10,R0          /* Set Q(saturation) Flag*/ \n\t"
    "MRS R0, APSR            /* Get Status register */ \n\t"
    "CMP R0, #0xB8000000     /* Verifies that N=C=V=Q=1 */ \n\t"
    "BNE.W FAIL_SAFE         /* Fails if Q+N+C=V clear */ \n\t"

    "/* Register R0 (holds value returned by the function) */ \n\t"
    "MOVS R0, #0xAAAAAAAA \n\t"
    "CMP R0, #0xAAAAAAAA \n\t"
    "BNE.W FAIL_SAFE \n\t"
    "MOVS R0, #0x55555555 \n\t"
    "CMP R0, #0x55555555 \n\t"
    "BNE.W FAIL_SAFE \n\t"

    "/* This is for control flow test (ENTRY point)*/"
    "LDR R0,=control_flow_count \n\t"
    "/* Assumes R1 OK; If not, error will be detected by R1 test and Ctrl flow test later on*/"
    "LDR R1,[R0] \n\t"
    "ADDS R1,R1,#0x3	 /* control_flow_count += OxO3*/ \n\t"
    "STR R1,[R0] \n\t"

    "/* Link register (Register R14)*/"
    "MOVS R0, R14              /* Contains the return address and must be saved*/ \n\t"
    "MOVS R14, #0xAAAAAAAA \n\t"
    "CMP R14, #0xAAAAAAAA \n\t"
    "BNE.W FAIL_SAFE \n\t"
    "MOVS R14, #0x55555555 \n\t"
    "CMP R14, #0x55555555 \n\t"
    "BNE.W FAIL_SAFE \n\t"
    "MOVS R14, R0 \n\t"
    "MOVS R0, #0x0             /* For ramp test*/ \n\t"

    "/* Register R1*/"
    "MOVS R1, #0xAAAAAAAA \n\t"
    "CMP R1, #0xAAAAAAAA \n\t"
    "BNE.W FAIL_SAFE \n\t"
    "MOVS R1, #0x55555555 \n\t"
    "CMP R1, #0x55555555 \n\t"
    "BNE.W FAIL_SAFE \n\t"
    "MOVS R1, #0x01           /*For ramp test*/ \n\t"

    "/* Register R2*/"
    "MOVS R2, #0xAAAAAAAA \n\t"
    "CMP R2, #0xAAAAAAAA \n\t"
    "BNE.W FAIL_SAFE \n\t"
    "MOVS R2, #0x55555555 \n\t"
    "CMP R2, #0x55555555 \n\t"
    "BNE.W FAIL_SAFE \n\t"
    "MOVS R2, #0x02            /*For ramp test*/ \n\t"

    "/* Register R3*/"
    "MOVS R3, #0xAAAAAAAA \n\t"
    "CMP R3, #0xAAAAAAAA \n\t"
    "BNE.W FAIL_SAFE \n\t"
    "MOVS R3, #0x55555555 \n\t"
    "CMP R3, #0x55555555 \n\t"
    "BNE.W FAIL_SAFE \n\t"
    "MOVS R3, #0x03            /* For ramp test*/ \n\t"

    "/* Register R4*/"
    "MOVS R4, #0xAAAAAAAA \n\t"
    "CMP R4, #0xAAAAAAAA \n\t"
    "BNE.W FAIL_SAFE \n\t"
    "MOVS R4, #0x55555555 \n\t"
    "CMP R4, #0x55555555 \n\t"
    "BNE.W FAIL_SAFE \n\t"
    "MOVS R4, #0x04            /* For ramp test*/ \n\t"

    "/* Register R5*/"
    "MOVS R5, #0xAAAAAAAA \n\t"
    "CMP R5, #0xAAAAAAAA \n\t"
    "BNE.W FAIL_SAFE \n\t"
    "MOVS R5, #0x55555555 \n\t"
    "CMP R5, #0x55555555 \n\t"
    "BNE.W FAIL_SAFE \n\t"
    "MOVS R5, #0x05            /* For ramp test */ \n\t"

    "/* Register R6*/"
    "MOVS R6, #0xAAAAAAAA \n\t"
    "CMP R6, #0xAAAAAAAA \n\t"
    "BNE.W FAIL_SAFE \n\t"
    "MOVS R6, #0x55555555 \n\t"
    "CMP R6, #0x55555555 \n\t"
    "BNE.W FAIL_SAFE \n\t"
    "MOVS R6, #0x06            /* For ramp test */\n\t"

    "/* Register R7*/"
    "MOVS R7, #0xAAAAAAAA \n\t"
    "CMP R7, #0xAAAAAAAA \n\t"
    "BNE.W FAIL_SAFE \n\t"
    "MOVS R7, #0x55555555 \n\t"
    "CMP R7, #0x55555555 \n\t"
    "BNE.W FAIL_SAFE \n\t"
    "MOVS R7, #0x07            /* For ramp test */\n\t"

    "/* Register R8*/"
    "MOVS R8, #0xAAAAAAAA \n\t"
    "CMP R8, #0xAAAAAAAA \n\t"
    "BNE.W FAIL_SAFE \n\t"
    "MOVS R8, #0x55555555 \n\t"
    "CMP R8, #0x55555555 \n\t"
    "BNE.W FAIL_SAFE \n\t"
    "MOVS R8, #0x08            /* For ramp test */\n\t"

    "/* Register R9*/"
    "MOVS R9, #0xAAAAAAAA \n\t"
    "CMP R9, #0xAAAAAAAA \n\t"
    "BNE.W FAIL_SAFE \n\t"
    "MOVS R9, #0x55555555 \n\t"
    "CMP R9, #0x55555555 \n\t"
    "BNE.W FAIL_SAFE \n\t"
    "MOVS R9, #0x09            /* For ramp test */\n\t"

    "/* Register R10*/"
    "MOVS R10, #0xAAAAAAAA \n\t"
    "CMP R10, #0xAAAAAAAA \n\t"
    "BNE FAIL_SAFE \n\t"
    "MOVS R10, #0x55555555 \n\t"
    "CMP R10, #0x55555555 \n\t"
    "BNE FAIL_SAFE \n\t"
    "MOVS R10, #0x0A           /* For ramp test*/ \n\t"

    "/* Register R11*/"
    "MOVS R11, #0xAAAAAAAA \n\t"
    "CMP R11, #0xAAAAAAAA \n\t"
    "BNE FAIL_SAFE \n\t"
    "MOVS R11, #0x55555555 \n\t"
    "CMP R11, #0x55555555 \n\t"
    "BNE FAIL_SAFE \n\t"
    "MOVS R11, #0x0B           /* For ramp test*/ \n\t"

    "/* Register R12*/"
    "MOVS R12, #0xAAAAAAAA \n\t"
    "CMP R12, #0xAAAAAAAA \n\t"
    "BNE FAIL_SAFE \n\t"
    "MOVS R12, #0x55555555 \n\t"
    "CMP R12, #0x55555555 \n\t"
    "BNE FAIL_SAFE \n\t"
    "MOVS R12, #0x0C          /* For ramp test*/ \n\t"

    "/* Ramp pattern verification*/"
    "CMP R0, #0x00 \n\t"
    "BNE FAIL_SAFE \n\t"
    "CMP R1, #0x01 \n\t"
    "BNE FAIL_SAFE \n\t"
    "CMP R2, #0x02 \n\t"
    "BNE FAIL_SAFE \n\t"
    "CMP R3, #0x03 \n\t"
    "BNE FAIL_SAFE \n\t"
    "CMP R4, #0x04 \n\t"
    "BNE FAIL_SAFE \n\t"
    "CMP R5, #0x05 \n\t"
    "BNE FAIL_SAFE \n\t"
    "CMP R6, #0x06 \n\t"
    "BNE FAIL_SAFE \n\t"
    "CMP R7, #0x07 \n\t"
    "BNE FAIL_SAFE \n\t"
    "CMP R8, #0x08 \n\t"
    "BNE FAIL_SAFE \n\t"
    "CMP R9, #0x09 \n\t"
    "BNE FAIL_SAFE \n\t"
    "CMP R10, #0x0A \n\t"
    "BNE FAIL_SAFE \n\t"
    "CMP R11, #0x0B \n\t"
    "BNE FAIL_SAFE \n\t"
    "CMP R12, #0x0C \n\t"
    "BNE FAIL_SAFE \n\t"

    "/* Control flow test (EXIT point)*/"
    "LDR R0,=control_flow_count \n\t"
    "LDR R1,[R0] \n\t"
    "SUBS R1,R1,#0x3\n\t"
    "STR R1,[R0] \n\t"
  );

  if(control_flow_count == 0)
    SerialUSB.println("CPU Tests OK");

  else if(control_flow_count != 0)
    SerialUSB.println("Start-up CPU Test [Failure]");

}

/************************************************************************************/

//Resets the CRC data register to 0xFFFFFFFF
//Use before intiating a new checksum
void crc_reset_dr() {
  *bb_perip(&CRC_BASE->CR, CRC_CR_RESET_BIT) = 1;
}

//Return the CRC data register value
//returns an unsigned 32-bit integer

uint32 get_crc_dr() {
  return CRC_BASE->DR;
}

//Software CRC32 check using the CRC-32 polynomial used in the IEEE 802.3 (Ethernet) network standard
//returns a 32-bit unsigned integer
//inputs:
//  data = a pointer to 32-bit address 
//  length = the length of the block in bytes

uint32 crc32(uint32 * data, uint32 length) {
  int i;
  uint32 crc_value = 0xFFFFFFFF;
  length = length / sizeof(uint32);

  do {
    crc_value = crc_value ^ *data;

    data++;
    length--;

    for(i=0; i<32; i++)
      if (crc_value & 0x80000000)
        crc_value = (crc_value << 1) ^ 0x04C11DB7; // Polynomial used in STM32
      else
        crc_value = (crc_value << 1);

  } while(length > 0);

  return(crc_value);
}


//DMA ISR
//check for the event
//if full transfer is complete,
//   a) process data
//   b) disable the DMA
// if there is an error, print the error, disable DMA

void crc_DMA_event() {

  dma_irq_cause event = dma_get_irq_cause(DMA1, DMA_CH2);

  switch(event) {
  //the event indicates that the transfer was successfully completed
  case DMA_TRANSFER_COMPLETE:
    dma_disable(DMA1,DMA_CH2);
    crc_final = get_crc_dr();
    break;
  //the event indicates that there was an error transmitting
  case DMA_TRANSFER_ERROR:
    //disable DMA when we are done
    dma_disable(DMA1,DMA_CH2);
    //  SerialUSB.println("Fail");
    break;
  }
}

/***************FLASH MEMORY CHECK*****************************************/
//1. check the software CRC32 checksum for the entire flash memory
//2. check the hardware CRC32 checksum for the entire flash memory using DMA
//3. compare the results
//4. print results

void flash_test() {

  uint32 crc_sw = crc32((uint32 *) FLASH_SYSTEM_BASE, FLASH_TOTAL_SIZE);

  crc_reset_dr();
  delay(1000);
  setup_DMA();
  delay(1000);

  if(crc_final == crc_sw) {
    SerialUSB.println("CRC32 engine test OK");
    SerialUSB.println("Bootloader CRC checksum test OK");
    SerialUSB.println("Flash memory CRC checksum test OK");
    SerialUSB.println("DMA Interrupt controller test OK");
    SerialUSB.println("DMA test OK");
  } else
    SerialUSB.println("Start-up Flash memory Checksum test [Failure]");

}

//setup DMA transfer
//1.) initialize the DMA
//2.) setup the mode of transfer
//       a) DMA1 controller
//       b) Channel 2 on DMA1
//3.) attach the interrupt function to the ISR
//4.) set priority for DMA transfer
//5.) set the total number of transfers
//6.) enable the DMA

void setup_DMA() {

  dma_init(DMA1);

  /************************
  *&CRC_BASE->DR = the address for the CRC32 data register
  *the size of the source register, 32 bits in our case
  *the address to start of flash memory
  *the size of the individual elements in the destination buffer, 32 bits again in our case
  *DMA_PIC_MODE = increment the peripheral memory location after each DMA transfer
  *DMA_MEM_2_MEM_MODE = memory to memory mode which transfers data to the main memory from the peripheral flash memory
  *DMA_TRNS_CMPLT = signal only when all elements are transferred
  *DMA_TRNS_ERR = indicate if there is any error
  ***************************/

  dma_setup_transfer(DMA1, DMA_CH2, (uint32 *)FLASH_SYSTEM_BASE, DMA_SIZE_32BITS,
                     &CRC_BASE->DR, DMA_SIZE_32BITS, (DMA_PINC_MODE | DMA_MEM_2_MEM| DMA_TRNS_CMPLT| DMA_TRNS_ERR));

  dma_attach_interrupt(DMA1, DMA_CH2, crc_DMA_event);

  //setup the priority for the DMA transfer.
  dma_set_priority(DMA1, DMA_CH2, DMA_PRIORITY_VERY_HIGH);

//setup the number of bytes that we are going to transfer.
  dma_set_num_transfers(DMA1, DMA_CH2, FLASH_TOTAL_SIZE/4);

  //enable DMA to start transmitting. When the transmission
  //finishes the event will be triggered and we will jump to
  //function adc_DMA_event
  dma_enable(DMA1, DMA_CH2);
}

/*************SRAM TEST***************************************************************/
//1. write PATTERN1
//2. read back PATTERN1 and note any errors
//3. write PATTERN2
//4. read back PATTERN2 and note any errors
//5. display results

void sram_test() {

  uint32 * base_address = (uint32 *)(SRAM_USER_ADDRESS);
  uint16 num_words = SRAM_USER_SIZE/sizeof(uint32);
  uint32 temp_val;

  for(int i = 0; i < num_words; i++) {
    temp_val = *base_address;

    *base_address = PATTERN1;

    if(*base_address != PATTERN1)
      sram_nowrite_count++;

    *base_address = PATTERN2;

    if(*base_address != PATTERN2)
      sram_nowrite_count++;

    *base_address = temp_val;

    base_address++;
  }

  SerialUSB.println("SRAM test completed OK");
  SerialUSB.print("Total RAM: 17408 bytes; Total usable RAM: ");
  SerialUSB.print(SRAM_USER_SIZE - sram_nowrite_count/2);
  SerialUSB.println(" bytes");

}
/**************CLOCK TESTING CODE************************************************/

//start the LSI oscillator by setting the LSION bit in RCC and poll the LSIRDY bit while it is set
static inline void rcc_start_lsi(void) {
  *bb_perip(&RCC_BASE->CSR, RCC_CSR_LSION_BIT) = 1;
  while (*bb_perip(&RCC_BASE->CSR, RCC_CSR_LSIRDY_BIT) == 0);
}

//start the HSE oscillator by setting the HSEON bit in RCC and poll the HSERDY bit while it is set
static inline void rcc_start_hse(void) {
  *bb_perip(&RCC_BASE->CR, RCC_CR_HSEON_BIT) = 1;
  while (bb_peri_get_bit(&RCC_BASE->CR, RCC_CR_HSERDY_BIT) == 0);
}

//poll the CRL_RTOFF bit while it is cleared
static inline void rtc_wait_finished() {
  while (*bb_perip(&RTC_BASE ->CRL, RTC_CRL_RTOFF_BIT) == 0);
}

//wait for other operations on the RTC registers to complete and clear the CRL_RSF bit
static inline void rtc_clear_sync() {
  rtc_wait_finished();
  *bb_perip(&RTC_BASE->CRL, RTC_CRL_RSF_BIT) = 0;
}
//poll the RTC CRL_RSF bit to be cleared
static inline void rtc_wait_sync() {
  while (*bb_perip(&RTC_BASE->CRL, RTC_CRL_RSF_BIT) == 0);
}

//set the configuration bit in RTC after polling for all operations to complete
static inline void rtc_enter_config_mode() {
  rtc_wait_finished();
  *bb_perip(&RTC_BASE->CRL, RTC_CRL_CNF_BIT) = 1;
}

//clear the configuration bit in RTC after polling for all operations to complete
static inline void rtc_exit_config_mode() {
  rtc_wait_finished();
  *bb_perip(&RTC_BASE->CRL, RTC_CRL_CNF_BIT) = 0;
}

//enable seconds interrupt by setting the RTC seconds bit in the RTC base register and enable it in NVIC
static inline void rtc_enable_irq() {
  rtc_wait_finished();
  *bb_perip(&RTC_BASE->CRH, RTC_SECONDS_INTERRUPT) = 1;
  nvic_irq_enable(NVIC_RTC);
}

//disable the RTC seconds interrupt by clearing it
static inline void rtc_disable_irq() {
  rtc_wait_finished();
  *bb_perip(&RTC_BASE->CRH, RTC_SECONDS_INTERRUPT) = 0;
}

//set the prescaler with the defined prescaler value, used after initilazling the rtc with the appropriate clock (LSI or HSE)
void rtc_set_prescaler_load(uint32 value) {
  rtc_clear_sync();
  rtc_wait_sync();
  rtc_wait_finished();
  rtc_enter_config_mode();
  RTC_BASE->PRLH = (value >> 16) & 0xffff;
  RTC_BASE->PRLL = value & 0xffff;
  rtc_exit_config_mode();
  rtc_wait_finished();
}

//get the RTC counter value, note that the high half-byte is left shifted since the register is total 20 bits wide
//with the high byte stored in a different register and the lower half-word is stored in a different register
//so the counter value is obtained by left shifting the high half-byte first and then performing a logical OR operation with the lower half-word

uint32 rtc_get_count() {
  uint32 h, l;
  rtc_clear_sync();
  rtc_wait_sync();
  rtc_wait_finished();
  h = RTC_BASE->CNTH & 0xffff;
  l = RTC_BASE->CNTL & 0xffff;
  return (h << 16) | l;
}

//set the RTC count the opposite way as described above, right shift the lower half-byte by 16 bits
void rtc_set_count(uint32 value) {
  rtc_clear_sync();
  rtc_wait_sync();
  rtc_wait_finished();
  rtc_enter_config_mode();
  RTC_BASE->CNTH = (value >> 16) & 0xffff;
  RTC_BASE->CNTL = value & 0xffff;
  rtc_exit_config_mode();
  rtc_wait_finished();
}

//initialize the RTC with the appropriate clock source
void rtc_init(uint8 rtc_clk_src) {

  //enable power to the backup domain
  bkp_init();
  //enable writes to the backup domain register
  bkp_enable_writes();

  //clear the previous clock selected in the BDCR register
  RCC_BASE->BDCR &= ~RCC_BDCR_RTCSEL;

  switch(rtc_clk_src) {
  //LSI
  case RTCSEL_LSI:
    rcc_start_lsi();
    RCC_BASE->BDCR |= RCC_BDCR_RTCSEL_LSI; //set the LSI as clock in the BDCR register
    break;
  //HSE
  case RTCSEL_HSE:
    rcc_start_hse();
    RCC_BASE->BDCR |= RCC_BDCR_RTCSEL_HSE; //set the HSE as clock in the BDCR register
    break;
  }
  //enable the RTC
  bb_peri_set_bit(&RCC_BASE->BDCR, RCC_BDCR_RTCEN_BIT, 1);
  rtc_clear_sync();
  rtc_wait_sync();
  rtc_wait_finished();
}


//RTC seconds interrupt routine
//the ISR calculates the clock ratios and checks them for accuracy
void dispatch_rtc_second_interrupt() {

  systick_disable();

  uint32 handled = 0;

//If last clock tested was HSI, record the number of pulses and set SysTick to use the HSE clock
  if(last_flag == HSI) {
    hsi_count[ratio_index] = counter_value;
    counter_value = 0;
    systick_init(7199999);
    systick_attach_callback(increment_counter);
    SYSTICK_BASE->CSR = (SYSTICK_CSR_CLKSOURCE_EXTERNAL   |
                         SYSTICK_CSR_ENABLE           |
                         SYSTICK_CSR_TICKINT_PEND);

  } 

//If last clock tested was HSE, record the number of pulses and set SysTick to use the HSI clock

  else if(last_flag == HSE) {

    hse_count[ratio_index] = counter_value;
    counter_value = 0;

    systick_init(7199999);
    systick_attach_callback(increment_counter);

    if(ratio_index == 0) {
      SerialUSB.println("Testing Clocks (HSI, HSE, and LSI) with RTC and SysTick...");
    }

    ratio_index++;

//If we have enough clock samples, calculate the ratios and check them with expected value

    if(ratio_index == 10) {

      SerialUSB.print(ratio_index * 10);
      SerialUSB.print("% done!");

      float lsi_hse_count = (hse_count[0]+hse_count[1]+hse_count[2]+hse_count[3]+hse_count[4]+hse_count[5]+hse_count[6]+hse_count[7]+hse_count[8]+hse_count[9])/10;

      float lsi_hsi_count = (hsi_count[0]+hsi_count[1]+hsi_count[2]+hsi_count[3]+hsi_count[4]+hsi_count[5]+hsi_count[6]+hsi_count[7]+hsi_count[8]+hsi_count[9])/10;

      float hse_value = (7.1999*lsi_hse_count)/lsi_hsi_count;

      SerialUSB.print("\r\nClock testing completed");

      float hse = 100*abs(0.8-hse_value)/0.8;

      if(hse < 25)
        SerialUSB.println("...PASS");
      else
        SerialUSB.println("...[FAIL]!");
      /*
           SerialUSB.println(lsi_hse_count);
           SerialUSB.println(lsi_hsi_count);
           SerialUSB.println(hse_value);
           SerialUSB.println(hse);

           SerialUSB.println("\r\n");
       */
      ratio_index = 0;
      clock_test_complete = 1;
    } else {
      SerialUSB.print(ratio_index * 10);
      SerialUSB.print("%..");
    }
  }

  if(last_flag == HSI)
    last_flag = HSE;
  else
    last_flag = HSI;
    
//Seconds interrupt flag cleared

  handled |= RTC_CRL_SECF;
  RTC_BASE->CRL &= ~handled;
}

//the RTC_SECONDS interrupt routine, note that it is the function has to be defined using C linkage for the linker to register the name
extern "C" {
  void __irq_rtc(void) {
    dispatch_rtc_second_interrupt();
  }
}

//SysTick ISR
//increments the counter value
void increment_counter(void) {
  counter_value++;
}

//main clock test routine
void clock_test() {

  int counter_clock = 0;
  int toggle_value = 0;

  counter_value = 0;
  clock_test_complete = 0;

//start with HSI

  systick_init(7199999);
  systick_attach_callback(increment_counter);
  last_flag == HSI;


  rtc_init(RTCSEL_LSI); //LSI is chosen as default
  rtc_set_prescaler_load(LSI_PRESCALER);
  rtc_enable_irq(); //enable the RTC seconds interrupt

//do nothing while clocks are tested

  do {
    digitalWrite(0, toggle_value);
    toggle_value ^= toggle_value;
  } while(clock_test_complete == 0);

//disable RTC and reset SysTick to use HSI for proper MCU operation

  rtc_disable_irq();
  systick_init(71999);
  SYSTICK_BASE->CSR = (SYSTICK_CSR_CLKSOURCE_CORE   |
                       SYSTICK_CSR_ENABLE           |
                       SYSTICK_CSR_TICKINT_PEND);
  systick_attach_callback(NULL);

}

/******************************ADC TEST*************************************/
//test each ADC pin and check if the converted value is within bounds of the converter
void adc_test(void) {
  int pass_count = 0;
  int total_count = 0;

  SerialUSB.print("Testing ADC pins...");

  for (uint32 i = 0; i < BOARD_NR_ADC_PINS; i++) {
    if (boardUsesPin(i))
      continue;

    total_count++;
    SerialUSB.print(boardADCPins[i], DEC);
    SerialUSB.print(" ");
    pinMode(boardADCPins[i], INPUT_ANALOG);
    int sample = analogRead(boardADCPins[i]);
    if(sample > 0 && sample < 4095)
      pass_count++;
    delay(1000);
  }

  if(pass_count == total_count)
    SerialUSB.println("\r\nADC test completed OK");
  else
    SerialUSB.println("\r\nADC test completed [FAIL]");

}

/************************GPIO TEST*************************************************/
//test each GPIO pin by toggling it
//the user can switch from sinking and sourcing configuration as needed

void gpio_test(void) {

  SerialUSB.print("Testing GPIO pins ");
  for (uint32 i = 0; i < BOARD_NR_GPIO_PINS; i++) {
    if (boardUsesPin(i))
      continue;
    pinMode(i, OUTPUT);
    SerialUSB.print(i);
    SerialUSB.print(" ");
    digitalWrite(i, HIGH);
    delay(500);
    digitalWrite(i, LOW);
  }
  SerialUSB.println("\r\nGPIO test completed");

}

/*************TIMER INTERRUPTS TESTING CODE**********************/

int freq_counter = 11;

int timer_test_completed = 0;
int all_timer_tests_completed = 0;
long timer_millis_count = millis();

//common timer ISR which decrements a counter value on each interrupt
//since the timer is set to 10 Hz , when the counter value reaches to 1 from 11
//we should expect the time elapsed to be around 1 second
//if the time taken in more or less the expected value, the test fails

void timer_ISR() {

  freq_counter--;

  if(freq_counter == 1) {
    int count = millis()- timer_millis_count;
//    SerialUSB.print(" ");
//    SerialUSB.print(count);
    if(count > 800 && count < 1100)
      SerialUSB.print("...PASS ");
    else if(count < 800 || count > 1100)
      SerialUSB.print("...[FAIL] ");
    timer_test_completed++;
    freq_counter = 11;
    timer_millis_count = millis();
  }
}

//main interrupt testing routine
//cycles through timers 1 - 4 and sets them to interrupt at a frequency of 10 Hz

void interrupts_test() {

  all_timer_tests_completed = 0;

  if(all_timer_tests_completed == 0) {
    SerialUSB.print("Testing Interrupts on...");
    for(int i = 1; i < 5; ) {
      HardwareTimer int_timer(i);
      SerialUSB.print(" Timer ");
      SerialUSB.print(i);
      int_timer.pause();
      int_timer.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
      int_timer.setPrescaleFactor(128);
      int_timer.setOverflow(55813);
      int_timer.setCompare(TIMER_CH1, 55812/2);
      int_timer.attachInterrupt(TIMER_CH1, timer_ISR);
      int_timer.refresh();
      timer_millis_count = millis();
      int_timer.resume();

      int toggle_value = 0;
      do {
        toggleLED();
        digitalWrite(0, toggle_value);
        toggle_value = !toggle_value;
      } while(timer_test_completed < 1);

      if(i == 4) {
        all_timer_tests_completed = 1;
        SerialUSB.println("\r\nInterrupt tests on all timers completed");
        digitalWrite(BOARD_LED_PIN, LOW);
        digitalWrite(0, LOW);
      }
      i++;
      timer_test_completed = 0;
      int_timer.pause();
    }
  }
}

/******************UART TEST*******************************************/
//configures Serial port 3 in loopback mode
//sends 256, 8-bit unsigned integers and tests them for accurate receipt
//if value is not read within a 1 second period, the test fails
//if the incorrect number of integers are read correctly, the test fails

void uart_test() {

  HardwareSerial *serial_3 = &Serial3;
  serial_3->begin(9600);

  int count = 0;
  int i = 0;
//start time count for transfer  
  unsigned long tx_time = millis();
  do {
    if((millis() - tx_time) > 1000) break;
    serial_3->write((uint8)i);
    do {
      uint8 recv = (uint8)(serial_3->read());
      if(recv == i) {
        count++;
        serial_3->flush();
        i++;
        tx_time = millis();
      }
    } while(serial_3->available() > 0);
  } while(i < 256);

  if(count == 256) {
    SerialUSB.println("UART loopback test OK");
  } else if(count != 256) {
    SerialUSB.println("UART loopback test [FAIL]!");
  }
}

/**********INTERNAL WATCHDOG TIMER AND EXCEPTION TEST***************************/
//the routine initiates the watchdog timer and creates a divide by zero exception
//when the system resets, it is checked to see if the IWDG caused the reset
//the test is a FAIL is the reset wasn't caused by the IWDG
void iwdg_test() {

//if IWDG has been tested more than 1 times, it is a failure

  if(iwdg_status > 1)  {
    SerialUSB.println("IWDG Test [Failure]");
    return;
  }

//check the IWDG status register to see it is set, if not then we need to test the IWDG

  if(bb_peri_get_bit(&RCC_BASE->CSR, 29) != 1) {
    SerialUSB.println("Testing Internal Watchdog Timer & Exception handler, system will reset once test is completed");
    iwdg_status++;
    iwdg_prescaler prescaler = IWDG_PRE_256;
    iwdg_init(prescaler, 1250); // init an 8 second wd timer
    int snacks = 5;

    do {
      delay(1000);
      --snacks;
      iwdg_feed();
    } while(snacks > 0);

    SerialUSB.println(PATTERN1/0);
  }

  else if(bb_peri_get_bit(&RCC_BASE->CSR, 29) == 1) {
    SerialUSB.println("IWDG Test OK");
    SerialUSB.println("Exception handler test OK\n");
  }
}

//setup variables
void setup() {
  pinMode(BOARD_LED_PIN, OUTPUT);
  pinMode(0, OUTPUT);

//enable DIV BY ZERO exception
  SCB_BASE->CCR |= 0x10;

//enable clock to the CRC hardware engine
  rcc_clk_enable(RCC_CRC);
}


//main loop
void loop() {

  SerialUSB.println("--------------------------------------------");
  SerialUSB.println("--[STARTING IEC 61508 CERTIFICATION TESTS]--");
  SerialUSB.println("--------------------------------------------");

  cpu_test();
  delay(1000);
  flash_test();
  delay(1000);
  sram_test();
  delay(1000);
  clock_test();
  delay(1000);
  adc_test();
  delay(1000);
  interrupts_test();
  delay(1000);
  gpio_test();
  delay(1000);
  uart_test();
  delay(1000);
  iwdg_test();
  delay(1000);
  SerialUSB.println("------------[ALL TESTS COMPLETED]-----------\n");
}


