#pragma once

#include <unordered_set>

//ADC registers, accessed via i2c port
#define CLK_CTRL  (0x000)
#define PLL_DEN (0x001)
#define PLL_NUM (0x002)
#define PLL_CTRL (0x003)
#define PLL_LOCK (0x005)
#define MASTER_ENABLE (0x040)
#define ADC_ENABLE (0x041)
#define POWER_ENABLE (0x042)
#define ASIL_CLEAR (0x080)
#define ASIL_MASK (0x081)
#define ASIL_FLAG (0x082)
#define ASIL_ERROR (0x083)
#define CRC_VALUE_L (0x084)
#define CRC_VALUE_L (0x084)
#define CRC_VALUE_H (0x085)
#define RM_CRC_ENABLE (0x086)
#define RM_CRC_DONE (0x087)
#define RM_CRC_VALUE_L (0x088)
#define RM_CRC_VALUE_H (0x089)
#define LNA_GAIN (0x100)
#define PGA_GAIN (0x101)
#define ADC_ROUTING1_4 (0x102)
#define DECIM_RATE (0x140)
#define HIGH_PASS (0x141)
#define ACK_MODE (0x143)
#define TRUNCATE_MODE (0x144)
#define SERIAL_MODE (0x1C0)
#define PARALLEL_MODE (0x1C1)
#define OUTPUT_MODE (0x1C2)
#define ADC_READ_0 (0x200)
#define ADC_READ1 (0x201)
#define ADC_SPEED (0x210)
#define ADC_MODE (0x211)
#define MP0_MODE (0x250)
#define MP1_MODE (0x251)
#define MP0_WRITE (0x260)
#define MP1_WRITE (0x261)
#define MP0_READ (0x270)
#define MP1_READ (0x271)
#define SPI_CLK_PIN (0x280)
#define MISO_PIN (0x281)
#define SS_PIN (0x282)
#define MOSI_PIN (0x283)
#define ADDR15_PIN (0x284)
#define FAULT_PIN (0x285)
#define FS_ADC_PIN (0x286)
#define CS_PIN (0x287)
#define SCLK_ADC_PIN (0x288)
#define ADC_DOUT0_PIN (0x289)
#define ADC_DOUT1_PIN (0x28A)
#define ADC_DOUT2_PIN (0x28B)
#define ADC_DOUT3_PIN (0x28C)
#define ADC_DOUT4_PIN (0x28D)
#define ADC_DOUT5_PIN (0x28E)
#define DATA_READY_PIN (0x291)
#define XTAL_CTRL (0x292)
#define ADC_SETTING1 (0x301)
#define ADC_SETTING2 (0x308)
#define ADC_SETTING3 (0x30A)
#define DEJITTER_WINDOW (0x30E)
#define CRC_EN (0xFD00)

//Pins (all in terms of WiringPI numbers);
#define RPI_N_CONV_START_PIN 4
#define RPI_N_FAULT_PIN 5
#define RPI_N_STATUS_R_PIN 12
#define RPI_N_STATUS_G_PIN 16
#define RPI_N_STATUS_B_PIN 13
#define RPI_N_RESET_PWDN_PIN 20
#define RPI_DATA_READY_PIN 6
#define RPI_SCLK_ADC_PIN 21

//These are in terms of the broadcom pins, because that's how we read them
#define DIN_PIN_0 7
#define DIN_PIN_1 25
#define DIN_PIN_2 24
#define DIN_PIN_3 22
#define DIN_PIN_4 13
#define DIN_PIN_5 27
#define DIN_PIN_6 17
#define DIN_PIN_7 18


std::unordered_set<unsigned short> raspiInputPins{RPI_SCLK_ADC_PIN, RPI_DATA_READY_PIN, RPI_N_FAULT_PIN,
                                                  DIN_PIN_0, DIN_PIN_1, DIN_PIN_2, DIN_PIN_3,
                                                  DIN_PIN_4, DIN_PIN_5, DIN_PIN_6, DIN_PIN_7};
std::unordered_set<unsigned short> raspiOutputPins{RPI_N_CONV_START_PIN, RPI_N_STATUS_R_PIN, RPI_N_STATUS_G_PIN, RPI_N_STATUS_B_PIN, RPI_N_RESET_PWDN_PIN};
