/*****************************************************************************
 *   i2c.h:  Header file for NXP LPC23xx/24xx Family Microprocessors
 *
 *   History
 *   2006.07.19  ver 1.00    Prelimnary version, first Release
 *
******************************************************************************/
#ifndef __ADE7858_H
#define __ADE7858_H

#include "stdint.h"
#include "stm32f4xx_hal_i2c.h"




//Functions
void init_ADE7858_ADC(void); 
void Read_ADE7858_I2C(I2C_HandleTypeDef *hi2c, uint16_t RegisterAddress,uint8_t RegisterSize,uint8_t RxBuffer[], uint8_t DataSize,uint32_t Timeout);
void Write_ADE7858_I2C(I2C_HandleTypeDef *hi2c, uint16_t RegisterAddress, uint32_t Data, uint16_t DataSize,uint32_t Timeout);
int Read_Energy_Meter(I2C_HandleTypeDef *hi2c, uint16_t RegisterAddress,uint8_t RegisterSize, uint8_t RxBuffer[], uint8_t DataSize, uint16_t Modbusregister, uint32_t Timeout);
void Select_PhaseMeasurement(I2C_HandleTypeDef *hi2c,uint16_t RegisterAddress,uint8_t RegisterSize,uint8_t WriteValue,uint32_t Timeout);
void Select_PeriodMeasurement(I2C_HandleTypeDef *hi2c,uint16_t RegisterAddress,uint8_t RegisterSize,uint8_t WriteValue,uint32_t Timeout); 
void Calculate_Average_AVRMS(uint8_t RxBuffer[],uint16_t ModbusRegister);
void Calculate_Average_BVRMS(uint8_t RxBuffer[],uint16_t ModbusRegister);
void Calculate_Average_CVRMS(uint8_t RxBuffer[],uint16_t ModbusRegister);
int Calculate_ApparentPower(uint32_t Phase_VRMS,uint32_t Phase_IRMS,uint16_t Modbusregister);
int Calc_PowerFactor(uint32_t Phase_Apparent_Power,uint32_t Phase_AWATT,uint16_t Modbusregister);
void Start_DSP(void);
uint32_t roundoff(uint32_t product,uint32_t multiplier);


void GPIO_Configuration(void); 
void Read_EMeterConfig_Registers(void );



//Macros
#define   ANGLE0             0xE601
#define   ANGLE1             0xE602
#define   ANGLE2             0xE603

#define   APHCAL             0xE614
#define   BPHCAL             0xE615
#define   CPHCAL             0xE616

#define   AWGAIN             0x4391 
#define   BWGAIN             0x4393
#define   CWGAIN             0x4395

#define   AWATT              0xE513
#define   BWATT              0xE514
#define   CWATT              0xE515

#define   AWATTHR            0xE400
#define   BWATTHR            0xE401
#define   CWATTHR            0xE402

#define   LCYCMODE           0xE702
#define   LINECYC            0xE60C
#define   VLEVEL             0x43B3

#define   AVA                0xE519
#define   BVA                0xE51A
#define   CVA                0xE51B

#define   PERIOD             0xE607

#define   RUN                0xE228
#define   STOP               0xE203

#define   CONFIG             0xE618
#define   CONFIG2            0xEC01    //bit length 8
#define   DICOEFF            0x43B8
#define   STATUS1            0xE503
#define   GAIN               0xE60F    //bit length 16

#define   AVGAIN             0x4381
#define   AVRMSOS            0x4388

#define   WTHR1              0x43AB    //32 bits length
#define   WTHR0              0x43AC

#define   VARTHR0            0x43AE
#define   VATHR0             0x43AA

#define   VARTHR1            0x43AD
#define   VATHR1             0x43A9

#define   AWATTHR            0xE400
#define   BWATTHR            0xE401
#define   CWATTHR            0xE402

#define   CFMODE             0xE610    //16bits
#define   CF1DEN             0xE611
#define   CF2DEN             0xE612
#define   CF3DEN             0xE613

#define   AIGAIN             0x4380
#define   BIGAIN             0x4382
#define   CIGAIN             0x4384
#define   NIGAIN             0x4386

#define   AVRMS              0x43C1
#define   BVRMS              0x43C3
#define   CVRMS              0x43C5

#define   AIRMS              0x43C0
#define   BIRMS              0x43C2
#define   CIRMS              0x43C4

#define   VNOM               0xE520
#define   VPEAK              0xE501
#define   HPFDIS             0x43B6

#define   MASK00             0xE50A
#define   MASK01             0xE50B

#define   STATUS0            0xE502
#define   STATUS1            0xE503
#define   VPEAK              0xE501

#define   COMPMODE           0xE60E
#define   MMODE              0xE700

#define   ANGLE2             0xE603

#define   AVGAIN             0x4381
#define   BVGAIN             0x4383
#define   CVGAIN             0x4385

#define   AIGAIN             0x4380
#define   BIGAIN             0x4382
#define   CIGAIN             0x4384

#define   PEAKCYC            0xE703

#define   CF1DEN             0xE611
#define   CF2DEN             0xE612
#define   CF3DEN             0xE613

#define   MASK_0             0xE50A
#define   MASK_1             0xE50B



#endif 



/****************************************************************************
**                            End Of File
*****************************************************************************/