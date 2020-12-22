#include <stdbool.h>
#include "math.h"
#include "ADE7858.h" 
#include "main.h"





I2C_HandleTypeDef I2c2Handle;


uint8_t ADE7858_DevAddress_WR=0x70;      //0111 0000; MSB LSB
uint8_t ADE7858_DevAddress_RD=0x71;      //0111 0001 ; MSB LSB



/* =================================================================== */
/*                         Enetgy meter calibration parameters         */
/* =================================================================== */
volatile uint32_t AVGAIN_DATA;
volatile uint32_t BVGAIN_DATA;
volatile uint32_t CVGAIN_DATA;

volatile uint32_t AIGAIN_DATA;
volatile uint32_t BIGAIN_DATA;
volatile uint32_t CIGAIN_DATA;

volatile uint32_t AWGAIN_DATA;
volatile uint32_t BWGAIN_DATA;
volatile uint32_t CWGAIN_DATA;

volatile uint16_t APHCAL_DATA; 
volatile uint16_t BPHCAL_DATA;
volatile uint16_t CPHCAL_DATA;

/*
Initialize energy meter 
*/
void init_ADE7858_ADC(void)
{

  uint8_t RxBuff[4]={0,0,0,0};
  uint32_t temp=0; 
  uint32_t i=0; 

  Read_EMeterConfig_Registers();

	HAL_GPIO_WritePin(GPIOD,ADE7858_RESETB , GPIO_PIN_SET);
  HAL_Delay(20);
	HAL_GPIO_WritePin(GPIOD,ADE7858_RESETB , GPIO_PIN_RESET);
  HAL_Delay(20);
	HAL_GPIO_WritePin(GPIOD,ADE7858_RESETB , GPIO_PIN_SET);
  HAL_Delay(20);

  Read_ADE7858_I2C(&I2c2Handle,STATUS1,2,&RxBuff[0],0x04,1000);

  for(i=0; i<4;i++)
  {
  temp=temp|(uint32_t)RxBuff[i];
     if(i<3)
     {
     temp=temp<<8;
     }
  }

  Write_ADE7858_I2C(&I2c2Handle,STATUS1,temp,6,1000);

  //loCk i2C as the active serial port, set Bit 1 (I2C_LOCK) of the CONFIG2 register to 1: sending 3 bytes( 2 bytes for registers+ 2Bytes of data)
   Write_ADE7858_I2C(&I2c2Handle,CONFIG2,1,3,1000);

  //set CF1DEN  to 156
  Write_ADE7858_I2C(&I2c2Handle,CF1DEN,0x009C,4,1000);  //MC here is: 6400im/KWH
  //set CF2DEN  to 156
  Write_ADE7858_I2C(&I2c2Handle,CF2DEN,0x009C,4,1000);
  //set CF3DEN  to 156
  Write_ADE7858_I2C(&I2c2Handle,CF3DEN,0x009C,4,1000);

 //set COMPMODE Set COMPMODE = 0x0111 to enable the CF1 function of the Phase A energy, the CF2 function of the Phase B energy, and the CF3 function of the Phase C energy.
  Write_ADE7858_I2C(&I2c2Handle,COMPMODE,0x0111,4,1000);
 
  //Enable the CF1, CF2, and CF3 pins by setting Bit 9 (CF1DIS), Bit 10 (CF2DIS), and Bit 11 (CF3DIS) to 0 in the CFMODE register. 
  //temp=0;
  //temp=temp|(uint32_t)RxBuff[0];
 // temp=temp=temp<<8;
 // temp=temp|(uint32_t)RxBuff[1];
 // temp=temp&0xF1FF;

  //Enable the CF1, CF2, and CF3 pins by setting Bit 9 (CF1DIS), Bit 10 (CF2DIS), and Bit 11 (CF3DIS) to 0 in the CFMODE register. 
  Write_ADE7858_I2C(&I2c2Handle,CFMODE,0x7000,4,1000);  //see page92 of datasheet REv.H

  //set PEAKCYC  to 16 cycles
  Write_ADE7858_I2C(&I2c2Handle,PEAKCYC,16,3,1000);

  Write_ADE7858_I2C(&I2c2Handle,MMODE,4,3,1000);

  //set VNOM to 120
  Write_ADE7858_I2C(&I2c2Handle,VNOM,0x0013B7E7,6,1000); // VNOM=(120/Vfs)*4191910; Vfs= 0.5*(1100)/2^0.5; Vn=120 Vrms see page 58 of datasheet REV H

  //Voltage gain calibration: 
  //SET THE AVGIAN
  Write_ADE7858_I2C(&I2c2Handle,AVGAIN,AVGAIN_DATA,6,1000);  
  //SET THE AVGIAN
  Write_ADE7858_I2C(&I2c2Handle,BVGAIN,BVGAIN_DATA,6,1000);
  //SET THE AVGIAN
  Write_ADE7858_I2C(&I2c2Handle,CVGAIN,CVGAIN_DATA,6,1000);

  //Calibrate the Phase shift
  Write_ADE7858_I2C(&I2c2Handle,APHCAL,APHCAL_DATA,4,1000);
  Write_ADE7858_I2C(&I2c2Handle,BPHCAL,BPHCAL_DATA,4,1000);
  Write_ADE7858_I2C(&I2c2Handle,CPHCAL,CPHCAL_DATA,4,1000);

 //enable LENERGY interrupt (bit5)
  Write_ADE7858_I2C(&I2c2Handle,MASK_0,0x00000020,6,1000);

  //Line Cycle Active Energy Accumulation Mode 
  Write_ADE7858_I2C(&I2c2Handle,LINECYC,0x2EDF,4,1000); //this value is calculated based on the equation in the spreadsheet; 0x1C17 for 15A; 

  //BIT0: The line cycle energy accumulation mode is activated by setting Bit 0 (LWATT) in the LCYCMODE register
  //BIT6: Setting Bit 6 (RSTREAD) of the LCYCMODE register enables a read-with-reset for all watt-hour accumulation registers, that is,the registers are reset to 0 after a read operation.
  //BITS[5:3] when sets phaseA,B, and C zero crossings are enabled. select only one phase at a time for calibration
  
  //Select only one phase at a time for inclusion in the zero crossings count during calibration
  Write_ADE7858_I2C(&I2c2Handle,LCYCMODE,0x09,1,1000); //phase A zero crossings enabled.
 // Write_ADE7858_I2C(&I2c2Handle,LCYCMODE,0x11,1,1000); //phase B zero crossings enabled.
  //Write_ADE7858_I2C(&I2c2Handle,LCYCMODE,0x21,1,1000); //phase C zero crossings enabled.

  //Active Power Gain Calibration 
  Write_ADE7858_I2C(&I2c2Handle,AWGAIN,AWGAIN_DATA,6,1000);  
  Write_ADE7858_I2C(&I2c2Handle,BWGAIN,BWGAIN_DATA,6,1000);   
  Write_ADE7858_I2C(&I2c2Handle,CWGAIN,CWGAIN_DATA,6,1000);    

  //WTHR0 register value: 0x00FF6A6B
  Write_ADE7858_I2C(&I2c2Handle,WTHR0,0x0047E36C,6,1000);   

  //WTHR1 register value: 0x00000001
  Write_ADE7858_I2C(&I2c2Handle,WTHR1,0x00000006,6,1000);   

  //Make VARTHR0 equal to VTHR1 
  Write_ADE7858_I2C(&I2c2Handle,VARTHR0,0x0047E36C,6,1000);  

  //Make VARTHR1 equal to WTHR1 
  Write_ADE7858_I2C(&I2c2Handle,VARTHR1,0x00000006,6,1000);

  //Make VATHR0 equal to WTHR0 
  Write_ADE7858_I2C(&I2c2Handle,VATHR0,0x0047E36C,6,1000); 

  //Make VARTHR1 equal to WTHR1 
  Write_ADE7858_I2C(&I2c2Handle,VARTHR1,0x00000006,6,1000);  

  
  //SET VLEVEL
  Write_ADE7858_I2C(&I2c2Handle,VLEVEL,0x00185432,6,1000);   //VELEVEL=( VfullScale/Vnominal)*491520= 0x00185432; Vnominal=120V see page48 of datasheet REVH  185432


  //Current gain calibration:
  //SET THE AIGAIN
  Write_ADE7858_I2C(&I2c2Handle,AIGAIN,AIGAIN_DATA,6,1000);   //it was 0x0FB80000
  //SET THE BIGAIN
  Write_ADE7858_I2C(&I2c2Handle,BIGAIN,BIGAIN_DATA,6,1000);   //it was 0x0FC20000
  //SET THE CIGAIN
  Write_ADE7858_I2C(&I2c2Handle,CIGAIN,CIGAIN_DATA,6,1000);   //it was 0x0FD00000

  //setting [10:9] bits to 01; measure phase between voltages
  //0x00 delay between voltages and currents per phase are measured 
  //0x10 delay between currents per phase are measured 
  Select_PhaseMeasurement(&I2c2Handle,COMPMODE,2,0x01,1000);     

  //Run DSP
  Write_ADE7858_I2C(&I2c2Handle,RUN,1,4,1000);


}
/******************************************************************************
Write ADE7858     
*****************************************************************************/
void Write_ADE7858_I2C(I2C_HandleTypeDef *hi2c,uint16_t RegisterAddress, uint32_t Data, uint16_t DataSize,uint32_t Timeout)
{
 uint8_t Tx_Buffer[6]={0,0,0,0,0,0}; 
 uint8_t status;

 Tx_Buffer[0]=RegisterAddress>>8;         //set Bit1 to 1 so that I2C is selected.
 Tx_Buffer[1]=RegisterAddress&0xff;
 
 if(DataSize==3)  //8 bits data size
 {
	 Tx_Buffer[2]=Data;
 }
 else if(DataSize==4) //16 bits data size
 {
	 Tx_Buffer[2]=Data>>8;
	 Tx_Buffer[3]=Data&0xff;
 }
 else if(DataSize==5)  //24 bits data size
 {
	 Tx_Buffer[2]=Data>>16;
	 Tx_Buffer[3]=Data>>8;
	 Tx_Buffer[4]=Data&0xff;
 }
 else if(DataSize==6)  //32 bits data size
 {
	 Tx_Buffer[2]=Data>>24;
	 Tx_Buffer[3]=Data>>16;
	 Tx_Buffer[4]=Data>>8;
	 Tx_Buffer[5]=Data&0xff;
 }
 else
 {
	 Tx_Buffer[2]=Data;
	 DataSize=3;
 }

  status=  HAL_I2C_Master_Transmit(&I2c2Handle, ADE7858_DevAddress_WR,Tx_Buffer,DataSize, Timeout);

}

/*****************************************************************************
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  ADE7858_I2cHandle: pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @param  ADE7858_DevAddress_RD: Target device address
  * @param  IntReg_Addres: Internal memory address
  * @param  MemAddSize: Size of internal memory address
  * @param  RxBuffer: Pointer to data buffer
  * @param  Size: Amount of data to be sent
  * @param  Timeout: Timeout duration
  * @retval HAL status
 
*****************************************************************************/
void Read_ADE7858_I2C(I2C_HandleTypeDef *hi2c, uint16_t RegisterAddress,uint8_t RegisterSize, uint8_t RxBuffer[], uint8_t DataSize,uint32_t Timeout)
{

 uint8_t Tx_Buffer[3]={0}; 
 uint8_t status;

 RxBuffer[0]=0;
 RxBuffer[1]=0;
 RxBuffer[2]=0;
 RxBuffer[3]=0;

 Tx_Buffer[0]=RegisterAddress>>8; //set Bit1 to 1 so that I2C is selected.
 Tx_Buffer[1]=RegisterAddress&0xff;
 Tx_Buffer[2]=RegisterSize;

  status=  HAL_I2C_Master_Transmit(&I2c2Handle, ADE7858_DevAddress_WR, Tx_Buffer,RegisterSize,Timeout);   //sending 2 bytes: config register address
  status=  HAL_I2C_Master_Receive(&I2c2Handle, ADE7858_DevAddress_RD,RxBuffer,DataSize, Timeout);    //read data into RxBuffer

}

/******************************************************************************
-this function starts the DSP by writing RUN=0x01 and STOP=0x00
-the protocol is:
  -'N'=0x4E
  -N=1
  -byte that does not have any meaning. Is introduced just to have N>0
-The MCU responds with:
  -'R'=0x52
  -0x7E
*****************************************************************************/
void Start_DSP(void) 
{ 

 Write_ADE7858_I2C(&I2c2Handle,RUN,1,3,1000);
 Write_ADE7858_I2C(&I2c2Handle,STOP,0,3,1000);

}
/******************************************************************************
-this function stops the DSP by writing RUN=0x00 and STOP=0x01
-the protocol is:
  -'O'=0x4F
  -N=1
  -byte that does not have any meaning. Is introduced just to have N>0
-The MCU responds with:
  -'R'=0x52
  -0x7E
*****************************************************************************/
void Stop_DSP(void) 
{

  Write_ADE7858_I2C(&I2c2Handle,RUN,0,3,1000);
  Write_ADE7858_I2C(&I2c2Handle,STOP,1,3,1000);

}
/*****************************************************************************
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  ADE7858_I2cHandle: pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @param  ADE7858_DevAddress_RD: Target device address
  * @param  IntReg_Addres: Internal memory address
  * @param  MemAddSize: Size of internal memory address
  * @param  RxBuffer: Pointer to data buffer
  * @param  Size: Amount of data to be sent
  * @param  Timeout: Timeout duration
  * @retval HAL status
 
*****************************************************************************/
int Read_Energy_Meter(I2C_HandleTypeDef *hi2c, uint16_t RegisterAddress,uint8_t RegisterSize, uint8_t RxBuff[], uint8_t DataSize, uint16_t Modbusregister, uint32_t Timeout)
{

 uint8_t Tx_Buffer[3]={0}; 
 uint8_t RxBuffer[4];

 uint32_t temp=0;
 uint32_t tempfloat1=0;
 uint32_t tempfloat2=0;

 RxBuffer[0]=0;
 RxBuffer[1]=0;
 RxBuffer[2]=0;
 RxBuffer[3]=0;

 Tx_Buffer[0]=RegisterAddress>>8; //set Bit1 to 1 so that I2C is selected.
 Tx_Buffer[1]=RegisterAddress&0xff;
 Tx_Buffer[2]=RegisterSize;

   HAL_I2C_Master_Transmit(&I2c2Handle, ADE7858_DevAddress_WR, Tx_Buffer,RegisterSize,Timeout);   //sending 2 bytes: config register address
   HAL_I2C_Master_Receive(&I2c2Handle, ADE7858_DevAddress_RD,RxBuffer,DataSize, Timeout);    //read data into RxBuffer


  if(DataSize==1)
  {
  temp=RxBuffer[0];

  return(temp);
  }
  else if(DataSize==2)
  {

  temp=(RxBuffer[0] & 0xff)<<8;
  temp=temp|(RxBuffer[1] & 0xff);
  }


  else if(DataSize==3)
  {
  temp=(RxBuffer[0] & 0xff)<<16;
  temp=temp|(RxBuffer[1] & 0xff)<<8;
  temp=temp|(RxBuffer[2] & 0xff);
  }
  else if(DataSize==4)
  {
  temp=(RxBuffer[0] & 0xff)<<24;
  temp=temp|(RxBuffer[1] & 0xff)<<16;
  temp=temp|(RxBuffer[2] & 0xff)<<8;
  temp=temp|(RxBuffer[3] & 0xff);
  }

  switch(RegisterAddress) 
  {
   case AVRMS:
   case BVRMS:
   case CVRMS:

   //=((0.5/2^0.5)/(4191910))*1101=((0.5/2^0.5)/(full Scale RMS))*voltage divider ratio
   temp=temp*0.000092860363;
   tempfloat1=temp*0.000092860363;

   tempfloat2=tempfloat1-temp;
     if(tempfloat2>=0.5)
     {
      temp++;
     }

   //SMRs_holding_regs[Modbusregister]=temp;
   break;

   case AIRMS:
   case BIRMS:
   case CIRMS:

   //=(((0.5/(2^0.5))/4191910)/30)*2000*1000=(((0.5/2^0.5)/(full Scale RMS)))/shunt resistors value)*CT_Ratio*1000; 
   //we multiplied by a thousand so that current are displayed in mA uinits
   temp=temp*0.0056227;
   tempfloat1=temp*0.0056227;
   tempfloat2=tempfloat1-temp;
     if(tempfloat2>=0.5)
     {
      temp++;
     }
   break;

   case ANGLE0:
   case ANGLE1:
   case ANGLE2:

   temp=temp*0.0843;
   tempfloat1=temp*0.0843;
     tempfloat2=tempfloat1-temp;
     if(tempfloat2>=0.5)
     {
      temp++;
     }
   break;

   case PERIOD:  
   temp++;   //add 1 accordong of formula page 36 of datasheet
   //temp=temp*3.90625;  //period in microseconds
   temp=256000/temp;  //frequency
   tempfloat1=256000/temp;  //frequency
   tempfloat2=tempfloat1-temp;
     if(tempfloat2>=0.5)
     {
      temp++;
     }

   break;

   case AWATT:
   case BWATT:
   case CWATT:

   temp=temp*0.00438; //(max full scale power)/(PMAX/16); max full scale power=VFS*IFS
   tempfloat1=temp*0.00438; //(max full scale power)/(PMAX/16); max full scale power=VFS*IFS
   tempfloat2=tempfloat1-temp;
      if(tempfloat2>=0.5)
     {
      temp++;
     }
    break;

    case MASK_0:
    case MASK_1:
    case AWATTHR:
    case BWATTHR:
    case CWATTHR:

    default: 
    break;
   }

   return(temp);

}

bool Read_ADE7858_DataReady (I2C_HandleTypeDef *hi2c, uint16_t RegisterAddress,uint8_t RegisterSize, uint8_t RxBuffer[], uint8_t DataSize,uint32_t Timeout)
{

 uint8_t Tx_Buffer[3]={0}; 
 uint8_t status;
 uint32_t temp=0;


 RxBuffer[0]=0;
 RxBuffer[1]=0;
 RxBuffer[2]=0;
 RxBuffer[3]=0;

 Tx_Buffer[0]=RegisterAddress>>8; //set Bit1 to 1 so that I2C is selected.
 Tx_Buffer[1]=RegisterAddress&0xff;
 Tx_Buffer[2]=RegisterSize;

  status=  HAL_I2C_Master_Transmit(&I2c2Handle, ADE7858_DevAddress_WR, Tx_Buffer,RegisterSize,Timeout);   //sending 2 bytes: config register address
  status=  HAL_I2C_Master_Receive(&I2c2Handle, ADE7858_DevAddress_RD,RxBuffer,DataSize, Timeout);    //read data into RxBuffer


  status=RxBuffer[1]&(0x02);

  if(status==2)
  {

  temp=temp|(uint32_t)RxBuffer[0];
  temp=temp<<8;
  temp=temp|(uint32_t)RxBuffer[1];
  temp=temp<<8;
  temp=temp|(uint32_t)RxBuffer[2];
  temp=temp<<8;
  temp=temp|(uint32_t)RxBuffer[3];

  Write_ADE7858_I2C(&I2c2Handle,MASK00,temp,6,1000); //clear DREADY bit  
    return(true);
  }
  else
  {
   //_TOGGLE GPIO
    return(false);
  }

}

void Calculate_Average_AVRMS(uint8_t RxBuffer[],uint16_t ModbusRegister)
{
    uint32_t temp=0; 
    static uint32_t temp_AVRMS=0;
    static uint32_t count_AVRMS=0;

    temp=temp|(uint32_t)RxBuffer[0];
    temp=temp<<8;
    temp=temp|(uint32_t)RxBuffer[1];
    temp=temp<<8;
    temp=temp|(uint32_t)RxBuffer[2];
    temp=temp<<8;
    temp=temp|(uint32_t)RxBuffer[3];
 
    temp_AVRMS=temp_AVRMS+temp;
    temp=0;

    count_AVRMS++;

    if(count_AVRMS==100)
       {
        temp_AVRMS=temp_AVRMS/100;
        temp_AVRMS=0;
        count_AVRMS=0;
       }
}
void Calculate_Average_BVRMS(uint8_t RxBuffer[],uint16_t ModbusRegister)
{
    uint32_t temp=0; 
    static uint32_t temp_BVRMS=0;
    static uint32_t count_BVRMS=0;

    temp=temp|(uint32_t)RxBuffer[0];
    temp=temp<<8;
    temp=temp|(uint32_t)RxBuffer[1];
    temp=temp<<8;
    temp=temp|(uint32_t)RxBuffer[2];
    temp=temp<<8;
    temp=temp|(uint32_t)RxBuffer[3];
 
    temp_BVRMS=temp_BVRMS+temp;
    temp=0;

    count_BVRMS++;

    if(count_BVRMS==100)
      {
       temp_BVRMS=temp_BVRMS/100;
       temp_BVRMS=0;
       count_BVRMS=0;
      }
}
void Calculate_Average_CVRMS(uint8_t RxBuffer[],uint16_t ModbusRegister)
{
    uint32_t temp=0; 
    static uint32_t temp_CVRMS=0;
    static uint32_t count_CVRMS=0;

    temp=temp|(uint32_t)RxBuffer[0];
    temp=temp<<8;
    temp=temp|(uint32_t)RxBuffer[1];
    temp=temp<<8;
    temp=temp|(uint32_t)RxBuffer[2];
    temp=temp<<8;
    temp=temp|(uint32_t)RxBuffer[3];
 
    temp_CVRMS=temp_CVRMS+temp;
    temp=0;

    count_CVRMS++;

    if(count_CVRMS==100)
       {
        temp_CVRMS=temp_CVRMS/100;
        temp_CVRMS=0;
        count_CVRMS=0;
       }
}

void Select_PhaseMeasurement(I2C_HandleTypeDef *hi2c,uint16_t RegisterAddress,uint8_t RegisterSize,uint8_t WriteValue,uint32_t Timeout)
{
  uint8_t RxBuffer[2]={0,0};
  uint16_t temp=0;


  if(WriteValue==0x01)
  {
    Read_ADE7858_I2C(&I2c2Handle,RegisterAddress,RegisterSize,&RxBuffer[0],0x02,Timeout);
    
    temp=0;
    temp=RxBuffer[0];
    temp=temp<<8;
    temp=temp|RxBuffer[1];

    temp=temp|1<<9; //set bit 9
    temp&= ~(1 << 10); //clear bit 10

    Write_ADE7858_I2C(&I2c2Handle,COMPMODE,temp,4,1000);
    Read_ADE7858_I2C(&I2c2Handle,COMPMODE,2,&RxBuffer[0],0x02,1000);
   
  }
  else if(WriteValue==0x10)
  {

    Read_ADE7858_I2C(&I2c2Handle,RegisterAddress,RegisterSize,&RxBuffer[0],0x02,Timeout);

    temp=0;
    temp=RxBuffer[0];
    temp=temp<<8;
    temp=temp|RxBuffer[1];

    temp&= ~(1 <<9);//clear bit 09
    temp=temp|1<<10; //set bit 10

    Write_ADE7858_I2C(&I2c2Handle,COMPMODE,temp,4,1000);
    Read_ADE7858_I2C(&I2c2Handle,COMPMODE,2,&RxBuffer[0],0x02,1000);
  }
  else if(WriteValue==0x00)
  {
    Read_ADE7858_I2C(&I2c2Handle,RegisterAddress,RegisterSize,&RxBuffer[0],0x02,Timeout);

    temp=0;
    temp=RxBuffer[0];
    temp=temp<<8;
    temp=temp|RxBuffer[1];

    temp&= ~(1 <<9) ;//clear bit 09
    temp&= ~(1 <<10);//clear bit 09

    Write_ADE7858_I2C(&I2c2Handle,COMPMODE,temp,4,1000);
  }

}
void Select_PeriodMeasurement(I2C_HandleTypeDef *hi2c,uint16_t RegisterAddress,uint8_t RegisterSize,uint8_t WriteValue,uint32_t Timeout)
{

  uint8_t RxBuffer[1]={0};
  uint8_t temp=0;

  if(WriteValue==0x00)
  {
    Read_ADE7858_I2C(&I2c2Handle,RegisterAddress,RegisterSize,&RxBuffer[0],0x01,Timeout);

    temp=0;
    temp=RxBuffer[0];

    temp&= ~(1 <<0) ;//clear bit 0
    temp&= ~(1 <<1) ;//clear bit 1

    Write_ADE7858_I2C(&I2c2Handle,MMODE,temp,3,1000);
  }
  else if(WriteValue==0x01)  //period for phase B
  {
    Read_ADE7858_I2C(&I2c2Handle,RegisterAddress,RegisterSize,&RxBuffer[0],0x01,Timeout);

    temp=0;
    temp=RxBuffer[0];

    temp|= 1 <<0 ;//clear bit 0
    temp&= ~(1 <<1) ;//clear bit 1

    Write_ADE7858_I2C(&I2c2Handle,MMODE,temp,3,1000);
  }
  else  if(WriteValue==0x10) //period for phase C
  {
    Read_ADE7858_I2C(&I2c2Handle,RegisterAddress,RegisterSize,&RxBuffer[0],0x01,Timeout);

    temp=0;
    temp=RxBuffer[0];

    temp&= ~(1 <<0) ;//clear bit 0
    temp|= 1 <<1 ; //set bit one

    Write_ADE7858_I2C(&I2c2Handle,MMODE,temp,3,1000);
  }

}

int Calculate_ApparentPower(uint32_t Phase_VRMS,uint32_t Phase_IRMS,uint16_t Modbusregister)
{
  uint32_t temp=0; 
  float tempfloat1=0; 
  float tempfloat2=0;

  temp=Phase_VRMS*Phase_IRMS;
  tempfloat1=temp*0.001;
  temp=temp*0.001;

  tempfloat2=tempfloat1-temp;
     if(tempfloat2>=0.5)
     {
      temp++;
     }

  return(temp);
}

int Calc_PowerFactor(uint32_t Phase_Apparent_Power,uint32_t Phase_AWATT,uint16_t Modbusregister)
{
  uint32_t temp=0; 
  float tempfloat1=0; 
  float tempfloat2=0;

//   temp=Phase_AWATT; 
//   temp=temp&0x00FFFFFF;
//   temp2=temp*0.00438;               //(max full scale power)/(PMAX/16); max full scale power=VFS*IFS
//   temp2=temp2/Phase_Apparent_Power;
//   temp= temp2*100;   

//   temp=Phase_AWATT/Phase_Apparent_Power;
//   temp= temp*100;

   temp= Phase_AWATT*100;

   temp=temp/Phase_Apparent_Power;
   tempfloat1=temp/Phase_Apparent_Power;
 
   tempfloat2=tempfloat1-temp;
     if(tempfloat2>=0.5)
     {
      temp++;
     }
		 
   return(temp);
 
}
/*********************************************************************************************************
Read Energy meter config registers 
*********************************************************************************************************/

void Read_EMeterConfig_Registers(void)
{
    uint16_t I2CMemAddress;
	
	  HAL_GPIO_WritePin(GPIOA,I2C1_RW , GPIO_PIN_SET);
	 
    //Read VRMS registers 
    I2CMemAddress=14;  
    HAL_I2C_Mem_Read(&I2c2Handle,ADE7858_DevAddress_RD, I2CMemAddress, 2,(uint8_t*)&AVGAIN_DATA,4,100);

    I2CMemAddress=18;  
    HAL_I2C_Mem_Read(&I2c2Handle,ADE7858_DevAddress_RD, I2CMemAddress, 2,(uint8_t*)&BVGAIN_DATA,4,100);
 
    I2CMemAddress=22;  
    HAL_I2C_Mem_Read(&I2c2Handle,ADE7858_DevAddress_RD, I2CMemAddress, 2,(uint8_t*)&CVGAIN_DATA,4,100);

    //Read IRMS registers 
    I2CMemAddress=26;  
    HAL_I2C_Mem_Read(&I2c2Handle,ADE7858_DevAddress_RD, I2CMemAddress, 2,(uint8_t*)&AIGAIN_DATA,4,100);

    I2CMemAddress=30;  
    HAL_I2C_Mem_Read(&I2c2Handle,ADE7858_DevAddress_RD, I2CMemAddress, 2,(uint8_t*)&BIGAIN_DATA,4,100);

    I2CMemAddress=34;  
    HAL_I2C_Mem_Read(&I2c2Handle,ADE7858_DevAddress_RD, I2CMemAddress, 2,(uint8_t*)&CIGAIN_DATA,4,100);

    //Read Active Power registers 
    I2CMemAddress=38;  
    HAL_I2C_Mem_Read(&I2c2Handle,ADE7858_DevAddress_RD, I2CMemAddress, 2,(uint8_t*)&AWGAIN_DATA,4,100);

    I2CMemAddress=42;  
    HAL_I2C_Mem_Read(&I2c2Handle,ADE7858_DevAddress_RD, I2CMemAddress, 2,(uint8_t*)&BWGAIN_DATA,4,100);

    I2CMemAddress=46;  
    HAL_I2C_Mem_Read(&I2c2Handle,ADE7858_DevAddress_RD, I2CMemAddress, 2,(uint8_t*)&CWGAIN_DATA,4,100);

    I2CMemAddress=50;  
    HAL_I2C_Mem_Read(&I2c2Handle,ADE7858_DevAddress_RD, I2CMemAddress, 2,(uint8_t*)&APHCAL_DATA,2,100);

    I2CMemAddress=52;  
    HAL_I2C_Mem_Read(&I2c2Handle,ADE7858_DevAddress_RD, I2CMemAddress, 2,(uint8_t*)&BPHCAL_DATA,2,100);

    I2CMemAddress=54;  
    HAL_I2C_Mem_Read(&I2c2Handle,ADE7858_DevAddress_RD, I2CMemAddress, 2,(uint8_t*)&CPHCAL_DATA,2,100);

}
/*********************************************************************************************************
Round OFF Function
*********************************************************************************************************/
uint32_t roundoff(uint32_t product,uint32_t multiplier)
{
   double tempfloat1=0; 
   double tempfloat2=0;

   product=multiplier/product;  //frequency
   tempfloat1=(double)multiplier/(double)product;     //frequency


   tempfloat2=tempfloat1-(double)product;

     if(tempfloat2>=0.5)
     {
      product++;
     }

    return(product);
}
/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
 
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* Enable both GPIO clocks */ 
	__GPIOA_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
	
	
  GPIO_InitStructure.Pin = ADE7858_RESETB;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
    
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.Pin = I2C1_RW;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
	
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
}