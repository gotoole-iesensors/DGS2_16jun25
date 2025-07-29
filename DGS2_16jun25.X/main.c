/**
  Generated main.c file from MPLAB Code Configurator

  @Company
    Microchip Technology Inc.

  @File Name
    main.c

  @Summary
    This is the generated main.c using PIC24 / dsPIC33 / PIC32MM MCUs.

  @Description
    This source file provides main entry point for system initialization and application code development.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.171.1
        Device            :  PIC24F16KM202
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.35
        MPLAB 	          :  MPLAB X v5.50
*/

/*
    (c) 2020 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

/**
  Section: Included Files
*/
#include "mcc_generated_files/system.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define String_Firmware_Revision_Date "16JUN25"                                 
#define Si7021_SHT20_HTU21D
//#define STS21
//#define No_TPRH_Sensor
#define Rgain_Fixed_Value 512E3                                                 //Used for external resistor
#define Experimental_Function_Info
//#define Experimental_Function_BiasBump
//#define Experimental_Function_CeReDisconnect
//#define Experimental_Function_BiasStaircases
//#define Experimental_Function_BiasAdjustTo0nA

#define SCL2        TRISBbits.TRISB14
#define SDA2        TRISBbits.TRISB15
#define SCL2_IN     PORTBbits.RB14
#define SDA2_IN     PORTBbits.RB15
#define SCL2_OUT    LATBbits.LATB14
#define SDA2_OUT    LATBbits.LATB15
#define AD5941_SEL  LATBbits.LATB12
#define SPI_SCLK    LATBbits.LATB5
#define SPI_MOSI    LATBbits.LATB6
#define SPI_MISO    PORTAbits.RA7

char Buffer[100];
int __attribute__ ((space(eedata))) eeData; 
char uart_receive_buffer_[128];
unsigned int GLOBAL_EEPROM_Unlocked;

void writeNVM(int addr, int newData)
{
    unsigned int offset;
    NVMCON = 0x4004;
                                                                                // Set up a pointer to the EEPROM location to be erased
    TBLPAG = __builtin_tblpage(&eeData);                                        // Initialize EE Data page pointer
    offset = __builtin_tbloffset(&eeData) + (addr & 0x01ff);                    // Initizlize lower word of address
    __builtin_tblwtl(offset, newData);                                          // Write EEPROM data to write latch
    asm volatile ("disi #5");                                                   // Disable Interrupts For 5 Instructions
    __builtin_write_NVM();                                                      // Issue Unlock Sequence & Start Write Cycle
    while(NVMCONbits.WR == 1);                                                    // Optional: Poll WR bit to wait for write sequence to complete
    DELAY_milliseconds(10);
}

int readNVM(int addr)
{
    int data; 
    unsigned int offset;
                                                                                // Set up a pointer to the EEPROM location to be erased
    TBLPAG = __builtin_tblpage(&eeData);                                        // Initialize EE Data page pointer
    offset = __builtin_tbloffset(&eeData) + (addr & 0x01ff);                    // Initizlize lower word of address
    data = __builtin_tblrdl(offset);                                            // Write EEPROM data to write latch
    DELAY_milliseconds(1);
    return data;
}

void UART2_xmitc(char UART_char_input)
{
    while(!IFS1bits.U2TXIF);
    U2TXREG = UART_char_input;
    DELAY_milliseconds(1);
}

void UART2_xmits(char* UART_string_input)
{
    while(*UART_string_input != 0)
    {
        UART2_xmitc(*UART_string_input++);
    }
}

void UART_Entry(unsigned int Echo_Enabled)
{
    unsigned int i;
    char user_input;
    
    ClrWdt();
    i = 0;
    while(i < 128)
    {
        uart_receive_buffer_[i] = '\0';
        i++;
    }

    i = 0;
    while(i < 128)
    {    
        if(IFS1bits.U2RXIF)
        {            
            IFS1bits.U2RXIF = 0;
            user_input = U2RXREG;
            if(Echo_Enabled)
            {
                UART2_xmitc(user_input);
            }
            if(user_input == '\r')
            {
                break;
            }
            else
            {
                uart_receive_buffer_[i] = user_input;
            }

            i++;
        }
    }
}

void Unlock(void)
{
    unsigned int i;
    char user_input;

    if(GLOBAL_EEPROM_Unlocked)
    {
        printf("L");
    }
    else
    {
        printf("Unl");
    }
    printf("ock EEPROM?: ");

    i = 0;
    while(i < 1)
    {
        if(IFS1bits.U2RXIF)
        {
            IFS1bits.U2RXIF = 0;
            user_input = U2RXREG;
            i++;
        }
    }

    if(user_input == 'Y')
    {
        if(GLOBAL_EEPROM_Unlocked)
        {
            GLOBAL_EEPROM_Unlocked = 0x0000;
        }
        else
        {
            GLOBAL_EEPROM_Unlocked = 0xFFFF;
        } 
        writeNVM(0xCE, GLOBAL_EEPROM_Unlocked);
        printf("Success\r\n");
    }
    else
    {
        printf("Fail\r\n");
        DELAY_milliseconds(10);
    }
}

void Asterix_Separator(unsigned int size, char NewLine)
{
    unsigned int i;
    
    i = 0;
    while(i < size)
    {
        UART2_xmitc('*');
        while(!U2STAbits.TRMT);
        
        i++;
    }
    if(NewLine)
    {
        printf("\r\n");
    }
}

void I2C2_dly(void)
{
    int dly;
    for (dly = 0; dly < 10; dly++)
    {
        Nop();
    }
}

void I2C2_init(void)
{
    SDA2 = 1;
    SCL2 = 1;
    SCL2_OUT = 0;
    SDA2_OUT = 0;
}

void I2C2_start(void)
{
    SDA2 = 1;
    I2C2_dly();
    SCL2 = 1;
    I2C2_dly();
    SDA2 = 0;
    I2C2_dly();
    SCL2 = 0;
    I2C2_dly();
}

void I2C2_stop(void)
{
    I2C2_dly();
    SDA2 = 0;
    I2C2_dly();
    SCL2 = 1;
    I2C2_dly();
    SDA2 = 1;
    I2C2_dly();
}

char I2C2_tx(unsigned char d)
{
    char x;
    static char b;

    SCL2 = 0;
    for(x=8; x; x--)
    {
        if(d&0x80)
        SDA2 = 1;
        else
        SDA2 = 0;
        I2C2_dly();
        SCL2 = 1;
        d <<= 1;
        I2C2_dly();
        SCL2 = 0;
    }
    SDA2 = 1;
    I2C2_dly();
    SCL2 = 1;
    I2C2_dly();
    b = SDA2_IN;
    SCL2 = 0;
    return b;
}

unsigned char I2C2_rx(char ack)
{
    char x, d=0;
    SDA2 = 1;
    SCL2 = 0;
    for(x=0; x<8; x++)
    {
        I2C2_dly();
        d <<= 1;
        do
        {
            SCL2 = 1;
        }while(SCL2_IN==0);
        I2C2_dly();
        if(SDA2_IN) d |= 1;
        SCL2 = 0;
    }
    if(ack) SDA2 = 0;
    else SDA2 = 1;
    I2C2_dly();
    SCL2 = 1;
    I2C2_dly();
    SCL2 = 0;
    SDA2 = 1;
    return d;
}

void I2C2_reset(void)
{
    char i;

    SDA2 = 1;
    for(i=0; i<9; i++)
    {
        SCL2 = 1;
        I2C2_dly();
        SCL2 = 0;
        I2C2_dly();
        if(SDA2_IN) break;
    }
    I2C2_stop();
}

unsigned int STS21_Read_T(void)
{
    unsigned int T;

    I2C2_start();
    while(I2C2_tx(0b10010100));
    while(I2C2_tx(0xE3));
    I2C2_stop();I2C2_start();
    while(I2C2_tx(0b10010101));
    T = I2C2_rx(1);
    T = T << 8;
    T |= I2C2_rx(0);
    I2C2_stop();
    T = T & 0xFFFF;

    return T;
}

unsigned int Si7021_SHT20_HTU21D_Read_T(void)
{
    unsigned int T;

    I2C2_start();
    while(I2C2_tx(0b10000000));
    while(I2C2_tx(0xE3));
    I2C2_stop();I2C2_start();
    while(I2C2_tx(0b10000001));
    T = I2C2_rx(1);
    T = T << 8;
    T |= I2C2_rx(0);
    I2C2_stop();
    T = T & 0xFFFF;

    return T;
}

double Si7021_SHT20_HTU21D_STS21_Calculate_T(unsigned int T, int Default_1000x_Temperature_Offset)
{
    double T_float;

    T_float = ((float)T * 175.72 / 65536) - 46.85;
    T_float = T_float + ((double)Default_1000x_Temperature_Offset / 1000);

    return T_float;
}

unsigned int Si7021_SHT20_HTU21D_Read_RH(void)
{
    unsigned int RH;

    I2C2_start();
    while(I2C2_tx(0b10000000));
    while(I2C2_tx(0xE5));
    I2C2_stop();I2C2_start();
    while(I2C2_tx(0b10000001));
    RH = I2C2_rx(1);
    RH = RH << 8;
    RH |= I2C2_rx(0);
    I2C2_stop();
    RH = RH & 0xFFFF;

    return RH;
}

double Si7021_SHT20_HTU21D_Calculate_RH(unsigned int RH, int Default_1000x_Humidity_Offset)
{
    double RH_float;

    RH_float = ((float)RH * 125 / 65536) - 6;
    RH_float = RH_float + ((double)Default_1000x_Humidity_Offset / 1000);
    if(RH_float > 100)
    {
        RH_float = 100;
    }
    else if(RH_float < 0)
    {
        RH_float = 0;
    }
    else;

    return RH_float;
}

void ADC_Dummy_Read(void)
{
    AD1CON1bits.SAMP = 1;
    while (!AD1CON1bits.DONE){};
}

void ADC_Enable(void)
{
    AD1CON1bits.ADON = 1;
}

void ADC_Disable(void)
{
    AD1CON1bits.ADON = 0;
}

void ADC_Set_Channels(unsigned int channel_P, unsigned int channel_N)
{    
    AD1CHSbits.CH0SA = channel_P;
    AD1CHSbits.CH0NA = (channel_N);                                             //channel_N0 = Vss
    AD1CHSbits.CH0SB = channel_P;
    AD1CHSbits.CH0NB = (channel_N);                                             //channel_N0 = Vss
}

int ADC_Read(void)
{
    int ADC_Result;
    
    AD1CON1bits.SAMP = 1;
    while(!AD1CON1bits.DONE){};
    ADC_Result = ADC1BUF0;

    return ADC_Result;
}

void AD5941_write_ADDR(unsigned int address)
{
	unsigned int i;
    unsigned int SPICMD_SETADDR = 0x20;
	unsigned int temp_data;
	
    SPI_SCLK = 0;
    AD5941_SEL = 0;
    
    i = 8;
	while(i > 0)
	{
		i--;

		temp_data = SPICMD_SETADDR >> i;
		temp_data = temp_data & 0x0001;
		SPI_MOSI = temp_data;
		SPI_SCLK = 1;
		SPI_SCLK = 0;		
	}
    
    i = 16;
	while(i > 0)
	{
		i--;

		temp_data = address >> i;
		temp_data = temp_data & 0x0001;
		SPI_MOSI = temp_data;
		SPI_SCLK = 1;
		SPI_SCLK = 0;
	}
	
	AD5941_SEL = 1;
}

void AD5941_write_DATA_16b_(unsigned int data)
{
	unsigned int i;
    unsigned int SPICMD_WRITEREG = 0x2D;
	unsigned int temp_data;
    
    SPI_SCLK = 0;
    AD5941_SEL = 0;
    
    i = 8;
	while(i > 0)
	{
		i--;

		temp_data = SPICMD_WRITEREG >> i;
		temp_data = temp_data & 0x0001;
		SPI_MOSI = temp_data;
		SPI_SCLK = 1;
		SPI_SCLK = 0;		
	}
    
    i = 16;
	while(i > 0)
	{
		i--;

		temp_data = data >> i;
		temp_data = temp_data & 0x0001;
		SPI_MOSI = temp_data;
		SPI_SCLK = 1;
		SPI_SCLK = 0;		
	}
	
	AD5941_SEL = 1;
}

void AD5941_write_DATA_32b_(unsigned long int data)
{
	unsigned int i;
    unsigned int SPICMD_WRITEREG = 0x2D;
	unsigned long int temp_data;
    
    SPI_SCLK = 0;
    AD5941_SEL = 0;
    
    i = 8;
	while(i > 0)
	{
		i--;

		temp_data = SPICMD_WRITEREG >> i;
		temp_data = temp_data & 0x00000001;
		SPI_MOSI = temp_data;
		SPI_SCLK = 1;
		SPI_SCLK = 0;		
	}
    
    i = 32;
	while(i > 0)
	{
		i--;

		temp_data = data >> i;
		temp_data = temp_data & 0x00000001;
		SPI_MOSI = temp_data;
		SPI_SCLK = 1;
		SPI_SCLK = 0;		
	}
	
	AD5941_SEL = 1;
}

unsigned int AD5941_read_DATA_16b_(void)
{
	unsigned int i;
    unsigned int SPICMD_READREG = 0x6D;
	unsigned int data_out;
	unsigned int temp_data;
	
    SPI_SCLK = 0;
	AD5941_SEL = 0;
	
	i = 8;
	while(i > 0)
	{
		i--;

		temp_data = SPICMD_READREG >> i;
		temp_data = temp_data & 0x0001;
		SPI_MOSI = temp_data;
        SPI_SCLK = 1;
        SPI_SCLK = 0;	
	} 
    i = 8;
	while(i > 0)
	{
		i--;

		temp_data = 0xFF >> i;
		temp_data = temp_data & 0x0001;
		SPI_MOSI = temp_data;
        SPI_SCLK = 1;
        SPI_SCLK = 0;	
	}
	data_out = 0;
	i = 0;
	while(i < 16)
	{
        SPI_SCLK = 1;
        data_out = data_out << 1;
		data_out = data_out | SPI_MISO;
        SPI_SCLK = 0;	
			
		i++;
	}
	
    AD5941_SEL = 1;
	
	return data_out;
}

unsigned long int AD5941_read_DATA_32b_(void)
{
	unsigned int i;
    unsigned int SPICMD_READREG = 0x6D;
	unsigned long int data_out;
	unsigned int temp_data;
	
    SPI_SCLK = 0;
	AD5941_SEL = 0;
	
	i = 8;
	while(i > 0)
	{
		i--;

		temp_data = SPICMD_READREG >> i;
		temp_data = temp_data & 0x0001;
		SPI_MOSI = temp_data;
        SPI_SCLK = 1;
        SPI_SCLK = 0;	
	} 
    i = 8;
	while(i > 0)
	{
		i--;

		temp_data = 0xFF >> i;
		temp_data = temp_data & 0x0001;
		SPI_MOSI = temp_data;
        SPI_SCLK = 1;
        SPI_SCLK = 0;	
	}
	data_out = 0;
	i = 0;
	while(i < 32)
	{
        SPI_SCLK = 1;
        data_out = data_out << 1;
		data_out = data_out | SPI_MISO;
        SPI_SCLK = 0;	
			
		i++;
	}
	
    AD5941_SEL = 1;
	
	return data_out;
}

unsigned long int Rgain_Calc(unsigned long int LPTIACON0)
{
    unsigned long int Rgain;
       
    Rgain = (LPTIACON0 >> 5) & 0x0000001F;
    if(Rgain == 26)
    {
        Rgain = 512E3;
    }
    else if(Rgain == 25)
    {
        Rgain = 256E3;
    }
    else if(Rgain == 24)
    {
        Rgain = 196E3;
    }
    else if(Rgain == 23)
    {
        Rgain = 160E3;
    }
    else if(Rgain == 22)
    {
        Rgain = 128E3;
    }
    else if(Rgain == 21)
    {
        Rgain = 120E3;
    }
    else if(Rgain == 20)
    {
        Rgain = 100E3;
    }
    else if(Rgain == 19)
    {
        Rgain = 96E3;
    }
    else if(Rgain == 18)
    {
        Rgain = 85E3;
    }
    else if(Rgain == 17)
    {
        Rgain = 64E3;
    }
    else if(Rgain == 16)
    {
        Rgain = 48E3;
    }
    else if(Rgain == 15)
    {
        Rgain = 40E3;
    }
    else if(Rgain == 14)
    {
        Rgain = 32E3;
    }
    else if(Rgain == 13)
    {
        Rgain = 30E3;
    }
    else if(Rgain == 12)
    {
        Rgain = 24E3;
    }
    else if(Rgain == 11)
    {
        Rgain = 20E3;
    }
    else if(Rgain == 10)
    {
        Rgain = 16E3;
    }
    else if(Rgain == 9)
    {
        Rgain = 12E3;
    }
    else if(Rgain == 8)
    {
        Rgain = 10E3;
    }
    else if(Rgain == 7)
    {
        Rgain = 8E3;
    }
    else if(Rgain == 6)
    {
        Rgain = 6E3;
    }
    else if(Rgain == 5)
    {
        Rgain = 4E3;
    }
    else if(Rgain == 4)
    {
        Rgain = 3E3;
    }
    else if(Rgain == 3)
    {
        Rgain = 2E3;
    }
    else if(Rgain == 2)
    {
        Rgain = 1E3;
    }
    else if(Rgain == 1)
    {
        Rgain = 200;
    }
    else
    {
        Rgain = Rgain_Fixed_Value;                                              //DGS2 has Rgain (R1 + C9) unpopulated
    }
    
    return Rgain;
}

unsigned long int Read_AD5941_ADC(void)
{
    unsigned long int AD5941_Result_32bit;
    
    AD5941_write_ADDR(0x2000);                                                  //AFECON
    AD5941_write_DATA_32b_(0x00390180);                                         //Trigger sample
    DELAY_milliseconds(2);                                                      //Wait 2ms for sample (900SPS setting in use)
    AD5941_write_ADDR(0x2080);                                                  //SINC2DAT
    AD5941_Result_32bit = AD5941_read_DATA_32b_();                              //Read sample from SINC2 reg and add to summation 
    
    return AD5941_Result_32bit;
}

int main(void)
{
    unsigned long int i;
    unsigned long int j;
    unsigned long int k;
    char uart_command;
    char uart_receive_dummy;
    int ADC_Average;
    unsigned int Output_On_Demand;
    unsigned long int Time_On_Counter;
    unsigned int DGS2_Output_Enabled;
    unsigned int Open_Circuit_Adjusted = 0x0000;
    
    long int nA_per_PPM_x100; 
    
    unsigned long int AD5941_Result_32bit;
    unsigned long int AD5941_Result_16bit_Summation;
    unsigned int AD5941_Result_16bit;
    unsigned int AD5941_Result_16bit_[256];
    unsigned int AD5941_Result_16bit_OC;
    unsigned int AD5941_Result_16bit_Zero;
    unsigned int AD5941_Result_16bit_Span;
    double Vout;
    double Vzero;
    double Voc;
    double Zcf;
    double nA;
    double PPB;
    double Rgain;
    
    unsigned int RE0_Vbias_Cap;
    unsigned int SE0_Vbias_Cap;
    long int Bias_uV;
    unsigned int Ce_Re_Connected;
    
    unsigned int Bias_Bump_Time;
    
    unsigned long int LPDACDAT0;
    unsigned long int LPDACDAT0_Setting;
    unsigned long int LPDACDAT0_Setting_New;
    unsigned long int LPTIACON0_Setting;
    unsigned long int LPTIACON0_Setting_New;
    
    unsigned int TPRH_Counter;
    unsigned int T_Raw = 25000;
    unsigned int T_Raw_OC;
    unsigned int T_Raw_Zero;
    unsigned int T_Raw_Span;
    double T_float = 20;
    double T_OC_float;
    double T_Zero_float;
    int T_x100 = 2000;
    int Temperature_Offset_x1000; 
    unsigned int n_factor_x10;
    double n_factor;
    
    unsigned int H_Raw = 29500;
    unsigned int H_Raw_OC;
    unsigned int H_Raw_Zero;
    unsigned int H_Raw_Span;
    double H_float = 50;
    double H_Zero_float;
    unsigned int H_x100 = 5000;
    int Humidity_Offset_x1000;
 
    char Info_[129];
    char Module_Barcode_[21];
    char Barcode_[65];
    char Serial_Number_[13];
    char Part_Number_[7];
    long int Part_Number;
    
    unsigned int DAYS;
    unsigned int HOURS;
    unsigned int MINUTES;
    unsigned int SECONDS;

    SYSTEM_Initialize();
    RCONbits.SWDTEN = 1;   
    I2C2_init();   
    DELAY_milliseconds(750);
    
    if((RCONbits.BOR) || (RCONbits.POR))                                        //Clear BOR+POR flags and init Time_On_Counter 
    {
        RCONbits.BOR = 0; 
        RCONbits.POR = 0;
        
        Time_On_Counter = 0;
    }
    
    i = 0;                                                                      //Begin EEPROM read and setup
    while(i < 64)
    {
        Barcode_[i] = readNVM(0x0000 + (i*2));
        i++;
    }
    Barcode_[64] = '\0';
    i = 0;
    while(i < 12)
    {
        Serial_Number_[i] = Barcode_[i];
        i++;
    }
    Serial_Number_[12] = '\0';
    i = 13;
    while(i < 19)
    {
        Part_Number_[i - 13] = Barcode_[i];
        i++;
    } 
    Part_Number_[i - 13] = '\0'; 
    Part_Number = atol(Part_Number_);
    
    LPDACDAT0_Setting               = (unsigned int)readNVM(0x80);                            
    LPDACDAT0_Setting               = LPDACDAT0_Setting << 16;                  
    LPDACDAT0_Setting               = LPDACDAT0_Setting | (unsigned int)readNVM(0x82);  
    if((LPDACDAT0_Setting == 0x00000000) || (LPDACDAT0_Setting == 0xFFFFFFFF))
    {
        LPDACDAT0_Setting = 0x0001F7BF;       
    }   
    LPTIACON0_Setting               = (unsigned int)readNVM(0x84);                            
    LPTIACON0_Setting               = LPTIACON0_Setting << 16;                  
    LPTIACON0_Setting               = LPTIACON0_Setting | (unsigned int)readNVM(0x86); 
    if((LPTIACON0_Setting == 0x00000000) || (LPTIACON0_Setting == 0xFFFFFFFF))
    {
        LPTIACON0_Setting = 0x00002740;                                         //0=Rlpf, 10=Rload, Rtia = 512k
    }
          
    AD5941_Result_16bit_OC          = readNVM(0x90);           
    AD5941_Result_16bit_Zero        = readNVM(0x94);
    AD5941_Result_16bit_Span        = readNVM(0x98);
    nA_per_PPM_x100                 = (((unsigned long int)readNVM(0x009C)) << 16) | (readNVM(0x009E) & (0x0000FFFF));
//    nA_per_PPM_x100                 = ((unsigned long int)readNVM(0x9C) * 65536) + (unsigned long int)readNVM(0x9E);
//    S1C2F_S1C1F_Zero    = (((unsigned long int)readNVM(0x0114)) << 16) | (readNVM(0x0116) & (0x0000FFFF));
    
    T_Raw_OC                        = readNVM(0xA0);
    T_Raw_Zero                      = readNVM(0xA4);
    T_Raw_Span                      = readNVM(0xA8);
    Temperature_Offset_x1000        = readNVM(0xAC); 
    T_Zero_float = Si7021_SHT20_HTU21D_STS21_Calculate_T(T_Raw_Zero, Temperature_Offset_x1000);
    n_factor_x10                    = readNVM(0xAE);

    H_Raw_OC                        = readNVM(0xB0);
    H_Raw_Zero                      = readNVM(0xB4);
    H_Raw_Span                      = readNVM(0xB8); 
    Humidity_Offset_x1000           = readNVM(0xBC); 
	H_Zero_float = Si7021_SHT20_HTU21D_Calculate_RH(H_Raw_Zero, Humidity_Offset_x1000);

    GLOBAL_EEPROM_Unlocked          = readNVM(0xCE);
    
	ADC_Average                     = readNVM(0xD0); 
    if(ADC_Average < 1)
    {
        ADC_Average = 1;
    }
    if(ADC_Average > 256)
    {
        ADC_Average = 256;
    }
    Output_On_Demand                = readNVM(0xD2);
    DGS2_Output_Enabled             = readNVM(0xD4);
    Ce_Re_Connected                 = readNVM(0xD6);
    i = 0;
    while(i < 20)
    {
        Module_Barcode_[i] = readNVM(0x00D8 + (i*2));
        i++;
    }
    Module_Barcode_[20] = '\0';
    i = 0;
    while(i < 128)
    {
        Info_[i] = readNVM(0x0100 + (i*2));
        i++;
    }
    Info_[128] = '\0';
    
/*Mandatory AD5941 init*/    
    AD5941_write_ADDR(0x0908);AD5941_write_DATA_16b_(0x02C9);
    AD5941_write_ADDR(0x0C08);AD5941_write_DATA_16b_(0x206C);
    AD5941_write_ADDR(0x21F0);AD5941_write_DATA_16b_(0x0010);
    AD5941_write_ADDR(0x0410);AD5941_write_DATA_16b_(0x02C9);
    AD5941_write_ADDR(0x0A28);AD5941_write_DATA_16b_(0x0009);
    AD5941_write_ADDR(0x238C);AD5941_write_DATA_16b_(0x0104);
    AD5941_write_ADDR(0x0A04);AD5941_write_DATA_16b_(0x4859);
    AD5941_write_ADDR(0x0A04);AD5941_write_DATA_16b_(0xF27B);
    AD5941_write_ADDR(0x0A00);AD5941_write_DATA_16b_(0x8009);
    AD5941_write_ADDR(0x22F0);AD5941_write_DATA_16b_(0x0000);    
    DELAY_milliseconds(250);
 
/*Custom AD5941 init*/    
    AD5941_write_ADDR(0x20E4);                                                  //LPTIASW0
    if(Ce_Re_Connected)
    {
        AD5941_write_DATA_32b_(0x0000000C);
        
    }
    else
    {
        AD5941_write_DATA_32b_(0x00000000);
    } 
//    AD5941_write_ADDR(0x20E4);                                                  //LPTIASW0
//    AD5941_write_DATA_32b_(0x0000000C);                                         //Vbias + Vzero pins unconnected                                                                        
    AD5941_write_ADDR(0x2128);                                                  //LPDACCON0
    AD5941_write_DATA_32b_(0x00000001);
    AD5941_write_ADDR(0x2124);                                                  //LPDACSW0
    AD5941_write_DATA_32b_(0x00000034);                                         //Vbias + Vzero pins unconnected    
    AD5941_write_ADDR(0x238C);                                                  //ADCBUFCON
    AD5941_write_DATA_32b_(0x005F3D0F); 
    AD5941_write_ADDR(0x2180);                                                  //ADCBUFSENCON
    AD5941_write_DATA_32b_(0x00000113);
    AD5941_write_ADDR(0x2044);                                                  //ADCFILTERCON
    AD5941_write_DATA_32b_(0x00000301);        
    AD5941_write_ADDR(0x21A8);                                                  //ADCCON 
    AD5941_write_DATA_32b_(0x00000214);                                         //AIN4 - Low power TIA negative input

    AD5941_write_ADDR(0x2120);                                                  //LPDACDAT0
    AD5941_write_DATA_32b_(LPDACDAT0_Setting);
    AD5941_write_ADDR(0x20EC);                                                  //LPTIACON0
    AD5941_write_DATA_32b_(LPTIACON0_Setting);
    
    i = 0;
    while(i < 256)                                                              //Init running average window at 32k
    {
        AD5941_Result_16bit_[i] = AD5941_Result_16bit_Zero;
        i++;
    }
         
    uart_command = '\0';                                                                            
    TPRH_Counter = 0;
    AD5941_Result_32bit = 0;
    while (1)
    {
        if(Ce_Re_Connected)
        {
            AD5941_Result_32bit = AD5941_Result_32bit + Read_AD5941_ADC();       
            TPRH_Counter++;        
        }
        else
        {
            AD5941_write_ADDR(0x21A8);                                          //ADCCON 
            AD5941_write_DATA_32b_(0x0000081A);                                 //RE0 - Vbias_Cap
            DELAY_milliseconds(100);
            RE0_Vbias_Cap = Read_AD5941_ADC();    

            AD5941_write_ADDR(0x21A8);                                          //ADCCON 
            AD5941_write_DATA_32b_(0x0000080E);                                 //SE0 - Vbias_Cap
            DELAY_milliseconds(100);     
            SE0_Vbias_Cap = Read_AD5941_ADC();
            
            TPRH_Counter = TPRH_Counter + 64;
        }        
//        AD5941_Result_32bit = AD5941_Result_32bit + Read_AD5941_ADC();       
//        TPRH_Counter++;

        if(TPRH_Counter >= 256)                                                 //After 256 samples, average for a 1s reading
        {
            TPRH_Counter = 0;
            Time_On_Counter++;
            
            AD5941_write_ADDR(0x2120);                                          //LPDACDAT0
            LPDACDAT0 = AD5941_read_DATA_32b_();
            if(LPDACDAT0 == 0)                                                  //Check if AD5941 is initialised
            {
                __asm__ volatile ("RESET");
                while(1);
            }
            
            i = 255;
            while(i > 0)                                                        //Enter 1s reading into running average window (1-256)
            {
                AD5941_Result_16bit_[i] = AD5941_Result_16bit_[i - 1];
                i--;
            }
            
            if(Open_Circuit_Adjusted)
            {
                Open_Circuit_Adjusted = 0x0000;
                AD5941_Result_16bit_[0] = AD5941_Result_16bit_[1];
            }
            else
            {
                AD5941_Result_16bit_[0] = AD5941_Result_32bit >> 8;   
            }
            AD5941_Result_32bit = 0;
            AD5941_Result_16bit_Summation = 0;                                                        
            i = 0;
            while(i < ADC_Average)
            {
                AD5941_Result_16bit_Summation = AD5941_Result_16bit_Summation + AD5941_Result_16bit_[i];
                i++;
            }
            AD5941_Result_16bit = AD5941_Result_16bit_Summation / ADC_Average;

#ifdef Si7021_SHT20_HTU21D                                                      //Take T&RH reading
            T_Raw = (unsigned long int)Si7021_SHT20_HTU21D_Read_T();            
            H_Raw = Si7021_SHT20_HTU21D_Read_RH();
            T_float = Si7021_SHT20_HTU21D_STS21_Calculate_T((unsigned int)T_Raw, Temperature_Offset_x1000);
            H_float = Si7021_SHT20_HTU21D_Calculate_RH(H_Raw, Humidity_Offset_x1000);
            T_x100 = 100 * T_float;
            H_x100 = 100 * H_float;   
#endif  
#ifdef STS21                                                                    //Take T reading
            T_Raw = (unsigned long int)STS21_Read_T();                          
            T_float = Si7021_SHT20_HTU21D_STS21_Calculate_T((unsigned int)T_Raw, Temperature_Offset_x1000);
            H_Raw = 26500;
            H_float = 50;
            T_x100 = 100 * T_float;
            H_x100 = 100 * H_float;
#endif  
#ifdef No_TPRH_Sensor                                                           //Skip T&RH reading
            T_Raw = 25000;
            H_Raw = 26500;
            T_x100 = 2350;
            H_x100 = 5500;
#endif

            if(Ce_Re_Connected)
            {
                Rgain = Rgain_Calc(LPTIACON0_Setting);
                Vout = ((double)AD5941_Result_16bit - 32768) / 32768 * 1.82;        //AIN4 - Low power TIA negative input
                Vzero = ((double)AD5941_Result_16bit_Zero - 32768) / 32768 * 1.82;  //AIN4 - Low power TIA negative input
                Voc = ((double)AD5941_Result_16bit_OC - 32768) / 32768 * 1.82;      //AIN4 - Low power TIA negative input
                nA = ((double)Vout - (double)Voc) / Rgain * (double)1E9;
                Zcf = ((double)Vzero - (double)Voc) / Rgain * (double)1E9;
                n_factor = (double)n_factor_x10 / 10;
                Zcf = Zcf * exp((T_float - T_Zero_float) / (double)n_factor);       
                PPB = ((double)nA - (double)Zcf) / (double)nA_per_PPM_x100 * (double)1E2 * (double)1E3;
            }
            else
            {
                Bias_uV = ((double)SE0_Vbias_Cap - (double)RE0_Vbias_Cap) / 32768 * 1.82 * 1E6;
                PPB = Bias_uV;
            }

            if(!Output_On_Demand)                                               //If 'C' selected,'\r' entered into UART input buffer                                                  
            {
                uart_command = '\r';
            }
        }

        if(IFS1bits.U2RXIF)                                                     //Poll U2RXIF for UART entry
        {
            uart_command = U2RXREG;
            
            while(U2STAbits.URXDA)
            {
                i = 0;
                while(i < 1)
                {
                    uart_receive_dummy = U2RXREG;
                    i++;
                }
            }
            IFS1bits.U2RXIF = 0;
            U2STAbits.OERR = 0;              
        } 
        if(uart_command == '\r')                                                //Single read
        {   
            if(DGS2_Output_Enabled)
            {
                printf("%s, %ld, %d, %u, %u, %u, %u \r\n", Serial_Number_, (long int)PPB, T_x100, H_x100, AD5941_Result_16bit, T_Raw, H_Raw);
//                printf("%s, %ld, %d, %u, %u, %u, %u, %lu \r\n", Serial_Number_, (long int)PPB, T_x100, H_x100, AD5941_Result_16bit, T_Raw, H_Raw, (unsigned long int)Rgain);
                
//                printf("%s, %ld, %d, %u, %u, %u, %u, ", Serial_Number_, (long int)PPB, T_x100, H_x100, AD5941_Result_16bit, T_Raw, H_Raw);
//                i = 0;
//                while(i < 5)
//                {
//                    printf("%u, ", AD5941_Result_16bit_[i]);
//                    i++;
//                }
//                printf("\r\n");
            }
            else
            {
                DAYS = (double)Time_On_Counter / 86400;
                HOURS = ((double)Time_On_Counter - ((double)DAYS * 86400)) / 3600;
                MINUTES = ((double)Time_On_Counter - ((double)DAYS * 86400) - ((double)HOURS * 3600)) / 60;
                SECONDS = ((double)Time_On_Counter - ((double)DAYS * 86400) - ((double)HOURS * 3600) - ((double)MINUTES * 60));
                printf("%s, %ld, %d, %u, %u, %u, %u, %02u, %02u, %02u, %02u \r\n", Serial_Number_, (long int)PPB, (T_x100 /100), (H_x100 / 100), AD5941_Result_16bit, T_Raw, H_Raw, DAYS, HOURS, MINUTES, SECONDS);
            }                 
        }
        if((uart_command == 'L') && (GLOBAL_EEPROM_Unlocked))                   //Legacy DGS output toggle
        {            
            if(DGS2_Output_Enabled)
            {
                DGS2_Output_Enabled = 0x0000;
            }
            else
            {
                DGS2_Output_Enabled = 0xFFFF;
            }
            writeNVM(0xD4, DGS2_Output_Enabled); 
        }  
        if(uart_command == 'h')
        {
            if(DGS2_Output_Enabled)
            {
                printf("Serial, PPB, Tx100, Hx100, ADC_Raw, T_Raw, H_Raw \r\n");  
            }
            else
            {
                printf("Serial, PPB, T, H, ADC_Raw, T_Raw, H_Raw, D, H, M, S \r\n");  
            }
        }
        if((uart_command == 'e')||(uart_command == '^')||(uart_command == 'P')) //EEPROM Output
        {            
            AD5941_write_ADDR(0x21A8);                                          //ADCCON 
            AD5941_write_DATA_32b_(0x0000081A);                                 //RE0 - Vbias_Cap
            DELAY_milliseconds(100);
            RE0_Vbias_Cap = Read_AD5941_ADC();    
            AD5941_write_ADDR(0x21A8);                                          //ADCCON 
            AD5941_write_DATA_32b_(0x0000080E);                                 //SE0 - Vbias_Cap
            DELAY_milliseconds(100);     
            SE0_Vbias_Cap = Read_AD5941_ADC();
            Bias_uV = ((double)SE0_Vbias_Cap - (double)RE0_Vbias_Cap) / 32768 * 1.82 * 1E6;
            AD5941_write_ADDR(0x21A8);                                          //ADCCON 
            AD5941_write_DATA_32b_(0x00000214);                                 //AIN4 - Low power TIA negative input
            DELAY_milliseconds(100);
            
            Asterix_Separator(25, 0);
            if(GLOBAL_EEPROM_Unlocked)
            {
                printf("EEPROM Unlocked");
            }
            else
            {
                printf("*EEPROM Locked*");
            }
            Asterix_Separator(27, 1);
            printf("FW Date %s \r\n", String_Firmware_Revision_Date);
            printf("Sensor Barcode %s \r\n", Barcode_);
            printf("pA/PPM  %ld, Avg %u, ", (10 * (long int)nA_per_PPM_x100), ADC_Average);   
            if(Ce_Re_Connected)
            {
                printf("Bias %ld, ", Bias_uV); 
            }
            else
            {
                printf("CE+RE Disconnected, ");
            }
            printf("Ton %lu \r\n", Time_On_Counter);
//            printf("pA/PPM  %ld, Avg %u, Bias %ld, Ton %lu \r\n", (10 * (long int)nA_per_PPM_x100), ADC_Average, Bias_uV, Time_On_Counter);
            printf("Nx10 %u, Toff %d, Hoff %d \r\n", n_factor_x10, Temperature_Offset_x1000, Humidity_Offset_x1000);
            
            printf("LPDACDAT0 %lu, LPTIACON0 %lu \r\n", LPDACDAT0_Setting, LPTIACON0_Setting);
            
            printf("OC   ADC G %u, T %u, H %u \r\n", AD5941_Result_16bit_OC, T_Raw_OC, H_Raw_OC);  
            printf("Zero ADC G %u, T %u, H %u \r\n", AD5941_Result_16bit_Zero, T_Raw_Zero, H_Raw_Zero); 
            printf("Span ADC G %u, T %u, H %u \r\n", AD5941_Result_16bit_Span , T_Raw_Span, H_Raw_Span);            
            
            if(Module_Barcode_[0] > 0x00)
            {
                printf("Module Barcode %s ", Module_Barcode_);
            }
            printf("\r\n");
            Asterix_Separator(66, 1);
        }
        if((uart_command == 'C') && (GLOBAL_EEPROM_Unlocked))                   //Continuous_Output
        {            
            if(Output_On_Demand)
            {
                Output_On_Demand = 0x0000;
            }
            else
            {
                Output_On_Demand = 0xFFFF;
            }
            writeNVM(0xD2, Output_On_Demand); 
        }    
        if((uart_command == 'A') && (GLOBAL_EEPROM_Unlocked))                   //ADC_Average
        {          
            Asterix_Separator(66, 1);
            printf("Enter ADC Average: ");
            UART_Entry(1);
            ADC_Average = (unsigned int)atoi(uart_receive_buffer_);
            printf("\r\n");
            Asterix_Separator(66, 1);
            
            if(ADC_Average < 1)
            {
                ADC_Average = 1;
            }
            if(ADC_Average > 256)
            {
                ADC_Average = 256;
            }
            writeNVM(0xD0, ADC_Average);
        }
        if((uart_command == 'M') && (GLOBAL_EEPROM_Unlocked))                    //Set Module Barcode
        {
            Asterix_Separator(66, 1);
            printf("Enter Module Barcode: ");
            UART_Entry(0);
            printf("\r\n");
            i = 0;
            while(i < 20)
            {
                Module_Barcode_[i] = uart_receive_buffer_[i];
                i++;
            }
            Module_Barcode_[20] = '\0';
            i = 0;
            while(i < 20)
            {
                writeNVM((0x00D8 + (i * 2)), Module_Barcode_[i]);
                i++;
            }
            Asterix_Separator(66, 1);            
        }
        if((uart_command == 'B') && (Ce_Re_Connected) && (GLOBAL_EEPROM_Unlocked)) //Set Barcode
        {
            Asterix_Separator(66, 1);
            printf("Enter Sensor Barcode: ");
            UART_Entry(0);
            printf("\r\n");
            i = 0;
            while(i < 64)
            {
                Barcode_[i] = uart_receive_buffer_[i];
                i++;
            }
            Barcode_[64] = '\0';
            i = 0;
            while(i < 64)
            {
                writeNVM((0x0000 + (i * 2)), Barcode_[i]);
                i++;
            } 
            i = 0;
            while(i < 12)
            {
                Serial_Number_[i] = Barcode_[i];
                i++;
            } 
            Serial_Number_[i] = '\0';  
            i = 13;
            while(i < 19)
            {
                Part_Number_[i - 13] = Barcode_[i];
                i++;
            } 
            Part_Number_[i - 13] = '\0'; 
            Part_Number = atol(Part_Number_);  
            LPDACDAT0_Setting = 0x0001F7BF;                                     //0mV bias, default setting
            LPTIACON0_Setting = 0x00002740;                                     //0=Rlpf, 10=Rload, Rtia = 512k
//            LPTIACON0_Setting = 0x00002680;                                     //0=Rlpf, 10=Rload, Rtia = 100k
            if((Part_Number == 110102) || (Part_Number == 110114))              //CO
            {
                LPDACDAT0_Setting = 0x0001F7AC;                                 //+10mV bias             
            }  
            else if(Part_Number == 110202)                                      //EtOH
            {
                LPDACDAT0_Setting = 0x0001F775;                                 //+40mV bias
                LPTIACON0_Setting = 0x00002640;                                 //0=Rlpf, 10=Rload, Rtia = 85k
            }
            else if(Part_Number == 110303)                                      //H2S
            {
                LPDACDAT0_Setting = 0x0001F7AC;                                 //+10mV bias
            }            
            else if((Part_Number == 110406) || (Part_Number == 110507) || (Part_Number == 110440) || (Part_Number == 110540)) //NO2&O3
            {
                LPDACDAT0_Setting = 0x0001F7E5;                                 //-20mV bias      
            } 
            else if((Part_Number == 110610) || (Part_Number == 110640))         //SO2
            {
//                LPDACDAT0_Setting = 0x0001F7BF;                               //0mV bias, same as default
            }
            else if(Part_Number == 110801)                                      //IAQ
            {
                LPDACDAT0_Setting = 0x0001F6A8;                                 //+150mV bias
            }
            else if(Part_Number == 110901)                                      //IRR
            {
                LPDACDAT0_Setting = 0x0001F934;                                 //-200mV bias
            }
            else if(Part_Number == 110005)                                      //H2
            {
                LPDACDAT0_Setting = 0x0001F79A;                                 //+20mV bias
            }
            else if(Part_Number == 110450)                                      //Chlorine
            {
                LPDACDAT0_Setting = 0x0001F7EE;                                 //-25 mv bias
            }  
            else if(Part_Number == 110650)                                      //Ethylene
            {
                LPDACDAT0_Setting = 0x0001F533;                                 //+350 mV bias
            }
            else if(Part_Number == 110701)                                      //Nitric Oxide
            {
                LPDACDAT0_Setting = 0x0001F590;                                 //+300 mV bias
            }
            else if(Part_Number == 110850)                                      //Formaldehyde
            {
                LPDACDAT0_Setting = 0x0001F6A8;                                 //+150 mV bias
            }
            else if(Part_Number == 110860)                                      //Formaldehyde
            {
//                LPDACDAT0_Setting = 0x0001F7BF;                               //0mV bias, same as default
            }
            else
            {
                
            }
            writeNVM(0x80, (LPDACDAT0_Setting >> 16));                          
            writeNVM(0x82, (LPDACDAT0_Setting & 0x0000FFFF));                   
            AD5941_write_ADDR(0x2120);                                          //LPDACDAT0
            AD5941_write_DATA_32b_(LPDACDAT0_Setting);        
            writeNVM(0x84, (LPTIACON0_Setting >> 16));                          
            writeNVM(0x86, (LPTIACON0_Setting & 0x0000FFFF)); 
            AD5941_write_ADDR(0x20EC);                                          //LPTIACON0
            AD5941_write_DATA_32b_(LPTIACON0_Setting);   
            Temperature_Offset_x1000 = 0;    
            writeNVM(0xAC, Temperature_Offset_x1000); 
            n_factor_x10 = 50000;
            writeNVM(0xAE, n_factor_x10);
            Humidity_Offset_x1000 = 0;
            writeNVM(0xBC, Humidity_Offset_x1000);
            ADC_Average = 60;
            writeNVM(0xD0, ADC_Average);              
            Asterix_Separator(66, 1);
        }
        if(((uart_command == 'O') || (uart_command == 'B')) && (Ce_Re_Connected) && (GLOBAL_EEPROM_Unlocked)) //Set Open Circuit
        {            
            Asterix_Separator(66, 1);
            printf("Setting OC...");    
            
            TPRH_Counter = 0;
            AD5941_write_ADDR(0x20E4);                                          //LPTIASW0
            AD5941_write_DATA_32b_(0x00000000);
            DELAY_milliseconds(2750);
                
            AD5941_Result_16bit_OC = Read_AD5941_ADC() - 1;
            T_Raw_OC = (unsigned int)T_Raw;
            T_OC_float = Si7021_SHT20_HTU21D_STS21_Calculate_T(T_Raw_OC, Temperature_Offset_x1000);       
            H_Raw_OC = H_Raw;
            writeNVM(0x90, AD5941_Result_16bit_OC);
            writeNVM(0xA0, T_Raw_OC);
            writeNVM(0xB0, H_Raw_OC);    

            AD5941_write_ADDR(0x20E4);                                          //LPTIASW0
            AD5941_write_DATA_32b_(0x0000000C); 
            DELAY_milliseconds(250);
            Open_Circuit_Adjusted = 0xFFFF;
                                                                                
            printf("done\r\n");
            Asterix_Separator(66, 1);
        }
        if(((uart_command == 'Z') || (uart_command == 'B')) && (Ce_Re_Connected) && (GLOBAL_EEPROM_Unlocked)) //Set Zero
        {           
            Asterix_Separator(66, 1);
            printf("Setting zero..."); 
            if(uart_command == 'B')
            {
                AD5941_Result_16bit_Zero = AD5941_Result_16bit_OC;
            }
            else
            {
                AD5941_Result_16bit_Zero = AD5941_Result_16bit;   
            }
            T_Raw_Zero = (unsigned int)T_Raw;
            T_Zero_float = Si7021_SHT20_HTU21D_STS21_Calculate_T(T_Raw_Zero, Temperature_Offset_x1000);
            H_Raw_Zero = H_Raw;
            writeNVM(0x94, AD5941_Result_16bit_Zero);
            writeNVM(0xA4, T_Raw_Zero); 
            writeNVM(0xB4, H_Raw_Zero);
            
            printf("done\r\n");
            Asterix_Separator(66, 1);
        }                          
        if((uart_command == 'P') && (GLOBAL_EEPROM_Unlocked))                   //Set Pstat
        {
            Asterix_Separator(66, 1);
            printf("Enter LPDACDAT0: ");
            UART_Entry(1);
            printf("\r\n");
            
            LPDACDAT0_Setting_New = (unsigned long int)atol(uart_receive_buffer_);
            if(LPDACDAT0_Setting_New != 0)
            {
                writeNVM(0x80, (LPDACDAT0_Setting_New >> 16));                         
                writeNVM(0x82, (LPDACDAT0_Setting_New & 0x0000FFFF));
            }      
            
            printf("Enter LPTIACON0: ");
            UART_Entry(1);
            printf("\r\n");
            
            LPTIACON0_Setting_New = (unsigned long int)atol(uart_receive_buffer_); 
            if(LPTIACON0_Setting_New != 0)
            {
                writeNVM(0x84, (LPTIACON0_Setting_New >> 16));                         
                writeNVM(0x86, (LPTIACON0_Setting_New & 0x0000FFFF)); 
            }

            printf("done....restarting\r\n");    
            Asterix_Separator(66, 1);
            DELAY_milliseconds(100);

//            __asm__ volatile ("RESET");
//            while(1);
            uart_command = 'r';
        }
        if((uart_command == 'S') && (GLOBAL_EEPROM_Unlocked))                   //Set sensitivity (nA/PPM x 1000)
        {
            Asterix_Separator(66, 1);
            printf("Enter pA/PPM: ");
            UART_Entry(1);
            printf("\r\n");
            AD5941_Result_16bit_Span = 65535;
            nA_per_PPM_x100 = (atol(uart_receive_buffer_)) / 10;                
            T_Raw_Span = 65535;
            H_Raw_Span = 65535;
            writeNVM(0x98, AD5941_Result_16bit_Span);
            writeNVM(0x9C, ((nA_per_PPM_x100 >> 16) & 0xFFFF));
            writeNVM(0x9E, ((nA_per_PPM_x100 >> 0) & 0xFFFF));
            writeNVM(0xA8, T_Raw_Span);
            writeNVM(0xB8, H_Raw_Span);
            Asterix_Separator(66, 1);
        }
        if((uart_command == 'G') && (GLOBAL_EEPROM_Unlocked))                   //Set span 
        {     
            AD5941_Result_16bit_Span = (unsigned int)AD5941_Result_16bit;
            T_Raw_Span = (unsigned int)T_Raw;
            H_Raw_Span = H_Raw;

            Asterix_Separator(66, 1);
            printf("Enter PPB: ");
            UART_Entry(1);
            printf("\r\n");
            unsigned long int PPB_Span = (unsigned long int)atol(uart_receive_buffer_); 
            
            nA_per_PPM_x100  = ((double)nA - (double)Zcf) / (double)PPB_Span * (double)1E2 * (double)1E3;
                
            writeNVM(0x98, AD5941_Result_16bit_Span);
            writeNVM(0x9C, ((nA_per_PPM_x100 >> 16) & 0xFFFF));
            writeNVM(0x9E, ((nA_per_PPM_x100 >> 0) & 0xFFFF));
            writeNVM(0xA8, T_Raw_Span);
            writeNVM(0xB8, H_Raw_Span);
            Asterix_Separator(66, 1);
        }   
        if(uart_command == 's')                                                 //Set sleep 
        {   
            AD5941_write_ADDR(0x0A04);                                          //PWRKEY
            AD5941_write_DATA_16b_(0x4859);
            AD5941_write_ADDR(0x0A04);                                          //PWRKEY
            AD5941_write_DATA_16b_(0xF27B);
            AD5941_write_ADDR(0x0A00);                                          //PWRMOD
            AD5941_write_DATA_16b_(0x800A);              
            DELAY_milliseconds(100);
            
            U2MODEbits.UARTEN = 0;
            U2MODEbits.USIDL = 0;
            U2MODEbits.WAKE = 1;
            U2BRG = 25;	
            IEC1bits.U2RXIE = 1;
            U2MODEbits.UARTEN = 1;	
            U2STAbits.UTXEN = 1;
            RCONbits.WDTO = 0;
            RCONbits.SWDTEN = 0;
            DELAY_milliseconds(500);
            Sleep();
            while(1);
        }
        if((uart_command == 'T') && (GLOBAL_EEPROM_Unlocked))                   //Set temperature offset in degrees C/1000
        {
            Asterix_Separator(66, 1);
            printf("Enter T Offset: ");
            UART_Entry(1);
            Temperature_Offset_x1000 = (int)(atoi(uart_receive_buffer_));    
            writeNVM(0xAC, Temperature_Offset_x1000);        
            printf("\r\n");
            Asterix_Separator(66, 1);
        }
        if((uart_command == 'H') && (GLOBAL_EEPROM_Unlocked))                   //Set RH offset in %/1000
        {
            Asterix_Separator(66, 1);
            printf("Enter H Offset: ");
            UART_Entry(1);
            Humidity_Offset_x1000 = (int)(atoi(uart_receive_buffer_));
            writeNVM(0xBC, Humidity_Offset_x1000);        
            printf("\r\n");
            Asterix_Separator(66, 1);
        }
        if((uart_command == 'N') && (GLOBAL_EEPROM_Unlocked))                   //Set temp comp factor
        {
            Asterix_Separator(66, 1);
            printf("Enter Nx10: ");
            UART_Entry(1);
            n_factor_x10 = (unsigned int)atoi(uart_receive_buffer_);
            writeNVM(0xAE, n_factor_x10);        
            printf("\r\n");
            Asterix_Separator(66, 1);
        }  
        if(uart_command == 'E')                                                 //Lock/unlock EEPROM
        {
            Asterix_Separator(66, 1);
            Unlock();
            Asterix_Separator(66, 1);
        }
#ifdef Experimental_Function_Info         
        if(uart_command == 'I')                                                 //Enter Info
        {
            Asterix_Separator(66, 1);
            printf("Enter Info: ");
            UART_Entry(1);
            printf("\r\n");
            i = 0;
            while(i < 128)
            {
                Info_[i] = uart_receive_buffer_[i];
                i++;
            }
            Info_[128] = '\0';
            i = 0;
            while(i < 128)
            {
                writeNVM((0x0100 + (i * 2)), Info_[i]);
                i++;
            }
            Asterix_Separator(66, 1);           
        }
        if(uart_command == 'i')                                                 //Read out Info
        {
            Asterix_Separator(66, 1);
            if(Info_[0] > 0x00)
            {
                printf("%s \r\n", Info_);
            }
            Asterix_Separator(66, 1);
        }
#endif
#ifdef Experimental_Function_BiasBump        
        if(uart_command == '^')                                                 //Bias Bump
        {
            AD5941_write_ADDR(0x20EC);                                          //LPTIACON0
            AD5941_write_DATA_32b_(0x00002680);                                 //0=Rlpf, 10=Rload, Rtia = 100k
            
//            Asterix_Separator();    
            printf("Enter LPDACDAT0: ");                                        //LPDACDAT0_Setting = 0x0001F7AC; //+10mV bias
            UART_Entry(1);
            LPDACDAT0_Setting_New = (unsigned long int)atol(uart_receive_buffer_);  
            printf("\r\nEnter Bump Time: ");
            UART_Entry(1);
            Bias_Bump_Time = (unsigned int)atoi(uart_receive_buffer_);
            if(Bias_Bump_Time > 192)
            {
                Bias_Bump_Time = 192;
            }
            printf("\r\n");
            Asterix_Separator();

            ClrWdt();
            i = 0;
            while(i < 32)
            {
                AD5941_Result_16bit_[i] = (unsigned int)Read_AD5941_ADC(); 
                i++;
            }  
            AD5941_write_ADDR(0x2120);                                          //LPDACDAT0
            AD5941_write_DATA_32b_(LPDACDAT0_Setting_New);
            while(i < (Bias_Bump_Time + 32))
            {  
                AD5941_Result_16bit_[i] = (unsigned int)Read_AD5941_ADC(); 
                i++;
            }  
            AD5941_write_ADDR(0x2120);                                          //LPDACDAT0
            AD5941_write_DATA_32b_(LPDACDAT0_Setting); 
            while(i < 256)
            {  
                AD5941_Result_16bit_[i] = (unsigned int)Read_AD5941_ADC(); 
                i++;
            } 
            
            i = 0;
            while(i < 256)
            {
                printf("%u\r\n", AD5941_Result_16bit_[i]);
                i++;
            }  
            
            i = 0;
            while(i < 256)                                                      //Set running average window at 32k
            {
                AD5941_Result_16bit_[i] = AD5941_Result_16bit_Zero;
                i++;
            }
            Asterix_Separator();
            
            AD5941_write_ADDR(0x20EC);                                          //LPTIACON0
            AD5941_write_DATA_32b_(LPTIACON0_Setting);                                 
            DELAY_milliseconds(10000);
        }
#endif
#ifdef Experimental_Function_CeReDisconnect        
        if(uart_command == '!')
        {
            if(Ce_Re_Connected)
            {           
                Ce_Re_Connected = 0x0000;
            }
            else
            {
                Ce_Re_Connected = 0xFFFF;
            }
            writeNVM(0xD6, Ce_Re_Connected);
            DELAY_milliseconds(100); 
            uart_command = 'r';
        }
#endif
#ifdef Experimental_Function_BiasStairCases
        if(uart_command == '1')
        {
            RCONbits.SWDTEN = 0;
            
            LPDACDAT0_Setting = 0x0001F7BF;          
            i = 0;
            while(i < 279)
            { 
                AD5941_write_ADDR(0x2120);                                          //LPDACDAT0
                AD5941_write_DATA_32b_(LPDACDAT0_Setting);  
                
                DELAY_milliseconds(992);                
                AD5941_Result_16bit = (unsigned int)Read_AD5941_ADC(); 
                printf("%u\r\n", AD5941_Result_16bit);
                LPDACDAT0_Setting--;
                i++;
            }
            i = 0;
            while(i < 559)
            { 
                AD5941_write_ADDR(0x2120);                                          //LPDACDAT0
                AD5941_write_DATA_32b_(LPDACDAT0_Setting);  
                
                DELAY_milliseconds(992);                
                AD5941_Result_16bit = (unsigned int)Read_AD5941_ADC(); 
                printf("%u\r\n", AD5941_Result_16bit);
                LPDACDAT0_Setting++;
                i++;
            } 
            i = 0;
            while(i < 281)
            { 
                AD5941_write_ADDR(0x2120);                                          //LPDACDAT0
                AD5941_write_DATA_32b_(LPDACDAT0_Setting);  
                
                DELAY_milliseconds(992);                
                AD5941_Result_16bit = (unsigned int)Read_AD5941_ADC(); 
                printf("%u\r\n", AD5941_Result_16bit);
                LPDACDAT0_Setting--;
                i++;
            } 
            DELAY_milliseconds(1000);
            uart_command = 'r';
        }  
        if(uart_command == '2')
        {
            RCONbits.SWDTEN = 0;
                
            printf("\r\nEnter Interval Time in ms (100 - 60000): ");
            UART_Entry(1);
            unsigned int Interval_Time = (unsigned int)atoi(uart_receive_buffer_);
            if(Interval_Time < 100)
            {
                Interval_Time = 100;
            }
            if(Interval_Time > 60000)
            {
                Interval_Time = 60000;
            }
            Interval_Time = Interval_Time - 8;
            printf("\r\n");
            
            printf("\r\nEnter Number of Bias Change Intervals (1 - 820): ");
            UART_Entry(1);
            unsigned int Bias_Intervals = (unsigned int)atoi(uart_receive_buffer_);
            if(Bias_Intervals < 1)
            {
                Bias_Intervals = 1;
            }
            if(Bias_Intervals > 820)
            {
                Bias_Intervals = 820;
            }
            printf("\r\n");
            
            LPDACDAT0_Setting = 0x0001F7BF;          
            i = 0;
            while(i < (Bias_Intervals - 1))
            { 
                AD5941_write_ADDR(0x2120);                                          //LPDACDAT0
                AD5941_write_DATA_32b_(LPDACDAT0_Setting);  
                
                DELAY_milliseconds(Interval_Time);                
                AD5941_Result_16bit = (unsigned int)Read_AD5941_ADC(); 
                printf("%u\r\n", AD5941_Result_16bit);
                LPDACDAT0_Setting--;
                i++;
            }
            i = 0;
            while(i < (2 * Bias_Intervals - 1))
            { 
                AD5941_write_ADDR(0x2120);                                          //LPDACDAT0
                AD5941_write_DATA_32b_(LPDACDAT0_Setting);  
                
                DELAY_milliseconds(Interval_Time);                
                AD5941_Result_16bit = (unsigned int)Read_AD5941_ADC(); 
                printf("%u\r\n", AD5941_Result_16bit);
                LPDACDAT0_Setting++;
                i++;
            } 
            i = 0;
            while(i < (Bias_Intervals + 1))
            { 
                AD5941_write_ADDR(0x2120);                                          //LPDACDAT0
                AD5941_write_DATA_32b_(LPDACDAT0_Setting);  
                
                DELAY_milliseconds(Interval_Time);                
                AD5941_Result_16bit = (unsigned int)Read_AD5941_ADC(); 
                printf("%u\r\n", AD5941_Result_16bit);
                LPDACDAT0_Setting--;
                i++;
            } 
            DELAY_milliseconds(1000);
            uart_command = 'r';  
        }  
        if(uart_command == '3')
        {
            RCONbits.SWDTEN = 0;
                
            printf("\r\nEnter Interval Time in ms (100 - 60000): ");
            UART_Entry(1);
            unsigned int Interval_Time = (unsigned int)atoi(uart_receive_buffer_);
            if(Interval_Time < 100)
            {
                Interval_Time = 100;
            }
            if(Interval_Time > 60000)
            {
                Interval_Time = 60000;
            }
            Interval_Time = Interval_Time - 8;
            printf("\r\n"); 
            
            printf("\r\nEnter Bias Change Magnitude (1 - 82): ");
            UART_Entry(1);
            unsigned int Bias_Change_Magnitude = (unsigned int)atoi(uart_receive_buffer_);
            if(Bias_Change_Magnitude < 1)
            {
                Bias_Change_Magnitude = 1;
            }
            if(Bias_Change_Magnitude > 82)
            {
                Bias_Change_Magnitude = 82;
            }
            printf("\r\n");
            
            printf("\r\nEnter Number of Bias Change Intervals (1 - 820): ");
            UART_Entry(1);
            unsigned int Bias_Intervals = (unsigned int)atoi(uart_receive_buffer_);
            if(Bias_Intervals < 1)
            {
                Bias_Intervals = 1;
            }
            if(Bias_Intervals > 820)
            {
                Bias_Intervals = 820;
            }
            if(((double)Bias_Intervals * (double)Bias_Change_Magnitude) > 820)
            {
                Bias_Intervals = Bias_Intervals / ceil(((double)Bias_Intervals * (double)Bias_Change_Magnitude / 820));
            }
            printf("\r\n");
            
            LPDACDAT0_Setting = 0x0001F7BF;          
            i = 0;
            while(i < (Bias_Intervals - 1))
            { 
                AD5941_write_ADDR(0x2120);                                          //LPDACDAT0
                AD5941_write_DATA_32b_(LPDACDAT0_Setting);  
                
                DELAY_milliseconds(Interval_Time);                
                AD5941_Result_16bit = (unsigned int)Read_AD5941_ADC(); 
                printf("%u\r\n", AD5941_Result_16bit);
                LPDACDAT0_Setting = LPDACDAT0_Setting - Bias_Change_Magnitude;
                i++;
            }
            i = 0;
            while(i < (2 * Bias_Intervals - 1))
            { 
                AD5941_write_ADDR(0x2120);                                          //LPDACDAT0
                AD5941_write_DATA_32b_(LPDACDAT0_Setting);  
                
                DELAY_milliseconds(Interval_Time);                
                AD5941_Result_16bit = (unsigned int)Read_AD5941_ADC(); 
                printf("%u\r\n", AD5941_Result_16bit);
                LPDACDAT0_Setting = LPDACDAT0_Setting + Bias_Change_Magnitude;
                i++;
            } 
            i = 0;
            while(i < (Bias_Intervals + 1))
            { 
                AD5941_write_ADDR(0x2120);                                          //LPDACDAT0
                AD5941_write_DATA_32b_(LPDACDAT0_Setting);  
                
                DELAY_milliseconds(Interval_Time);                
                AD5941_Result_16bit = (unsigned int)Read_AD5941_ADC(); 
                printf("%u\r\n", AD5941_Result_16bit);
                LPDACDAT0_Setting = LPDACDAT0_Setting - Bias_Change_Magnitude;
                i++;
            } 
            DELAY_milliseconds(1000);
            uart_command = 'r';  
        }
#endif
#ifdef  Experimental_Function_BiasAdjustTo0nA    
        if(uart_command == '*')
        {
            RCONbits.SWDTEN = 0;
             
//            LPDACDAT0_Setting = 0x0001F7BF;       
//            LPTIACON0_Setting = 0x00002740;
//            
//            AD5941_write_ADDR(0x2120);                                                  //LPDACDAT0
//            AD5941_write_DATA_32b_(LPDACDAT0_Setting);
//            AD5941_write_ADDR(0x20EC);                                                  //LPTIACON0
//            AD5941_write_DATA_32b_(LPTIACON0_Setting);
            
            i = 0;
            while(1)
            {
                i++;
                
                T_Raw = (unsigned long int)Si7021_SHT20_HTU21D_Read_T();            
                H_Raw = Si7021_SHT20_HTU21D_Read_RH();
                T_float = Si7021_SHT20_HTU21D_STS21_Calculate_T((unsigned int)T_Raw, Temperature_Offset_x1000);
                H_float = Si7021_SHT20_HTU21D_Calculate_RH(H_Raw, Humidity_Offset_x1000);
                T_x100 = 100 * T_float;
                H_x100 = 100 * H_float;
                AD5941_Result_32bit = 0;
                j = 0;
                while(j < 16)
                {
                    AD5941_Result_32bit = AD5941_Result_32bit + Read_AD5941_ADC(); 
                    j++;
                }
                AD5941_Result_16bit = AD5941_Result_32bit >> 4;
                AD5941_Result_16bit_[i] = AD5941_Result_16bit;
                Rgain = Rgain_Calc(LPTIACON0_Setting);
                Vout = ((double)AD5941_Result_16bit - 32768) / 32768 * 1.82;        //AIN4 - Low power TIA negative input
                Vzero = ((double)AD5941_Result_16bit_Zero - 32768) / 32768 * 1.82;  //AIN4 - Low power TIA negative input
                Voc = ((double)AD5941_Result_16bit_OC - 32768) / 32768 * 1.82;      //AIN4 - Low power TIA negative input
                nA = ((double)Vout - (double)Voc) / Rgain * (double)1E9;
//                Zcf = ((double)Vzero - (double)Voc) / Rgain * (double)1E9;
//                n_factor = (double)n_factor_x10 / 10;
//                Zcf = Zcf * exp((T_float - T_Zero_float) / (double)n_factor);       
//                PPB = ((double)nA - (double)Zcf) / (double)nA_per_PPM_x100 * (double)1E2 * (double)1E3;
                PPB = ((double)nA) / (double)nA_per_PPM_x100 * (double)1E2 * (double)1E3;       
                DELAY_milliseconds(5000);
                
                AD5941_write_ADDR(0x21A8);                                          //ADCCON 
                AD5941_write_DATA_32b_(0x0000081A);                                 //RE0 - Vbias_Cap
                DELAY_milliseconds(100);
                RE0_Vbias_Cap = Read_AD5941_ADC();    
                AD5941_write_ADDR(0x21A8);                                          //ADCCON 
                AD5941_write_DATA_32b_(0x0000080E);                                 //SE0 - Vbias_Cap
                DELAY_milliseconds(100);     
                SE0_Vbias_Cap = Read_AD5941_ADC();
                Bias_uV = ((double)SE0_Vbias_Cap - (double)RE0_Vbias_Cap) / 32768 * 1.82 * 1E6;
                AD5941_write_ADDR(0x21A8);                                          //ADCCON 
                AD5941_write_DATA_32b_(0x00000214);                                 //AIN4 - Low power TIA negative input
                DELAY_milliseconds(5000);

                printf("%u, %ld, %ld, %ld, %d, %u, %lu, %lu \r\n", AD5941_Result_16bit, (long int)(1000 * nA), (long int)PPB, Bias_uV, T_x100, H_x100, LPDACDAT0_Setting, LPTIACON0_Setting);
                DELAY_milliseconds(4770); 
                
                if(i == 60)
                {
                    i = 0;
                    
                    AD5941_Result_32bit = 0;
                    j = 49;
                    while(j < 61)
                    {
                        AD5941_Result_32bit = AD5941_Result_32bit + AD5941_Result_16bit_[j];
                        j++;
                    }
                    AD5941_Result_16bit = AD5941_Result_32bit / 12;
                    Rgain = Rgain_Calc(LPTIACON0_Setting);
                    Vout = ((double)AD5941_Result_16bit - 32768) / 32768 * 1.82;        //AIN4 - Low power TIA negative input
                    Voc = ((double)AD5941_Result_16bit_OC - 32768) / 32768 * 1.82;      //AIN4 - Low power TIA negative input
                    nA = ((double)Vout - (double)Voc) / Rgain * (double)1E9;   
                    printf("%u, %ld \r\n", AD5941_Result_16bit, (long int)(1000 * nA));                
                    
                    if(nA > 50)
                    {
                        LPDACDAT0_Setting++;
                        LPDACDAT0_Setting++;
                        LPDACDAT0_Setting++;
                        LPDACDAT0_Setting++;
                    }
                    else if(nA > 25)
                    {
                        LPDACDAT0_Setting++;
                        LPDACDAT0_Setting++;
                        LPDACDAT0_Setting++;
                    }
                    else if(nA > 10)
                    {
                        LPDACDAT0_Setting++;
                        LPDACDAT0_Setting++;
                    }
                    else if(nA > 0)
                    {
                        LPDACDAT0_Setting++;
                    }
                    else if(nA < -50)
                    {
                        LPDACDAT0_Setting--;
                        LPDACDAT0_Setting--;
                        LPDACDAT0_Setting--;
                        LPDACDAT0_Setting--;
                    }
                    else if(nA < -25)
                    {
                        LPDACDAT0_Setting--;
                        LPDACDAT0_Setting--;
                        LPDACDAT0_Setting--;
                    }
                    else if(nA < -10)
                    {
                        LPDACDAT0_Setting--;
                        LPDACDAT0_Setting--;
                    }
                    else if(nA < 0)
                    {
                        LPDACDAT0_Setting--;
                    }
                    else
                    {
                        
                    }
                    AD5941_write_ADDR(0x2120);                                  //LPDACDAT0
                    AD5941_write_DATA_32b_(LPDACDAT0_Setting);
                }
            }
        }
#endif        
        if(uart_command == 'r')                                                 //Reset MCU
        {           
            __asm__ volatile ("RESET");
            while(1);
        }

        uart_command = '\0';
        ClrWdt();
    }
    return 1;
}
/**
 End of File
*/

