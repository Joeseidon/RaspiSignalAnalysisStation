//#############################################################################
//
// FILE:   SignalAnalysisStation_ThreePhaseGen.c
//
// TITLE:  Three Phase Power Generator
//
// AUTHOR:  Joseph Cutino
//#############################################################################
// $TI Release: F2837xS Support Library v3.02.00.00 $
// $Release Date: Sat Sep 16 15:30:24 CDT 2017 $
// $Copyright:
// Copyright (C) 2014-2017 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
#include "F28x_Project.h"
#include "sgen.h"         // Signal Generation Headerfile
#include "driverlib.h"
#include "device.h"

/*Standard includes*/
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>


//
// Three Phase Gen Defines
//
/*
    DACX    |   ChannelX    |   SGEN #  |   PIN# (J3)
  -------------------------------------------------
    DACA        Channel 1       SGEN 1      PIN 27
    DACB        Channel 2       SGEN 2      PIN 29
    DACC        Channel 3       SGEN 3      PIN 24
*/
#define DACA                  1
#define DACB                  2
#define DACC                  3

/*NOTE:
When REFERENCE is set to REFERENCE_VDAC PIN 28 on J3 must
be connected to 3v3.
*/
#define REFERENCE_VDAC        0
#define REFERENCE_VREF        1
#define REFERENCE             REFERENCE_VDAC

#define CPUFREQ_MHZ           200
#define PI 					  3.14159265358979323846

//
// Three Phase Generator Modes
//
#define LINEAR_INT			  1
#define NORMAL_GEN			  0
#define PHASE_GEN_TYPE	      LINEAR_INT

//
// I2C Defines
//
#define SLAVE_ADDRESS         8
#define BUFFER_SIZE           16    

//
// Operation Codes to Control Work Flow
//
enum opCode
{
    Idle        =   1,
    Start       =   2,
    Stop        =   3,
    PowerDown   =   4
};

//
// Globals
//
enum opCode op_code = Idle; //System starts in Initialize state
volatile struct DAC_REGS* DAC_PTR[4] = {0x0,&DacaRegs,&DacbRegs,&DaccRegs}; //DAC Registers

//
//Waveform settings
//
Uint32 samplingFreq_hz  = 790000;    //default to 200000 
                                     //Max when running from RAM = 950000
                                     //Max when running from FLASH = 790000

/*Output frequency default. This value will change up start up of the
  ThreePhase GUI
*/
Uint32 outputFreq_hz    = 1000;        //default to 1000

/*NOTE:
  It appears that the actual max output frequency, as determined by external
  o-scope, is maxOutputFreq_hz/2
*/
Uint32 maxOutputFreq_hz = 20000;    //default to 5000 (try setting to 10000)
float waveformGain      = 0.8003; // Range 0.0 -> 1.0
float waveformOffset    = 0.0;      // Range -1.0 -> 1.0

/*//
//Channel Phase
//
const unsigned long int phase1_shift = 0;
const unsigned long int phase2_shift = 21845;	//120 degrees off
const unsigned long int phase3_shift = 43690;	//240 degrees off
*/
//Define Signal Generator
#if PHASE_GEN_TYPE==LINEAR_INT
SGENTI_1 sgen_1 = SGENTI_1_DEFAULTS;
SGENTI_1 sgen_2 = SGENTI_1_DEFAULTS;
SGENTI_1 sgen_3 = SGENTI_1_DEFAULTS;
#elif PHASE_GEN_TYPE==NORMAL_GEN
SGENT_1 sgen_1 = SGENT_1_DEFAULTS;
SGENT_1 sgen_2 = SGENT_1_DEFAULTS;
SGENT_1 sgen_3 = SGENT_1_DEFAULTS;
#endif

//Sgen Channel Value Storage
int sgen_1_out = 0;
int sgen_2_out = 0;
int sgen_3_out = 0;

float channel1_offset = 0.0;
float channel2_offset = 0.0;
float channel3_offset = 0.0;
float channel1_gain = 0.0;
float channel2_gain = 0.0;
float channel3_gain = 0.0;

//I2C Globals
uint16_t rData[BUFFER_SIZE];    // Send data buffer

//Error Flags
bool RX_ERROR = false;
bool CMD_ERROR = false;

//Function Prototypes
static inline void setFreq(void);
static inline void setGain(void);
static inline void setOffset(void);
void configureDAC(void);
void configureWaveform(void);
interrupt void cpu_timer0_isr(void);
static inline void disableSignalGen(void);
static inline void enableSignalGen(void);
//static inline void channelOffsetDecode(int ch, uint16_t code);

void initI2CFIFO(void);
inline void decodeMsg(void); 
__interrupt void i2cFIFOISR(void);

inline float balanceGain(float generalGain, float channelGain);
inline float balanceOffset(float generalOffset, float channelOffset);
inline void resetPhase(void);
//
// Main
//
void main(void)
{

//
// Initialize device clock and peripherals
//
    Device_init();

//
// Disable CPU interrupts
//
    DINT;
    
//
// Disable pin locks and enable internal pullups.
//
    Device_initGPIO();

//
// Initialize PIE and clear PIE registers. Disables CPU interrupts.
//
    Interrupt_initModule();

//
// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
//
    Interrupt_initVectorTable();

//
// Interrupts that are used in this example are re-mapped to ISR functions
// found within this file.
//
    Interrupt_register(INT_I2CA_FIFO, &i2cFIFOISR);
    Interrupt_register(INT_TIMER0, &cpu_timer0_isr);
   
//
// Set I2C use, initializing it for FIFO mode
//
    initI2CFIFO();

//
// Initialize the data buffers
//
    uint16_t i;
    for(i = 0; i < BUFFER_SIZE; i++)
    {
        rData[i] = i;   //i for testing. After dev should be 0
    }
//
// Configure DAC
//
    configureDAC();
    
//
// Configure Waveform
//
    configureWaveform();

//
// Initialize Cpu Timers
//
    InitCpuTimers();

//
// Configure Cpu Timer0 to interrupt at specified sampling frequency
//
    ConfigCpuTimer(&CpuTimer0, CPUFREQ_MHZ, 1000000.0/samplingFreq_hz);

//
// Start Cpu Timer0
//
    CpuTimer0Regs.TCR.all = 0x4000;

//
// Enable interrupts required for this example
//
    Interrupt_enable(INT_I2CA_FIFO);

//
// Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
//
    EINT;
    ERTM;
    

    while(1)
    {
        while(op_code == Idle)
        {
            //wait for op_code change from Master
        //
        //Delay for 1ms to allow new op_code value to load
        //
            DELAY_US(1000);
        }
        if(op_code == Start){
            enableSignalGen();
            while(op_code == Start)
            {
                setFreq();   // Set Output Frequency and Max Output Frequency
                setGain();   // Set Magnitude of Waveform
                setOffset(); // Set Offset of Waveform
            }
        }
        if(op_code == Stop){
            //turn off sgen
            //To turn off just set all sgen outputs to 0 and disable timer0 interrupt
            disableSignalGen();
            //change op_code to idle and wait for further instruction
            op_code = Idle;
        }
        //As of now this functionality is not active. 
        if(op_code == PowerDown){
            //turn off sgen
            //To turn off just set all sgen outputs to 0 and disable timer0 interrupt
            disableSignalGen();
            //put system in low power mode and wait for wake commnad(Start)
            while(op_code != Start){
                //TODO:
                ;
            }
        }
        
	}
}//End Main

//
// setFreq - Set the SINE frequency in SGEN
//
static inline void setFreq(void)
{
	//
    // Range(Q0) = 0x0000 -> 0x7FFF, step_max(Q0) =
    // (Max_Freq_hz*0x10000)/Sampling_Freq_hz
    //
    sgen_1.step_max = (maxOutputFreq_hz*0x10000)/samplingFreq_hz;
	sgen_2.step_max = (maxOutputFreq_hz*0x10000)/samplingFreq_hz;
	sgen_3.step_max = (maxOutputFreq_hz*0x10000)/samplingFreq_hz;

    //
    // Range(Q15) = 0x0000 -> 0x7FFF, freq(Q15) =
    // (Required_Freq_hz/Max_Freq_hz)*0x8000
    //
    sgen_1.freq = ((float)outputFreq_hz/maxOutputFreq_hz)*0x8000;
	sgen_2.freq = ((float)outputFreq_hz/maxOutputFreq_hz)*0x8000;
	sgen_3.freq = ((float)outputFreq_hz/maxOutputFreq_hz)*0x8000;
}


//
// setGain - Set the gain in SGEN
//
static inline void setGain(void)
{
	sgen_1.gain = channel1_gain * 0x7FFF;   // Range(Q15) = 0x0000 -> 0x7FFF
	sgen_2.gain = channel2_gain * 0x7FFF;   // Range(Q15) = 0x0000 -> 0x7FFF
	sgen_3.gain = channel3_gain * 0x7FFF;   // Range(Q15) = 0x0000 -> 0x7FFF
}

//
// setOffset - Set the offset in SGEN
//
static inline void setOffset(void)
{
	sgen_1.offset = channel1_offset * 0x7FFF; // Range(Q15) = 0x8000 -> 0x7FFF
	sgen_2.offset = channel2_offset * 0x7FFF; // Range(Q15) = 0x8000 -> 0x7FFF
	sgen_3.offset = channel3_offset * 0x7FFF; // Range(Q15) = 0x8000 -> 0x7FFF
}

//
// enableSignalGen - Enables DAC outputs and starts generators timer interrupt
//
static inline void enableSignalGen(void)
{
    Interrupt_enable(INT_TIMER0);
    //reset DACs and enable
    EALLOW;
    DAC_PTR[DACA]->DACVALS.all = 0;
    DAC_PTR[DACB]->DACVALS.all = 0;
    DAC_PTR[DACC]->DACVALS.all = 0;
    DAC_PTR[DACA]->DACOUTEN.bit.DACOUTEN = 1;
    DAC_PTR[DACB]->DACOUTEN.bit.DACOUTEN = 1;
    DAC_PTR[DACC]->DACOUTEN.bit.DACOUTEN = 1;
    DELAY_US(10); //Allow for buffered DAC to power up
    EDIS;
}
//
// disableSignalGen - Disables DAC outputs to avoid noise on the lines when sig not generated
//
static inline void disableSignalGen(void)
{
    Interrupt_disable(INT_TIMER0);
    //reset DACs and disable
    EALLOW;
    DAC_PTR[DACA]->DACVALS.all = 0;
    DAC_PTR[DACB]->DACVALS.all = 0;
    DAC_PTR[DACC]->DACVALS.all = 0;
    DAC_PTR[DACA]->DACOUTEN.bit.DACOUTEN = 0;
    DAC_PTR[DACB]->DACOUTEN.bit.DACOUTEN = 0;
    DAC_PTR[DACC]->DACOUTEN.bit.DACOUTEN = 0;
    DELAY_US(10); //Allow for buffered DAC to power up
    EDIS;
}
//
// configureDAC - Enable and configure DAC modules
//
void configureDAC(void)
{
	EALLOW;
	
	//
	//Configure DACA
	//
	DAC_PTR[DACA]->DACCTL.bit.DACREFSEL = REFERENCE;

	//
	//Configure DACB
	//
	DAC_PTR[DACB]->DACCTL.bit.DACREFSEL = REFERENCE;

	//
	//Configure DACC
	//
	DAC_PTR[DACC]->DACCTL.bit.DACREFSEL = REFERENCE;
	
	DELAY_US(10); //Allow for buffered DAC to power up
	
	EDIS;
}
inline void resetPhase(void)
{
    /*
    phase = ((desired phase(radians) / 2pi) * 2^16)
    */
	sgen_1.alpha = 0; // Range(16) = 0x0000 -> 0xFFFF
	sgen_2.alpha = 21845;	//120 degrees off
	sgen_3.alpha = 43690;	//240 degrees off
}
//
// configureWaveform - Configure the SINE waveform
//
void configureWaveform(void)
{
    //
    //In this case we are initilizing with a phase but this function is reused to reset the phase angles
    //
    resetPhase();
	
    setFreq();
    setGain();
    setOffset();
}

//
// cpu_timer0_isr - Timer ISR that writes the sine value to DAC, log the sine
//                  value, compute the next sine value, and calculate interrupt
//                  duration
//
interrupt void cpu_timer0_isr(void)
{
	//
	// Write current sine value to buffered DACs
	//
	EALLOW;
	DAC_PTR[DACA]->DACVALS.all = sgen_1_out;
	DAC_PTR[DACB]->DACVALS.all = sgen_2_out;
	DAC_PTR[DACC]->DACVALS.all = sgen_3_out;
	EDIS;
	
	//
    // Scale next sine value
    //
    sgen_1_out = ((sgen_1.out + 32768) >> 4);
    sgen_2_out = ((sgen_2.out + 32768) >> 4);
    sgen_3_out = ((sgen_3.out + 32768) >> 4);
    
    //atempting to provide individual channel modification
    //
    // Compute next sine value
    //
    sgen_1.calc(&sgen_1);
	sgen_2.calc(&sgen_2);
	sgen_3.calc(&sgen_3);
	
	//
    // Acknowledge this interrupt to receive more interrupts from group 1
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//
// Function to configure I2C A in FIFO mode.
//
void initI2CFIFO()
{
    //
    //SDA and SCL pin config with pull-up resistors
    //
    GPIO_setPadConfig(42, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPadConfig(43, GPIO_PIN_TYPE_PULLUP);
    GPIO_SetupPinMux(42, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinMux(43, GPIO_MUX_CPU1, 6);

    //
    // Must put I2C into reset before configuring it
    //
    I2C_disableModule(I2CA_BASE);

    //
    // I2C configuration.
    //
    I2C_setOwnSlaveAddress(I2CA_BASE, SLAVE_ADDRESS);
    I2C_setConfig(I2CA_BASE, I2C_SLAVE_SEND_MODE);
    I2C_setBitCount(I2CA_BASE, I2C_BITCOUNT_8);

    //
    // FIFO and interrupt configuration
    //
    I2C_enableFIFO(I2CA_BASE);
    I2C_clearInterruptStatus(I2CA_BASE, I2C_INT_RXFF | I2C_INT_TXFF);
    I2C_setFIFOInterruptLevel(I2CA_BASE, I2C_FIFO_TX16, I2C_FIFO_RX16);
    I2C_enableInterrupt(I2CA_BASE, I2C_INT_RXFF);

    //
    // Configuration complete. Enable the module.
    //
    I2C_enableModule(I2CA_BASE);
}

//
// I2C A Receive FIFO ISR
//
 __interrupt void i2cFIFOISR(void)
{
    uint16_t i;
    RX_ERROR = false;
    //
    // If receive FIFO interrupt flag is set, read data
    //
    if((I2C_getInterruptStatus(I2CA_BASE) & I2C_INT_RXFF) != 0)
    {
        for(i = 0; i < BUFFER_SIZE; i++)
        {
            rData[i] = I2C_getData(I2CA_BASE);
        }
        decodeMsg();
        
        //
        // Clear interrupt flag
        //
        I2C_clearInterruptStatus(I2CA_BASE, I2C_INT_RXFF);
        
        //
        //Reset Phase After Every Modification to the Waveforms
        //
        resetPhase();
    }
    //
    // Issue ACK
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP8);
}

inline void decodeMsg(void)
{
    //define temp variables 
    uint16_t msbs = 0;
    uint16_t lsbs = 0;
    int temp = 0;
    
    //determine freq
    msbs = rData[2];
    lsbs = rData[3];
    outputFreq_hz = ((msbs << 8) | lsbs);
    
    //determine gain
    temp = rData[5];
    waveformGain = temp/100.0; //accounts for alterations made during sending from master
    
    //determine offset
    temp = rData[7];
    if(rData[6] & 0x01)   //account for negative offset
    {
        temp=temp*-1;
    }
    waveformOffset = temp/100.0;//accounts for alterations made during sending from master
    
    //determine op_code
    temp = rData[8];
    switch(temp)
    {
        case 1:
            op_code = Idle;
            break;
        case 2:
            op_code = Start;
            break;
        case 3:
            op_code = Stop;
            break;
        case 4:
            op_code = PowerDown;
            break;
        default:
            op_code = Idle;
            break;
    }
    
    //
    //Channel 1 Offset
    //
    temp=rData[9];
    if(rData[6] & 0x02)   //account for negative offset
    {
        temp=temp*-1;
    }
    channel1_offset = balanceOffset(waveformOffset,(temp/100.0));
    
    //
    //Channel 2 Offset
    //
    temp=rData[10];
    if(rData[6] & 0x04)   //account for negative offset
    {
        temp=temp*-1;
    }
    channel2_offset = balanceOffset(waveformOffset,(temp/100.0));
    
    //
    //Channel 3 Offset
    //
    temp=rData[11];
    if(rData[6] & 0x08)   //account for negative offset
    {
        temp=temp*-1;
    }
    channel3_offset = balanceOffset(waveformOffset,(temp/100.0));
    
    //
    //Channel 1 Gain
    //
    temp = rData[12];
    if(rData[6] & 0x10)   //account for negative gain
    {
        temp=temp*-1;
    }
    channel1_gain = balanceGain(waveformGain, (temp/100.0));
   
    //
    //Channel 2 Gain
    //
    temp = rData[13];
    if(rData[6] & 0x20)   //account for negative gain
    {
        temp=temp*-1;
    }
    channel2_gain = balanceGain(waveformGain, (temp/100.0));
    
    //
    //Channel 3 Gain
    //
    temp = rData[14];
    if(rData[6] & 0x40)   //account for negative gain
    {
        temp=temp*-1;
    }
    channel3_gain = balanceGain(waveformGain, (temp/100.0));
}

inline float balanceGain(float generalGain, float channelGain)
{
    float total_gain = (generalGain+channelGain);
    if(total_gain > 1.0){
        return 1.0;
    }else
    {
        return total_gain;
    }
}

inline float balanceOffset(float generalOffset, float channelOffset)
{
    float total_offset = generalOffset + channelOffset;
    if(total_offset > 1.0){
        return 1.0;
    }else if(total_offset < -1.0){
        return -1.0;
    }else{
        return total_offset;
    }
}
//
// End of File
//
