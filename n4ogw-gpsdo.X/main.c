/*
 GPSDO controller based on "Lars GPSDO" design using PIC18F27Q84

 R. Torsten Clay N4OGW
  
 for board version 2
 05/20/2023

  1. Counts 10 MHz directly using PIC SMT
  2. Divides 10 MHz down to 1 MHz; measures phase difference between 1 MHz and
     GPS PPS signal

  Frequency division and phase difference measurement are done using PIC
  CLC gates instead of by external hardware:
 
  -two CLC D flip-flops are used (CLC1 and CLC7) for the phase detector
  -CLC2, CLC3, CLC5, CLC6 are configured as a divide-by-10 counter
      -> these together create a 1 MHz signal from the OCXO (duty
      cycle is not 50%)
  -CLC8 and timer6 are used to de-bounce buttons
  -SMT is used to count 10 MHz directly (24 bit counter), gated by PPS
  -16 bit PWM generated directly instead of combining two 8-bit PWMs
 
  Pin connections:
 
  RA0 - RA7 : outputs; data for 2x16 LCD in 8 bit mode
  RB0 : LCD RS
  RB1 : ADC input; reads TIC voltage
  RB2 : 10 MHz in
  RB3 : TIC output; to 3.9 k resistor
  RB4 : button 1
  RB5 : button 2
  RB6 : ICSP clock
  RB7 : ICSP data
  RC0 : pps in
  RC1 : LCD E
  RC2 : uart2 rx (gps)
  RC3 : uart2 tx (gps)
  RC4 unused : can be 2nd pwm output
  RC5 : 16 bit PWM out
  RC6 : uart1 tx (computer)
  RC7 : uart1 rx (computer)
 */

// suppress MCC "function never called" messages
#pragma warning disable 520
#pragma warning disable 2053

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "defines.h"
#include "mcc_generated_files/pin_manager.h"
#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/pwm1_16bit.h"
#include "xlcd.h"

void adcInterrupt();
void buttonInterrupt(void);
void button2Func(void);
void calculation(void);
void checkButtons(void);
void counterSetup(void);
void counterUpdate(void);
void getCommand(void);
void gpsConfig(void);
void gpsExtraOff(void);
void gpsExtraOn(void);
void gpsParse(void);
void gpsRead(void);
inline uint8_t highByte(uint16_t x);
inline uint8_t lowByte(uint16_t x);
void lcdSetup(void);
void lcdUpdate(void);
long parseInt(void);
void printDataToSerial(void);
void printHeader1_ToSerial(void);
void printHeader2_ToSerial(void);
void printHeader3_ToSerial(void);
void setup(void);
void setScreen(screenType newScreen);

uint16_t warmUpTime = 300; // 300 gives five minutes hold during eg OCXO or Rb warmup. Set to eg 3 for VCTCXO
int32_t dacValueOut = 32768; // 16bit PWM-DAC setvalue=startvalue Max 65535 (if nothing stored in the EEPROM)
int32_t dacValue; // this is also same as "DACvalueOld" Note: is "0-65535" * timeconst

volatile uint16_t TIC_Value; // analog read 0  - time value. About 0.25ns per bit with 3.9kohm+1nF
uint16_t TIC_ValueCopy;
uint16_t TIC_ValueOld; // old not filtered TIC_value

int32_t TIC_ValueFiltered; // pre-filtered TIC value
int32_t TIC_ValueFilteredOld; // old filtered value
int32_t TIC_ValueFilteredForPPS_lock; // pre-filtered value just for PPS lock
const uint8_t lockFilterConst = 16;

volatile uint32_t timer1CounterValue; // counts 10 MHz clock directly
uint32_t timer1CounterValueOld;

long timer_us; // timer1 value in microseconds offset from 1pps
long timer_us_old; // used for diff_ns
// timer_us is calculated, but not used for dac correction in this
// version of code
long diff_ns; // difference between old and new TIC_Value
long diff_ns_ForPPS_lock; // pre-filtered value just for PPS lock

long timeConst = 128; // Time constant in seconds
long timeConstOld = 128; // old Time constant
int filterDiv = 2; // filterConst = timeConst / filterDiv
long filterConst = 64; // pre-filter time const in secs (TIC-filtering)
long filterConstOld = 64; // old Filter time constant

float I_term; //for PI-loop
float P_term;
long I_term_long;
float I_term_remain;

long gain = 275;

// Arduino TIC calculation:
// VCO freq DAC bits per TIC bit (65536/VCOrange in ppb (eg. with 1nS/bit and 100ppb DACrange gives gain=655))
//
// PIC TIC calculation:
//     12 bit ADC, 1 MHz : 1000ns/4096 = 0.24 ns/bit, so now gain 
//     is 0.24*65536/VCO_ppb

float damping = 3.0; // Damping in loop

unsigned long time; //seconds since start
unsigned long timeOld; //last seconds since start
volatile bool ADCFlag = false; // set true every time ADC reads TIC
volatile bool SMTFlag = false;

#ifdef VENUS838T
extern uint8_t venusElevMask;
extern uint8_t venusMode;
extern uint8_t venusSurveyCnt[8];
extern void setVenusElevMask(uint8_t x);
#endif

float quantErr;
bool quantErrFlag;      // correct for gps time quantization error if available
int lockPPSlimit = 200; // if TIC filtered for PPS within +- this for lockPSfactor * timeConst = PPSlocked
                        // ~ 100 per 1000 of ADC range
int lockPPSfactor = 5;  // see above
unsigned long lockPPScounter; // counter for PPSlocked
bool PPSlocked; // prints 0 or 1

int i; // counter for 300secs before storing temp and dac readings average
int j; // counter for stored 300sec readings
int k; // counter for stored 3hour readings
unsigned int StoreTIC_A[NSTORE]; //300sec storage
unsigned int StoreDAC_A[NSTORE];
long sumTIC;
long sumTIC2;
long sumTemp;
long sumTemp2;
unsigned long sumDAC;
unsigned long sumDAC2;
unsigned int totalTime3h; // counter for power-up time updated every third hour

unsigned int ID_Number;

bool lessInfoDisplayed;
bool nsDisplayedDecimals;

SerialModes serialMode = on;
unsigned int holdValue; //DAC value for Hold mode
Modes opMode; //operating mode
Modes newMode; // used to reset timer_us when run is set and at to many missing PPS

// for TIC linearization
// RTC: best parameters found using a PicDiv
float TICmin = 60.0;
float TICmax = 4036.0;
long TIC_Offset = 2048; // ADC value for Reference time. This is chosen in the middle point of
// the ADC range.
float x3 = 0.03;
float x2 = 0.08;
float x1;
float TIC_Scaled;
float TIC_ValueCorr;
float TIC_ValueCorrOld;
float TIC_ValueCorrOffset;

const int dacSign = -1; // sign of OCXO EFC correction. +1 in original code, -1 if tuning is inverted
volatile uint16_t cnt; // counts measurements 65535 is max
volatile uint32_t elapsed;
volatile int totalErr; // total error count
uint16_t nCnt = 1000; // frequency counter gate time in seconds
int lostPPSCount;
const uint8_t nScreen = 5;
screenType screen;
uint8_t gpsTime[9]; // latest time from GPS
char errStr[8]; // displayed frequency error
char pwmStr[6]; // displayed pwm setting
uint8_t gpsSentence[256]; // one GPS NMEA line
uint8_t grid[7]; // 6 character grid square from GPS
uint8_t nsat[3]; // number of satellites received by GPS
uint8_t gpsCnt; // counts up to 256 to read one GPS line
uint8_t gpsCheckCnt; // counter for gps checksum
uint16_t gridCnt; // counter to periodically update grid 
const uint16_t gridFreq = 3000; // time in seconds to recheck grid square

volatile uint8_t button1State; // button 1. 0=not pushed 1=pushed
volatile uint8_t button2State; // button 2. 0=not pushed 1=pushed

void main(void) {
    // Initialize the device
    SYSTEM_Initialize();

    ADC_SetADIInterruptHandler(adcInterrupt);
    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global Interrupts
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    __delay_ms(1000);
    setup();
    counterSetup();
    lcdSetup();
    PWM1_16BIT_Enable();
    SMT1_DataAcquisitionEnable();
    
    INTERRUPT_GlobalInterruptEnable();
    
    while (1) {
        if (SMTFlag == true) {
            if (cnt >= nCnt) {
                counterUpdate();
            }
            SMTFlag = false;
        }
        if (ADCFlag == true) {
            calculation();
            ADCFlag = false;
            if (serialMode == on) printDataToSerial();
            lcdUpdate();
        }
        if (serialMode == on || serialMode == gps) getCommand();
        gpsRead();
        checkButtons();
    }
}

/* 
 Interrupt triggered when ADC conversion of TIC value is
 ready. This should always come after the SMT timer
 interrupt
 
 MCC generated function clears the interrupt bit
 */
void adcInterrupt() {
    TIC_Value = (uint16_t)((ADRESH << 8) + ADRESL);
    ADCFlag = true;
}

/*
   Interrupt triggered when SMT finishes measurement
   this is triggered every PPS pulse
 */
void SMT1_PW_ACQ_ISR(void) {
    timer1CounterValueOld = timer1CounterValue;
    timer1CounterValue = SMT1CPW;

    // handle counter rollover (SMT is 24 bit counter, value is held as
    // 32-bit unsigned)
    elapsed = timer1CounterValue - timer1CounterValueOld;
    if (timer1CounterValue < timer1CounterValueOld) {
      elapsed += 0xff000000;
    }
   
    totalErr += (elapsed - FREQ);
    cnt++;

    SMTFlag = true;
    PIR1bits.SMT1PWAIF = 0;
}

/*
  Update frequency counter. This does a simple count
  of the frequency over nCnt seconds.
 */
void counterUpdate(void) {

    // first count is invalid since timer1CounterValueOld undefined
    if (time < (warmUpTime + nCnt)) {
        cnt = 0;
        totalErr = 0;
        return;
    }
    // average error in Hz
    float err = totalErr / (float) nCnt;

    // construct frequency error string
    switch (nCnt) {
        case 10:
            sprintf(errStr, "%3.1f    ", err);
            break;
        case 100:
            sprintf(errStr, "%4.2f   ", err);
            break;
        case 1000:
            sprintf(errStr, "%5.3f  ", err);
            break;
        default:
            sprintf(errStr, "%6.4f ", err);
            break;
    }
    // replace leading zero with sign or blank
    // for zero
    if (totalErr == 0) {
        errStr[0] = 0x20; //' '
    } else if (totalErr > 0) {
        errStr[0] = 0x2b; //'+'
    } else {
        errStr[0] = 0x2d; //'-'
    }
    if (screen == home) {
        CLCD_SetPos(0, 0);
        CLCD_PutS((char *) errStr);
    }
    if (serialMode == on) printf("FREQ COUNTER ERROR = %f Hz\r\n", err);
    cnt = 0;
    totalErr = 0;

}

void calculation(void) {
    if ((timer1CounterValue - timer1CounterValueOld) == 0) {
        if (serialMode == on) printf("No PPS\r\n");
        lostPPSCount++;
        if (lostPPSCount > 20) {
            newMode = run;
        }
        if (lostPPSCount > 200) {
            lockPPScounter = 0;
        }

    } else {
        lostPPSCount = 0;
    }

    // TIC linearization
    TIC_ValueCopy = TIC_Value;
    x1 = (float) 1.0 - x3 - x2;
    TIC_Scaled = ((float) TIC_Offset - TICmin) / (TICmax - TICmin)*1000; // Scaling for TIC_Offset
    TIC_ValueCorrOffset = TIC_Scaled * x1 + TIC_Scaled * TIC_Scaled * x2 / (float) 1000.0 +
            TIC_Scaled * TIC_Scaled * TIC_Scaled * x3 / (float) 1000000.0;

    TIC_Scaled = ((float) TIC_ValueCopy - TICmin) / (TICmax - TICmin)*1000; // Scaling for TIC_Value
    TIC_ValueCorr = TIC_Scaled * x1 + TIC_Scaled * TIC_Scaled * x2 / (float) 1000.0 +
            TIC_Scaled * TIC_Scaled * TIC_Scaled * x3 / (float) 1000000.0;

    if (quantErrFlag) TIC_ValueCorr += quantErr;

    // timer_us
    // dt is in units of 10^-7 = 0.1 microseconds = 100 ns
    long dt = elapsed - FREQ;

    // timer_us is in microseconds
    if (quantErrFlag) {
        timer_us = timer_us - (dt * 100
                + (TIC_ValueCopy + (int) quantErr - TIC_ValueOld)*1000 / 4096) / 1000;
    } else {
        timer_us = timer_us - (dt * 100
                + (TIC_ValueCopy - TIC_ValueOld)*1000 / 4096) / 1000;
    }

    if (newMode == run) {
        // reset timer_us if change from hold mode to run mode
        timer_us = 0;
        timer_us_old = 0;
        TIC_ValueFilteredOld = TIC_Offset * filterConst;
        newMode = hold;
    }

    // reset in the beginning and end of warmup 
    if (time < 3 || (time > (warmUpTime - 1) && time < (warmUpTime + 1))) {
        timer_us = 0;
    }

    if ((labs(timer_us) - 2) > timeConst * 65536 / gain / 1000 && opMode == run && time > warmUpTime) {
        timer_us = 0;
        timer_us_old = 0;
        TIC_ValueFilteredOld = TIC_Offset * filterConst;
    }

    // Diff_ns
    if (TIC_ValueCorr > TIC_ValueCorrOld) {
        // = Frequency in ppb if updated every second! Note: TIC linearized
        diff_ns = (timer_us - timer_us_old)*1000 +
                (long) (TIC_ValueCorr - TIC_ValueCorrOld + 0.5);
    } else {
        diff_ns = (timer_us - timer_us_old)*1000 +
                (long) (TIC_ValueCorr - TIC_ValueCorrOld - 0.5);
    }

    // time - seconds since start
    if ((timer1CounterValue - timer1CounterValueOld) != 0) time++;

    ////// PPS locked

    // Low Pass Filter of TIC_Value for PPS lock  
    // /16 is used as 500ns error and /16 is about 30ns that seems reasonable
    TIC_ValueFilteredForPPS_lock = TIC_ValueFilteredForPPS_lock + (TIC_ValueCopy * lockFilterConst - TIC_ValueFilteredForPPS_lock) / lockFilterConst;

    // Low Pass Filter of diff_ns for PPS lock                                                                             
    diff_ns_ForPPS_lock = diff_ns_ForPPS_lock + (diff_ns * lockFilterConst - diff_ns_ForPPS_lock) / lockFilterConst;

    lockPPScounter = lockPPScounter + 1;
    if (labs(TIC_ValueFilteredForPPS_lock / lockFilterConst - TIC_Offset) > lockPPSlimit) {
        lockPPScounter = 0;
    }
    // if freq more than 20ppb wrong (had to add this to avoid certain combinations not covered by above)
    if (labs(diff_ns_ForPPS_lock / lockFilterConst) > 20) {
        lockPPScounter = 0;
    }
    
    if (lockPPScounter > timeConst * lockPPSfactor) {
        PPSlocked = 1;
        if (screen == home) {
            CLCD_SetPos(0, 12);
            CLCD_PutS("L");
        }
    } else {
        PPSlocked = 0;
        if (screen == home) {
            CLCD_SetPos(0, 12);
            if (time > warmUpTime) {
                CLCD_PutS(" ");
            } else {
                CLCD_PutS("W");
            }
        }
    }
    if (screen == home) {
        switch (time % 2) {
            case 0:
                CLCD_SetPos(0, 11);
                CLCD_PutS("|");
                break;
            case 1:
                CLCD_SetPos(0, 11);
                CLCD_PutS("-");
                break;
        }
    }
     
    // set filter constant
    filterConst = timeConst / filterDiv;
    if (filterConst < 1) {
        filterConst = 1;
    } else if (filterConst > 1024) {
        filterConst = 1024;
    }
    if (PPSlocked == 0 || opMode == hold) filterConst = 1;

    // recalculation of value  
    if (timeConst != timeConstOld) {
        dacValue = dacValue / timeConstOld * timeConst;
    }

    if (filterConst != filterConstOld) {
        TIC_ValueFilteredOld = TIC_ValueFilteredOld / filterConstOld * filterConst;
        TIC_ValueFiltered = TIC_ValueFiltered / filterConstOld * filterConst;
    }

    // Low Pass Filter for TICvalue (Phase Error)
    // Remember that TIC_ValueFiltered is multiplied by filterConst

    // Don´t update if outlier. Accepts diff_ns less than same ns as vco range in ppb + 200ns
    // First check to avoid overflow in next calculation (also max VCO range is about 6500ns/s)

    // RTC changes: ignore large TIC changes if locked.
    if (abs(diff_ns < 260))
        // if (PPSlocked==0 || (PPSlocked==1 && diff_ns < 260))
    {
        if (labs(diff_ns * gain) < (65535 + 200 * gain)) {
            if (quantErrFlag) {
                TIC_ValueFiltered = TIC_ValueFiltered +
                        ((timer_us * 1000 +
                        TIC_ValueCopy + (int) quantErr) * filterConst - TIC_ValueFiltered + (filterConst / 2)) / filterConst;
            } else {
                TIC_ValueFiltered = TIC_ValueFiltered +
                        ((timer_us * 1000 +
                        TIC_ValueCopy) * filterConst - TIC_ValueFiltered + (filterConst / 2)) / filterConst;
            }
        }
    }

    // Don't change DAC-value during warm-up time or if in hold mode
    if (time > (warmUpTime + 1) && opMode == run) {
        ////// PI-loop ///////
        // remember /timeConst is done before dacValue is sent out
        P_term = (TIC_ValueFiltered - TIC_Offset * filterConst) / (float) (filterConst) * gain;

        I_term = P_term / damping / (float) (timeConst) + I_term_remain;
        I_term_long = (long) (I_term);
        I_term_remain = I_term - I_term_long;
        dacValue += dacSign * I_term_long;
        dacValueOut = dacValue + dacSign * P_term;
    } else {
        dacValueOut = dacValue; // No change
    }
    // Check that dacvalue is within limits
    if (dacValue < 0) {
        dacValue = 0;
    }
    if (dacValue > (65535 * timeConst)) {
        dacValue = (65535 * timeConst);
    }

    dacValueOut /= timeConst; // PWM-DAC value
    if (dacValueOut < 0) {
        dacValueOut = 0;
    }
    if (dacValueOut > 65535) {
        dacValueOut = 65535;
    }

    // manual set of dacvalue if in hold and not 0, if zero hold old value
    if (holdValue > 0 && opMode == hold) {
        dacValueOut = holdValue;
    }

    // Set 16bit DAC
    PWM1_16BIT_SetSlice1Output1DutyCycleRegister((uint16_t) dacValueOut);
    PWM1_16BIT_LoadBufferRegisters();

    sprintf(pwmStr, "%4x", (uint16_t) dacValueOut);
    if (screen == home) {
        CLCD_SetPos(0, 7);
        CLCD_PutS((char*) pwmStr);
    }

    //Storage of average readings that is later printed
    sumTIC = sumTIC + (TIC_ValueCopy * 10);
    sumDAC = sumDAC + dacValueOut;
    i = i + 1;
    if (i == 300) // 300sec
    {
        if (opMode == run) {
            StoreTIC_A[j] = sumTIC / i;
        } else {
            StoreTIC_A[j] = TIC_ValueCopy;
        }
        sumTIC2 = sumTIC2 + sumTIC / i;
        sumTIC = 0;
        if (opMode == run) {
            StoreDAC_A[j] = sumDAC / i;
        } else {
            StoreDAC_A[j] = ((FREQ - 1) - timer1CounterValue);
        }

        sumDAC2 = sumDAC2 + sumDAC / i;
        sumDAC = 0;
        i = 0;
        j = j + 1;
        if (j % 36 == 0) // store every 36 x 300sec (3 hours)
        {
            sumTIC2 = sumTIC2 / 36;
            if (opMode == run) {
                DATAEE_WriteByte((uint16_t) k, highByte((uint16_t) sumTIC2));
                DATAEE_WriteByte((uint16_t) (k + NSTORE), lowByte((uint16_t) sumTIC2));
            } else {
                DATAEE_WriteByte((uint16_t) k, highByte((uint16_t) TIC_ValueCopy));
                DATAEE_WriteByte((uint16_t) (k + NSTORE), lowByte((uint16_t) TIC_ValueCopy));
            }
            sumTIC2 = 0;

            sumDAC2 = sumDAC2 / 36;

            if (opMode == run) {
                DATAEE_WriteByte((uint16_t) (k + 2*NSTORE), highByte((uint16_t) sumDAC2));
                DATAEE_WriteByte((uint16_t) (k + 3*NSTORE), lowByte((uint16_t) sumDAC2));
            } else {
                DATAEE_WriteByte((uint16_t) (k + 2*NSTORE), highByte((uint16_t) (((FREQ - 1) - timer1CounterValue))));
                DATAEE_WriteByte((uint16_t) (k + 3*NSTORE), lowByte((uint16_t) ((FREQ - 1) - timer1CounterValue)));
            }

            if (opMode == run && lockPPScounter > 10800) {
                DATAEE_WriteByte(1017, highByte((uint16_t) sumDAC2));
                DATAEE_WriteByte(1018, lowByte((uint16_t) sumDAC2));
            }

            sumDAC2 = 0;

            if (j == NSTORE) // 144 x 300sec (12 hours)
            {
                j = 0;
            }
            k = k + 1;
            if (k == NSTORE) // 144 x 10800sec (18 days)
            {
                k = 0;
            }
            DATAEE_WriteByte(1023, (uint8_t) k); // store present k (index of 3 hour average, used in setup)

            totalTime3h = totalTime3h + 1;
            DATAEE_WriteByte(993, highByte(totalTime3h));
            DATAEE_WriteByte(994, lowByte(totalTime3h));
        }
    }

    // storage of old parameters
    TIC_ValueOld = TIC_ValueCopy;
    if (quantErrFlag) TIC_ValueOld += (int) quantErr;
    TIC_ValueCorrOld = TIC_ValueCorr;
    timer_us_old = timer_us;
    timeConstOld = timeConst;
    filterConstOld = filterConst;
    timeOld = time;
    TIC_ValueFilteredOld = TIC_ValueFiltered;
}

void getCommand(void) {
    uint8_t ch;
    long z;

    enum Command { // this is the command set
        a = 'a', A = 'A', // set damping
        b = 'b', B = 'B', // toggle gps error correction on "b1", off "b0"
        c = 'c', C = 'C', // set frequency counter time
        d = 'd', D = 'D', // set dacvalue (followed by a value)
        e = 'e', E = 'E', // erase (followed by a value)
        f = 'f', F = 'F', // help (followed by a value)
        g = 'g', G = 'G', // gain (followed by new value)
        h = 'h', H = 'H', // hold (followed by a DAC value note: 0 will set only hold)
        i = 'i', I = 'I', // toggles less or more info
        l = 'l', L = 'L', // set TIC linearization parameters min (enter value*10))
        m = 'm', M = 'M', // set TIC linearization parameters max (enter value*10)
        n = 'n', N = 'N', // set ID number
        o = 'o', O = 'O', // TIC_Offset (followed by new value)
        p = 'p', P = 'P', // set prefilter div
        q = 'q', Q = 'Q', // set TIC linearization parameters square (enter value*100))
        r = 'r', R = 'R', // run
        s = 's', S = 'S', // save (followed by a value)
        t = 't', T = 'T', // time const (followed by new value)
        u = 'u', U = 'U', // set TIC linearization parameters cube (enter value*100))
        w = 'w', W = 'W', // set warmup time (to allow for warm up of oscillator)
#ifdef VENUS838T
        x = 'x', X = 'X',  // set GPS elevation mask
#endif        
        y ='y', Y = 'Y' // set frequency counter gate time
    };

    //process if something is there
    if (UART1_is_rx_ready()) {
        ch = UART1_Read();
        // reject if not letter or number
        if (( ch < 48 ) || (ch > 122)) return;
        
        switch (ch) {

            case a: // set damping command
            case A:
                z = parseInt(); //needs new line or carriage return set in Arduino serial monitor
                if (z >= 50 && z <= 1000) {
                    damping = z / (float) 100.0;
                    printf("Damping %f\r\n", damping);
                } else {
                    printf("Not a valid damping value - Shall be between 50 and 1000\r\n");
                }
                break;

            case b: // gps error correction
            case B:
                z = parseInt();
#ifdef VENUS838T
                if (z == 0) {
                    quantErrFlag = false;
                    printf("Error correction off\r\n");
                } else {
                    quantErrFlag = true;
                    printf("Error correction on\r\n");
                }
#endif
                break;
            case c:
            case C:
                z = parseInt();
                if (z >= 1 && z <= 10000) {
                    nCnt = (int) z;
                    printf("Counter time %d\r\n", nCnt);
                }
                break;

            case d: // set dacValue command
            case D:
                z = parseInt();
                if (z >= 1 && z <= 65535) {
                    dacValue = z * timeConst;
                    printf("dacValue %ld\r\n", z);
                } else {
                    printf("Not a valid dacValue - Shall be between 1 and 65535\r\n");
                }
                break;

            case e: // erase command
            case E:
                z = parseInt();
                switch (z) {

                    case 1:
                        printf("Erase 3h storage in EEPROM \r\n");
                        for (int i = 0; i < 864; i++) {
                            DATAEE_WriteByte((uint16_t) i, 0);
                        }
                        DATAEE_WriteByte(1023, 0);
                        k = 0; //reset 3hours counter
                        break;

                    case 22:
                        printf("Erase all EEPROM to zero\r\n");
                        for (int i = 0; i < 1024; i++) {
                            DATAEE_WriteByte((uint16_t) i, 0);
                        }
                        k = 0; //reset 3hours counter
                        break;

                    case 33:
                        printf("Erase all EEPROM to -1\r\n");
                        for (int i = 0; i < 1024; i++) {
                            DATAEE_WriteByte((uint16_t) i, 255);
                        }
                        k = 0; //reset 3hours counter
                        break;

                    default:
                        printf("Not a valid value for erase - Shall be 1, 22, or 33\r\n");
                }

                break;

            case f: // help command
            case F:
                z = parseInt();

                switch (z) {

                    case 1:
                        printf("\r\n");
                        printHeader1_ToSerial();
                        printf("\t");
                        printHeader2_ToSerial();
                        printf("\r\n\r\nInfo and help - To get values for gain etc type f2 <enter> and f4 <enter> EEPROM\r\n");
                        printf("\r\nTyping a<value><enter> will set a new damping between between 0.50 and 10.00 set 50 to 1000\r\n");
                        printf("Typing b<value><enter> turns GPS error correction on (1) or off (0)\r\n");
                        printf("Typing d<value><enter> will set a new dacValue between 1 and 65535\r\n");
                        printf("Typing e<value><enter> will erase the 3 hour storage in EEPROM if value 1 and all EEPROM if 22 (33 sets all EEPROM to FF)\r\n");
                        printf("Typing g<value><enter> will set a new gain between 10 and 65535\r\n");
                        printf("  gain = (65536/settable VCOrange in ppb) (eg. 100ppb DACrange gives gain=655)\r\n");
                        printf("Typing h<value><enter> will set hold mode and the entered dacValue if not h0 that uses the old\r\n");
                        printf("Typing i<value><enter> with value 1 will toggle ns decimal point else will toggle amount of information \r\n");
                        printf("Typing l<value><enter> sets the TIC linearization min (enter value*10)\r\n");
                        printf("Typing m<value><enter> sets the TIC linearization max (enter value*10)\r\n");
                        printf("Typing q<value><enter> sets the TIC linearization square (enter value*10000)\r\n");
                        printf("Typing u<value><enter> sets the TIC linearization cube (enter value*10000)\r\n");
                        printf("Typing n<value><enter> will set ID number 0-65535 that is displayed \r\n");
                        printf("Typing o<value><enter> will set a new TIC_Offset between (200-4090)\r\n");
                        printf("Typing p<value><enter> will set a new prefilter div between 2 and 4\r\n");
                        printf("Typing r<enter> will set run mode\r\n");
                        printf("Typing s<value><enter> will save gain etc to EEPROM if value 1 and dacvalue if 2\r\n");
                        printf("Typing t<value><enter> will set a new time constant; must be power of 2 between 4 and 1024 seconds\r\n");
                        printf("Typing w<value><enter> will set a new warmup time between 2 and 1000 seconds\r\n");
#ifdef VENUS838T
			printf("Typing x<value><enter> will set the GPS elevation mask between 5 and 85 degrees\r\n");
#endif
                        printf("Typing y<value><enter> will set the frequency counter gate time; possible values 10, 100, 1000, or 10000 seconds\r\n");
                        printf("\r\n");
                        printHeader3_ToSerial();
                        break;

                    case 2:
                        printf("\r\n");
                        printf("Gain               %ld\r\n", gain);
                        printf("Damping            %f\r\n", damping);
                        printf("TimeConst          %ld\r\n", timeConst);
                        printf("FilterDiv          %d\r\n", filterDiv);
                        printf("TIC_Offset         %ld\r\n", TIC_Offset);
                        printf("TICmin             %f\r\n", TICmin);
                        printf("TICmax             %f\r\n", TICmax);
                        printf("Square comp        %f\r\n", x2);
                        printf("Cube   comp        %f\r\n", x3);
                        printf("Warm up time       %u\r\n", warmUpTime);
                        printf("LockPPScounter     %ld\r\n", lockPPScounter);
                        printf("FreqCount          %u\r\n", nCnt);
                        printf("ID_Number          %u\r\n", ID_Number);
                        printf("Total hours        %u\r\n", totalTime3h * 3);
#ifdef VENUS838T
                        printf("Venus 838LPx-T GPS\r\n  mode = ");
                        switch (venusMode) {
                            case 0:
                                printf(" PVT\r\n");
                                break;
                            case 1:
                                printf(" Survey: count = %7s\r\n", venusSurveyCnt);
                                break;
                            case 2:
                                printf(" Static\r\n");
                                break;
                        }
                        printf("  error correction:");
                        if (quantErrFlag) {
                            printf(" on\r\n");
                        } else {
                            printf(" off\r\n");
                        }
                        printf("  elevation mask: %2u deg\r\n", venusElevMask);
#endif
                        printf("\r\n");
                        printHeader3_ToSerial();
                        break;

                    case 4:
                        printf("\r\nEEPROM content: \r\ntotalTime3h = ");
                        z = (DATAEE_ReadByte(993)*256 + DATAEE_ReadByte(994));
                        printf("%ld\r\n", z);
                        z = DATAEE_ReadByte(995);
                        printf("Use quantization error = %ld\r\n", z);
                        z = DATAEE_ReadByte(996);
                        printf("Elevation mask = %ld\r\n", z);
                        printf("ID_Number = ");
                        z = (DATAEE_ReadByte(997)*256 + DATAEE_ReadByte(998));
                        printf("%ld\r\n", z);
                        printf("TICmin = ");
                        z = (DATAEE_ReadByte(999)*256 + DATAEE_ReadByte(1000));
                        printf("%ld\r\n", z);
                        printf("TICmax = ");
                        z = (DATAEE_ReadByte(1001)*256 + DATAEE_ReadByte(1002));
                        printf("%ld\r\n", z);
                        printf("x2 = ");
                        z = (DATAEE_ReadByte(1003)*256 + DATAEE_ReadByte(1004));
                        printf("%ld\r\n", z);
                        printf("TIC_Offset = ");
                        z = (DATAEE_ReadByte(1005)*256 + DATAEE_ReadByte(1006));
                        printf("%ld\r\n", z);
                        printf("filterDiv = ");
                        z = (DATAEE_ReadByte(1007)*256 + DATAEE_ReadByte(1008));
                        printf("%ld\r\n", z);
                        printf("warmUpTime = ");
                        z = (DATAEE_ReadByte(1009)*256 + DATAEE_ReadByte(1010));
                        printf("%ld\r\n", z);
                        printf("damping = ");
                        z = (DATAEE_ReadByte(1011)*256 + DATAEE_ReadByte(1012));
                        printf("%ld\r\n", z);
                        printf("dacValueOut = ");
                        z = (DATAEE_ReadByte(1017)*256 + DATAEE_ReadByte(1018));
                        printf("%ld\r\n", z);
                        printf("gain = ");
                        z = (DATAEE_ReadByte(1019)*256 + DATAEE_ReadByte(1020));
                        printf("%ld\r\n", z);
                        printf("timeConst = ");
                        z = (DATAEE_ReadByte(1021)*256 + DATAEE_ReadByte(1022));
                        printf("%ld\r\n", z);
                        printf("k = %d\r\n", DATAEE_ReadByte(1023));
                        printf("counter = ");
                        z = (DATAEE_ReadByte(1013)*256 + DATAEE_ReadByte(1014));
                        printf("%ld\r\n", z);
                        printf("\r\n");
                        break;

                    default:
                        printf("Not a valid value for help - Shall be 1 to 4\r\n");
                }
                break;

            case g: // gain command
            case G:
                z = parseInt();
                if (z >= 10 && z <= 65534) {
                    gain = z;
                    printf("Gain ");
                    printf("%ld\r\n", z);
                } else {
                    printf("Not a valid gain value - Shall be between 10 and 65534\r\n");
                }
                break;

            case h: // hold command
            case H:
                z = parseInt();
                if (z >= 0 && z <= 65535) {
                    opMode = hold;
                    newMode = hold;
                    printf("Hold ");
                    holdValue = (unsigned int) z;
                    printf("%u\r\n", holdValue);
                } else {
                    printf("Not a valid holdValue - Shall be between 0 and 65535\r\n");
                }

                break;

            case i: // help command
            case I:
                z = parseInt();
                if (z == 1) {
                    nsDisplayedDecimals = !nsDisplayedDecimals;
                } else {
                    lessInfoDisplayed = !lessInfoDisplayed;
                }
                break;
            case l:
            case L: // set TIC linearization minimum
                z = parseInt();
                TICmin = z / (float) 10.0;
                printf("TICmin ");
                printf("%f\r\n", TICmin);
                break;
            case m:
            case M: // set TIC linearization maximum
                z = parseInt();
                TICmax = z / (float) 10.0;
                printf("TICmax ");
                printf("%f\r\n", TICmax);
                break;
            case q:
            case Q: // set TIC linearization square compensation
                z = parseInt();
                x2 = z / (float) 10000.0;
                printf("x2 ");
                printf("%f\r\n", x2);
                break;
            case u:
            case U: // set TIC linearization cube compensation
                z = parseInt();
                x3 = z / (float) 10000.0;
                printf("x3 ");
                printf("%f\r\n", x3);
                break;
            case n: // ID_number      
            case N:
                z = parseInt();
                if (z >= 0 && z <= 65534) {
                    ID_Number = (unsigned int) z;
                    printf("ID_Number ");
                    printf("%ld\r\n", z);
                } else {
                    printf("Not a valid ID_Number value - Shall be between 0 and 65534\r\n");
                }
                break;

            case o: // TIC_Offset command
            case O:
                z = parseInt();
                if (z >= 200 && z <= 4090) {
                    TIC_Offset = z;
                    printf("TIC_Offset ");
                    printf("%ld\r\n", z);
                } else {
                    printf("Not a valid TIC_offset - Shall be between 200 and 4090\r\n");
                }
                break;

            case p: // set prefilter div command
            case P:
                z = parseInt();
                if (z >= 2 && z <= 4) {
                    filterDiv = (int) z;
                    printf("Prefilter div ");
                    printf("%ld\r\n", z);
                } else {
                    printf("Not a valid prefilter value - Shall be between 2 and 4");
                }
                break;

            case r: // run command
            case R:
                printf("Run\r\n");
                opMode = run;
                newMode = run;
                break;

            case s: // save command
            case S:
                z = parseInt();
                switch (z) {

                    case 1:
                        printf("Saved Gain and TimeConstant etc");
                        if (quantErrFlag) {
                            DATAEE_WriteByte(995, 1);
                        } else {
                            DATAEE_WriteByte(995, 0);
                        }
#ifdef VENUS838T
                        DATAEE_WriteByte(996, venusElevMask);
#endif                        
                        DATAEE_WriteByte(997, highByte(ID_Number));
                        DATAEE_WriteByte(998, lowByte(ID_Number));
                        DATAEE_WriteByte(999, highByte((uint16_t) (TICmin * 10.0)));
                        DATAEE_WriteByte(1000, lowByte((uint16_t) (TICmin * 10.0)));
                        DATAEE_WriteByte(1001, highByte((uint16_t) (TICmax)));
                        DATAEE_WriteByte(1002, lowByte((uint16_t) (TICmax)));
                        DATAEE_WriteByte(1003, highByte((uint16_t) (x2 * 1000.0)));
                        DATAEE_WriteByte(1004, lowByte((uint16_t) (x2 * 1000.0)));
                        DATAEE_WriteByte(1005, highByte((uint16_t) TIC_Offset));
                        DATAEE_WriteByte(1006, lowByte((uint16_t) TIC_Offset));
                        DATAEE_WriteByte(1007, highByte((uint16_t) filterDiv));
                        DATAEE_WriteByte(1008, lowByte((uint16_t) filterDiv));
                        DATAEE_WriteByte(1009, highByte((uint16_t) warmUpTime));
                        DATAEE_WriteByte(1010, lowByte((uint16_t) warmUpTime));
                        DATAEE_WriteByte(1011, highByte((uint16_t) (damping * 100)));
                        DATAEE_WriteByte(1012, lowByte((uint16_t) (damping * 100)));
                        DATAEE_WriteByte(1013, highByte((uint16_t) nCnt));
                        DATAEE_WriteByte(1014, lowByte((uint16_t) nCnt));
                        DATAEE_WriteByte(1015, highByte((uint16_t) (x3 * 1000.0)));
                        DATAEE_WriteByte(1016, lowByte((uint16_t) (x3 * 1000.0)));
                        DATAEE_WriteByte(1019, highByte((uint16_t) gain));
                        DATAEE_WriteByte(1020, lowByte((uint16_t) gain));
                        DATAEE_WriteByte(1021, highByte((uint16_t) timeConst));
                        DATAEE_WriteByte(1022, lowByte((uint16_t) timeConst));
                        printf("\r\n");
                        break;

                    case 2:
                        printf("Saved DacValue");
                        DATAEE_WriteByte(1017, highByte((uint16_t) dacValueOut));
                        DATAEE_WriteByte(1018, lowByte((uint16_t) dacValueOut));
                        printf("\r\n");
                        break;

                    default:
                        printf("Not a valid value for save - Shall be 1 or 2\r\n");
                }
                break;

            case t: // time constant command
            case T:
                z = parseInt();
                // must match with possible values allowed by LCD/button interface
                switch (z) {
                    case 2:
                    case 4:
                    case 8:
                    case 16:
                    case 32:
                    case 64:
                    case 128:
                    case 256:
                    case 512:
                    case 1024:
                        timeConst = z;
                        printf("time constant ");
                        printf("%ld\r\n", z);
                        if (screen == sett) {
                            CLCD_SetPos(0, 7);
                            switch (timeConst) {
                                case 4:
                                    CLCD_PutS("4   ");
                                    break;
                                case 8:
                                    CLCD_PutS("8   ");
                                    break;
                                case 16:
                                    CLCD_PutS("16  ");
                                    break;
                                case 32:
                                    CLCD_PutS("32  ");
                                    break;
                                case 64:
                                    CLCD_PutS("64  ");
                                    break;
                                case 128:
                                    CLCD_PutS("128 ");
                                    break;
                                case 256:
                                    CLCD_PutS("256 ");
                                    break;
                                case 512:
                                    CLCD_PutS("512 ");
                                    break;
                                case 1024:
                                    CLCD_PutS("1024");
                                    break;
                            }
                        }
                        break;
                    default:      
                        printf("Not a valid time constant - Shall be one of 4, 8, 16, 32, 64, 128, 256, 512, or 1024\r\n");
                }
                break;


            case w: // set warm up time command
            case W:
                z = parseInt();
                if (z >= 2 && z <= 1000) {
                    warmUpTime = (uint16_t) z;
                    printf("Warmup time ");
                    printf("%ld\r\n", z);
                } else {
                    printf("Not a valid warmup time - Shall be between 2 and 1000\r\n");
                }
                break;
#ifdef VENUS838T
            case x: // elevation mask
            case X:
                z = parseInt();
                if (z >= 5 && z <= 85) {
                    venusElevMask = (uint8_t) z;
                    printf("Elevation mask ");
                    printf("%ld\r\n", z);
                    setVenusElevMask(z);
                } else {
                    printf("Not a valid elevation - Shall be between 5 and 85 degrees\r\n");
                }
                break;
#endif
            case y: // freq counter gate time
            case Y:
                z = parseInt();
                switch (z) {
                    case 10:
                    case 100:
                    case 1000:
                    case 10000:
                        nCnt = z;
                        if (screen == setncnt) {
                            CLCD_SetPos(0, 5);
                            switch (nCnt) {
                            case 10:
                                CLCD_PutS("10   ");
                                break;
                            case 100:
                                CLCD_PutS("100  ");
                                break;
                            case 1000:
                                CLCD_PutS("1000 ");
                                break;
                            case 10000:
                                CLCD_PutS("10000");
                                break;
                            }
                        }
                        printf("Frequency counter gate time %ld\r\n", z);
                        cnt = 0;
                        totalErr = 0;
                        break;
                    default:
                        printf("Not a valid frequency counter time. Must be 10, 100, 1000, or 10000 seconds\r\n");
                }
                break;

            default:
                printf("Not valid command <%c>\r\n",ch);
                break;
        };

        while (UART1_is_rx_ready()) {
            ch = UART1_Read(); //flush rest of line
        }
    }
}

void printDataToSerial(void) {
    printf("%lu \t", time);
    if (time > warmUpTime) {
        if (nsDisplayedDecimals == false) {
            printf("%.0f",
                    ((float)timer_us *1000) + 
                    TIC_ValueCorr - TIC_ValueCorrOffset);
            printf("\t");
        } else {
            printf("%.1f",
                    ((float)timer_us *1000) + 
                    TIC_ValueCorr - TIC_ValueCorrOffset);
            printf("\t");
        }
    }
    printf("%ld\t", dacValueOut);

    if (time > warmUpTime && opMode == run) {
        if (PPSlocked == 0) {
            printf("NoLock");
        } else {
            printf("Locked");
        }
        printf("\t");
    } else if (time > warmUpTime) {
        printf("Hold\t");
    } else {
        printf("WarmUp\r\n");
        return;
    }

    if (lessInfoDisplayed == false) {
        printf("%ld\t", diff_ns);
#ifdef VENUS838T
        printf("%.1f", quantErr);
        printf("\t");
#endif
        printf("%ld\t", TIC_ValueFiltered * 10 / filterConst);
        printf("%ld\t", timeConst);
        printf("%ld\t", filterConst);
        
        if (i == 1) {
            printf("Five minute averages: TIC+DAC\t");
        }
        if (i == 2) {
            printf("Now acquiring value: %d\t", j);
        }
        if ((i >= 4) && (i <= 147)) {
            printf("%d\t", (i - 4));
            printf("%d\t", (StoreTIC_A[i - 4]));
            printf("%u\t", (StoreDAC_A[i - 4]));
        }
        if (i == 148) {
            printf("Three hour averages: TIC+DAC\t");
        }
        if (i == 149) {
            printf("Now acquiring value: %d\t", k);
        }
        if ((i >= 150) && (i <= 293)) {
            printf("%d\t", (i - 150 + 1000));
            printf("%d\t", (DATAEE_ReadByte((uint16_t) (i - 150 + 0))*256 + DATAEE_ReadByte((uint16_t) (i - 150 + NSTORE))));
            unsigned int x = DATAEE_ReadByte((uint16_t) (i - 150 + 2*NSTORE))*256 + DATAEE_ReadByte((uint16_t) (i - 150 + 3*NSTORE));
            printf("%u\t", x);
	    x = DATAEE_ReadByte((uint16_t) (i-150+4*NSTORE))*256 + DATAEE_ReadByte((uint16_t) i-150+5*NSTORE);
            int y = x / 10240;
            switch (y) {
                case 0:
                    if (x > 0) {
                        printf("Hold");
                    }
                    break;
                case 1:
                    printf("Restarted+hold");
                    break;
                case 2:
                    printf("noLock");
                    break;
                case 3:
                    printf("Restarted");
                    break;
                case 4:
                    printf("Locked");
                    break;
            }
            printf("\t");
        }
        if (i == 295) {
            printf("TimeConst = %ld sec\t", timeConst);
        }
        if (i == 296) {
            printf("Prefilter = %ld sec\t", filterConst);
        }
        if (i == 297) {
	  printf("Damping = %4.1f Gain = %ld\t", damping, gain);
        }
        if (i == 298) {
            printf("Type f1<enter> to get help+info\t");
        }
        if (i == 299) {
            printHeader2_ToSerial();
        }

    } // end of If (lessInfoDisplayed) 
    printf("\r\n");

}

////////////////////////////////////////////////////////////////////////////////////////////////

void printHeader1_ToSerial(void) {
    printf("GPSDO with 1ns TIC by Lars Walenius\r\n");
    printf("PIC18 version by R. Torsten Clay N4OGW\r\n");
    printf("\r\n");
}

////////////////////////////////////////////////////////////////////////////////////////////////

void printHeader2_ToSerial(void) {
    printf("Version 05/20/2023  ID: %d\t", ID_Number);
}

////////////////////////////////////////////////////////////////////////////////////////////////

void printHeader3_ToSerial(void) {
    printf("time\tns\tdac\tstatus\tdiff_ns\t");
#ifdef VENUS838T
    printf("gps err");
#endif
    printf("\tfiltX10\ttc\tfilt\r\n");
}

void counterSetup(void) {
    cnt = 0;
    totalErr = 0;
    timer1CounterValue = 0;
}

void lcdSetup(void) {
    sprintf((char*) gpsTime, "--:--:--");
    sprintf((char*) grid, "EM53nk");
    errStr[6] = 0;
    errStr[1] = 0x2e; // '.'
    gridCnt = 0;
    gpsCnt = 0;
    gpsCheckCnt = 0;
    screen = home;
    CLCD_Initialize();
    CLCD_SetPos(0, 7);
    CLCD_PutS((char*) pwmStr);
    CLCD_SetPos(1, 0);
    CLCD_PutS((char*) gpsTime);
    CLCD_SetPos(1, 9);
    CLCD_PutS((char*) grid);
}

void setup(void) {
    // Print info and header in beginning
    if (serialMode == on) {
        printf("\r\n\r\n\r\n");
        printHeader1_ToSerial();
        printf("\t"); // prints a tab
    }
    time = 0;
    lostPPSCount = 0;
#ifdef VENUS838T
    venusElevMask = 25;
#endif    
    quantErrFlag = false;
    
    // Read data from EEPROM to variables
    unsigned int y;
    y = DATAEE_ReadByte(993)*256 + DATAEE_ReadByte(994);
    if ((y > 0) && (y < 65535)) // set last stored xx if not 0 or 65535
    {
        totalTime3h = y;
    }
    if (DATAEE_ReadByte(995) > 0) quantErrFlag = true;
    else quantErrFlag = false;
#ifdef VENUS838T
    venusElevMask = DATAEE_ReadByte(996);
    if (venusElevMask<5 || venusElevMask>85) venusElevMask = 20;
#endif    
    y = DATAEE_ReadByte(997)*256 + DATAEE_ReadByte(998);
    if ((y > 0) && (y < 65535)) // set last stored xx if not 0 or 65535
    {
        ID_Number = y;
    }
    y = DATAEE_ReadByte(999)*256 + DATAEE_ReadByte(1000);
    if ((y > 0) && (y < 65535)) // set last stored xx if not 0 or 65535
    {
        TICmin = y / (float) 10.0;
    }
    y = DATAEE_ReadByte(1001)*256 + DATAEE_ReadByte(1002);
    if ((y > 0) && (y < 65535)) // set last stored xx if not 0 or 65535
    {
        TICmax = y;
    }
    y = DATAEE_ReadByte(1003)*256 + DATAEE_ReadByte(1004);
    if ((y > 0) && (y < 65535)) // set last stored xx if not 0 or 65535
    {
        x2 = y / (float) 1000.0;
    }
    y = DATAEE_ReadByte(1005)*256 + DATAEE_ReadByte(1006);
    if ((y > 0) && (y < 65535)) // set last stored xx if not 0 or 65535
    {
        TIC_Offset = y;
    }
    y = DATAEE_ReadByte(1007)*256 + DATAEE_ReadByte(1008);
    if ((y > 0) && (y < 65535)) // set last stored xx if not 0 or 65535
    {
        filterDiv = (int) y;
    }
    y = DATAEE_ReadByte(1009)*256 + DATAEE_ReadByte(1010);
    if ((y > 0) && (y < 65535)) // set last stored xx if not 0 or 65535
    {
        warmUpTime = y;
    }
    y = DATAEE_ReadByte(1011)*256 + DATAEE_ReadByte(1012);
    if ((y > 0) && (y < 65535)) // set last stored xx if not 0 or 65535
    {
        damping = y / (float) 100.0;
    }
    y = DATAEE_ReadByte(1013)*256 + DATAEE_ReadByte(1014);
    if ((y >= 10) && (y <= 10000)) {
        nCnt = (uint16_t) y;
    }
    y = DATAEE_ReadByte(1015)*256 + DATAEE_ReadByte(1016);
    if ((y > 0) && (y < 65535)) // set last stored xx if not 0 or 65535
    {
        x3 = y / (float) 1000.0;
    }
    y = DATAEE_ReadByte(1017)*256 + DATAEE_ReadByte(1018);
    if ((y > 0) && (y < 65535)) // set last stored dacValueOut if not 0 or 65535
    {
        dacValueOut = y;
    }
    y = DATAEE_ReadByte(1019)*256 + DATAEE_ReadByte(1020);
    if ((y >= 10) && (y <= 65534)) // set last stored gain if between 10 and 65534
    {
        gain = y;
    }
    y = DATAEE_ReadByte(1021)*256 + DATAEE_ReadByte(1022);
    if ((y >= 4) && (y <= 32000)) // set last stored timeConst if between 4 and 32000
    {
        timeConst = y;
    }
    k = DATAEE_ReadByte(1023); // last index of stored 3 hour average
    if ((k >= NSTORE) || (k < 0)) k = 0; //reset if k is invalid (eg with a new processor)

    // Set "16bit DAC"  
    PWM1_16BIT_SetSlice1Output1DutyCycleRegister((uint16_t) dacValueOut);
    PWM1_16BIT_LoadBufferRegisters();

    // Set initial values  
    dacValue = dacValueOut * timeConst;
    timeConstOld = timeConst;
    filterConstOld = filterConst;
    TIC_ValueFilteredOld = TIC_Offset * filterConst;
    TIC_ValueFiltered = TIC_Offset * filterConst;
    TIC_ValueFilteredForPPS_lock = TIC_Offset * lockFilterConst;
    nsDisplayedDecimals = true;
    opMode = run;
    newMode = hold;

    //clear  PPS flag and go on to main loop  
    ADCFlag = false;

    button1State = 0;
    button2State = 0;
    TMR6_SetInterruptHandler(buttonInterrupt);

    gpsConfig();

    // Print info and header in beginning
    if (serialMode == on) {
        printHeader2_ToSerial();
        printf("\r\nType f1+enter to get help\r\n");
        printHeader3_ToSerial();
    }
}


// reads an integer from UART1; terminated by newline

long parseInt(void) {
    static uint8_t buff[32];
    uint8_t ptr;
    long val;
    uint8_t c;

    // read until newline or cr
    ptr = 0;
    while (!UART1_is_rx_ready());
    c = UART1_Read();
    while ((c != 0x0d) && (c != 0x0a)) {
        buff[ptr++] = c;
        if (ptr == 32) ptr = 0;
        while (!UART1_is_rx_ready());
        c = UART1_Read();
    }
    buff[ptr] = 0;
    if (sscanf((char*) buff, "%ld", &val) != EOF) {
        return val;
    } else {
        return 0;
    }
}

inline uint8_t highByte(uint16_t x) {
    return ( (x >> 8) & 0xff);
}

inline uint8_t lowByte(uint16_t x) {
    return ( x & 0xff);
}

/* update LCD. Called every second
 */
void lcdUpdate(void) {
    if (screen == home) {
        // time
        CLCD_SetPos(1, 0);
        CLCD_PutS((char*) gpsTime);

        // number of satellites
        CLCD_SetPos(0, 14);
        CLCD_PutS((char*) nsat);

        // grid
        if (!(gridCnt % gridFreq)) {
            CLCD_SetPos(1, 9);
            CLCD_PutS((char*) grid);
        }
    }
    gridCnt++;
    gridCnt = gridCnt % gridFreq;
}

// button functions

// TMR6 interrupt function
// check and debounce buttons. Buttons are active low

void buttonInterrupt(void) {
    if (!PORTBbits.RB4) button1State++;
    if (!PORTBbits.RB5) button2State++;
}

void checkButtons(void) {
    if (button1State > 0) {
        button1State = 0;
        setScreen((screen + 1) % nScreen);
    }
    if (button2State > 0) {
        button2State = 0;
        button2Func();
    }
}

// process second button for each screen

void button2Func(void) {
    char tmp[5];
    switch (screen) {
        case home:
            return;
            break;
        case setncnt:
            if (nCnt == 10000) nCnt = 10;
            else nCnt *= 10;
            DATAEE_WriteByte(1013, highByte((uint16_t) nCnt));
            DATAEE_WriteByte(1014, lowByte((uint16_t) nCnt));
            CLCD_SetPos(0, 5);
            switch (nCnt) {
                case 10:
                    CLCD_PutS("10   ");
                    break;
                case 100:
                    CLCD_PutS("100  ");
                    break;
                case 1000:
                    CLCD_PutS("1000 ");
                    break;
                case 10000:
                    CLCD_PutS("10000");
                    break;
            }
            cnt = 0;
            totalErr = 0;
            break;
        case sett:
            if (timeConst == 1024) timeConst = 4;
            else timeConst *= 2;
            DATAEE_WriteByte(1021, highByte((uint16_t) timeConst));
            DATAEE_WriteByte(1022, lowByte((uint16_t) timeConst));
            CLCD_SetPos(0, 7);
            switch (timeConst) {
                case 4:
                    CLCD_PutS("4   ");
                    break;
                case 8:
                    CLCD_PutS("8   ");
                    break;
                case 16:
                    CLCD_PutS("16  ");
                    break;
                case 32:
                    CLCD_PutS("32  ");
                    break;
                case 64:
                    CLCD_PutS("64  ");
                    break;
                case 128:
                    CLCD_PutS("128 ");
                    break;
                case 256:
                    CLCD_PutS("256 ");
                    break;
                case 512:
                    CLCD_PutS("512 ");
                    break;
                case 1024:
                    CLCD_PutS("1024");
                    break;
            }
            break;
        case setd:
            if (damping > 9.9) damping = 0.5;
            else damping += 0.5;
            DATAEE_WriteByte(1011, highByte((uint16_t) (damping * 100)));
            DATAEE_WriteByte(1012, lowByte((uint16_t) (damping * 100)));
            CLCD_SetPos(0, 8);
            CLCD_PutS("    ");
            sprintf(tmp, "%4.1f", damping);
            CLCD_SetPos(0, 8);
            CLCD_PutS(tmp);
            break;
        case setserial:
            serialMode = (serialMode + 1) % 3;
            CLCD_SetPos(0, 7);
            switch (serialMode) {
                case off:
                    CLCD_PutS("OFF");
                    break;
                case on:
                    CLCD_PutS("ON ");
#ifdef YIC51612
                    gpsExtraOff();
#endif
                    break;
                case gps:
                    CLCD_PutS("GPS");
#ifdef YIC51612
                    gpsExtraOn();
#endif
                    break;
            }
            break;
    }
}



// called when switching to new screen

void setScreen(screenType newScreen) {
    char tmp[5];
    if (newScreen == screen) return;
    CLCD_Clear();
    CLCD_SetPos(0, 0);
    switch (newScreen) {
        case home:
            // home screen
            CLCD_PutS(errStr);
            if (PPSlocked) {
                CLCD_SetPos(0, 12);
                CLCD_PutS("L");
            } else if (time < warmUpTime) {
                CLCD_SetPos(0, 12);
                CLCD_PutS("W");
            } else {
                CLCD_SetPos(0, 12);
                CLCD_PutS(" ");
            }

            CLCD_SetPos(0, 7);
            CLCD_PutS((char*) pwmStr);
            CLCD_SetPos(0, 14);
            CLCD_PutS((char*) nsat);
            CLCD_SetPos(1, 0);
            CLCD_PutS((char*) gpsTime);
            CLCD_SetPos(1, 9);
            CLCD_PutS((char*) grid);

            break;
        case setncnt:
            // set ncnt
            CLCD_PutS("NCNT:");
            CLCD_SetPos(0, 5);
            switch (nCnt) {
                case 10:
                    CLCD_PutS("10   ");
                    break;
                case 100:
                    CLCD_PutS("100  ");
                    break;
                case 1000:
                    CLCD_PutS("1000 ");
                    break;
                case 10000:
                    CLCD_PutS("10000");
                    break;
            }
            break;
        case sett:
            // set time const
            CLCD_PutS("TCONST:");
            CLCD_SetPos(0, 7);
            switch (timeConst) {
                case 4:
                    CLCD_PutS("4   ");
                    break;
                case 8:
                    CLCD_PutS("8   ");
                    break;
                case 16:
                    CLCD_PutS("16  ");
                    break;
                case 32:
                    CLCD_PutS("32  ");
                    break;
                case 64:
                    CLCD_PutS("64  ");
                    break;
                case 128:
                    CLCD_PutS("128 ");
                    break;
                case 256:
                    CLCD_PutS("256 ");
                    break;
                case 512:
                    CLCD_PutS("512 ");
                    break;
                case 1024:
                    CLCD_PutS("1024");
                    break;
                default:
                    CLCD_PutS("    ");
                    break;
            }
            break;
        case setd:
            // set damping
            CLCD_PutS("DAMPING:    ");
            CLCD_SetPos(0, 8);
            sprintf(tmp, "%4.1f", damping);
            CLCD_PutS(tmp);
            break;
        case setserial:
            // set serial mode
            CLCD_PutS("SERIAL:");
            CLCD_SetPos(0, 7);
            switch (serialMode) {
                case off:
                    CLCD_PutS("OFF");
                    break;
                case on:
                    CLCD_PutS("ON ");
                    break;
                case gps:
                    CLCD_PutS("GPS");
                    break;
            }
            break;
    }
    screen = newScreen;
}
