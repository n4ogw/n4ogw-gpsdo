#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include "defines.h"
#include "mcc_generated_files/mcc.h"
#include "xlcd.h"
#include "mcc_generated_files/uart2.h"

void gpsParse(void);
void gpsRead(void);
void gpsExtraOff(void);
void gpsExtraOn(void);
void sendGps(const uint8_t *data, int len);

extern uint8_t gpsTime[9];
extern uint8_t gpsSentence[256];
extern uint8_t grid[7];
extern uint8_t nsat[3];
extern uint8_t gpsCnt;
extern uint16_t gridCnt;
extern const int gridFreq;
extern screenType screen;
extern SerialModes serialMode;

#ifdef QUANTERR
extern float quantErr;
#endif

#ifdef VENUS838T
extern uint8_t venusMode;
extern uint8_t venusSurveyCnt[8];
#endif


// configure gps

void gpsConfig(void) {
    gpsExtraOff();
}


// send cmd to gps to turn off extra NMEA messages

void gpsExtraOff(void) {

#ifdef YIC51612
    const uint8_t gpsOffStr[] = "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n";
    sendGps(gpsOffStr, 51);
#endif

#ifdef VENUS838T
    const uint8_t gpsOffStr[16]
            = {0xa0, 0xa1, 0x00, // header
        0x09, // length   
        0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // data
        0x09, // checksum
        0x0d, 0x0a}; // end

    const uint8_t setBaudStr[11]
            = {0xa0, 0xa1, 0x00, // header
        0x04, // length   
        0x05, 0x00, 0x05, 0x02, // data
        0x02, // checksum
        0x0d, 0x0a}; // end

    const uint8_t setTimingStr[42]
            = {0xa0, 0xa1, 0x00, // header
        0x1f, // length   
        0x54, 0x01, 0x00, 0x00, 0x0f, 0xd0, // uint8 mode, uint32 survey len 4048
        0x00, 0x00, 0x00, 0x1e, // uint32 survey std dev
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00,
        0x94, // checksum
        0x0d, 0x0a}; // end

    // set timing mode and survey length
    sendGps(setTimingStr, 42);
    __delay_ms(50);

    // turn off extra nmea (might not be needed)
    sendGps(gpsOffStr, 16);
    __delay_ms(50);

    // switch to 115200 baud
    sendGps(setBaudStr, 11);
    __delay_ms(50);

    // switch uart2 baud rate to 115200
    INTERRUPT_GlobalInterruptDisable();
    UART2_Initialize(0x8c, 0x00);
    __delay_ms(10);
    INTERRUPT_GlobalInterruptEnable();


#endif

}

// send cmd to gps to turn on extra NMEA messages

void gpsExtraOn(void) {

#ifdef YIC51612  
    const uint8_t gpsOnStr[] = "$PMTK314,-1*04\r\n";
    sendGps(gpsOnStr, 16);
#endif

}

/* 
send a serial command to gps
*/
void sendGps(const uint8_t *data, int len) {
    int i;
    for (i = 0; i < len; i++) {
        while (!UART2_is_tx_ready()) {
        }
        UART2_Write(data[i]);
        __delay_ms(1);
    }
    __delay_ms(10);
}

/*
 reads from serial port 2 (GPS)
 */
void gpsRead(void) {
    uint8_t c;
    if (UART2_is_rx_ready()) {
        c = UART2_Read();
        // echo to computer serial port in gps mode
        if (serialMode == gps) {
            UART1_Write(c);
        }
        if (c == 0x24) { // '$'
            gpsCnt = 0;
            return;
        } else if (c == 0x2a) { // '*'
            // ignore checksum, parse sentence
            gpsParse();
        } else
#ifdef VENUS838T
            /*
            if (c == 0x02) { // ack
                printf("ACK\r\n");
            } else if (c = 0x03) { // nack
                printf("NACK\r\n");
            } else
             */
#endif  
        {
            gpsCnt++;
            gpsSentence[gpsCnt] = c;
        }
    }
}

/* 
 * parse NMEA line for time and grid square 
 */
void gpsParse(void) {

#ifdef VENUS838T
    int ii, jj;
    uint8_t error[6];

    if (gpsSentence[1] == 0x50 && // 'P'
            gpsSentence[2] == 0x53 && // 'S'
            gpsSentence[3] == 0x54) { // 'T'

        // mode: 0 = PVT  1 = Survey  2 = Static
        venusMode = gpsSentence[9] - 0x30; // 0x30 = '0'

        // get survey count
        ii = 11;
        jj = 0;
        while ((gpsSentence[ii] != 0x2c) && (jj < 7) && (ii < 256)) { // 0x2c = ','
            venusSurveyCnt[jj] = gpsSentence[ii];
            ii++;
            jj++;
        }
        // fill rest of string with spaces
        for (jj = ii; jj < 7; jj++) {
            venusSurveyCnt[jj] = 0x20;
        }
        venusSurveyCnt[7] = 0;
#ifdef QUANTERR
        // get quantization error
        ii++;
        jj = 0;
        while ((ii < 256) && (gpsSentence[ii] != 0x2c)) { //','
            error[jj] = gpsSentence[ii];
            ii++;
            jj++;
        }
        jj++;
        error[jj] = 0;
        sscanf((char*) error, "%f", &quantErr);
#endif
    }
#endif  
    if (gpsSentence[3] == 0x47 && // 'G'
            gpsSentence[4] == 0x47 && // 'G'
            gpsSentence[5] == 0x41) { // 'A'

        if (screen == 0) {
            // time
            gpsTime[0] = gpsSentence[7];
            gpsTime[1] = gpsSentence[8];
            gpsTime[3] = gpsSentence[9];
            gpsTime[4] = gpsSentence[10];
            gpsTime[6] = gpsSentence[11];
            gpsTime[7] = gpsSentence[12];

            // number of satellites; at positions 45 and 46
            nsat[0] = gpsSentence[45];
            nsat[1] = gpsSentence[46];
            if (nsat[1] == 0x2c) { //','
                nsat[1] = 0x20; //' '
            }
            nsat[2] = 0;
        }

        if (!(gridCnt % gridFreq)) {
            // grid square
            // longitude fields 30-39 DDDMM.MMMM
            // 41: E or W

            // degrees longitude
            int l = (gpsSentence[30] - 0x30)*100 + (gpsSentence[31] - 0x30)*10 + (gpsSentence[32] - 0x30);
            int lm = (gpsSentence[33] - 0x30)*10 + (gpsSentence[34] - 0x30);

            // E-W referenced from 180 degree from longitude 0
            if (gpsSentence[41] == 0x57) { //'W'
                l = 180 - l - 1;
                lm = 60 - lm;
            } else {
                l += 180;
            }
            int ll = l / 20;
            grid[0] = (ll + 0x41);
            ll = l - ll * 20;
            grid[2] = (ll / 2 + 0x30);
            ll = ll - (ll / 2)*2;
            lm += ll * 60;
            grid[4] = (lm / 5 + 0x61);

            // latitude fields 18-26 DDMM.MMMM
            // 28: N or S
            // N-S referenced from south pole
            l = (gpsSentence[18] - 0x30)*10 + (gpsSentence[19] - 0x30);
            lm = (gpsSentence[20] - 0x30)*10 + (gpsSentence[21] - 0x30);
            if (gpsSentence[28] == 0x4e) { // 'N'
                l += 90;
            } else {
                l = 90 - l - 1;
                lm = 60 - lm;
            }
            grid[1] = ((l / 10) + 0x41);
            grid[3] = ((l - (l / 10)*10) + 0x30);
            grid[5] = (2 * lm / 5 + 0x61);
        }
    }
}

