#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "defines.h"
#include "mcc_generated_files/mcc.h"
#include "xlcd.h"
#include "mcc_generated_files/uart2.h"

void gpsParse(void);
void gpsRead(void);
void gpsStartup(void);
void gpsExtraOff(void);
void gpsExtraOn(void);
void sendGps(const uint8_t *data, uint8_t len);

extern uint8_t gpsTime[9];
extern uint8_t gpsSentence[256];
extern uint8_t grid[7];
extern uint8_t nsat[3];
extern uint8_t gpsCnt;
extern uint8_t gpsCheckCnt;
extern uint16_t gridCnt;
extern const uint16_t gridFreq;
extern screenType screen;
extern SerialModes serialMode;

#ifdef QUANTERR
extern float quantErr;
#endif

#ifdef VENUS838T
uint8_t venusMode;
uint8_t venusSurveyCnt[8];
uint8_t venusElevMask;
void sendVenus(const uint8_t *cmd, uint8_t len);
uint8_t readVenusAck(void);
void setVenusElevMask(uint8_t x);
#endif


// configure gps

void gpsConfig(void) {
    // delay to allow gps to start up
    __delay_ms(2000);
    gpsStartup();
    gpsExtraOff();
}

/*
 * send command to gps to turn off extra NMEA messages
 */
void gpsExtraOff(void) {

#ifdef YIC51612
    const uint8_t gpsOffStr[] = "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n";
    sendGps(gpsOffStr, 51);
#endif
    
}

/* 
 * commands to configure gps at startup
 */
void gpsStartup(void) {
    int8_t c;
    
#ifdef VENUS838T
    const uint8_t setTimingStr[31]
            = { 0x54, 0x01, 0x00, 0x00, 0x0f, 0xd0, // uint8 mode, uint32 survey len 4048
                0x00, 0x00, 0x00, 0x1e, // uint32 survey std dev
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00}; 
    const uint8_t resetStr[2] = {0x04, 0x00};
    
    if (serialMode == on) printf("\r\nConfigure GPS Venus 838LPx-T\r\n\r\n");

    // reset to factory command does not seem to generate ack/nack
    if (serialMode == on) printf("->Reset GPS\r\n");
    sendVenus(resetStr, 2);
    __delay_ms(1000);
        
    if (serialMode == on) printf("->Set timing mode and survey length\r\n");
    sendVenus(setTimingStr, 31);
    readVenusAck();
    __delay_ms(100);
    
    setVenusElevMask(venusElevMask);
    __delay_ms(100);
    
#ifdef VENUS_115200_BAUD
    // switch to 115200 baud temporarily
    
    const uint8_t setBaudStr[4]
            = { 0x05, 0x00, 0x05, 0x02};
    if (serialMode == on) printf("->Set 115200 baud\r\n");
    sendVenus(setBaudStr, 4);
    readVenusAck();
    
    // switch uart2 baud rate to 115200
    INTERRUPT_GlobalInterruptDisable();
    UART2_Initialize(0x8a, 0x00);
    INTERRUPT_GlobalInterruptEnable();
    __delay_ms(10);
#endif

#endif
}

#ifdef VENUS838T
/*
 * set elevation mask; x is mask value in degrees
 */
void setVenusElevMask(uint8_t x) {
    uint8_t maskStr[5];
   
    if ((x<5) || (x>85)) return;
    
    maskStr[0] = 0x2b;
    maskStr[1] = 0x02; // set elev mask only
    maskStr[2] = x;
    maskStr[3] = 0x0a; // min SNR=10; not used
    maskStr[4] = 0x00;
        
    if (serialMode == on) printf("->Set %d deg elevation mask\r\n", x);
    sendVenus(maskStr, 5);
    readVenusAck();
}

/* 
 * send binary command to Venus GPS 
 */
void sendVenus(const uint8_t *cmd, uint8_t len) {
    uint8_t data[42];
    uint8_t i,cnt,n;
    uint8_t checksum;
    
    n = len+7;
    memset(data, 0, n);
    data[0] = 0xa0;
    data[1] = 0xa1;
    data[2] = 0x00;
    data[3] = (uint8_t)len;
    cnt = 4;
    checksum = 0;
    for (i=0; i<len; i++) {
        data[cnt++] = cmd[i];
        checksum = checksum ^ cmd[i];
    }
    data[cnt++] = checksum;
    data[cnt++] = 0x0d;
    data[cnt] = 0x0a;
    sendGps(data, n);
}

/* 
 * Listen for Venus GPS Ack/Nack message. Returns
 0 Nack
 1 Ack
 2 error 
 
 ack format:
  
 0x24 0x47 0x00 0x02 0x83 0x54 0xd7 0x0d 0x0a
 
 0x83 = ack, 0x84 = nack
 
 */
uint8_t readVenusAck(void) {
    uint8_t i;
    uint8_t cs;
    uint8_t dat[5];
    
    // wait for an ack/nack
    do {
        while (!UART2_is_rx_ready()) {
        }
        dat[0] = UART2_Read();
    } while (dat[0]!=0x83 && dat[0]!=0x84);
    
    // read rest of response
    for (i = 1; i < 5; i++) {
        while (!UART2_is_rx_ready()) {
        }
        dat[i] = UART2_Read();
    }
    cs = 0;
    cs = cs ^ dat[0];
    cs = cs ^ dat[1];
    if (cs != dat[2]) {
        if (serialMode == on) printf("  uart error\r\n");
        return 2;
    }
    if (dat[0] == 0x83) {
        if (serialMode == on) printf("  OK\r\n");
        return 1;
    } else {
        if (serialMode == on) printf("  Not OK\r\n");
        return 0;
    }
}
#endif

/*
 * send cmd to gps to turn on extra NMEA messages
 */
void gpsExtraOn(void) {

#ifdef YIC51612  
    const uint8_t gpsOnStr[] = "$PMTK314,-1*04\r\n";
    sendGps(gpsOnStr, 16);
#endif

}

/* 
* send a serial command to gps
*/
void sendGps(const uint8_t *data, uint8_t len) {
    uint8_t i;
    for (i = 0; i < len; i++) {
        while (!UART2_is_tx_ready()) {
        }
        UART2_Write(data[i]);
        __delay_ms(1);
    }
    __delay_ms(5);
}

/*
 reads from serial port 2 (GPS)
 */
void gpsRead(void) {
    uint8_t c;
    static uint8_t checksum = 0;
    static uint8_t last = 0;
    uint8_t x,i;
    
    if (UART2_is_rx_ready()) {
        c = UART2_Read();
        // reject if not letter or number
        if ((c < 32 ) || (c > 122)) {
	  gpsCnt = 0;
	  gpsCheckCnt = 0;
	  last = 0;
	  return;
	}
        	
        // echo to computer serial port in gps mode
        if (serialMode == gps) {
            UART1_Write(c);
        }
        if (c == 0x24) { // '$' : start of GPS line
            gpsCnt = 0;
            gpsCheckCnt = 0;
            last = 0;
            return;
        } else if (c == 0x2a) { // '*'
            // get ready to read checksum
            gpsCheckCnt = 1;
            gpsCnt++;
            last = gpsCnt;
        } else { // any other character, put in buffer
            gpsCnt++;
            gpsSentence[gpsCnt] = c;
            
            // read checksum bytes and verify checksum
	    if (gpsCheckCnt == 1) {
	      if (c < 58) c -= 48; // digit 0-9
	      else if (c < 71) c-= 55; // hex digit A-F
	      else c -= 87; // hex digit a-f
	      checksum = c*16;
	      gpsCheckCnt ++;
	    } else if (gpsCheckCnt == 2) {
	      if (c < 58) c -= 48; // digit 0-9
	      else if (c < 71) c-= 55; // hex digit A-F
	      else c -= 87; // hex digit a-f
	      checksum += c;
	      gpsCheckCnt = 0;
              
	      // calculate checksum
	      x = 0;
	      for (i = 1;i < last; i++) {
		x ^= gpsSentence[i];
	      }
	      if (x == checksum) gpsParse();
            }
        }
    }
}

/* 
 * parse NMEA line for time, grid square, and time quantization error 
 */
void gpsParse(void) {

#ifdef VENUS838T
    // NMEA line for Venus838LPx-T timing mode
    uint8_t ii, jj;
    uint8_t error[6];

    if (gpsSentence[1] == 0x50 && // 'P'
            gpsSentence[2] == 0x53 && // 'S'
            gpsSentence[3] == 0x54) { // 'T'

        // mode: 0 = PVT  1 = Survey  2 = Static
        venusMode = gpsSentence[9] - 0x30; // 0x30 = '0'

        // get survey count
        ii = 11;
        jj = 0;
        while ((gpsSentence[ii] != 0x2c) && (jj < 7)) { // 0x2c = ','
            venusSurveyCnt[jj] = gpsSentence[ii];
            ii++;
            jj++;
        }
        // fill rest of string with spaces
	while (jj < 7) {
            venusSurveyCnt[jj] = 0x20;
	    jj++;
        }
        // make zero-terminated string
        venusSurveyCnt[7] = 0;
        
#ifdef QUANTERR
        // get quantization error
        ii++;
        jj = 0;
        while ((gpsSentence[ii] != 0x2c) && (jj < 5)) { //','
            error[jj] = gpsSentence[ii];
            ii++;
            jj++;
        }
        // make zero-terminated string
        jj++;
        error[jj] = 0;
        if (sscanf((char*) error, "%f", &quantErr) != 1) {
            // in case sscanf failed
            quantErr = 0.0;
        }
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

