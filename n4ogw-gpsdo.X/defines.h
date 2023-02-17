#ifndef DEFINES_H
#define	DEFINES_H

#include <xc.h> 

// ocxo frequency
#define FREQ 10000000

// size of tic history saved
#define NSTORE 144

// type of GPS module
//#define YIC51612
#define VENUS838T
#define QUANTERR

typedef enum 
{
    hold,
    run
} Modes;

typedef enum
{
    off,
    on,
    gps
            
} SerialModes;


// lcd screen index
// 0 = main screen
// 1 = set nCnt
// 2 = set timeConst
// 3 = set damping
// 4 = set serial mode
typedef enum {
    home,
    setncnt,
    sett,
    setd,
    setserial
} screenType;



#endif	
