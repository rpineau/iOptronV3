#pragma once
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <memory.h>
#include <string.h>
#include <time.h>
#ifdef SB_MAC_BUILD
#include <unistd.h>
#endif

// C++ includes
#include <string>
#include <vector>
#include <sstream>
#include <iostream>

#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/theskyxfacadefordriversinterface.h"
#include "../../licensedinterfaces/sleeperinterface.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"
#include "../../licensedinterfaces/mountdriverinterface.h"

#include "StopWatch.h"


#define IOPTRON_DEBUG 2   // define this to have log files, 1 = bad stuff only, 2 and up.. full debug

enum iOptron {IOPTRON_OK=0, NOT_CONNECTED, IOPTRON_CANT_CONNECT, IOPTRON_BAD_CMD_RESPONSE, COMMAND_FAILED, IOPTRON_ERROR};
// Response: “0010”, “0011”, “0025”, “0026”, “0030”, “0045”, “0060”, “0061”, “0120” “0121”, “0122”, “5010”, “5035”, “5045”
// This command gets the mount model. “0010” means Cube II EQ mode, “0011” means SmartEQ Pro+, “0025” means CEM25(/P), “0026” means CEM25-EC,
// “0030” means iEQ30 Pro, “0045” means iEQ45 Pro EQ mode, “0060” means CEM60, “0061” means CEM60-EC, “0120” means CEM120, “0121” means CEM120-EC,
// “0122” means CEM120-EC2, “5010” means Cube II AA mode, “5035” means AZ Mount Pro, “5045” means iEQ45 Pro AA mode.

enum mountModels {CubeIIEQmode = 0010,
                SmartEQProPlus = 0011,
                CEM25 = 0025,
                CEM25_EC = 0026,
                iEQ30Pro = 0030,
                iEQ45ProEQmode = 0045,
                CEM60 = 0060,
                CEM60_EC = 0061,
                CEM120 = 0120,
                CEM120_EC = 0121,
                CEM120_EC2 = 0122,
                CubeIIAAmode = 5010,
                AZMountPro = 5035,
                iEQ45ProAAmode = 5045
};

enum iOptronStatus {STOPED = 0, TRACKING, SLEWING, GUIDING, FLIPPING, PEC_TRACKING, PARKED, HOMED};

enum iOptronGPSStatus {GPS_BROKE_OR_MISSING=0, GPS_WORKING_NOT_RECEIVED_DATA, GPS_RECEIVING_VALID_DATA};

enum iOptronTimeSource {RS232_or_ETHERNET=1, HAND_CONTROLLER, GPS_CONTROLLER};

#define SERIAL_BUFFER_SIZE 256
#define MAX_TIMEOUT 500         // 500 ms
#define IOPTRON_LOG_BUFFER_SIZE 1024
#define ERR_PARSE   1


#define IOPTRON_NB_SLEW_SPEEDS 7
#define IOPTRON_SLEW_NAME_LENGHT 5


// Define Class for Astrometric Instruments IOPTRON controller.
class CiOptron
{
public:
	CiOptron();
	~CiOptron();
	
	int Connect(char *pszPort);
	int Disconnect();
	bool isConnected() const { return m_bIsConnected; }

    void setSerxPointer(SerXInterface *p) { m_pSerx = p; }
    void setLogger(LoggerInterface *pLogger) { m_pLogger = pLogger; };
    void setTSX(TheSkyXFacadeForDriversInterface *pTSX) { m_pTsx = pTSX;};
    void setSleeper(SleeperInterface *pSleeper) { m_pSleeper = pSleeper;};

    int getNbSlewRates();
    int getRateName(int nZeroBasedIndex, char *pszOut, unsigned int nOutMaxSize);

    int getMountInfo(char *model, unsigned int strMaxLen);
    int getFirmwareVersion(char *version, unsigned int strMaxLen);

    int getRaAndDec(double &dRa, double &dDec);
    int syncTo(double dRa, double dDec);
    int isGPSGood(bool &bGPSGood);

    int setTrackingRates(bool bTrackingOn, bool bIgnoreRates, double dTrackRaArcSecPerHr, double dTrackDecArcSecPerHr);
    int getTrackRates(bool &bTrackingOn, double &dTrackRaArcSecPerHr, double &dTrackDecArcSecPerHr);

    int startSlewTo(double dRa, double dDec);
    int isSlewToComplete(bool &bComplete);

    int startOpenSlew(const MountDriverInterface::MoveDir Dir, unsigned int nRate);
    int stopOpenLoopMove();

    int parkMount();
    int setParkPosition(double dAz, double dAlt);
    int getAtPark(bool &bParked);
    int unPark();

    int getRefractionCorrEnabled(bool &bEnabled);
    int setRefractionCorrEnabled(bool bEnable);

    int getLimits(double &dHoursEast, double &dHoursWest);

    int Abort();

private:

    SerXInterface                       *m_pSerx;
    LoggerInterface                     *m_pLogger;
    TheSkyXFacadeForDriversInterface    *m_pTsx;
    SleeperInterface                    *m_pSleeper;

    float   m_fLat;
    float   m_fLong;
    int     m_nStatus;
    int		m_nGPSStatus;		// CEM120_EC and EC2 mounts are crap without GPS receiving signal
    int		m_nTimeSource;		// CEM120xxx mounts rely heavily on DST being set and time being accurate

    bool    m_bParked;

    bool    m_bDebugLog;
    char    m_szLogBuffer[IOPTRON_LOG_BUFFER_SIZE];

	bool    m_bIsConnected;                               // Connected to the mount?
    char    m_szFirmwareVersion[SERIAL_BUFFER_SIZE];

    char    m_szHardwareModel[SERIAL_BUFFER_SIZE];

	double  m_dGotoRATarget;						  // Current Target RA;
	double  m_dGotoDECTarget;                      // Current Goto Target Dec;
	
    MountDriverInterface::MoveDir      m_nOpenLoopDir;

    // limits don't change mid-course so we cache them
    bool    m_bLimitCached;
    
    int     sendCommand(const char *pszCmd, char *pszResult, int nResultMaxLen, int nExpectedResultLen = SERIAL_BUFFER_SIZE);
    int     readResponse( char *szRespBuffer, int nBufferLen, int nResultLen = SERIAL_BUFFER_SIZE);
    int     parseFields(const char *pszIn, std::vector<std::string> &svFields, char cSeparator);

    int     getInfoAndSettings();

    const char m_aszSlewRateNames[IOPTRON_NB_SLEW_SPEEDS][IOPTRON_SLEW_NAME_LENGHT] = { "1x", "2x", "8x", "16x",  "64x", "128x", "256x"};

    CStopWatch      timer;


#ifdef IOPTRON_DEBUG
    std::string m_sLogfilePath;
	// timestamp for logs
    char *timestamp;
	time_t ltime;
	FILE *Logfile;	  // LogFile
#endif
	
};


