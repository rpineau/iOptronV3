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


// #define IOPTRON_DEBUG 3   // define this to have log files, 1 = bad stuff only, 2 and up.. full debug

#define DRIVER_VERSION 1.21

enum iOptron {IOPTRON_OK=0, NOT_CONNECTED, IOPTRON_CANT_CONNECT, IOPTRON_BAD_CMD_RESPONSE, COMMAND_FAILED, IOPTRON_ERROR};
// Response: “0010”, “0011”, “0025”, “0026”, “0030”, “0045”, “0060”, “0061”, “0120” “0121”, “0122”, “5010”, “5035”, “5045”
// This command gets the mount model. “0010” means Cube II EQ mode, “0011” means SmartEQ Pro+, “0025” means CEM25(/P), “0026” means CEM25-EC,
// “0030” means iEQ30 Pro, “0045” means iEQ45 Pro EQ mode, “0060” means CEM60, “0061” means CEM60-EC, “0120” means CEM120, “0121” means CEM120-EC,
// “0122” means CEM120-EC2, “5010” means Cube II AA mode, “5035” means AZ Mount Pro, “5045” means iEQ45 Pro AA mode.

#define CEM26 "0026"
#define CEM26_EC "0027"
#define GEM28 "0028"
#define GEM28_EC "0029"
#define IEQ30PRO  "0030"
#define CEM60  "0060"
#define CEM60_EC  "0061"
#define CEM70  "0070"
#define CEM70_EC  "0071"
#define CEM120  "0120"
#define CEM120_EC  "0121"
#define CEM120_EC2  "0122"
#define UKNOWN_MOUNT "9999"

enum iOptronStatus {STOPPED = 0, TRACKING, SLEWING, GUIDING, FLIPPING, PEC_TRACKING, PARKED, HOMED};

enum iOptronGPSStatus {GPS_BROKE_OR_MISSING=0, GPS_WORKING_NOT_RECEIVED_DATA, GPS_RECEIVING_VALID_DATA};

enum iOptronTrackingRate {TRACKING_SIDEREAL=0, TRACKING_LUNAR, TRACKING_SOLAR, TRACKING_KING, TRACKING_CUSTOM};

enum iOptronTimeSource {TIME_SRC_UNKNOWN=0, RS232_or_ETHERNET=1, HAND_CONTROLLER, GPS_CONTROLLER};

enum iOptronPierStatus {PIER_EAST=0, PIER_WEST=1, PIER_INDETERMINATE};

enum iOptronCounterWeightStatus {COUNTER_WEIGHT_UP=0, COUNTER_WEIGHT_NORMAL};

enum iSlewTrackLimitsStatus {LIMITS_EXCEEDED_OR_BELOW_ALTITUDE=0, NO_ISSUE_SLEW_TRACK_ONE_OPTION, NO_ISSUE_SLEW_TRACK_TWO_OPTIONS, NO_STATUS};

enum iSlewResponse {SLEW_EXCEED_LIMIT_OR_BELOW_ALTITUDE=0, SLEW_ACCEPTED};

enum iMeridianBehavior {STOP_AT_POSITION_LIMIT=0, FLIP_AT_POSITION_LIMIT};

#define SERIAL_BUFFER_SIZE 256
#define MAX_TIMEOUT 1000         // was 500 ms
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
#ifdef IOPTRON_DEBUG
	void setLogFile(FILE *);
#endif
	
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

    int getRaAndDec(double &dRa, double &dDec, bool bForceMountCall);
    int setRaAndDec(char *pszLocationCalling, double dRaInDecimalHours, double dDecInDecimalDegrees);
    int syncTo(double dRa, double dDec);
    int isGPSGood(bool &bGPSGood);
    int isGPSOrLatLongGood(bool &bGPSOrLatLongGood);
    int getGPSStatusString(char *gpsStatus, unsigned int strMaxLen);
    int getTimeSource(char *timeSourceString, unsigned int strMaxLen);
    int getSystemStatusPassive(char *strSystemStatus, unsigned int strMaxLen);

    int setTrackingRates(bool bTrackingOn, bool bIgnoreRates, double dRaRateArcSecPerSec, double dDecRateArcSecPerSec);
    int getTrackRates(bool &bTrackingOn, double &dRaRateArcSecPerSec, double &dDecRateArcSecPerSec);
    int setSiderealTrackingOn();
    int setTrackingOff();
    int getTrackingStatusPassive(char *strTrackingStatus, unsigned int strMaxLen);

    int startSlewTo(double dRaInDecimalHours, double dDecInDecimalDegrees);
    int isSlewToComplete(bool &bComplete);
    int endSlewTo(void);

    int startOpenSlew(const MountDriverInterface::MoveDir Dir, unsigned int nRate);
    int stopOpenLoopMove();

    int parkMount();
    int setParkPosition(double dAz, double dAlt);
    int getParkPosition(double &dAz, double &dAlt);
    int getAtPark(bool &bParked);
    int unPark();

    int getRefractionCorrEnabled(bool &bEnabled);
    int beyondThePole(bool& bYes);
    int getLimits(double &dHoursEast, double &dHoursWest);
    double flipHourAngle();

    int Abort();

    int gotoZeroPosition();
    int getAtZeroPosition(bool &bAtZero);
    int getAtParkedPositionPassive(bool &bAtParked);
    int gotoFlatsPosition();
    int findZeroPosition();
    int getUtcOffset(char *pszUtcOffsetInMins);
    int setUtcOffset(char *pszUtcOffsetInMins);
    int getDST(bool &bDaylight);
    int setDST(bool bDaylight);
    int getLocation(float &fLat, float &fLong);
    int setLocation(float fLat, float fLong);
    int setTimeAndDate(double julianDateOfUTCTimeIncludingMillis);

    int getMeridianTreatment(int &iBehavior, int &iDegreesPastMeridian);
    int getAltitudeLimit(int &iDegreesAltLimit);
    int setMeridianTreatement(int iBehavior, int iDegreesPastMeridian);
    int setAltitudeLimit(int iDegreesAltLimit);
private:

    SerXInterface                       *m_pSerx;
    LoggerInterface                     *m_pLogger;
    TheSkyXFacadeForDriversInterface    *m_pTsx;
    SleeperInterface                    *m_pSleeper;

    float   m_fLat;
    float   m_fLong;
    int     m_nStatus;			// defined in iOptronStatus (stopped tracking slewing.. etc)
    int  	m_nTrackingRate;    // sidereal, lunar, solar, king, custom defined by iOptronTrackingRate
    float	m_fCustomRaMultiplier; // cached tracking rate multiplier received from :GTR# call in getTrackRates when tracking custom
    int     m_pierStatus;         // which side of the meridian is the OTA
    int     m_counterWeightStatus; // counterweight up (about to get ugly) or normal
    int     m_nDegreesPastMeridian;  // degrees past the meridian
    int     m_nAltitudeLimit;       // degrees from 0 could be negative or positive  [-89, +89] but when set through me [-10,55]
    int	 	m_nCacheLimitStatus; // cache if we had no, 1, or 2 slew options last time we slewed.  Filled when we startSlewTo and issue command :QAP#
    int		m_nGPSStatus;		// CEM120_EC and EC2 mounts are crap without GPS receiving signal
    int		m_nTimeSource;		// CEM120xxx mounts rely heavily on DST being set and time being accurate
    char    m_sModel[5];		// save a selectable/comparable version of the model of mount

    bool    m_bParked;

    bool    m_bDebugLog;
    char    m_szLogBuffer[IOPTRON_LOG_BUFFER_SIZE];

	bool    m_bIsConnected;                               // Connected to the mount?
    char    m_szFirmwareVersion[SERIAL_BUFFER_SIZE];

    char    m_szHardwareModel[SERIAL_BUFFER_SIZE];

	double  m_dGotoRATarget;						  // Current Target RA;
	double  m_dGotoDECTarget;                      // Current Goto Target Dec;
	
    MountDriverInterface::MoveDir      m_nOpenLoopDir;
    
    int     sendCommand(const char *pszCmd, char *pszResult, int nExpectedResultLen);
    int     readResponse(char *szRespBuffer, int nBytesToRead);

    int     getInfoAndSettings();

    const char m_aszSlewRateNames[IOPTRON_NB_SLEW_SPEEDS][IOPTRON_SLEW_NAME_LENGHT] = { "1x", "2x", "8x", "16x",  "64x", "128x", "256x"};

    CStopWatch      slewToTimer;
    CStopWatch      cmdTimer;
    CStopWatch      trackRatesTimer;
    CStopWatch      getAtParkTimer;

    float       m_dRa;
    float       m_dDec;

#ifdef IOPTRON_DEBUG
    std::string m_sLogfilePath;
	// timestamp for logs
    char *timestamp;
	time_t ltime;
	FILE *Logfile;	  // LogFile
	char* getTimestamp();
#endif
	
};


