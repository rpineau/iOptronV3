#include "iOptronV3.h"

// Constructor for IOPTRON
CiOptron::CiOptron() {

    m_bIsConnected = false;

    m_bParked = false;  // probably not good to assume we're parked.  Power could have shut down or we're at zero position or we're parked
    m_nGPSStatus = GPS_BROKE_OR_MISSING;  // unread to start (stating broke or missing)
    m_nTimeSource = TIME_SRC_UNKNOWN;  // unread to start

#if defined IOPTRON_DEBUG
    Logfile = NULL;
#endif
    m_dRa = 0.0;
    m_dDec = 0.0;
    m_nDegreesPastMeridian = 0;
    m_nCacheLimitStatus = NO_STATUS;   // initialize to no status
    m_fCustomRaMultiplier = 1.0;   // sidereal to start
    slewToTimer.Reset();
    cmdTimer.Reset();
    trackRatesTimer.Reset();
    getAtParkTimer.Reset();
}

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 1
void CiOptron::setLogFile(FILE *daFile) {
    Logfile = daFile;
    if (Logfile) {
        fprintf(Logfile, "[%s] IOPTRON setLogFile Called\n", getTimestamp());
        fflush(Logfile);
    }
}
#endif

CiOptron::~CiOptron(void)
{
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] IOPTRON Destructor Called\n", getTimestamp());
        fflush(Logfile);
    }
#endif
}

int CiOptron::Connect(char *pszPort)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    int iBehavior, iDegreesPastMeridian, iDegreesAltLimit;
    int connectSpeed = 115200;  // default for CEM120xxx

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 1
    if (Logfile) {
        fprintf(Logfile, "[%s] CiOptron::Connect Called %s\n", getTimestamp(), pszPort);
        fflush(Logfile);
    }
#endif

    // 9600 8N1 (non CEM120xxx mounts) or 115200 (CEM120xx mounts)
    while(true) {
        nErr = m_pSerx->open(pszPort, connectSpeed, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1") ;
        if(nErr == 0)
            m_bIsConnected = true;
        else {
            m_pSerx->flushTx();
            m_pSerx->purgeTxRx();
            m_pSerx->close();
            m_bIsConnected = false;
            return nErr;
        }
        // get mount model to see if we're properly connected
        nErr = getMountInfo(m_szHardwareModel, SERIAL_BUFFER_SIZE);
        if(nErr)
            m_bIsConnected = false;

        if(!m_bIsConnected && connectSpeed == 115200) {
            m_pSerx->flushTx();
            m_pSerx->purgeTxRx();
            m_pSerx->close();
            connectSpeed = 9600;
            continue;
        }
        else if(!m_bIsConnected && connectSpeed == 9600) {
            // connection failed at both speed.
            m_pSerx->flushTx();
            m_pSerx->purgeTxRx();
            m_pSerx->close();
            return ERR_NORESPONSE;
        }
        break;
    }

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 1
    if (Logfile) {
        fprintf(Logfile, "[%s] CiOptron::Connect connected at %d on %s\n", getTimestamp(), connectSpeed, pszPort);
        fflush(Logfile);
    }
#endif

    nErr = sendCommand(":RT3#", szResp, 1);  // sets tracking rate to King by default .. effectively clears any custom rate that existed before
    if(nErr) {
        m_bIsConnected = false;
        return nErr;
    }

    nErr = getMeridianTreatment(iBehavior, iDegreesPastMeridian);
    if(nErr) {
        m_bIsConnected = false;
        return nErr;
    }
    m_nDegreesPastMeridian = iDegreesPastMeridian;  // save degrees past meridian value

    nErr = getAltitudeLimit(iDegreesAltLimit);
    if(nErr) {
        m_bIsConnected = false;
        return nErr;
    }
    m_nAltitudeLimit = iDegreesAltLimit;  // save altitude limit

    getInfoAndSettings();
    return nErr;
}


int CiOptron::Disconnect(void)
{
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 1
    if (Logfile) {
        fprintf(Logfile, "[%s] CiOptron::Disconnect Called\n", getTimestamp());
        fflush(Logfile);
    }
#endif
	if (m_bIsConnected) {
        if(m_pSerx){
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 1
            if (Logfile) {
                fprintf(Logfile, "[%s] CiOptron::Disconnect closing serial port\n", getTimestamp());
                fflush(Logfile);
            }
#endif
            m_pSerx->flushTx();
            m_pSerx->purgeTxRx();
            m_pSerx->close();
        }
    }
	m_bIsConnected = false;

	return SB_OK;
}

#pragma mark - Used by OpenLoopMoveInterface
int CiOptron::getNbSlewRates()
{
    return IOPTRON_NB_SLEW_SPEEDS;
}

int CiOptron::getRateName(int nZeroBasedIndex, char *pszOut, unsigned int nOutMaxSize)
{
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getRateName] index was %i\n", getTimestamp(), nZeroBasedIndex);
        fflush(Logfile);
    }
#endif

    if (nZeroBasedIndex > IOPTRON_NB_SLEW_SPEEDS)
        return IOPTRON_ERROR;

    strncpy(pszOut, m_aszSlewRateNames[nZeroBasedIndex], nOutMaxSize);

    return IOPTRON_OK;
}

int CiOptron::startOpenSlew(const MountDriverInterface::MoveDir Dir, unsigned int nRate) // todo: not trivial how to slew in V3
{
    int nErr = IOPTRON_OK;
    char szCmd[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];

    m_nOpenLoopDir = Dir;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::startOpenSlew] setting to Dir %d\n", getTimestamp(), Dir);
        fprintf(Logfile, "[%s] [CiOptron::startOpenSlew] Setting rate to %d\n", getTimestamp(), nRate);
    fflush(Logfile);
    }
#endif

    nErr = getInfoAndSettings();
    if(nErr)
        return nErr;
    if (m_nStatus == SLEWING) {
        // interrupt slewing since user pressed button
        nErr = sendCommand(":Q#", szResp, 1);
    }

    // select rate.  :SRn# n=1..7  1=1x, 2=2x, 3=8x, 4=16x, 5=64x, 6=128x, 7=256x
    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":SR%1d#", nRate+1);
    nErr = sendCommand(szCmd, szResp, 1);

    // figure out direction
    switch(Dir){
        case MountDriverInterface::MD_NORTH:
            nErr = sendCommand(":mn#", szResp, 0);
            break;
        case MountDriverInterface::MD_SOUTH:
            nErr = sendCommand(":ms#", szResp, 0);
            break;
        case MountDriverInterface::MD_EAST:
            nErr = sendCommand(":me#", szResp, 0);
            break;
        case MountDriverInterface::MD_WEST:
            nErr = sendCommand(":mw#", szResp, 0);
            break;
    }

    return nErr;
}

int CiOptron::stopOpenLoopMove()
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::stopOpenLoopMove] Dir was %d\n", getTimestamp(), m_nOpenLoopDir);
        fflush(Logfile);
    }
#endif

    switch(m_nOpenLoopDir){
        case MountDriverInterface::MD_NORTH:
        case MountDriverInterface::MD_SOUTH:
            nErr = sendCommand(":qD#", szResp, 1);
            break;
        case MountDriverInterface::MD_EAST:
        case MountDriverInterface::MD_WEST:
            nErr = sendCommand(":qR#", szResp, 1);
            break;
    }

    return nErr;
}


#pragma mark - IOPTRON communication
int CiOptron::sendCommand(const char *pszCmd, char *pszResult, int nExpectedResultLen)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    unsigned long  ulBytesWrite;

    m_pSerx->purgeTxRx();

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] *** CiOptron::sendCommand sending : '%s'\n", getTimestamp(), pszCmd);
        fflush(Logfile);
    }
#endif

    nErr = m_pSerx->writeFile((void *)pszCmd, strlen((char*)pszCmd), ulBytesWrite);
    m_pSerx->flushTx();
    if(nErr) {
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        if (Logfile) {
            fprintf(Logfile, "[%s] *** CiOptron::sendCommand ***** ERROR SENDING COMMAND **** error = %d , pszCmd : '%s'\n", getTimestamp(), nErr, pszCmd);
            fflush(Logfile);
        }
#endif
        return nErr;
    }
    // read response
    nErr = readResponse(szResp, nExpectedResultLen);
    if(nErr) {
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        if (Logfile) {
            fprintf(Logfile, "[%s] *** CiOptron::sendCommand ***** ERROR READING RESPONSE **** error = %d , response : '%s'\n", getTimestamp(), nErr, szResp);
            fflush(Logfile);
        }
#endif
        return nErr;
    }
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] *** CiOptron::sendCommand response : '%s'\n", getTimestamp(), szResp);
        fflush(Logfile);
    }
#endif

    if(pszResult)
        strncpy(pszResult, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CiOptron::readResponse(char *szRespBuffer, int nBytesToRead)
{
    int nErr = IOPTRON_OK;
    unsigned long ulBytesActuallyRead = 0;
    char *pszBufPtr;

    if (nBytesToRead == 0)
        return nErr;

    memset(szRespBuffer, 0, (size_t) SERIAL_BUFFER_SIZE);
    pszBufPtr = szRespBuffer;

    nErr = m_pSerx->readFile(pszBufPtr, nBytesToRead, ulBytesActuallyRead, MAX_TIMEOUT);
    if(nErr) {
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 3
        if (Logfile) {
            fprintf(Logfile, "[%s] [CiOptron::readResponse] szRespBuffer = '%s'\n", getTimestamp(), szRespBuffer);
            fflush(Logfile);
        }
#endif
        return nErr;
    }

    if (ulBytesActuallyRead !=nBytesToRead) { // timeout or something screwed up with command passed in
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        if (Logfile) {
            fprintf(Logfile, "[%s] CiOptron::readResponse number of bytes read not what expected.  Number of bytes read: %lu.  Number expected: %i\n", getTimestamp(), ulBytesActuallyRead, nBytesToRead);
            fflush(Logfile);
        }
#endif

        nErr = IOPTRON_BAD_CMD_RESPONSE;
    }

    if(ulBytesActuallyRead && *(pszBufPtr-1) == '#')
        *(pszBufPtr-1) = 0; //remove the #

    return nErr;
}


#pragma mark - mount controller informations
int CiOptron::getMountInfo(char *model, unsigned int strMaxLen)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getMountInfo] called\n", getTimestamp());
        fflush(Logfile);
    }
#endif

    nErr = sendCommand(":MountInfo#", szResp, 4);
    if(nErr)
        return nErr;

    if (strcmp(szResp, IEQ30PRO) == 0) {
        strncpy(model, "iEQ30 Pro", strMaxLen);
        strncpy(m_sModel, IEQ30PRO, 5);
    } else if (strcmp(szResp, CEM26) == 0) {
        strncpy(model, "CEM26", strMaxLen);
        strncpy(m_sModel, CEM26, 5);
    } else if (strcmp(szResp, CEM26_EC) == 0) {
        strncpy(model, "CEM26-EC", strMaxLen);
        strncpy(m_sModel, CEM26_EC, 5);
    } else if (strcmp(szResp, GEM28) == 0) {
        strncpy(model, "GEM28", strMaxLen);
        strncpy(m_sModel, GEM28, 5);
    } else if (strcmp(szResp, GEM28_EC) == 0) {
        strncpy(model, "GEM28-EC", strMaxLen);
        strncpy(m_sModel, GEM28_EC, 5);
    } else if (strcmp(szResp, CEM70) == 0) {
        strncpy(model, "CEM70(G)", strMaxLen);
        strncpy(m_sModel, CEM70, 5);
    } else if (strcmp(szResp, CEM70_EC) == 0) {
        strncpy(model, "CEM70(G)-EC", strMaxLen);
        strncpy(m_sModel, CEM70_EC, 5);
    } else if (strcmp(szResp, CEM60) == 0) {
        strncpy(model, "CEM60", strMaxLen);
        strncpy(m_sModel, CEM60, 5);
    } else if (strcmp(szResp, CEM60_EC) == 0) {
        strncpy(model, "CEM60-EC", strMaxLen);
        strncpy(m_sModel, CEM60_EC, 5);
    } else if (strcmp(szResp, CEM120) == 0) {
        strncpy(model, "CEM120", strMaxLen);
        strncpy(m_sModel, CEM120, 5);
    } else if (strcmp(szResp, CEM120_EC) == 0) {
        strncpy(model, "CEM120-EC", strMaxLen);
        strncpy(m_sModel, CEM120_EC, 5);
    } else if (strcmp(szResp, CEM120_EC2) == 0) {
        strncpy(model, "CEM120-EC2", strMaxLen);
        strncpy(m_sModel, CEM120_EC2, 5);
    } else {
        strncpy(model, "Unsupported Mount", strMaxLen);
        strncpy(m_sModel, UKNOWN_MOUNT, 5);
    }
    return nErr;
}

int CiOptron::mountHasFunctioningGPSPassive(bool &bMountHasFunctioningGPS) {

    int nErr = IOPTRON_OK;

    bMountHasFunctioningGPS = (m_nGPSStatus != GPS_BROKE_OR_MISSING);
    return nErr;
}

int CiOptron::getFirmwareVersion(char *pszVersion, unsigned int nStrMaxLen)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    std::string sFirmwares;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getFirmwareVersion] called\n", getTimestamp());
        fflush(Logfile);
    }
#endif


    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = sendCommand(":FW1#", szResp, 13);
    if(nErr)
        return nErr;

    sFirmwares+= szResp;
    sFirmwares+= " ";

    nErr = sendCommand(":FW2#", szResp, 13);
    if(nErr)
        return nErr;
    sFirmwares+= szResp;

    strncpy(pszVersion, sFirmwares.c_str(), nStrMaxLen);
    return nErr;
}

#pragma mark - Mount Coordinates
int CiOptron::getRaAndDec(double &dRaInDecimalHours, double &dDecInDecimalDegrees, bool bForceMountCall)
{


#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getRaAndDec] called \n", getTimestamp());
        fflush(Logfile);
    }
#endif
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    char szRa[SERIAL_BUFFER_SIZE], szDec[SERIAL_BUFFER_SIZE], szPier[SERIAL_BUFFER_SIZE], szCounter[SERIAL_BUFFER_SIZE];
    int nRa, nDec;

    // don't ask the mount too often, returned cached value
    if(cmdTimer.GetElapsedSeconds()<0.1 && !bForceMountCall) {
        dRaInDecimalHours = m_dRa;
        dDecInDecimalDegrees = m_dDec;
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        if (Logfile) {
            fprintf(Logfile, "[%s] [CiOptron::getRaAndDec] SHORT circuiting TSX from going nuts on the mount. \n", getTimestamp());
            fflush(Logfile);
        }
#endif
        return nErr;
    }
    cmdTimer.Reset();
    nErr = sendCommand(":GEP#", szResp, 21);
    if(nErr)
        return nErr;
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getRaAndDec] response to :GEP# %s\n", getTimestamp(), szResp);
        fflush(Logfile);
    }
#endif

    memset(szRa, 0, SERIAL_BUFFER_SIZE);
    memset(szDec, 0, SERIAL_BUFFER_SIZE);
    memcpy(szDec, szResp, 9);
    memcpy(szRa, szResp+9, 9);
    nRa = atoi(szRa);
    nDec = atoi(szDec);
    dRaInDecimalHours = (nRa*0.01 * 24.0 / 360.0)/ 60.0 /60.0 ;
    dDecInDecimalDegrees = (nDec * 0.01)/ 60.0 /60.0 ;

    m_dRa = dRaInDecimalHours;
    m_dDec = dDecInDecimalDegrees;

    memcpy(szPier, szResp+18, 1);
    m_pierStatus = atoi(szPier);

    memcpy(szCounter, szResp+19, 1);
    m_counterWeightStatus = atoi(szCounter);

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getRaAndDec] nRa : %d\n", getTimestamp(), nRa);
        fprintf(Logfile, "[%s] [CiOptron::getRaAndDec] nDec : %d\n", getTimestamp(), nDec);
        fprintf(Logfile, "[%s] [CiOptron::getRaAndDec] Ra : %f\n", getTimestamp(), dRaInDecimalHours);
        fprintf(Logfile, "[%s] [CiOptron::getRaAndDec] Dec : %f\n", getTimestamp(), dDecInDecimalDegrees);
        fprintf(Logfile, "[%s] [CiOptron::getRaAndDec] pier side: : %s\n", getTimestamp(), (m_pierStatus==PIER_EAST)?"pier east" : (m_pierStatus==PIER_WEST)?"pier west":"pier indeterminate");
        fprintf(Logfile, "[%s] [CiOptron::getRaAndDec] counterweight status: : %s\n", getTimestamp(), (m_counterWeightStatus==COUNTER_WEIGHT_UP)?"counterweight up" : "counterweight normal");
        fflush(Logfile);
    }
#endif

    return nErr;
}

#pragma mark - Sync and Cal - used by SyncMountInterface
int CiOptron::syncTo(double dRaInDecimalHours, double dDecInDecimalDegrees)
{
    int nErr = IOPTRON_OK;
    int nRa, nDec;
    char szCmd[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::syncTo] called Ra : %f  Dec: %f\n", getTimestamp(), dRaInDecimalHours, dDecInDecimalDegrees);
        fflush(Logfile);
    }
#endif

    if (m_nGPSStatus != GPS_RECEIVING_VALID_DATA)
        return ERR_CMDFAILED;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = setRaAndDec("CiOptron::syncTo", dRaInDecimalHours, dDecInDecimalDegrees);
    if (nErr) {
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        fprintf(Logfile, "[%s] [CiOptron::syncTo] Error: error setting ra and Dec.  nErr: %i\n", getTimestamp(), nErr);
        fflush(Logfile);
#endif
        return nErr;
    }

    nErr = sendCommand(":CM#", szResp, 1);  // call Snc

     return nErr;
}

#pragma mark - tracking rates
int CiOptron::setSiderealTrackingOn() {
    // special implementation avoiding manually setting the tracking rate

    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::setSiderealTrackingOn] called \n", getTimestamp());
        fflush(Logfile);
    }
#endif

    // Set tracking to sidereal
    nErr = sendCommand(":RT0#", szResp, 1);  // use macro command to set this

    if (nErr)
        return nErr;

    // and turn on
    nErr = sendCommand(":ST1#", szResp, 1);  // and start tracking

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::setSiderealTrackingOn] finished.  Result: %s\n", getTimestamp(), szResp);
        fflush(Logfile);
    }
#endif

    return nErr;

}

int CiOptron::setTrackingOff() {
    // special implementation avoiding manually setting the tracking rate

    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::setTrackingOff] called \n", getTimestamp());
        fflush(Logfile);
    }
#endif

    nErr = sendCommand(":ST0#", szResp, 1);  // use macro command to set this

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::setTrackingOff] finished.  Result: %s\n", getTimestamp(), szResp);
        fflush(Logfile);
    }
#endif

    return nErr;

}

int CiOptron::setTrackingRates(bool bTrackingOn, bool bIgnoreRates, double dRaRateArcSecPerSec, double dDecRateArcSecPerSec)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmdTmp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];
    double dMountMultiplierRa = 1.0;
    bool bCustomRate = false;  // assume not a custom rate

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::setTrackingRates] called bTrackingOn: %s, bIgnoreRate: %s, dRaRateArcSecPerSec: %f, dDecRateArcSecPerSec %f\n", getTimestamp(), bTrackingOn?"true":"false", bIgnoreRates?"true":"false", dRaRateArcSecPerSec, dDecRateArcSecPerSec);
        fflush(Logfile);
    }
#endif

// :RRnnnnn# - set the tracking rate of the RA axis to n.nnnn *sidereal rate
//           - Valid data range is [0.1000, 1.9000] * sidereal rate.
//           - Data entered with this command will be remembered through a power cycle and automatically re- applied on the next power up.
// :RT4# - “Custom Tracking Rate”  must be selected before this command to take effect

// These commands select the tracking rate: select sidereal (“:RT0#”),
// lunar (“:RT1#”), solar (“:RT2#”), King (“:RT3#”), or custom (“:RT4#”).

    if (bTrackingOn) {
        if (bIgnoreRates) {
            strcpy(szCmd, ":RT3#");  // use 'macro' command to set sidereal/king (King is better)
        } else {
            // Sidereal rate
            if (-0.00001 < dRaRateArcSecPerSec && dRaRateArcSecPerSec < 0.00001 && -0.00001 < dDecRateArcSecPerSec && dDecRateArcSecPerSec < 0.00001) {
                strcpy(szCmd, ":RT3#");  // use 'macro' command to set sidereal/king (King is better)
                nErr = ERR_COMMANDNOTSUPPORTED;
                m_fCustomRaMultiplier = 1.0;  // set immediately b/c we dont overwhelm the mount and take current cached values
                m_nTrackingRate = TRACKING_KING; // set immediately b/c we dont overwhelm the mount and take current cached values
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
                if (Logfile) {
                    fprintf(Logfile, "[%s] [CiOptron::setTrackingRates] interpreted incoming rate as sidereal! \n", getTimestamp());
                    fflush(Logfile);
                }
#endif
            }
            // Lunar rate (tolerances increased based on JPL ephemeris generator)
            else if (0.30 < dRaRateArcSecPerSec && dRaRateArcSecPerSec < 0.83 && -0.25 < dDecRateArcSecPerSec && dDecRateArcSecPerSec < 0.25) {
                strcpy(szCmd, ":RT1#");  // use 'macro' command to set to lunar
                nErr = ERR_COMMANDNOTSUPPORTED;
                m_fCustomRaMultiplier = 1.0;  // set cache immediately
                m_nTrackingRate = TRACKING_LUNAR; // set immediately b/c we dont overwhelm the mount and take current cached values
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
                if (Logfile) {
                    fprintf(Logfile, "[%s] [CiOptron::setTrackingRates] interpreted incoming rate as lunar! \n", getTimestamp());
                    fflush(Logfile);
                }
#endif
            }
            // Solar rate (tolerances increased based on JPL ephemeris generator, since TSX demanded a rate outside previous tolerance)
            else if (0.037 < dRaRateArcSecPerSec && dRaRateArcSecPerSec < 0.043 && -0.017 < dDecRateArcSecPerSec && dDecRateArcSecPerSec < 0.017) {
                strcpy(szCmd, ":RT2#");  // use 'macro' command to set to solar
                nErr = ERR_COMMANDNOTSUPPORTED;
                m_fCustomRaMultiplier = 1.0; // set cache immediately
                m_nTrackingRate = TRACKING_SOLAR; // set immediately b/c we dont overwhelm the mount and take current cached values
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
                if (Logfile) {
                    fprintf(Logfile, "[%s] [CiOptron::setTrackingRates] interpreted incoming rate as solar! \n", getTimestamp());
                    fflush(Logfile);
                }
#endif
            } else {
                // full custom rate (tracking a satellite or comet or TSX asked (via user request) to add tracking)
                dMountMultiplierRa = (15.0410681 - dRaRateArcSecPerSec) / 15.0410681;
                if (dMountMultiplierRa < 0.0001) {
                    // trying to 'stop' tracking by sending us sidereal.  ok,.. lets not do custom tracking
                    strcpy(szCmd, ":ST0#");  // use command to stop tracking
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
                    if (Logfile) {
                        fprintf(Logfile, "[%s] [CiOptron::setTrackingRates] interpreted incoming rate as wanting to be stopped! \n", getTimestamp());
                        fflush(Logfile);
                    }
#endif
                } else {
                    bCustomRate = true;
                    m_fCustomRaMultiplier = dMountMultiplierRa;  // cache on instance since we dont ask mount over and over all the time
                    m_nTrackingRate = TRACKING_CUSTOM; // set immediately b/c we dont overwhelm the mount and take current cached values
                    memset(szCmdTmp, 0, SERIAL_BUFFER_SIZE);  // prep temp buffer to write to
                    snprintf(szCmdTmp, SERIAL_BUFFER_SIZE, ":RR%1.4f#", dMountMultiplierRa);  // write including decimal to make it easy to remove

                    memset(szCmd, 0, SERIAL_BUFFER_SIZE);  // prep actual buffer to send to mount
                    memcpy(szCmd, szCmdTmp, 4);  // copy before decimal
                    memcpy(szCmd+4, szCmdTmp+5, 5);  // copy after decimal
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
                    if (Logfile) {
                        fprintf(Logfile, "[%s] [CiOptron::setTrackingRates] interpreted incoming rate as custom! \n", getTimestamp());
                        fprintf(Logfile, "[%s] [CiOptron::setTrackingRates] we are at a custom rate!  Sending mount ra multiplier command: %s\n", getTimestamp(), szCmd);
                        fflush(Logfile);
                    }
#endif
                    nErr = sendCommand(szCmd, szResp, 1);  // sets tracking rate and returns a single byte
                    if (nErr)
                        return nErr;
                    strcpy(szCmd, ":RT4#");  // use 'macro' command to set to custom
                }

            }

        }

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        fprintf(Logfile, "[%s] [CiOptron::setTrackingRates] tracking on: %s, determined we are custom: %s.  Sending command: %s\n", getTimestamp(), bTrackingOn?"true":"false", bCustomRate?"true":"false", szCmd);
        fflush(Logfile);
#endif
        nErr = sendCommand(szCmd, szResp, 1);  // set tracking 'go'.  all commands return a single byte
        if (nErr)
            return nErr;
    }

    return nErr;
}

int CiOptron::getTrackRates(bool &bTrackingOn, double &dTrackRaArcSecPerSec, double &dTrackDecArcSecPerSec)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    double fRa = m_fCustomRaMultiplier;  // initialize with cached value

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getTrackRates] called.\n", getTimestamp());
        fflush(Logfile);
    }
#endif
    memset(szResp, 0, SERIAL_BUFFER_SIZE);

    // don't ask the mount its general status too often .. doesn't change much
    if(trackRatesTimer.GetElapsedSeconds()>1.0) {
        getInfoAndSettings();
        trackRatesTimer.Reset();

        // iOptron bug in firmware: this always returns 1.0000: :GTR#
        // Response: “nnnnn#”
        // This command gets the saved custom tracking rate, the tracking rate is n.nnnn * sidereal rate.
        // Valid data range is [0.1000, 1.9000] * sidereal rate.

//        nErr = sendCommand(":GTR#", szResp, 6);  // lets see what we're at anyway
//
//        memset(szRa, 0, SERIAL_BUFFER_SIZE);
//        szRa[0] = szResp[0];
//        szRa[1] = '.';
//        memcpy(szRa+2, szResp+1, 4);
//        fRa = atof(szRa);

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        if (Logfile) {
            fprintf(Logfile, "[%s] [CiOptron::getTrackRates] asked mount for actual rate multiplier.  response: %s.  And interpreted to be a double: %f.\n", getTimestamp(), szResp, fRa);
            fflush(Logfile);
        }
#endif

    }

    switch (m_nStatus) {
        case STOPPED:
            bTrackingOn = false;
            dTrackRaArcSecPerSec = 15.0410681;
            dTrackDecArcSecPerSec = 0.0;
            break;
        case PARKED:
            bTrackingOn = false;
            dTrackRaArcSecPerSec = 15.0410681;
            dTrackDecArcSecPerSec = 0.0;
            break;
        case HOMED:
            bTrackingOn = false;
            dTrackRaArcSecPerSec = 15.0410681;
            dTrackDecArcSecPerSec = 0.0;
            break;
        case TRACKING:
            if (m_nTrackingRate == TRACKING_SIDEREAL) {
                bTrackingOn = true;
                dTrackRaArcSecPerSec = 0.0;
                dTrackDecArcSecPerSec = 0.0;
            } else if (m_nTrackingRate == TRACKING_LUNAR) {
                bTrackingOn = true;
                dTrackRaArcSecPerSec = 0.5490149;
                dTrackDecArcSecPerSec = 0.0;
            } else if (m_nTrackingRate == TRACKING_SOLAR) {
                bTrackingOn = true;
                dTrackRaArcSecPerSec = 0.0410681;
                dTrackDecArcSecPerSec = 0.0;
            } else if (m_nTrackingRate == TRACKING_KING) {
                bTrackingOn = true;
                dTrackRaArcSecPerSec = 0.0;
                dTrackDecArcSecPerSec = 0.0;
            } else if (m_nTrackingRate == TRACKING_CUSTOM) {
                bTrackingOn = true;
                dTrackRaArcSecPerSec = 15.0410681 - (15.0410681 * fRa);
                dTrackDecArcSecPerSec = 0.0;

            }
            break;
        case PEC_TRACKING:
            if (m_nTrackingRate == TRACKING_SIDEREAL) {
                bTrackingOn = true;
                dTrackRaArcSecPerSec = 0.0;
                dTrackDecArcSecPerSec = 0.0;
            } else if (m_nTrackingRate == TRACKING_LUNAR) {
                bTrackingOn = true;
                dTrackRaArcSecPerSec = 0.5490149;
                dTrackDecArcSecPerSec = 0.0;
            } else if (m_nTrackingRate == TRACKING_SOLAR) {
                bTrackingOn = true;
                dTrackRaArcSecPerSec = 0.0410681;
                dTrackDecArcSecPerSec = 0.0;
            } else if (m_nTrackingRate == TRACKING_KING) {
                bTrackingOn = true;
                dTrackRaArcSecPerSec = 0.0;
                dTrackDecArcSecPerSec = 0.0;
            } else if (m_nTrackingRate == TRACKING_CUSTOM) {
                bTrackingOn = true;
                dTrackRaArcSecPerSec = 15.0410681 - (15.0410681 * fRa);
                dTrackDecArcSecPerSec = 0.0;
            }
            break;
        default:
            bTrackingOn = false;
            dTrackRaArcSecPerSec = 15.0410681;
            dTrackDecArcSecPerSec = 0.0;
            break;
    }
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getTrackRates] done. returning: bTrackingOn: %s, dTrackRaArcSecPerSec: %f, dTrackDecArcSecPerSec: %f\n", getTimestamp(), bTrackingOn?"true":"false", dTrackRaArcSecPerSec, dTrackDecArcSecPerSec);
        fflush(Logfile);
    }
#endif
    return nErr;
}

#pragma mark - UI controlls
int CiOptron::gotoZeroPosition() {
    // special implementation for CEM120xx and CEM60xx only

    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::gotoZeroPosition] called \n", getTimestamp());
        fflush(Logfile);
    }
#endif

    // Goto Zero position / home position
    nErr = sendCommand(":MH#", szResp, 1);

    if (nErr)
        return nErr;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::gotoZeroPosition] finished.  Result: %s\n", getTimestamp(), szResp);
        fflush(Logfile);
    }
#endif

    return nErr;

}

int CiOptron::getAtZeroPositionPassive(bool &bAtZero) {
    // special call which is used by UI.. and we've assumed getInfoAndSettings() already called for other UI elements
    bAtZero = m_nStatus == HOMED;
    return IOPTRON_OK;
}

int CiOptron::getAtParkedPositionPassive(bool &bAtParked) {
    // special call which is used by UI.. and we've assumed getInfoAndSettings() already called for other UI elements
    bAtParked = m_nStatus == PARKED;
    return IOPTRON_OK;
}

int CiOptron::gotoFlatsPosition() {
    // special convience function to take flats

    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::gotoFlatsPosition] called \n", getTimestamp());
        fflush(Logfile);
    }
#endif

    // set alt/az position
    // altitude: :SasTTTTTTTT# (Valid data range is [-32,400,000, 32,400,000])
    nErr = sendCommand(":Sa+32400000#", szResp, 1);  // point straight up
    if (nErr) {
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        if (Logfile) {
            fprintf(Logfile, "[%s] [CiOptron::gotoFlatsPosition] error %i sending :Sa+32400000# : %s\n", getTimestamp(), nErr, szResp);
            fflush(Logfile);
        }
#endif
        return nErr;
    }
    // azimuth: :SzTTTTTTTTT# (Valid data range is [0, 129,600,000])
    nErr = sendCommand(":Sz000000000#", szResp, 1);  // point north
    if (nErr) {
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        if (Logfile) {
            fprintf(Logfile, "[%s] [CiOptron::gotoFlatsPosition] error %i sending :Sz000000000# : %s\n", getTimestamp(), nErr, szResp);
            fflush(Logfile);
        }
#endif
        return nErr;
    }

    // Goto Zero alt/az position defined
    nErr = sendCommand(":MSS#", szResp, 1);
    if (nErr) {
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        if (Logfile) {
            fprintf(Logfile, "[%s] [CiOptron::gotoFlatsPosition] error %i sending :MSS# : %s\n", getTimestamp(), nErr, szResp);
            fflush(Logfile);
        }
#endif
        return nErr;
    }

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::gotoFlatsPosition] MSS slew command finished.  Result (want 1): %s\n", getTimestamp(), szResp);
        fflush(Logfile);
    }
#endif
    // dont track
    nErr = sendCommand(":ST0#", szResp, 1);

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::gotoFlatsPosition] finished.  Result of final stop-tracking command (want 0): %i\n", getTimestamp(), nErr);
        fflush(Logfile);
    }
#endif

    return nErr;

}

int CiOptron::findZeroPosition() {
    // special implementation for CEM120xx and CEM60xx only

    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::calibrateZeroPosition] called \n", getTimestamp());
        fflush(Logfile);
    }
#endif

    // Find Zero position / home position
    nErr = sendCommand(":MSH#", szResp, 1);

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::calibrateZeroPosition] finished.  Result: %s\n", getTimestamp(), szResp);
        fflush(Logfile);
    }
#endif

    return nErr;

}

int CiOptron::getUtcOffsetAndDST(char *pszUtcOffsetInMins, bool &bDaylight)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szTmp[SERIAL_BUFFER_SIZE];

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getUtcOffsetAndDST] called \n", getTimestamp());
        fflush(Logfile);
    }
#endif

    // Get time related info
    nErr = sendCommand(":GUT#", szResp, 19);

    memset(pszUtcOffsetInMins,0, SERIAL_BUFFER_SIZE);
    memcpy(pszUtcOffsetInMins, szResp, 4);

    memset(szTmp,0, SERIAL_BUFFER_SIZE);
    memcpy(szTmp, szResp+4, 1);
    bDaylight = (atoi(szTmp) == 1);

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getUtcOffsetAndDST] finished.  nErr = %i, Command Result: %s, utcOffsetInMins: %s, daylight: %s\n", getTimestamp(), nErr, szResp, pszUtcOffsetInMins, bDaylight?"true":"false");
        fflush(Logfile);
    }
#endif

    return nErr;
}

int CiOptron::setUtcOffset(char *pszUtcOffsetInMins)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::setUtcOffset] called \n", getTimestamp());
        fflush(Logfile);
    }
#endif

    // :SGsMMM#
    // This command sets the minute offset from UTC (The Daylight-Saving Time will
    // not be take account into this value). Valid data range is [-720, +780].
    // Note: the resolution is 1 minute.

    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":SG%s#", pszUtcOffsetInMins);

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::setUtcOffset] buffer to send to mount %s\n", getTimestamp(), szCmd);
        fflush(Logfile);
    }
#endif

    nErr = sendCommand(szCmd, szResp, 1);

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::setUtcOffset] done, nErr = %i\n", getTimestamp(), nErr);
        fflush(Logfile);
    }
#endif

    return nErr;
}

int CiOptron::setDST(bool bDaylight)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::setDST] called \n", getTimestamp());
        fflush(Logfile);
    }
#endif

    //  Command: “:SDS0#” or “:SDS1#”
    //  Response: “1”
    //  These commands set the status of Daylight Saving Time.
    //  “:SDS1#” means Daylight Saving Time has been observed,
    //  “:SDS0#” means Daylight Saving Time has not been observed.

    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":SDS%.1d#", bDaylight?1:0);

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::setDST] buffer to send to mount %s\n", getTimestamp(), szCmd);
        fflush(Logfile);
    }
#endif

    nErr = sendCommand(szCmd, szResp, 1);

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::setDST] done, nErr = %i\n", getTimestamp(), nErr);
        fflush(Logfile);
    }
#endif

    return nErr;
}

int CiOptron::getLocation(float &fLat, float &fLong) // make passive version
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szTmp[SERIAL_BUFFER_SIZE];

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getLocation] called \n", getTimestamp());
        fflush(Logfile);
    }
#endif

    getInfoAndSettings();  // this case we want to be accurate
    getLocationPassive(fLat, fLong); // no way error would have been returned

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getLocation] finished.  Result: Latitude: %g, Longitude %g, with error code: %i\n", getTimestamp(), fLat, fLong, nErr);
        fflush(Logfile);
    }
#endif

    return nErr;
}

int CiOptron::getLocationPassive(float &fLat, float &fLong)
{
    int nErr = IOPTRON_OK;

    fLat = m_fLat;
    fLong = m_fLong;

    return nErr;
}

int CiOptron::setLocation(float fLat, float fLong)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];
    unsigned long ulLatToSend;
    unsigned long ulLongToSend;

#if defined IOPTRON_DEBUG
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::setLocation] called \n", getTimestamp());
        fflush(Logfile);
    }
#endif

    //  extracting lat:   m_fLat = ((atof(szTmp)-32400000)*0.01)/ 60.0 /60.0;
    ulLatToSend = (fLat * 60.0 * 60.0 / 0.01);

#ifdef IOPTRON_DEBUG
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::setLocation] setting Latitude from %f to iOptron value %lu\n", getTimestamp(), m_fLat, ulLatToSend);
        fflush(Logfile);
    }
#endif

    //  Command: ":SLAsTTTTTTTT#"
    //  Response: "1"
    //  This command sets the current latitude. Valid data range is [-32,400,000, +32,400,000].
    //  Note: North is positive, and the resolution is 0.01 arc-second.

    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":SLA%+09d#", ulLatToSend);

#if defined IOPTRON_DEBUG
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::setLocation] buffer to send for Lat to mount %s\n", getTimestamp(), szCmd);
        fflush(Logfile);
    }
#endif

    nErr = sendCommand(szCmd, szResp, 1);

    if (atoi(szResp) != 1) {
        return 1; // meaning error
    } else {
        // extracing long:   m_fLong = (atof(szTmp)*0.01)/ 60.0 /60.0;
        ulLongToSend = (fLong * 60.0 * 60.0 / 0.01);

#if defined IOPTRON_DEBUG
        if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::setLocation] setting Longitude from %f to iOptron value %lu\n", getTimestamp(), m_fLong, ulLongToSend);
        fflush(Logfile);
    }
#endif
        //
        //  Command: ":SLOsTTTTTTTT#"
        //  Response: "1"
        //  This command sets the current longitude. Valid data range is [-64,800,000, +64,800,000].
        //  Note: East is positive, and the resolution is 0.01 arc-second.

        uint32_t ulLongToSend = fabs(fLong) * 60 * 60 * 100;

        snprintf(szCmd, SERIAL_BUFFER_SIZE, ":SLO%c%08u#", fLong >= 0 ? '+' : '-', ulLongToSend);

#if defined IOPTRON_DEBUG
        if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::setLocation] buffer to send for Long to mount %s\n", getTimestamp(), szCmd);
        fflush(Logfile);
    }
#endif
        nErr = sendCommand(szCmd, szResp, 1);

#if defined IOPTRON_DEBUG
        if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::setLocation] done, nErr = %i\n", getTimestamp(), nErr);
        fflush(Logfile);
    }
#endif

        return nErr;
    }

}

int CiOptron::setTimeAndDate(double dJulianDateRightNow)
{
    int nErr = IOPTRON_OK;
    double dMountsDesiredJulianDateOffset;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

//    Command: “:SUTXXXXXXXXXXXXX#”
//    Response: “1”
//    This command sets the current UTC Time. The number equals (JD(current UTC Time) – J2000) * 8.64e+7.
//    Note: JD(current UTC time) means Julian Date of current UTC time. The resolution is 1 millisecond.

    dMountsDesiredJulianDateOffset = (dJulianDateRightNow-2451545.0L)*86400000.0L;
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::setTimeAndDate] calculated float: %f as the JD time offset\n", getTimestamp(), dMountsDesiredJulianDateOffset);
        fflush(Logfile);
    }
#endif
    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":SUT%013.0f#", dMountsDesiredJulianDateOffset);

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::setTimeAndDate] will send command %s to mount\n", getTimestamp(), szCmd);
        fflush(Logfile);
    }
#endif

    nErr = sendCommand(szCmd, szResp, 1);

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::setTimeAndDate] done, response is %s nErr = %i\n", getTimestamp(), szResp, nErr);
        fflush(Logfile);
    }
#endif

    if (atoi(szResp) != 1) {
        return 1; // meaning error
    } else {
        return nErr;  // return any communication error or 0 if none
    }

}


int CiOptron::getGPSStatusString(char *gpsStatus, unsigned int strMaxLen) // make passive version
{
    int nErr = IOPTRON_OK;
    getInfoAndSettings();  // this case we want to be accurate

    getGPSStatusStringPassive(gpsStatus, strMaxLen);
    return nErr;
}

int CiOptron::getGPSStatusStringPassive(char *gpsStatus, unsigned int strMaxLen) {

    int nErr = IOPTRON_OK;
    switch(m_nGPSStatus){
        case GPS_BROKE_OR_MISSING:
            strncpy(gpsStatus, "Broke or Missing", strMaxLen);
            break;
        case GPS_WORKING_NOT_RECEIVED_DATA:
            strncpy(gpsStatus, "Working not Received Data", strMaxLen);
            break;
        case GPS_RECEIVING_VALID_DATA:
            strncpy(gpsStatus, "Working Data Valid", strMaxLen);
            break;
    }
    return nErr;
}

int CiOptron::getTimeSource(char *timeSourceString, unsigned int strMaxLen)
{
    int nErr = IOPTRON_OK;
    getInfoAndSettings();  // this case we want to be accurate
    getTimeSourcePassive(timeSourceString, strMaxLen);
    return nErr;
}

int CiOptron::getTimeSourcePassive(char *timeSourceString, unsigned int strMaxLen)
{
    int nErr = IOPTRON_OK;
    switch(m_nTimeSource){
        case TIME_SRC_UNKNOWN:
            strncpy(timeSourceString, "Uknown or Missing", strMaxLen);
            break;
        case RS232_or_ETHERNET:
            strncpy(timeSourceString, "RS232 or Ethernet", strMaxLen);
            break;
        case HAND_CONTROLLER:
            strncpy(timeSourceString, "Hand Controller", strMaxLen);
            break;
        case GPS_CONTROLLER:
            strncpy(timeSourceString, "GPS Controller", strMaxLen);
            break;
    }
    return nErr;
}

int CiOptron::getSystemStatusPassive(char *strSystemStatus, unsigned int strMaxLen) {
    int nErr = IOPTRON_OK;
    // getInfoAndSettings();  // passive means someone else called this
    switch(m_nStatus){
        case STOPPED:
            strncpy(strSystemStatus, "stopped at non-zero position", strMaxLen);
            break;
        case TRACKING:
            strncpy(strSystemStatus, "tracking with PEC disabled", strMaxLen);
            break;
        case SLEWING:
            strncpy(strSystemStatus, "mount slewing", strMaxLen);
            break;
        case GUIDING:
            strncpy(strSystemStatus, "mount auto-guiding", strMaxLen);
            break;
        case FLIPPING:
            strncpy(strSystemStatus, "mount meridian flipping", strMaxLen);
            break;
        case PEC_TRACKING:
            strncpy(strSystemStatus, "tracking with PEC enabled", strMaxLen);
            break;
        case PARKED:
            strncpy(strSystemStatus, "mount parked", strMaxLen);
            break;
        case HOMED:
            strncpy(strSystemStatus, "mount stopped at zero position", strMaxLen);
            break;
    }
    return nErr;

}

int CiOptron::getTrackingStatusPassive(char *strTrackingStatus, unsigned int strMaxLen)
{
    int nErr = IOPTRON_OK;
    // getInfoAndSettings();  // passive means someone else called this
    switch(m_nTrackingRate){
        case TRACKING_SIDEREAL:
            strncpy(strTrackingStatus, "sidereal rate", strMaxLen);
            break;
        case TRACKING_LUNAR:
            strncpy(strTrackingStatus, "lunar rate", strMaxLen);
            break;
        case TRACKING_SOLAR:
            strncpy(strTrackingStatus, "solar rate", strMaxLen);
            break;
        case TRACKING_KING:
            strncpy(strTrackingStatus, "King rate", strMaxLen);
            break;
        case TRACKING_CUSTOM:
            strncpy(strTrackingStatus, "custom rate", strMaxLen);
            break;
    }
    return nErr;

}

#pragma mark - AsymmetricalEquatorialInterface
int CiOptron::getLimits(double &dHoursEast, double &dHoursWest)
{
    int nErr = IOPTRON_OK;

    dHoursWest = m_nDegreesPastMeridian / 15.0;
    dHoursEast = m_nDegreesPastMeridian / 15.0;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getLimits] called. scope setup for %i degrees past meridian. returning hoursEast %f, hoursWest %f\n", getTimestamp(), m_nDegreesPastMeridian, dHoursEast, dHoursWest);
        fflush(Logfile);
    }
#endif
    return nErr;
}

int CiOptron::beyondThePole(bool& bYes)
{
    int nErr = IOPTRON_OK;
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::beyondThePole] called. \n", getTimestamp());
        fflush(Logfile);
    }
#endif

    //bYes = (m_pierStatus == PIER_WEST) && (m_counterWeightStatus == COUNTER_WEIGHT_UP); // this means beyond the meridian
    bYes = (m_pierStatus == PIER_WEST);  // this means OTA even hinting to be on that side of the pier.  Likely this is what TSX wants

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::beyondThePole] finished.  Returned: %s since piers is: %s \n", getTimestamp(), bYes?"true":"false", (m_pierStatus==PIER_EAST)?"pier east" : (m_pierStatus==PIER_WEST)?"pier west":"pier indeterminate");
        fflush(Logfile);
    }
#endif
    return nErr;
}

double CiOptron::flipHourAngle() {
    return m_nDegreesPastMeridian / 15.0;
}

#pragma mark - Slew
int CiOptron::startSlewTo(double dRaInDecimalHours, double dDecInDecimalDegrees)
{
    int nErr = IOPTRON_OK;
    bool bGPSOrLatLongGood;
    char szCmdRa[SERIAL_BUFFER_SIZE];
    char szCmdDec[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
    double dRaArcSec, dDecArcSec;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::startSlewTo] called Ra: %f and Dec: %f\n", getTimestamp(), dRaInDecimalHours, dDecInDecimalDegrees);
        fflush(Logfile);
    }
#endif

    nErr = isGPSOrLatLongGood(bGPSOrLatLongGood);
    if (nErr) {
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        fprintf(Logfile, "[%s] [CiOptron::startSlewTo] Error calling isGPSOrLatLongGood.  nErr: %i\n", getTimestamp(), nErr);
        fflush(Logfile);
#endif
        return nErr;
    }

    if (!bGPSOrLatLongGood) {
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        if (Logfile) {
            fprintf(Logfile, "[%s] [CiOptron::startSlewTo] called Ra: %f and Dec: %f .. ABORTING due to GPS signal not being good OR lat/long not being set properly\n", getTimestamp(), dRaInDecimalHours, dDecInDecimalDegrees);
            fflush(Logfile);
        }
#endif
        return ERR_ABORTEDPROCESS;
    }
    nErr = setRaAndDec("CiOptron::startSlewTo", dRaInDecimalHours, dDecInDecimalDegrees);
    if (nErr) {
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        fprintf(Logfile, "[%s] [CiOptron::startSlewTo] Error: error setting ra and Dec.  nErr: %i\n", getTimestamp(), nErr);
        fflush(Logfile);
#endif
        return nErr;
    }

    memset(szResp, 0, SERIAL_BUFFER_SIZE);  // start over with response buffer
    // :QAP#  Response: “0”, “1”, “2”
    //This command queries the number of available position for most recently defined right ascension and declination coordinates
    // which not exceed the mechanical limits, altitude limits and meridian flip limits (including normal position and counterweight up position).
    // Checking if we will exceed mount's limits.  The possible response is  0#, 1# and 2#.
//    nErr = sendCommand(":QAP#", szResp, 2);
//    if (nErr) {
//#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
//        fprintf(Logfile, "[%s] [CiOptron::startSlewTo] Error: sendCommand bombed sending :QAP#.  nErr: %i\n", getTimestamp(), nErr);
//        fflush(Logfile);
//#endif
//        return nErr;
//    }
//    szResp[1] = 0;
//    m_nCacheLimitStatus = atoi(szResp);   // again 0 = problem, 1=ok with 1 position to slew to, 2=ok with two positions to slew to
//
//    if (m_nCacheLimitStatus == LIMITS_EXCEEDED_OR_BELOW_ALTITUDE) {
//#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
//        if (Logfile) {
//            fprintf(Logfile, "[%s] [CiOptron::startSlewTo] Warning: Commands:  Ra [0, 129,600,000]: %s and Dec [-32,400,000, +32,400,000]: %s exceed mechanical limits, altitude limits or meridian flip limits.\n", getTimestamp(), szCmdRa, szCmdDec);
//            fflush(Logfile);
//        }
//#endif
//        return ERR_LIMITSEXCEEDED;
//    }

//#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
//    if (Logfile) {
//        fprintf(Logfile, "[%s] [CiOptron::startSlewTo] Slewing to 1 of %i position(s) using commands:  Ra [0, 129,600,000]: %s and Dec [-32,400,000, +32,400,000]: %s\n", getTimestamp(), m_nCacheLimitStatus, szCmdRa, szCmdDec);
//        fflush(Logfile);
//    }
//#endif
    m_nCacheLimitStatus = NO_ISSUE_SLEW_TRACK_ONE_OPTION;
    if (m_nCacheLimitStatus == NO_ISSUE_SLEW_TRACK_ONE_OPTION) {
        // :MS1#   slew to normal position
        memset(szResp, 0, SERIAL_BUFFER_SIZE);  // clear response buffer
        nErr = sendCommand(":MS1#", szResp, 1);
        #if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        if (nErr) {
            fprintf(Logfile, "[%s] [CiOptron::startSlewTo] Error: sendCommand bombed sending :MS1.  nErr: %i\n", getTimestamp(), nErr);
            fflush(Logfile);
        }
        #endif
    } else if (m_nCacheLimitStatus == NO_ISSUE_SLEW_TRACK_TWO_OPTIONS) {
        // :MS2#   slew to counterweight up position I think
        memset(szResp, 0, SERIAL_BUFFER_SIZE);  // clear response buffer
        nErr = sendCommand(":MS2#", szResp, 1);
        #if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        if (nErr) {
            if (Logfile) {
                fprintf(Logfile, "[%s] [CiOptron::startSlewTo] Error: sendCommand bombed sending :MS2.  nErr: %i\n", getTimestamp(), nErr);
                fflush(Logfile);
            }
        }
        #endif
    } else if (m_nCacheLimitStatus == LIMITS_EXCEEDED_OR_BELOW_ALTITUDE) {
        #if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        if (Logfile) {
            fprintf(Logfile, "[%s] [CiOptron::startSlewTo] m_nCacheLimitStatus == LIMITS_EXCEEDED_OR_BELOW_ALTITUDE !!!  \n", getTimestamp());
            fflush(Logfile);
        }
        #endif
        return ERR_LIMITSEXCEEDED;  // redundant but just in case
    }

    if (nErr) {
        return nErr;
    } else if (atoi(szResp) == SLEW_EXCEED_LIMIT_OR_BELOW_ALTITUDE && m_nCacheLimitStatus == NO_ISSUE_SLEW_TRACK_ONE_OPTION) {
        #if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        //        fprintf(Logfile, "[%s] [CiOptron::startSlewTo] Error: Slewing to normal position was rejected by mount even though it told me it only had one position to go to.  Gettn out of dodge.\n", getTimestamp());
        if (Logfile) {
            fprintf(Logfile, "[%s] [CiOptron::startSlewTo] Error: Slewing to normal position was rejected by mount likely due to limit issues.  Gettn out of dodge.\n", getTimestamp());
            fflush(Logfile);
        }
        #endif
        return ERR_LIMITSEXCEEDED;  // regular slew to a place that is bad for mount
    } else if (atoi(szResp) == SLEW_EXCEED_LIMIT_OR_BELOW_ALTITUDE && m_nCacheLimitStatus == NO_ISSUE_SLEW_TRACK_TWO_OPTIONS) {
        // attempt was made to slew to counterweight up position, and mount said NO.. so attempt normal
        #if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        if (Logfile) {
            fprintf(Logfile, "[%s] [CiOptron::startSlewTo] Slewing to counterweight up position was rejected by mount.  Slewing to normal position.\n", getTimestamp());
            fflush(Logfile);
        }
        #endif
        m_nCacheLimitStatus = NO_ISSUE_SLEW_TRACK_ONE_OPTION;  // act as if we had only one option
        memset(szResp, 0, SERIAL_BUFFER_SIZE);  // clear response buffer
        nErr = sendCommand(":MS1#", szResp, 1);
        if (nErr) {
            #if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
            if (Logfile) {
                fprintf(Logfile, "[%s] [CiOptron::startSlewTo] Error: sendCommand bombed sending :MS2 then :MS1  nErr: %i\n", getTimestamp(), nErr);
                fflush(Logfile);
            }
            #endif
            return nErr;
        } else if (atoi(szResp) == SLEW_EXCEED_LIMIT_OR_BELOW_ALTITUDE) {
            #if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
            if (Logfile) {
                fprintf(Logfile, "[%s] [CiOptron::startSlewTo] Error: Slewing to normal position was rejected by mount after attempting to slew to counterweight up option.  (:MS2 then :MS1).  Gettn out of dodge.\n", getTimestamp());
                fflush(Logfile);
            }
            #endif
            return ERR_LIMITSEXCEEDED;
        } else {
            m_nStatus = SLEWING;
            slewToTimer.Reset();  // keep TSX under control
        }
    } else {
        m_nStatus = SLEWING;
        slewToTimer.Reset();  // keep TSX under control
    }

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::startSlewTo] end. \n", getTimestamp());
        fflush(Logfile);
    }
#endif

    return nErr;
}

int CiOptron::endSlewTo()
{
    // this is implemented in case we're pier West and will run into a flip limit soon
    int nErr = IOPTRON_OK;
    double dRaInDecimalHours, dDecInDecimalDegrees;
    char szResp[SERIAL_BUFFER_SIZE];

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::endSlewTo] called\n", getTimestamp());
        fflush(Logfile);
    }
#endif

    // find out where we are at
    getRaAndDec(dRaInDecimalHours, dDecInDecimalDegrees, true);

    // if we slewed to a place where we're about to flip anyway, reslew but only
    // if we had two options and are in counterweight up and are in pier west (OTA on west side of pier)
    if (m_nCacheLimitStatus==NO_ISSUE_SLEW_TRACK_TWO_OPTIONS) {
        // we had two options and chose counterweight up
        if (m_pierStatus==PIER_WEST && m_counterWeightStatus==COUNTER_WEIGHT_UP) {
            // picked the 'wrong' slew.  Re-slew to normal position
            memset(szResp, 0, SERIAL_BUFFER_SIZE);  // clear response buffer
            nErr = sendCommand(":MS1#", szResp, 1);
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
            if (nErr) {
                if (Logfile) {
                    fprintf(Logfile, "[%s] [CiOptron::endSlewTo] Error: sendCommand bombed sending :MS1 with value nErr: %i.  Gettn out of dodge.\n", getTimestamp(), nErr);
                    fflush(Logfile);
                }
                return nErr;
            }
#endif
            if (atoi(szResp) == 0) {
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
                if (Logfile) {
                    fprintf(Logfile, "[%s] [CiOptron::endSlewTo] Error: reslewing to 'normal' counterweight down position gave me a '0' back (The desired object is below the altitude limit or exceed the mechanical limits.).  Gettn out of dodge.\n", getTimestamp());
                    fflush(Logfile);
                }
#endif
                nErr = ERR_LIMITSEXCEEDED;  // all is lost
            } else {
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
                if (Logfile) {
                    fprintf(Logfile, "[%s] [CiOptron::endSlewTo] reslewing to 'normal' counterweight down position\n", getTimestamp());
                    fflush(Logfile);
                }
#endif
            }
        }
    }

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::endSlewTo] end\n", getTimestamp());
        fflush(Logfile);
    }
#endif

    m_nCacheLimitStatus = NO_STATUS;  // we've processed everygthing we could for now this must happen at end of slewTo
    return nErr;

}

int CiOptron::isSlewToComplete(bool &bComplete)
{
    int nErr = IOPTRON_OK;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::isSlewToComplete] called\n", getTimestamp());
        fflush(Logfile);
    }
#endif

    if(slewToTimer.GetElapsedSeconds()>2) {
        // go ahead and check by calling mount for status
        slewToTimer.Reset();

        nErr = getInfoAndSettings();

    } else {
        // we're checking for comletion too quickly and too often for no reason, just use local variable
    }

    if (m_nStatus == SLEWING || m_nStatus == FLIPPING) {
        bComplete = false;
    } else {
        bComplete = true;
    }

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::isSlewToComplete] returning : %s\n", getTimestamp(), bComplete?"true":"false");
        fflush(Logfile);
    }
#endif

    return nErr;
}

int CiOptron::isGPSReceivingDataPassive(bool &bGPSReceivingData)
{
    int nErr = IOPTRON_OK;
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::isGPSReceivingDataPassive] called \n", getTimestamp());
        fflush(Logfile);
    }
#endif

    bGPSReceivingData = (m_nGPSStatus == GPS_RECEIVING_VALID_DATA);

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::isGPSReceivingDataPassive] end. Result %s \n", getTimestamp(), bGPSReceivingData?"true":"false");
        fflush(Logfile);
    }
#endif
    return nErr;
}

int CiOptron::isGPSOrLatLongGood(bool &bGPSOrLatLongGood)
{
    float fLat;
    float fLong;
    bool bGPSGood;
    int nErr = IOPTRON_OK;
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::isGPSOrLatLongGood] called \n", getTimestamp());
        fflush(Logfile);
    }
#endif
    getInfoAndSettings();
    nErr = isGPSOrLatLongGoodPassive(bGPSOrLatLongGood);

    if (nErr) {
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        if (Logfile) {
            fprintf(Logfile, "[%s] [CiOptron::isGPSOrLatLongGood] Error: calling isGPSOrLatLongGoodPassive.  nErr: %i\n", getTimestamp(), nErr);
            fflush(Logfile);
        }
#endif
        return nErr;
    }

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::isGPSOrLatLongGood] end. Result %s \n", getTimestamp(), bGPSOrLatLongGood?"true":"false");
        fflush(Logfile);
    }
#endif
    return nErr;
}

int CiOptron::isGPSOrLatLongGoodPassive(bool &bGPSOrLatLongGood)
{
    float fLat;
    float fLong;
    bool bGPSReceivingData;
    int nErr = IOPTRON_OK;
    int nErr2 = IOPTRON_OK;
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::isGPSOrLatLongGoodPassive] called \n", getTimestamp());
        fflush(Logfile);
    }
#endif
    nErr = getLocationPassive(fLat, fLong);
    nErr = isGPSReceivingDataPassive(bGPSReceivingData);

    // determine if mount supports GPS and make determinations based on that
    bool bMountHasFunctioningGPS;
    mountHasFunctioningGPSPassive(bMountHasFunctioningGPS); // ignore error since never will err out

    if (bMountHasFunctioningGPS) {
        bGPSOrLatLongGood = (fLat && (fLat <=90) && (fLat >= -90) && fLong && (fLong <= 180) && (fLong >=-180)) || bGPSReceivingData;
    } else {
        bGPSOrLatLongGood = (fLat && (fLat <=90) && (fLat >= -90) && fLong && (fLong <= 180) && (fLong >=-180));
    }

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::isGPSOrLatLongGoodPassive] end. Result %s \n", getTimestamp(), bGPSOrLatLongGood?"true":"false");
        fflush(Logfile);
    }
#endif
    return nErr;
}

int CiOptron::parkMount()
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    int nParkResult;

    // ER: the scope comes with park already set
    nErr = sendCommand(":MP1#", szResp, 1);  // merely ask to park
    if(nErr)
        return nErr;

    nParkResult = atoi(szResp);
    if(nParkResult != 1)
        return ERR_CMDFAILED;

    return nErr; // todo: szResp says '1' or '0' for accepted or failed
}

int CiOptron::setParkPosition(double dAz, double dAlt)
{
    int nErr = IOPTRON_OK;
    char szCmd[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
    double dAzArcSec, dAltArcSec;

    // set az park position :  “:SPATTTTTTTTT#”
    // convert dAz to arcsec then to 0.01 arcsec
//    dAzArcSec = (dAz * 60 * 60) / 0.01;   wrong!
    dAzArcSec = ((dAz / 24.0 * 360.0) * 60.0 * 60.0) / 0.01;  // actually hundreths of arc sec
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::setParkPosition] setting  Park Az to : %d\n", getTimestamp(), int(dAzArcSec));
        fflush(Logfile);
    }
#endif
    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":SPA%09d#", int(dAzArcSec));
    nErr = sendCommand(szCmd, szResp, 1);
    if(nErr)
        return nErr;

    // set Alt park postion : “:SPHTTTTTTTT#”
    // convert dAlt to arcsec then to 0.01 arcsec
    dAltArcSec = (dAlt * 60.0 * 60.0) / 0.01;
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::setParkPosition] setting  Park Alt to : %d\n", getTimestamp(), int(dAltArcSec));
        fflush(Logfile);
    }
#endif
    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":SPH%08d#", int(dAltArcSec));
    nErr = sendCommand(szCmd, szResp, 1);
    if(nErr)
        return nErr;

    return nErr;

}

int CiOptron::getParkPosition(double &dAz, double &dAlt)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szParkAz[SERIAL_BUFFER_SIZE], szParkAlt[SERIAL_BUFFER_SIZE];
    int nAzArcSec, nAltArcSec;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getParkPosition] called\n", getTimestamp());
        fflush(Logfile);
    }
#endif

    // Response: “TTTTTTTTTTTTTTTTT#”
    nErr = sendCommand(":GPC#", szResp, 18);

    if(nErr)
        return nErr;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getParkPosition] :GPC# command response %s\n", getTimestamp(), szResp);
        fflush(Logfile);
    }
#endif

    memset(szParkAz, 0, SERIAL_BUFFER_SIZE);
    memset(szParkAlt, 0, SERIAL_BUFFER_SIZE);

    memcpy(szParkAlt, szResp, 8); // The first 8 digits indicate the altitude of parking position. Valid data range is [0, 32,400,000]. Note: The resolution is 0.01 arc-second.
    memcpy(szParkAz, szResp+8, 9); // The last 9 digits indicate the azimuth of parking position. Valid data range is [0, 129,600,000]. Note: The resolution is 0.01 arc-second.
    nAzArcSec = atoi(szParkAz);
    nAltArcSec = atoi(szParkAlt);
//    dAz = (nAzArcSec*0.01)/ 60 /60 ;   az calculated the same??
    dAz = (nAzArcSec * 0.01 * 24.0 / 360.0)/ 60.0 /60.0 ;
    dAlt = (nAltArcSec * 0.01)/ 60.0 /60.0 ;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getParkPosition] azmuth: %f and alt: %f\n", getTimestamp(), dAz, dAlt);
        fflush(Logfile);
    }
#endif
    return nErr;
}

int CiOptron::getAtPark(bool &bParked)
{
    int nErr = IOPTRON_OK;

    bParked = false;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getAtPark] called \n", getTimestamp());
        fflush(Logfile);
    }
#endif
    if(getAtParkTimer.GetElapsedSeconds()>2) {
        // go ahead and check by calling mount for status
        getAtParkTimer.Reset();

        nErr = getInfoAndSettings();
        if(nErr)
          return nErr;

    }
    // use m_bParked even if it was cached

    bParked = m_bParked;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getAtPark] end. Result: %s \n", getTimestamp(), bParked?"true":"false");
        fflush(Logfile);
    }
#endif
    return nErr;
}

int CiOptron::unPark()
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::unPark] \n", getTimestamp());
        fflush(Logfile);
    }
#endif
    nErr = sendCommand(":MP0#", szResp, 1);  // merely ask to unpark

    return nErr;
}


int CiOptron::getRefractionCorrEnabled(bool &bEnabled)
{
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getRefractionCorrEnabled] called. Current m_sModel value: %s.  Value of CEM120_EC2 %s\n", getTimestamp(), m_sModel, CEM120_EC2);
        fflush(Logfile);
    }
#endif
    int nErr = IOPTRON_OK;

    if (strcmp(m_sModel, IEQ30PRO) == 0) {
        bEnabled = false;
    } else if (strcmp(m_sModel, CEM60) == 0) {
        bEnabled = false;
    } else if (strcmp(m_sModel, CEM60_EC) == 0) {
        bEnabled = false;
    } else if (strcmp(m_sModel, CEM120) == 0) {
        bEnabled = true;
    } else if (strcmp(m_sModel, CEM120_EC) == 0) {
        bEnabled = true;
    } else if (strcmp(m_sModel, CEM120_EC2) == 0) {
        bEnabled = true;
    } else {
        bEnabled = false;
    }
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getRefractionCorrEnabled] finished result %s \n", getTimestamp(), bEnabled ? "true":"false");
        fflush(Logfile);
    }
#endif
    return nErr;
}

#pragma mark - used by MountDriverInterface
int CiOptron::Abort()
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::Abort]  abort called.  Stopping slewing and stopping tracking.\n", getTimestamp());
        fflush(Logfile);
    }
#endif

    // stop slewing
    nErr = sendCommand(":Q#", szResp, 1);
    if(nErr)
        return nErr;

    // stop tracking
    nErr = sendCommand(":ST0#", szResp, 1);

    return nErr;
}


int CiOptron::getInfoAndSettings()
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szTmp[SERIAL_BUFFER_SIZE];

    nErr = sendCommand(":GLS#", szResp, 24);
    if(nErr)
        return nErr;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getInfoAndSettings]  :GLS# response is: %s\n", getTimestamp(), szResp);
        fflush(Logfile);
    }
#endif

    memset(szTmp, 0, SERIAL_BUFFER_SIZE);
    memcpy(szTmp, szResp, 9);
    m_fLong = (atof(szTmp) * 0.01)/ 60.0 /60.0;

    memset(szTmp, 0, SERIAL_BUFFER_SIZE);
    memcpy(szTmp, szResp + 9, 8);
    m_fLat = ((atof(szTmp)-32400000)*0.01)/ 60.0 /60.0;

    memset(szTmp, 0, SERIAL_BUFFER_SIZE);
    memcpy(szTmp, szResp + 17, 1);
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getInfoAndSettings]  GPS status from mount returned is: %s, \n", getTimestamp(), szTmp);
        fflush(Logfile);
    }
#endif
    m_nGPSStatus = atoi(szTmp);

    memset(szTmp,0, SERIAL_BUFFER_SIZE);
    memcpy(szTmp, szResp+18, 1);
    m_nStatus = atoi(szTmp);

    memset(szTmp,0, SERIAL_BUFFER_SIZE);
    memcpy(szTmp, szResp+19, 1);
    m_nTrackingRate = atoi(szTmp);

    memset(szTmp,0, SERIAL_BUFFER_SIZE);
    memcpy(szTmp, szResp+21, 1);
    m_nTimeSource = atoi(szTmp);

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getInfoAndSettings]  MOUNT lat is : %f, MOUNT long is: %f, status is: %i, trackingRate is: %i, gpsStatus is: %i, timeSource is: %i\n", getTimestamp(), m_fLat , m_fLong , m_nStatus, m_nTrackingRate, m_nGPSStatus, m_nTimeSource);
        fflush(Logfile);
    }
#endif

    m_bParked = m_nStatus == PARKED?true:false;
    return nErr;

}

#pragma mark - internal set ra/dec on mount
int CiOptron::setRaAndDec(char *pszLocationCalling, double dRaInDecimalHours, double dDecInDecimalDegrees)
{
    int nErr = IOPTRON_OK;
    char szCmdRa[SERIAL_BUFFER_SIZE];
    char szCmdDec[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
    double dRaArcSec, dDecArcSec;


    dRaArcSec = ((dRaInDecimalHours / 24.0 * 360.0) * 60.0 * 60.0) / 0.01;  // actually hundreths of arc sec
    // :SRATTTTTTTTT#   ra  Valid data range is [0, 129,600,000].
    // Note: The resolution is 0.01 arc-second.
    snprintf(szCmdRa, SERIAL_BUFFER_SIZE, ":SRA%09d#", int(dRaArcSec));

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [%s] computed command for RA coordinate set: %s\n", getTimestamp(), pszLocationCalling, szCmdRa);
        fflush(Logfile);
    }
#endif

    nErr = sendCommand(szCmdRa, szResp, 1); // set RA
    if (nErr) {
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        if (Logfile) {
            fprintf(Logfile, "[%s] [%s] Error: sendCommand bombed sending %s.  nErr: %i\n", getTimestamp(), pszLocationCalling, szCmdRa, nErr);
            fflush(Logfile);
        }
#endif
        return nErr;
    }

    dDecArcSec = (dDecInDecimalDegrees * 60.0 * 60.0) / 0.01; // actually hundreths of arc sec - converts same way
    // :SdsTTTTTTTT#    dec  Valid data range is [-32,400,000, +32,400,000].
    // Note: The resolution is 0.01 arc-second.
    snprintf(szCmdDec, SERIAL_BUFFER_SIZE, ":Sd%+09d#", int(dDecArcSec));

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [%s] computed command for DEC coordinate set: %s\n", getTimestamp(), pszLocationCalling, szCmdDec);
        fflush(Logfile);
    }
#endif
    nErr = sendCommand(szCmdDec, szResp, 1);  // set DEC
    if (nErr) {
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        if (Logfile) {
            fprintf(Logfile, "[%s] [%s] Error: sendCommand bombed sending %s.  nErr: %i\n", getTimestamp(), pszLocationCalling, szCmdDec, nErr);
            fflush(Logfile);
        }
#endif
        return nErr;
    }

    return nErr;
}

int CiOptron::getMeridianTreatment(int &iBehavior, int &iDegreesPastMeridian)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szDegreesPastMeridian[SERIAL_BUFFER_SIZE], szBehavior[2];

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getMeridianTreatment] called\n", getTimestamp());
        fflush(Logfile);
    }
#endif

    // Response: “nnn#”
    // The first digit 0 stands for stop at the position limit set below.
    // The first digit 1 stands for flip at the position limit set below.
    // The last 2 digits stands for the position limit of degrees past meridian.
    nErr = sendCommand(":GMT#", szResp, 4);

    if(nErr)
        return nErr;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getMeridianTreatment] :GMT# command response %s\n", getTimestamp(), szResp);
        fflush(Logfile);
    }
#endif

    memset(szDegreesPastMeridian, 0, SERIAL_BUFFER_SIZE);
    memset(szBehavior, 0, 2);

    memcpy(szBehavior, szResp, 1); // The first digit is the behavior
    memcpy(szDegreesPastMeridian, szResp+1, 2); // The last 2 digits indicate degrees past meridian
    iBehavior = atoi(szBehavior);
    iDegreesPastMeridian = atoi(szDegreesPastMeridian);

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getMeridianTreatment] behavior: %i and degrees: %i\n", getTimestamp(), iBehavior, iDegreesPastMeridian);
        fflush(Logfile);
    }
#endif
    return nErr;
}

int CiOptron::getAltitudeLimit(int &iDegreesAltLimit)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szDegreesAltLimit[SERIAL_BUFFER_SIZE];

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getAltitudeLimit] called\n", getTimestamp());
        fflush(Logfile);
    }
#endif

    // Response: “snn#”
    // The first digit is the sign of the degree (why that would be negative is beyond me)
    // The last 2 digits stands for the degrees altitude limit
    nErr = sendCommand(":GAL#", szResp, 4);

    if(nErr)
        return nErr;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getAltitudeLimit] :GAL# command response %s\n", getTimestamp(), szResp);
        fflush(Logfile);
    }
#endif

    memset(szDegreesAltLimit, 0, SERIAL_BUFFER_SIZE);

    memcpy(szDegreesAltLimit, szResp, 3);
    iDegreesAltLimit = atoi(szDegreesAltLimit);

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] [CiOptron::getAltitudeLimit] degrees: %i\n", getTimestamp(), iDegreesAltLimit);
        fflush(Logfile);
    }
#endif
    return nErr;
}

int CiOptron::setMeridianTreatement(int iBehavior, int iDegreesPastMeridian)
{
    int nErr = IOPTRON_OK;
    char szCmd[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];

    // Command: “:SMTnnn#”
    //  Response: “1”
    //  This command will set the behavior about meridian treatment.
    //  The first digit 0 stands for stop at the position limit set below.
    //  The first digit 1 stands for flip at the position limit set below.
    //  The last 2 digits stands for the position limit of degrees past meridian.
    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":SMT%.1d%02d#", iBehavior, iDegreesPastMeridian);

    #if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] computed command for setting meridian treatment: %s\n", getTimestamp(), szCmd);
        fflush(Logfile);
    }
    #endif

    nErr = sendCommand(szCmd, szResp, 1);  // set meridian treatment
    if (nErr) {
        #if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        if (Logfile) {
            fprintf(Logfile, "[%s] Error: sendCommand for setting meridian treatment bombed: command was: %s. nErr: %i\n", getTimestamp(), szCmd, nErr);
            fflush(Logfile);
        }
        #endif
        return nErr;
    }
    m_nDegreesPastMeridian = iDegreesPastMeridian; // cache this value but only if everything above worked

    return nErr;
}
int CiOptron::setAltitudeLimit(int iDegreesAltLimit)
{
    int nErr = IOPTRON_OK;
    char szCmd[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];

    // Command: “:SALsnn#”
    // Response: “1”
    // This command sets the altitude limit. The altitude limit not only applies to tracking,
    // but also applies to slewing. Movement caused by arrow buttons does not affect by this limit.
    // Tracking will be stopped if you move the mount to a position exceeds any limit.
    // Note: Valid data range is [-89, +89]. The resolution is 1 degree.
    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":SAL%+.2d#", iDegreesAltLimit);

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    if (Logfile) {
        fprintf(Logfile, "[%s] computed command for setting altitude limit: %s\n", getTimestamp(), szCmd);
        fflush(Logfile);
    }
#endif

    nErr = sendCommand(szCmd, szResp, 1);  // set altitude limit
    if (nErr) {
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        if (Logfile) {
            fprintf(Logfile, "[%s] Error: sendCommand for setting altitude limit bombed: command was: %s. nErr: %i\n", getTimestamp(), szCmd, nErr);
            fflush(Logfile);
        }
#endif
        return nErr;
    }
    m_nAltitudeLimit = iDegreesAltLimit; // set only if all succeeded

    return nErr;
}

#ifdef IOPTRON_DEBUG
char* CiOptron::getTimestamp()
{
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    return timestamp;
}
#endif


