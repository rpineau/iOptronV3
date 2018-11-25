#include "iOptronV3.h"

// Constructor for IOPTRON
CiOptron::CiOptron() {

    m_bIsConnected = false;

    m_bParked = false;  // probably not good to assume we're parked.  Power could have shut down or we're at zero position or we're parked
    m_nGPSStatus = GPS_BROKE_OR_MISSING;  // unread to start (stating broke or missing)
    m_nTimeSource = TIME_SRC_UNKNOWN;  // unread to start

    m_dRa = 0.0;
    m_dDec = 0.0;
    slewToTimer.Reset();
    cmdTimer.Reset();
    trackRatesTimer.Reset();
    getAtParkTimer.Reset();
}

void CiOptron::setLogFile(FILE *daFile) {
    Logfile = daFile;
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] IOPTRON setLogFile Called\n", timestamp);
    fflush(Logfile);
#endif
}

CiOptron::~CiOptron(void)
{
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] IOPTRON Destructor Called\n", timestamp );
    fflush(Logfile);
#endif
#ifdef IOPTRON_DEBUG
    // Close LogFile
    if (Logfile) fclose(Logfile);
#endif
}

int CiOptron::Connect(char *pszPort)
{
    int nErr = IOPTRON_OK;
    int connectSpeed = 115200;  // default for CEM120xxx

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] CiOptron::Connect Called %s\n", timestamp, pszPort);
    fflush(Logfile);
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

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CiOptron::Connect connected at %d on %s\n", timestamp, connectSpeed, pszPort);
    fflush(Logfile);
#endif
    // get more info and status
    getInfoAndSettings();
    return nErr;
}


int CiOptron::Disconnect(void)
{
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] CiOptron::Disconnect Called\n", timestamp);
    fflush(Logfile);
#endif
	if (m_bIsConnected) {
        if(m_pSerx){
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] CiOptron::Disconnect closing serial port\n", timestamp);
            fflush(Logfile);
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
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::getRateName] index was %i\n", timestamp, nZeroBasedIndex);
    fflush(Logfile);
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
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::startOpenSlew] setting to Dir %d\n", timestamp, Dir);
    fprintf(Logfile, "[%s] [CiOptron::startOpenSlew] Setting rate to %d\n", timestamp, nRate);
    fflush(Logfile);
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
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::stopOpenLoopMove] Dir was %d\n", timestamp, m_nOpenLoopDir);
    fflush(Logfile);
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
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] *** CiOptron::sendCommand sending : %s\n", timestamp, pszCmd);
    fflush(Logfile);
#endif

    nErr = m_pSerx->writeFile((void *)pszCmd, strlen((char*)pszCmd), ulBytesWrite);
    m_pSerx->flushTx();
    if(nErr)
        return nErr;

    // read response
    nErr = readResponse(szResp, nExpectedResultLen);
    if(nErr) {
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] *** CiOptron::sendCommand ***** ERROR READING RESPONSE **** error = %d , response : %s\n", timestamp, nErr, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
#if defined ND_DEBUG && ND_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] *** CiOptron::sendCommand response : %s\n", timestamp, szResp);
    fflush(Logfile);
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
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CiOptron::readResponse] szRespBuffer = %s\n", timestamp, szRespBuffer);
        fflush(Logfile);
#endif
        return nErr;
    }

    if (ulBytesActuallyRead !=nBytesToRead) { // timeout or something screwed up with command passed in
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] CiOptron::readResponse number of bytes read not what expected.  Number of bytes read: %lu.  Number expected: %i\n", timestamp, ulBytesActuallyRead, nBytesToRead);
        fflush(Logfile);
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
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::getMountInfo] called\n", timestamp);
    fflush(Logfile);
#endif

    nErr = sendCommand(":MountInfo#", szResp, 4);
    if(nErr)
        return nErr;

    if (strcmp(szResp, IEQ30PRO) == 0) {
        strncpy(model, "iEQ30 Pro", strMaxLen);
        strncpy(m_sModel, IEQ30PRO, 5);
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


int CiOptron::getFirmwareVersion(char *pszVersion, unsigned int nStrMaxLen)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    std::string sFirmwares;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::getFirmwareVersion] called\n", timestamp);
    fflush(Logfile);
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
int CiOptron::getRaAndDec(double &dRa, double &dDec)
{


#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::getRaAndDec] called \n", timestamp);
    fflush(Logfile);
#endif
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    char szRa[SERIAL_BUFFER_SIZE], szDec[SERIAL_BUFFER_SIZE];
    int nRa, nDec;

    // don't ask the mount too often, returned cached value
    if(cmdTimer.GetElapsedSeconds()<0.1) {
        dRa = m_dRa;
        dDec = m_dDec;
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CiOptron::getRaAndDec] SHORT circuiting TSX from going nuts on the mount. \n", timestamp);
        fflush(Logfile);
#endif
        return nErr;
    }
    cmdTimer.Reset();
    nErr = sendCommand(":GEP#", szResp, 21);
    if(nErr)
        return nErr;
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::getRaAndDec] response to :GEP# %s\n", timestamp, szResp);
    fflush(Logfile);
#endif

    memset(szRa, 0, SERIAL_BUFFER_SIZE);
    memset(szDec, 0, SERIAL_BUFFER_SIZE);
    memcpy(szDec, szResp, 9);
    memcpy(szRa, szResp+9, 9);
    nRa = atoi(szRa);
    nDec = atoi(szDec);
    dRa = (nRa*0.01)/ 60 /60 ;
    dDec = (nDec*0.01)/ 60 /60 ;

    m_dRa = dRa;
    m_dDec = dDec;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::getRaAndDec] nRa : %d\n", timestamp, nRa);
    fprintf(Logfile, "[%s] [CiOptron::getRaAndDec] nDec : %d\n", timestamp, nDec);
    fprintf(Logfile, "[%s] [CiOptron::getRaAndDec] Ra : %f\n", timestamp, dRa);
    fprintf(Logfile, "[%s] [CiOptron::getRaAndDec] Dec : %f\n", timestamp, dDec);
    fflush(Logfile);
#endif

    return nErr;
}

#pragma mark - Sync and Cal - used by SyncMountInterface
int CiOptron::syncTo(double dRa, double dDec)
{
    int nErr = IOPTRON_OK;
    int nRa, nDec;
    char szCmd[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::syncTo] called Ra : %f  Dec: %f\n", timestamp, dRa, dDec);
    fflush(Logfile);
#endif
    nErr = getInfoAndSettings();
    if(nErr)
        return nErr;

    if (m_nGPSStatus != GPS_RECEIVING_VALID_DATA)
        return ERR_CMDFAILED;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::syncTo] verify GPS good and connected.  Ra : %f  Dec: %f\n", timestamp, dRa, dDec);
    fflush(Logfile);
#endif

    // iOptron:
    // TTTTTTTT(T) in 0.01 arc-seconds

    //  current logitude comes from getInfoAndSettings() and returns sTTTTTTTT (1+8) where
    //    s is the sign -/+ and TTTTTTTT is longitude in 0.01 arc-seconds
    //    range: [-64,800,000, +64,800,000] East is positive, and the resolution is 0.01 arc-second.
    // TSX provides RA and DEC in degrees with a decimal
    nRa = int((dRa*60*60)/0.01);
    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":SRA%+.8d#", nRa);

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::syncTo] computed command for RA coordinate set: %s\n", timestamp, szCmd);
    fflush(Logfile);
#endif

    nErr = sendCommand(szCmd, szResp, 1);  // set RA
    if(nErr) {
        return nErr;
    }

    //  current latitude againg from getInfoAndSettings() returns TTTTTTTT (8)
    //    which is current latitude plus 90 degrees.
    //    range is [0, 64,800,000]. Note: North is positive, and the resolution is 0.01 arc-second
    nDec = int(((dDec+90)*60*60)/0.01);
    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":Sds%+.8d#", nDec);

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::syncTo] computed command for DEC coordinate set: %s\n", timestamp, szCmd);
    fflush(Logfile);
#endif

    nErr = sendCommand(szCmd, szResp, 1);  // set DEC
    if(nErr) {
        // clear RA?
        return nErr;
    }

    nErr = sendCommand(":CM#", szResp, 1);  // call Snc
    if(nErr) {
        // clear RA?
        // clear DEC?
        return nErr;
    }

     return nErr;
}

#pragma mark - tracking rates
int CiOptron::setSiderealTrackingOn() {
    // special implementation avoiding manually setting the tracking rate

    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::setSiderealTrackingOn] called \n", timestamp);
    fflush(Logfile);
#endif

    // Set tracking to sidereal
    nErr = sendCommand(":RT0#", szResp, 1);  // use macro command to set this

    if (nErr)
        return nErr;

    // and turn on
    nErr = sendCommand(":ST1#", szResp, 1);  // and start tracking

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::setSiderealTrackingOn] finished.  Result: %s\n", timestamp, szResp);
    fflush(Logfile);
#endif

    return nErr;

}

int CiOptron::setTrackingOff() {
    // special implementation avoiding manually setting the tracking rate

    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::setTrackingOff] called \n", timestamp);
    fflush(Logfile);
#endif

    nErr = sendCommand(":ST0#", szResp, 1);  // use macro command to set this

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::setTrackingOff] finished.  Result: %s\n", timestamp, szResp);
    fflush(Logfile);
#endif

    return nErr;

}

int CiOptron::setTrackingRates(bool bTrackingOn, bool bIgnoreRates, double dRaRateArcSecPerSec, double dDecRateArcSecPerSec)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];
    bool bCustomRate = false;  // assume not a custom rate

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::setTrackingRates] called bTrackingOn: %s, bIgnoreRate: %s, dRaRateArcSecPerSec: %f, dDecRateArcSecPerSec %f\n", timestamp, bTrackingOn?"true":"false", bIgnoreRates?"true":"false", dRaRateArcSecPerSec, dDecRateArcSecPerSec);
    fflush(Logfile);
#endif

// :RRnnnnn# - set the tracking rate of the RA axis to n.nnnn *sidereal rate
//           - Valid data range is [0.1000, 1.9000] * sidereal rate.
//           - Data entered with this command will be remembered through a power cycle and automatically re- applied on the next power up.
// :RT4# - “Custom Tracking Rate”  must be selected before this command to take effect

// These commands select the tracking rate: select sidereal (“:RT0#”), lunar (“:RT1#”), solar (“:RT2#”), King (“:RT3#”), or custom (“:RT4#”).

    if (dRaRateArcSecPerSec == 0) {
        strcpy(szCmd, ":RT3#");  // use 'macro' command to set sidereal/king (King is better)
    } else if (dRaRateArcSecPerSec == 0.5490149) {
        strcpy(szCmd, ":RT1#");  // use 'macro' command to set to lunar
    } else if (dRaRateArcSecPerSec == 0.0410681) {
        strcpy(szCmd, ":RT2#");  // use 'macro' command to set to solar
    } else {
        bCustomRate = true;
        // some custom rate (tracking a satellite or commet or something close)
        snprintf(szCmd, SERIAL_BUFFER_SIZE, ":RR%1.4f#", dRaRateArcSecPerSec);
        //nErr = sendCommand(szCmd, szResp, 1);  // sets tracking rate and returns a single byte
        if (nErr)
            return nErr;
        strcpy(szCmd, ":RT4#");  // use 'macro' command to set to custom
    }

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::setTrackingRates] determined we are custom: %s.  Sending command: %s\n", timestamp, bCustomRate?"true":"false", szCmd);
    fflush(Logfile);
#endif
    //nErr = sendCommand(szCmd, szResp, 1);  // set tracking 'go'.  all commands return a single byte
    if (nErr)
        return nErr;

    return nErr;
}

int CiOptron::getTrackRates(bool &bTrackingOn, double &dTrackRaArcSecPerSec, double &dTrackDecArcSecPerSec)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::getTrackRates] called.\n", timestamp);
    fflush(Logfile);
#endif

    // don't ask the mount its general status too often .. doesn't change much
    if(trackRatesTimer.GetElapsedSeconds()>1.0) {
        getInfoAndSettings();
        trackRatesTimer.Reset();

        // if we really wanted to be mega flexible, we'd call :GTR#
        // Response: “nnnnn#”
        // This command gets the saved custom tracking rate, the tracking rate is n.nnnn * sidereal rate.
        // Valid data range is [0.1000, 1.9000] * sidereal rate.

        nErr = sendCommand(":GTR#", szResp, 6);  // lets see what we're at anyway

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CiOptron::getTrackRates] asking mount for actual rate: %s.\n", timestamp, szResp);
        fflush(Logfile);
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
                dTrackRaArcSecPerSec = 0.0;
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
                dTrackRaArcSecPerSec = 0.0;
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
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::getTrackRates] done. returning: bTrackingOn: %s, dTrackRaArcSecPerHr: %f, dTrackDecArcSecPerHr: %f\n", timestamp, bTrackingOn?"true":"false", dTrackRaArcSecPerSec, dTrackDecArcSecPerSec);
    fflush(Logfile);
#endif
    return nErr;
}

#pragma mark - UI controlls
int CiOptron::gotoZeroPosition() {
    // special implementation for CEM120xx and CEM60xx only

    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::gotoZeroPosition] called \n", timestamp);
    fflush(Logfile);
#endif

    // Goto Zero position / home position
    nErr = sendCommand(":MH#", szResp, 1);

    if (nErr)
        return nErr;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::gotoZeroPosition] finished.  Result: %s\n", timestamp, szResp);
    fflush(Logfile);
#endif

    return nErr;

}

int CiOptron::getAtZeroPosition(bool &bAtZero) {
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
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::gotoFlatsPosition] called \n", timestamp);
    fflush(Logfile);
#endif

    // set alt/az position
    // altitude: :SasTTTTTTTT# (Valid data range is [-32,400,000, 32,400,000])
    nErr = sendCommand(":Sa+32400000#", szResp, 1);  // point straight up
    if (nErr)
        return nErr;

    // azimuth: :SzTTTTTTTTT# (Valid data range is [0, 129,600,000])
    nErr = sendCommand(":Sz000000000#", szResp, 1);  // point north
    if (nErr)
        return nErr;

    // Goto Zero alt/az position defined
    nErr = sendCommand(":MSS#", szResp, 1);
    if (nErr)
        return nErr;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::gotoFlatsPosition] MSS slew command finished.  Result (want 1): %s\n", timestamp, szResp);
    fflush(Logfile);
#endif
    // dont track
    nErr = sendCommand(":ST0#", szResp, 1);

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::gotoFlatsPosition] finished.  Result of final stop-tracking command (want 0): %i\n", timestamp, nErr);
    fflush(Logfile);
#endif

    return nErr;

}

int CiOptron::findZeroPosition() {
    // special implementation for CEM120xx and CEM60xx only

    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::calibrateZeroPosition] called \n", timestamp);
    fflush(Logfile);
#endif

    // Find Zero position / home position
    nErr = sendCommand(":MSH#", szResp, 1);

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::calibrateZeroPosition] finished.  Result: %s\n", timestamp, szResp);
    fflush(Logfile);
#endif

    return nErr;

}

int CiOptron::getUtcOffset(char *pszUtcOffsetInMins)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::getUtcOffset] called \n", timestamp);
    fflush(Logfile);
#endif

    // Get time related info
    nErr = sendCommand(":GUT#", szResp, 19);

    memset(pszUtcOffsetInMins,0, SERIAL_BUFFER_SIZE);
    memcpy(pszUtcOffsetInMins, szResp, 4);

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::getUtcOffset] finished.  Result: %s\n", timestamp, pszUtcOffsetInMins);
    fflush(Logfile);
#endif
    return nErr;
}

int CiOptron::setUtcOffset(char *pszUtcOffsetInMins)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::setUtcOffset] called \n", timestamp);
    fflush(Logfile);
#endif

    // :SGsMMM#
    // This command sets the minute offset from UTC (The Daylight-Saving Time will
    // not be take account into this value). Valid data range is [-720, +780].
    // Note: the resolution is 1 minute.

    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":SG%s#", pszUtcOffsetInMins);

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::setUtcOffset] buffer to send to mount %s\n", timestamp, szCmd);
    fflush(Logfile);
#endif

    nErr = sendCommand(szCmd, szResp, 1);

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::setUtcOffset] done\n", timestamp);
    fflush(Logfile);
#endif

    return nErr;
}

int CiOptron::getDST(bool &bDaylight)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szTmp[SERIAL_BUFFER_SIZE];

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::getDST] called \n", timestamp);
    fflush(Logfile);
#endif

    // Get time related info
    nErr = sendCommand(":GUT#", szResp, 19);

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::getDST] send of :GUT# finished.  Result: %s\n", timestamp, szResp);
    fflush(Logfile);
#endif

    memset(szTmp,0, SERIAL_BUFFER_SIZE);
    memcpy(szTmp, szResp+4, 1);
    bDaylight = (atoi(szTmp) == 1);

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::getDST] finished.  Result: %s with error code: %i\n", timestamp, bDaylight ?"true":"false", nErr);
    fflush(Logfile);
#endif

    return nErr;
}

int CiOptron::setDST(bool bDaylight)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::setDST] called \n", timestamp);
    fflush(Logfile);
#endif

    //  Command: “:SDS0#” or “:SDS1#”
    //  Response: “1”
    //  These commands set the status of Daylight Saving Time.
    //  “:SDS1#” means Daylight Saving Time has been observed,
    //  “:SDS0#” means Daylight Saving Time has not been observed.

    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":SDS%.1d#", bDaylight?1:0);

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::setDST] buffer to send to mount %s\n", timestamp, szCmd);
    fflush(Logfile);
#endif

    nErr = sendCommand(szCmd, szResp, 1);

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::setDST] done\n", timestamp);
    fflush(Logfile);
#endif

    return nErr;
}


int CiOptron::getGPSStatusString(char *gpsStatus, unsigned int strMaxLen)
{
    int nErr = IOPTRON_OK;
    getInfoAndSettings();  // this case we want to be accurate
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

#pragma mark - Limits
int CiOptron::getLimits(double &dHoursEast, double &dHoursWest)
{
    int nErr = IOPTRON_OK;
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::getLimits] called. doing nothing for now\n", timestamp);
    fflush(Logfile);
#endif
    // todo: cache result and return them as these don't change much.
    return nErr;
}



#pragma mark - Slew
int CiOptron::startSlewTo(double dRaInDecimalHours, double dDecInDecimalDegrees)
{
    int nErr = IOPTRON_OK;
    bool bGPSGood;
    char szCmdRa[SERIAL_BUFFER_SIZE];
    char szCmdDec[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
    double dRaArcSec, dDecArcSec;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::startSlewTo] called Ra: %f and Dec: %f\n", timestamp, dRaInDecimalHours, dDecInDecimalDegrees);
    fflush(Logfile);
#endif

    nErr = isGPSGood(bGPSGood);

    if (!bGPSGood) {
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CiOptron::startSlewTo] called Ra: %f and Dec: %f .. ABORTING due to GPS signal not being good\n", timestamp, dRaInDecimalHours, dDecInDecimalDegrees);
        fflush(Logfile);
#endif
        return ERR_ABORTEDPROCESS;
    }

    dRaArcSec = (dRaInDecimalHours * 60 * 60) / 0.01;  // actually hundreths of arc sec
    // :SRATTTTTTTTT#   ra  Valid data range is [0, 129,600,000]. Note: The resolution is 0.01 arc-second.
    snprintf(szCmdRa, SERIAL_BUFFER_SIZE, ":SRA%09d#", int(dRaArcSec));
    nErr = sendCommand(szCmdRa, szResp, 1);
    if(nErr)
        return nErr;

    dDecArcSec = (dDecInDecimalDegrees * 60 * 60) / 0.01; // actually hundreths of arc sec - converts same way
    // :SdsTTTTTTTT#    dec  Valid data range is [-32,400,000, +32,400,000]. Note: The resolution is 0.01 arc-second.
    snprintf(szCmdDec, SERIAL_BUFFER_SIZE, ":Sd%+08d#", int(dDecArcSec));
    nErr = sendCommand(szCmdDec, szResp, 1);
    if(nErr)
        return nErr;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::startSlewTo] Commands:  Ra [0, 129,600,000]: %s and Dec [-32,400,000, +32,400,000]: %s\n", timestamp, szCmdRa, szCmdDec);
    fflush(Logfile);
#endif

    // :MS1#   slew to them
    nErr = sendCommand(":MS1#", szResp, 1);
    if (nErr) {
        return nErr;
    } else if (atoi(szResp) == 0) {
        return ERR_LIMITSEXCEEDED;
    } else {
        m_nStatus = SLEWING;
        slewToTimer.Reset();  // keep TSX under control
    }

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::startSlewTo] end. \n", timestamp);
    fflush(Logfile);
#endif
    if(nErr)
        return nErr;

    return nErr;
}

int CiOptron::isSlewToComplete(bool &bComplete)
{
    int nErr = IOPTRON_OK;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::isSlewToComplete] called\n", timestamp);
    fflush(Logfile);
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
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::isSlewToComplete] returning : %s\n", timestamp, bComplete?"true":"false");
    fflush(Logfile);
#endif

    return nErr;
}

int CiOptron::isGPSGood(bool &bGPSGood)
{
    int nErr = IOPTRON_OK;
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::isGPSGood] called \n", timestamp);
    fflush(Logfile);
#endif

    bGPSGood = m_nGPSStatus == GPS_RECEIVING_VALID_DATA;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::isGPSGood] end. Result %s \n", timestamp, bGPSGood?"true":"false");
    fflush(Logfile);
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
    dAzArcSec = (dAz * 60 * 60) / 0.01;
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::setParkPosition] setting  Park Az to : %d\n", timestamp, int(dAzArcSec));
    fflush(Logfile);
#endif
    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":SPA%09d#", int(dAzArcSec));
    nErr = sendCommand(szCmd, szResp, 1);
    if(nErr)
        return nErr;

    // set Alt park postion : “:SPHTTTTTTTT#”
    // convert dAlt to arcsec then to 0.01 arcsec
    dAltArcSec = (dAlt * 60 * 60) / 0.01;
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::setParkPosition] setting  Park Alt to : %d\n", timestamp, int(dAltArcSec));
    fflush(Logfile);
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
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::getParkPosition] called\n", timestamp);
    fflush(Logfile);
#endif

    // Response: “TTTTTTTTTTTTTTTTT#”
    nErr = sendCommand(":GPC#", szResp, 18);

    if(nErr)
        return nErr;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::getParkPosition] :GPC# command response %s\n", timestamp, szResp);
    fflush(Logfile);
#endif

    memset(szParkAz, 0, SERIAL_BUFFER_SIZE);
    memset(szParkAlt, 0, SERIAL_BUFFER_SIZE);

    memcpy(szParkAlt, szResp, 8); // The first 8 digits indicate the altitude of parking position. Valid data range is [0, 32,400,000]. Note: The resolution is 0.01 arc-second.
    memcpy(szParkAz, szResp+8, 9); // The last 9 digits indicate the azimuth of parking position. Valid data range is [0, 129,600,000]. Note: The resolution is 0.01 arc-second.
    nAzArcSec = atoi(szParkAz);
    nAltArcSec = atoi(szParkAlt);
    dAz = (nAzArcSec*0.01)/ 60 /60 ;
    dAlt = (nAltArcSec*0.01)/ 60 /60 ;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::getParkPosition] azmuth: %f and alt: %f\n", timestamp, dAz, dAlt);
    fflush(Logfile);
#endif
    return nErr;
}

int CiOptron::getAtPark(bool &bParked)
{
    int nErr = IOPTRON_OK;

    bParked = false;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::getAtPark] called \n", timestamp);
    fflush(Logfile);
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
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::getAtPark] end. Result: %s \n", timestamp, bParked?"true":"false");
    fflush(Logfile);
#endif
    return nErr;
}

int CiOptron::unPark()
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::unPark] \n", timestamp);
    fflush(Logfile);
#endif
    nErr = sendCommand(":MP0#", szResp, 1);  // merely ask to unpark
    if(nErr)
        return nErr;

    // dont start tracking.  nErr = setSiderealTrackingOn();  // use simple command which may use "King" tracking which is better than straight sidereal

    if(nErr) {
#ifdef IOPTRON_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CiOptron::unPark] Error setting tracking to sidereal\n", timestamp);
        fflush(Logfile);
#endif
        snprintf(m_szLogBuffer,IOPTRON_LOG_BUFFER_SIZE,"[CiOptron::unPark] Error setting tracking to sidereal");
        m_pLogger->out(m_szLogBuffer);
    }
    return nErr;
}


int CiOptron::getRefractionCorrEnabled(bool &bEnabled)
{
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::getRefractionCorrEnabled] called. Current m_sModel value: %s.  Value of CEM120_EC2 %s\n", timestamp, m_sModel, CEM120_EC2);
    fflush(Logfile);
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
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::getRefractionCorrEnabled] finished result %s \n", timestamp, bEnabled ? "true":"false");
    fflush(Logfile);
#endif
    return nErr;
}

#pragma mark - used by MountDriverInterface
int CiOptron::Abort()
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::Abort]  abort called.  Stopping slewing and stopping tracking.\n", timestamp);
    fflush(Logfile);
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
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::getInfoAndSettings]  :GLS# response is: %s\n", timestamp, szResp);
    fflush(Logfile);
#endif

    memset(szTmp,0, SERIAL_BUFFER_SIZE);
    memcpy(szTmp, szResp, 9);
    m_fLong = atof(szTmp);

    memset(szTmp,0, SERIAL_BUFFER_SIZE);
    memcpy(szTmp, szResp+9, 8);
    m_fLat = atof(szTmp)-32400000;

    memset(szTmp,0, SERIAL_BUFFER_SIZE);
    memcpy(szTmp, szResp+17, 1);
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
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::getInfoAndSettings]  GPS lat is : %f, GPS long is: %f, status is: %i, trackingRate is: %i, gpsStatus is: %i, timeSource is: %i\n", timestamp, (m_fLat*0.01)/ 60 /60 , (m_fLong*0.01)/ 60 /60 , m_nStatus, m_nTrackingRate, m_nGPSStatus, m_nTimeSource);
    fflush(Logfile);
#endif

    m_bParked = m_nStatus == PARKED?true:false;
    return nErr;

}


