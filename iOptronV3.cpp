#include "iOptronV3.h"

// Constructor for IOPTRON
CiOptron::CiOptron() {

    m_bIsConnected = false;

    m_bParked = false;  // probably not good to assume we're parked.  Power could have shut down or we're at zero position or we're parked

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

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] CiOptron::Connect Called %s\n", timestamp, pszPort);
    fflush(Logfile);
#endif

    // 9600 8N1 (non CEM120xxx mounts) or 115200 (CEM120xx mounts)
    if(m_pSerx->open(pszPort, 115200, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1") == 0)
        m_bIsConnected = true;
    else
        m_bIsConnected = false;

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    // get mount model to see if we're properly connected
    nErr = getMountInfo(m_szHardwareModel, SERIAL_BUFFER_SIZE);
    if(nErr)
        m_bIsConnected = false;

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

#pragma mark - Used by OpenLoopMoveInterface
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

#pragma mark - Used by OpenLoopMoveInterface
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
        nErr = sendCommand(":Q#", szResp, SERIAL_BUFFER_SIZE);
    }

    // select rate.  :SRn# n=1..7  1=1x, 2=2x, 3=8x, 4=16x, 5=64x, 6=128x, 7=256x
    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":SR%1d#", nRate+1);
    nErr = sendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    // figure out direction
    switch(Dir){
        case MountDriverInterface::MD_NORTH:
            nErr = sendCommand(":mn#", szResp, SERIAL_BUFFER_SIZE);
            break;
        case MountDriverInterface::MD_SOUTH:
            nErr = sendCommand(":ms#", szResp, SERIAL_BUFFER_SIZE);
            break;
        case MountDriverInterface::MD_EAST:
            nErr = sendCommand(":me#", szResp, SERIAL_BUFFER_SIZE);
            break;
        case MountDriverInterface::MD_WEST:
            nErr = sendCommand(":mw#", szResp, SERIAL_BUFFER_SIZE);
            break;
    }

    return nErr;
}

#pragma mark - Used by OpenLoopMoveInterface
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
            nErr = sendCommand(":qD#", szResp, SERIAL_BUFFER_SIZE);
            break;
        case MountDriverInterface::MD_EAST:
        case MountDriverInterface::MD_WEST:
            nErr = sendCommand(":qR#", szResp, SERIAL_BUFFER_SIZE);
            break;
    }

    return nErr;
}


#pragma mark - IOPTRON communication
int CiOptron::sendCommand(const char *pszCmd, char *pszResult, int nResultMaxLen, int nExpectedResultLen)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    unsigned long  ulBytesWrite;

    m_pSerx->purgeTxRx();

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CiOptron::sendCommand sending : %s\n", timestamp, pszCmd);
    fflush(Logfile);
#endif

    nErr = m_pSerx->writeFile((void *)pszCmd, strlen((char*)pszCmd), ulBytesWrite);
    m_pSerx->flushTx();
    if(nErr)
        return nErr;

    // read response
    nErr = readResponse(szResp, nResultMaxLen, nExpectedResultLen);
    if(nErr) {
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] CiOptron::sendCommand ***** ERROR READING RESPONSE **** error = %d , response : %s\n", timestamp, nErr, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
#if defined ND_DEBUG && ND_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CiOptron::sendCommand response : %s\n", timestamp, szResp);
    fflush(Logfile);
#endif

    if(pszResult)
        strncpy(pszResult, szResp, nResultMaxLen);

    return nErr;
}

int CiOptron::readResponse(char *szRespBuffer, int nBufferLen, int nResultLen)
{
    int nErr = IOPTRON_OK;
    unsigned long ulBytesRead = 0;
    unsigned long ulTotalBytesRead = 0;
    char *pszBufPtr;

    memset(szRespBuffer, 0, (size_t) nBufferLen);
    pszBufPtr = szRespBuffer;

    do {
        nErr = m_pSerx->readFile(pszBufPtr, 1, ulBytesRead, MAX_TIMEOUT);
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
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] CiOptron::readResponse Timeout while waiting for response from controller\n", timestamp);
        fflush(Logfile);
#endif


        if (ulBytesRead !=1) {// timeout
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] CiOptron::readResponse Timeout while waiting for response from controller\n", timestamp);
            fflush(Logfile);
#endif

            nErr = IOPTRON_BAD_CMD_RESPONSE;
            break;
        }
        ulTotalBytesRead += ulBytesRead;
        if(ulTotalBytesRead == nResultLen)
            break;
    } while ( *pszBufPtr++ != '#' && ulTotalBytesRead < nBufferLen  );

    if(ulTotalBytesRead && *(pszBufPtr-1) == '#')
        *(pszBufPtr-1) = 0; //remove the #

    return nErr;
}



#pragma mark - mount controller informations
int CiOptron::getMountInfo(char *model, unsigned int strMaxLen)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    int nModel;
    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::getMountInfo] called\n", timestamp);
    fflush(Logfile);
#endif

    nErr = sendCommand(":MountInfo#", szResp, SERIAL_BUFFER_SIZE, 4);
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

    nErr = sendCommand(":FW1#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    sFirmwares+= szResp;
    sFirmwares+= " ";

    nErr = sendCommand(":FW2#", szResp, SERIAL_BUFFER_SIZE);
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

    nErr = sendCommand(":GEP#", szResp, SERIAL_BUFFER_SIZE);
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

    nErr = sendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);  // set RA
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

    nErr = sendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);  // set DEC
    if(nErr) {
        // clear RA?
        return nErr;
    }

    nErr = sendCommand(":CM#", szResp, SERIAL_BUFFER_SIZE);  // call Snc
    if(nErr) {
        // clear RA?
        // clear DEC?
        return nErr;
    }

     return nErr;
}


int CiOptron::isGPSGood(bool &bGPSGood)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::isGPSGood] called \n", timestamp);
    fflush(Logfile);
#endif
    nErr = getInfoAndSettings();

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

    nErr = sendCommand(":RT0#", szResp, SERIAL_BUFFER_SIZE, 1);  // use macro command to set this

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::setSiderealTrackingOn] finished.  Result: %s\n", timestamp, szResp);
    fflush(Logfile);
#endif

    return nErr;

}

#pragma mark - tracking rates
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

    nErr = sendCommand(":ST0#", szResp, SERIAL_BUFFER_SIZE, 1);  // use macro command to set this

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::setTrackingOff] finished.  Result: %s\n", timestamp, szResp);
    fflush(Logfile);
#endif

    return nErr;

}

#pragma mark - tracking rates
int CiOptron::setTrackingRates(bool bTrackingOn, bool bIgnoreRates, double dTrackRaArcSecPerHr, double dTrackDecArcSecPerHr)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::setTrackingRates] called bTrackingOn: %s, bIgnoreRate: %s, bTrackRaArcSecPerHr: %f, bTrackDecArcSecPerHr %f\n", timestamp, bTrackingOn?"true":"false", bIgnoreRates?"true":"false", dTrackRaArcSecPerHr, dTrackDecArcSecPerHr);
    fflush(Logfile);
#endif

// :RRnnnnn# - set
// :GTR# - get with response of nnnnn#
// These commands select the tracking rate: select sidereal (“:RT0#”), lunar (“:RT1#”), solar (“:RT2#”), King (“:RT3#”), or custom (“:RT4#”).

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::setTrackingRates] done.  DID NOTHING\n", timestamp);
    fflush(Logfile);
#endif
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

    getInfoAndSettings();
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
    fprintf(Logfile, "[%s] [CiOptron::getTrackRates] called. bTrackingOn: %s, dTrackRaArcSecPerHr: %f, dTrackDecArcSecPerHr: %f\n", timestamp, bTrackingOn?"true":"false", dTrackRaArcSecPerSec, dTrackDecArcSecPerSec);
    fflush(Logfile);
#endif
    return nErr;
}


#pragma mark - Limis
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
    return nErr;
}



#pragma mark - Slew
int CiOptron::startSlewTo(double dRa, double dDec)
{
    int nErr = IOPTRON_OK;
    bool bGPSGood;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::startSlewTo] called Ra: %f and Dec: %f\n", timestamp, dRa, dDec);
    fflush(Logfile);
#endif

    nErr = isGPSGood(bGPSGood);

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
    char szResp[SERIAL_BUFFER_SIZE];
    int nPrecentRemaining;

    bComplete = false;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::isSlewToComplete] called\n", timestamp);
    fflush(Logfile);
#endif

    if(timer.GetElapsedSeconds()<2) {
        // we're checking for comletion too quickly, assume it's moving for now
        return nErr;
    }

    nErr = getInfoAndSettings();
    if(nErr)
        return nErr;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::isSlewToComplete] after getInfoAndSettings() status is : %i\n", timestamp, m_nStatus);
    fflush(Logfile);
#endif

    if (m_nStatus == SLEWING || m_nStatus == FLIPPING) {
        bComplete = false;
    } else {
        bComplete = true;
    }

    return nErr;
}

int CiOptron::parkMount()
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    int nParkResult;

    // set park position ?
    // RP : we probably need to set the mount park position from the settings dialog. I'll look into it.
    //      from the doc : "This command parks to the most recently defined parking position" ... so we need to define it
    //      if it's not already defined (and saved) in the mount
    // or goto ?    // RP : Goto and Park should be different.
    nErr = sendCommand(":MP1#", szResp, SERIAL_BUFFER_SIZE, 1);  // merely ask to park
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
    fprintf(Logfile, "[%s] [CiOptron::markParkPosition] setting  Park Az to : %d\n", timestamp, int(dAzArcSec));
    fflush(Logfile);
#endif
    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":SPA%09d#", int(dAzArcSec));
    nErr = sendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE, 1);
    if(nErr)
        return nErr;

    // set Alt park postion : “:SPHTTTTTTTT#”
    // convert dAlt to arcsec then to 0.01 arcsec
    dAltArcSec = (dAlt * 60 * 60) / 0.01;
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::markParkPosition] setting  Park Alt to : %d\n", timestamp, int(dAltArcSec));
    fflush(Logfile);
#endif
    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":SPA%09d#", int(dAltArcSec));
    nErr = sendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE, 1);
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

    nErr = sendCommand(":GPC#", szResp, SERIAL_BUFFER_SIZE, 1);  // merely ask to unpark
    if(nErr)
        return nErr;
    memset(szParkAz, 0, SERIAL_BUFFER_SIZE);
    memset(szParkAlt, 0, SERIAL_BUFFER_SIZE);
    memcpy(szParkAz, szResp, 8);
    memcpy(szParkAlt, szResp+8, 9);
    nAzArcSec = atoi(szParkAz);
    nAltArcSec = atoi(szParkAlt);
    dAz = (nAzArcSec*0.01)/ 60 /60 ;
    dAlt = (nAltArcSec*0.01)/ 60 /60 ;
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

    nErr = getInfoAndSettings();
    if(nErr)
        return nErr;

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
    nErr = sendCommand(":MP0#", szResp, SERIAL_BUFFER_SIZE, 1);  // merely ask to unpark
    if(nErr)
        return nErr;

    nErr = setTrackingRates(true, true, 0, 0); // sidereal
    if(nErr) {
#ifdef IOPTRON_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CiOptron::unPark] Error setting track rate to Sidereal\n", timestamp);
        fflush(Logfile);
#endif
        snprintf(m_szLogBuffer,IOPTRON_LOG_BUFFER_SIZE,"[CiOptron::unPark] Error setting track rate to Sidereal");
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
    fprintf(Logfile, "[%s] [CiOptron::getRefractionCorrEnabled] call result %s \n", timestamp, bEnabled ? "true":"false");
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
    nErr = sendCommand(":Q#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // stop tracking
    nErr = sendCommand(":ST0#", szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}


int CiOptron::getInfoAndSettings()
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szTmp[SERIAL_BUFFER_SIZE];

    nErr = sendCommand(":GLS#", szResp, SERIAL_BUFFER_SIZE);
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
    m_fLat = atof(szTmp);

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
    fprintf(Logfile, "[%s] [CiOptron::getInfoAndSettings]  lat is : %f, long is: %f, status is: %i, trackingRate is: %i, gpsStatus is: %i, timeSource is: %i\n", timestamp, m_fLat, m_fLong, m_nStatus, m_nTrackingRate, m_nGPSStatus, m_nTimeSource);
    fflush(Logfile);
#endif

    m_bParked = m_nStatus == PARKED?true:false;
    return nErr;

}


