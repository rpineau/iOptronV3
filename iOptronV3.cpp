#include "iOptronV3.h"

// Constructor for IOPTRON
CiOptron::CiOptron()
{

	m_bIsConnected = false;

    m_bParked = false;  // probably not good to assume we're parked.  Power could have shut down or we're at zero position or we're parked

#ifdef IOPTRON_DEBUG
#if defined(SB_WIN_BUILD)
    m_sLogfilePath = getenv("HOMEDRIVE");
    m_sLogfilePath += getenv("HOMEPATH");
    m_sLogfilePath += "\\iOptronV3Log.txt";
#elif defined(SB_LINUX_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/iOptronV3Log.txt";
#elif defined(SB_MAC_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/iOptronV3Log.txt";
#endif
	Logfile = fopen(m_sLogfilePath.c_str(), "w");
#endif

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] IOPTRON New Constructor Called\n", timestamp);
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
    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":SR%d#", nRate+1);
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
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CiOptron::readResponse] readFile error\n", timestamp);
            fflush(Logfile);
#endif
            return nErr;
        }

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

    nErr = sendCommand(":MountInfo#", szResp, SERIAL_BUFFER_SIZE, 4);
    if(nErr)
        return nErr;

    nModel = atoi(szResp);

    switch(nModel) {
        case CubeIIEQmode:
            strncpy(model, "Cube II EQ mode", strMaxLen);
            break;
        case SmartEQProPlus:
            strncpy(model, "SmartEQ Pro+", strMaxLen);
            break;
        case CEM25:
            strncpy(model, "CEM25(/P)", strMaxLen);
            break;
        case CEM25_EC:
            strncpy(model, "CEM25-EC", strMaxLen);
            break;
        case iEQ30Pro:
            strncpy(model, "iEQ30 Pro", strMaxLen);
            break;
        case iEQ45ProEQmode:
            strncpy(model, "iEQ45 Pro EQ mode", strMaxLen);
            break;
        case  CEM60:
            strncpy(model, "CEM60", strMaxLen);
            break;
        case CEM60_EC:
            strncpy(model, "CEM60-EC", strMaxLen);
            break;
        case CEM120:
            strncpy(model, "CEM120", strMaxLen);
            break;
        case CEM120_EC:
            strncpy(model, "CEM120-EC", strMaxLen);
            break;
        case CEM120_EC2:
            strncpy(model, "CEM120-EC2", strMaxLen);
            break;
        case CubeIIAAmode:
            strncpy(model, "Cube II AA mode", strMaxLen);
            break;
        case AZMountPro:
            strncpy(model, "AZ Mount Pro", strMaxLen);
            break;
        case iEQ45ProAAmode:
            strncpy(model, "iEQ45 Pro AA mode", strMaxLen);
            break;
        default :
            strncpy(model, "Unknown mount", strMaxLen);
            break;
    }
    return nErr;
}


int CiOptron::getFirmwareVersion(char *pszVersion, unsigned int nStrMaxLen)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    std::string sFirmwares;

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
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    nErr = getInfoAndSettings();
    if(nErr)
        return nErr;

    dRa = m_fLat;
    dDec = m_fLong;

    return nErr;
}

// where Eric left off
#pragma mark - Sync and Cal
int CiOptron::syncTo(double dRa, double dDec)
{
    int nErr = IOPTRON_OK;
    bool bGPSGood;

#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::syncTo] Ra : %f\n", timestamp, dRa);
    fprintf(Logfile, "[%s] [CiOptron::syncTo] Dec : %f\n", timestamp, dDec);
    fflush(Logfile);
#endif

     return nErr;
}


int CiOptron::isGPSGood(bool &bGPSGood)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = getInfoAndSettings();

    bGPSGood = m_nGPSStatus == GPS_RECEIVING_VALID_DATA;
    return nErr;
}


#pragma mark - tracking rates
int CiOptron::setTrackingRates(bool bTrackingOn, bool bIgnoreRates, double dTrackRaArcSecPerHr, double dTrackDecArcSecPerHr)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    return nErr;
}

int CiOptron::getTrackRates(bool &bTrackingOn, double &dTrackRaArcSecPerHr, double &dTrackDecArcSecPerHr)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    return nErr;
}


#pragma mark - Limis
int CiOptron::getLimits(double &dHoursEast, double &dHoursWest)
{
    int nErr = IOPTRON_OK;

    return nErr;
}



#pragma mark - Slew
int CiOptron::startSlewTo(double dRa, double dDec)
{
    int nErr = IOPTRON_OK;
    bool bGPSGood;

    nErr = isGPSGood(bGPSGood);
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

    if(timer.GetElapsedSeconds()<2) {
        // we're checking for comletion to quickly, assume it's moving for now
        return nErr;
    }

    nErr = sendCommand("!GGgr;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::isSlewToComplete] szResp : %s\n", timestamp, szResp);
    fflush(Logfile);
#endif

    // remove the %
    szResp[strlen(szResp) -1 ] = 0;
    
#if defined IOPTRON_DEBUG && IOPTRON_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CiOptron::isSlewToComplete] szResp : %s\n", timestamp, szResp);
    fflush(Logfile);
#endif

    nPrecentRemaining = atoi(szResp);
    if(nPrecentRemaining == 0)
        bComplete = true;

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
    //      if it's not already define (and saved) in the mount
    // or goto ?    // RP : Goto and Park should be different.
    // goto park
    nErr = sendCommand(":MP1#", szResp, SERIAL_BUFFER_SIZE, 1);  // merely ask to park
    if(nErr)
        return nErr;

    nParkResult = atoi(szResp);
    if(nParkResult != 1)
        return ERR_CMDFAILED;

    return nErr; // todo: szResp says '1' or '0' for accepted or failed
}

int CiOptron::markParkPosition(double dAz, double dAlt)
{
    int nErr = IOPTRON_OK;

    // set az park position :  “:SPATTTTTTTTT#”
    // nErr = sendCommand(":SPA%09d#", szResp, SERIAL_BUFFER_SIZE, 1);
    if(nErr)
        return nErr;
    // set Alt park postion : “:SPHTTTTTTTT#”
    // nErr = sendCommand(":SPH%09d#", szResp, SERIAL_BUFFER_SIZE, 1);

    return nErr;

}

int CiOptron::getAtPark(bool &bParked)
{
    int nErr = IOPTRON_OK;

    bParked = false;

    nErr = getInfoAndSettings();
    if(nErr)
        return nErr;

    bParked = m_bParked;

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
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    bEnabled = false;
    nErr = sendCommand("!PGre;", szResp, SERIAL_BUFFER_SIZE);
    if(strncmp(szResp,"Yes",SERIAL_BUFFER_SIZE) == 0) {
        bEnabled = true;
    }
    return nErr;
}

int CiOptron::setRefractionCorrEnabled(bool bEnable)
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;
    if(bEnable) {
        snprintf(szCmd, SERIAL_BUFFER_SIZE, "!PSreYes;");
    }
    else  {
        snprintf(szCmd, SERIAL_BUFFER_SIZE, "!PSreNo;");
    }
    nErr = sendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}


int CiOptron::Abort()
{
    int nErr = IOPTRON_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = sendCommand("!XXxx;", szResp, SERIAL_BUFFER_SIZE);
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
    memset(szTmp,0, SERIAL_BUFFER_SIZE);
    memcpy(szTmp, szResp, 9);
    m_fLong = atof(szTmp);

    memset(szTmp,0, SERIAL_BUFFER_SIZE);
    memcpy(szTmp+9, szResp, 9);
    m_fLat = atof(szTmp);

    memset(szTmp,0, SERIAL_BUFFER_SIZE);
    memcpy(szTmp+18, szResp, 1);
    m_nStatus = atoi(szTmp);

    memset(szTmp,0, SERIAL_BUFFER_SIZE);
    memcpy(szTmp+17, szResp, 1);
    m_nGPSStatus = atoi(szTmp);

    memset(szTmp,0, SERIAL_BUFFER_SIZE);
    memcpy(szTmp+21, szResp, 1);
    m_nTimeSource = atoi(szTmp);

    m_bParked = m_nStatus == PARKED?true:false;
    return nErr;

}

int CiOptron::parseFields(const char *pszIn, std::vector<std::string> &svFields, char cSeparator)
{
    int nErr = IOPTRON_OK;
    std::string sSegment;
    std::stringstream ssTmp(pszIn);

    svFields.clear();
    // split the string into vector elements
    while(std::getline(ssTmp, sSegment, cSeparator))
    {
        svFields.push_back(sSegment);
    }

    if(svFields.size()==0) {
        nErr = ERR_PARSE;
    }
    return nErr;
}

