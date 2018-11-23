#include "x2mount.h"

X2Mount::X2Mount(const char* pszDriverSelection,
				 const int& nInstanceIndex,
				 SerXInterface					* pSerX,
				 TheSkyXFacadeForDriversInterface	* pTheSkyX,
				 SleeperInterface					* pSleeper,
				 BasicIniUtilInterface			* pIniUtil,
				 LoggerInterface					* pLogger,
				 MutexInterface					* pIOMutex,
				 TickCountInterface				* pTickCount)
{

	m_nPrivateMulitInstanceIndex	= nInstanceIndex;
	m_pSerX							= pSerX;
	m_pTheSkyXForMounts				= pTheSkyX;
	m_pSleeper						= pSleeper;
	m_pIniUtil						= pIniUtil;
	m_pLogger						= pLogger;
	m_pIOMutex						= pIOMutex;
	m_pTickCount					= pTickCount;
	
#ifdef IOPTRON_X2_DEBUG
#if defined(SB_WIN_BUILD)
    m_sLogfilePath = getenv("HOMEDRIVE");
    m_sLogfilePath += getenv("HOMEPATH");
    m_sLogfilePath += "\\iOptronV3_X2_Logfile.txt";
#elif defined(SB_LINUX_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/iOptronV3_X2_Logfile.txt";
#elif defined(SB_MAC_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/iOptronV3_X2_Logfile.txt";
#endif
	LogFile = fopen(m_sLogfilePath.c_str(), "w");
    m_iOptronV3.setLogFile(LogFile);
#endif
	

	m_bSynced = false;
	m_bParked = false;
    m_bLinked = false;

    m_iOptronV3.setSerxPointer(m_pSerX);
    m_iOptronV3.setTSX(m_pTheSkyXForMounts);
    m_iOptronV3.setSleeper(m_pSleeper);
    m_iOptronV3.setLogger(m_pLogger);

    m_CurrentRateIndex = 0;

	// Read the current stored values for the settings
	if (m_pIniUtil)
	{
	}

}

X2Mount::~X2Mount()
{
	// Write the stored values

    if(m_bLinked)
        m_iOptronV3.Disconnect();
    
    if (m_pSerX)
		delete m_pSerX;
	if (m_pTheSkyXForMounts)
		delete m_pTheSkyXForMounts;
	if (m_pSleeper)
		delete m_pSleeper;
	if (m_pIniUtil)
		delete m_pIniUtil;
	if (m_pLogger)
		delete m_pLogger;
	if (m_pIOMutex)
		delete m_pIOMutex;
	if (m_pTickCount)
		delete m_pTickCount;
	
#ifdef IOPTRON_X2_DEBUG
	// Close LogFile
	if (LogFile) {
        fflush(LogFile);
		fclose(LogFile);
	}
#endif
	
}

int X2Mount::queryAbstraction(const char* pszName, void** ppVal)
{
	*ppVal = NULL;
	
	if (!strcmp(pszName, SyncMountInterface_Name))
	    *ppVal = dynamic_cast<SyncMountInterface*>(this);
	if (!strcmp(pszName, SlewToInterface_Name))
		*ppVal = dynamic_cast<SlewToInterface*>(this);
	else if (!strcmp(pszName, AsymmetricalEquatorialInterface_Name))
		*ppVal = dynamic_cast<AsymmetricalEquatorialInterface*>(this);
	else if (!strcmp(pszName, OpenLoopMoveInterface_Name))
		*ppVal = dynamic_cast<OpenLoopMoveInterface*>(this);
	else if (!strcmp(pszName, NeedsRefractionInterface_Name))
		*ppVal = dynamic_cast<NeedsRefractionInterface*>(this);
	else if (!strcmp(pszName, ModalSettingsDialogInterface_Name))
		*ppVal = dynamic_cast<ModalSettingsDialogInterface*>(this);
	else if (!strcmp(pszName, X2GUIEventInterface_Name))
		*ppVal = dynamic_cast<X2GUIEventInterface*>(this);
	else if (!strcmp(pszName, TrackingRatesInterface_Name))
		*ppVal = dynamic_cast<TrackingRatesInterface*>(this);
	else if (!strcmp(pszName, ParkInterface_Name))
		*ppVal = dynamic_cast<ParkInterface*>(this);
	else if (!strcmp(pszName, UnparkInterface_Name))
		*ppVal = dynamic_cast<UnparkInterface*>(this);
	else if (!strcmp(pszName, LoggerInterface_Name))
        *ppVal = GetLogger();
    else if (!strcmp(pszName, SerialPortParams2Interface_Name))
        *ppVal = dynamic_cast<SerialPortParams2Interface*>(this);
    else if (!strcmp(pszName, DriverSlewsToParkPositionInterface_Name))
        *ppVal = dynamic_cast<DriverSlewsToParkPositionInterface*>(this);
	return SB_OK;
}

#pragma mark - OpenLoopMoveInterface

int X2Mount::startOpenLoopMove(const MountDriverInterface::MoveDir& Dir, const int& nRateIndex)
{
    int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

	m_CurrentRateIndex = nRateIndex;
#ifdef IOPTRON_X2_DEBUG
	if (LogFile) {
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] startOpenLoopMove called Dir: %d , Rate: %d\n", timestamp, Dir, nRateIndex);
        fflush(LogFile);
	}
#endif

    nErr = m_iOptronV3.startOpenSlew(Dir, nRateIndex);
    if(nErr) {
#ifdef IOPTRON_X2_DEBUG
        if (LogFile) {
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] startOpenLoopMove ERROR %d\n", timestamp, nErr);
            fflush(LogFile);
        }
#endif
        m_pLogger->out("startOpenLoopMove ERROR");
        return ERR_CMDFAILED;
    }
    return SB_OK;
}

int X2Mount::endOpenLoopMove(void)
{
	int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

#ifdef IOPTRON_X2_DEBUG
	if (LogFile){
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(LogFile, "[%s] endOpenLoopMove Called\n", timestamp);
        fflush(LogFile);
	}
#endif

    nErr = m_iOptronV3.stopOpenLoopMove();
    if(nErr) {
#ifdef IOPTRON_X2_DEBUG
        if (LogFile) {
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] endOpenLoopMove ERROR %d\n", timestamp, nErr);
            fflush(LogFile);
        }
#endif
        m_pLogger->out("endOpenLoopMove ERROR");
        return ERR_CMDFAILED;
    }
    return nErr;
}

int X2Mount::rateCountOpenLoopMove(void) const
{
    X2Mount* pMe = (X2Mount*)this;

    X2MutexLocker ml(pMe->GetMutex());
	return pMe->m_iOptronV3.getNbSlewRates();
}

int X2Mount::rateNameFromIndexOpenLoopMove(const int& nZeroBasedIndex, char* pszOut, const int& nOutMaxSize)
{
    int nErr = SB_OK;
    nErr = m_iOptronV3.getRateName(nZeroBasedIndex, pszOut, nOutMaxSize);
    if(nErr) {
#ifdef IOPTRON_X2_DEBUG
        if (LogFile) {
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] rateNameFromIndexOpenLoopMove ERROR %d\n", timestamp, nErr);
            fflush(LogFile);
        }
#endif
        m_pLogger->out("rateNameFromIndexOpenLoopMove ERROR");
        return ERR_CMDFAILED;
    }
    return nErr;
}

int X2Mount::rateIndexOpenLoopMove(void)
{
	return m_CurrentRateIndex;
}

#pragma mark - UI binding

int X2Mount::execModalSettingsDialog(void)
{
	int nErr = SB_OK;
	X2ModalUIUtil uiutil(this, m_pTheSkyXForMounts);
	X2GUIInterface*					ui = uiutil.X2UI();
	X2GUIExchangeInterface*			dx = NULL;//Comes after ui is loaded
	bool bPressedOK = false;
	char szTmpBuf[SERIAL_BUFFER_SIZE];
    char szGPSStatus[SERIAL_BUFFER_SIZE];
    char szTimeSource[SERIAL_BUFFER_SIZE];
    char szUtcOffsetInMins[SERIAL_BUFFER_SIZE];
    char szUtcOffsetReadInMins[SERIAL_BUFFER_SIZE];
    char szSystemStatus[SERIAL_BUFFER_SIZE];
    char szTrackingRate[SERIAL_BUFFER_SIZE];
    bool bDaylight = true;  // most of us want daylight all the time.. unless your Ben Franklin
    bool bAtZero = false;
    bool bAtParked = false;
    double dParkAz, dParkAlt;

	if (NULL == ui) return ERR_POINTER;
	
	if ((nErr = ui->loadUserInterface("iOptronV3.ui", deviceType(), m_nPrivateMulitInstanceIndex)))
		return nErr;
	
	if (NULL == (dx = uiutil.X2DX())) {
		return ERR_POINTER;
	}

    memset(szGPSStatus,0,SERIAL_BUFFER_SIZE);
    memset(szTimeSource,0,SERIAL_BUFFER_SIZE);

    X2MutexLocker ml(GetMutex());

	// Set values in the userinterface
    if(m_bLinked) {
        dx->setEnabled("parkAz", true);
        dx->setEnabled("parkAlt", true);
        dx->setEnabled("pushButton_2", true);  // parked button
        dx->setEnabled("pushButton_3", true);  // goto zero button
        dx->setEnabled("pushButton_4", true);  // find zero button
        dx->setEnabled("pushButton_5", true); // goto flats position button
        dx->setEnabled("lineEdit_utc", true); // utc minutes offset
        dx->setEnabled("comboBox_dst", true); // daylight or not
        m_iOptronV3.getParkPosition(dParkAz, dParkAlt);
        dx->setPropertyDouble("parkAz", "value", dParkAz);
        dx->setPropertyDouble("parkAlt", "value", dParkAlt);

        m_iOptronV3.getGPSStatusString(szGPSStatus, SERIAL_BUFFER_SIZE);
        dx->setText("label_kv_1", szGPSStatus);
        m_iOptronV3.getTimeSource(szTimeSource, SERIAL_BUFFER_SIZE);
        dx->setText("label_kv_3", szTimeSource);
        m_iOptronV3.getUtcOffset(szUtcOffsetInMins);
        dx->setText("lineEdit_utc", szUtcOffsetInMins);
        nErr = m_iOptronV3.getDST(bDaylight);
        if (nErr) {
            dx->setCurrentIndex("comboBox_dst", 0);
        } else {
            dx->setCurrentIndex("comboBox_dst", bDaylight?1:2);
        }
        m_iOptronV3.getAtZeroPosition(bAtZero);  // must be after other checks since position status already set by those calls
        dx->setChecked("checkBox_z", bAtZero ? 1:0);
        m_iOptronV3.getAtParkedPositionPassive(bAtParked);
        dx->setChecked("checkBox_p", bAtParked ? 1:0);
        m_iOptronV3.getSystemStatusPassive(szSystemStatus, SERIAL_BUFFER_SIZE);
        dx->setText("label_actual_sys_stat", szSystemStatus);
        m_iOptronV3.getTrackingStatusPassive(szTrackingRate, SERIAL_BUFFER_SIZE);
        dx->setText("label_actual_track_rate", szTrackingRate);

        dx->setEnabled("pushButtonOK", true);  // cant really hit OK button
    }
    else {
        dx->setEnabled("parkAz", false);
        dx->setEnabled("parkAlt", false);
        dx->setEnabled("pushButton_2", false); // parked button
        dx->setEnabled("pushButton_3", false); // goto zero button
        dx->setEnabled("pushButton_4", false); // find zero button
        dx->setEnabled("pushButton_5", false); // goto flats position button
        dx->setEnabled("lineEdit_utc", false); // utc minutes offset
        dx->setEnabled("comboBox_dst", false); // daylight or not
        dx->setEnabled("pushButtonOK", false);  // cant really hit OK button
    }

    dx->setEnabled("checkBox_z", false);  // checkbox indicating if you are at zero position.. output only
    dx->setEnabled("checkBox_p", false);  // checkbox indicating if you are at park position.. output only

	//Display the user interface
    m_nCurrentDialog = MAIN;
	if ((nErr = ui->exec(bPressedOK)))
		return nErr;
	
	//Retreive values from the user interface
	if (bPressedOK) {
#ifdef IOPTRON_X2_DEBUG
        if (LogFile) {
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] execModalSettingsDialog pressedOK: do the needful and check if stuff changed\n", timestamp);
            fflush(LogFile);
        }
#endif
        dx->text("lineEdit_utc", szUtcOffsetReadInMins, SERIAL_BUFFER_SIZE);
        if (strcmp(szUtcOffsetReadInMins, szUtcOffsetInMins) != 0) {
#ifdef IOPTRON_X2_DEBUG
            if (LogFile) {
                ltime = time(NULL);
                timestamp = asctime(localtime(&ltime));
                timestamp[strlen(timestamp) - 1] = 0;
                fprintf(LogFile, "[%s] execModalSettingsDialog pressedOK: utc value changed value is %s.  first character %c\n", timestamp, szUtcOffsetReadInMins, szUtcOffsetReadInMins[0]);
                fflush(LogFile);
            }
#endif
            // changed utc offset
            if (atoi(szUtcOffsetReadInMins) > 780 || atoi(szUtcOffsetReadInMins) < -720) {
                // out of range,..
                snprintf(szTmpBuf, SERIAL_BUFFER_SIZE, "Valid values for UTC offset are -720 to +780");
                dx->messageBox("Error", szTmpBuf);
            } else if (szUtcOffsetReadInMins[0] != '+' && szUtcOffsetReadInMins[0] != '-') {
                snprintf(szTmpBuf, SERIAL_BUFFER_SIZE, "You must prefix your offset with a + or - sign");
                dx->messageBox("Error", szTmpBuf);
            } else {
                m_iOptronV3.setUtcOffset(szUtcOffsetReadInMins);
            }
        }
        if (dx->currentIndex("comboBox_dst") != 0) {
#ifdef IOPTRON_X2_DEBUG
            if (LogFile) {
                ltime = time(NULL);
                timestamp = asctime(localtime(&ltime));
                timestamp[strlen(timestamp) - 1] = 0;
                fprintf(LogFile, "[%s] execModalSettingsDialog pressedOK: dst value set.  Value read is %i\n", timestamp, dx->currentIndex("comboBox_dst"));
                fflush(LogFile);
            }
#endif
            if (dx->currentIndex("comboBox_dst") == 1 && !bDaylight) {
                // change to daylight
                m_iOptronV3.setDST(true);
            } else if (dx->currentIndex("comboBox_dst") == 2 && bDaylight) {
                // change to standard
                m_iOptronV3.setDST(false);
            }
        }
	}
	return nErr;
}

int X2Mount::doConfirm(bool &bPressedOK, const char *szText)
{
    int nErr = SB_OK;

    X2ModalUIUtil uiutil(this, m_pTheSkyXForMounts);
    X2GUIInterface*                    ui = uiutil.X2UI();
    X2GUIExchangeInterface*            dx = NULL;//Comes after ui is loaded

    bPressedOK = false;

#ifdef IOPTRON_X2_DEBUG
    if (LogFile) {
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] doConfirm called: X2GUIInterface is %s.  \n", timestamp, ui==NULL?"NULL":"*good*");
        fflush(LogFile);
    }
#endif

    if (NULL == ui)
        return ERR_POINTER;
    nErr = ui->loadUserInterface("iOptronV3Conf.ui", deviceType(), m_nPrivateMulitInstanceIndex);

    if (nErr) {
#ifdef IOPTRON_X2_DEBUG
        if (LogFile) {
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] doConfirm error when loading user interface: %i.  \n", timestamp, nErr);
            fflush(LogFile);
        }
#endif
        return nErr;
    }

    dx = uiutil.X2DX();
    if (NULL == dx)
        return ERR_POINTER;

    m_nCurrentDialog = CONFIRM;

    dx->setText("messageText", szText);
    //Display the user interface
    nErr = ui->exec(bPressedOK);
    if (nErr) {
#ifdef IOPTRON_X2_DEBUG
        if (LogFile) {
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] doConfirm dialog ended.  Error when exec-ing user interface: %i.  \n", timestamp, nErr);
            fflush(LogFile);
        }
#endif
        return nErr;
    }

#ifdef IOPTRON_X2_DEBUG
    if (LogFile) {
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] doConfirm dialog ended successfully.  value of bPressedOK: %s.  \n", timestamp, bPressedOK?"true":"false");
        fflush(LogFile);
    }
#endif

    m_nCurrentDialog = MAIN;

    return nErr;

}

void X2Mount::uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent)
{

    switch(m_nCurrentDialog) {
        case MAIN:
            doMainDialogEvents(uiex, pszEvent);
            break;
        case CONFIRM:
            doConfirmDialogEvents(uiex, pszEvent);
            break;
    }


}

int X2Mount::doMainDialogEvents(X2GUIExchangeInterface* uiex, const char* pszEvent)
{
    int nErr = SB_OK;
    double dParkAz, dParkAlt;
    char szTmpBuf[SERIAL_BUFFER_SIZE];
    bool bOk = false;

#ifdef IOPTRON_X2_DEBUG
    if (LogFile) {
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] X2Mount::uiEvent called.  Value of pszEvent: %s\n", timestamp, pszEvent);
        fflush(LogFile);
    }
#endif
    if(!m_bLinked)
        return ERR_NOLINK ;

#ifdef IOPTRON_X2_DEBUG
    if (LogFile) {
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] X2Mount::uiEvent called.  We're linked so doing something\n", timestamp);
        fflush(LogFile);
    }
#endif
    if (!strcmp(pszEvent, "on_pushButton_2_clicked")) { //Set the park position
#ifdef IOPTRON_X2_DEBUG
        if (LogFile) {
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] X2Mount::uiEvent on_pushButton_2_clicked (_2 means parked)\n", timestamp);
            fflush(LogFile);
        }
#endif
        doConfirm(bOk, "Are you sure you want to set the park position ?");
        if(bOk) {
            uiex->propertyDouble("parkAz", "value", dParkAz);
            uiex->propertyDouble("parkAlt", "value", dParkAlt);
            nErr = m_iOptronV3.setParkPosition(dParkAz, dParkAlt);
            if(nErr) {
                snprintf(szTmpBuf,SERIAL_BUFFER_SIZE, "Error setting park position : %d", nErr);
                uiex->messageBox("Error",szTmpBuf);
            }
        }
    } else if (!strcmp(pszEvent, "on_pushButton_3_clicked")) {
#ifdef IOPTRON_X2_DEBUG
        if (LogFile) {
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] X2Mount::uiEvent on_pushButton_3_clicked (_3 means goto zero position)\n", timestamp);
            fflush(LogFile);
        }
#endif
        doConfirm(bOk, "Are you sure you want to go to zero position ?");
        if(bOk) {
            nErr = m_iOptronV3.gotoZeroPosition();
            if (nErr) {
                snprintf(szTmpBuf, SERIAL_BUFFER_SIZE, "Error going to zero/home position : %d", nErr);
                uiex->messageBox("Error", szTmpBuf);
            }
        }

    } else if (!strcmp(pszEvent, "on_pushButton_4_clicked")) {
#ifdef IOPTRON_X2_DEBUG
        if (LogFile) {
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] X2Mount::uiEvent on_pushButton_4_clicked (_4 means find zero)\n", timestamp);
            fflush(LogFile);
        }
#endif
        doConfirm(bOk, "Are you sure you want to search for mechanical zero position ?");
        if(bOk) {
            nErr = m_iOptronV3.findZeroPosition();
            if (nErr) {
                snprintf(szTmpBuf, SERIAL_BUFFER_SIZE, "Error searching mechanical zero/home position : %d", nErr);
                uiex->messageBox("Error", szTmpBuf);
            }
        }
    } else if (!strcmp(pszEvent, "on_pushButton_5_clicked")) {
#ifdef IOPTRON_X2_DEBUG
        if (LogFile) {
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] X2Mount::uiEvent on_pushButton_5_clicked (_5 means goto flats position)\n", timestamp);
            fflush(LogFile);
        }
#endif
        doConfirm(bOk, "Are you sure you want to move the mount to point straight up and take flats ?");
        if(bOk) {
            nErr = m_iOptronV3.gotoFlatsPosition();
            if (nErr) {
                snprintf(szTmpBuf, SERIAL_BUFFER_SIZE, "Error going to straight-up position to take flats : %d", nErr);
                uiex->messageBox("Error", szTmpBuf);
            }
        }
    }
    return nErr;
}

int X2Mount::doConfirmDialogEvents(X2GUIExchangeInterface* uiex, const char* pszEvent)
{
    int nErr = SB_OK;

    return nErr;
}

#pragma mark - LinkInterface
int X2Mount::establishLink(void)
{
    int nErr;
    char szPort[DRIVER_MAX_STRING];

	X2MutexLocker ml(GetMutex());
	// get serial port device name
    portNameOnToCharPtr(szPort,DRIVER_MAX_STRING);

	nErr =  m_iOptronV3.Connect(szPort);
    if(nErr) {
        m_bLinked = false;
    }
    else {
        m_bLinked = true;
    }
    return nErr;
}

int X2Mount::terminateLink(void)
{
    int nErr = SB_OK;

	X2MutexLocker ml(GetMutex());

    nErr = m_iOptronV3.Disconnect();
    m_bLinked = false;

    return nErr;
}

bool X2Mount::isLinked(void) const
{

	return m_iOptronV3.isConnected();;
}

bool X2Mount::isEstablishLinkAbortable(void) const
{
    return false;
}

#pragma mark - AbstractDriverInfo

void	X2Mount::driverInfoDetailedInfo(BasicStringInterface& str) const
{
	str = "iOptron V3 X2 plugin by Eric Roubal and Rodolphe Pineau";
}

double	X2Mount::driverInfoVersion(void) const
{
	return DRIVER_VERSION;
}

void X2Mount::deviceInfoNameShort(BasicStringInterface& str) const
{
    if(m_bLinked) {
        X2Mount* pMe = (X2Mount*)this;
        X2MutexLocker ml(pMe->GetMutex());
        char cModel[SERIAL_BUFFER_SIZE];
        pMe->m_iOptronV3.getMountInfo(cModel, SERIAL_BUFFER_SIZE);
        str = cModel;
    }
    else
        str = "Not connected1";
}
void X2Mount::deviceInfoNameLong(BasicStringInterface& str) const
{
	str = "iOptron CEM120xx V3 Mount";
	
}
void X2Mount::deviceInfoDetailedDescription(BasicStringInterface& str) const
{
	str = "iOptron CEM120xx V3 Mount by Eric Roubal and Rodolphe Pineau";
	
}
void X2Mount::deviceInfoFirmwareVersion(BasicStringInterface& str)
{
    if(m_bLinked) {
        char cFirmware[SERIAL_BUFFER_SIZE];
        X2MutexLocker ml(GetMutex());
        m_iOptronV3.getFirmwareVersion(cFirmware, SERIAL_BUFFER_SIZE);
        str = cFirmware;
    }
    else
        str = "Not connected";
}
void X2Mount::deviceInfoModel(BasicStringInterface& str)
{
    if(m_bLinked) {
        char cModel[SERIAL_BUFFER_SIZE];
        X2MutexLocker ml(GetMutex());
        m_iOptronV3.getMountInfo(cModel, SERIAL_BUFFER_SIZE);
        str = cModel;
    }
    else
        str = "Not connected";
}

#pragma mark - Common Mount specifics
int X2Mount::raDec(double& ra, double& dec, const bool& bCached)
{
	int nErr = 0;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

	// Get the RA and DEC from the mount
	nErr = m_iOptronV3.getRaAndDec(ra, dec);
    if(nErr)
        nErr = ERR_CMDFAILED;

#ifdef IOPTRON_X2_DEBUG
    if (LogFile) {
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] raDec Called. Ra : %f , Dec : %f \n", timestamp, ra, dec);
        fprintf(LogFile, "[%s] nErr = %d \n", timestamp, nErr);
        fflush(LogFile);
    }
#endif

	return nErr;
}

int X2Mount::abort()
{
    int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

#ifdef IOPTRON_X2_DEBUG
	if (LogFile) {
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(LogFile, "[%s] abort Called\n", timestamp);
        fflush(LogFile);
	}
#endif

    nErr = m_iOptronV3.Abort();
    if(nErr)
        nErr = ERR_CMDFAILED;

#ifdef IOPTRON_X2_DEBUG
    if (LogFile) {
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] Abort nErr = %d \n", timestamp, nErr);
        fflush(LogFile);
    }
#endif

    return nErr;
}

int X2Mount::startSlewTo(const double& dRa, const double& dDec)
{
	int nErr = SB_OK;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

#ifdef IOPTRON_X2_DEBUG
	if (LogFile) {
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(LogFile, "[%s] startSlewTo Called %f %f\n", timestamp, dRa, dDec);
        fflush(LogFile);
	}
#endif
    nErr = m_iOptronV3.startSlewTo(dRa, dDec);
    if(nErr) {
#ifdef IOPTRON_X2_DEBUG
        if (LogFile) {
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] startSlewTo nErr = %d \n", timestamp, nErr);
            fflush(LogFile);
        }
#endif
        m_pLogger->out("startSlewTo ERROR");
        return ERR_CMDFAILED;
    }

    return nErr;
}

int X2Mount::isCompleteSlewTo(bool& bComplete) const
{
    int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2Mount* pMe = (X2Mount*)this;
    X2MutexLocker ml(pMe->GetMutex());

    nErr = pMe->m_iOptronV3.isSlewToComplete(bComplete);
    if(nErr)
        return ERR_CMDFAILED;

#ifdef IOPTRON_X2_DEBUG
    if (LogFile) {
        pMe->ltime = time(NULL);
        pMe->timestamp = asctime(localtime(&(pMe->ltime)));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] isCompleteSlewTo nErr = %d \n", timestamp, nErr);
        fflush(LogFile);
    }
#endif

	return nErr;
}

int X2Mount::endSlewTo(void)
{
#ifdef IOPTRON_X2_DEBUG
    if (LogFile) {
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] endSlewTo Called\n", timestamp);
        fflush(LogFile);
    }
#endif
    return SB_OK;
}


int X2Mount::syncMount(const double& ra, const double& dec)
{
	int nErr = SB_OK;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

#ifdef IOPTRON_X2_DEBUG
    if (LogFile) {
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] syncMount Called : %f\t%f\n", timestamp, ra, dec);
        fflush(LogFile);
    }
#endif

    nErr = m_iOptronV3.syncTo(ra, dec);
    if(nErr)
        nErr = ERR_CMDFAILED;

#ifdef IOPTRON_X2_DEBUG
    if (LogFile) {
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] syncMount nErr = %d \n", timestamp, nErr);
        fflush(LogFile);
    }
#endif

    return nErr;
}

bool X2Mount::isSynced(void)
{
    int nErr;

    if(!m_bLinked)
        return false;

    X2MutexLocker ml(GetMutex());

   nErr = m_iOptronV3.isGPSGood(m_bSynced);

#ifdef IOPTRON_X2_DEBUG
    if (LogFile) {
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] isSynced Called : m_bSynced = %s\n", timestamp, m_bSynced?"true":"false");
        fprintf(LogFile, "[%s] isSynced nErr = %d \n", timestamp, nErr);
        fflush(LogFile);
    }
#endif

    return m_bSynced;
}

#pragma mark - TrackingRatesInterface
int X2Mount::setTrackingRates(const bool& bTrackingOn, const bool& bIgnoreRates, const double& dRaRateArcSecPerSec, const double& dDecRateArcSecPerSec)
{
    int nErr = SB_OK;
    double dTrackRaArcSecPerHr;
    double dTrackDecArcSecPerHr;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    dTrackRaArcSecPerHr = dRaRateArcSecPerSec * 3600;
    dTrackDecArcSecPerHr = dDecRateArcSecPerSec * 3600;

    nErr = m_iOptronV3.setTrackingRates(bTrackingOn, bIgnoreRates, dTrackRaArcSecPerHr, dTrackDecArcSecPerHr);
#ifdef IOPTRON_X2_DEBUG
    if (LogFile) {
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] setTrackingRates Called. Tracking On: %s , Ra rate : %f , Dec rate: %f\n", timestamp, bTrackingOn?"true":"false", dRaRateArcSecPerSec, dDecRateArcSecPerSec);
        fflush(LogFile);
    }
#endif
    if(nErr)
        return ERR_CMDFAILED;

#ifdef IOPTRON_X2_DEBUG
    if (LogFile) {
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] setTrackingRates nErr = %d \n", timestamp, nErr);
        fflush(LogFile);
    }
#endif

    return nErr;
	
}

int X2Mount::trackingRates(bool& bTrackingOn, double& dRaRateArcSecPerSec, double& dDecRateArcSecPerSec)
{
    int nErr = SB_OK;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    nErr = m_iOptronV3.getTrackRates(bTrackingOn, dRaRateArcSecPerSec, dDecRateArcSecPerSec);
    if(nErr) {
#ifdef IOPTRON_X2_DEBUG
        if (LogFile) {
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] trackingRates  m_iOptronV3.getTrackRates nErr = %d \n", timestamp, nErr);
            fflush(LogFile);
        }
#endif
        return ERR_CMDFAILED;
    }

#ifdef IOPTRON_X2_DEBUG
    if (LogFile) {
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] trackingRates Called. Tracking On: %d , Ra rate : %f , Dec rate: %f\n", timestamp, bTrackingOn, dRaRateArcSecPerSec, dDecRateArcSecPerSec);
        fflush(LogFile);
    }
#endif

	return nErr;
}

int X2Mount::siderealTrackingOn()
{
    int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

#ifdef IOPTRON_X2_DEBUG
    if (LogFile) {
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] siderealTrackingOn Called \n", timestamp);
        fflush(LogFile);
    }
#endif

    nErr = m_iOptronV3.setSiderealTrackingOn();

    if(nErr)
        return ERR_CMDFAILED;

#ifdef IOPTRON_X2_DEBUG
    if (LogFile) {
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] siderealTrackingOn nErr = %d \n", timestamp, nErr);
        fflush(LogFile);
    }
#endif

    return nErr;
}

int X2Mount::trackingOff()
{
    int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

#ifdef IOPTRON_X2_DEBUG
    if (LogFile) {
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] trackingOff Called \n", timestamp);
        fflush(LogFile);
    }
#endif
    nErr = m_iOptronV3.setTrackingOff();
    if(nErr)
        nErr = ERR_CMDFAILED;

#ifdef IOPTRON_X2_DEBUG
    if (LogFile) {
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] trackingOff nErr = %d \n", timestamp, nErr);
        fflush(LogFile);
    }
#endif

    return nErr;
}

#pragma mark - NeedsRefractionInterface
bool X2Mount::needsRefactionAdjustments(void)
{
    bool bEnabled;
    int nErr;

    if(!m_bLinked)
        return false;

    X2MutexLocker ml(GetMutex());

    // check if iOptron V3 refraction adjustment is on.
    nErr = m_iOptronV3.getRefractionCorrEnabled(bEnabled);

#ifdef IOPTRON_X2_DEBUG
    if (LogFile) {
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] getRefractionCorrEnabled nErr = %d \n", timestamp, nErr);
        fprintf(LogFile, "[%s] getRefractionCorrEnabled bEnabled = %s\n", timestamp, bEnabled?"true":"false");
        fflush(LogFile);
    }
#endif

    return !bEnabled; // if enabled in iOptron V3, don't ask TSX to do it.
}

#pragma mark - Parking Interface
bool X2Mount::isParked(void)
{
    int nErr;
    bool bTrackingOn;
    bool bIsPArked;
    double dTrackRaArcSecPerHr, dTrackDecArcSecPerHr;

    if(!m_bLinked)
        return false;

    X2MutexLocker ml(GetMutex());

    nErr = m_iOptronV3.getAtPark(bIsPArked);
    if(nErr) {
#ifdef IOPTRON_X2_DEBUG
        if (LogFile) {
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] isParked m_iOptronV3.getAtPark nErr = %d \n", timestamp, nErr);
            fflush(LogFile);
        }
#endif
        return false;
    }
    if(!bIsPArked) // not parked
        return false;

    // get tracking state.
    nErr = m_iOptronV3.getTrackRates(bTrackingOn, dTrackRaArcSecPerHr, dTrackDecArcSecPerHr);
    if(nErr) {
#ifdef IOPTRON_X2_DEBUG
        if (LogFile) {
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] isParked m_iOptronV3.getTrackRates nErr = %d \n", timestamp, nErr);
            fflush(LogFile);
        }
#endif
        return false;
    }
    // if AtPark and tracking is off, then we're parked, if not then we're unparked.
    if(bIsPArked && !bTrackingOn)
        m_bParked = true;
    else
        m_bParked = false;
    return m_bParked;
}

int X2Mount::startPark(const double& dAz, const double& dAlt)
{
	int nErr = SB_OK;

    if(!m_bLinked)
        return ERR_NOLINK;
	
	X2MutexLocker ml(GetMutex());
#ifdef IOPTRON_X2_DEBUG
	if (LogFile) {
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] startPark Called.\n", timestamp);
        fflush(LogFile);
	}
#endif
    // Park mount to pre-define park position (in the mount).
    nErr = m_iOptronV3.parkMount();
    if(nErr) {
        nErr = ERR_CMDFAILED;
#ifdef IOPTRON_X2_DEBUG
        if (LogFile) {
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] startPark  m_iOptronV3.gotoPark nErr = %d \n", timestamp, nErr);
            fflush(LogFile);
        }
#endif
    }
	return nErr;
}


int X2Mount::isCompletePark(bool& bComplete) const
{
    int nErr = SB_OK;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2Mount* pMe = (X2Mount*)this;

    X2MutexLocker ml(pMe->GetMutex());

#ifdef IOPTRON_X2_DEBUG
	if (LogFile) {
		pMe->ltime = time(NULL);
		pMe->timestamp = asctime(localtime(&(pMe->ltime)));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(LogFile, "[%s] isCompletePark Called\n", timestamp);
        fflush(LogFile);
	}
#endif
    nErr = pMe->m_iOptronV3.getAtPark(bComplete);
    if(nErr)
        nErr = ERR_CMDFAILED;

#ifdef IOPTRON_X2_DEBUG
    if (LogFile) {
        pMe->ltime = time(NULL);
        pMe->timestamp = asctime(localtime(&(pMe->ltime)));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] isCompletePark  m_iOptronV3.getAtPark nErr = %d \n", timestamp, nErr);
        fflush(LogFile);
    }
#endif

	return nErr;
}

int X2Mount::endPark(void)
{
    return SB_OK;
}

int X2Mount::startUnpark(void)
{
    int nErr = SB_OK;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    nErr = m_iOptronV3.unPark();
    if(nErr) {
#ifdef IOPTRON_X2_DEBUG
        if (LogFile) {
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] startUnpark : m_iOptronV3.unPark() failled !\n", timestamp);
            fflush(LogFile);
        }
#endif
        nErr = ERR_CMDFAILED;
    }
    m_bParked = false;
    return nErr;
}

/*!Called to monitor the unpark process.
 \param bComplete Set to true if the unpark is complete, otherwise set to false.
*/
int X2Mount::isCompleteUnpark(bool& bComplete) const
{
    int nErr;
    bool bIsParked;
    bool bTrackingOn;
    double dTrackRaArcSecPerHr, dTrackDecArcSecPerHr;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2Mount* pMe = (X2Mount*)this;

    X2MutexLocker ml(pMe->GetMutex());

    bComplete = false;

    nErr = pMe->m_iOptronV3.getAtPark(bIsParked);
    if(nErr) {
#ifdef IOPTRON_X2_DEBUG
        if (LogFile) {
            pMe->ltime = time(NULL);
            pMe->timestamp = asctime(localtime(&(pMe->ltime)));
            pMe->timestamp[strlen(pMe->timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] isCompleteUnpark  m_iOptronV3.getAtPark nErr = %d \n", pMe->timestamp, nErr);
            fflush(LogFile);
        }
#endif
        nErr = ERR_CMDFAILED;
    }
    if(!bIsParked) { // no longer parked.
        bComplete = true;
        pMe->m_bParked = false;
        return nErr;
    }

    // if we're still at the park position
    // get tracking state. If tracking is off, then we're parked, if not then we're unparked.
    nErr = pMe->m_iOptronV3.getTrackRates(bTrackingOn, dTrackRaArcSecPerHr, dTrackDecArcSecPerHr);
    if(nErr)
        nErr = ERR_CMDFAILED;
#ifdef IOPTRON_X2_DEBUG
    if (LogFile) {
        pMe->ltime = time(NULL);
        pMe->timestamp = asctime(localtime(&(pMe->ltime)));
        pMe->timestamp[strlen(pMe->timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] isCompleteUnpark  m_iOptronV3.getTrackRates nErr = %d \n", pMe->timestamp, nErr);
        fflush(LogFile);
    }
#endif

    if(bTrackingOn) {
        bComplete = true;
        pMe->m_bParked = false;
    }
    else {
        bComplete = false;
        pMe->m_bParked = true;
    }
	return SB_OK;
}

/*!Called once the unpark is complete.
 This is called once for every corresponding startUnpark() allowing software implementations of unpark.
 */
int		X2Mount::endUnpark(void)
{
	return SB_OK;
}

#pragma mark - AsymmetricalEquatorialInterface

bool X2Mount::knowsBeyondThePole()
{
    return true;
}

int X2Mount::beyondThePole(bool& bYes) {
	// bYes = m_iOptronV3.GetIsBeyondThePole();
	return SB_OK;
}


double X2Mount::flipHourAngle() {
#ifdef IOPTRON_X2_DEBUG
	if (LogFile) {
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(LogFile, "[%s] flipHourAngle called\n", timestamp);
        fflush(LogFile);
	}
#endif

	return 0.0;
}


int X2Mount::gemLimits(double& dHoursEast, double& dHoursWest)
{
    int nErr = SB_OK;
#ifdef IOPTRON_X2_DEBUG
    if (LogFile) {
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] gemLimits called.\n", timestamp);
        fflush(LogFile);
    }
#endif
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    nErr = m_iOptronV3.getLimits(dHoursEast, dHoursWest);

#ifdef IOPTRON_X2_DEBUG
    if (LogFile) {
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] gemLimits m_iOptronV3.getLimits nErr = %d\n", timestamp, nErr);
        fprintf(LogFile, "[%s] gemLimits dHoursEast = %f\n", timestamp, dHoursEast);
        fprintf(LogFile, "[%s] gemLimits dHoursWest = %f\n", timestamp, dHoursWest);
        fflush(LogFile);
    }
#endif
    // temp debugging.
	dHoursEast = 0.0;
	dHoursWest = 0.0;
	return SB_OK;
}


#pragma mark - SerialPortParams2Interface

void X2Mount::portName(BasicStringInterface& str) const
{
    char szPortName[DRIVER_MAX_STRING];

    portNameOnToCharPtr(szPortName, DRIVER_MAX_STRING);

    str = szPortName;

}

void X2Mount::setPortName(const char* pszPort)
{
    if (m_pIniUtil)
        m_pIniUtil->writeString(PARENT_KEY, CHILD_KEY_PORT_NAME, pszPort);

}


void X2Mount::portNameOnToCharPtr(char* pszPort, const unsigned int& nMaxSize) const
{
    if (NULL == pszPort)
        return;

    snprintf(pszPort, nMaxSize,DEF_PORT_NAME);

    if (m_pIniUtil)
        m_pIniUtil->readString(PARENT_KEY, CHILD_KEY_PORT_NAME, pszPort, pszPort, nMaxSize);

}




