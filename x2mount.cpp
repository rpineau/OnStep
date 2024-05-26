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

	m_bSynced = false;
	m_bParked = false;
	m_bLinked = false;
	m_bSyncOnConnect = false;
	m_bStopTrackingOnDisconnect = false;

	m_nParkPosIndex = 0;

	m_OnStep.setSerxPointer(m_pSerX);
	m_OnStep.setTSX(m_pTheSkyXForMounts);

	m_CurrentRateIndex = 0;

	// Read the current stored values for the settings
	if (m_pIniUtil)
	{
		m_bSyncOnConnect = (m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_SYNC_TIME, 0) == 0 ? false : true);
		m_bStopTrackingOnDisconnect = (m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_STOP_TRK, 1) == 0 ? false : true);
		m_nSlewRateIndex = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_SLEW_RATE, 6);
		m_OnStep.setGoToSlewRate(m_nSlewRateIndex);
		m_nParkPosIndex = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_PARK_POS, 0);
	}

	m_OnStep.setSyncLocationDataConnect(m_bSyncOnConnect);
	m_OnStep.setStopTrackingOnDisconnect(m_bStopTrackingOnDisconnect);
}

X2Mount::~X2Mount()
{
	// Write the stored values

	if(m_bLinked)
		m_OnStep.Disconnect();

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
	nErr = m_OnStep.startOpenLoopMove(Dir, nRateIndex);
	if(nErr) {
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

	nErr = m_OnStep.stopOpenLoopMove();
	if(nErr) {
		return ERR_CMDFAILED;
	}
	return nErr;
}

int X2Mount::rateCountOpenLoopMove(void) const
{
	X2Mount* pMe = (X2Mount*)this;

	X2MutexLocker ml(pMe->GetMutex());
	return pMe->m_OnStep.getNbSlewRates();
}

int X2Mount::rateNameFromIndexOpenLoopMove(const int& nZeroBasedIndex, char* pszOut, const int& nOutMaxSize)
{
	int nErr = SB_OK;
	std::string sTmp;

	X2MutexLocker ml(GetMutex());

	nErr = m_OnStep.getRateName(nZeroBasedIndex, sTmp);
	if(nErr) {
		return ERR_CMDFAILED;
	}
	strncpy(pszOut, sTmp.c_str(), nOutMaxSize);
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
	std::string sTmp;
	std::string sTime;
	std::string sDate;
	std::string sLongitude;
	std::string sLatitude;
	std::string sTimeZone;

	if (NULL == ui) return ERR_POINTER;

	if ((nErr = ui->loadUserInterface("OnStep.ui", deviceType(), m_nPrivateMulitInstanceIndex)))
		return nErr;

	if (NULL == (dx = uiutil.X2DX())) {
		return ERR_POINTER;
	}

	X2MutexLocker ml(GetMutex());

	m_bHoming = false;
	// Set values in the userinterface
	if(m_bLinked) {
		dx->setEnabled("pushButton",true);
		dx->setEnabled("pushButton_2",true);
		dx->setEnabled("pushButton_3",true);
		dx->setEnabled("pushButton_4",true);
		dx->setEnabled("comboBox", true);

		nErr = m_OnStep.getLocalTime(sTime);
		nErr |= m_OnStep.getLocalDate(sDate);
		if(!nErr) {
			sTmp =sDate + " - " + sTime;
			dx->setText("time_date", sTmp.c_str());
		}
		m_OnStep.getSiteData(sLongitude, sLatitude, sTimeZone);
		sTimeZone = std::string("GMT ") + sTimeZone;

		dx->setText("longitude", sLongitude.c_str());
		dx->setText("latitude", sLatitude.c_str());
		dx->setText("timezone", sTimeZone.c_str());
	}
	else {
		dx->setEnabled("pushButton",false);
		dx->setEnabled("pushButton_2",false);
		dx->setEnabled("pushButton_3",false);
		dx->setEnabled("pushButton_4",false);
		dx->setEnabled("comboBox", false);
		dx->setText("time_date", "");
		dx->setText("siteName", "");
		dx->setText("longitude", "");
		dx->setText("latitude", "");
		dx->setText("timezone", "");
	}

	dx->setEnabled("comboBox_2", true);
	m_nSlewRateIndex = m_OnStep.getGoToSlewRate();
	dx->setCurrentIndex("comboBox_2", m_nSlewRateIndex);
	dx->setCurrentIndex("comboBox", m_nParkPosIndex);
	dx->setChecked("checkBox", (m_bSyncOnConnect?1:0));
	dx->setChecked("checkBox_2", (m_bStopTrackingOnDisconnect?1:0));
	dx->setEnabled("checkBox_3", false); // not supported yet.
	dx->setText("homingProgress","");
	dx->setText("parkingProgress","");

	//Display the user interface
	if ((nErr = ui->exec(bPressedOK)))
		return nErr;

	//Retreive values from the user interface
	if (bPressedOK) {
		m_bSyncOnConnect = (dx->isChecked("checkBox")==1?true:false);
		m_bStopTrackingOnDisconnect = (dx->isChecked("checkBox_2")==1?true:false);
		nErr |= m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_SYNC_TIME, (m_bSyncOnConnect?1:0));
		nErr |= m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_STOP_TRK, (m_bStopTrackingOnDisconnect?1:0));
		m_nParkPosIndex = dx->currentIndex("comboBox");
		nErr |= m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_PARK_POS, m_nParkPosIndex);
		m_nSlewRateIndex =  dx->currentIndex("comboBox_2");

		m_OnStep.setGoToSlewRate(m_nSlewRateIndex);
		m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_SLEW_RATE, m_nSlewRateIndex);
		m_OnStep.setStopTrackingOnDisconnect(m_bStopTrackingOnDisconnect);
	}
	return nErr;
}

void X2Mount::uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent)
{
	int nErr = SB_OK;
	std::string sLongitude;
	std::string sLatitude;
	std::string sTimeZone;
	std::string sTime;
	std::string sDate;
	std::string sTmp;
	std::stringstream sErrorMessage;
	std::stringstream ssTmp;
	double dAlt, dAz;
	int nParkPosIndex;
	bool bComplete;
	char c;

	if(!m_bLinked)
		return ;

	if (!strcmp(pszEvent, "on_timer")) {
		nErr = m_OnStep.getLocalTime(sTime);
		nErr |= m_OnStep.getLocalDate(sDate);
		if(!nErr) {
			sTmp =sDate + " - " + sTime;
			uiex->setText("time_date", sTmp.c_str());
		}
		// Homing
		if(m_bHoming) {
			nErr = m_OnStep.isHomingDone(bComplete);
			if(nErr) {
				sErrorMessage << "Error while homing : " << nErr;
				uiex->messageBox("OnStep Homing", sErrorMessage.str().c_str());
				setHomingButton(uiex, false);
				m_bHoming = false;
				return;
			}
			if(!bComplete) {
				getProgress(c);
				ssTmp <<  "Homing " << c;
				uiex->setText("homingProgress",ssTmp.str().c_str());
				return;
			}
			// enable buttons
			setHomingButton(uiex, true);
			m_bHoming = false;
			uiex->setText("homingProgress","Homing done");
		}
		// Parking
		if(m_bSettingPark) {
			nErr = m_OnStep.isSlewToComplete(bComplete);
			if(nErr) {
				sErrorMessage << "Error while parking : " << nErr;
				uiex->messageBox("OnStep Parking", sErrorMessage.str().c_str());
				setParkingButton(uiex, true);
				m_bSettingPark = false;
				return;
			}
			if(!bComplete) {
				getProgress(c);
				ssTmp <<  "Slewing to new park position " << c;
				uiex->setText("parkingProgress",ssTmp.str().c_str());
				return;
			}
			// set the current postion as the park postion
			m_OnStep.setCurentPosAsPark();
			setParkingButton(uiex, true);
			m_bSettingPark = false;
			uiex->setText("parkingProgress","New parking position set");
		}
	}
	// Sync
	if (!strcmp(pszEvent, "on_pushButton_clicked")) {
		m_OnStep.syncDate();
		m_OnStep.syncTime();
		nErr = m_OnStep.getLocalTime(sTime);
		nErr |= m_OnStep.getLocalDate(sDate);
		if(!nErr) {
			sTmp =sDate + " - " + sTime;
			uiex->setText("time_date", sTmp.c_str());
		}

		m_OnStep.setSiteData( m_pTheSkyXForMounts->longitude(),
							 m_pTheSkyXForMounts->latitude(),
							 m_pTheSkyXForMounts->timeZone());
		m_OnStep.getSiteData(sLongitude, sLatitude, sTimeZone);
		sTimeZone = std::string("GMT ") + sTimeZone;

		uiex->setText("longitude", sLongitude.c_str());
		uiex->setText("latitude", sLatitude.c_str());
		uiex->setText("timezone", sTimeZone.c_str());
	}
	// Home
	if (!strcmp(pszEvent, "on_pushButton_3_clicked")) {
		if( m_bHoming) { // Abort
			// enable buttons
			setHomingButton(uiex, true);
		} else {								// home
			// disable buttons
			setHomingButton(uiex, false);
			m_bHoming = true;
			m_OnStep.homeMount();
			getProgress(c, true);
			ssTmp <<  "Homing " << c;
			uiex->setText("homingProgress",ssTmp.str().c_str());
		}
	}
	// Goto new  Park
	if (!strcmp(pszEvent, "on_pushButton_2_clicked")) {
		if( m_bSettingPark) { // Abort
			// enable buttons
			setParkingButton(uiex, true);
		} else {								// park
			// disable buttons
			setParkingButton(uiex, false);
			nParkPosIndex = uiex->currentIndex("comboBox");
			switch(nParkPosIndex) {
				case 0:
					dAlt = 0.0;
					dAz = 270.0;
					break;
				case 1:
					dAlt = 0.0;
					dAz = 180.0;
					break;
				case 2:
					dAlt = 0.0;
					dAz = 90.0;
					break;
				default:
					dAlt = 0.0;
					dAz = 270.0;
					break;
			}
			nErr = m_OnStep.gotoParkPos(dAlt, dAz);
			m_bSettingPark = true;
			getProgress(c, true);
			ssTmp <<  "Slewing to new park position " << c;
			uiex->setText("parkingProgress",ssTmp.str().c_str());
		}
	}
	// Set park to current
	if (!strcmp(pszEvent, "on_pushButton_4_clicked")) {
		m_OnStep.setCurentPosAsPark();
		setParkingButton(uiex, true);
		m_bSettingPark = false;
		uiex->setText("parkingProgress","New parking position set");
	}
	return;
}

void X2Mount::setHomingButton(X2GUIExchangeInterface* uiex, bool bEnable)
{
	uiex->setEnabled("pushButtonOK",bEnable);
	uiex->setEnabled("pushButtonCancel", bEnable);
	uiex->setEnabled("pushButton", bEnable);
	uiex->setEnabled("pushButton_2", bEnable);
	uiex->setEnabled("pushButton_4", bEnable);
	if(bEnable)
		uiex->setText("pushButton_3", "Home mount");
	else
		uiex->setText("pushButton_3", "Abort");
}

void X2Mount::setParkingButton(X2GUIExchangeInterface* uiex, bool bEnable)
{
	uiex->setEnabled("pushButtonOK",bEnable);
	uiex->setEnabled("pushButtonCancel", bEnable);
	uiex->setEnabled("pushButton", bEnable);
	uiex->setEnabled("pushButton_3", bEnable);
	uiex->setEnabled("pushButton_4", bEnable);
	uiex->setEnabled("combobox", bEnable);
	if(bEnable)
		uiex->setText("pushButton_2", "Goto park position and set in mount");
	else
		uiex->setText("pushButton_2", "Abort");
}

void X2Mount::getProgress(char &c, bool bReset)
{
	if(bReset) {
		m_nProgress_index = 0;
	}
	m_nProgress_index = m_nProgress_index % 4;
	c = m_svProgressState.at(m_nProgress_index);
	m_nProgress_index++;
}

#pragma mark - LinkInterface
int X2Mount::establishLink(void)
{
	int nErr;
	std::string sPortName;

	X2MutexLocker ml(GetMutex());

	// get serial port device name
	getPortName(sPortName);

	nErr =  m_OnStep.Connect(sPortName);
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

	nErr = m_OnStep.Disconnect();
	m_bLinked = false;

	return nErr;
}

bool X2Mount::isLinked(void) const
{
	return m_OnStep.isConnected();
}

bool X2Mount::isEstablishLinkAbortable(void) const
{
	return false;
}

#pragma mark - AbstractDriverInfo

void	X2Mount::driverInfoDetailedInfo(BasicStringInterface& str) const
{
#ifdef PLUGIN_DEBUG
	str = "OnStep X2 plugin by Rodolphe Pineau [DEBUG]";
#else
	str = "OnStep X2 plugin by Rodolphe Pineau";
#endif
}

double	X2Mount::driverInfoVersion(void) const
{
	return PLUGIN_VERSION;
}

void X2Mount::deviceInfoNameShort(BasicStringInterface& str) const
{
	if(m_bLinked) {
		str = "OnStep";
	}
	else
		str = "Not connected1";
}
void X2Mount::deviceInfoNameLong(BasicStringInterface& str) const
{
	str = "OnStep Mount";

}
void X2Mount::deviceInfoDetailedDescription(BasicStringInterface& str) const
{
	str = "OnStep mount";

}
void X2Mount::deviceInfoFirmwareVersion(BasicStringInterface& str)
{
	if(m_bLinked) {
		std::string sFirmware;
		X2MutexLocker ml(GetMutex());
		m_OnStep.getFirmwareVersion(sFirmware);
		str = sFirmware.c_str();
	}
	else
		str = "Not connected";
}
void X2Mount::deviceInfoModel(BasicStringInterface& str)
{
	if(m_bLinked) {
		str = "OnStep";
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
	nErr = m_OnStep.getRaAndDec(ra, dec);
	if(nErr)
		nErr = ERR_CMDFAILED;

	return nErr;
}

int X2Mount::abort()
{
	int nErr = SB_OK;
	if(!m_bLinked)
		return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

	nErr = m_OnStep.Abort();
	if(nErr) {
		nErr = ERR_CMDFAILED;
	}
	return nErr;
}

int X2Mount::startSlewTo(const double& dRa, const double& dDec)
{
	int nErr = SB_OK;

	if(!m_bLinked)
		return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());
	nErr = m_OnStep.startSlewTo(dRa, dDec);
	if(nErr) {
		return nErr;
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
	nErr = pMe->m_OnStep.isSlewToComplete(bComplete);
	return nErr;
}

int X2Mount::endSlewTo(void)
{
	return SB_OK;
}


int X2Mount::syncMount(const double& ra, const double& dec)
{
	int nErr = SB_OK;

	if(!m_bLinked)
		return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());
	nErr = m_OnStep.syncTo(ra, dec);
	if(nErr) {
		nErr = ERR_CMDFAILED;
	}
	return nErr;
}

bool X2Mount::isSynced(void)
{
	int nErr;

	if(!m_bLinked)
		return false;

	X2MutexLocker ml(GetMutex());

	nErr = m_OnStep.isAligned(m_bSynced);

	return m_bSynced;
}

#pragma mark - TrackingRatesInterface
int X2Mount::setTrackingRates(const bool& bSiderialTrackingOn, const bool& bIgnoreRates, const double& dRaRateArcSecPerSec, const double& dDecRateArcSecPerSec)
{
	int nErr = SB_OK;
	if(!m_bLinked)
		return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

	nErr = m_OnStep.setTrackingRates(bSiderialTrackingOn, bIgnoreRates, dRaRateArcSecPerSec, dDecRateArcSecPerSec);

	return nErr;
}

int X2Mount::trackingRates(bool& bSiderialTrackingOn, double& dRaRateArcSecPerSec, double& dDecRateArcSecPerSec)
{
	int nErr = SB_OK;

	if(!m_bLinked)
		return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

	nErr = m_OnStep.getTrackRates(bSiderialTrackingOn, dRaRateArcSecPerSec, dDecRateArcSecPerSec);
	if(nErr) {
		return ERR_CMDFAILED;
	}

	return nErr;
}

int X2Mount::siderealTrackingOn()
{
	int nErr = SB_OK;
	if(!m_bLinked)
		return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());
	nErr = setTrackingRates( true, true, 0.0, 0.0);
	return nErr;
}

int X2Mount::trackingOff()
{
	int nErr = SB_OK;
	if(!m_bLinked)
		return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());
	nErr = setTrackingRates( false, true, 0.0, 0.0);
	return nErr;
}

#pragma mark - NeedsRefractionInterface
bool X2Mount::needsRefactionAdjustments(void)
{

	if(!m_bLinked)
		return false;

	return true;
}

#pragma mark - Parking Interface
bool X2Mount::isParked(void)
{
	int nErr;

	if(!m_bLinked)
		return false;

	X2MutexLocker ml(GetMutex());
	nErr = m_OnStep.getAtPark(m_bParked);
	if(nErr) {
		return false;
	}
	return m_bParked;
}

int X2Mount::startPark(const double& dAz, const double& dAlt)
{
	int nErr = SB_OK;

	if(!m_bLinked)
		return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

	nErr = m_OnStep.gotoPark();
	if (nErr) {
		nErr = ERR_CMDFAILED;
	}
	return nErr;
}


int X2Mount::isCompletePark(bool& bComplete) const
{
	int nErr = SB_OK;

	if(!m_bLinked)
		return ERR_NOLINK;

	X2Mount* pMe = (X2Mount*)this;
	X2MutexLocker ml(pMe ->GetMutex());

	nErr =  pMe->m_OnStep.isParkingComplete(bComplete);
	if(nErr)
		return nErr;

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
	nErr = m_OnStep.unPark();
	if(nErr) {
		nErr = ERR_CMDFAILED;
	}

	return nErr;
}

/*!Called to monitor the unpark process.
 \param bComplete Set to true if the unpark is complete, otherwise set to false.
 */
int X2Mount::isCompleteUnpark(bool& bComplete) const
{
	int nErr = SB_OK;

	if(!m_bLinked)
		return ERR_NOLINK;

	X2Mount* pMe = (X2Mount*)this;

	X2MutexLocker ml(pMe ->GetMutex());
	bComplete = false;

	nErr = pMe->m_OnStep.isUnparkDone(bComplete);

	if(bComplete) { // no longer parked.
		pMe->m_bParked = false;
	}
	else
		pMe->m_bParked = true;

	return nErr;
}

/*!Called once the unpark is complete.
 This is called once for every corresponding startUnpark() allowing software implementations of unpark.
 */
int X2Mount::endUnpark(void)
{
	return SB_OK;
}

#pragma mark - AsymmetricalEquatorialInterface

bool X2Mount::knowsBeyondThePole()
{
	X2MutexLocker ml(GetMutex());
	return true;
}

int X2Mount::beyondThePole(bool& bYes) {
	int nErr = SB_OK;
	X2MutexLocker ml(GetMutex());

	if(!m_bLinked)
		return ERR_NOLINK;

	// “beyond the pole” =  “telescope west of the pier”,
	nErr = m_OnStep.IsBeyondThePole(bYes);
	return nErr;
}


double X2Mount::flipHourAngle()
{
	int nErr = SB_OK;
	double dHourAngle = 0.0;

	if(!m_bLinked)
		return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());
	nErr = m_OnStep.getflipHourAngle(dHourAngle);

	return -dHourAngle;
}

MountTypeInterface::Type X2Mount::mountType()
{
	return  MountTypeInterface::Symmetrical_Equatorial;
}

int X2Mount::gemLimits(double& dHoursEast, double& dHoursWest)
{
	int nErr = SB_OK;

	if(!m_bLinked)
		return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());
	nErr = m_OnStep.getLimits(dHoursEast, dHoursWest);

	return SB_OK;
}

#pragma mark - SerialPortParams2Interface

void X2Mount::portName(BasicStringInterface& str) const
{
	std::string sPortName;
	getPortName(sPortName);

	str = sPortName.c_str();
}

void X2Mount::setPortName(const char* pszPort)
{
	if (m_pIniUtil)
		m_pIniUtil->writeString(PARENT_KEY, CHILD_KEY_PORT_NAME, pszPort);

}

void X2Mount::getPortName(std::string &sPortName) const
{
	sPortName.assign(DEF_PORT_NAME);

	if (m_pIniUtil) {
		char port[255];
		m_pIniUtil->readString(PARENT_KEY, CHILD_KEY_PORT_NAME, sPortName.c_str(), port, 255);
		sPortName.assign(port);
	}

}
