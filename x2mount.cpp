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
	m_bFindHomeInitiated = false;
	m_bSyncOnConnect = false;
	m_bStopTrackingOnDisconnect = false;
	m_nDebugLevel = 0;

	m_nParkPosIndex = 0;

	m_bZWOHeightLimitsEnabled = false;
	m_nZWOHeightLimitUpper = 90;
	m_nZWOHeightLimitLower = 0;
	m_nZWOMeridianTrack = 0;
	m_nZWOMeridianSlew = 0;

	std::string sSelection(pszDriverSelection);
	if(sSelection.find("ZWO") != std::string::npos) {
		m_pMount = new ZWOMount();
		m_bIsZWOMount = true;
	} else {
		m_pMount = new OnStep();
		m_bIsZWOMount = false;
	}

	m_pMount->setSerxPointer(m_pSerX);
	m_pMount->setTSX(m_pTheSkyXForMounts);
	m_CurrentRateIndex = 0;

	if (m_pIniUtil)
	{
		m_bSyncOnConnect = (m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_SYNC_TIME, 0) == 0 ? false : true);
		m_bStopTrackingOnDisconnect = (m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_STOP_TRK, 1) == 0 ? false : true);
		m_nPortSpeed = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_PORT_SPEED, 9600);
		m_pMount->setPortSpeed(m_nPortSpeed);
		m_nSlewRateIndex = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_SLEW_RATE, 6);
		m_pMount->setGoToSlewRate(m_nSlewRateIndex);
		m_GuideRateIndex = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_GUIDE_RATE, 2);
		m_dZWOGuideRate = m_pIniUtil->readDouble(PARENT_KEY, CHILD_KEY_ZWO_GUIDE_RATE, 0.5);
		m_pMount->setZWOGuideRate(m_dZWOGuideRate);
		m_nParkPosIndex = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_PARK_POS, 0);
		m_nDebugLevel = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_DEBUG_LVL, 0);
		m_pMount->setDebugLevel(m_nDebugLevel);
		m_pMount->log(std::string("pszDriverSelection = '") + sSelection + "'");

		m_bZWOHeightLimitsEnabled = (m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_ZWO_HEIGHT_ENABLED, 0) != 0);
		m_nZWOHeightLimitUpper = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_ZWO_HEIGHT_UPPER, 90);
		m_nZWOHeightLimitLower = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_ZWO_HEIGHT_LOWER, 0);
		m_nZWOMeridianTrack = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_ZWO_MERIDIAN_TRACK, 0);
		m_nZWOMeridianSlew  = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_ZWO_MERIDIAN_SLEW, 0);
	}

	m_pMount->setSyncLocationDataConnect(m_bSyncOnConnect);
	m_pMount->setStopTrackingOnDisconnect(m_bStopTrackingOnDisconnect);
}

X2Mount::~X2Mount()
{
	// Write the stored values

	if(m_bLinked)
		m_pMount->Disconnect();

	if (m_pMount)
		delete m_pMount;
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
	m_pMount->log("[queryAbstraction] queried for: " + std::string(pszName));
	*ppVal = NULL;

	if (!strcmp(pszName, SyncMountInterface_Name))
		*ppVal = dynamic_cast<SyncMountInterface*>(this);
	if (!strcmp(pszName, SlewToInterface_Name))
		*ppVal = dynamic_cast<SlewToInterface*>(this);
	else if (!strcmp(pszName, AsymmetricalEquatorialInterface_Name))
		*ppVal = dynamic_cast<AsymmetricalEquatorialInterface*>(this);
	else if (!strcmp(pszName, OpenLoopMoveInterface_Name))
		*ppVal = dynamic_cast<OpenLoopMoveInterface*>(this);
	else if (!strcmp(pszName, PulseGuideInterface2_Name))
		*ppVal = dynamic_cast<PulseGuideInterface2*>(this);
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
	else if (!strcmp(pszName, "DirectGuideInterface"))
		*ppVal = dynamic_cast<DirectGuideInterface*>(this);
	else if (!strcmp(pszName, "FindHomeInterface") && m_bIsZWOMount)
		*ppVal = static_cast<FindHomeInterface*>(this);
	else if (!strcmp(pszName, "MotorStatusInterface") && m_bIsZWOMount)
		*ppVal = static_cast<MotorStatusInterface*>(this);

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
	nErr = m_pMount->startOpenLoopMove(Dir, nRateIndex);
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

	nErr = m_pMount->stopOpenLoopMove();
	if(nErr) {
		return ERR_CMDFAILED;
	}
	return nErr;
}

int X2Mount::rateCountOpenLoopMove(void) const
{
	X2Mount* pMe = (X2Mount*)this;

	X2MutexLocker ml(pMe->GetMutex());
	return pMe->m_pMount->getNbSlewRates();
}

int X2Mount::rateNameFromIndexOpenLoopMove(const int& nZeroBasedIndex, char* pszOut, const int& nOutMaxSize)
{
	int nErr = SB_OK;
	std::string sTmp;

	X2MutexLocker ml(GetMutex());

	nErr = m_pMount->getRateName(nZeroBasedIndex, sTmp);
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

int X2Mount::useOpenLoopMoveInterface(int& nGuideRateIndex, OpenLoopMoveInterface** pOLSI)
{
	nGuideRateIndex = m_GuideRateIndex;
	return queryAbstraction(OpenLoopMoveInterface_Name, (void**)pOLSI);
}

#pragma mark - DirectGuideInterface

int X2Mount::directGuideMoveTelescope(const double& dRA, const double& dDec)
{
	int nErr = SB_OK;
	if(!m_bLinked)
		return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

	// Convert dRA and dDec (arcseconds) into pulse duration (milliseconds)
	// Base guide rate in arcsec/sec
	double siderealArcsecPerSec = 15.04106858;
	double guideArcsecPerSec = m_dZWOGuideRate * siderealArcsecPerSec;
	if (guideArcsecPerSec == 0.0)
		return ERR_CMDFAILED;

	int raMs = std::abs(dRA) / guideArcsecPerSec * 1000.0;
	int decMs = std::abs(dDec) / guideArcsecPerSec * 1000.0;

	// Send RA pulse
	if (raMs > 0) {
		std::string dir = (dRA > 0) ? "e" : "w";
		nErr = m_pMount->startPulseGuide(dir, raMs);
		if (nErr) return nErr;
	}

	// Send DEC pulse
	if (decMs > 0) {
		std::string dir = (dDec > 0) ? "n" : "s";
		nErr = m_pMount->startPulseGuide(dir, decMs);
		if (nErr) return nErr;
	}

	return SB_OK;
}

int X2Mount::directGuideAbort()
{
	if(!m_bLinked)
		return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());
	return m_pMount->Abort();
}

bool X2Mount::directGuideAsynchronous()
{
	return true;
}

int X2Mount::setDirectGuideAsynchronous(bool /* bAsync */)
{
	return SB_OK;
}

#pragma mark - FindHomeInterface

int X2Mount::startFindHome()
{
	int nErr = SB_OK;
	m_pMount->log("[startFindHome] Called. m_bLinked=" + std::string(m_bLinked ? "Yes" : "No"));
	if(!m_bLinked)
		return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());
	m_bFindHomeInitiated = true;
	m_pMount->log("[startFindHome] m_bFindHomeInitiated set to true");
	nErr = m_pMount->homeMount();
	m_pMount->log("[startFindHome] homeMount returned nErr=" + std::to_string(nErr));
	if(nErr) {
		m_bFindHomeInitiated = false;
		return ERR_CMDFAILED;
	}
	return nErr;
}

int X2Mount::isCompleteFindHome(bool& bComplete) const
{
	int nErr = SB_OK;
	if(!m_bLinked)
		return ERR_NOLINK;

	X2Mount* pMe = (X2Mount*)this;
	X2MutexLocker ml(pMe->GetMutex());

	// TSX calls this both from findHomeLoop (after startFindHome) and from background
	// polls / slot_SC_TELE_CONNECT (without a prior startFindHome) to determine whether
	// the mount is currently at the home position.  In both cases we should return the
	// real homed state.  When no active homing is in progress we return the cached value
	// so we don't send a serial command on every poll.
	if(!pMe->m_bFindHomeInitiated) {
		bComplete = pMe->m_pMount->hasCachedHomedState();
		pMe->m_pMount->log("[isCompleteFindHome] no active homing — bComplete=" + std::string(bComplete ? "Yes" : "No"));
		return SB_OK;
	}

	nErr = pMe->m_pMount->isHomingDone(bComplete);
	pMe->m_pMount->log("[isCompleteFindHome] bComplete=" + std::string(bComplete ? "Yes" : "No") + " nErr=" + std::to_string(nErr));
	return nErr;
}

int X2Mount::endFindHome()
{
	m_pMount->log("[endFindHome] Called. m_bFindHomeInitiated was=" + std::string(m_bFindHomeInitiated ? "Yes" : "No"));
	m_bFindHomeInitiated = false;
	return SB_OK;
}

#pragma mark - MotorStatusInterface

int X2Mount::motorStatus(unsigned short& u1, unsigned short& u2)
{
	// poll1HomeAndJoysticking (0x634c40) checks bit 0x1000 in BOTH u1 AND u2.
	// If either bit is clear → mount state = 7 (NOT_HOMED) → ERR_MOUNTNOTHOMED on slew.
	// All known call sites pass valid stack addresses for x2, so writing u2 is safe here.
	// (The "do not write u2" restriction only applies to motorStatus2 in updateHomeStatus.)
	u1 = 0;
	u2 = 0;
	if(m_bLinked && m_pMount->hasCachedHomedState()) {
		u1 = 0x1000;
		u2 = 0x1000;
	}
	m_pMount->log("[motorStatus] u1=" + std::to_string(u1));
	return SB_OK;
}

int X2Mount::motorStatus2(unsigned short& u1, unsigned short& u2)
{
	// TSX's updateHomeStatus() calls this and treats u1 != 0 as "Mount is homed." (green).
	// findHomeLoop also calls this every 100ms: u1/u2 == 0xfa2 means motor position error.
	// Must NOT issue serial commands — called at ~10Hz. Use cached state only.
	//
	// Do NOT write to u2 — TSX passes only one output arg (x1=sp+63). At the call site,
	// x2 holds the motorStatus2 thunk address, not a second output pointer. Writing u2
	// would corrupt the thunk's machine code (sub x0,x0,#0xa0 → sub x0,x0,#0x20) via COW.
	u1 = 0;
	if(m_bLinked && m_pMount->hasCachedHomedState())
		u1 = 1;
	if(m_nDebugLevel >= 3)
		m_pMount->log("[motorStatus2] u1=" + std::to_string(u1));
	return SB_OK;
}

#pragma mark - UI binding

int X2Mount::execModalSettingsDialog(void)
{
	int nErr = SB_OK;
	X2ModalUIUtil uiutil(this, m_pTheSkyXForMounts);
	X2GUIInterface*					ui = uiutil.X2UI();
	X2GUIExchangeInterface*			dx = NULL;//Comes after ui is loaded
	bool bPressedOK = false;
	int nPortSpeedIndex;
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
		dx->setEnabled("pushButton_syncNow",true);
		dx->setEnabled("pushButton_gotoPark",true);
		dx->setEnabled("pushButton_homeMount",true);
		dx->setEnabled("pushButton_setCurrentPark",true);
		dx->setEnabled("comboBox_parkPosition", true);

		nErr = m_pMount->getLocalTime(sTime);
		nErr |= m_pMount->getLocalDate(sDate);
		if(!nErr) {
			sTmp = sDate + " - " + sTime.substr(0,8);
			dx->setText("time_date", sTmp.c_str());
		}
		m_pMount->getSiteData(sLongitude, sLatitude, sTimeZone);
		sTimeZone = std::string("GMT ") + sTimeZone;

		dx->setText("longitude", sLongitude.c_str());
		dx->setText("latitude", sLatitude.c_str());
		dx->setText("timezone", sTimeZone.c_str());
	}
	else {
		dx->setEnabled("pushButton_syncNow",false);
		dx->setEnabled("pushButton_gotoPark",false);
		dx->setEnabled("pushButton_homeMount",false);
		dx->setEnabled("pushButton_setCurrentPark",false);
		dx->setEnabled("comboBox_parkPosition", false);
		dx->setText("time_date", "");
		dx->setText("siteName", "");
		dx->setText("longitude", "");
		dx->setText("latitude", "");
		dx->setText("timezone", "");
	}
	
	// what is our port speed
	nPortSpeedIndex = 0;
	for(int &i : m_svPortSpeed){
		if (i == m_nPortSpeed)
			break;
		nPortSpeedIndex ++;
	}
	dx->setCurrentIndex("comboBox_connectionSpeed", nPortSpeedIndex);

	dx->setEnabled("comboBox_slewRate", true);
	m_nSlewRateIndex = m_pMount->getGoToSlewRate();
	dx->setCurrentIndex("comboBox_slewRate", m_nSlewRateIndex);
	dx->setCurrentIndex("comboBox_parkPosition", m_nParkPosIndex);

	dx->setEnabled("comboBox_guideRate", true);
	dx->setCurrentIndex("comboBox_guideRate", m_GuideRateIndex);

	if (m_bIsZWOMount) {
		// Swap logo to ZWO branding
		dx->setPropertyString("label_logo", "X2_PhotoFileName", "ZWO.png");
		// Populate mount status info
		if(m_bLinked) {
			std::string sFirmware, sProduct;
			bool bHomed = false;
			m_pMount->getFirmwareVersion(sFirmware);
			m_pMount->getDeviceName(sProduct);
			m_pMount->isAligned(bHomed);
			dx->setText("label_zwoFirmwareValue", sFirmware.c_str());
			dx->setText("label_zwoProductValue",  sProduct.c_str());
			dx->setText("label_zwoHomedValue", bHomed ? "Yes" : "No — use Startup > Find Home");
		} else {
			dx->setText("label_zwoFirmwareValue", "Not connected");
			dx->setText("label_zwoProductValue",  "Not connected");
			dx->setText("label_zwoHomedValue",    "Not connected");
		}
		// ZWO guide rate
		dx->setEnabled("spinBox_zwoGuideRate", true);
		dx->setPropertyDouble("spinBox_zwoGuideRate", "value", m_dZWOGuideRate);
		// ZWO always syncs on connect — time/location section not needed
		dx->setPropertyInt("groupBox_timeLocation", "visible", 0);
		// Connection speed is fixed at 9600 for ZWO — hide the selector
		dx->setPropertyInt("label_connectionSpeed", "visible", 0);
		dx->setPropertyInt("comboBox_connectionSpeed", "visible", 0);
		dx->setPropertyInt("label_connectionSpeedHint", "visible", 0);
		// Alignment star sync not applicable to ZWO protocol
		dx->setPropertyInt("checkBox_addAlignStars", "visible", 0);
		// Home and park are handled by TheSkyX via FindHomeInterface/ParkInterface
		dx->setPropertyInt("pushButton_homeMount", "visible", 0);
		dx->setPropertyInt("homingProgress", "visible", 0);
		dx->setPropertyInt("groupBox_parking", "visible", 0);

		// Read live values from mount if connected
		if(m_bLinked) {
			bool bIgnored;
			m_pMount->getGuideRate(m_dZWOGuideRate);
			m_pMount->getHeightLimits(bIgnored, m_nZWOHeightLimitUpper, m_nZWOHeightLimitLower);
			m_pMount->getMeridianConfig(m_nZWOMeridianTrack, m_nZWOMeridianSlew);
		}
		dx->setChecked("checkBox_zwoHeightLimitsEnabled", m_bZWOHeightLimitsEnabled ? 1 : 0);
		dx->setPropertyInt("spinBox_zwoHeightLimitUpper", "value", m_nZWOHeightLimitUpper);
		dx->setPropertyInt("spinBox_zwoHeightLimitLower", "value", m_nZWOHeightLimitLower);
		dx->setPropertyInt("spinBox_zwoMeridianTrack", "value", m_nZWOMeridianTrack);
		dx->setPropertyInt("spinBox_zwoMeridianSlew", "value", m_nZWOMeridianSlew);
		// Set Park button only useful when connected (mount must be at the desired park position)
		dx->setEnabled("pushButton_zwoSetPark", m_bLinked);
	} else {
		// Hide ZWO-specific controls for standard OnStep mounts
		dx->setPropertyInt("label_zwoGuideRate", "visible", 0);
		dx->setPropertyInt("spinBox_zwoGuideRate", "visible", 0);
		// "Set to current position" uses ZWO-specific :Sp01# — hide for standard OnStep
		dx->setPropertyInt("pushButton_setCurrentPark", "visible", 0);
		dx->setEnabled("checkBox_addAlignStars", false); // not supported yet
		// ZWO-only groups
		dx->setPropertyInt("groupBox_zwoInfo", "visible", 0);
		dx->setPropertyInt("groupBox_zwoAdvanced", "visible", 0);
	}

	dx->setCurrentIndex("comboBox_debugLevel", m_nDebugLevel);

	dx->setChecked("checkBox_syncOnConnect", (m_bSyncOnConnect?1:0));
	dx->setChecked("checkBox_stopTrackingOnDisconnect", (m_bStopTrackingOnDisconnect?1:0));
	dx->setText("homingProgress","");
	dx->setText("parkingProgress","");

	//Display the user interface
	if ((nErr = ui->exec(bPressedOK)))
		return nErr;

	//Retreive values from the user interface
	if (bPressedOK) {
		m_bSyncOnConnect = (dx->isChecked("checkBox_syncOnConnect")==1?true:false);
		nErr |= m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_SYNC_TIME, (m_bSyncOnConnect?1:0));

		m_bStopTrackingOnDisconnect = (dx->isChecked("checkBox_stopTrackingOnDisconnect")==1?true:false);
		m_pMount->setStopTrackingOnDisconnect(m_bStopTrackingOnDisconnect);
		nErr |= m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_STOP_TRK, (m_bStopTrackingOnDisconnect?1:0));

		nPortSpeedIndex = dx->currentIndex("comboBox_connectionSpeed");
		m_nPortSpeed = m_svPortSpeed.at(nPortSpeedIndex);
		m_pMount->Reconnect(m_nPortSpeed);
		nErr |= m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_PORT_SPEED, m_nPortSpeed);

		m_nParkPosIndex = dx->currentIndex("comboBox_parkPosition");
		nErr |= m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_PARK_POS, m_nParkPosIndex);

		m_nSlewRateIndex =  dx->currentIndex("comboBox_slewRate");
		m_pMount->setGoToSlewRate(m_nSlewRateIndex);
		m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_SLEW_RATE, m_nSlewRateIndex);

		m_GuideRateIndex =  dx->currentIndex("comboBox_guideRate");
		m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_GUIDE_RATE, m_GuideRateIndex);

		if (m_bIsZWOMount) {
			dx->propertyDouble("spinBox_zwoGuideRate", "value", m_dZWOGuideRate);
			nErr |= m_pIniUtil->writeDouble(PARENT_KEY, CHILD_KEY_ZWO_GUIDE_RATE, m_dZWOGuideRate);

			m_bZWOHeightLimitsEnabled = (dx->isChecked("checkBox_zwoHeightLimitsEnabled") == 1);
			dx->propertyInt("spinBox_zwoHeightLimitUpper", "value", m_nZWOHeightLimitUpper);
			dx->propertyInt("spinBox_zwoHeightLimitLower", "value", m_nZWOHeightLimitLower);
			dx->propertyInt("spinBox_zwoMeridianTrack", "value", m_nZWOMeridianTrack);
			dx->propertyInt("spinBox_zwoMeridianSlew", "value", m_nZWOMeridianSlew);

			if(m_bLinked) {
				m_pMount->setGuideRate(m_dZWOGuideRate);
				m_pMount->setHeightLimits(m_bZWOHeightLimitsEnabled, m_nZWOHeightLimitUpper, m_nZWOHeightLimitLower);
				m_pMount->setMeridianConfig(m_nZWOMeridianTrack, m_nZWOMeridianSlew);
			}

			nErr |= m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_ZWO_HEIGHT_ENABLED, m_bZWOHeightLimitsEnabled ? 1 : 0);
			nErr |= m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_ZWO_HEIGHT_UPPER, m_nZWOHeightLimitUpper);
			nErr |= m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_ZWO_HEIGHT_LOWER, m_nZWOHeightLimitLower);
			nErr |= m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_ZWO_MERIDIAN_TRACK, m_nZWOMeridianTrack);
			nErr |= m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_ZWO_MERIDIAN_SLEW, m_nZWOMeridianSlew);
		}

		m_nDebugLevel = dx->currentIndex("comboBox_debugLevel");
		m_pMount->setDebugLevel(m_nDebugLevel);
		nErr |= m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_DEBUG_LVL, m_nDebugLevel);
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
		nErr = m_pMount->getLocalTime(sTime);
		nErr |= m_pMount->getLocalDate(sDate);
		if(!nErr) {
			sTmp = sDate + " - " + sTime.substr(0,8);
			uiex->setText("time_date", sTmp.c_str());
		}
		// Homing
		if(m_bHoming) {
			nErr = m_pMount->isHomingDone(bComplete);
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
			nErr = m_pMount->isSlewToComplete(bComplete);
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
			m_pMount->setCurentPosAsPark();
			setParkingButton(uiex, true);
			m_bSettingPark = false;
			uiex->setText("parkingProgress","New parking position set");
		}
	}
	// Sync
	if (!strcmp(pszEvent, "on_pushButton_syncNow_clicked")) {
		m_pMount->syncDate();
		m_pMount->syncTime();
		nErr = m_pMount->getLocalTime(sTime);
		nErr |= m_pMount->getLocalDate(sDate);
		if(!nErr) {
			sTmp =sDate + " - " + sTime.substr(0,8);
			uiex->setText("time_date", sTmp.c_str());
		}

		m_pMount->setSiteData( m_pTheSkyXForMounts->longitude(),
							 m_pTheSkyXForMounts->latitude(),
							 m_pTheSkyXForMounts->timeZone());
		m_pMount->getSiteData(sLongitude, sLatitude, sTimeZone);
		sTimeZone = std::string("GMT ") + sTimeZone;

		uiex->setText("longitude", sLongitude.c_str());
		uiex->setText("latitude", sLatitude.c_str());
		uiex->setText("timezone", sTimeZone.c_str());
	}
	// Home
	if (!strcmp(pszEvent, "on_pushButton_homeMount_clicked")) {
		if( m_bHoming) { // Abort
			// enable buttons
			setHomingButton(uiex, true);
		} else {								// home
			// disable buttons
			setHomingButton(uiex, false);
			m_bHoming = true;
			m_pMount->homeMount();
			getProgress(c, true);
			ssTmp <<  "Homing " << c;
			uiex->setText("homingProgress",ssTmp.str().c_str());
		}
	}
	// Goto new  Park
	if (!strcmp(pszEvent, "on_pushButton_gotoPark_clicked")) {
		if( m_bSettingPark) { // Abort
			// enable buttons
			setParkingButton(uiex, true);
		} else {								// park
			// disable buttons
			setParkingButton(uiex, false);
			nParkPosIndex = uiex->currentIndex("comboBox_parkPosition");
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
			nErr = m_pMount->gotoParkPos(dAlt, dAz);
			m_bSettingPark = true;
			getProgress(c, true);
			ssTmp <<  "Slewing to new park position " << c;
			uiex->setText("parkingProgress",ssTmp.str().c_str());
		}
	}
	// Set park to current
	if (!strcmp(pszEvent, "on_pushButton_setCurrentPark_clicked")) {
		m_pMount->setCurentPosAsPark();
		setParkingButton(uiex, true);
		m_bSettingPark = false;
		uiex->setText("parkingProgress","New parking position set");
	}
	// ZWO: set current position as park position in mount
	if (!strcmp(pszEvent, "on_pushButton_zwoSetPark_clicked")) {
		nErr = m_pMount->setCurentPosAsPark();
		if(nErr)
			uiex->messageBox("ZWO Park", "Error setting park position.");
		else
			uiex->messageBox("ZWO Park", "Park position set to current mount position.");
	}
	return;
}

void X2Mount::setHomingButton(X2GUIExchangeInterface* uiex, bool bEnable)
{
	uiex->setEnabled("pushButtonOK",bEnable);
	uiex->setEnabled("pushButtonCancel", bEnable);
	uiex->setEnabled("pushButton_syncNow", bEnable);
	uiex->setEnabled("pushButton_gotoPark", bEnable);
	uiex->setEnabled("pushButton_setCurrentPark", bEnable);
	if(bEnable)
		uiex->setText("pushButton_homeMount", "Home mount");
	else
		uiex->setText("pushButton_homeMount", "Abort");
}

void X2Mount::setParkingButton(X2GUIExchangeInterface* uiex, bool bEnable)
{
	uiex->setEnabled("pushButtonOK",bEnable);
	uiex->setEnabled("pushButtonCancel", bEnable);
	uiex->setEnabled("pushButton_syncNow", bEnable);
	uiex->setEnabled("pushButton_homeMount", bEnable);
	uiex->setEnabled("pushButton_setCurrentPark", bEnable);
	uiex->setEnabled("comboBox_parkPosition", bEnable);
	if(bEnable)
		uiex->setText("pushButton_gotoPark", "Goto park position and set in mount");
	else
		uiex->setText("pushButton_gotoPark", "Abort");
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

	m_pMount->log("[establishLink] Called");
	X2MutexLocker ml(GetMutex());

	// get serial port device name
	getPortName(sPortName);

	nErr =  m_pMount->Connect(sPortName);
	if(nErr) {
		m_bLinked = false;
	}
	else {
		m_bLinked = true;
		// Sync guide rate from mount (ZWO reads actual hardware value during Connect)
		if(m_bIsZWOMount)
			m_dZWOGuideRate = m_pMount->getZWOGuideRate();
	}
	m_pMount->log("[establishLink] m_bLinked=" + std::string(m_bLinked ? "Yes" : "No") + " nErr=" + std::to_string(nErr));
	return nErr;
}

int X2Mount::terminateLink(void)
{
	int nErr = SB_OK;

	m_pMount->log("[terminateLink] Called");
	X2MutexLocker ml(GetMutex());

	nErr = m_pMount->Disconnect();
	m_bLinked = false;
	m_pMount->log("[terminateLink] m_bLinked -> false");

	return nErr;
}

bool X2Mount::isLinked(void) const
{
	return m_pMount->isConnected();
}

bool X2Mount::isEstablishLinkAbortable(void) const
{
	return false;
}

#pragma mark - AbstractDriverInfo

void	X2Mount::driverInfoDetailedInfo(BasicStringInterface& str) const
{
	if(m_nDebugLevel > 0)
		str = "OnStep X2 plugin by Rodolphe Pineau [DEBUG]";
	else
		str = "OnStep X2 plugin by Rodolphe Pineau";
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
		m_pMount->getFirmwareVersion(sFirmware);
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
int X2Mount::raDec(double& ra, double& dec, const bool& )
{
	int nErr = 0;

	if(!m_bLinked)
		return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

	// Get the RA and DEC from the mount
	nErr = m_pMount->getRaAndDec(ra, dec);
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

	nErr = m_pMount->Abort();
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
	nErr = m_pMount->startSlewTo(dRa, dDec);
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
	nErr = pMe->m_pMount->isSlewToComplete(bComplete);
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
	nErr = m_pMount->syncTo(ra, dec);
	if(nErr) {
		nErr = ERR_CMDFAILED;
	}
	return nErr;
}

bool X2Mount::isSynced(void)
{
	if(!m_bLinked)
		return false;

	X2MutexLocker ml(GetMutex());

	int nErr = m_pMount->isAligned(m_bSynced);
	if(nErr)
		m_pMount->log("isSynced: isAligned error");

	return m_bSynced;
}

#pragma mark - TrackingRatesInterface
int X2Mount::setTrackingRates(const bool& bSiderialTrackingOn, const bool& bIgnoreRates, const double& dRaRateArcSecPerSec, const double& dDecRateArcSecPerSec)
{
	int nErr = SB_OK;
	if(!m_bLinked)
		return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

	nErr = m_pMount->setTrackingRates(bSiderialTrackingOn, bIgnoreRates, dRaRateArcSecPerSec, dDecRateArcSecPerSec);

	return nErr;
}

int X2Mount::trackingRates(bool& bSiderialTrackingOn, double& dRaRateArcSecPerSec, double& dDecRateArcSecPerSec)
{
	int nErr = SB_OK;

	if(!m_bLinked)
		return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

	nErr = m_pMount->getTrackRates(bSiderialTrackingOn, dRaRateArcSecPerSec, dDecRateArcSecPerSec);
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
	nErr = m_pMount->setTrackingRates(true, true, 0.0, 0.0);
	return nErr;
}

int X2Mount::trackingOff()
{
	int nErr = SB_OK;
	if(!m_bLinked)
		return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());
	nErr = m_pMount->setTrackingRates(false, true, 0.0, 0.0);
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
	nErr = m_pMount->getAtPark(m_bParked);
	if(nErr) {
		return false;
	}
	m_pMount->log("[isParked] m_bParked=" + std::string(m_bParked ? "Yes" : "No"));
	return m_bParked;
}

int X2Mount::startPark(const double& , const double& )
{
	int nErr = SB_OK;

	if(!m_bLinked)
		return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

	nErr = m_pMount->gotoPark();
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

	nErr =  pMe->m_pMount->isParkingComplete(bComplete);
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
	nErr = m_pMount->unPark();
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

	nErr = pMe->m_pMount->isUnparkDone(bComplete);

	if(bComplete) { // no longer parked.
		pMe->m_bParked = false;
	}
	else
		pMe->m_bParked = true;

	pMe->m_pMount->log("[isCompleteUnpark] bComplete=" + std::string(bComplete ? "Yes" : "No") + " m_bParked=" + std::string(pMe->m_bParked ? "Yes" : "No"));
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
	return true;
}

int X2Mount::beyondThePole(bool& bYes) {
	int nErr = SB_OK;

	if(!m_bLinked)
		return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());

	// “beyond the pole” =  “telescope west of the pier”,
	nErr = m_pMount->IsBeyondThePole(bYes);
	return nErr;
}


double X2Mount::flipHourAngle()
{
	double dHourAngle = 0.0;

	if(!m_bLinked)
		return ERR_NOLINK;

	X2MutexLocker ml(GetMutex());
	int nErr = m_pMount->getflipHourAngle(dHourAngle);
	if(nErr)
		m_pMount->log("flipHourAngle: getflipHourAngle error");

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
	nErr = m_pMount->getLimits(dHoursEast, dHoursWest);
	if(nErr)
		return ERR_CMDFAILED;

	return nErr;
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
