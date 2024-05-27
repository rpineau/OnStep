#ifndef __X2_MOUNT_OnStep__
#define __X2_MOUNT_OnStep__
#pragma once
#include <string.h>
#include <math.h>

#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/basicstringinterface.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/basiciniutilinterface.h"
#include "../../licensedinterfaces/theskyxfacadefordriversinterface.h"
#include "../../licensedinterfaces/sleeperinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"
#include "../../licensedinterfaces/mutexinterface.h"
#include "../../licensedinterfaces/tickcountinterface.h"
#include "../../licensedinterfaces/serialportparams2interface.h"
#include "../../licensedinterfaces/modalsettingsdialoginterface.h"
#include "../../licensedinterfaces/x2guiinterface.h"
#include "../../licensedinterfaces/mountdriverinterface.h"
#include "../../licensedinterfaces/mount/slewtointerface.h"
#include "../../licensedinterfaces/mount/syncmountinterface.h"
#include "../../licensedinterfaces/mount/asymmetricalequatorialinterface.h"
#include "../../licensedinterfaces/mount/openloopmoveinterface.h"
#include "../../licensedinterfaces/mount/needsrefractioninterface.h"
#include "../../licensedinterfaces/mount/trackingratesinterface.h"
#include "../../licensedinterfaces/parkinterface.h"
#include "../../licensedinterfaces/unparkinterface.h"
#include "../../licensedinterfaces/driverslewstoparkpositioninterface.h"

// Include files for OnStep mount
#include "OnStep.h"


#define PARENT_KEY			 "OnStepMount"
#define CHILD_KEY_PORT_NAME  "PortName"
#define CHILD_KEY_PORT_SPEED "PortSpeed"
#define CHILD_KEY_SYNC_TIME  "SyncTime"
#define CHILD_KEY_PARK_POS   "ParkPos"
#define CHILD_KEY_STOP_TRK   "StopTrackingOnDisconnect"
#define CHILD_KEY_SLEW_RATE  "SlewRate"

#define MAX_PORT_NAME_SIZE 120

#define DEF_PORT_NAME		"No port found"

#if defined(WIN32)
class X2Mount : public MountDriverInterface
#else
class __attribute__((weak,visibility("default"))) X2Mount : public MountDriverInterface
#endif
						,public SyncMountInterface
						,public SlewToInterface
                        ,public AsymmetricalEquatorialInterface
						,public OpenLoopMoveInterface
						,public TrackingRatesInterface
						,public ParkInterface
						,public UnparkInterface
						,public ModalSettingsDialogInterface
                        ,public X2GUIEventInterface
                        ,public SerialPortParams2Interface
                        ,public DriverSlewsToParkPositionInterface
{
public:
	/*!Standard X2 constructor*/
	X2Mount(const char* pszDriverSelection,
			const int& nInstanceIndex,
			SerXInterface					* pSerX,
			TheSkyXFacadeForDriversInterface	* pTheSkyX,
			SleeperInterface					* pSleeper,
			BasicIniUtilInterface			* pIniUtil,
			LoggerInterface					* pLogger,
			MutexInterface					* pIOMutex,
			TickCountInterface				* pTickCount);

	~X2Mount();

	// Operations
public:

	/*!\name DriverRootInterface Implementation
	 See DriverRootInterface.*/
	//@{
	virtual DeviceType							deviceType(void)							  {return DriverRootInterface::DT_MOUNT;}
	virtual int									queryAbstraction(const char* pszName, void** ppVal) ;
	//@}

	/* See LinkInterface.*/
	//@{
	virtual int									establishLink(void)						;
	virtual int									terminateLink(void)						;
	virtual bool								isLinked(void) const					;
	virtual bool								isEstablishLinkAbortable(void) const	;
	//@}

	/*!\name DriverInfoInterface Implementation
	 See DriverInfoInterface.*/
	//@{
	virtual void								driverInfoDetailedInfo(BasicStringInterface& str) const;
	virtual double								driverInfoVersion(void) const				;
	//@}

	/*!\name HardwareInfoInterface Implementation
	 See HardwareInfoInterface.*/
	//@{
	virtual void deviceInfoNameShort(BasicStringInterface& str) const				;
	virtual void deviceInfoNameLong(BasicStringInterface& str) const				;
	virtual void deviceInfoDetailedDescription(BasicStringInterface& str) const	;
	virtual void deviceInfoFirmwareVersion(BasicStringInterface& str)				;
	virtual void deviceInfoModel(BasicStringInterface& str)						;
	//@}

	virtual int									raDec(double& ra, double& dec, const bool& bCached = false)					;
	virtual int									abort(void)																	;

	//Optional interfaces, uncomment and implement as required.

	//SyncMountInterface
	virtual int syncMount(const double& ra, const double& dec)									;
	virtual bool isSynced()																		;

	//SlewToInterface
	virtual int								startSlewTo(const double& dRa, const double& dDec)	;
	virtual int								isCompleteSlewTo(bool& bComplete) const				;
	virtual int								endSlewTo(void)										;

	//AsymmetricalEquatorialInterface
	virtual bool knowsBeyondThePole();
	virtual int beyondThePole(bool& bYes);
	virtual double flipHourAngle();
	virtual int gemLimits(double& dHoursEast, double& dHoursWest);

	// SymmetricalEquatorialInterface
	virtual MountTypeInterface::Type mountType();

	//OpenLoopMoveInterface
	virtual int								startOpenLoopMove(const MountDriverInterface::MoveDir& Dir, const int& nRateIndex);
	virtual int								endOpenLoopMove(void);
	virtual bool							allowDiagonalMoves() {return true;}
	virtual int								rateCountOpenLoopMove(void) const;
	virtual int								rateNameFromIndexOpenLoopMove(const int& nZeroBasedIndex, char* pszOut, const int& nOutMaxSize);
	virtual int								rateIndexOpenLoopMove(void);

	//NeedsRefractionInterface
	virtual bool							needsRefactionAdjustments(void);

	//TrackingRatesInterface
	virtual int setTrackingRates( const bool& bSiderialTrackingOn, const bool& bIgnoreRates, const double& dRaRateArcSecPerSec, const double& dDecRateArcSecPerSec);
	virtual int trackingRates( bool& bSiderialTrackingOn, double& dRaRateArcSecPerSec, double& dDecRateArcSecPerSec);
	virtual int siderealTrackingOn();
	virtual int trackingOff();

	/* Parking Interface */
	virtual bool							isParked(void);
	virtual int								startPark(const double& dAz, const double& dAlt);
	virtual int								isCompletePark(bool& bComplete) const;
	virtual int								endPark(void);

	/* Unparking Interface */
	int								startUnpark(void);
	int								isCompleteUnpark(bool& bComplete) const;
	int								endUnpark(void);

	//SerialPortParams2Interface
	virtual void            portName(BasicStringInterface& str) const            ;
	virtual void            setPortName(const char* szPort)                        ;
	virtual unsigned int    baudRate() const            {return 115200;};
	virtual void            setBaudRate(unsigned int)    {};
	virtual bool            isBaudRateFixed() const        {return true;}

	virtual SerXInterface::Parity    parity() const                {return SerXInterface::B_NOPARITY;}
	virtual void                    setParity(const SerXInterface::Parity& parity){};
	virtual bool                    isParityFixed() const        {return true;}

	// GUI Interface
	virtual int initModalSettingsDialog(void) { return 0; }
	virtual int execModalSettingsDialog(void);
	void uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent); // Process a UI event


	// Implementation
private:
	// Sky Interfaces
	SerXInterface 							*GetSerX() {return m_pSerX; }
	TheSkyXFacadeForDriversInterface		*GetTheSkyXFacadeForMounts() {return m_pTheSkyXForMounts;}
	SleeperInterface						*GetSleeper() {return m_pSleeper; }
	BasicIniUtilInterface					*GetSimpleIniUtil() {return m_pIniUtil; }
	LoggerInterface							*GetLogger() {return m_pLogger; }
	MutexInterface							*GetMutex()  {return m_pIOMutex;}
	TickCountInterface						*GetTickCountInterface() {return m_pTickCount;}

	void 	setHomingButton(X2GUIExchangeInterface* uiex, bool bEnable);
	void	setParkingButton(X2GUIExchangeInterface* uiex, bool bEnable);
	
	void	getProgress(char &c, bool bReset = false);
	int		m_nProgress_index;
	std::vector<char>    m_svProgressState = {'|','/','-','\\'};

	// Variables to store Sky X interfaces
	int m_nPrivateMulitInstanceIndex;
	SerXInterface*							m_pSerX;
	TheSkyXFacadeForDriversInterface* 		m_pTheSkyXForMounts;
	SleeperInterface*						m_pSleeper;
	BasicIniUtilInterface*					m_pIniUtil;
	LoggerInterface*						m_pLogger;
	MutexInterface*							m_pIOMutex;
	TickCountInterface*						m_pTickCount;

	// Variables for OnStep
	OnStep m_OnStep;

	bool 	m_bLinked;

	bool 	m_bSynced;
	bool 	m_bParked;
	bool 	m_bHoming;
	bool 	m_bSettingPark;

	int 	m_nParkPosIndex;
	bool 	m_bSyncOnConnect;
	int 	m_nSlewRateIndex;
	bool 	m_bStopTrackingOnDisconnect;

	char 	m_PortName[MAX_PORT_NAME_SIZE];
	int		m_nPortSpeed;
	int 	m_CurrentRateIndex;

	void getPortName(std::string &sPortName) const;

	std::vector<int>    m_svPortSpeed = {9600, 19200, 57600, 115200, 230400, 460800 };

};

#endif // __X2_MOUNT_OnStep__
