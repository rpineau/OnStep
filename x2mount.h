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
#include "../../licensedinterfaces/mount/pulseguideinterface2.h"

#include "OnStep.h"
#include "ZWOMount.h"


#define PARENT_KEY			 "OnStepMount"
#define CHILD_KEY_PORT_NAME  "PortName"
#define CHILD_KEY_PORT_SPEED "PortSpeed"
#define CHILD_KEY_SYNC_TIME  "SyncTime"
#define CHILD_KEY_PARK_POS   "ParkPos"
#define CHILD_KEY_STOP_TRK   "StopTrackingOnDisconnect"
#define CHILD_KEY_SLEW_RATE  "SlewRate"
#define CHILD_KEY_GUIDE_RATE "GuideRate"
#define CHILD_KEY_ZWO_GUIDE_RATE     "ZWOGuideRate"
#define CHILD_KEY_DEBUG_LVL          "DebugLevel"
#define CHILD_KEY_ZWO_HEIGHT_ENABLED "ZWOHeightLimitsEnabled"
#define CHILD_KEY_ZWO_HEIGHT_UPPER   "ZWOHeightLimitUpper"
#define CHILD_KEY_ZWO_HEIGHT_LOWER   "ZWOHeightLimitLower"
#define CHILD_KEY_ZWO_MERIDIAN_TRACK "ZWOMeridianTrack"
#define CHILD_KEY_ZWO_MERIDIAN_SLEW  "ZWOMeridianSlew"

#define MAX_PORT_NAME_SIZE 120

#define DEF_PORT_NAME		"No port found"

#if defined(WIN32)
#define __CLASS_ATTRIBUTE__(x)
#elif defined(__APPLE__)
// Clang/macOS: weak is valid on types (marks vtable/typeinfo symbols weak)
#define __CLASS_ATTRIBUTE__(x) __attribute__(x)
#else
// GCC/Linux: weak is invalid on types; keep only visibility
#define __CLASS_ATTRIBUTE__(x) __attribute__((visibility("default")))
#endif

class DirectGuideInterface {
public:
    virtual ~DirectGuideInterface() {}
    virtual int directGuideMoveTelescope(const double& dRA, const double& dDec) = 0;
    virtual int directGuideAbort() = 0;
    virtual bool directGuideAsynchronous() = 0;
    virtual int setDirectGuideAsynchronous(bool bAsync) = 0;
};

// FindHomeInterface — undocumented TSX interface, reverse-engineered from TheSkyX binary.
//
// TSX vtable layout (ARM64, 8-byte slots):
//   slot 0 (+0x00): deleting destructor
//   slot 1 (+0x08): complete object destructor
//   slot 2 (+0x10): startFindHome()
//   slot 3 (+0x18): isCompleteFindHome(bool& bComplete) const
//   slot 4 (+0x20): endFindHome()
//   slot 5 (+0x28): motorStatus(unsigned short& u1, unsigned short& u2)
//
// TSX background thread MountThread::findHomeLoop() at binary offset 0x63786c:
//   1. Sets mount state = 18
//   2. Calls startFindHome()
//   3. Polls every ~100ms: isCompleteFindHome() + motorStatus() + motorStatus2()
//      - If motorStatus/motorStatus2 returns u1==0xfa2 OR u2==0xfa2: abort with ERR_MKS_MOTOR_POSERRORLIM
//      - 0,0 is safe and expected for non-MKS mounts
//   4. When bComplete==true: endFindHome()
//   5. emitAsyncOpComplete(opType=18, err, Mount::isIndex())
//
// DlgWorkbench::hardwareFindHome() at 0x48ba64:
//   - Calls startFindHome() synchronously then immediately returns (async hand-off)
//   - Does NOT wait for findHomeLoop completion
//   - Calls updateHomeStatus(1) before startFindHome, updateHomeStatus(0) after
//     (but only if motorStatus2 returned true — otherwise skips second call)
class FindHomeInterface {
public:
    virtual ~FindHomeInterface() {}
    virtual int startFindHome() = 0;
    virtual int isCompleteFindHome(bool& bComplete) const = 0;
    virtual int endFindHome() = 0;
    virtual int motorStatus(unsigned short& u1, unsigned short& u2) = 0;
};

// ============================================================================
// motorStatus / motorStatus2 bit-field reference (reverse-engineered)
//
// MountThread::poll1HomeAndJoysticking (0x634c40) calls motorStatus (via
// MotorStatusInterface vtable slot +0x10) on every poll cycle and uses the
// results to drive the TSX mount state machine ([Mount+88] via setMountState).
//
// Bit fields in motorStatus u1 / u2  (unsigned short, 16-bit):
//
//   Value/Bit  | Who checks it        | Effect when set
//   -----------+----------------------+------------------------------------------
//   == 0x0fa2  | u1 OR u2             | state → 5  (ERR_MKS_MOTOR_POSERRORLIM)
//   bit 0x0100 | u1 AND u2 both set   | state → 18 (joystick active)
//   bit 0x1000 | u1 AND u2 both set   | state → 0  (READY — slews allowed)
//              | u1 OR u2 CLEAR       | state → 7  (NOT_HOMED → ERR_MOUNTNOTHOMED=231)
//   bit 0x2000 | u1 OR u2             | state → 14 (unknown; possibly secondary joystick)
//
// Priority order (highest wins): 0x0fa2 → bit 0x100 → bit 0x1000 missing → bit 0x2000
//
// motorStatus2 (MotorStatusInterface vtable slot +0x18) is SEPARATE and has
// different semantics — see MotorStatusInterface below.
//
// findHomeLoop (0x63786c) also polls motorStatus during homing:
//   - If u1 == 0x0fa2 OR u2 == 0x0fa2 → abort homing immediately
//   - All other values (including 0,0) are safe for non-MKS hardware
//
// IMPORTANT: For motorStatus, u2 IS safe to write. All known call sites pass a
// valid stack address in x2. (The "do not write u2" restriction is only for
// motorStatus2 in updateHomeStatus, where x2 holds a vtable thunk pointer.)
// ============================================================================

// MotorStatusInterface — undocumented TSX interface, reverse-engineered from TheSkyX binary.
//
// TSX vtable layout (ARM64, 8-byte slots):
//   slot 0 (+0x00): deleting destructor
//   slot 1 (+0x08): complete object destructor
//   slot 2 (+0x10): motorStatus(unsigned short& u1, unsigned short& u2)   ← drives mount state machine
//   slot 3 (+0x18): motorStatus2(unsigned short& u1, unsigned short& u2)  ← drives UI homed indicator
//
// motorStatus (slot +0x10):
//   Called by poll1HomeAndJoysticking every poll cycle. Drives mount state machine.
//   See bit-field reference above for full semantics.
//   When homed and ready: return u1=0x1000, u2=0x1000.
//   When not homed:       return u1=0, u2=0.
//   MUST write u2 — poll1HomeAndJoysticking checks bit 0x1000 on BOTH u1 AND u2.
//
// motorStatus2 (slot +0x18) — DIFFERENT semantics from motorStatus:
//   Drives the "Mount is homed." / "Mount not homed!" UI button in DlgWorkbench.
//   DlgWorkbench::updateHomeStatus(bool bHoming) at 0x48bce0 reads u1 with ldrb
//   and checks: nErr==0 AND u1==1 → green ("Mount is homed."), else red.
//   NOTE: the comparison is == 1, not != 0. Do NOT return 0x1000 here.
//     u1 == 1  →  button shows "Mount is homed."  (green stylesheet)
//     u1 == 0  →  button shows "Mount not homed!" or "Homing mount..." (red)
//   Also called by findHomeLoop every 100ms — u1==0xfa2 aborts homing.
//   MUST NOT write u2 — at the updateHomeStatus call site x2 holds the vtable thunk
//   address, not an output pointer. Writing u2 corrupts the thunk. (Bug 5)
//
// Must NOT issue serial commands — called at ~10Hz from status polls and findHomeLoop.
class MotorStatusInterface {
public:
    virtual ~MotorStatusInterface() {}
    virtual int motorStatus(unsigned short& u1, unsigned short& u2) = 0;
    virtual int motorStatus2(unsigned short& u1, unsigned short& u2) = 0;
};

class __CLASS_ATTRIBUTE__((weak,visibility("default"))) X2Mount : public MountDriverInterface
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
						,public PulseGuideInterface2
						,public FindHomeInterface
						,public DirectGuideInterface
						,public MotorStatusInterface
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

	//PulseGuideInterface
	virtual int useOpenLoopMoveInterface(int& nGuideRateIndex, OpenLoopMoveInterface** pOLSI);

	// DirectGuideInterface
	virtual int directGuideMoveTelescope(const double& dRA, const double& dDec);
	virtual int directGuideAbort();
	virtual bool directGuideAsynchronous();
	virtual int setDirectGuideAsynchronous(bool bAsync);

	// FindHomeInterface
	virtual int startFindHome();
	virtual int isCompleteFindHome(bool& bComplete) const;
	virtual int endFindHome();

	// MotorStatusInterface
	virtual int motorStatus(unsigned short& u1, unsigned short& u2);
	virtual int motorStatus2(unsigned short& u1, unsigned short& u2);

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
	virtual void                    setParity(const SerXInterface::Parity& ){};
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

	OnStep *m_pMount;
	bool m_bIsZWOMount;

	bool 	m_bLinked;

	bool 	m_bSynced;
	bool 	m_bParked;
	bool 	m_bHoming;
	bool 	m_bSettingPark;
	bool	m_bFindHomeInitiated;

	int 	m_nParkPosIndex;
	bool 	m_bSyncOnConnect;
	int 	m_nSlewRateIndex;
	bool 	m_bStopTrackingOnDisconnect;
	int		m_nPortSpeed;
	int 	m_CurrentRateIndex;
	int 	m_GuideRateIndex;
	double	m_dZWOGuideRate;

	int 	m_nDebugLevel;

	bool	m_bZWOHeightLimitsEnabled;
	int		m_nZWOHeightLimitUpper;
	int		m_nZWOHeightLimitLower;
	int		m_nZWOMeridianTrack;
	int		m_nZWOMeridianSlew;

	void getPortName(std::string &sPortName) const;

	std::vector<int>    m_svPortSpeed = {9600, 19200, 57600, 115200, 230400, 460800 };

};

#endif // __X2_MOUNT_OnStep__
