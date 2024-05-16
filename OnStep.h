#ifndef __OnStep__
#define __OnStep__

#pragma once
// C++ includes
#include <string>
#include <cstring>
#include <vector>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <thread>
#include <ctime>
#include <cmath>
#include <algorithm>

#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/theskyxfacadefordriversinterface.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/mountdriverinterface.h"
#include "../../licensedinterfaces/mount/asymmetricalequatorialinterface.h"

#include "StopWatch.h"

#define PLUGIN_VERSION 1.000

#define PLUGIN_DEBUG 3   // define this to have log files, 1 = bad stuff only, 2 and up.. full debug

enum OnStepErrors {PLUGIN_OK=0, NOT_CONNECTED, PLUGIN_CANT_CONNECT, PLUGIN_BAD_CMD_RESPONSE, COMMAND_FAILED, PLUGIN_ERROR, COMMAND_TIMEOUT};
enum OnStepTrackRate {NOT_TRACKING, SIDEREAL, LUNAR, SOLAR, KING, TRACKING_OTHER};
enum OnStepSideOfPier {WEST, EAST};

#define SERIAL_BUFFER_SIZE 256
#define MAX_TIMEOUT 2000            // WiFi  on tht OnStep can take up to 1600 ms to respond !!!
#define MAX_READ_WAIT_TIMEOUT 25
#define ND_LOG_BUFFER_SIZE 256
#define ERR_PARSE   1

#define PLUGIN_NB_SLEW_SPEEDS 10
#define INTER_COMMAND_DELAY_SECONDS     0.150
#define SHORT_RESPONSE  0x04   // EOT

#define SmallestFloat               0.0000005F
#define SIDEREAL_RATE_HZ            60.16427456104770L
#define hzToSidereal(x)             ((x)/(double)SIDEREAL_RATE_HZ)
#define siderealToHz(x)             ((x)*(double)SIDEREAL_RATE_HZ)
#define fequal(x,y)                 (fabs((x)-(y))<SmallestFloat)
#define fgt(x,y)                    ((x)-(y)>SmallestFloat)

#define TSX_ARCSEC_SEC				15.0410681

// Define Class for Astrometric Instruments OnStep controller.
class OnStep
{
public:
	OnStep();
	~OnStep();

	int Connect(std::string sPort);
	int Disconnect();
	bool isConnected() const { return m_bIsConnected; }

	void setSerxPointer(SerXInterface *p) { m_pSerx = p; }
	void setTSX(TheSkyXFacadeForDriversInterface *pTSX) { m_pTsx = pTSX;};

	int getFirmwareVersion(std::string &sFirmware);

	int getRaAndDec(double &dRa, double &dDec);
	int getAltAndAz(double &dAlt, double &dAz);
	int syncTo(double dRa, double dDec);
	int isAligned(bool &bAligned);

	int setTrackingRates(bool bSiderialTrackingOn, bool bIgnoreRates, double dRaRateArcSecPerSec, double dDecRateArcSecPerSec);
	int getTrackRates(bool &bSiderialTrackingOn, double &dRaRateArcSecPerSec, double &dDecRateArcSecPerSec);
	int isTrackingOn(bool &bTrakOn);

	int setSlewRate(int nRate);
	void setGoToSlewRate(int nRate);
	int gsetGoToSlewRate();
	int startSlewTo(double dRa, double dDec);
	int isSlewToComplete(bool &bComplete);

	int startOpenLoopMove(const MountDriverInterface::MoveDir Dir, unsigned int nRate);
	int stopOpenLoopMove();
	int getNbSlewRates();
	int getRateName(int nZeroBasedIndex, std::string &sOut);

	int gotoParkPos(double dAlt, double dAz);
	int gotoPark();

	int isParkingComplete(bool &bComplete);
	int getAtPark(bool &bParked);
	int unPark();
	int isUnparkDone(bool &bcomplete);
	int setCurentPosAsPark();

	int getLimits(double &dHoursEast, double &dHoursWest);
	int getflipHourAngle(double &dHourAngle);
	int Abort();

	int setSiteData(double dLongitude, double dLatitute, double dTimeZone);
	int getSiteData(std::string &sLongitude, std::string &sLatitude, std::string &sTimeZone);
	void setSyncLocationDataConnect(bool bSync);

	int getLocalTime(std::string &sTime);
	int getLocalDate(std::string &sDate);
	int syncTime();
	int syncDate();

	int homeMount();
	int isHomingDone(bool &bIsHomed);

	int IsBeyondThePole(bool &bBeyondPole);

	void setStopTrackingOnDisconnect(bool bLeaveOn);


#ifdef PLUGIN_DEBUG
	void log(std::string sLogEntry);
#endif
private:

	SerXInterface                       *m_pSerx;
	TheSkyXFacadeForDriversInterface    *m_pTsx;

	bool    m_bIsConnected;                               // Connected to the mount?
	std::string m_sFirmwareVersion;
	double  m_dRa;
	double  m_dDec;
	double  m_dAlt;
	double  m_dAz;

	bool    m_bSyncLocationDataConnect;
	bool    m_bHomeOnUnpark;
	bool	m_bIsHoming;
	bool    m_bIsAtHome;
	bool    m_bIsParked;
	bool	m_bIsTracking;
	bool	m_bIsParking;
	bool	m_bIsSlewing;
	int     m_nNbHomingTries;
	bool    m_bStopTrackingOnDisconnect;
	int		m_nTrackRate;
	int		m_nSideOfPier;
	int     m_nGoToSlewRate;

	double m_dRaRateArcSecPerSec;
	double m_dDecRateArcSecPerSec;

	double  m_dParkAz;
	double  m_dParkAlt;

	bool    m_bSyncDone;
	int		m_nAlignementStars;

	std::string     m_sTime;
	std::string     m_sDate;

	double  m_dGotoRATarget;						  // Current Target RA;
	double  m_dGotoDECTarget;                      // Current Goto Target Dec;

	MountDriverInterface::MoveDir      m_nOpenLoopDir;

	// limits don't change mid-course so we cache them
	bool    m_bLimitCached;
	double  m_dHoursEast;
	double  m_dHoursWest;

	int     sendCommand(const std::string sCmd, std::string &sResp, int nTimeout = MAX_TIMEOUT, char cEndOfResponse = '#', int nExpectedResLen = 1);
	int     readResponse(std::string &sResp, int nTimeout = MAX_TIMEOUT, char cEndOfResponse = '#', int nExpectedResLen = 1);

	int     getStatus();

	int     setSiteLongitude(const std::string sLongitude);
	int     setSiteLatitude(const std::string sLatitude);
	int     setSiteTimezone(const std::string sTimezone);

	int     getSiteLongitude(std::string &sLongitude);
	int     getSiteLatitude(std::string &sLatitude);
	int     getSiteTZ(std::string &sTimeZone);

	int     setTarget(double dRa, double dDec);
	int     setTargetAltAz(double dAlt, double dAz);
	int     slewTargetRaDecEpochNow();
	int		slewTargetAltAszEpochNow();
	int		setSlewSpeed(int nSlewRateIndex);

	void    convertDecDegToDDMMSS(double dDeg, std::string &sResult);
	void    convertDecDegToDDMMSS_ForDecl(double dDeg, std::string &sResult);
	void    convertDecAzToDDMMSSs(double dDeg, std::string &sResult);

	int     convertDDMMSSToDecDeg(const std::string sStrDeg, double &dDecDeg);
	void    convertRaToHHMMSSt(double dRa, std::string &sResult);
	int     convertHHMMSStToRa(const std::string szStrRa, double &dRa);

	int     parseFields(const std::string sIn, std::vector<std::string> &svFields, char cSeparator);

	std::vector<std::string>    m_svSlewRateNames = {"0.25x", "0.5x", "1x (Guide)", "2x", "4x (Centering)", "8x (Move)", "24x (Slew)", "48x", "Half-Max", "Max" };

	CStopWatch  m_commandDelayTimer;

#ifdef PLUGIN_DEBUG
	// timestamp for logs
	const std::string getTimeStamp();
	std::ofstream m_sLogFile;
	std::string m_sLogfilePath;
#endif

};

#endif // __OnStep__
