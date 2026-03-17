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

#define PLUGIN_VERSION 1.200


enum OnStepErrors {PLUGIN_OK=0, NOT_CONNECTED, PLUGIN_CANT_CONNECT, PLUGIN_BAD_CMD_RESPONSE, COMMAND_FAILED, PLUGIN_ERROR, COMMAND_TIMEOUT};
enum OnStepTrackRate {NOT_TRACKING, SIDEREAL, LUNAR, SOLAR, KING, TRACKING_OTHER};
enum OnStepSideOfPier {WEST, EAST};

#define SERIAL_BUFFER_SIZE 		256
#define MAX_TIMEOUT 			2000	// WiFi on the OnStep can take up to 1600 ms to respond !!!
#define MAX_READ_WAIT_TIMEOUT 	25
#define ND_LOG_BUFFER_SIZE 		256
#define ERR_PARSE   			1

#define INTER_COMMAND_WAIT				100 //ms
#define PLUGIN_NB_SLEW_SPEEDS 			10
#define NO_RESPONSE_COMMAND_DELAY_MS	100
#define SHORT_RESPONSE  				0x04   // EOT

// Define Class for Astrometric Instruments OnStep controller.
class OnStep
{
public:
	OnStep();
	~OnStep();

	int Connect(std::string sPort);
	int Disconnect();
	void Reconnect(int nNewPortSpeed);
	bool isConnected() const { return m_bIsConnected; }
	
	void setPortSpeed(int nPortSpeed);

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
	int getGoToSlewRate();
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


	void setDebugLevel(int nLevel);
	void log(std::string sLogEntry);
private:

	SerXInterface                       *m_pSerx;
	TheSkyXFacadeForDriversInterface    *m_pTsx;

	bool    m_bIsConnected = false;                               // Connected to the mount?
	int		m_nPortSpeed = 9600;
	std::string	m_sPort;

	std::string m_sFirmwareVersion;
	double  m_dRa = 0;
	double  m_dDec = 0;
	double  m_dAlt = 0;
	double  m_dAz = 270.00;

	bool    m_bSyncLocationDataConnect = false;
	bool    m_bHomeOnUnpark = false;
	bool	m_bIsHoming = false;
	bool    m_bIsAtHome = false;
	bool    m_bIsParked = false;
	bool	m_bIsTracking = false;
	bool	m_bIsParking = false;
	bool	m_bIsSlewing = false;
	int     m_nNbHomingTries = 0;
	bool    m_bStopTrackingOnDisconnect = false;
	int		m_nTrackRate = 0;
	int		m_nSideOfPier = 0;
	int     m_nGoToSlewRate = 0;

	double m_dRaRateArcSecPerSec = 0;
	double m_dDecRateArcSecPerSec = 0;

	double  m_dParkAz = 270.00;
	double  m_dParkAlt = 0;

	bool    m_bSyncDone = false;
	int		m_nAlignementStars = 0;

	std::string     m_sTime;
	std::string     m_sDate;

	double  m_dGotoRATarget = 0;						  // Current Target RA;
	double  m_dGotoDECTarget = 0;                      // Current Goto Target Dec;

	MountDriverInterface::MoveDir      m_nOpenLoopDir;

	// limits don't change mid-course so we cache them
	bool    m_bLimitCached = false;
	double  m_dHoursEast = 8.0;
	double  m_dHoursWest = 8.0;

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
	void	convertDecDegToDDMMSS_ForAlt(double dAlt, std::string &sResult);
	void    convertDecAzToDDMMSSs(double dDeg, std::string &sResult);

	int     convertDDMMSSToDecDeg(const std::string sStrDeg, double &dDecDeg);
	void    convertRaToHHMMSSt(double dRa, std::string &sResult);
	int     convertHHMMSStToRa(const std::string szStrRa, double &dRa);

	int     parseFields(const std::string sIn, std::vector<std::string> &svFields, char cSeparator);

	std::vector<std::string>    m_svSlewRateNames = {"0.25x", "0.5x", "1x (Guide)", "2x", "4x (Centering)", "8x (Move)", "24x (Slew)", "48x", "Half-Max", "Max" };

	std::string&    trim(std::string &str, const std::string &filter );
	std::string&    ltrim(std::string &str, const std::string &filter);
	std::string&    rtrim(std::string &str, const std::string &filter);

	
	CStopWatch  m_commandDelayTimer;

	int m_nDebugLevel;
	const std::string getTimeStamp();
	std::ofstream m_sLogFile;
	std::string m_sLogfilePath;

};

#endif // __OnStep__
