#include <cstdio>
#include "ZWOMount.h"

int ZWOMount::Connect(std::string sPort)
{
	std::string sResp;
	int nErr = PLUGIN_OK;

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called. port=" << sPort << std::endl;
		m_sLogFile.flush();
	}

	m_sPort.assign(sPort);
	m_bIsConnected = false;

	if (!m_pSerx->isConnected()) {
		nErr = m_pSerx->open(m_sPort.c_str(), m_nPortSpeed, SerXInterface::B_NOPARITY);
		if(nErr == 0)
			m_bIsConnected = true;
		else
			m_bIsConnected = false;
	}
	else
		m_bIsConnected = true;

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_bIsConnected: -> " << (m_bIsConnected?"Yes":"No") << std::endl;
		m_sLogFile.flush();
	}

	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

	{
		double dLongitude = m_pTsx->longitude();
		double dLatitude = m_pTsx->latitude();
		double dTimeZone = m_pTsx->timeZone();

		int yy, mm, dd, h, min, dst;
		double sec;
		m_pTsx->localDateTime(yy, mm, dd, h, min, sec, dst);

		if(dst)
			dTimeZone += 1.0;
		double dTzForMount = -dTimeZone;
		char cTzSign = dTzForMount >= 0 ? '+' : '-';
		int nTzHH = (int)std::fabs(dTzForMount);
		int nTzMM = (int)((std::fabs(dTzForMount) - nTzHH) * 60.0);

		std::string sLat, sLong;
		convertDecDegToDDMMSS(dLatitude, sLat);
		convertDecDegToDDMMSS(dLongitude, sLong);

		if(m_nDebugLevel >= 2) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] sLong : " << sLong << std::endl;
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] sLat  : " << sLat << std::endl;
			m_sLogFile.flush();
		}

		// ZWO compound command: :SMGEsDD*MM:SS&sDDD*MM:SS#
		// Longitude needs 3-digit degrees (DDD), pad if needed
		std::string sLongPadded = sLong;
		char cLongSign = sLongPadded[0];
		std::string sLongBody = sLongPadded.substr(1);
		size_t nStarPos = sLongBody.find('*');
		if(nStarPos != std::string::npos && nStarPos < 3) {
			sLongBody = std::string(3 - nStarPos, '0') + sLongBody;
		}
		sLongPadded = cLongSign + sLongBody;

		std::stringstream ssGeoCmd;
		ssGeoCmd << ":SMGE" << sLat << "&" << sLongPadded << "#";
		if(m_nDebugLevel >= 2) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] SMGE cmd : " << ssGeoCmd.str() << std::endl;
			m_sLogFile.flush();
		}
		nErr = sendCommand(ssGeoCmd.str(), sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1);
		if(nErr == COMMAND_TIMEOUT)
			nErr = PLUGIN_OK;
		if(m_nDebugLevel >= 2) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] SMGE response : '" << sResp << "' (nErr=" << nErr << ")" << std::endl;
			m_sLogFile.flush();
		}

		yy = yy - (int(yy / 1000) * 1000);
		int nSec = (int)sec;

		// ZWO compound command: :SMTIMM/DD/YY&HH:MM:SS&sHH:MM#
		std::stringstream ssCmd;
		ssCmd << ":SMTI"
			<< std::setfill('0') << std::setw(2) << mm << "/"
			<< std::setfill('0') << std::setw(2) << dd << "/"
			<< std::setfill('0') << std::setw(2) << yy << "&"
			<< std::setfill('0') << std::setw(2) << h << ":"
			<< std::setfill('0') << std::setw(2) << min << ":"
			<< std::setfill('0') << std::setw(2) << nSec << "&"
			<< cTzSign
			<< std::setfill('0') << std::setw(2) << nTzHH << ":"
			<< std::setfill('0') << std::setw(2) << nTzMM << "#";

		if(m_nDebugLevel >= 2) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] SMTI cmd : " << ssCmd.str() << std::endl;
			m_sLogFile.flush();
		}

		nErr |= sendCommand(ssCmd.str(), sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1);
		if(nErr == COMMAND_TIMEOUT)
			nErr = PLUGIN_OK;
		if(m_nDebugLevel >= 2) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] SMTI response : '" << sResp << "' (nErr=" << nErr << ")" << std::endl;
			m_sLogFile.flush();
		}

		if(nErr) {
			if(m_nDebugLevel >= 1) {
				m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Site/time sync error " << nErr << std::endl;
				m_sLogFile.flush();
			}
			m_bIsConnected = false;
			return nErr;
		}
	}

	// Query current homing state without moving the mount.
	nErr = isHomingDone(m_bIsAtHome);
	if(nErr) {
		if(nErr == ERR_TXTIMEOUT)
			m_bIsConnected = false;
		if(m_nDebugLevel >= 1) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] isHomingDone error : " << nErr << std::endl;
			m_sLogFile.flush();
		}
		m_bIsConnected = false;
		return nErr;
	}
	if(m_bIsAtHome && !m_bHasBeenHomed) {
		if(m_nDebugLevel >= 2) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_bHasBeenHomed: No -> Yes (at home at connect)" << std::endl;
			m_sLogFile.flush();
		}
		m_bHasBeenHomed = true;
	}

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_bIsAtHome=" << (m_bIsAtHome?"Yes":"No") << " m_bHasBeenHomed=" << (m_bHasBeenHomed?"Yes":"No") << std::endl;
		m_sLogFile.flush();
	}

	setSlewRate(m_nGoToSlewRate);

	// Read guide rate from mount so m_dZWOGuideRate reflects actual hardware state.
	double dGuideRate;
	if(getGuideRate(dGuideRate) != PLUGIN_OK) {
		if(m_nDebugLevel >= 1) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] getGuideRate failed, keeping default" << std::endl;
			m_sLogFile.flush();
		}
	}

	// Initialize m_bIsParked from :Gps# — ZWO firmware does not set the P flag in
	// :GU#, so getStatus() (called inside isHomingDone above) leaves m_bIsParked
	// incorrect. getAtPark() queries :Gps# and syncs m_bIsParked as a side-effect.
	{ bool bDummy; getAtPark(bDummy); }

	// NOTE: This function deliberately does not call OnStep::Connect() because the
	// base would call setSiteData() (separate :Sg/:St/:SG commands), which ZWO
	// firmware does not support. ZWO requires the compound SMGE+SMTI commands sent
	// above instead. If OnStep::Connect() gains new init in the future, replicate
	// the relevant parts here.

	return nErr;
}

int ZWOMount::getDeviceName(std::string &sName)
{
	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
		m_sLogFile.flush();
	}
	int nErr = sendCommand(":GVP#", sName);
	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] sName='" << sName << "' nErr=" << nErr << std::endl;
		m_sLogFile.flush();
	}
	return nErr;
}

int ZWOMount::getAtPark(bool &bParked)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
		m_sLogFile.flush();
	}

	bParked = false;

	// ZWO uses :Gps# for park status, not the P/p flag in :GU#
	// :Gps# returns: 0=not parked, 1=park in progress, 2=park completed, 3=park error
	nErr = sendCommand(":Gps#", sResp);
	if(nErr || sResp.empty()) {
		// :Gps# unreliable on this firmware — fall back to m_bZWOParked which is
		// set by finalizepark() and cleared by gotoPark()/unPark().
		bParked = m_bZWOParked;
		if(m_nDebugLevel >= 2) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :Gps# failed (nErr=" << nErr << "), fallback bParked=" << (bParked?"Yes":"No") << std::endl;
			m_sLogFile.flush();
		}
		return PLUGIN_OK;
	}

	bParked = (sResp[0] == '2');
	m_bIsParked = bParked;

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :Gps# response='" << sResp << "' bParked=" << (bParked?"Yes":"No") << std::endl;
		m_sLogFile.flush();
	}

	return PLUGIN_OK;
}

int ZWOMount::isTrackingOn(bool &bTrackOn)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
		m_sLogFile.flush();
	}

	bTrackOn = false;

	// ZWO uses :GAT# for tracking status rather than the 'n' flag in :GU#
	// Response: 0 = not tracking, non-zero = tracking active
	nErr = sendCommand(":GAT#", sResp);
	if(nErr || sResp.empty()) {
		// Fall back to cached state from last :GU# parse
		bTrackOn = m_bIsTracking;
		if(m_nDebugLevel >= 2) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :GAT# failed (nErr=" << nErr << "), fallback bTrackOn=" << (bTrackOn?"Yes":"No") << std::endl;
			m_sLogFile.flush();
		}
		return PLUGIN_OK;
	}

	bTrackOn = (sResp[0] != '0');
	m_bIsTracking = bTrackOn;

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :GAT# response='" << sResp << "' bTrackOn=" << (bTrackOn?"Yes":"No") << std::endl;
		m_sLogFile.flush();
	}

	return PLUGIN_OK;
}

int ZWOMount::getLimits(double &dHoursEast, double &dHoursWest)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called."<< std::endl;
		m_sLogFile.flush();
	}

	nErr = sendCommand(":GTa#", sResp);
	if(nErr) {
		if(m_nDebugLevel >= 1) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :GTa# ERROR : " << nErr << " , sResp : " << sResp << std::endl;
			m_sLogFile.flush();
		}
		dHoursEast = 6.0;
		dHoursWest = 6.0;
		return PLUGIN_OK;
	}

	try {
		// ZWO :GTa# response format: nnsnn# — digits 3-5 are limit angle past meridian in degrees
		if (sResp.length() >= 5) {
			std::string sAngle = sResp.substr(2, 3);
			double dLimitDeg = std::stod(sAngle);
			double dLimitHours = dLimitDeg / 15.0;
			
			dHoursEast = dLimitHours;
			dHoursWest = dLimitHours;
		} else {
			dHoursEast = 6.0;
			dHoursWest = 6.0;
		}
	} catch (const std::exception& e) {
		if(m_nDebugLevel >= 1) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :GTa# Parse exception : " << e.what() << std::endl;
			m_sLogFile.flush();
		}
		dHoursEast = 6.0;
		dHoursWest = 6.0;
	}

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] dHoursEast  : " << std::fixed << std::setprecision(8) << dHoursEast << std::endl;
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] dHoursWest  : " << std::fixed << std::setprecision(8) << dHoursWest << std::endl;
		m_sLogFile.flush();
	}

	return PLUGIN_OK;
}

int ZWOMount::gotoPark()
{
	std::string sResp;

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
		m_sLogFile.flush();
	}

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_bIsParking: " << (m_bIsParking?"Yes":"No") << " -> Yes" << std::endl;
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_bIsParked: " << (m_bIsParked?"Yes":"No") << " -> No" << std::endl;
		m_sLogFile.flush();
	}
	m_bIsParking = true;
	m_bIsParked = false;
	m_bZWOParked = false;

	// TSX has already slewed the mount to the park position.
	// Register the current position as custom park 1, then execute the park.
	// :Sp01# requires homing to have been done (error 5 = PARK_NOT_GO_HOME).
	// :hP# has no response — fire-and-forget (spec §Park command).
	sendCommand(":Sp01#", sResp);
	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :Sp01# response='" << sResp << "'" << std::endl;
		m_sLogFile.flush();
	}
	sendCommand(":hP#", sResp, 0);

	return PLUGIN_OK;
}

int ZWOMount::isParkingComplete(bool &bComplete)
{
	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
		m_sLogFile.flush();
	}

	// ZWO :Gps# is unreliable on this firmware (always returns empty).
	// TSX has already slewed the mount to the park position before calling startPark;
	// :Sp01# + :hP# in gotoPark() complete essentially instantly since the mount is
	// already there. Declare success immediately so TSX advances to endPark.
	bComplete = true;
	m_bIsParking = false;
	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] bComplete=Yes (ZWO: already at park position)" << std::endl;
		m_sLogFile.flush();
	}
	return PLUGIN_OK;
}
int ZWOMount::unPark()
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
		m_sLogFile.flush();
	}

	m_bZWOParked = false;

	// ZWO-specific unpark — :Spu# sends no response, fire-and-forget like :hP#
	nErr = sendCommand(":Spu#", sResp, 0);
	if(nErr) {
		if(m_nDebugLevel >= 1) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :Spu# ERROR : " << nErr << " , sResp : " << sResp << std::endl;
			m_sLogFile.flush();
		}
		return nErr;
	}

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :Spu# success" << std::endl;
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_bIsParked: " << (m_bIsParked?"Yes":"No") << " -> No" << std::endl;
		m_sLogFile.flush();
	}
	m_bIsParked = false;
	setTrackingRates(true, true, 0.0, 0.0);
	return PLUGIN_OK;
}

int ZWOMount::isUnparkDone(bool &bComplete)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
		m_sLogFile.flush();
	}

	// ZWO doesn't have an unparking state, it's instantaneous.
	// But we can check if it's still parked just in case.
	nErr = sendCommand(":Gps#", sResp);
	if(nErr) {
		// Fallback if Gps fails
		nErr = getStatus();
		bComplete = !m_bIsParked;
		if(m_nDebugLevel >= 2) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :Gps# failed, fallback bComplete=" << (bComplete?"Yes":"No") << std::endl;
			m_sLogFile.flush();
		}
		return PLUGIN_OK;
	}

	if (sResp.length() >= 1) {
		int parkStatus = sResp[0] - '0';
		if (parkStatus == 2) {
			// Still parked
			bComplete = false;
			if(m_nDebugLevel >= 2) {
				m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] bComplete=No (still parked per :Gps#)" << std::endl;
				m_sLogFile.flush();
			}
			return PLUGIN_OK;
		}
	}

	bComplete = true;
	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] bComplete=Yes" << std::endl;
		m_sLogFile.flush();
	}
	return PLUGIN_OK;
}

int ZWOMount::homeMount()
{
	std::string sResp;
	int nErr = PLUGIN_OK;

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called. m_bIsAtHome=" << (m_bIsAtHome?"Yes":"No") << " m_bHasBeenHomed=" << (m_bHasBeenHomed?"Yes":"No") << std::endl;
		m_sLogFile.flush();
	}

	// The ZWO firmware sets the H flag in :GU# at power-up even before a physical
	// homing sweep, so m_bIsAtHome cannot be used to skip sending :hC#.
	// Always send the homing command when explicitly requested.
	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_bHasBeenHomed: " << (m_bHasBeenHomed?"Yes":"No") << " -> No" << std::endl;
		m_sLogFile.flush();
	}
	m_bHasBeenHomed = false;

	nErr = sendCommand(":hC#", sResp, 0);
	if(nErr) {
		if(m_nDebugLevel >= 1) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :hC# error " << nErr << std::endl;
			m_sLogFile.flush();
		}
		return nErr;
	}

	// Give the mount time to start moving before the first isHomingDone poll.
	// Without this, getStatus() immediately after :hC# may still see H=set, h=clear.
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	return nErr;
}

int ZWOMount::isHomingDone(bool &bIsHomed)
{
	int nErr = PLUGIN_OK;

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
		m_sLogFile.flush();
	}

	bIsHomed = false;

	nErr = getStatus();
	if(nErr) {
		if(m_nDebugLevel >= 1) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] getStatus error " << nErr << std::endl;
			m_sLogFile.flush();
		}
		return nErr;
	}

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_bIsHoming=" << (m_bIsHoming?"Yes":"No") << " m_bIsSlewing=" << (m_bIsSlewing?"Yes":"No") << " m_bIsAtHome=" << (m_bIsAtHome?"Yes":"No") << std::endl;
		m_sLogFile.flush();
	}

	// Still homing or still moving — not done yet
	if(m_bIsHoming || m_bIsSlewing) {
		bIsHomed = false;
		if(m_nDebugLevel >= 2) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] homing in progress, bIsHomed=No" << std::endl;
			m_sLogFile.flush();
		}
		return PLUGIN_OK;
	}

	bIsHomed = m_bIsAtHome;
	if(bIsHomed && !m_bHasBeenHomed) {
		if(m_nDebugLevel >= 2) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_bHasBeenHomed: No -> Yes" << std::endl;
			m_sLogFile.flush();
		}
		m_bHasBeenHomed = true;
	}

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] bIsHomed=" << (bIsHomed?"Yes":"No") << std::endl;
		m_sLogFile.flush();
	}

	return PLUGIN_OK;
}

int ZWOMount::isAligned(bool &bAligned)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
		m_sLogFile.flush();
	}
	if(m_nDebugLevel >= 3) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_bHasBeenHomed=" << (m_bHasBeenHomed?"Yes":"No") << std::endl;
		m_sLogFile.flush();
	}

	nErr = sendCommand(":Gh#", sResp);
	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :Gh# nErr=" << nErr << " sResp='" << sResp << "'" << std::endl;
		m_sLogFile.flush();
	}
	if(!nErr && sResp == "1")
		bAligned = true;
	else
		bAligned = m_bHasBeenHomed;

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] bAligned : " << (bAligned?"Yes":"No") << std::endl;
		m_sLogFile.flush();
	}

	return PLUGIN_OK;
}
int ZWOMount::gotoParkPos(double dAlt, double dAz)
{
	// ZWO has no AltAz GOTO (:MA# doesn't exist). Ignore the TSX-provided coordinates
	// and use the mount's stored park position via :hP# instead.
	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called. Alt=" << dAlt << " Az=" << dAz << " (ignoring coords — no AltAz slew on ZWO, delegating to gotoPark())" << std::endl;
		m_sLogFile.flush();
	}
	(void)dAlt; (void)dAz;
	return gotoPark();
}

int ZWOMount::finalizepark()
{
	// TSX calls endPark() after isCompletePark() returns true.
	// Set m_bZWOParked so getAtPark() returns true even though :Gps# and
	// :GU# are unreliable on this ZWO firmware.
	m_bZWOParked = true;
	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called. m_bZWOParked=true." << std::endl;
		m_sLogFile.flush();
	}
	return PLUGIN_OK;
}

int ZWOMount::setCurentPosAsPark()
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
		m_sLogFile.flush();
	}

	// ZWO sets custom park position 1 (assuming index 01)
	// command format: :Sp01#
	nErr = sendCommand(":Sp01#", sResp);
	if(nErr) {
		if(m_nDebugLevel >= 1) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :Sp01# ERROR " << nErr << std::endl;
			m_sLogFile.flush();
		}
		return nErr;
	}

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :Sp01# success" << std::endl;
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_bIsParked: " << (m_bIsParked?"Yes":"No") << " -> No" << std::endl;
		m_sLogFile.flush();
	}
	m_bIsParked = false;
	return PLUGIN_OK;
}

int ZWOMount::getHeightLimits(bool &bEnabled, int &nUpperDeg, int &nLowerDeg)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
		m_sLogFile.flush();
	}

	// Enable/disable state has no read-back command; caller uses INI-cached value.
	bEnabled = false;

	nErr = sendCommand(":GLH#", sResp);
	if(nErr || sResp.empty()) {
		nUpperDeg = 90;
	} else {
		try { nUpperDeg = std::stoi(sResp); }
		catch (...) { nUpperDeg = 90; }
	}

	nErr = sendCommand(":GLL#", sResp);
	if(nErr || sResp.empty()) {
		nLowerDeg = 0;
	} else {
		try { nLowerDeg = std::stoi(sResp); }
		catch (...) { nLowerDeg = 0; }
	}

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] upper=" << nUpperDeg << " lower=" << nLowerDeg << std::endl;
		m_sLogFile.flush();
	}

	return PLUGIN_OK;
}

int ZWOMount::setHeightLimits(bool bEnable, int nUpperDeg, int nLowerDeg)
{
	int nErr = PLUGIN_OK;
	std::string sResp;
	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called. bEnable=" << (bEnable?"Yes":"No") << " upper=" << nUpperDeg << " lower=" << nLowerDeg << std::endl;
		m_sLogFile.flush();
	}

	nErr = sendCommand(bEnable ? ":SLE#" : ":SLD#", sResp, 0);
	if(nErr == COMMAND_TIMEOUT) nErr = PLUGIN_OK;

	{
		std::ostringstream oss;
		oss << ":SLH" << std::setw(2) << std::setfill('0') << nUpperDeg << "#";
		nErr = sendCommand(oss.str(), sResp);
	}
	if(nErr && m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :SLH error " << nErr << std::endl;
		m_sLogFile.flush();
	}

	{
		std::ostringstream oss;
		oss << ":SLL" << std::setw(2) << std::setfill('0') << nLowerDeg << "#";
		nErr = sendCommand(oss.str(), sResp);
	}
	if(nErr && m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :SLL error " << nErr << std::endl;
		m_sLogFile.flush();
	}

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] enabled=" << bEnable << " upper=" << nUpperDeg << " lower=" << nLowerDeg << std::endl;
		m_sLogFile.flush();
	}

	return PLUGIN_OK;
}

int ZWOMount::getMeridianConfig(int &nTrackPastDeg, int &nSlewPastDeg)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
		m_sLogFile.flush();
	}

	nErr = sendCommand(":GTa#", sResp);
	if(nErr || sResp.length() < 5) {
		nTrackPastDeg = 0;
		nSlewPastDeg = 0;
		return PLUGIN_OK;
	}

	try {
		// ZWO :GTa# response: nnsnn# — first nn = two single-bit flags (digit1=flip, digit2=continue-tracking),
		// snn = signed slew limit angle in degrees (0..15). We store nn as an opaque int so the round-trip
		// through setMeridianConfig (:STa%02d%+03d#) preserves the flags exactly.
		nTrackPastDeg = std::stoi(sResp.substr(0, 2));
		nSlewPastDeg  = std::stoi(sResp.substr(2, 3));
	} catch (...) {
		nTrackPastDeg = 0;
		nSlewPastDeg  = 0;
	}

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] track=" << nTrackPastDeg << " slew=" << nSlewPastDeg << std::endl;
		m_sLogFile.flush();
	}

	return PLUGIN_OK;
}

int ZWOMount::setMeridianConfig(int nTrackPastDeg, int nSlewPastDeg)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called. track=" << nTrackPastDeg << " slew=" << nSlewPastDeg << std::endl;
		m_sLogFile.flush();
	}

	// :STannsnn# — nn = two single-bit flags (digit1=flip, digit2=continue-tracking), snn = signed slew limit angle (0..15 degrees past meridian)
	std::ostringstream oss;
	oss << ":STa"
	    << std::setw(2) << std::setfill('0') << nTrackPastDeg
	    << std::showpos << std::internal << std::setw(3) << std::setfill('0') << nSlewPastDeg
	    << "#";
	std::string cmd = oss.str();
	nErr = sendCommand(cmd, sResp);
	if(nErr && m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :STa error " << nErr << std::endl;
		m_sLogFile.flush();
	}

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] cmd='" << cmd << "'" << std::endl;
		m_sLogFile.flush();
	}

	return PLUGIN_OK;
}

int ZWOMount::getGuideRate(double &dRate)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
		m_sLogFile.flush();
	}

	nErr = sendCommand(":Ggr#", sResp);
	if(nErr) {
		if(m_nDebugLevel >= 1) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :Ggr# ERROR " << nErr << std::endl;
			m_sLogFile.flush();
		}
		return nErr;
	}

	try {
		dRate = std::stod(sResp);
	} catch(...) {
		if(m_nDebugLevel >= 1) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] parse error sResp='" << sResp << "'" << std::endl;
			m_sLogFile.flush();
		}
		return COMMAND_FAILED;
	}

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_dZWOGuideRate: " << m_dZWOGuideRate << " -> " << dRate << std::endl;
		m_sLogFile.flush();
	}
	m_dZWOGuideRate = dRate;

	return PLUGIN_OK;
}

int ZWOMount::setGuideRate(double dRate)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called. dRate=" << dRate << std::endl;
		m_sLogFile.flush();
	}

	std::ostringstream oss;
	oss << std::fixed << std::setprecision(2) << ":Rg" << dRate << "#";
	std::string cmd = oss.str();
	nErr = sendCommand(cmd, sResp);
	if(nErr) {
		if(m_nDebugLevel >= 1) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] " << cmd << " ERROR " << nErr << std::endl;
			m_sLogFile.flush();
		}
		return nErr;
	}

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_dZWOGuideRate: " << m_dZWOGuideRate << " -> " << dRate << std::endl;
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] cmd='" << cmd << "'" << std::endl;
		m_sLogFile.flush();
	}
	m_dZWOGuideRate = dRate;

	return PLUGIN_OK;
}

int ZWOMount::getflipHourAngle(double &dHourAngle)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called."<< std::endl;
		m_sLogFile.flush();
	}

	nErr = sendCommand(":GTa#", sResp);
	if(nErr) {
		if(m_nDebugLevel >= 1) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :GTa# ERROR : " << nErr << " , sResp : " << sResp << std::endl;
			m_sLogFile.flush();
		}
		dHourAngle = 0.0;
		return PLUGIN_OK;
	}

	try {
		// ZWO :GTa# response format: nnsnn# — digits 3-5 are limit angle past meridian in degrees
		if (sResp.length() >= 5) {
			std::string sAngle = sResp.substr(2, 3);
			double dLimitDeg = std::stod(sAngle);
			dHourAngle = std::fabs(dLimitDeg) / 15.0;
		} else {
			dHourAngle = 0.0;
		}
	} catch (const std::exception& e) {
		if(m_nDebugLevel >= 1) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :GTa# Parse exception : " << e.what() << std::endl;
			m_sLogFile.flush();
		}
		dHourAngle = 0.0;
	}

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] dHourAngle  : " << std::fixed << std::setprecision(8) << dHourAngle << std::endl;
		m_sLogFile.flush();
	}

	return PLUGIN_OK;
}

// ZWO spec v2.1: :Rn# speed levels 0-9 = 0.25x, 0.5x, 1x, 2x, 4x, 8x, 20x, 60x, 720x, 1440x sidereal.
// These differ from standard OnStep at indices 6-9 (OnStep: 24x, 48x, Half-Max, Max).
static const std::vector<std::string> s_ZWOSlewRateNames = {
	"0.25x", "0.5x", "1x", "2x", "4x", "8x", "20x", "60x", "720x", "1440x"
};

int ZWOMount::getRateName(int nZeroBasedIndex, std::string &sOut)
{
	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called. nZeroBasedIndex=" << nZeroBasedIndex << std::endl;
		m_sLogFile.flush();
	}
	if(nZeroBasedIndex < 0 || nZeroBasedIndex >= (int)s_ZWOSlewRateNames.size())
		return PLUGIN_ERROR;
	sOut = s_ZWOSlewRateNames[nZeroBasedIndex];
	return PLUGIN_OK;
}

// ZWO GOTO always runs at firmware-fixed maximum speed.
// Per ZWO protocol spec v2.1 procedure example: GOTO flow is :Sr# -> :Sd# -> :MS#.
// :Rn# is never part of the GOTO sequence — it only controls OLM (manual move) speed.
// Do NOT call setSlewRate() here; it would clobber the user's OLM rate setting for no benefit.
int ZWOMount::startSlewTo(double dRa, double dDec)
{
	int nErr = PLUGIN_OK;
	bool bAligned;

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called. dRa=" << dRa << " dDec=" << dDec << std::endl;
		m_sLogFile.flush();
	}

	nErr = isAligned(bAligned);
	if(nErr)
		return nErr;
	if(!bAligned) {
		if(m_nDebugLevel >= 1) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Mount not homed/aligned, refusing slew." << std::endl;
			m_sLogFile.flush();
		}
		return ERR_MOUNTNOTHOMED;
	}

	nErr = setTarget(dRa, dDec);
	if(nErr)
		return nErr;

	nErr = slewTargetRaDecEpochNow();
	if(nErr) {
		if(m_nDebugLevel >= 1) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] error " << nErr << std::endl;
			m_sLogFile.flush();
		}
		return nErr;
	}
	m_bIsSlewing = true;
	return PLUGIN_OK;
}
