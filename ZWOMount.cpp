#include "ZWOMount.h"

int ZWOMount::Connect(std::string sPort)
{
	std::string sResp;
	int nErr = PLUGIN_OK;

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] ZWOMount Connect Called." << std::endl;
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

	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

	if(m_bSyncLocationDataConnect) {
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
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] SMGE response : '" << sResp << "'" << std::endl;
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
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] SMTI response : '" << sResp << "'" << std::endl;
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

	m_bSyncDone = false;

	if(m_bSyncLocationDataConnect) {
		if(m_nDebugLevel >= 2) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Sending :hC# (auto-home)" << std::endl;
			m_sLogFile.flush();
		}
		m_bIsAtHome = false;
		m_bHasBeenHomed = false;
		homeMount();
		int nHomingAttempts = 0;
		while(!m_bIsAtHome && nHomingAttempts < 120) {
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			isHomingDone(m_bIsAtHome);
			nHomingAttempts++;
		}
		if(m_bIsAtHome) {
			m_bHasBeenHomed = true;
			if(m_nDebugLevel >= 2) {
				m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Homing complete" << std::endl;
				m_sLogFile.flush();
			}
		} else {
			if(m_nDebugLevel >= 1) {
				m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Homing timed out after 60s" << std::endl;
				m_sLogFile.flush();
			}
		}
	} else {
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
	}

	setSlewRate(m_nGoToSlewRate);
	return nErr;
}

int ZWOMount::getLimits(double &dHoursEast, double &dHoursWest)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] ZWOMount Called." << std::endl;
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
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] ZWOMount Called." << std::endl;
		m_sLogFile.flush();
	}

	m_bIsParking = true;
	m_bIsParked = false;
	m_bParkUsesHome = false;

	sendCommand(":hP#", sResp, 0);
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	// ZWO :Gps# — 0: not parked, 1: park in progress, 2: park completed, 3: park error
	sendCommand(":Gps#", sResp);
	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :Gps# after :hP# = '" << sResp << "'" << std::endl;
		m_sLogFile.flush();
	}

	if(sResp == "1" || sResp == "2") {
		if(m_nDebugLevel >= 2) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :hP# accepted, using park position" << std::endl;
			m_sLogFile.flush();
		}
	} else {
		if(m_nDebugLevel >= 2) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :hP# not available, falling back to :hC# (home)" << std::endl;
			m_sLogFile.flush();
		}
		m_bParkUsesHome = true;
		sendCommand(":hC#", sResp, 0);
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}

	return PLUGIN_OK;
}

int ZWOMount::isParkingComplete(bool &bComplete)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_bParkUsesHome) {
		nErr = getStatus();
		if(nErr) {
			bComplete = false;
			return nErr;
		}
		if(m_nDebugLevel >= 2) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] home mode: m_bIsAtHome=" << (m_bIsAtHome?"Yes":"No") << std::endl;
			m_sLogFile.flush();
		}
		if(m_bIsAtHome) {
			bComplete = true;
			m_bIsParking = false;
			m_bIsParked = true;
		} else {
			bComplete = false;
		}
		return PLUGIN_OK;
	}

	// ZWO :Gps# — 0: not parked, 1: park in progress, 2: park completed, 3: park error
	nErr = sendCommand(":Gps#", sResp);
	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :Gps# response='" << sResp << "'" << std::endl;
		m_sLogFile.flush();
	}

	if(nErr || sResp.empty()) {
		nErr = getStatus();
		if(m_bIsAtHome) {
			bComplete = true;
			m_bIsParking = false;
			m_bIsParked = true;
		} else {
			bComplete = false;
		}
		return PLUGIN_OK;
	}

	if(sResp == "2") {
		bComplete = true;
		m_bIsParking = false;
		m_bIsParked = true;
	} else if(sResp == "3") {
		bComplete = true;
		m_bIsParking = false;
		m_bIsParked = false;
		if(m_nDebugLevel >= 1) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Park error reported by mount" << std::endl;
			m_sLogFile.flush();
		}
	} else {
		bComplete = false;
	}

	return PLUGIN_OK;
}
int ZWOMount::isAligned(bool &bAligned)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] ZWOMount Called. m_bHasBeenHomed=" << (m_bHasBeenHomed?"Yes":"No") << std::endl;
		m_sLogFile.flush();
	}

	// ZWO :Gh# — returns 0 (never homed) or 1 (has been homed)
	nErr = sendCommand(":Gh#", sResp);
	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :Gh# nErr=" << nErr << " sResp='" << sResp << "' len=" << sResp.size() << std::endl;
		if(sResp.size()) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :Gh# bytes:";
			for(size_t i = 0; i < sResp.size(); i++)
				m_sLogFile << " 0x" << std::hex << (int)(unsigned char)sResp[i];
			m_sLogFile << std::dec << std::endl;
		}
		m_sLogFile.flush();
	}

	if(!nErr && (sResp == "1" || sResp == "1#"))
		bAligned = true;
	else
		bAligned = m_bHasBeenHomed;

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] bAligned=" << (bAligned?"Yes":"No") << std::endl;
		m_sLogFile.flush();
	}

	return PLUGIN_OK;
}
int ZWOMount::getflipHourAngle(double &dHourAngle)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] ZWOMount Called." << std::endl;
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
