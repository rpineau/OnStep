#include "OnStep.h"

// Constructor for OnStep
OnStep::OnStep()
{

    m_commandDelayTimer.Reset();

	m_nDebugLevel = 0;
#if defined(WIN32)
	m_sLogfilePath = getenv("HOMEDRIVE");
	m_sLogfilePath += getenv("HOMEPATH");
	m_sLogfilePath += "\\OnStepLog.txt";
#else
	m_sLogfilePath = getenv("HOME");
	m_sLogfilePath += "/OnStepLog.txt";
#endif
}


OnStep::~OnStep(void)
{
	if(m_sLogFile.is_open())
		m_sLogFile.close();
}

int OnStep::Connect(std::string sPort)
{
	std::string sResp;
	int nErr = PLUGIN_OK;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Connect Called." << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Trying to connect to port " << sPort<< std::endl;
	m_sLogFile.flush();
	}
	m_sPort.assign(sPort);
	m_bIsConnected = false;

	if (!m_pSerx->isConnected()) {
		nErr = m_pSerx->open(m_sPort.c_str(), m_nPortSpeed, SerXInterface::B_NOPARITY);
		if(nErr == 0) {
			m_bIsConnected = true;
		}
		else
			m_bIsConnected = false;
	}
	else
		m_bIsConnected = true;

	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

	if(m_bSyncLocationDataConnect) {
		nErr = setSiteData(m_pTsx->longitude(),
						   m_pTsx->latitude(),
						   m_pTsx->timeZone());
		if(nErr) {
	if(m_nDebugLevel >= 1) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] error " << nErr << ", response = " << sResp << std::endl;
			m_sLogFile.flush();
	}
			m_bIsConnected = false;
			return nErr;
		}
	}
	nErr = isHomingDone(m_bIsAtHome);
	if(nErr) {
		if(nErr == ERR_TXTIMEOUT)
			m_bIsConnected = false;
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] error calling isHomingDone :" << nErr << ", response = " << sResp << std::endl;
		m_sLogFile.flush();
	}
		m_bIsConnected = false;

	}
	setSlewRate(m_nGoToSlewRate);
	// unPark();

	// NOTE: ZWOMount::Connect() does NOT call this function. If you add new
	// initialization here, check whether it also needs to be added to ZWOMount::Connect().

	return nErr;
}


int OnStep::Disconnect(void)
{
	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Disconnect Called." << std::endl;
	m_sLogFile.flush();
	}
	if (m_bIsConnected) {
		if(m_bStopTrackingOnDisconnect)
			setTrackingRates( false, true, 0.0, 0.0); // stop tracking on disconnect.
		if(m_pSerx){
	if(m_nDebugLevel >= 2) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] closing serial port." << std::endl;
			m_sLogFile.flush();
	}
			m_pSerx->flushTx();
			m_pSerx->purgeTxRx();
			m_pSerx->close();
		}
	}
	m_bIsConnected = false;

	return SB_OK;
}

void OnStep::Reconnect(int nNewPortSpeed)
{
	m_nPortSpeed = nNewPortSpeed;
	if(m_bIsConnected) {
		m_pSerx->flushTx();
		m_pSerx->purgeTxRx();
		m_pSerx->close();
		if(m_pSerx->open(m_sPort.c_str(), m_nPortSpeed, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1") == 0)
			m_bIsConnected = true;
		else
			m_bIsConnected = false;
	}
}

void OnStep::setPortSpeed(int nPortSpeed)
{
	m_nPortSpeed = nPortSpeed;
}

// --- OnStep communication ---
int OnStep::sendCommand(const std::string sCmd, std::string &sResp, int nTimeout, char cEndOfResponse, int nExpectedResLen)
{
	int nErr = PLUGIN_OK;
	unsigned long  ulBytesWrite;
	std::vector<std::string> vFieldsData;
	int nBytesWaiting;
	int dDelayMs;

	if(m_commandDelayTimer.GetElapsedSeconds()<INTER_COMMAND_WAIT) {
		dDelayMs = INTER_COMMAND_WAIT - int(m_commandDelayTimer.GetElapsedSeconds() *1000);
		if(dDelayMs>0) {
			std::this_thread::sleep_for(std::chrono::milliseconds(dDelayMs));
			std::this_thread::yield();
		}
	}

	
	nErr = m_pSerx->bytesWaitingRx(nBytesWaiting);
	if(nBytesWaiting) {
	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [nBytesWaiting] calling purgeTxR"<< std::endl;
		m_sLogFile.flush();
	}
		m_pSerx->purgeTxRx();
	}

	sResp.clear();

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] sending '" << sCmd << "'" << std::endl;
	m_sLogFile.flush();
	}

	nErr = m_pSerx->writeFile((void *)sCmd.c_str(), sCmd.size(), ulBytesWrite);
	m_commandDelayTimer.Reset();
	m_pSerx->flushTx();
	if(nErr) {
		if(nErr == ERR_TXTIMEOUT)
			m_bIsConnected = false;
		return nErr;
	}
	// read response
	if(nTimeout == 0) {// no response expected
		std::this_thread::sleep_for(std::chrono::milliseconds(NO_RESPONSE_COMMAND_DELAY_MS));
		return nErr;
	}
	// no response expected
	if(cEndOfResponse == SHORT_RESPONSE && nExpectedResLen==0) {
		std::this_thread::sleep_for(std::chrono::milliseconds(NO_RESPONSE_COMMAND_DELAY_MS));
		return nErr;
	}
	nErr = readResponse(sResp, nTimeout, cEndOfResponse, nExpectedResLen);
	if(nErr) {
	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] ***** ERROR READING RESPONSE **** error = " << nErr << " , response : '" << sResp << "'" << std::endl;
		m_sLogFile.flush();
	}
		return nErr;
	}
	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] response : '" << sResp << "'" <<  std::endl;
	m_sLogFile.flush();
	}

	return nErr;
}


int OnStep::readResponse(std::string &sResp, int nTimeout, char cEndOfResponse, int nExpectedResLen)
{
	int nErr = PLUGIN_OK;
	char pszBuf[SERIAL_BUFFER_SIZE];
	unsigned long ulBytesRead = 0;
	unsigned long ulTotalBytesRead = 0;
	char *pszBufPtr;
	int nBytesWaiting = 0 ;
	int nbTimeouts = 0;

	memset(pszBuf, 0, SERIAL_BUFFER_SIZE);
	pszBufPtr = pszBuf;

	do {
		nErr = m_pSerx->bytesWaitingRx(nBytesWaiting);
	if(m_nDebugLevel >= 3) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] nBytesWaiting      : " << nBytesWaiting << std::endl;
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] nBytesWaiting nErr : " << nErr << std::endl;
		m_sLogFile.flush();
	}
		if(!nBytesWaiting) {
			nbTimeouts += MAX_READ_WAIT_TIMEOUT;
			if(nbTimeouts >= nTimeout) {
	if(m_nDebugLevel >= 3) {
				m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] bytesWaitingRx timeout, no data for " << nbTimeouts << " ms"<< std::endl;
				m_sLogFile.flush();
	}
				nErr = COMMAND_TIMEOUT;
				break;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(MAX_READ_WAIT_TIMEOUT));
			std::this_thread::yield();
			continue;
		}
		nbTimeouts = 0;
		if(ulTotalBytesRead + nBytesWaiting <= SERIAL_BUFFER_SIZE)
			nErr = m_pSerx->readFile(pszBufPtr, nBytesWaiting, ulBytesRead, nTimeout);
		else {
			nErr = ERR_RXTIMEOUT;
			break; // buffer is full.. there is a problem !!
		}
		if(nErr) {
	if(m_nDebugLevel >= 1) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] readFile error : " << nErr << std::endl;
			m_sLogFile.flush();
	}
			return nErr;
		}

		if (ulBytesRead != (unsigned long)nBytesWaiting) {
	if(m_nDebugLevel >= 1) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] readFile Timeout Error." << std::endl;
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] readFile nBytesWaiting : " << nBytesWaiting << std::endl;
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] readFile ulBytesRead   : " << ulBytesRead << std::endl;
			m_sLogFile.flush();
	}
		}

		ulTotalBytesRead += ulBytesRead;
		pszBufPtr+=ulBytesRead;
		// response not ending with the normal end of response char.
		if(cEndOfResponse == SHORT_RESPONSE && ulTotalBytesRead >= (unsigned long)nExpectedResLen)
			break;
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] pszBuf : "  << pszBuf <<  std::endl;
		if(ulBytesRead>1)
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] *(pszBufPtr-1) : "  <<  *(pszBufPtr-1) <<  std::endl;
		if(ulBytesRead>2)
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] *(pszBufPtr-2) : "  << *(pszBufPtr-2) <<  std::endl;
		if(ulBytesRead>3)
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] *(pszBufPtr-3) : "  << *(pszBufPtr-3) <<  std::endl;
		m_sLogFile.flush();
	}
		// NYX hack
		if(ulTotalBytesRead >= 3) {
			if (*(pszBufPtr-2) == cEndOfResponse || *(pszBufPtr-3) == cEndOfResponse)
				break;
		}

	}  while (ulTotalBytesRead < SERIAL_BUFFER_SIZE  && *(pszBufPtr-1) != cEndOfResponse);

	if(!ulTotalBytesRead) {
		nErr = COMMAND_TIMEOUT; // we didn't get an answer.. so timeout
	}
	if(ulTotalBytesRead) {
		sResp.assign(pszBuf);
		sResp = trim(sResp,"\n\r#");
	}
	else
		sResp.clear();

	if(m_nDebugLevel >= 3) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] sResp : '" << sResp << "'" << std::endl;
	m_sLogFile.flush();
	}

	return nErr;
}

int OnStep::getFirmwareVersion(std::string &sFirmware)
{
	int nErr = PLUGIN_OK;
	std::string sResp;
	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	nErr = sendCommand(":GVN#", sResp);
	if(sResp.size() == 0)
		return ERR_CMDFAILED;
	sFirmware.assign(sResp);
	return nErr;
}

int OnStep::getStatus()
{
	int nErr = PLUGIN_OK;
	std::string sStatus;
	int nIndex = 0;
	int nSize;
	nErr = sendCommand(":GU#", sStatus);
	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] ERROR : " << nErr << " , sResp : " << sStatus << std::endl;
		m_sLogFile.flush();
	}
	}

	// setting some default
	m_nTrackRate = SIDEREAL;
	m_bIsTracking = true;
	m_bIsSlewing = true;
	m_bIsParked = false;
	m_bIsParking = false;
	m_bIsHoming = false;
	m_bIsAtHome = false;

	nSize = (int)sStatus.size();
	if(m_nDebugLevel >= 1) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] sStatus (nSize) : " << sStatus << " (" << nSize << ")"<<std::endl;
	m_sLogFile.flush();
	}
	if(nSize) {
		while(nIndex < nSize) {
			switch(sStatus.at(nIndex++)) {
				case 'n':
					m_bIsTracking = false;
					break;
				case 'N':
					m_bIsSlewing = false;
					break;
				case 'p':
					m_bIsParked = false;
					break;
				case 'P':
					m_bIsParked = true;
					break;
				case 'I':
					m_bIsParking = true;
					break;
				case 'h':
					m_bIsHoming = true;
					break;
				case 'H':
					m_bIsAtHome = true;
					break;
				case '(':
					m_nTrackRate = LUNAR;
					break;
				case 'O':
					m_nTrackRate = SOLAR;
					break;
				case 'k':
					m_nTrackRate = KING;
					break;
				case 'T':
					m_nSideOfPier = EAST;
					break;
				case 'W':
					m_nSideOfPier = WEST;
					break;
				default:
					break;
			}
		}
	}

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_bIsTracking : " << (m_bIsTracking?"Yes":"No") << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_bIsSlewing  : " << (m_bIsSlewing?"Yes":"No") << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_bIsParked   : " << (m_bIsParked?"Yes":"No") << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_bIsParking  : " << (m_bIsParking?"Yes":"No") << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_bIsAtHome   : " << (m_bIsAtHome?"Yes":"No") << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_bIsHoming   : " << (m_bIsHoming?"Yes":"No") << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_nTrackRate  : " << m_nTrackRate << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_nSideOfPier : " << (m_nSideOfPier==EAST?"East":"West") << std::endl;
	m_sLogFile.flush();
	}


	return nErr;
}

// --- Mount Coordinates ---
int OnStep::getRaAndDec(double &dRa, double &dDec)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	// get RA
	nErr = sendCommand(":GRH#", sResp);
	if(nErr) {
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		std::this_thread::yield();
		// retry
		nErr = sendCommand(":GRH#", sResp);
		if(nErr) {
	if(m_nDebugLevel >= 1) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :GR# ERROR : " << nErr << " , sResp : " << sResp << std::endl;
			m_sLogFile.flush();
	}
			dRa = m_dRa;
			dDec = m_dDec;
			return PLUGIN_OK; // we will retry
		}
	}

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] sResp : " << sResp << std::endl;
	m_sLogFile.flush();
	}
	if(sResp.size() == 0)
		return ERR_CMDFAILED;

	nErr = convertHHMMSStToRa(sResp, dRa);
	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :GR# convertHHMMSStToRa error : " << nErr << " , sResp : " << sResp << std::endl;
		m_sLogFile.flush();
	}
		dRa = m_dRa;
		dDec = m_dDec;
		return PLUGIN_OK;
	}
	m_dRa = dRa;

	// get DEC
	nErr = sendCommand(":GDH#", sResp);
	if(nErr) {
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		std::this_thread::yield();
		// retry
		nErr = sendCommand(":GDH#", sResp);
		if(nErr) {
	if(m_nDebugLevel >= 1) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :GD# ERROR : " << nErr << " , sResp : " << sResp << std::endl;
			m_sLogFile.flush();
	}
			dRa = m_dRa;
			dDec = m_dDec;
			return PLUGIN_OK; // we will retry
		}
	}
	if(sResp.size() == 0)
		return ERR_CMDFAILED;

	nErr = convertDDMMSSToDecDeg(sResp, dDec);
	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :GD# convertDDMMSSToDecDeg error : " << nErr << " , sResp : " << sResp << std::endl;
		m_sLogFile.flush();
	}
		dRa = m_dRa;
		dDec = m_dDec;
		return PLUGIN_OK;
	}

	m_dDec = dDec;
	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] dDec : " << std::fixed << std::setprecision(12) << dDec << std::endl;
	m_sLogFile.flush();
	}

	return nErr;
}

int OnStep::getAltAndAz(double &dAlt, double &dAz)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	// get Az
	nErr = sendCommand(":GZH#", sResp);
	if(nErr) {
		// retry
		nErr = sendCommand(":GZH#", sResp);
		if(nErr) {
	if(m_nDebugLevel >= 1) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :GZ# ERROR : " << nErr << " , sResp : " << sResp << std::endl;
			m_sLogFile.flush();
	}
			dAlt = m_dAlt;
			dAz = m_dAz;
			return PLUGIN_OK;
		}
	}

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] sResp : " << sResp << std::endl;
	m_sLogFile.flush();
	}
	if(sResp.size() == 0)
		return ERR_CMDFAILED;

	nErr = convertDDMMSSToDecDeg(sResp, dAz);
	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :GZ# convertDDMMSSToDecDeg error : " << nErr << " , sResp : " << sResp << std::endl;
		m_sLogFile.flush();
	}
		dAlt = m_dAlt;
		dAz = m_dAz;
		return PLUGIN_OK;
	}

	m_dAz = dAz;
	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] dAz : " << std::fixed << std::setprecision(12) << dAz << std::endl;
	m_sLogFile.flush();
	}

	// get Alt
	nErr = sendCommand(":GAH#", sResp);
	if(nErr) {
		// retry
		nErr = sendCommand(":GAH#", sResp);
		if(nErr) {
	if(m_nDebugLevel >= 1) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :GA# ERROR : " << nErr << " , sResp : " << sResp << std::endl;
			m_sLogFile.flush();
	}
			dAlt = m_dAlt;
			dAz = m_dAz;
			return PLUGIN_OK;
		}
	}
	if(sResp.size() == 0)
		return ERR_CMDFAILED;
	nErr = convertDDMMSSToDecDeg(sResp, dAlt);
	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :GA# convertDDMMSSToDecDeg error : " << nErr << " , sResp : " << sResp << std::endl;
		m_sLogFile.flush();
	}
		dAlt = m_dAlt;
		dAz = m_dAz;
		return PLUGIN_OK;
	}

	m_dAlt = dAlt;
	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] dAlt : " << std::fixed << std::setprecision(12) << dAlt << std::endl;
	m_sLogFile.flush();
	}

	return nErr;

}


int OnStep::setTarget(double dRa, double dDec)
{
	int nErr;
	std::stringstream ssTmp;
	std::string sResp;
	std::string sTemp;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Ra  : " << std::fixed << std::setprecision(8) << dRa << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Dec : " << std::fixed << std::setprecision(8) << dDec << std::endl;
	m_sLogFile.flush();
	}

	// convert Ra value to HH:MM:SS.SSSS before passing them to OnStep
	convertRaToHHMMSSt(dRa, sTemp);

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Converted Ra : " << sTemp << std::endl;
	m_sLogFile.flush();
	}
	// set target Ra
	ssTmp<<":Sr"<<sTemp<<"#";
	nErr = sendCommand(ssTmp.str(), sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1);
	if(sResp.size() && sResp.at(0)=='1') {
		nErr = PLUGIN_OK;
	}
	else if(nErr) {
	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Error setting target Ra, response : " << sResp << std::endl;
		m_sLogFile.flush();
	}
		return nErr;
	}

	// convert target dec to sDD*MM:SS.SSS
	convertDecDegToDDMMSS_ForDecl(dDec, sTemp);

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Converted Dec : " <<sTemp << std::endl;
	m_sLogFile.flush();
	}
	std::stringstream().swap(ssTmp);
	// set target Dec
	ssTmp<<":Sd"<<sTemp<<"#";
	nErr = sendCommand(ssTmp.str(), sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1);
	if(sResp.size() && sResp.at(0)=='1')
		nErr = PLUGIN_OK;
	else if(nErr) {
	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Error setting target Dec, response : " << sResp << std::endl;
		m_sLogFile.flush();
	}
		return nErr;
	}

	return nErr;
}

int OnStep::setTargetAltAz(double dAlt, double dAz)
{
	int nErr;
	std::stringstream ssTmp;
	std::string sResp;
	std::string sTemp;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Az  : " << std::fixed << std::setprecision(8) << dAz << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Alt : " << std::fixed << std::setprecision(8) << dAlt << std::endl;
	m_sLogFile.flush();
	}

	// convert Az value to DDD*MM:SS
	convertDecAzToDDMMSSs(dAz, sTemp);

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] szTemp(Az)  : " << sTemp << std::endl;
	m_sLogFile.flush();
	}
	// set target Az
	ssTmp<<":Sz"<<sTemp<<"#";
	nErr = sendCommand(ssTmp.str(), sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1);
	if(nErr)
		return nErr;


	// convert Alt value sDD:MM:SS.SSS
	convertDecDegToDDMMSS_ForAlt(dAlt, sTemp);

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "]  szTemp(Alt)  : " <<sTemp << std::endl;
	m_sLogFile.flush();
	}
	// set target Alt
	std::stringstream().swap(ssTmp);
	ssTmp<<":Sa"<<sTemp<<"#";
	nErr = sendCommand(ssTmp.str(), sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1);
	if(nErr)
		return nErr;

	return nErr;
}

// --- Sync and Cal ---
int OnStep::syncTo(double dRa, double dDec)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "]  Ra Hours   : " << std::fixed << std::setprecision(5) << dRa << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "]  Ra Degrees : " << std::fixed << std::setprecision(5) << dRa*15.0 << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "]  Dec        : " << std::fixed << std::setprecision(5) << dDec << std::endl;
	m_sLogFile.flush();
	}

	nErr = setTarget(dRa, dDec);
	if(nErr) {
	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Error setting sync target." << std::endl;
		m_sLogFile.flush();
	}
		return nErr;
	}

//	if(!m_bSyncDone) {
		nErr = sendCommand(":CM#", sResp); // sync
		if(nErr) {
	if(m_nDebugLevel >= 2) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Error syncing to target." << std::endl;
			m_sLogFile.flush();
	}
			return nErr;
		}
		if(sResp.at(0) == 'E') {
			nErr = ERR_CMDFAILED;
			// process error
		}
//	}
//	else {
		// add alignement start
		
//	}
	m_dRa = dRa;
	m_dDec = dDec;

	return nErr;
}

int OnStep::isAligned(bool &bAligned)
{
	int nErr = PLUGIN_OK;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}
	if(m_nDebugLevel >= 3) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_bHasBeenHomed=" << (m_bHasBeenHomed?"Yes":"No") << std::endl;
	m_sLogFile.flush();
	}
	// for now
	bAligned = true;
	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] bAligned=" << (bAligned?"Yes":"No") << std::endl;
	m_sLogFile.flush();
	}
	return nErr;
}

// --- tracking rates ---
int OnStep::setTrackingRates(bool bSiderialTrackingOn, bool bIgnoreRates, double dRaRateArcSecPerSec, double dDecRateArcSecPerSec)
{
	int nErr = PLUGIN_OK;
	std::string sResp;
	std::stringstream ssTmp;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] bSiderialTrackingOn  : " << (bSiderialTrackingOn?"Yes":"No") << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] bIgnoreRates         : " << (bIgnoreRates?"Yes":"No") << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] dRaRateArcSecPerSec  : " << std::fixed << std::setprecision(8) << dRaRateArcSecPerSec << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] dDecRateArcSecPerSec : " << std::fixed << std::setprecision(8) << dDecRateArcSecPerSec << std::endl;
	m_sLogFile.flush();
	}
	// stop tracking
	if(!bSiderialTrackingOn && bIgnoreRates) {
	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] setting to stopped" << std::endl;
		m_sLogFile.flush();
	}
		m_dRaRateArcSecPerSec = 15.0410681;
		m_dDecRateArcSecPerSec = 0.0;
		nErr = sendCommand(":Td#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1); // tracking off
	}
	// sidereal
	else if(bSiderialTrackingOn && bIgnoreRates) {
	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] setting to Sidereal" << std::endl;
		m_sLogFile.flush();
	}
		m_dRaRateArcSecPerSec = 0.0;
		m_dDecRateArcSecPerSec = 0.0;
		nErr = sendCommand(":TQ#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 0); // Sidereal rate
		nErr = sendCommand(":Te#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1); //tracking on
	}
	// Lunar
	else if (0.30 < dRaRateArcSecPerSec && dRaRateArcSecPerSec < 0.83 && -0.25 < dDecRateArcSecPerSec && dDecRateArcSecPerSec < 0.25) {
	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] setting to Lunar" << std::endl;
		m_sLogFile.flush();
	}
		m_dRaRateArcSecPerSec = dRaRateArcSecPerSec;
		m_dDecRateArcSecPerSec = dDecRateArcSecPerSec;
		nErr = sendCommand(":TL#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 0); // Lunar rate
		nErr = sendCommand(":Te#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1); //tracking on
	}
	// solar
	else if (0.037 < dRaRateArcSecPerSec && dRaRateArcSecPerSec < 0.043 && -0.017 < dDecRateArcSecPerSec && dDecRateArcSecPerSec < 0.017) {
	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] setting to Solar" << std::endl;
		m_sLogFile.flush();
	}
		m_dRaRateArcSecPerSec = dRaRateArcSecPerSec;
		m_dDecRateArcSecPerSec = dDecRateArcSecPerSec;
		nErr = sendCommand(":TS#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 0); // Solar rate
		nErr = sendCommand(":Te#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1); //tracking on
	}
	// default to sidereal
	else {
	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] default to sidereal" << std::endl;
		m_sLogFile.flush();
	}
		m_dRaRateArcSecPerSec = 0.0;
		m_dDecRateArcSecPerSec = 0.0;
		nErr = sendCommand(":TQ#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 0); // Sidereal rate
		nErr = sendCommand(":Te#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1); //tracking on
	}

	if(m_nDebugLevel >= 2) {
	nErr = sendCommand(":GT#", sResp);
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] check current tracking value : " << sResp << std::endl;
	m_sLogFile.flush();
	}


	return nErr;
}

int OnStep::getTrackRates(bool &bSiderialTrackingOn, double &dRaRateArcSecPerSec, double &dDecRateArcSecPerSec)
{
	int nErr = PLUGIN_OK;
	std::string sResp;
	bool bTrackingOn;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}
	isTrackingOn(bTrackingOn);
	if(!bTrackingOn) {
		dRaRateArcSecPerSec = 15.0410681; // Convention to say tracking is off - see TSX documentation
		dDecRateArcSecPerSec = 0;
		bSiderialTrackingOn = false;
		return nErr;
	}

	else if(m_nTrackRate == SIDEREAL) {
		dRaRateArcSecPerSec = 0.0;
		dDecRateArcSecPerSec = 0.0;
		bSiderialTrackingOn = true;
	}
	else if(m_nTrackRate == LUNAR ||  m_nTrackRate == SOLAR || m_nTrackRate == KING) {
		dRaRateArcSecPerSec = m_dRaRateArcSecPerSec;	// return the speed we set in TSX
		dDecRateArcSecPerSec = m_dDecRateArcSecPerSec;	// same on Dec.
		bSiderialTrackingOn = false;
	}
	else { // Sidereal by default
		dRaRateArcSecPerSec = 0.0;
		dDecRateArcSecPerSec = 0.0;
		bSiderialTrackingOn = true;
	}
	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] bSiderialTrackingOn  : " << (bSiderialTrackingOn?"Yes":"No") << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] dRaRateArcSecPerSec  : " << std::fixed << std::setprecision(8) << dRaRateArcSecPerSec << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] dDecRateArcSecPerSec : " << std::fixed << std::setprecision(8) << dDecRateArcSecPerSec << std::endl;
	m_sLogFile.flush();
	}
	return nErr;
}


// --- Limits ---
int OnStep::getLimits(double &dHoursEast, double &dHoursWest)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	nErr = sendCommand(":GXEe#", sResp);
	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :GXEe# ERROR : " << nErr << " , sResp : " << sResp << std::endl;
		m_sLogFile.flush();
	}
	} else {
		try {
			dHoursEast = std::stod(sResp)/15.0;
		} catch (const std::exception& e) {
			dHoursEast = 0.0;
		}
	}

	nErr = sendCommand(":GXEw#", sResp);
	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :GXEw# ERROR : " << nErr << " , sResp : " << sResp << std::endl;
		m_sLogFile.flush();
	}
	} else {
		try {
			dHoursWest = std::stod(sResp)/15.0;
		} catch (const std::exception& e) {
			dHoursWest = 0.0;
		}
	}

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] dHoursEast  : " << std::fixed << std::setprecision(8) << dHoursEast << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] dHoursWest  : " << std::fixed << std::setprecision(8) << dHoursWest << std::endl;
	m_sLogFile.flush();
	}

	return nErr;

}

int OnStep::getflipHourAngle(double &dHourAngle)
{
	int nErr = PLUGIN_OK;
	std::string sResp;
	double dWest = 0.0;
	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	nErr = sendCommand(":GXEA#", sResp);
	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] :GXEA# ERROR : " << nErr << " , sResp : " << sResp << std::endl;
		m_sLogFile.flush();
	}
	} else {
		try {
			dWest = std::fabs(std::stod(sResp))/15.0;
		} catch (const std::exception& e) {
			dWest = 0.0;
		}
	}

	dHourAngle = dWest;
	
	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] dHourAngle  : " << std::fixed << std::setprecision(8) << dHourAngle << std::endl;
	m_sLogFile.flush();
	}

	return nErr;
}

// --- Slew ---

int OnStep::setSlewRate(int nRate)
{
	int nErr;
	std::stringstream ssCmd;
	std::string sResp;

	if(nRate>(PLUGIN_NB_SLEW_SPEEDS-1))
		return COMMAND_FAILED;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called. nRate=" << nRate << std::endl;
	m_sLogFile.flush();
	}

	ssCmd << ":R" << nRate << "#";
	nErr = sendCommand(ssCmd.str(), sResp, MAX_TIMEOUT, SHORT_RESPONSE, 0);
	return nErr;
}

void OnStep::setGoToSlewRate(int nRate)
{
	m_nGoToSlewRate = nRate;
	if(m_bIsConnected) {
		setSlewRate(m_nGoToSlewRate);
	}
}

int OnStep::getGoToSlewRate()
{
	return m_nGoToSlewRate;
}

int OnStep::startSlewTo(double dRa, double dDec)
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

	setSlewRate(m_nGoToSlewRate);
	// set sync target coordinate
	nErr = setTarget(dRa, dDec);
	if(nErr)
		return nErr;

	nErr = slewTargetRaDecEpochNow();
	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] error " << nErr << std::endl;
		m_sLogFile.flush();
	}

	}
	m_bIsSlewing = true;

	return nErr;
}

int OnStep::slewTargetRaDecEpochNow()
{
	int nErr;
	std::string sResp;
	int nRespCode;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}
	
	nErr = sendCommand(":MS#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1);
	if(nErr == COMMAND_TIMEOUT)
		nErr = PLUGIN_OK;
	else if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Error slewing, response : " << sResp << std::endl;
		m_sLogFile.flush();
	}
		return ERR_CMDFAILED;
	}
	if(sResp.size()) {
		std::string sCode = sResp;
		if(sCode.size() && sCode[0] == 'e')
			sCode = sCode.substr(1);
		try {
			nRespCode = std::stoi(sCode);
		} catch (const std::exception& e) {
			if(m_nDebugLevel >= 1) {
				m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Parse error on MS# response : '" << sResp << "' : " << e.what() << std::endl;
				m_sLogFile.flush();
			}
			return ERR_CMDFAILED;
		}
		switch(nRespCode) {
			case 0:
				break;

			case 1:
	if(m_nDebugLevel >= 1) {
				m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Limit error below horizon."  << std::endl;
				m_sLogFile.flush();
	}
				nErr = ERR_LX200DESTBELOWHORIZ;
				break;

			case 2:
	if(m_nDebugLevel >= 1) {
				m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Limit error no object."  << std::endl;
				m_sLogFile.flush();
	}
				nErr = ERR_NOOBJECTSELECTED;
				break;

			case 4:
	if(m_nDebugLevel >= 1) {
				m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Limit error position unreachable."  << std::endl;
				m_sLogFile.flush();
	}
				nErr = ERR_GEMINI_POSITION_UNREACHABLE;
				break;

			case 5:
	if(m_nDebugLevel >= 1) {
				m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Limit error not aligned."  << std::endl;
				m_sLogFile.flush();
	}
				nErr = ERR_GEMINI_NOT_ALIGNED;
				break;

			case 6:
	if(m_nDebugLevel >= 1) {
				m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Limit error outside limits."  << std::endl;
				m_sLogFile.flush();
	}
				nErr = ERR_LX200OUTSIDELIMIT;
				break;

			case 7:
	if(m_nDebugLevel >= 1) {
				m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Time and position not synchronized."  << std::endl;
				m_sLogFile.flush();
	}
				nErr = ERR_CMDFAILED;
				break;

			default:
	if(m_nDebugLevel >= 1) {
				m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Unknown GOTO error code : " << nRespCode << std::endl;
				m_sLogFile.flush();
	}
				nErr = ERR_MKS_SLEW_PAST_LIMIT;
				break;

		}
	}
	return nErr;
}

int OnStep::slewTargetAltAszEpochNow()
{
	int nErr;
	std::string sResp;
	int nRespCode;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	nErr = sendCommand(":MA#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1);
	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Error slewing, response : " << sResp << std::endl;
		m_sLogFile.flush();
	}
		return ERR_CMDFAILED;
	}
	if(sResp.size()) {
		std::string sCode = sResp;
		if(sCode.size() && sCode[0] == 'e')
			sCode = sCode.substr(1);
		try {
			nRespCode = std::stoi(sCode);
		} catch (const std::exception& e) {
			if(m_nDebugLevel >= 1) {
				m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Parse error on MA# response : '" << sResp << "' : " << e.what() << std::endl;
				m_sLogFile.flush();
			}
			return ERR_CMDFAILED;
		}
		switch(nRespCode) {
			case 0:
				// all good
				break;

			case 1:
	if(m_nDebugLevel >= 1) {
				m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Limit error below horizon."  << std::endl;
				m_sLogFile.flush();
	}
				nErr = ERR_LX200DESTBELOWHORIZ;
				break;

			case 2:
	if(m_nDebugLevel >= 1) {
				m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Limit error no object."  << std::endl;
				m_sLogFile.flush();
	}
				nErr = ERR_NOOBJECTSELECTED;
				break;

			case 4:
	if(m_nDebugLevel >= 1) {
				m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Limit error position unreachable."  << std::endl;
				m_sLogFile.flush();
	}
				nErr = ERR_GEMINI_POSITION_UNREACHABLE;
				break;

			case 5:
	if(m_nDebugLevel >= 1) {
				m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Limit error not aligned."  << std::endl;
				m_sLogFile.flush();
	}
				nErr = ERR_GEMINI_NOT_ALIGNED;
				break;

			case 6:
	if(m_nDebugLevel >= 1) {
				m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Limit error outside limits."  << std::endl;
				m_sLogFile.flush();
	}
				nErr = ERR_LX200OUTSIDELIMIT;
				break;

			default:
				nErr = ERR_MKS_SLEW_PAST_LIMIT;
				break;

		}
	}
	return nErr;
}

int OnStep::getNbSlewRates()
{
	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called : PLUGIN_NB_SLEW_SPEEDS = " << PLUGIN_NB_SLEW_SPEEDS << std::endl;
	m_sLogFile.flush();
	}
	return PLUGIN_NB_SLEW_SPEEDS;
}


int OnStep::getRateName(int nZeroBasedIndex, std::string &sOut)
{
	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	if (nZeroBasedIndex > PLUGIN_NB_SLEW_SPEEDS)
		return PLUGIN_ERROR;

	sOut.assign(m_svSlewRateNames[nZeroBasedIndex]);
	return PLUGIN_OK;
}

int OnStep::startOpenLoopMove(const MountDriverInterface::MoveDir Dir, unsigned int nRate)
{
	int nErr = PLUGIN_OK;
	std::string sResp;
	std::string sCmd;
	std::stringstream sTmp;
	m_nOpenLoopDirMask |= (1u << static_cast<unsigned>(Dir));

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] setting dir to  : " << Dir << " mask now: " << m_nOpenLoopDirMask << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] setting rate to : " << nRate << std::endl;
	m_sLogFile.flush();
	}

	// select rate
	nErr = setSlewRate(nRate);

	if(nErr)
		return nErr;

	// figure out direction
	switch(Dir){
		case MountDriverInterface::MD_NORTH:
			sCmd = ":Mn#";
			break;
		case MountDriverInterface::MD_SOUTH:
			sCmd = ":Ms#";
			break;
		case MountDriverInterface::MD_EAST:
			sCmd = ":Me#";
			break;
		case MountDriverInterface::MD_WEST:
			sCmd = ":Mw#";
			break;
	}
	nErr = sendCommand(sCmd, sResp, MAX_TIMEOUT, SHORT_RESPONSE, 0);
	return nErr;
}

int OnStep::startPulseGuide(std::string sDirection, int nDurationMs)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] dir: " << sDirection << " ms: " << nDurationMs << std::endl;
		m_sLogFile.flush();
	}

	// Make sure duration is exactly 4 digits
	std::stringstream ss;
	ss << ":Mg" << sDirection << std::setfill('0') << std::setw(4) << nDurationMs << "#";

	nErr = sendCommand(ss.str(), sResp, MAX_TIMEOUT, SHORT_RESPONSE, 0);
	return nErr;
}


int OnStep::stopOpenLoopMove()
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] active dir mask: " << m_nOpenLoopDirMask << std::endl;
	m_sLogFile.flush();
	}

	if(m_nOpenLoopDirMask & (1u << MountDriverInterface::MD_NORTH))
		nErr = sendCommand(":Qn#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 0);
	if(m_nOpenLoopDirMask & (1u << MountDriverInterface::MD_SOUTH))
		nErr = sendCommand(":Qs#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 0);
	if(m_nOpenLoopDirMask & (1u << MountDriverInterface::MD_EAST))
		nErr = sendCommand(":Qe#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 0);
	if(m_nOpenLoopDirMask & (1u << MountDriverInterface::MD_WEST))
		nErr = sendCommand(":Qw#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 0);
	m_nOpenLoopDirMask = 0;

	return nErr;
}


int OnStep::isSlewToComplete(bool &bComplete)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	bComplete = false;
	if(!m_bIsSlewing ) {
		bComplete = true;
	if(m_nDebugLevel >= 2) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_bIsSlewing : " << (m_bIsSlewing?"Yes":"No") << std::endl;
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] bComplete    : " << (bComplete?"Yes":"No") << std::endl;
		m_sLogFile.flush();
	}
		return nErr;
	}

	nErr = getStatus();
	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] getStatus error " << nErr << std::endl;
		m_sLogFile.flush();
	}
		return nErr;
	}

	bComplete = m_bIsSlewing?false:true;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_bIsSlewing : " << (m_bIsSlewing?"Yes":"No") << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] bComplete    : " << (bComplete?"Yes":"No") << std::endl;
	m_sLogFile.flush();
	}

	return nErr;
}

int OnStep::gotoParkPos(double dAlt, double dAz)
{
	int nErr = PLUGIN_OK;
	std::string sResp;
	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	m_bIsParking = false;

	// stop tracking
	nErr = setTrackingRates( false, true, 0.0, 0.0);
	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] setTrackingRates error " << nErr << std::endl;
		m_sLogFile.flush();
	}
		return nErr;
	}
	
	// nErr = m_pTsx->HzToEq(dAz, dAlt, dRa, dDec);

	// go to park coordinate
	nErr = setTargetAltAz(dAlt, dAz);
	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] setTargetAltAz error " << nErr << std::endl;
		m_sLogFile.flush();
	}
		return nErr;
	}


	nErr = slewTargetAltAszEpochNow();
	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] slewTargetAltAszEpochNow error " << nErr << std::endl;
		m_sLogFile.flush();
	}
		return nErr;
	}

	m_bIsParking = true;
	m_bIsSlewing = true;
	return nErr;
}

int OnStep::gotoPark()
{
	int nErr = PLUGIN_OK;
	std::string sResp;
	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	m_bIsParking = false;

	nErr = sendCommand(":hP#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1);

	if(!nErr)
		m_bIsParking = true;
	return nErr;
}

int OnStep::isParkingComplete(bool &bComplete)
{
	int nErr = PLUGIN_OK;

	nErr = getStatus();
	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] getStatus error " << nErr << std::endl;
		m_sLogFile.flush();
	}
		return nErr;
	}

	bComplete = m_bIsParking?false:true;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_bIsParking : " << (m_bIsParking?"Yes":"No") << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] bComplete    : " << (bComplete?"Yes":"No") << std::endl;
	m_sLogFile.flush();
	}

	return nErr;
}

int OnStep::getAtPark(bool &bParked)
{
	int nErr = PLUGIN_OK;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}
	bParked = false;

	nErr = getStatus(); // will update the flags
	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] getStatus error " << nErr << std::endl;
		m_sLogFile.flush();
	}
		return nErr;
	}

	bParked = m_bIsParked;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] bParked   " << (bParked?"Yes":"No") << std::endl;
	m_sLogFile.flush();
	}

	return nErr;
}

int OnStep::unPark()
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	nErr = sendCommand(":hR#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1);
	if(nErr) {
		m_bIsParked = true;
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] error " << nErr << std::endl;
		m_sLogFile.flush();
	}
	}

	nErr = getStatus(); // will update the flags
	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] getStatus error " << nErr << std::endl;
		m_sLogFile.flush();
	}
	}

	return nErr;
}

int OnStep::isUnparkDone(bool &bComplete)
{
	int nErr = PLUGIN_OK;
	bool bAtPArk;
	// double dRa, dDec;
	std::string sResp;
	bool bTrackingOn = false;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	bComplete = false;
	nErr = getAtPark(bAtPArk);
	if(!bAtPArk)
		bComplete = true;
	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] bAtPArk   " << (bAtPArk?"Yes":"No") << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] bComplete " << (bComplete?"Yes":"No") << std::endl;
	m_sLogFile.flush();
	}

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_bIsAtHome   " << (m_bIsAtHome?"Yes":"No") << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] bComplete " << (bComplete?"Yes":"No") << std::endl;
	m_sLogFile.flush();
	}

	setTrackingRates(true, true, 0.0, 0.0);

	isTrackingOn(bTrackingOn);
	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] bTrackingOn   " << (bTrackingOn?"Yes":"No") << std::endl;
	m_sLogFile.flush();
	}

	m_bIsParked = false;
	return nErr;
}

int OnStep::setCurentPosAsPark()
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	nErr = sendCommand(":hQ#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1);
	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] error " << nErr << std::endl;
		m_sLogFile.flush();
	}
	}

	nErr = getStatus(); // will update the flags
	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] getStatus error " << nErr << std::endl;
		m_sLogFile.flush();
	}
	}

	return nErr;
}

int OnStep::homeMount()
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}
	if(m_nDebugLevel >= 3) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_bIsAtHome=" << (m_bIsAtHome?"Yes":"No") << " m_bHasBeenHomed=" << (m_bHasBeenHomed?"Yes":"No") << std::endl;
	m_sLogFile.flush();
	}
	if(m_bIsAtHome) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] already homed." << std::endl;
		m_sLogFile.flush();
	}
		return nErr;
	}

	nErr = sendCommand(":hC#", sResp, 0);
	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] error " << nErr << " , response :" << sResp << std::endl;
		m_sLogFile.flush();
	}
	}

	return nErr;
}


int OnStep::isHomingDone(bool &bIsHomed)
{
	int nErr = PLUGIN_OK;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}
	bIsHomed = false;

	nErr = getStatus(); // will update the flags
	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] getStatus error " << nErr << std::endl;
		m_sLogFile.flush();
	}
		return nErr;
	}

	bIsHomed = m_bIsAtHome;
	if(m_nDebugLevel >= 3) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_bIsAtHome=" << (m_bIsAtHome?"Yes":"No") << " bIsHomed=" << (bIsHomed?"Yes":"No") << std::endl;
	m_sLogFile.flush();
	}
	if(m_bIsAtHome) {
		m_bHasBeenHomed = true;
	if(m_nDebugLevel >= 3) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] m_bHasBeenHomed -> true" << std::endl;
		m_sLogFile.flush();
	}
	}
	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] bIsHomed=" << (bIsHomed?"Yes":"No") << std::endl;
	m_sLogFile.flush();
	}
	return nErr;
}


int OnStep::isTrackingOn(bool &bTrackOn)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	bTrackOn = false;
	nErr = getStatus(); // will update the flags
	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] getStatus error " << nErr << std::endl;
		m_sLogFile.flush();
	}
		return nErr;
	}

	bTrackOn = m_bIsTracking;

	if(m_nDebugLevel >= 1) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] bTrackOn : " << (bTrackOn?"Yes":"No")<< std::endl;
	m_sLogFile.flush();
	}

	return nErr;
}

int OnStep::Abort()
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	nErr = sendCommand(":Q#", sResp, 0);

	return nErr;
}

// --- time and site methods ---
int OnStep::syncTime()
{
	int nErr = PLUGIN_OK;
	int yy, mm, dd, h, min, dst;
	double sec;
	std::string sResp;
	std::stringstream ssTmp;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	m_pTsx->localDateTime(yy, mm, dd, h, min, sec, dst);

	ssTmp << ":SL" << std::setfill('0') << std::setw(2) << h << ":" << std::setfill('0') << std::setw(2) << min << ":" << std::setfill('0') << std::setw(6) << std::fixed << std::setprecision(3) << sec << "#";
	nErr = sendCommand(ssTmp.str(), sResp, 0);
	getLocalTime(m_sTime);

	return nErr;
}


int OnStep::syncDate()
{
	int nErr = PLUGIN_OK;
	int yy, mm, dd, h, min, dst;
	double sec;
	std::string sResp;
	std::stringstream ssTmp;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	m_pTsx->localDateTime(yy, mm, dd, h, min, sec, dst);
	// yy is actually yyyy, need conversion to yy, 2017 -> 17
	yy = yy - (int(yy / 1000) * 1000);

	ssTmp << ":SC" << std::setfill('0') << std::setw(2) << mm << "/" << std::setfill('0') << std::setw(2) << dd << "/" << std::setfill('0') << std::setw(2) << yy << "#";
	nErr = sendCommand(ssTmp.str(), sResp, 0);
	getLocalDate(m_sDate);
	return nErr;
}

int OnStep::setSiteLongitude(const std::string sLongitude)
{
	int nErr = PLUGIN_OK;
	std::string sResp;
	std::stringstream ssTmp;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	// :SgsDDD*MM#
	ssTmp << ":Sg" << sLongitude << "#";
	nErr = sendCommand(ssTmp.str(), sResp, 0);
	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] error " << nErr << ", response : " << sResp << std::endl;
		m_sLogFile.flush();
	}
	}

	return nErr;
}

int OnStep::setSiteLatitude(const std::string sLatitude)
{
	int nErr = PLUGIN_OK;
	std::string sResp;
	std::stringstream ssTmp;
	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	// :StsDD*MM#
	ssTmp << ":St" << sLatitude << "#";
	nErr = sendCommand(ssTmp.str(), sResp, 0);

	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] error " << nErr << ", response : " << sResp << std::endl;
		m_sLogFile.flush();
	}
	}

	return nErr;
}

int OnStep::setSiteTimezone(const std::string sTimezone)
{
	int nErr = PLUGIN_OK;
	std::string sResp;
	std::stringstream ssTmp;
	std::string sCurrentTimeZone;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}
	ssTmp << ":SG" << sTimezone << "#";
	nErr = sendCommand(ssTmp.str(), sResp, 0);

	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] error " << nErr << ", response : " << sResp << std::endl;
		m_sLogFile.flush();
	}
	}

	return nErr;
}

int OnStep::getSiteLongitude(std::string &sLongitude)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	nErr = sendCommand(":GgH#", sResp);
	if(!nErr) {
		sLongitude.assign(sResp);
	}

	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] error " << nErr << ", response : " << sResp << std::endl;
		m_sLogFile.flush();
	}
	}

	return nErr;
}

int OnStep::getSiteLatitude(std::string &sLatitude)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	nErr = sendCommand(":GtH#", sResp);
	if(!nErr) {
		sLatitude.assign(sResp);
	}

	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] error " << nErr << ", response : " << sResp << std::endl;
		m_sLogFile.flush();
	}
	}

	return nErr;
}

int OnStep::getSiteTZ(std::string &sTimeZone)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	nErr = sendCommand(":GG#", sResp);
	if(!nErr) {
		if(sResp.size() == 0)
			return ERR_CMDFAILED;
		sTimeZone.assign(sResp);
		if(sTimeZone.size() && sTimeZone.at(0) == '-') {
			sTimeZone[0] = '+';
		}
		else {
			sTimeZone[0] = '-';
		}
	}

	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] error " << nErr << ", response : " << sResp << std::endl;
		m_sLogFile.flush();
	}
	}

	return nErr;
}

int OnStep::setSiteData(double dLongitude, double dLatitute, double dTimeZone)
{
	int nErr = PLUGIN_OK;
	std::string sLong;
	std::string sLat;
	std::stringstream ssTimeZone;
	std::stringstream  ssHH, ssMM;
	int yy, mm, dd, h, min, dst;
	double sec;
	char cSign;
	double dTimeZoneNew;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}


	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] dLongitude : " << std::fixed << std::setprecision(5) << dLongitude << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] dLatitute  : " << std::fixed << std::setprecision(5) << dLatitute << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] dTimeZone  : " << std::fixed << std::setprecision(2) << dTimeZone << std::endl;
	m_sLogFile.flush();
	}

	convertDecDegToDDMMSS(dLongitude, sLong);
	convertDecDegToDDMMSS(dLatitute, sLat);

	m_pTsx->localDateTime(yy, mm, dd, h, min, sec, dst);
	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] dst        : " << (dst != 0 ?"Yes":"No") << std::endl;
	m_sLogFile.flush();
	}

	if(dst) {
		dTimeZone += 1.0;
	}
	dTimeZoneNew = -dTimeZone;
	cSign = dTimeZoneNew>=0?'+':'-';
	dTimeZoneNew=std::fabs(dTimeZone);

	ssTimeZone << cSign << std::setfill('0') << std::setw(2) << dTimeZoneNew;

	sLong.assign(sLong);

	sLat.assign(sLat);

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] sLong      : " << sLong << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] sLat       : " << sLat<< std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] ssTimeZone : " << ssTimeZone.str() << std::endl;
	m_sLogFile.flush();
	}
	nErr = setSiteLongitude(sLong);

	nErr |= setSiteLatitude(sLat);

	nErr |= setSiteTimezone(ssTimeZone.str());

	nErr |= syncDate();

	nErr |= syncTime();

	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] error " << nErr  << std::endl;
		m_sLogFile.flush();
	}
	}

	return nErr;
}

int OnStep::getSiteData(std::string &sLongitude, std::string &sLatitude, std::string &sTimeZone)
{
	int nErr = PLUGIN_OK;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	nErr = getSiteLongitude(sLongitude);
	nErr |= getSiteLatitude(sLatitude);
	nErr |= getSiteTZ(sTimeZone);
	return nErr;
}

void OnStep::setSyncLocationDataConnect(bool bSync)
{
	m_bSyncLocationDataConnect = bSync;
}

// --- Time and Date ---

int OnStep::getLocalTime(std::string &sTime)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	nErr = sendCommand(":GLH#", sResp);
	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] error " << nErr << ", response : " << sResp << std::endl;
		m_sLogFile.flush();
	}
		return nErr;
	}
	if(sResp.size() == 0)
		return ERR_CMDFAILED;
	sTime.assign(sResp);

	return nErr;
}

int OnStep::getLocalDate(std::string &sDate)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	nErr = sendCommand(":GC#", sResp);
	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] error " << nErr << ", response : " << sResp << std::endl;
		m_sLogFile.flush();
	}
		return nErr;
	}
	if(sResp.size() == 0)
		return ERR_CMDFAILED;
	sDate.assign(sResp);
	return nErr;
}

void OnStep::convertDecDegToDDMMSS(double dDeg, std::string &sResult)
{
	int DD, MM, SS;
	double mm, ss;
	double dNewDeg;
	std::stringstream ssTmp;
	char cSign;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	sResult.clear();
	// convert dDeg decimal value to sDD:MM:SS
	dNewDeg = std::fabs(dDeg);
	cSign = dDeg>=0?'+':'-';
	DD = int(dNewDeg);
	mm = dNewDeg - DD;
	MM = int(mm*60);
	ss = (mm*60) - MM;
	SS = int(std::round(ss*60));

	ssTmp << cSign << DD << "*" << std::setfill('0') << std::setw(2) << MM << ":" << std::setfill('0') << std::setw(2) << SS;
	sResult.assign(ssTmp.str());
}

void OnStep::convertDecAzToDDMMSSs(double dDeg, std::string &sResult)
{
	int DD, MM;
	double mm, ss, SS;
	double dNewDeg;
	std::stringstream ssTmp;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	sResult.clear();
	// convert dDeg decimal value to DDD*MM:SS.SSS
	dNewDeg = std::fabs(dDeg);
	DD = int(dNewDeg);
	mm = dNewDeg - DD;
	MM = int(mm*60);
	ss = (mm*60) - MM;
	SS = ss*60;

	ssTmp << std::setfill('0') << std::setw(3) << DD << "*" << std::setfill('0') << std::setw(2) << MM << ":" << std::setfill('0') << std::setw(2) << std::fixed << std::setprecision(0) << SS;
	sResult.assign(ssTmp.str());
}

void OnStep::convertDecDegToDDMMSS_ForDecl(double dDeg, std::string &sResult)
{
	int DD, MM;
	double mm, ss, SS;
	double dNewDeg;
	char cSign;
	std::stringstream ssTmp;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	sResult.clear();
	// convert dDeg decimal value to sDD*MM:SS.SSS
	dNewDeg = std::fabs(dDeg);
	cSign = dDeg>=0?'+':'-';
	DD = int(dNewDeg);
	mm = dNewDeg - DD;
	MM = int(mm*60);
	ss = (mm*60) - MM;
	SS = ss*60;

	ssTmp << cSign << std::setfill('0') << std::setw(2) << DD << "*" << std::setfill('0') << std::setw(2) << MM << ":" << std::setfill('0') << std::setw(6) << std::fixed << std::setprecision(3)<< SS;
	sResult.assign(ssTmp.str());
}

void OnStep::convertDecDegToDDMMSS_ForAlt(double dAlt, std::string &sResult)
{
	int DD, MM;
	double mm, ss, SS;
	double dNewAlt;
	char cSign;
	std::stringstream ssTmp;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	sResult.clear();
	// convert dDeg decimal value to sDD:MM:SS
	dNewAlt = std::fabs(dAlt);
	cSign = dNewAlt>=0?'+':'-';
	DD = int(dNewAlt);
	mm = dNewAlt - DD;
	MM = int(mm*60);
	ss = (mm*60) - MM;
	SS = ss*60;

	ssTmp << cSign << std::setfill('0') << std::setw(2) << DD << "*" << std::setfill('0') << std::setw(2) << MM << ":" << std::setfill('0') << std::setw(2) << std::fixed << std::setprecision(0)<< SS;
	sResult.assign(ssTmp.str());
}


int OnStep::convertDDMMSSToDecDeg(const std::string sStrDeg, double &dDecDeg)
{
	int nErr = PLUGIN_OK;
	std::vector<std::string> vFieldsData;
	std::string newDec;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	dDecDeg = 0;
	// dec is in a weird format.
	newDec.assign(sStrDeg);

	std::replace(newDec.begin(), newDec.end(), '*', ':' );
	std::replace(newDec.begin(), newDec.end(), '\'', ':' );
	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] newDec = " << newDec << std::endl;
	m_sLogFile.flush();
	}

	nErr = parseFields(newDec, vFieldsData, ':');
	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] parseFields error " << nErr << std::endl;
		m_sLogFile.flush();
	}
		return nErr;
	}
	if(vFieldsData.size() >= 3) {
		try {
			dDecDeg = std::stod(vFieldsData[0]);
			if(dDecDeg <0) {
				dDecDeg = dDecDeg - std::stod(vFieldsData[1])/60.0 - std::stod(vFieldsData[2])/3600.0;
			}
			else {
				dDecDeg = dDecDeg + std::stod(vFieldsData[1])/60.0 + std::stod(vFieldsData[2])/3600.0;
			}
	if(m_nDebugLevel >= 2) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] dDecDeg = " << std::fixed << std::setprecision(12) << dDecDeg << std::endl;
			m_sLogFile.flush();
	}
		}
		catch(const std::exception& e) {
	if(m_nDebugLevel >= 1) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] conversion exception : " << e.what() << std::endl;
			m_sLogFile.flush();
	}
			return ERR_PARSE;
		}

	}
	else
		nErr = ERR_PARSE;

	return nErr;
}

void OnStep::convertRaToHHMMSSt(double dRa, std::string &sResult)
{
	int HH, MM;
	double hh, mm, SSt;
	std::stringstream ssTmp;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	sResult.clear();
	// convert Ra value to HH:MM:SS.SSSS
	HH = int(dRa);
	hh = dRa - HH;
	MM = int(hh*60);
	mm = (hh*60) - MM;
	SSt = mm * 60;

	ssTmp << std::setfill('0') << std::setw(2) << HH << ":" << std::setfill('0') << std::setw(2) << MM << ":" << std::setfill('0') << std::setw(7) << std::fixed << std::setprecision(4) << SSt;
	sResult.assign(ssTmp.str());
}


int OnStep::convertHHMMSStToRa(const std::string szStrRa, double &dRa)
{
	int nErr = PLUGIN_OK;
	std::vector<std::string> vFieldsData;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] szStrRa = '" <<  szStrRa << "'" << std::endl;
	m_sLogFile.flush();
	}

	dRa = 0;

	nErr = parseFields(szStrRa, vFieldsData, ':');
	if(nErr)
		return nErr;

	if(vFieldsData.size() >= 3) {
		try {
			dRa = std::stod(vFieldsData[0]) + std::stod(vFieldsData[1])/60.0 + std::stod(vFieldsData[2])/3600.0;
	if(m_nDebugLevel >= 2) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] dRa = " << std::fixed << std::setprecision(12) << dRa << std::endl;
			m_sLogFile.flush();
	}
		}
		catch(const std::exception& e) {
	if(m_nDebugLevel >= 1) {
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] conversion exception : " << e.what() << std::endl;
			m_sLogFile.flush();
	}
			return ERR_PARSE;
		}
	}
	else
		nErr = ERR_PARSE;

	return nErr;
}


int OnStep::IsBeyondThePole(bool &bBeyondPole)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< "[" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}

	bBeyondPole = false;

	nErr = sendCommand(":Gm#", sResp);
	if(nErr) {
	if(m_nDebugLevel >= 1) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] error " << nErr << ", response : " << sResp << std::endl;
		m_sLogFile.flush();
	}
	}

	// “beyond the pole” =  “telescope west of the pier”,
	if(sResp.find("W") != std::string::npos)
		bBeyondPole = true;

	return nErr;
}

void OnStep::setStopTrackingOnDisconnect(bool bStop)
{
	m_bStopTrackingOnDisconnect = bStop;
}


// --- Parse result ---
int OnStep::parseFields(const std::string sIn, std::vector<std::string> &svFields, char cSeparator)
{
	int nErr = PLUGIN_OK;
	std::string sSegment;
	std::stringstream ssTmp(sIn);

	if(m_nDebugLevel >= 2) {
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] Called." << std::endl;
	m_sLogFile.flush();
	}
	if(sIn.size() == 0)
		return ERR_PARSE;

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

std::string& OnStep::trim(std::string &str, const std::string& filter )
{
	return ltrim(rtrim(str, filter), filter);
}

std::string& OnStep::ltrim(std::string& str, const std::string& filter)
{
	str.erase(0, str.find_first_not_of(filter));
	return str;
}

std::string& OnStep::rtrim(std::string& str, const std::string& filter)
{
	str.erase(str.find_last_not_of(filter) + 1);
	return str;
}

void OnStep::setDebugLevel(int nLevel)
{
	if(nLevel > 0 && !m_sLogFile.is_open()) {
		m_sLogFile.open(m_sLogfilePath, std::ios::out | std::ios::trunc);
	}
	else if(nLevel == 0 && m_sLogFile.is_open()) {
		m_sLogFile.close();
	}
	m_nDebugLevel = nLevel;
	if(m_nDebugLevel >= 2 && m_sLogFile.is_open()) {
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setDebugLevel] Debug level set to " << m_nDebugLevel << std::endl;
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [OnStep] Version " << std::fixed << std::setprecision(2) << PLUGIN_VERSION << " build " << __DATE__ << " " << __TIME__ << std::endl;
		m_sLogFile.flush();
	}
}

void OnStep::log(std::string sLogEntry)
{
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [" << __func__ << "] " << sLogEntry << std::endl;
	m_sLogFile.flush();
}

const std::string OnStep::getTimeStamp()
{
	time_t     now = time(0);
	struct tm  tstruct;
	char       buf[80];
	tstruct = *localtime(&now);
	std::strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

	return buf;
}

