#include "OnStep.h"

// Constructor for OnStep
OnStep::OnStep()
{

	m_bIsConnected = false;
	m_bLimitCached = false;
	m_dHoursEast = 8.0;
	m_dHoursWest = 8.0;

	m_dRaRateArcSecPerSec = 0.0;
	m_dDecRateArcSecPerSec = 0.0;

	m_dParkAz = 270.00;
	m_dParkAlt = 0.00;

	m_dAlt = 0.00;
	m_dAz = 270.00;
	m_dRa = 0;
	m_dDec = 0;

	m_bSyncLocationDataConnect = false;
	m_bHomeOnUnpark = false;

	m_bSyncDone = false;
	m_bIsAtHome = false;
	m_bIsParked = true;
	m_bIsParking = false;
	m_bIsSlewing = false;
    m_bStopTrackingOnDisconnect = true;
    
    m_commandDelayTimer.Reset();


#ifdef PLUGIN_DEBUG
#if defined(WIN32)
	m_sLogfilePath = getenv("HOMEDRIVE");
	m_sLogfilePath += getenv("HOMEPATH");
	m_sLogfilePath += "\\OnStepLog.txt";
#else
	m_sLogfilePath = getenv("HOME");
	m_sLogfilePath += "/OnStepLog.txt";
#endif
	m_sLogFile.open(m_sLogfilePath, std::ios::out |std::ios::trunc);
#endif

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [OnStep] Version " << std::fixed << std::setprecision(2) << PLUGIN_VERSION << " build " << __DATE__ << " " << __TIME__ << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [OnStep] Constructor Called." << std::endl;
	m_sLogFile.flush();
#endif
}


OnStep::~OnStep(void)
{
#ifdef    PLUGIN_DEBUG
	// Close LogFile
	if(m_sLogFile.is_open())
		m_sLogFile.close();
#endif
}

int OnStep::Connect(std::string sPort)
{
	std::string sResp;
	int nErr = PLUGIN_OK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] Connect Called." << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] Trying to connect to port " << sPort<< std::endl;
	m_sLogFile.flush();
#endif

	// 9600 8N1
	if(m_pSerx->open(sPort.c_str(), 9600, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1") == 0)
		m_bIsConnected = true;
	else
		m_bIsConnected = false;
	/*
	 // reconnect at 56.7K
	 nErr = sendCommand("SB1#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1); // this command doesn't follow the usual format and doesn't end with #
	 std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command
	 m_pSerx->flushTx();
	 m_pSerx->purgeTxRx();
	 m_pSerx->close();
	 if(m_pSerx->open(sPort.c_str(), 56700, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1") == 0)
	 m_bIsConnected = true;
	 else
	 m_bIsConnected = false;
	 */


	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

	if(m_bSyncLocationDataConnect) {
		nErr = setSiteData(m_pTsx->longitude(),
						   m_pTsx->latitude(),
						   m_pTsx->timeZone());
	}
	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] error " << nErr << ", response = " << sResp << std::endl;
		m_sLogFile.flush();
#endif
		m_bIsConnected = false;
		return nErr;
	}
	m_bSyncDone = false;
	isHomingDone(m_bIsAtHome);

	return SB_OK;
}


int OnStep::Disconnect(void)
{
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Disconnect] Disconnect Called." << std::endl;
	m_sLogFile.flush();
#endif
	if (m_bIsConnected) {
		if(m_bStopTrackingOnDisconnect)
			setTrackingRates( false, true, 0.0, 0.0); // stop tracking on disconnect.
		if(m_pSerx){
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Disconnect] closing serial port." << std::endl;
			m_sLogFile.flush();
#endif
			m_pSerx->flushTx();
			m_pSerx->purgeTxRx();
			m_pSerx->close();
		}
	}
	m_bIsConnected = false;
	m_bSyncDone = false;

	return SB_OK;
}


#pragma mark - OnStep communication
int OnStep::sendCommand(const std::string sCmd, std::string &sResp, int nTimeout, char cEndOfResponse, int nExpectedResLen)
{
	int nErr = PLUGIN_OK;
	unsigned long  ulBytesWrite;
	std::vector<std::string> vFieldsData;

	m_pSerx->purgeTxRx();
	sResp.clear();

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [sendCommand] sending '" << sCmd << "'" << std::endl;
	m_sLogFile.flush();
#endif

	nErr = m_pSerx->writeFile((void *)sCmd.c_str(), sCmd.size(), ulBytesWrite);
	m_pSerx->flushTx();
	if(nErr)
		return nErr;

	// read response
	if(nTimeout == 0) // no response expected
		return nErr;
	// no response expected
	if(cEndOfResponse == SHORT_RESPONSE && nExpectedResLen==0)
		return nErr;

	nErr = readResponse(sResp, nTimeout, cEndOfResponse, nExpectedResLen);
	if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [sendCommand] ***** ERROR READING RESPONSE **** error = " << nErr << " , response : '" << sResp << "'" << std::endl;
		m_sLogFile.flush();
#endif
		return nErr;
	}
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [sendCommand] response : '" << sResp << "'" <<  std::endl;
	m_sLogFile.flush();
#endif

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
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] nBytesWaiting      : " << nBytesWaiting << std::endl;
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] nBytesWaiting nErr : " << nErr << std::endl;
		m_sLogFile.flush();
#endif
		if(!nBytesWaiting) {
			nbTimeouts += MAX_READ_WAIT_TIMEOUT;
			if(nbTimeouts >= nTimeout) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
				m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] bytesWaitingRx timeout, no data for " << nbTimeouts << " ms"<< std::endl;
				m_sLogFile.flush();
#endif
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
#if defined PLUGIN_DEBUG
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] readFile error : " << nErr << std::endl;
			m_sLogFile.flush();
#endif
			return nErr;
		}

		if (ulBytesRead != nBytesWaiting) { // timeout
#if defined PLUGIN_DEBUG
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] rreadFile Timeout Error." << std::endl;
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] readFile nBytesWaiting : " << nBytesWaiting << std::endl;
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] readFile ulBytesRead   : " << ulBytesRead << std::endl;
			m_sLogFile.flush();
#endif
		}

		ulTotalBytesRead += ulBytesRead;
		pszBufPtr+=ulBytesRead;
		// respnse not ending with the normal end of response char.
		if(cEndOfResponse == SHORT_RESPONSE && ulTotalBytesRead == nExpectedResLen)
			break;

	}  while (ulTotalBytesRead < SERIAL_BUFFER_SIZE  && *(pszBufPtr-1) != '#');

	if(!ulTotalBytesRead) {
		nErr = COMMAND_TIMEOUT; // we didn't get an answer.. so timeout
	}
	else if(*(pszBufPtr-1) == '#')
		*(pszBufPtr-1) = 0; //remove the #

	if(ulTotalBytesRead)
		sResp.assign(pszBuf);
	else
		sResp.clear();

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] sResp : '" << sResp << "'" << std::endl;
	m_sLogFile.flush();
#endif

	return nErr;
}

int OnStep::getFirmwareVersion(std::string &sFirmware)
{
	int nErr = PLUGIN_OK;
	std::string sResp;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getFirmwareVersion] Called." << std::endl;
	m_sLogFile.flush();
#endif

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
	unsigned long nSize;
	nErr = sendCommand(":GU#", sStatus);
	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getStatus] ERROR : " << nErr << " , sResp : " << sStatus << std::endl;
		m_sLogFile.flush();
#endif
	}

	// setting some default
	m_nTrackRate = SIDEREAL;
	m_bIsTracking = true;
	m_bIsSlewing = true;
	m_bIsParked = false;
	m_bIsParking = false;
	m_bIsHoming = false;
	m_bIsAtHome = false;

	nSize = sStatus.size();
#if defined PLUGIN_DEBUG
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getStatus] sStatus (nSize) : " << sStatus << " (" << nSize << ")"<<std::endl;
	m_sLogFile.flush();
#endif
	if(nSize) {
		while(nIndex < nSize) {
			switch(sStatus.at(nIndex++)) {
				case 'n':
					m_bIsTracking = false;
					continue;
				case 'N':
					m_bIsSlewing = false;
					continue;
				case 'p':
					m_bIsParked = false;
					continue;
				case 'P':
					m_bIsParked = true;
					continue;
				case 'I':
					m_bIsParking = true;
					continue;
				case 'h':
					m_bIsHoming = true;
					continue;
				case 'H':
					m_bIsAtHome = true;
					continue;
				case '(':
					m_nTrackRate = LUNAR;
					continue;
				case 'O':
					m_nTrackRate = SOLAR;
					continue;
				case 'k':
					m_nTrackRate = KING;
					continue;
				case 'T':
					m_nSideOfPier = EAST;
					continue;
				case 'W':
					m_nSideOfPier = EAST;
					continue;
				default:
					continue;
			}
		}
	}

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getStatus] m_bIsTracking : " << (m_bIsTracking?"Yes":"No") << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getStatus] m_bIsSlewing  : " << (m_bIsSlewing?"Yes":"No") << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getStatus] m_bIsParked 	 : " << (m_bIsParked?"Yes":"No") << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getStatus] m_bIsParking  : " << (m_bIsParking?"Yes":"No") << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getStatus] m_bIsAtHome   : " << (m_bIsAtHome?"Yes":"No") << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getStatus] m_bIsHoming   : " << (m_bIsAtHome?"Yes":"No") << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getStatus] m_nTrackRate  : " << m_nTrackRate << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getStatus] m_nSideOfPier : " << m_nSideOfPier << std::endl;
	m_sLogFile.flush();
#endif


	return nErr;
}

#pragma mark - Mount Coordinates
int OnStep::getRaAndDec(double &dRa, double &dDec)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRaAndDec] Called." << std::endl;
	m_sLogFile.flush();
#endif

	// get RA
	nErr = sendCommand(":GRH#", sResp);
	if(nErr) {
		// retry
		nErr = sendCommand(":GRH#", sResp);
		if(nErr) {
#if defined PLUGIN_DEBUG
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRaAndDec] :GR# ERROR : " << nErr << " , sResp : " << sResp << std::endl;
			m_sLogFile.flush();
#endif
			dRa = m_dRa;
			dDec = m_dDec;
			return PLUGIN_OK;
		}
	}

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRaAndDec]  sResp : " << sResp << std::endl;
	m_sLogFile.flush();
#endif
	if(sResp.size() == 0)
		return ERR_CMDFAILED;

	nErr = convertHHMMSStToRa(sResp, dRa);
	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRaAndDec] :GR# convertHHMMSStToRa error : " << nErr << " , sResp : " << sResp << std::endl;
		m_sLogFile.flush();
#endif
		dRa = m_dRa;
		dDec = m_dDec;
		return PLUGIN_OK;
	}
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRaAndDec]  dRa : " << std::fixed << std::setprecision(12) << dRa << std::endl;
	m_sLogFile.flush();
#endif
	m_dRa = dRa;

	std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command
	// get DEC
	nErr = sendCommand(":GDH#", sResp);
	if(nErr) {
		// retry
		nErr = sendCommand(":GDH#", sResp);
		if(nErr) {
#if defined PLUGIN_DEBUG
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRaAndDec] :GD# ERROR : " << nErr << " , sResp : " << sResp << std::endl;
			m_sLogFile.flush();
#endif
			dRa = m_dRa;
			dDec = m_dDec;
			return PLUGIN_OK;
		}
	}
	if(sResp.size() == 0)
		return ERR_CMDFAILED;

	nErr = convertDDMMSSToDecDeg(sResp, dDec);
	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRaAndDec] :GD# convertDDMMSSToDecDeg error : " << nErr << " , sResp : " << sResp << std::endl;
		m_sLogFile.flush();
#endif
		dRa = m_dRa;
		dDec = m_dDec;
		return PLUGIN_OK;
	}

	m_dDec = dDec;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRaAndDec] dDec : " << std::fixed << std::setprecision(12) << dDec << std::endl;
	m_sLogFile.flush();
#endif

	return nErr;
}

int OnStep::getAltAndAz(double &dAlt, double &dAz)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getAltAndAz] Called." << std::endl;
	m_sLogFile.flush();
#endif

	// get Az
	nErr = sendCommand(":GZH#", sResp);
	if(nErr) {
		// retry
		nErr = sendCommand(":GZH#", sResp);
		if(nErr) {
#if defined PLUGIN_DEBUG
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getAltAndAz] :GZ# ERROR : " << nErr << " , sResp : " << sResp << std::endl;
			m_sLogFile.flush();
#endif
			dAlt = m_dAlt;
			dAz = m_dAz;
			return PLUGIN_OK;
		}
	}

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getAltAndAz]  sResp : " << sResp << std::endl;
	m_sLogFile.flush();
#endif
	if(sResp.size() == 0)
		return ERR_CMDFAILED;

	nErr = convertDDMMSSToDecDeg(sResp, dAz);
	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getAltAndAz] :GZ# convertDDMMSSToDecDeg error : " << nErr << " , sResp : " << sResp << std::endl;
		m_sLogFile.flush();
#endif
		dAlt = m_dAlt;
		dAz = m_dAz;
		return PLUGIN_OK;
	}

	m_dAz = dAz;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getAltAndAz]  dAz : " << std::fixed << std::setprecision(12) << dAz << std::endl;
	m_sLogFile.flush();
#endif

	std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command

	// get Alt
	nErr = sendCommand(":GAH#", sResp);
	if(nErr) {
		// retry
		nErr = sendCommand(":GAH#", sResp);
		if(nErr) {
#if defined PLUGIN_DEBUG
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getAltAndAz] :GA# ERROR : " << nErr << " , sResp : " << sResp << std::endl;
			m_sLogFile.flush();
#endif
			dAlt = m_dAlt;
			dAz = m_dAz;
			return PLUGIN_OK;
		}
	}
	if(sResp.size() == 0)
		return ERR_CMDFAILED;
	nErr = convertDDMMSSToDecDeg(sResp, dAlt);
	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getAltAndAz] :GA# convertDDMMSSToDecDeg error : " << nErr << " , sResp : " << sResp << std::endl;
		m_sLogFile.flush();
#endif
		dAlt = m_dAlt;
		dAz = m_dAz;
		return PLUGIN_OK;
	}

	m_dAlt = dAlt;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getAltAndAz] dAlt : " << std::fixed << std::setprecision(12) << dAlt << std::endl;
	m_sLogFile.flush();
#endif

	return nErr;

}


int OnStep::setTarget(double dRa, double dDec)
{
	int nErr;
	std::stringstream ssTmp;
	std::string sResp;
	std::string sTemp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTarget] Ra  : " << std::fixed << std::setprecision(8) << dRa << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTarget] Dec : " << std::fixed << std::setprecision(8) << dDec << std::endl;
	m_sLogFile.flush();
#endif

	// convert Ra value to HH:MM:SS.SSSS before passing them to OnStep
	convertRaToHHMMSSt(dRa, sTemp);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTarget] Converted Ra : " << sTemp << std::endl;
	m_sLogFile.flush();
#endif
	// set target Ra
	ssTmp<<":Sr"<<sTemp<<"#";
	nErr = sendCommand(ssTmp.str(), sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1); // this command doesn't follow the usual format and doesn't end with #
	std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command
	if(sResp.size() && sResp.at(0)=='1') {
		nErr = PLUGIN_OK;
	}
	else if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTarget] Error setting target Ra, response : " << sResp << std::endl;
		m_sLogFile.flush();
#endif
		return nErr;
	}

	// convert target dec to sDD*MM:SS.SSS
	convertDecDegToDDMMSS_ForDecl(dDec, sTemp);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTarget] Converted Dec : " <<sTemp << std::endl;
	m_sLogFile.flush();
#endif
	std::stringstream().swap(ssTmp);
	// set target Dec
	ssTmp<<":Sd"<<sTemp<<"#";
	nErr = sendCommand(ssTmp.str(), sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1); // this command doesn't follow the usual format and doesn't end with #
	std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command
	if(sResp.size() && sResp.at(0)=='1')
		nErr = PLUGIN_OK;
	else if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTarget] Error setting target Dec, response : " << sResp << std::endl;
		m_sLogFile.flush();
#endif
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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTargetAltAz] Az  : " << std::fixed << std::setprecision(8) << dAz << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTargetAltAz] Alt : " << std::fixed << std::setprecision(8) << dAlt << std::endl;
	m_sLogFile.flush();
#endif

	// convert Az value to DDD*MM:SS.S
	convertDecAzToDDMMSSs(dAz, sTemp);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTargetAltAz] szTemp(Az)  : " << sTemp << std::endl;
	m_sLogFile.flush();
#endif
	// set target Az
	ssTmp<<":Sz"<<sTemp<<"#";
	nErr = sendCommand(ssTmp.str(), sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1); // this command doesn't follow the usual format and doesn't end with #
	std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command
	if(nErr)
		return nErr;


	// convert Alt value sDD*MM:SS.SSS
	convertDecDegToDDMMSS_ForDecl(dAlt, sTemp);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTargetAltAz]  szTemp(Alt)  : " <<sTemp << std::endl;
	m_sLogFile.flush();
#endif
	// set target Alt
	std::stringstream().swap(ssTmp);
	ssTmp<<":Sa"<<sTemp<<"#";
	nErr = sendCommand(ssTmp.str(), sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1); // this command doesn't follow the usual format and doesn't end with #
	std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command
	if(nErr)
		return nErr;

	return nErr;
}

#pragma mark - Sync and Cal
int OnStep::syncTo(double dRa, double dDec)
{
	int nErr = PLUGIN_OK;
	std::stringstream ssTmp;
	std::string sResp;
	char cSign;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [syncTo]  Ra Hours   : " << std::fixed << std::setprecision(5) << dRa << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [syncTo]  Ra Degrees : " << std::fixed << std::setprecision(5) << dRa*15.0 << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [syncTo]  Dec        : " << std::fixed << std::setprecision(5) << dDec << std::endl;
	m_sLogFile.flush();
#endif


	if(dDec <0) {
		cSign = '-';
		dDec = -dDec;
	} else {
		cSign = '+';
	}

	// TODO
	// setTarget
	// sync to target

	if (!m_bSyncDone) {

	}

	return nErr;
}

int OnStep::isAligned(bool &bAligned)
{
	int nErr = PLUGIN_OK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isAligned] Called." << std::endl;
	m_sLogFile.flush();
#endif
	// for now
	bAligned = true;
	return nErr;
}

#pragma mark - tracking rates
int OnStep::setTrackingRates(bool bSiderialTrackingOn, bool bIgnoreRates, double dRaRateArcSecPerSec, double dDecRateArcSecPerSec)
{
	int nErr = PLUGIN_OK;
	std::string sResp;
	std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTrackingRates] Called." << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTrackingRates] bSiderialTrackingOn  : " << (bSiderialTrackingOn?"Yes":"No") << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTrackingRates] bIgnoreRates         : " << (bIgnoreRates?"Yes":"No") << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTrackingRates] dRaRateArcSecPerSec  : " << std::fixed << std::setprecision(8) << dRaRateArcSecPerSec << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTrackingRates] dDecRateArcSecPerSec : " << std::fixed << std::setprecision(8) << dDecRateArcSecPerSec << std::endl;
	m_sLogFile.flush();
#endif
	// stop tracking
	if(!bSiderialTrackingOn && bIgnoreRates) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTrackingRates] setting to stopped" << std::endl;
		m_sLogFile.flush();
#endif
		m_dRaRateArcSecPerSec = 15.0410681;
		m_dDecRateArcSecPerSec = 0.0;
		nErr = sendCommand(":Td#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1); // tracking off
	}
	// sidereal
	else if(bSiderialTrackingOn && bIgnoreRates) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTrackingRates] setting to Sidereal" << std::endl;
		m_sLogFile.flush();
#endif
		m_dRaRateArcSecPerSec = 0.0;
		m_dDecRateArcSecPerSec = 0.0;
		nErr = sendCommand(":Te#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1); //tracking on
		nErr = sendCommand(":TQ#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 0); // Sidereal rate
	}
	// Lunar
	else if (0.30 < dRaRateArcSecPerSec && dRaRateArcSecPerSec < 0.83 && -0.25 < dDecRateArcSecPerSec && dDecRateArcSecPerSec < 0.25) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTrackingRates] setting to Lunar" << std::endl;
		m_sLogFile.flush();
#endif
		m_dRaRateArcSecPerSec = dRaRateArcSecPerSec;
		m_dDecRateArcSecPerSec = dDecRateArcSecPerSec;
		nErr = sendCommand(":Te#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1); //tracking on
		nErr = sendCommand(":TL#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 0); // Lunar rate
	}
	// solar
	else if (0.037 < dRaRateArcSecPerSec && dRaRateArcSecPerSec < 0.043 && -0.017 < dDecRateArcSecPerSec && dDecRateArcSecPerSec < 0.017) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTrackingRates] setting to Solar" << std::endl;
		m_sLogFile.flush();
#endif
		m_dRaRateArcSecPerSec = dRaRateArcSecPerSec;
		m_dDecRateArcSecPerSec = dDecRateArcSecPerSec;
		nErr = sendCommand(":Te#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1); //tracking on
		nErr = sendCommand(":TS#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 0); // Solar rate
	}
	// default to sidereal
	else {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTrackingRates] default to sidereal" << std::endl;
		m_sLogFile.flush();
#endif
		m_dRaRateArcSecPerSec = 0.0;
		m_dDecRateArcSecPerSec = 0.0;
		nErr = sendCommand(":Te#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1); //tracking on
		nErr = sendCommand(":TQ#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 0); // Sidereal rate
	}

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	nErr = sendCommand(":GT#", sResp);
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTrackingRates] check current tracking value : " << sResp << std::endl;
	m_sLogFile.flush();
#endif


	return nErr;
}

int OnStep::getTrackRates(bool &bSiderialTrackingOn, double &dRaRateArcSecPerSec, double &dDecRateArcSecPerSec)
{
	int nErr = PLUGIN_OK;
	std::string sResp;
	bool bTrackingOn;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getTrackRates] Called." << std::endl;
	m_sLogFile.flush();
#endif
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
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getTrackRates] bSiderialTrackingOn  : " << (bSiderialTrackingOn?"Yes":"No") << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getTrackRates] dRaRateArcSecPerSec  : " << std::fixed << std::setprecision(8) << dRaRateArcSecPerSec << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getTrackRates] dDecRateArcSecPerSec : " << std::fixed << std::setprecision(8) << dDecRateArcSecPerSec << std::endl;
	m_sLogFile.flush();
#endif
	return nErr;
}


#pragma mark - Limits
int OnStep::getLimits(double &dHoursEast, double &dHoursWest)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getLimits] Called." << std::endl;
	m_sLogFile.flush();
#endif

	nErr = sendCommand(":GXEe#", sResp);
	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getLimits] :GXEe# ERROR : " << nErr << " , sResp : " << sResp << std::endl;
		m_sLogFile.flush();
#endif
	}

	dHoursEast = std::stod(sResp)/15.0;

	nErr = sendCommand(":GXEw#", sResp);
	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getLimits] :GXEw# ERROR : " << nErr << " , sResp : " << sResp << std::endl;
		m_sLogFile.flush();
#endif
	}

	dHoursWest = std::stod(sResp)/15.0;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getLimits] dHoursEast  : " << std::fixed << std::setprecision(8) << dHoursEast << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getLimits] dHoursWest  : " << std::fixed << std::setprecision(8) << dHoursWest << std::endl;
	m_sLogFile.flush();
#endif

	return nErr;

}

int OnStep::getflipHourAngle(double &dHourAngle)
{
	int nErr = PLUGIN_OK;
	std::string sResp;
	double dEast, dWest;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getflipHourAngle] Called." << std::endl;
	m_sLogFile.flush();
#endif

	nErr = sendCommand(":GXE9#", sResp);
	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getflipHourAngle] :GXE9# ERROR : " << nErr << " , sResp : " << sResp << std::endl;
		m_sLogFile.flush();
#endif
	}
	dEast = std::fabs(std::stod(sResp))/15.0;

	nErr = sendCommand(":GXEA#", sResp);
	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getflipHourAngle] :GXEA# ERROR : " << nErr << " , sResp : " << sResp << std::endl;
		m_sLogFile.flush();
#endif
	}
	dWest = std::fabs(std::stod(sResp))/15.0;

	dHourAngle = (dEast>dWest)?dWest:dEast; // we take the smallest one as TSX only has 1 value

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getflipHourAngle] dHourAngle  : " << std::fixed << std::setprecision(8) << dHourAngle << std::endl;
	m_sLogFile.flush();
#endif

	return nErr;
}

#pragma mark - Slew

int OnStep::setSlewRate(int nRate)
{
	int nErr;
	std::stringstream ssCmd;
	std::string sResp;

	if(nRate>(PLUGIN_NB_SLEW_SPEEDS-1))
		return COMMAND_FAILED;

	ssCmd << ":R" << nRate << "#";
	nErr = sendCommand(ssCmd.str(), sResp, MAX_TIMEOUT, SHORT_RESPONSE, 0);
	return nErr;
}

void OnStep::setGoToSlewRate(int nRate)
{
	m_nGoToSlewRate = nRate;
}

int OnStep::gsetGoToSlewRate()
{
	return m_nGoToSlewRate;
}

int OnStep::startSlewTo(double dRa, double dDec)
{
	int nErr = PLUGIN_OK;
	bool bAligned;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [startSlewTo] Called." << std::endl;
	m_sLogFile.flush();
#endif

	nErr = isAligned(bAligned);
	if(nErr)
		return nErr;

	// set sync target coordinate
	nErr = setTarget(dRa, dDec);
	if(nErr)
		return nErr;

	setSlewRate(m_nGoToSlewRate);
	nErr = slewTargetRA_DecEpochNow();
	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [startSlewTo] error " << nErr << std::endl;
		m_sLogFile.flush();
#endif

	}
	m_bIsSlewing = true;

	return nErr;
}

int OnStep::slewTargetRA_DecEpochNow()
{
	int nErr;
	std::string sResp;
	int nRespCode;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [slewTargetRA_DecEpochNow] Called." << std::endl;
	m_sLogFile.flush();
#endif

	nErr = sendCommand(":MS#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1); // this command doesn't follow the usual format and doesn't end with #
	if(nErr == COMMAND_TIMEOUT) // normal if the command succeed
		nErr = PLUGIN_OK;
	else if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [slewTargetRA_DecEpochNow] Error slewing, response : " << sResp << std::endl;
		m_sLogFile.flush();
#endif
		return ERR_CMDFAILED;
	}
	if(sResp.size()) {
		nRespCode = std::stoi(sResp);
		switch(nRespCode) {
			case 0:
				// all good
				break;

			case 1:
#if defined PLUGIN_DEBUG
				m_sLogFile << "["<<getTimeStamp()<<"]"<< " [slewTargetRA_DecEpochNow] Limit error below horizon."  << std::endl;
				m_sLogFile.flush();
#endif
				nErr = ERR_LX200DESTBELOWHORIZ;
				break;

			case 2:
#if defined PLUGIN_DEBUG
				m_sLogFile << "["<<getTimeStamp()<<"]"<< " [slewTargetRA_DecEpochNow] Limit error no object."  << std::endl;
				m_sLogFile.flush();
#endif
				nErr = ERR_NOOBJECTSELECTED;
				break;

			case 4:
#if defined PLUGIN_DEBUG
				m_sLogFile << "["<<getTimeStamp()<<"]"<< " [slewTargetRA_DecEpochNow] Limit error position unreachable."  << std::endl;
				m_sLogFile.flush();
#endif
				nErr = ERR_GEMINI_POSITION_UNREACHABLE;
				break;

			case 5:
#if defined PLUGIN_DEBUG
				m_sLogFile << "["<<getTimeStamp()<<"]"<< " [slewTargetRA_DecEpochNow] Limit error not aligned."  << std::endl;
				m_sLogFile.flush();
#endif
				nErr = ERR_GEMINI_NOT_ALIGNED;
				break;

			case 6:
#if defined PLUGIN_DEBUG
				m_sLogFile << "["<<getTimeStamp()<<"]"<< " [slewTargetRA_DecEpochNow] Limit error outside limits."  << std::endl;
				m_sLogFile.flush();
#endif
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
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getNbSlewRates] Called : PLUGIN_NB_SLEW_SPEEDS = " << PLUGIN_NB_SLEW_SPEEDS << std::endl;
	m_sLogFile.flush();
#endif
	return PLUGIN_NB_SLEW_SPEEDS;
}

// returns "Slew", "ViewVel4", "ViewVel3", "ViewVel2", "ViewVel1"
int OnStep::getRateName(int nZeroBasedIndex, std::string &sOut)
{
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRateName] Called." << std::endl;
	m_sLogFile.flush();
#endif

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
	m_nOpenLoopDir = Dir;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [startOpenLoopMove] setting dir to  : " << Dir << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [startOpenLoopMove] setting rate to : " << nRate << std::endl;
	m_sLogFile.flush();
#endif

	// select rate
	nErr = setSlewRate(nRate);

	m_nOpenLoopDir = Dir;

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
	nErr = sendCommand(sCmd, sResp, MAX_TIMEOUT, SHORT_RESPONSE, 0); // this command doesn't follow the usual format and doesn't end with #
	return nErr;
}

int OnStep::stopOpenLoopMove()
{
	int nErr = PLUGIN_OK;
	std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [stopOpenLoopMove] dir was  : " << m_nOpenLoopDir << std::endl;
	m_sLogFile.flush();
#endif

	switch(m_nOpenLoopDir){
		case MountDriverInterface::MD_NORTH:
			nErr = sendCommand(":Qn#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 0); // this command doesn't follow the usual format and doesn't end with #
			break;
		case MountDriverInterface::MD_SOUTH:
			nErr = sendCommand(":Qs#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 0); // this command doesn't follow the usual format and doesn't end with #
			break;
		case MountDriverInterface::MD_EAST:
			nErr = sendCommand(":Qe#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 0); // this command doesn't follow the usual format and doesn't end with #
			break;
		case MountDriverInterface::MD_WEST:
			nErr = sendCommand(":Qw#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 0); // this command doesn't follow the usual format and doesn't end with #
			break;
	}

	return nErr;
}


int OnStep::isSlewToComplete(bool &bComplete)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isSlewToComplete] Called." << std::endl;
	m_sLogFile.flush();
#endif

	bComplete = false;
	if(!m_bIsSlewing ) {
		bComplete = true;
		return nErr;
	}

	nErr = getStatus();
	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isSlewToComplete] getStatus error " << nErr << std::endl;
		m_sLogFile.flush();
#endif
		return nErr;
	}

	bComplete = m_bIsSlewing?false:true;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isSlewToComplete] m_bIsSlewing : " << (m_bIsSlewing?"Yes":"No") << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isSlewToComplete] bComplete    : " << (bComplete?"Yes":"No") << std::endl;
	m_sLogFile.flush();
#endif

	return nErr;
}

int OnStep::gotoPark(double dAlt, double dAz)
{
	int nErr = PLUGIN_OK;
	std::string sResp;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [gotoPark] Called." << std::endl;
	m_sLogFile.flush();
#endif

	m_bIsParking = false;

	nErr = sendCommand(":hP#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1);
	std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command

	if(!nErr)
		m_bIsParking = true;
	return nErr;
}

int OnStep::isParkingComplete(bool &bComplete)
{
	int nErr = PLUGIN_OK;

	nErr = getStatus();
	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isParkingComplete] getStatus error " << nErr << std::endl;
		m_sLogFile.flush();
#endif
		return nErr;
	}

	bComplete = m_bIsParking?false:true;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isParkingComplete] m_bIsParking : " << (m_bIsParking?"Yes":"No") << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isParkingComplete] bComplete    : " << (bComplete?"Yes":"No") << std::endl;
	m_sLogFile.flush();
#endif

	return nErr;
}

int OnStep::getAtPark(bool &bParked)
{
	int nErr = PLUGIN_OK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getAtPark] Called." << std::endl;
	m_sLogFile.flush();
#endif
	bParked = false;

	nErr = getStatus(); // will update the flags
	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getAtPark] getStatus error " << nErr << std::endl;
		m_sLogFile.flush();
#endif
		return nErr;
	}

	bParked = m_bIsParked;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getAtPark] bParked   " << (bParked?"Yes":"No") << std::endl;
	m_sLogFile.flush();
#endif

	return nErr;
}

int OnStep::unPark()
{
	int nErr = PLUGIN_OK;
	std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [unPark] Called." << std::endl;
	m_sLogFile.flush();
#endif

	nErr = sendCommand(":hR#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1);
	if(nErr) {
		m_bIsParked = true;
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [unPark] error " << nErr << std::endl;
		m_sLogFile.flush();
#endif
	}

	nErr = getStatus(); // will update the flags
	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [unPark] getStatus error " << nErr << std::endl;
		m_sLogFile.flush();
#endif
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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isUnparkDone] Called." << std::endl;
	m_sLogFile.flush();
#endif

	bComplete = false;
	nErr = getAtPark(bAtPArk);
	if(!bAtPArk)
		bComplete = true;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isUnparkDone] bAtPArk   " << (bAtPArk?"Yes":"No") << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isUnparkDone] bComplete " << (bComplete?"Yes":"No") << std::endl;
	m_sLogFile.flush();
#endif

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isUnparkDone] m_bIsAtHome   " << (m_bIsAtHome?"Yes":"No") << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isUnparkDone] bComplete " << (bComplete?"Yes":"No") << std::endl;
	m_sLogFile.flush();
#endif

	setTrackingRates(true, true, 0.0, 0.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(50)); // need to give time to the mount to process the command

	isTrackingOn(bTrackingOn);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isUnparkDone] bTrackingOn   " << (bTrackingOn?"Yes":"No") << std::endl;
	m_sLogFile.flush();
#endif

	m_bIsParked = false;
	return nErr;
}


int OnStep::homeMount()
{
	int nErr = PLUGIN_OK;
	std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [homeMount] Called." << std::endl;
	m_sLogFile.flush();
#endif
	if(m_bIsAtHome) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [homeMount] already homed." << std::endl;
		m_sLogFile.flush();
#endif
		return nErr;
	}

	nErr = sendCommand(":Ch#", sResp, 0);
	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [homeMount] error " << nErr << " , response :" << sResp << std::endl;
		m_sLogFile.flush();
#endif
	}

	return nErr;
}


int OnStep::isHomingDone(bool &bIsHomed)
{
	int nErr = PLUGIN_OK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isHomingDone] Called." << std::endl;
	m_sLogFile.flush();
#endif
	bIsHomed = false;

	nErr = getStatus(); // will update the flags
	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isHomingDone] getStatus error " << nErr << std::endl;
		m_sLogFile.flush();
#endif
		return nErr;
	}

	bIsHomed = m_bIsAtHome;
	return nErr;
}


int OnStep::isTrackingOn(bool &bTrackOn)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isTrackingOn] Called." << std::endl;
	m_sLogFile.flush();
#endif

	bTrackOn = false;
	nErr = getStatus(); // will update the flags
	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isTrackingOn] getStatus error " << nErr << std::endl;
		m_sLogFile.flush();
#endif
		return nErr;
	}

	bTrackOn = m_bIsTracking;

#if defined PLUGIN_DEBUG
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isTrackingOn] bTrackOn : " << (bTrackOn?"Yes":"No")<< std::endl;
	m_sLogFile.flush();
#endif

	return nErr;
}

int OnStep::Abort()
{
	int nErr = PLUGIN_OK;
	std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Abort] Called." << std::endl;
	m_sLogFile.flush();
#endif

	nErr = sendCommand(":Q#", sResp, 0);

	return nErr;
}

#pragma mark - time and site methods
int OnStep::syncTime()
{
	int nErr = PLUGIN_OK;
	int yy, mm, dd, h, min, dst;
	double sec;
	std::string sResp;
	std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [syncTime] Called." << std::endl;
	m_sLogFile.flush();
#endif

	m_pTsx->localDateTime(yy, mm, dd, h, min, sec, dst);

	ssTmp << ":SL" << std::setfill('0') << std::setw(2) << h << ":" << std::setfill('0') << std::setw(2) << min << ":" << std::setfill('0') << std::setw(2) << int(sec) << "#";
	nErr = sendCommand(ssTmp.str(), sResp, 0);
	std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command
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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [syncDate] Called." << std::endl;
	m_sLogFile.flush();
#endif

	m_pTsx->localDateTime(yy, mm, dd, h, min, sec, dst);
	// yy is actually yyyy, need conversion to yy, 2017 -> 17
	yy = yy - (int(yy / 1000) * 1000);

	ssTmp << ":SC" << std::setfill('0') << std::setw(2) << mm << "/" << std::setfill('0') << std::setw(2) << dd << "/" << std::setfill('0') << std::setw(2) << yy << "#";
	nErr = sendCommand(ssTmp.str(), sResp, 0);
	std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command
	getLocalDate(m_sDate);
	return nErr;
}

int OnStep::setSiteLongitude(const std::string sLongitude)
{
	int nErr = PLUGIN_OK;
	std::string sResp;
	std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteLongitude] Called." << std::endl;
	m_sLogFile.flush();
#endif

	// :SgsDDD*MM#
	ssTmp << ":Sg" << sLongitude << "#";
	nErr = sendCommand(ssTmp.str(), sResp, 0);
	std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command

	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteLongitude] error " << nErr << ", response : " << sResp << std::endl;
		m_sLogFile.flush();
#endif
	}

	return nErr;
}

int OnStep::setSiteLatitude(const std::string sLatitude)
{
	int nErr = PLUGIN_OK;
	std::string sResp;
	std::stringstream ssTmp;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteLatitude] Called." << std::endl;
	m_sLogFile.flush();
#endif

	// :StsDD*MM#
	ssTmp << ":St" << sLatitude << "#";
	nErr = sendCommand(ssTmp.str(), sResp, 0);
	std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command

	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteLatitude] error " << nErr << ", response : " << sResp << std::endl;
		m_sLogFile.flush();
#endif
	}

	return nErr;
}

int OnStep::setSiteTimezone(const std::string sTimezone)
{
	int nErr = PLUGIN_OK;
	std::string sResp;
	std::stringstream ssTmp;
	std::string sCurrentTimeZone;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteTimezone] Called." << std::endl;
	m_sLogFile.flush();
#endif
	ssTmp << ":SG" << sTimezone << "#";
	nErr = sendCommand(ssTmp.str(), sResp, 0);
	std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command

	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteTimezone] error " << nErr << ", response : " << sResp << std::endl;
		m_sLogFile.flush();
#endif
	}

	return nErr;
}

int OnStep::getSiteLongitude(std::string &sLongitude)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getSiteLongitude] Called." << std::endl;
	m_sLogFile.flush();
#endif

	nErr = sendCommand(":Gg#", sResp);
	if(!nErr) {
		sLongitude.assign(sResp);
	}

	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getSiteLongitude] error " << nErr << ", response : " << sResp << std::endl;
		m_sLogFile.flush();
#endif
	}

	return nErr;
}

int OnStep::getSiteLatitude(std::string &sLatitude)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getSiteLatitude] Called." << std::endl;
	m_sLogFile.flush();
#endif

	nErr = sendCommand(":Gt#", sResp);
	if(!nErr) {
		sLatitude.assign(sResp);
	}

	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getSiteLatitude] error " << nErr << ", response : " << sResp << std::endl;
		m_sLogFile.flush();
#endif
	}

	return nErr;
}

int OnStep::getSiteTZ(std::string &sTimeZone)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getSiteTZ] Called." << std::endl;
	m_sLogFile.flush();
#endif

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
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getSiteTZ] error " << nErr << ", response : " << sResp << std::endl;
		m_sLogFile.flush();
#endif
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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] Called." << std::endl;
	m_sLogFile.flush();
#endif


#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] dLongitude : " << std::fixed << std::setprecision(5) << dLongitude << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] dLatitute  : " << std::fixed << std::setprecision(5) << dLatitute << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] dTimeZone  : " << std::fixed << std::setprecision(2) << dTimeZone << std::endl;
	m_sLogFile.flush();
#endif

	convertDecDegToDDMMSS(dLongitude, sLong);
	convertDecDegToDDMMSS(dLatitute, sLat);

	m_pTsx->localDateTime(yy, mm, dd, h, min, sec, dst);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] dst        : " << (dst != 0 ?"Yes":"No") << std::endl;
	m_sLogFile.flush();
#endif

	if(dst) {
		dTimeZone += 1.0;
	}
	dTimeZoneNew = -dTimeZone;
	cSign = dTimeZoneNew>=0?'+':'-';
	dTimeZoneNew=std::fabs(dTimeZone);

	ssTimeZone << cSign << std::setfill('0') << std::setw(2) << dTimeZoneNew;

	sLong.assign(sLong);

	sLat.assign(sLat);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] sLong      : " << sLong << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] sLat       : " << sLat<< std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] ssTimeZone : " << ssTimeZone.str() << std::endl;
	m_sLogFile.flush();
#endif
	nErr = setSiteLongitude(sLong);
	std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command

	nErr |= setSiteLatitude(sLat);
	std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command

	nErr |= setSiteTimezone(ssTimeZone.str());
	std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command

	nErr |= syncDate();
	std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command

	nErr |= syncTime();
	std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command

	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] error " << nErr  << std::endl;
		m_sLogFile.flush();
#endif
	}

	return nErr;
}

int OnStep::getSiteData(std::string &sLongitude, std::string &sLatitude, std::string &sTimeZone)
{
	int nErr = PLUGIN_OK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getSiteData] Called." << std::endl;
	m_sLogFile.flush();
#endif

	nErr = getSiteLongitude(sLongitude);
	nErr |= getSiteLatitude(sLatitude);
	nErr |= getSiteTZ(sTimeZone);
	return nErr;
}

void OnStep::setSyncLocationDataConnect(bool bSync)
{
	m_bSyncLocationDataConnect = bSync;
}

#pragma mark  - Time and Date

int OnStep::getLocalTime(std::string &sTime)
{
	int nErr = PLUGIN_OK;
	std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getLocalTime] Called." << std::endl;
	m_sLogFile.flush();
#endif

	nErr = sendCommand(":GL#", sResp);
	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getLocalTime] error " << nErr << ", response : " << sResp << std::endl;
		m_sLogFile.flush();
#endif
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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getLocalDate] Called." << std::endl;
	m_sLogFile.flush();
#endif

	nErr = sendCommand(":GC#", sResp);
	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getLocalDate] error " << nErr << ", response : " << sResp << std::endl;
		m_sLogFile.flush();
#endif
		return nErr;
	}
	if(sResp.size() == 0)
		return ERR_CMDFAILED;
	sDate.assign(sResp);
	return nErr;
}

int OnStep::getInputVoltage(double &dVolts)
{
	int nErr;
	std::string sResp;

	dVolts = 0.0;

	nErr = sendCommand(":Cv#", sResp);
	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getInputVoltage] error " << nErr << ", response : " << sResp << std::endl;
		m_sLogFile.flush();
#endif
		return nErr;
	}

	try {
		if(sResp.size() == 0)
			return ERR_CMDFAILED;
		dVolts = std::stod(sResp);
	}
	catch(const std::exception& e) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getInputVoltage] conversion exception : " << e.what() << std::endl;
		m_sLogFile.flush();
#endif
		return ERR_PARSE;
	}



	return nErr;
}

void OnStep::convertDecDegToDDMMSS(double dDeg, std::string &sResult)
{
	int DD, MM, SS;
	double mm, ss;
	double dNewDeg;
	std::stringstream ssTmp;
	char cSign;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [convertDecDegToDDMMSS] Called." << std::endl;
	m_sLogFile.flush();
#endif

	sResult.clear();
	// convert dDeg decimal value to sDD:MM:SS
	dNewDeg = std::fabs(dDeg);
	cSign = dDeg>=0?'+':'-';
	DD = int(dNewDeg);
	mm = dNewDeg - DD;
	MM = int(mm*60);
	ss = (mm*60) - MM;
	SS = int(std::round(ss*60));

	ssTmp << cSign << DD << "*" << std::setfill('0') << std::setw(2) << MM << "'" << std::setfill('0') << std::setw(2) << SS;
	sResult.assign(ssTmp.str());
}

void OnStep::convertDecAzToDDMMSSs(double dDeg, std::string &sResult)
{
	int DD, MM;
	double mm, ss, SS;
	double dNewDeg;
	std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [convertDecAzToDDMMSSs] Called." << std::endl;
	m_sLogFile.flush();
#endif

	sResult.clear();
	// convert dDeg decimal value to DDD*MM:SS.SSS
	dNewDeg = std::fabs(dDeg);
	DD = int(dNewDeg);
	mm = dNewDeg - DD;
	MM = int(mm*60);
	ss = (mm*60) - MM;
	SS = ss*60;

	ssTmp << std::setfill('0') << std::setw(3) << DD << "*" << std::setfill('0') << std::setw(2) << MM << "'" << std::setfill('0') << std::setw(6) << std::fixed << std::setprecision(3) << SS;
	sResult.assign(ssTmp.str());
}

void OnStep::convertDecDegToDDMMSS_ForDecl(double dDeg, std::string &sResult)
{
	int DD, MM;
	double mm, ss, SS;
	double dNewDeg;
	char cSign;
	std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [convertDecDegToDDMMSS_ForDecl] Called." << std::endl;
	m_sLogFile.flush();
#endif

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

int OnStep::convertDDMMSSToDecDeg(const std::string sStrDeg, double &dDecDeg)
{
	int nErr = PLUGIN_OK;
	std::vector<std::string> vFieldsData;
	std::string newDec;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [convertDDMMSSToDecDeg] Called." << std::endl;
	m_sLogFile.flush();
#endif

	dDecDeg = 0;
	// dec is in a weird format.
	newDec.assign(sStrDeg);

	std::replace(newDec.begin(), newDec.end(), '*', ':' );
	std::replace(newDec.begin(), newDec.end(), '\'', ':' );
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [convertDDMMSSToDecDeg] newDec = " << newDec << std::endl;
	m_sLogFile.flush();
#endif

	nErr = parseFields(newDec, vFieldsData, ':');
	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [convertDDMMSSToDecDeg] parseFields error " << nErr << std::endl;
		m_sLogFile.flush();
#endif
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
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [convertDDMMSSToDecDeg] dDecDeg = " << std::fixed << std::setprecision(12) << dDecDeg << std::endl;
			m_sLogFile.flush();
#endif
		}
		catch(const std::exception& e) {
#if defined PLUGIN_DEBUG
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [convertDDMMSSToDecDeg] conversion exception : " << e.what() << std::endl;
			m_sLogFile.flush();
#endif
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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [convertRaToHHMMSSt] Called." << std::endl;
	m_sLogFile.flush();
#endif

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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [convertHHMMSStToRa] Called." << std::endl;
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [convertHHMMSStToRa] szStrRa = '" <<  szStrRa << "'" << std::endl;
	m_sLogFile.flush();
#endif

	dRa = 0;

	nErr = parseFields(szStrRa, vFieldsData, ':');
	if(nErr)
		return nErr;

	if(vFieldsData.size() >= 3) {
		try {
			dRa = std::stod(vFieldsData[0]) + std::stod(vFieldsData[1])/60.0 + std::stod(vFieldsData[2])/3600.0;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [convertDDMMSSToDecDeg] dRa = " << std::fixed << std::setprecision(12) << dRa << std::endl;
			m_sLogFile.flush();
#endif
		}
		catch(const std::exception& e) {
#if defined PLUGIN_DEBUG
			m_sLogFile << "["<<getTimeStamp()<<"]"<< " [convertHHMMSStToRa] conversion exception : " << e.what() << std::endl;
			m_sLogFile.flush();
#endif
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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [IsBeyondThePole] Called." << std::endl;
	m_sLogFile.flush();
#endif

	bBeyondPole = false;

	nErr = sendCommand(":Gm#", sResp);
	if(nErr) {
#if defined PLUGIN_DEBUG
		m_sLogFile << "["<<getTimeStamp()<<"]"<< " [IsBeyondThePole] error " << nErr << ", response : " << sResp << std::endl;
		m_sLogFile.flush();
#endif
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


#pragma mark - Parse result
int OnStep::parseFields(const std::string sIn, std::vector<std::string> &svFields, char cSeparator)
{
	int nErr = PLUGIN_OK;
	std::string sSegment;
	std::stringstream ssTmp(sIn);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [parseFields] Called." << std::endl;
	m_sLogFile.flush();
#endif
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

#ifdef PLUGIN_DEBUG
void OnStep::log(std::string sLogEntry)
{
	m_sLogFile << "["<<getTimeStamp()<<"]"<< " [log] " << sLogEntry << std::endl;
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
#endif
