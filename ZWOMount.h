#ifndef __ZWOMount__
#define __ZWOMount__
#pragma once

#include "OnStep.h"

class ZWOMount : public OnStep
{
public:
	int Connect(std::string sPort) override;
	int getLimits(double &dHoursEast, double &dHoursWest) override;
	int getflipHourAngle(double &dHourAngle) override;
	int isAligned(bool &bAligned) override;
	int gotoPark() override;
	int isParkingComplete(bool &bComplete) override;
	int unPark() override;
	int isUnparkDone(bool &bComplete) override;
	int getDeviceName(std::string &sName) override;
	int getAtPark(bool &bParked) override;
	int isTrackingOn(bool &bTrackOn) override;
	int gotoParkPos(double dAlt, double dAz) override;
	int setCurentPosAsPark() override;
	int getHeightLimits(bool &bEnabled, int &nUpperDeg, int &nLowerDeg) override;
	int setHeightLimits(bool bEnable, int nUpperDeg, int nLowerDeg) override;
	int getMeridianConfig(int &nTrackPastDeg, int &nSlewPastDeg) override;
	int setMeridianConfig(int nTrackPastDeg, int nSlewPastDeg) override;
	int getGuideRate(double &dRate) override;
	int setGuideRate(double dRate) override;
	int homeMount() override;
	int isHomingDone(bool &bIsHomed) override;
	int getRateName(int nZeroBasedIndex, std::string &sOut) override;
	int startSlewTo(double dRa, double dDec) override;
};

#endif // __ZWOMount__
