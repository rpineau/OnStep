#ifndef __ZWOMount__
#define __ZWOMount__
#pragma once

#include "OnStep.h"

class ZWOMount : public OnStep
{
public:
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
	int finalizepark() override;
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

	bool supportsDriverSlewsToParkPosition() const override { return false; }
	bool supportsFindHome() const override { return true; }
	bool supportsMotorStatus() const override { return true; }
	bool isZWOVariant() const override { return true; }

private:
	// Persistent parked flag — not reset by getStatus() which never sees 'P' in :GU# on ZWO.
	// Set true by finalizepark(), false by gotoPark() and unPark().
	// getAtPark() falls back to this when :Gps# returns empty.
	bool m_bZWOParked = false;
};

#endif // __ZWOMount__
