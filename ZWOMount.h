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
};

#endif // __ZWOMount__
