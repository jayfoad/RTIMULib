#include "RTIMULib.h"

#pragma GCC visibility push(default)

extern "C"
{
	RTIMU *IMUCreateAndInit(const char *productType)
	{
		RTIMUSettings *settings = new RTIMUSettings(productType);
		RTIMU *imu = RTIMU::createIMU(settings);
		if (imu && imu->IMUType() == RTIMU_TYPE_NULL)
		{
			delete imu;
			imu = 0;
		}
		if (imu && !imu->IMUInit())
		{
			delete imu;
			imu = 0;
		}
		return imu;
	}

	int IMUGetPollInterval(RTIMU *imu)
	{
		return imu->IMUGetPollInterval();
	}

	int IMURead(RTIMU *imu, RTIMU_DATA *data)
	{
		if (imu->IMURead())
		{
			*data = imu->getIMUData();
			return true;
		}
		return false;
	}
}

#pragma GCC visibility pop
