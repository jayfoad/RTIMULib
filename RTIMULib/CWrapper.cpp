#include <pthread.h>
#include <semaphore.h>
#include <signal.h>

#include "RTIMULib.h"

namespace {

	bool started = false;
	bool stopping = false;

	pthread_mutex_t dataLock = PTHREAD_MUTEX_INITIALIZER;
	pthread_cond_t dataCond = PTHREAD_COND_INITIALIZER;
	bool dataValid = false;
	RTIMU_DATA data;

	struct Wrapper {
		RTIMUSettings *settings;
		RTIMU *imu;
		int interval; // in microseconds
		pthread_t thread;

		Wrapper() {
			settings = new RTIMUSettings("RTIMULib");
			imu = settings ? RTIMU::createIMU(settings) : 0;
			if (imu) {
				if (imu->IMUType() == RTIMU_TYPE_NULL || !imu->IMUInit()) {
					delete imu;
					imu = 0;
				}
			}
			if (imu) {
				interval = imu->IMUGetPollInterval() * 1000;

				sigset_t set;
				sigset_t old;
				if (sigfillset(&set) == 0
				    && pthread_sigmask(SIG_SETMASK, &set, &old) == 0) {
					if (pthread_create(&thread, 0, start, this) == 0)
						started = true;
					pthread_sigmask(SIG_SETMASK, &old, 0);
				}
			}

			if (!started) {
				// Clean up
				delete imu;
				delete settings;
			}
		}

		~Wrapper() {
			if (started) {
				stopping = true;
				pthread_join(thread, 0);
			}
		}

		static void *start(void *p) {
			static_cast<Wrapper *>(p)->run();
			return 0;
		}

		void run() {
			while (true) {
				while (imu->IMURead()) {
					pthread_mutex_lock(&dataLock);

					data = imu->getIMUData();

					dataValid = true;
					pthread_cond_signal(&dataCond);

					pthread_mutex_unlock(&dataLock);

					if (stopping)
						return;
				}

				usleep(interval);

				if (stopping)
					return;
			}
		}
	} wrapper;

} // anonymous namespace

#pragma GCC visibility push(default)

extern "C" int getData(RTIMU_DATA *pdata)
{
	if (!started || stopping)
		return false;

	pthread_mutex_lock(&dataLock);

	// Wait for the first read to complete.
	while (!dataValid)
		pthread_cond_wait(&dataCond, &dataLock);
	*pdata = data;

	pthread_mutex_lock(&dataLock);

	return true;
}

#pragma GCC visibility pop
