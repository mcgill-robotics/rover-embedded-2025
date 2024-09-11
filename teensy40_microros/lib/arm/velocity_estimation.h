#include <stdint.h>
#include "Arduino.h"

class velocity_estimation
{
public:
	void initialize_parameters(float ppr, float error, float timeScale, uint8_t skip, uint32_t delay);

	void update_readings(int32_t position, uint32_t timeStamp);

	int32_t get_last_position(void); // may be depreciated

	// float htt(uint32_t currentTime);

	typedef struct
	{
		int32_t _positions[64];
		uint32_t _timeStamps[64];
		float _vel[64];
		float _c;
		int k;
		int32_t lastPos;
		uint32_t lasttime;
		int32_t newPos;
		uint32_t newtime;
	} foaw_vars;

	float foaw(int best);

private:
	const uint8_t FILTER_SIZE = 16;

	const float _pi = 3.141592654;

	float _ppr;
	float _errorMargin;
	float _timeScale;
	uint8_t _SKIP;
	uint32_t _maxDelay;
	float _mult = 0.0f;
	uint32_t size = 32;

	float _coefficients[2];
	uint8_t _size;
	uint8_t _n;
	float _previousEvent;
	uint8_t _newEvent; // may not be necessary
	uint32_t _firstTime;
	int32_t _firstPos;
	float _pastValues[5];
	uint16_t _index;
	uint32_t _processTime;

	int _sortedIndex[5];

	foaw_vars _foaw;

	int32_t _positions[16];
	uint32_t _timeStamps[16];

	void _compare_swap(int a, int b);
};