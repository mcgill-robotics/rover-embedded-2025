#include "velocity_estimation.h"

void velocity_estimation::initialize_parameters(float ppr, float error, float timeScale, uint8_t skip, uint32_t delay)
{
	_ppr = ppr;
	_errorMargin = error;
	_timeScale = timeScale;
	_SKIP = skip;
	_maxDelay = delay;

	_n = 0;
	_size = 2;
	_firstTime = 0;
	_firstPos = 0;

	_foaw.newPos = 0;
	_foaw.newtime = 0;
	_foaw.lastPos = 0;
	_foaw.lasttime = 0;
	_foaw.k = 0;

	_mult = 360.0f * 1000000.0f / _ppr;

	for (uint8_t i = 0; i < 2; i++)
	{
		_coefficients[i] = 0.0;
	}

	for (uint8_t i = 0; i < 5; i++)
	{
		_pastValues[i] = 0.0;
		_sortedIndex[i] = i;
	}

	for (uint8_t i = 0; i < FILTER_SIZE; i++)
	{
		_positions[i] = 0;
		_timeStamps[i] = 0;
	}

	for (uint8_t i = 0; i < 32; i++)
	{
		_foaw._positions[i] = 0;
		_foaw._timeStamps[i] = 0;
	}
}

void velocity_estimation::update_readings(int32_t position, uint32_t timeStamp)
{
	_newEvent = 1;

	uint8_t count = _n % FILTER_SIZE;

	_foaw.newPos = position;
	_foaw.newtime = timeStamp;

	_n++;
}

int32_t velocity_estimation::get_last_position(void)
{
	return _positions[_n];
}

float velocity_estimation::foaw(int best)
{
	// As implemented by Steve Sinclair (https://github.com/radarsat1/FOAW/blob/master/foaw.c)
	// adapted to our data by Antoine Weill--Duflos
	int i, j, l;
	uint32_t bad;
	float b, p_mean, p_out, t_mean, t_out, dt;
	float velocity = 0.0;
	const float pos_err_max = 1.0;
	const int time_err_max = 5;
	const uint32_t max_outlier = 8;

	/* circular buffer */
	_foaw.k = (_foaw.k + 1) % size;
	_foaw._positions[_foaw.k] = _foaw.newPos;
	_foaw._timeStamps[_foaw.k] = _foaw.newtime;

	for (i = 1; i < size; i++) // size is n
	{
		// basically doing end fit instead of best fit -> why?
		dt = (float)(_foaw._timeStamps[_foaw.k] - _foaw._timeStamps[(_foaw.k - i + size) % size]);
		if (dt != 0)
		{

			if (best)
			{
				b = 0.0f;
				for (l = 0; l < (i + 1); l++)
				{
					b += (i - 2 * l) * _foaw._positions[(_foaw.k - l + size) % size];
				}
				b = 6.0f * (float)b / ((dt * (i + 1) * (i + 2)));
			}
			else
			{
				b = (float)(_foaw._positions[_foaw.k] - _foaw._positions[(_foaw.k - i + size) % size]) / dt;
			}
		}
		else
		{
			b = 0.0;
			continue;
		}

		bad = 0;
		for (j = 1; j < i; j++)
		{
			p_out = _foaw._positions[(_foaw.k - j + size) % size];
			t_out = _foaw._timeStamps[(_foaw.k - j + size) % size];
			p_mean = _foaw._positions[_foaw.k] + b * (t_out - _foaw._timeStamps[_foaw.k]);
			t_mean = (p_out - _foaw._positions[_foaw.k]) / b + _foaw._timeStamps[_foaw.k];

			if (p_mean + pos_err_max < p_out || p_mean - pos_err_max > p_out || t_mean + time_err_max < t_out || t_mean - time_err_max > t_out)
			{
				bad++;
				if (bad >= max_outlier)
					break;
			}
		}
		if (bad >= max_outlier)
			break;

		velocity = _mult * b; // deg/s formulation
							  // velocity = b / _ppr * 2 * _pi * 1000000.0; // rad/s formulation
	}
	return velocity;
}

void velocity_estimation::_compare_swap(int a, int b)
{
	int indexA = _sortedIndex[a];
	int indexB = _sortedIndex[b];

	if (indexA > 4 || indexB > 4)
	{
		indexA = 0;
		indexB = 1;
	}

	if (_pastValues[indexA] < _pastValues[indexB])
	{
		int temp = _sortedIndex[a];
		_sortedIndex[a] = _sortedIndex[b];
		_sortedIndex[b] = temp;
	}
}