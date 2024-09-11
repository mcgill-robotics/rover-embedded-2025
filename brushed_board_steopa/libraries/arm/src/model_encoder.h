/**
 * @file          model_sensor_sensors.h
 * @author        Steve Ding, Oliver Philbin-Briscoe, Colin Gallacher
 * @version       V1.1.0
 * @date          26-Janurary-2023
 * @brief         generic sensor object
 *
 * @attention     For prototype example only
 *				  V1.1.0 - added offset calculations for current and inkwell sensors
 *
 */

#ifndef MODEL_ENCODER_H
#define MODEL_ENCODER_H

#include "hardware_pins.h"
#include "velocity_estimation.h"
#include "QuadEncoder.h"

class model_encoder
{
public:
    QuadEncoder *_encoder;

    void initialize_encoder(uint8_t rotationalDirection, float offset, float resolution, uint8_t port);

    void reset_encoder(void);

    void set_current_angle_es(float offset);

    void poll_encoder_angle(void);

    float get_angle_multi(void);

    float get_angle_single(void);

    float get_full_angle_from_tick()
    {
        uint32_t tick = _encoder->read();
        return (float)tick * 360.0 / (float)_resolution;
    }

    void set_parameters(uint8_t direction, float offset, float resolution);

    /**
     * Main logic loop for calculating detected encoder velocity
     *
     * @param currentTime current micros count
     */
    void velocityEstimation(void);

    float getVelocity(void);

private:
    // QuadEncoder *_encoder;
    int32_t _offset;
    int32_t _resolution;
    uint8_t _port;
    int32_t _quad_enc_pos;
    uint8_t _pinA;
    uint8_t _pinB;

    float _angle;
    float _angle_single;
    float _angle_multi;
    float _angularVelocity;
    velocity_estimation _velocityEstimation;
    boolean _is_multi_turn;
    int _turn_count;
    float _last_angle;
    // Offset compared to boot position
    float _zero_angle_offset;
};

#endif