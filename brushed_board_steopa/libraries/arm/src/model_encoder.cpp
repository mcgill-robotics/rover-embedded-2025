/**
 * @file          model_encoder.cpp
 * @author        Vincent Boucher, Steve Ding
 * @version       V1.0.0
 * @date          30-May-2023
 * @brief         encoder object
 *
 * @attention     For prototype example only
 *				  V1.0.0 - created
 *
 */
#include <Arduino.h>
#include "model_encoder.h"

void model_encoder::initialize_encoder(uint8_t rotationalDirection, float offset, float resolution, uint8_t port)
{
    _offset = offset;
    _resolution = resolution;
    _port = port;

    _angularVelocity = 0.0;
    _quad_enc_pos = _offset * (_resolution / 360.0);
    _last_angle = 0.0;

    // TODO: check implementation multi-turn, should default true or false?
    _turn_count = 0;
    _angle_multi = 0.0;  // angle from 0 to 360 in degrees
    _angle_single = 0.0; // angle from 0 to 360 in degrees
    _is_multi_turn = true;

    switch (port)
    {
    case 1:
        _pinA = ENCPIN1_1;
        _pinB = ENCPIN1_2;
        break;
    case 2:
        _pinA = ENCPIN2_1;
        _pinB = ENCPIN2_2;
        break;
    case 3:
        _pinA = ENCPIN3_1;
        _pinB = ENCPIN3_2;
        break;
    default:
        _pinA = 0;
        _pinB = 0;
        break;
    }

    if (rotationalDirection)
    {
        uint8_t tempPin = _pinA;
        _pinA = _pinB;
        _pinB = tempPin;
    }

    _encoder = new QuadEncoder(port, _pinA, _pinB);

    _encoder->setInitConfig();
    _encoder->EncConfig.positionInitialValue = _quad_enc_pos;
    _encoder->EncConfig.revolutionCountCondition = 1;
    _encoder->init();

    _velocityEstimation.initialize_parameters(_resolution, 8.0, 1000000.0, 1, 200000);
}

void model_encoder::reset_encoder()
{
    _quad_enc_pos = _offset * (_resolution / 360.0);
    _encoder->write(_quad_enc_pos);
    _encoder->init();
}

void model_encoder::set_current_angle_es(float current_angle)
{
    Serial.printf("Setting as current: %f\n", current_angle);
    _offset = current_angle;
    _quad_enc_pos = current_angle * (_resolution / 360.0);
    _encoder->write(_quad_enc_pos);
}

void model_encoder::poll_encoder_angle()
{
    _quad_enc_pos = _encoder->read();

    // Useless since _position always positive?
    _angle_single = (_quad_enc_pos >= 0) ? (_quad_enc_pos % _resolution) : _resolution - (abs(_quad_enc_pos) % _resolution);
    _angle_single = (360.0 / _resolution) * _angle_single;
    _angle_multi = _quad_enc_pos * (360.0 / _resolution);

    _velocityEstimation.update_readings(_quad_enc_pos * (360.0 / _resolution), micros());
}

float model_encoder::get_angle_multi()
{
    return _angle_multi;
}

float model_encoder::get_angle_single()
{
    return _angle_single;
}

void model_encoder::set_parameters(uint8_t direction, float offset, float resolution)
{
    if (direction)
    {
        uint8_t tempPin = _pinA;
        _pinA = _pinB;
        _pinB = tempPin;
    }
    _encoder->enc_xbara_mapping(_pinA, PHASEA, 0);
    _encoder->enc_xbara_mapping(_pinB, PHASEB, 0);
    _encoder->disableInterrupts(_positionROEnable);
    _encoder->disableInterrupts(_positionRUEnable);

    _offset = offset;
    _resolution = resolution;
    _quad_enc_pos = _offset * (_resolution / 360.0);

    _encoder->setInitConfig();
    _encoder->EncConfig.positionInitialValue = _quad_enc_pos;
    _encoder->init();
}

void model_encoder::velocityEstimation(void)
{
    _angularVelocity = _velocityEstimation.foaw(0);
}

float model_encoder::getVelocity(void)
{
    return _angularVelocity;
}