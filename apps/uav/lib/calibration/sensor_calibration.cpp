/****************************************************************************
 * apps/uav/lib/calibration/sensor_calibration.cpp
 *
 * Implementation of sensor calibration library
 ****************************************************************************/

#include "sensor_calibration.hpp"
#include <cstdio>

namespace calibration
{

/*---------------------------------------------------------------------------
 * Accelerometer implementation
 *---------------------------------------------------------------------------*/

Accelerometer::Accelerometer()
    : _calibration_count(0)
{
    reset();
}

bool Accelerometer::set_offset(const Vector3f &offset_m_s2)
{
    /* Nguong nho de van chap nhan cac bias nho sau calibration */
    Vector3f delta = _offset - offset_m_s2;
    if (delta.norm() > 1e-6f) {
        if (offset_m_s2.is_finite()) {
            _offset = offset_m_s2;
            _calibration_count++;
            return true;
        }
    }
    return false;
}

bool Accelerometer::set_scale(const Vector3f &scale)
{
    /* Nguong nho de cap nhat scale chinh xac tu calibration */
    Vector3f delta = _scale - scale;
    if (delta.norm() > 1e-6f) {
        /* Verify all scale factors are positive and finite */
        if (scale.is_finite() && scale.x > 0.0f && scale.y > 0.0f && scale.z > 0.0f) {
            _scale = scale;
            _calibration_count++;
            return true;
        }
    }
    return false;
}

void Accelerometer::set_rotation(const Dcmf &rotation)
{
    _rotation = rotation;
    _calibration_count++;
}

Vector3f Accelerometer::correct(const Vector3f &raw_data) const
{
    /* Apply offset correction và scale factors trong sensor frame */
    Vector3f corrected_sensor = (raw_data - _offset).emult(_scale);

    /* Rotate to body frame */
    return _rotation * corrected_sensor;
}

void Accelerometer::reset()
{
    _offset.zero();
    _scale = Vector3f(1.0f, 1.0f, 1.0f);  /* Unity scale */
    _rotation = Dcmf::identity();
    _calibration_count = 0;
}

/*---------------------------------------------------------------------------
 * Gyroscope implementation
 *---------------------------------------------------------------------------*/

Gyroscope::Gyroscope()
    : _calibration_count(0)
{
    reset();
}

bool Gyroscope::set_offset(const Vector3f &offset_rad_s)
{
    /* Nguong nho de van chap nhan gyro bias nho (~0.001 rad/s) */
    Vector3f delta = _offset - offset_rad_s;
    if (delta.norm() > 1e-6f) {
        if (offset_rad_s.is_finite()) {
            _offset = offset_rad_s;
            _calibration_count++;
            return true;
        }
    }
    return false;
}

void Gyroscope::set_rotation(const Dcmf &rotation)
{
    _rotation = rotation;
    _calibration_count++;
}

Vector3f Gyroscope::correct(const Vector3f &raw_data) const
{
    /* Apply offset correction trong sensor frame */
    Vector3f corrected_sensor = raw_data - _offset;

    /* Rotate to body frame */
    return _rotation * corrected_sensor;
}

void Gyroscope::reset()
{
    _offset.zero();
    _rotation = Dcmf::identity();
    _calibration_count = 0;
}

} // namespace calibration
