/****************************************************************************
 * apps/uav/lib/dsp/filters.hpp
 *
 * DSP FILTERS - Minimal filters for UAV runtime
 *
 * Chỉ giữ LowPassFilter2p vì đây là filter duy nhất đang dùng trong
 * sensors_app cho pipeline ước lượng tư thế.
 *
 ****************************************************************************/

#ifndef __UAV_LIB_DSP_FILTERS_HPP
#define __UAV_LIB_DSP_FILTERS_HPP

#include <math.h>

namespace uav {
namespace dsp {

static constexpr float kPi = 3.14159265358979323846f;

/****************************************************************************
 * LowPassFilter2p - 2nd order Butterworth lowpass filter (biquad)
 *
 * Đặc điểm:
 * - Đáp ứng phẳng trong passband
 * - Rolloff 12 dB/octave
 * - Phase shift 90° tại cutoff
 *
 * Usage:
 *   LowPassFilter2p filter;
 *   filter.set_cutoff_frequency(1000.0f, 100.0f);  // fs=1kHz, fc=100Hz
 *   float output = filter.apply(input);
 *
 ****************************************************************************/

class LowPassFilter2p {
public:
    LowPassFilter2p() :
        m_b0(1.0f), m_b1(0.0f), m_b2(0.0f),
        m_a1(0.0f), m_a2(0.0f),
        m_delay1(0.0f), m_delay2(0.0f)
    {}

    /**
     * @brief Cấu hình filter với sample rate và cutoff frequency
     *
     * @param sample_freq   Sample frequency (Hz)
     * @param cutoff_freq   Cutoff frequency (Hz)
     */
    void set_cutoff_frequency(float sample_freq, float cutoff_freq)
    {
        /* Validate */
        if (cutoff_freq <= 0.0f || sample_freq <= 0.0f ||
            cutoff_freq >= sample_freq / 2.0f) {
            /* Bypass mode */
            m_b0 = 1.0f;
            m_b1 = m_b2 = 0.0f;
            m_a1 = m_a2 = 0.0f;
            return;
        }

        /* Bilinear transform */
        const float fr = sample_freq / cutoff_freq;
    const float ohm = tanf(kPi / fr);
    const float c = 1.0f + 2.0f * cosf(kPi / 4.0f) * ohm + ohm * ohm;

        m_b0 = ohm * ohm / c;
        m_b1 = 2.0f * m_b0;
        m_b2 = m_b0;
        m_a1 = 2.0f * (ohm * ohm - 1.0f) / c;
    m_a2 = (1.0f - 2.0f * cosf(kPi / 4.0f) * ohm + ohm * ohm) / c;
    }

    /**
     * @brief Apply filter với một sample
     *
     * @param sample    Input sample
     * @return          Filtered output
     */
    float apply(float sample)
    {
        /* Direct Form II Transposed */
        float output = m_b0 * sample + m_delay1;
        m_delay1 = m_b1 * sample - m_a1 * output + m_delay2;
        m_delay2 = m_b2 * sample - m_a2 * output;
        return output;
    }

    /**
     * @brief Reset filter state
     */
    void reset()
    {
        m_delay1 = m_delay2 = 0.0f;
    }

private:
    /* Filter coefficients */
    float m_b0, m_b1, m_b2;
    float m_a1, m_a2;

    /* Delay elements (state) */
    float m_delay1, m_delay2;
};

} /* namespace dsp */
} /* namespace uav */

#endif /* __UAV_LIB_DSP_FILTERS_HPP */
