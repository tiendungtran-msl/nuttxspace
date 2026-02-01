/****************************************************************************
 * apps/uav/lib/mathlib/filters.hpp
 *
 * Digital Filters cho signal processing
 *
 * MỤC ĐÍCH:
 * - Low-pass filter: lọc nhiễu high-frequency
 * - High-pass filter: lọc drift
 * - Notch filter: lọc nhiễu motor vibration
 *
 ****************************************************************************/

#ifndef UAV_LIB_MATHLIB_FILTERS_HPP
#define UAV_LIB_MATHLIB_FILTERS_HPP

#include <cmath>

namespace mathlib {

/****************************************************************************
 * LowPassFilter - First-order IIR low-pass filter
 *
 * y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
 *
 * alpha = dt / (RC + dt) = dt / (tau + dt)
 * tau = 1 / (2 * pi * fc)
 ****************************************************************************/

class LowPassFilter {
public:
    LowPassFilter() : m_alpha(1.0f), m_output(0.0f), m_initialized(false) {}

    /**
     * Set cutoff frequency
     *
     * @param fc Cutoff frequency (Hz)
     * @param dt Sample period (seconds)
     */
    void set_cutoff(float fc, float dt) {
        if (fc <= 0.0f || dt <= 0.0f) {
            m_alpha = 1.0f;  // No filtering
            return;
        }

        float tau = 1.0f / (2.0f * M_PI * fc);
        m_alpha = dt / (tau + dt);
    }

    /**
     * Set alpha directly
     */
    void set_alpha(float alpha) {
        m_alpha = (alpha > 1.0f) ? 1.0f : ((alpha < 0.0f) ? 0.0f : alpha);
    }

    /**
     * Reset filter state
     */
    void reset(float initial_value = 0.0f) {
        m_output = initial_value;
        m_initialized = (initial_value != 0.0f);
    }

    /**
     * Apply filter
     *
     * @param input New sample
     * @return Filtered output
     */
    float apply(float input) {
        if (!m_initialized) {
            m_output = input;
            m_initialized = true;
        } else {
            m_output = m_alpha * input + (1.0f - m_alpha) * m_output;
        }
        return m_output;
    }

    /**
     * Get current output without new input
     */
    float get_output() const {
        return m_output;
    }

private:
    float m_alpha;      // Filter coefficient
    float m_output;     // Previous output
    bool m_initialized;
};

/****************************************************************************
 * LowPassFilter2p - Second-order Butterworth low-pass filter
 *
 * Better frequency response than first-order
 ****************************************************************************/

class LowPassFilter2p {
public:
    LowPassFilter2p() : m_cutoff(0), m_sample_rate(0),
                         m_a1(0), m_a2(0), m_b0(0), m_b1(0), m_b2(0),
                         m_delay1(0), m_delay2(0) {}

    /**
     * Set cutoff frequency and sample rate
     */
    void set_cutoff_frequency(float sample_rate, float cutoff_freq) {
        m_sample_rate = sample_rate;
        m_cutoff = cutoff_freq;

        if (cutoff_freq <= 0.0f || sample_rate <= 0.0f) {
            // Disable filtering
            m_b0 = 1.0f;
            m_b1 = 0.0f;
            m_b2 = 0.0f;
            m_a1 = 0.0f;
            m_a2 = 0.0f;
            return;
        }

        // Warped frequency
        float fr = sample_rate / cutoff_freq;
        float ohm = tanf(M_PI / fr);
        float c = 1.0f + 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm;

        m_b0 = ohm * ohm / c;
        m_b1 = 2.0f * m_b0;
        m_b2 = m_b0;
        m_a1 = 2.0f * (ohm * ohm - 1.0f) / c;
        m_a2 = (1.0f - 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm) / c;
    }

    /**
     * Reset filter state
     */
    void reset(float value = 0.0f) {
        m_delay1 = value;
        m_delay2 = value;
    }

    /**
     * Apply filter
     */
    float apply(float sample) {
        // Direct Form II implementation
        float delay0 = sample - m_delay1 * m_a1 - m_delay2 * m_a2;
        float output = delay0 * m_b0 + m_delay1 * m_b1 + m_delay2 * m_b2;

        m_delay2 = m_delay1;
        m_delay1 = delay0;

        return output;
    }

    float get_cutoff() const { return m_cutoff; }
    float get_sample_rate() const { return m_sample_rate; }

private:
    float m_cutoff;
    float m_sample_rate;

    // Coefficients
    float m_a1, m_a2;
    float m_b0, m_b1, m_b2;

    // State
    float m_delay1, m_delay2;
};

/****************************************************************************
 * HighPassFilter - First-order high-pass filter
 *
 * y[n] = alpha * (y[n-1] + x[n] - x[n-1])
 ****************************************************************************/

class HighPassFilter {
public:
    HighPassFilter() : m_alpha(0.9f), m_prev_input(0), m_output(0),
                        m_initialized(false) {}

    /**
     * Set cutoff frequency
     *
     * @param fc Cutoff frequency (Hz)
     * @param dt Sample period (seconds)
     */
    void set_cutoff(float fc, float dt) {
        if (fc <= 0.0f || dt <= 0.0f) {
            m_alpha = 1.0f;
            return;
        }

        float tau = 1.0f / (2.0f * M_PI * fc);
        m_alpha = tau / (tau + dt);
    }

    /**
     * Reset filter state
     */
    void reset(float initial_value = 0.0f) {
        m_prev_input = initial_value;
        m_output = 0.0f;
        m_initialized = false;
    }

    /**
     * Apply filter
     */
    float apply(float input) {
        if (!m_initialized) {
            m_prev_input = input;
            m_output = 0.0f;
            m_initialized = true;
        } else {
            m_output = m_alpha * (m_output + input - m_prev_input);
            m_prev_input = input;
        }
        return m_output;
    }

private:
    float m_alpha;
    float m_prev_input;
    float m_output;
    bool m_initialized;
};

/****************************************************************************
 * NotchFilter - Lọc một tần số cụ thể (motor vibration)
 ****************************************************************************/

class NotchFilter {
public:
    NotchFilter() : m_a1(0), m_a2(0), m_b0(1), m_b1(0), m_b2(0),
                    m_delay1(0), m_delay2(0) {}

    /**
     * Set notch frequency
     *
     * @param sample_rate Sample rate (Hz)
     * @param notch_freq Frequency to notch (Hz)
     * @param bandwidth Bandwidth of notch (Hz)
     */
    void set_notch_frequency(float sample_rate, float notch_freq, float bandwidth) {
        if (notch_freq <= 0.0f || sample_rate <= 0.0f || bandwidth <= 0.0f) {
            m_b0 = 1.0f;
            m_b1 = 0.0f;
            m_b2 = 0.0f;
            m_a1 = 0.0f;
            m_a2 = 0.0f;
            return;
        }

        float omega = 2.0f * M_PI * notch_freq / sample_rate;
        float alpha = sinf(omega) * sinhf(logf(2.0f) / 2.0f * bandwidth * omega / sinf(omega));

        m_b0 = 1.0f;
        m_b1 = -2.0f * cosf(omega);
        m_b2 = 1.0f;

        float a0 = 1.0f + alpha;
        m_a1 = -2.0f * cosf(omega) / a0;
        m_a2 = (1.0f - alpha) / a0;

        m_b0 /= a0;
        m_b1 /= a0;
        m_b2 /= a0;
    }

    /**
     * Reset filter state
     */
    void reset() {
        m_delay1 = 0.0f;
        m_delay2 = 0.0f;
    }

    /**
     * Apply filter
     */
    float apply(float sample) {
        float output = m_b0 * sample + m_b1 * m_delay1 + m_b2 * m_delay2
                     - m_a1 * m_delay1 - m_a2 * m_delay2;

        m_delay2 = m_delay1;
        m_delay1 = sample;

        return output;
    }

private:
    float m_a1, m_a2;
    float m_b0, m_b1, m_b2;
    float m_delay1, m_delay2;
};

} // namespace mathlib

#endif // UAV_LIB_MATHLIB_FILTERS_HPP
