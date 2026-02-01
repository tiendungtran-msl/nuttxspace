/****************************************************************************
 * apps/uav/lib/dsp/filters.hpp
 *
 * DSP FILTERS - Bộ lọc số cho xử lý tín hiệu cảm biến
 *
 * MỤC ĐÍCH:
 * - Lọc nhiễu cao tần từ IMU (lowpass)
 * - Loại bỏ rung động motor (notch filter)
 * - Decimation khi cần giảm sample rate
 * - Tất cả filters đều có latency xác định
 *
 * THIẾT KẾ:
 * - Template-based cho flexibility
 * - Không dynamic memory allocation
 * - Deterministic performance (fixed-point friendly)
 * - State được lưu trữ trong filter object
 *
 * FILTERS AVAILABLE:
 * 1. LowPassFilter2p - Biquad lowpass filter (2nd order Butterworth)
 * 2. NotchFilter - Biquad notch filter để loại bỏ frequency cụ thể
 * 3. DecimationFilter - FIR filter với downsampling
 * 4. MedianFilter - Median filter cho spike removal
 *
 ****************************************************************************/

#ifndef __UAV_LIB_DSP_FILTERS_HPP
#define __UAV_LIB_DSP_FILTERS_HPP

#include <stdint.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace uav {
namespace dsp {

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
        m_delay1(0.0f), m_delay2(0.0f),
        m_cutoff_freq(0.0f), m_sample_freq(0.0f)
    {}

    /**
     * @brief Cấu hình filter với sample rate và cutoff frequency
     *
     * @param sample_freq   Sample frequency (Hz)
     * @param cutoff_freq   Cutoff frequency (Hz)
     */
    void set_cutoff_frequency(float sample_freq, float cutoff_freq)
    {
        m_sample_freq = sample_freq;
        m_cutoff_freq = cutoff_freq;

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
        const float ohm = tanf(M_PI / fr);
        const float c = 1.0f + 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm;

        m_b0 = ohm * ohm / c;
        m_b1 = 2.0f * m_b0;
        m_b2 = m_b0;
        m_a1 = 2.0f * (ohm * ohm - 1.0f) / c;
        m_a2 = (1.0f - 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm) / c;
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

    /**
     * @brief Reset filter state với initial value
     *
     * Dùng để tránh transient khi bắt đầu filter.
     */
    void reset(float value)
    {
        float denom = 1.0f + m_a1 + m_a2;
        if (fabsf(denom) > 1e-10f) {
            m_delay1 = m_delay2 = value * (m_b0 + m_b1 + m_b2) / denom;
        } else {
            m_delay1 = m_delay2 = value;
        }
    }

    float get_cutoff_freq() const { return m_cutoff_freq; }
    float get_sample_freq() const { return m_sample_freq; }

private:
    /* Filter coefficients */
    float m_b0, m_b1, m_b2;
    float m_a1, m_a2;

    /* Delay elements (state) */
    float m_delay1, m_delay2;

    /* Stored parameters */
    float m_cutoff_freq;
    float m_sample_freq;
};

/****************************************************************************
 * NotchFilter - Biquad notch filter
 *
 * Loại bỏ một frequency cụ thể (ví dụ: motor vibration).
 *
 * Usage:
 *   NotchFilter filter;
 *   filter.set_notch_frequency(1000.0f, 200.0f, 0.5f);  // fs=1kHz, notch=200Hz, Q=0.5
 *   float output = filter.apply(input);
 *
 ****************************************************************************/

class NotchFilter {
public:
    NotchFilter() :
        m_b0(1.0f), m_b1(0.0f), m_b2(1.0f),
        m_a1(0.0f), m_a2(0.0f),
        m_delay1(0.0f), m_delay2(0.0f),
        m_notch_freq(0.0f), m_bandwidth(0.0f)
    {}

    /**
     * @brief Cấu hình notch filter
     *
     * @param sample_freq   Sample frequency (Hz)
     * @param notch_freq    Frequency cần loại bỏ (Hz)
     * @param bandwidth     Bandwidth (Hz) - frequency range bị attenuate
     */
    void set_notch_frequency(float sample_freq, float notch_freq, float bandwidth)
    {
        m_notch_freq = notch_freq;
        m_bandwidth = bandwidth;

        /* Validate */
        if (notch_freq <= 0.0f || sample_freq <= 0.0f ||
            notch_freq >= sample_freq / 2.0f || bandwidth <= 0.0f) {
            /* Bypass mode */
            m_b0 = m_b2 = 1.0f;
            m_b1 = m_a1 = m_a2 = 0.0f;
            return;
        }

        /* Calculate coefficients */
        const float w0 = 2.0f * M_PI * notch_freq / sample_freq;
        const float alpha = sinf(w0) * sinhf(logf(2.0f) / 2.0f * bandwidth * w0 / sinf(w0));

        m_b0 = 1.0f;
        m_b1 = -2.0f * cosf(w0);
        m_b2 = 1.0f;

        float a0 = 1.0f + alpha;
        m_a1 = -2.0f * cosf(w0) / a0;
        m_a2 = (1.0f - alpha) / a0;

        m_b0 /= a0;
        m_b1 /= a0;
        m_b2 /= a0;
    }

    /**
     * @brief Apply filter với một sample
     */
    float apply(float sample)
    {
        float output = m_b0 * sample + m_delay1;
        m_delay1 = m_b1 * sample - m_a1 * output + m_delay2;
        m_delay2 = m_b2 * sample - m_a2 * output;
        return output;
    }

    void reset()
    {
        m_delay1 = m_delay2 = 0.0f;
    }

    float get_notch_freq() const { return m_notch_freq; }
    float get_bandwidth() const { return m_bandwidth; }

private:
    float m_b0, m_b1, m_b2;
    float m_a1, m_a2;
    float m_delay1, m_delay2;
    float m_notch_freq, m_bandwidth;
};

/****************************************************************************
 * MedianFilter - Median filter để loại bỏ spikes
 *
 * Loại bỏ outliers/spikes mà không làm mờ edges như lowpass.
 *
 * @tparam SIZE Window size (thường là số lẻ: 3, 5, 7)
 *
 ****************************************************************************/

template<uint32_t SIZE>
class MedianFilter {
public:
    MedianFilter() : m_index(0), m_count(0) {
        memset(m_buffer, 0, sizeof(m_buffer));
    }

    /**
     * @brief Apply filter
     *
     * @param sample Input sample
     * @return Median value trong window
     */
    float apply(float sample)
    {
        /* Add to circular buffer */
        m_buffer[m_index] = sample;
        m_index = (m_index + 1) % SIZE;

        if (m_count < SIZE) {
            m_count++;
        }

        /* Copy buffer for sorting */
        float sorted[SIZE];
        memcpy(sorted, m_buffer, m_count * sizeof(float));

        /* Simple insertion sort (OK for small SIZE) */
        for (uint32_t i = 1; i < m_count; i++) {
            float key = sorted[i];
            int j = i - 1;
            while (j >= 0 && sorted[j] > key) {
                sorted[j + 1] = sorted[j];
                j--;
            }
            sorted[j + 1] = key;
        }

        /* Return median */
        return sorted[m_count / 2];
    }

    void reset()
    {
        m_index = 0;
        m_count = 0;
        memset(m_buffer, 0, sizeof(m_buffer));
    }

private:
    float m_buffer[SIZE];
    uint32_t m_index;
    uint32_t m_count;
};

/****************************************************************************
 * DecimationFilter - FIR filter với downsampling
 *
 * Kết hợp lowpass filtering và decimation để giảm sample rate.
 *
 * @tparam FACTOR Decimation factor (output_rate = input_rate / FACTOR)
 *
 ****************************************************************************/

template<uint32_t FACTOR>
class DecimationFilter {
    static_assert(FACTOR >= 2 && FACTOR <= 16, "Decimation factor must be 2-16");

public:
    DecimationFilter() : m_index(0), m_count(0) {
        /* Default coefficients: simple averaging (box filter) */
        for (uint32_t i = 0; i < FACTOR; i++) {
            m_coeffs[i] = 1.0f / FACTOR;
        }
        memset(m_buffer, 0, sizeof(m_buffer));
    }

    /**
     * @brief Set custom FIR coefficients
     *
     * @param coeffs Array of FACTOR coefficients
     */
    void set_coefficients(const float* coeffs)
    {
        for (uint32_t i = 0; i < FACTOR; i++) {
            m_coeffs[i] = coeffs[i];
        }
    }

    /**
     * @brief Apply với một input sample
     *
     * @param sample    Input sample
     * @param output    Output (chỉ valid khi return true)
     * @return          true nếu có output (mỗi FACTOR samples)
     */
    bool apply(float sample, float& output)
    {
        m_buffer[m_index] = sample;
        m_index = (m_index + 1) % FACTOR;
        m_count++;

        if (m_count >= FACTOR) {
            m_count = 0;

            /* FIR convolution */
            float sum = 0.0f;
            uint32_t idx = m_index;
            for (uint32_t i = 0; i < FACTOR; i++) {
                sum += m_coeffs[i] * m_buffer[idx];
                idx = (idx + 1) % FACTOR;
            }

            output = sum;
            return true;
        }

        return false;
    }

    void reset()
    {
        m_index = 0;
        m_count = 0;
        memset(m_buffer, 0, sizeof(m_buffer));
    }

    constexpr uint32_t factor() const { return FACTOR; }

private:
    float m_coeffs[FACTOR];
    float m_buffer[FACTOR];
    uint32_t m_index;
    uint32_t m_count;
};

/****************************************************************************
 * Vector3Filter - Apply scalar filter cho vector 3D
 *
 * Wrapper để apply cùng một filter cho X, Y, Z.
 *
 ****************************************************************************/

template<typename FilterType>
class Vector3Filter {
public:
    template<typename... Args>
    void configure(Args... args)
    {
        m_x.set_cutoff_frequency(args...);
        m_y.set_cutoff_frequency(args...);
        m_z.set_cutoff_frequency(args...);
    }

    void apply(const float in[3], float out[3])
    {
        out[0] = m_x.apply(in[0]);
        out[1] = m_y.apply(in[1]);
        out[2] = m_z.apply(in[2]);
    }

    void reset()
    {
        m_x.reset();
        m_y.reset();
        m_z.reset();
    }

private:
    FilterType m_x, m_y, m_z;
};

} /* namespace dsp */
} /* namespace uav */

#endif /* __UAV_LIB_DSP_FILTERS_HPP */
