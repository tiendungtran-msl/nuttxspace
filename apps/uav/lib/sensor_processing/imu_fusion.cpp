/****************************************************************************
 * apps/uav/lib/sensor_processing/imu_fusion.cpp
 *
 * MULTI-IMU FUSION - Implementation
 *
 ****************************************************************************/

#include "imu_fusion.hpp"
#include <stdio.h>
#include <syslog.h>

namespace uav {
namespace sensor_processing {

/****************************************************************************
 * Helper Functions
 ****************************************************************************/

/* Quick sort partition cho median finding */
static int partition(float arr[], int low, int high)
{
    float pivot = arr[high];
    int i = low - 1;

    for (int j = low; j < high; j++) {
        if (arr[j] <= pivot) {
            i++;
            float temp = arr[i];
            arr[i] = arr[j];
            arr[j] = temp;
        }
    }

    float temp = arr[i + 1];
    arr[i + 1] = arr[high];
    arr[high] = temp;

    return i + 1;
}

/* Quick select để tìm median (O(n) average) */
static float quick_select(float arr[], int left, int right, int k)
{
    if (left == right) {
        return arr[left];
    }

    int pivot_index = partition(arr, left, right);

    if (k == pivot_index) {
        return arr[k];
    } else if (k < pivot_index) {
        return quick_select(arr, left, pivot_index - 1, k);
    } else {
        return quick_select(arr, pivot_index + 1, right, k);
    }
}

/****************************************************************************
 * ImuFusion Implementation
 ****************************************************************************/

ImuFusion::ImuFusion()
    : m_mode(FusionMode::VOTING)
    , m_num_imus(CONFIG_UAV_NUM_IMUS)
    , m_primary_imu(0)
    , m_initialized(false)
{
    memset(m_imu_data, 0, sizeof(m_imu_data));
    memset(m_imu_status, 0, sizeof(m_imu_status));
    memset(m_last_imu_data, 0, sizeof(m_last_imu_data));
}

int ImuFusion::init(uint8_t num_imus)
{
    if (num_imus > CONFIG_UAV_NUM_IMUS) {
        return -1;
    }

    m_num_imus = num_imus;

    /* Initialize status for each IMU */
    for (uint8_t i = 0; i < m_num_imus; i++) {
        m_imu_status[i].present = false;
        m_imu_status[i].functional = false;
        m_imu_status[i].selected = false;
        m_imu_status[i].fault_count = 0;
        m_imu_status[i].total_samples = 0;
        m_imu_status[i].error_samples = 0;
        m_imu_status[i].noise_estimate = 0.01f;  /* Default noise */
        m_imu_status[i].weight = 1.0f / m_num_imus;
    }

    m_initialized = true;
    syslog(LOG_INFO, "[imu_fusion] Initialized with %d IMUs\n", m_num_imus);

    return 0;
}

void ImuFusion::set_mode(FusionMode mode)
{
    m_mode = mode;

    const char* mode_str = "UNKNOWN";
    switch (mode) {
        case FusionMode::VOTING:
            mode_str = "VOTING";
            break;
        case FusionMode::WEIGHTED:
            mode_str = "WEIGHTED";
            break;
        case FusionMode::PRIMARY:
            mode_str = "PRIMARY";
            break;
    }

    syslog(LOG_INFO, "[imu_fusion] Mode set to %s\n", mode_str);
}

void ImuFusion::set_primary_imu(uint8_t imu_index)
{
    if (imu_index < m_num_imus) {
        m_primary_imu = imu_index;
    }
}

void ImuFusion::set_imu_present(uint8_t imu_index, bool present)
{
    if (imu_index < m_num_imus) {
        m_imu_status[imu_index].present = present;
        if (present) {
            m_imu_status[imu_index].functional = true;
        }
    }
}

void ImuFusion::update_imu(uint8_t imu_index, const ImuData& data)
{
    if (imu_index >= m_num_imus || !m_initialized) {
        return;
    }

    /* Store previous data for noise estimation */
    m_last_imu_data[imu_index] = m_imu_data[imu_index];

    /* Update current data */
    m_imu_data[imu_index] = data;
    m_imu_status[imu_index].total_samples++;

    /* Update noise estimate */
    if (m_imu_status[imu_index].total_samples > 1) {
        update_noise_estimate(imu_index, data);
    }
}

int ImuFusion::fuse(FusedImuData& result)
{
    if (!m_initialized) {
        return -1;
    }

    /* Check faults first */
    check_faults();

    /* Count healthy IMUs */
    uint8_t healthy_count = 0;
    for (uint8_t i = 0; i < m_num_imus; i++) {
        if (m_imu_status[i].present && m_imu_status[i].functional &&
            m_imu_data[i].valid) {
            healthy_count++;
            m_imu_status[i].selected = true;
        } else {
            m_imu_status[i].selected = false;
        }
    }

    if (healthy_count == 0) {
        result.valid = false;
        return -1;
    }

    /* Initialize result */
    memset(&result, 0, sizeof(result));
    result.healthy_mask = get_healthy_mask();
    result.num_imus_used = healthy_count;

    /* Fusion theo mode */
    switch (m_mode) {
        case FusionMode::VOTING:
            fuse_voting(result);
            break;
        case FusionMode::WEIGHTED:
            update_weights();
            fuse_weighted(result);
            break;
        case FusionMode::PRIMARY:
            fuse_primary(result);
            break;
    }

    /* Use newest timestamp */
    uint64_t newest = 0;
    for (uint8_t i = 0; i < m_num_imus; i++) {
        if (m_imu_status[i].selected && m_imu_data[i].timestamp_us > newest) {
            newest = m_imu_data[i].timestamp_us;
        }
    }
    result.timestamp_us = newest;
    result.valid = true;

    return 0;
}

void ImuFusion::fuse_voting(FusedImuData& result)
{
    /* Collect values từ healthy IMUs */
    float gyro_x[CONFIG_UAV_NUM_IMUS], gyro_y[CONFIG_UAV_NUM_IMUS], gyro_z[CONFIG_UAV_NUM_IMUS];
    float accel_x[CONFIG_UAV_NUM_IMUS], accel_y[CONFIG_UAV_NUM_IMUS], accel_z[CONFIG_UAV_NUM_IMUS];
    float temp[CONFIG_UAV_NUM_IMUS];
    uint8_t count = 0;

    for (uint8_t i = 0; i < m_num_imus; i++) {
        if (m_imu_status[i].selected) {
            gyro_x[count] = m_imu_data[i].gyro[0];
            gyro_y[count] = m_imu_data[i].gyro[1];
            gyro_z[count] = m_imu_data[i].gyro[2];
            accel_x[count] = m_imu_data[i].accel[0];
            accel_y[count] = m_imu_data[i].accel[1];
            accel_z[count] = m_imu_data[i].accel[2];
            temp[count] = m_imu_data[i].temperature;
            count++;
        }
    }

    /* Calculate median for each axis */
    result.gyro[0] = calculate_median(gyro_x, count);
    result.gyro[1] = calculate_median(gyro_y, count);
    result.gyro[2] = calculate_median(gyro_z, count);
    result.accel[0] = calculate_median(accel_x, count);
    result.accel[1] = calculate_median(accel_y, count);
    result.accel[2] = calculate_median(accel_z, count);
    result.temperature = calculate_median(temp, count);
}

void ImuFusion::fuse_weighted(FusedImuData& result)
{
    float weight_sum = 0.0f;

    for (uint8_t i = 0; i < m_num_imus; i++) {
        if (m_imu_status[i].selected) {
            float w = m_imu_status[i].weight;
            weight_sum += w;

            for (int j = 0; j < 3; j++) {
                result.gyro[j] += w * m_imu_data[i].gyro[j];
                result.accel[j] += w * m_imu_data[i].accel[j];
            }
            result.temperature += w * m_imu_data[i].temperature;
        }
    }

    /* Normalize */
    if (weight_sum > 0.0f) {
        for (int j = 0; j < 3; j++) {
            result.gyro[j] /= weight_sum;
            result.accel[j] /= weight_sum;
        }
        result.temperature /= weight_sum;
    }
}

void ImuFusion::fuse_primary(FusedImuData& result)
{
    /* Try primary first */
    if (m_imu_status[m_primary_imu].selected) {
        for (int j = 0; j < 3; j++) {
            result.gyro[j] = m_imu_data[m_primary_imu].gyro[j];
            result.accel[j] = m_imu_data[m_primary_imu].accel[j];
        }
        result.temperature = m_imu_data[m_primary_imu].temperature;
        return;
    }

    /* Fallback to first available */
    for (uint8_t i = 0; i < m_num_imus; i++) {
        if (m_imu_status[i].selected) {
            for (int j = 0; j < 3; j++) {
                result.gyro[j] = m_imu_data[i].gyro[j];
                result.accel[j] = m_imu_data[i].accel[j];
            }
            result.temperature = m_imu_data[i].temperature;
            return;
        }
    }
}

void ImuFusion::check_faults()
{
    /* Calculate median for comparison */
    float gyro_med[3], accel_med[3];
    float values[CONFIG_UAV_NUM_IMUS];
    uint8_t count;

    for (int axis = 0; axis < 3; axis++) {
        count = 0;
        for (uint8_t i = 0; i < m_num_imus; i++) {
            if (m_imu_status[i].present && m_imu_data[i].valid) {
                values[count++] = m_imu_data[i].gyro[axis];
            }
        }
        gyro_med[axis] = (count > 0) ? calculate_median(values, count) : 0.0f;

        count = 0;
        for (uint8_t i = 0; i < m_num_imus; i++) {
            if (m_imu_status[i].present && m_imu_data[i].valid) {
                values[count++] = m_imu_data[i].accel[axis];
            }
        }
        accel_med[axis] = (count > 0) ? calculate_median(values, count) : 0.0f;
    }

    /* Check each IMU against median */
    for (uint8_t i = 0; i < m_num_imus; i++) {
        if (!m_imu_status[i].present || !m_imu_data[i].valid) {
            continue;
        }

        bool fault_detected = false;

        /* Check gyro */
        for (int axis = 0; axis < 3; axis++) {
            float diff = fabsf(m_imu_data[i].gyro[axis] - gyro_med[axis]);
            if (diff > CONFIG_UAV_IMU_FAULT_THRESHOLD_GYRO) {
                fault_detected = true;
                break;
            }
        }

        /* Check accel */
        if (!fault_detected) {
            for (int axis = 0; axis < 3; axis++) {
                float diff = fabsf(m_imu_data[i].accel[axis] - accel_med[axis]);
                if (diff > CONFIG_UAV_IMU_FAULT_THRESHOLD_ACCEL) {
                    fault_detected = true;
                    break;
                }
            }
        }

        /* Update fault counter */
        if (fault_detected) {
            m_imu_status[i].fault_count++;
            m_imu_status[i].error_samples++;

            if (m_imu_status[i].fault_count >= CONFIG_UAV_IMU_FAULT_COUNT_THRESHOLD) {
                if (m_imu_status[i].functional) {
                    syslog(LOG_WARNING, "[imu_fusion] IMU %d marked as non-functional\n", i);
                }
                m_imu_status[i].functional = false;
            }
        } else {
            m_imu_status[i].fault_count = 0;
            m_imu_status[i].functional = true;
        }
    }
}

float ImuFusion::calculate_median(float values[], uint8_t count)
{
    if (count == 0) {
        return 0.0f;
    }

    if (count == 1) {
        return values[0];
    }

    if (count == 2) {
        return (values[0] + values[1]) / 2.0f;
    }

    /* Use quick select for larger arrays */
    uint8_t mid = count / 2;
    return quick_select(values, 0, count - 1, mid);
}

void ImuFusion::update_noise_estimate(uint8_t imu_index, const ImuData& data)
{
    /* Simple noise estimate: running average of squared differences */
    float alpha = 0.01f;  /* Smoothing factor */

    float diff_sq = 0.0f;
    for (int j = 0; j < 3; j++) {
        float d = data.gyro[j] - m_last_imu_data[imu_index].gyro[j];
        diff_sq += d * d;
    }

    float noise = sqrtf(diff_sq / 3.0f);
    m_imu_status[imu_index].noise_estimate =
        (1.0f - alpha) * m_imu_status[imu_index].noise_estimate + alpha * noise;
}

void ImuFusion::update_weights()
{
    /* Weights inversely proportional to noise */
    float total_inv_noise = 0.0f;

    for (uint8_t i = 0; i < m_num_imus; i++) {
        if (m_imu_status[i].selected) {
            float noise = m_imu_status[i].noise_estimate;
            if (noise < 0.0001f) {
                noise = 0.0001f;  /* Prevent division by zero */
            }
            total_inv_noise += 1.0f / noise;
        }
    }

    for (uint8_t i = 0; i < m_num_imus; i++) {
        if (m_imu_status[i].selected) {
            float noise = m_imu_status[i].noise_estimate;
            if (noise < 0.0001f) {
                noise = 0.0001f;
            }
            m_imu_status[i].weight = (1.0f / noise) / total_inv_noise;
        } else {
            m_imu_status[i].weight = 0.0f;
        }
    }
}

ImuStatus ImuFusion::get_imu_status(uint8_t imu_index) const
{
    if (imu_index >= m_num_imus) {
        ImuStatus dummy = {};
        return dummy;
    }
    return m_imu_status[imu_index];
}

uint8_t ImuFusion::get_healthy_count() const
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < m_num_imus; i++) {
        if (m_imu_status[i].present && m_imu_status[i].functional) {
            count++;
        }
    }
    return count;
}

uint8_t ImuFusion::get_healthy_mask() const
{
    uint8_t mask = 0;
    for (uint8_t i = 0; i < m_num_imus; i++) {
        if (m_imu_status[i].present && m_imu_status[i].functional) {
            mask |= (1 << i);
        }
    }
    return mask;
}

void ImuFusion::reset_fault_detection()
{
    for (uint8_t i = 0; i < m_num_imus; i++) {
        m_imu_status[i].fault_count = 0;
        m_imu_status[i].error_samples = 0;
        if (m_imu_status[i].present) {
            m_imu_status[i].functional = true;
        }
    }
}

void ImuFusion::print_status() const
{
    printf("[imu_fusion] Status:\n");

    const char* mode_str = "UNKNOWN";
    switch (m_mode) {
        case FusionMode::VOTING:
            mode_str = "VOTING";
            break;
        case FusionMode::WEIGHTED:
            mode_str = "WEIGHTED";
            break;
        case FusionMode::PRIMARY:
            mode_str = "PRIMARY";
            break;
    }

    printf("  Mode: %s\n", mode_str);
    printf("  Healthy IMUs: %d/%d\n", get_healthy_count(), m_num_imus);
    printf("  Healthy mask: 0x%02X\n", get_healthy_mask());

    printf("\n  Per-IMU status:\n");
    for (uint8_t i = 0; i < m_num_imus; i++) {
        const ImuStatus& s = m_imu_status[i];
        printf("    IMU %d: %s/%s (samples=%lu, errors=%lu, noise=%.4f, weight=%.2f)\n",
               i,
               s.present ? "PRESENT" : "ABSENT",
               s.functional ? "OK" : "FAIL",
               (unsigned long)s.total_samples,
               (unsigned long)s.error_samples,
               (double)s.noise_estimate,
               (double)s.weight);
    }
}

/****************************************************************************
 * Coning/Sculling Integration
 ****************************************************************************/

void integrate_imu_samples(const ImuData* samples, uint8_t count, DeltaState& output)
{
    if (count == 0) {
        memset(&output, 0, sizeof(output));
        return;
    }

    /* Initialize output */
    memset(&output, 0, sizeof(output));

    float total_dt = 0.0f;
    float prev_gyro[3] = {0}, prev_accel[3] = {0};
    uint64_t first_ts = samples[0].timestamp_us;
    uint64_t last_ts = first_ts;

    for (uint8_t i = 0; i < count; i++) {
        const ImuData& s = samples[i];

        if (!s.valid) {
            continue;
        }

        /* Calculate dt */
        float dt = (i == 0) ? 0.0f : (float)(s.timestamp_us - last_ts) * 1e-6f;
        last_ts = s.timestamp_us;

        if (dt <= 0.0f || dt > 0.01f) {
            /* Invalid dt, skip */
            continue;
        }

        total_dt += dt;

        /* Simple trapezoidal integration */
        for (int j = 0; j < 3; j++) {
            /* Delta angle (gyro integration) */
            if (i > 0) {
                output.delta_angle[j] += 0.5f * (s.gyro[j] + prev_gyro[j]) * dt;
            }

            /* Delta velocity (accel integration) */
            if (i > 0) {
                output.delta_velocity[j] += 0.5f * (s.accel[j] + prev_accel[j]) * dt;
            }

            prev_gyro[j] = s.gyro[j];
            prev_accel[j] = s.accel[j];
        }

        /* TODO: Add coning/sculling correction terms */
        /* For now, simple integration is used */
    }

    output.dt = total_dt;
    output.timestamp_us = last_ts;
}

} /* namespace sensor_processing */
} /* namespace uav */
