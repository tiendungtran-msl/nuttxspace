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

static constexpr float kEps = 1e-6f;

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
        m_imu_status[i].recovery_count = 0;
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
            m_imu_status[imu_index].fault_count = 0;
            m_imu_status[imu_index].recovery_count = 0;
        } else {
            m_imu_status[imu_index].functional = false;
            m_imu_status[imu_index].selected = false;
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

    uint64_t newest_ts = 0;
    for (uint8_t i = 0; i < m_num_imus; i++) {
        if (m_imu_status[i].present && m_imu_data[i].valid &&
            m_imu_data[i].timestamp_us > newest_ts) {
            newest_ts = m_imu_data[i].timestamp_us;
        }
    }

    if (newest_ts == 0) {
        result.valid = false;
        return -1;
    }

    /* Count healthy and fresh IMUs */
    uint8_t healthy_count = 0;
    for (uint8_t i = 0; i < m_num_imus; i++) {
        bool fresh = (newest_ts - m_imu_data[i].timestamp_us) <=
                     CONFIG_UAV_IMU_MAX_TIMESTAMP_SKEW_US;

        if (m_imu_status[i].present && m_imu_status[i].functional &&
            m_imu_data[i].valid && fresh) {
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

    /* Use newest selected timestamp */
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

void ImuFusion::compute_reference_median(float gyro_ref[3], float accel_ref[3], float &temp_ref) const
{
    float gx[CONFIG_UAV_NUM_IMUS];
    float gy[CONFIG_UAV_NUM_IMUS];
    float gz[CONFIG_UAV_NUM_IMUS];
    float ax[CONFIG_UAV_NUM_IMUS];
    float ay[CONFIG_UAV_NUM_IMUS];
    float az[CONFIG_UAV_NUM_IMUS];
    float tt[CONFIG_UAV_NUM_IMUS];

    uint8_t count = 0;
    for (uint8_t i = 0; i < m_num_imus; i++) {
        if (!m_imu_status[i].selected) {
            continue;
        }

        gx[count] = m_imu_data[i].gyro[0];
        gy[count] = m_imu_data[i].gyro[1];
        gz[count] = m_imu_data[i].gyro[2];
        ax[count] = m_imu_data[i].accel[0];
        ay[count] = m_imu_data[i].accel[1];
        az[count] = m_imu_data[i].accel[2];
        tt[count] = m_imu_data[i].temperature;
        count++;
    }

    if (count == 0) {
        gyro_ref[0] = gyro_ref[1] = gyro_ref[2] = 0.0f;
        accel_ref[0] = accel_ref[1] = accel_ref[2] = 0.0f;
        temp_ref = 0.0f;
        return;
    }

    gyro_ref[0] = calculate_median(gx, count);
    gyro_ref[1] = calculate_median(gy, count);
    gyro_ref[2] = calculate_median(gz, count);

    accel_ref[0] = calculate_median(ax, count);
    accel_ref[1] = calculate_median(ay, count);
    accel_ref[2] = calculate_median(az, count);

    temp_ref = calculate_median(tt, count);
}

float ImuFusion::compute_residual_score(uint8_t imu_index,
                                        const float gyro_ref[3],
                                        const float accel_ref[3]) const
{
    float gyro_ratio_max = 0.0f;
    float accel_ratio_max = 0.0f;

    for (int axis = 0; axis < 3; axis++) {
        float dg = fabsf(m_imu_data[imu_index].gyro[axis] - gyro_ref[axis]);
        float da = fabsf(m_imu_data[imu_index].accel[axis] - accel_ref[axis]);

        float rg = dg / (CONFIG_UAV_IMU_FAULT_THRESHOLD_GYRO + kEps);
        float ra = da / (CONFIG_UAV_IMU_FAULT_THRESHOLD_ACCEL + kEps);

        if (rg > gyro_ratio_max) {
            gyro_ratio_max = rg;
        }
        if (ra > accel_ratio_max) {
            accel_ratio_max = ra;
        }
    }

    return (gyro_ratio_max > accel_ratio_max) ? gyro_ratio_max : accel_ratio_max;
}

void ImuFusion::fuse_voting(FusedImuData& result)
{
    float gyro_ref[3];
    float accel_ref[3];
    float temp_ref = 0.0f;
    compute_reference_median(gyro_ref, accel_ref, temp_ref);

    /* Median baseline */
    for (int axis = 0; axis < 3; axis++) {
        result.gyro[axis] = gyro_ref[axis];
        result.accel[axis] = accel_ref[axis];
    }
    result.temperature = temp_ref;

    /* Inlier-weighted refinement quanh median để tăng độ chính xác */
    float sum_gyro[3] = {0.0f, 0.0f, 0.0f};
    float sum_accel[3] = {0.0f, 0.0f, 0.0f};
    float sum_temp = 0.0f;
    float wsum = 0.0f;

    for (uint8_t i = 0; i < m_num_imus; i++) {
        if (!m_imu_status[i].selected) {
            continue;
        }

        float score = compute_residual_score(i, gyro_ref, accel_ref);
        if (score > CONFIG_UAV_IMU_INLIER_GATE) {
            continue;
        }

        float w = 1.0f / (1.0f + score);
        wsum += w;

        for (int axis = 0; axis < 3; axis++) {
            sum_gyro[axis] += w * m_imu_data[i].gyro[axis];
            sum_accel[axis] += w * m_imu_data[i].accel[axis];
        }
        sum_temp += w * m_imu_data[i].temperature;
    }

    if (wsum > kEps) {
        for (int axis = 0; axis < 3; axis++) {
            result.gyro[axis] = sum_gyro[axis] / wsum;
            result.accel[axis] = sum_accel[axis] / wsum;
        }
        result.temperature = sum_temp / wsum;
    }
}

void ImuFusion::fuse_weighted(FusedImuData& result)
{
    float gyro_ref[3];
    float accel_ref[3];
    float temp_ref = 0.0f;
    compute_reference_median(gyro_ref, accel_ref, temp_ref);

    float weight_sum = 0.0f;

    for (uint8_t i = 0; i < m_num_imus; i++) {
        if (!m_imu_status[i].selected) {
            continue;
        }

        float score = compute_residual_score(i, gyro_ref, accel_ref);
        if (score > CONFIG_UAV_IMU_INLIER_GATE) {
            continue;
        }

        float robust = 1.0f / (1.0f + score * score);
        float w = m_imu_status[i].weight * robust;

        weight_sum += w;

        for (int j = 0; j < 3; j++) {
            result.gyro[j] += w * m_imu_data[i].gyro[j];
            result.accel[j] += w * m_imu_data[i].accel[j];
        }
        result.temperature += w * m_imu_data[i].temperature;
    }

    if (weight_sum > kEps) {
        for (int j = 0; j < 3; j++) {
            result.gyro[j] /= weight_sum;
            result.accel[j] /= weight_sum;
        }
        result.temperature /= weight_sum;
    } else {
        /* fallback an toàn khi không còn inlier */
        fuse_voting(result);
    }
}

void ImuFusion::fuse_primary(FusedImuData& result)
{
    if (m_imu_status[m_primary_imu].selected) {
        float gyro_ref[3];
        float accel_ref[3];
        float temp_ref = 0.0f;
        compute_reference_median(gyro_ref, accel_ref, temp_ref);

        float score = compute_residual_score(m_primary_imu, gyro_ref, accel_ref);
        if (score <= CONFIG_UAV_IMU_INLIER_GATE) {
            for (int j = 0; j < 3; j++) {
                result.gyro[j] = m_imu_data[m_primary_imu].gyro[j];
                result.accel[j] = m_imu_data[m_primary_imu].accel[j];
            }
            result.temperature = m_imu_data[m_primary_imu].temperature;
            return;
        }
    }

    /* Fallback: robust weighted fusion */
    update_weights();
    fuse_weighted(result);
}

void ImuFusion::check_faults()
{
    uint64_t newest_ts = 0;
    for (uint8_t i = 0; i < m_num_imus; i++) {
        if (m_imu_status[i].present && m_imu_data[i].valid &&
            m_imu_data[i].timestamp_us > newest_ts) {
            newest_ts = m_imu_data[i].timestamp_us;
        }
    }

    if (newest_ts == 0) {
        return;
    }

    /* Calculate reference median from fresh valid IMUs */
    float gyro_med[3] = {0.0f, 0.0f, 0.0f};
    float accel_med[3] = {0.0f, 0.0f, 0.0f};
    float values[CONFIG_UAV_NUM_IMUS];
    bool has_reference = false;

    for (int axis = 0; axis < 3; axis++) {
        uint8_t count = 0;
        for (uint8_t i = 0; i < m_num_imus; i++) {
            bool fresh = (newest_ts - m_imu_data[i].timestamp_us) <=
                         CONFIG_UAV_IMU_MAX_TIMESTAMP_SKEW_US;
            if (m_imu_status[i].present && m_imu_data[i].valid && fresh) {
                values[count++] = m_imu_data[i].gyro[axis];
            }
        }
        if (count > 0) {
            gyro_med[axis] = calculate_median(values, count);
            has_reference = true;
        }

        count = 0;
        for (uint8_t i = 0; i < m_num_imus; i++) {
            bool fresh = (newest_ts - m_imu_data[i].timestamp_us) <=
                         CONFIG_UAV_IMU_MAX_TIMESTAMP_SKEW_US;
            if (m_imu_status[i].present && m_imu_data[i].valid && fresh) {
                values[count++] = m_imu_data[i].accel[axis];
            }
        }
        if (count > 0) {
            accel_med[axis] = calculate_median(values, count);
            has_reference = true;
        }
    }

    if (!has_reference) {
        return;
    }

    /* Check each IMU against reference with hysteresis */
    for (uint8_t i = 0; i < m_num_imus; i++) {
        if (!m_imu_status[i].present) {
            m_imu_status[i].functional = false;
            m_imu_status[i].selected = false;
            continue;
        }

        bool fresh = m_imu_data[i].valid &&
                     (newest_ts - m_imu_data[i].timestamp_us) <=
                     CONFIG_UAV_IMU_MAX_TIMESTAMP_SKEW_US;

        bool fault_detected = true;
        if (fresh) {
            float score = compute_residual_score(i, gyro_med, accel_med);
            fault_detected = score > 1.0f;
        }

        if (fault_detected) {
            m_imu_status[i].fault_count++;
            m_imu_status[i].recovery_count = 0;
            m_imu_status[i].error_samples++;

            if (m_imu_status[i].fault_count >= CONFIG_UAV_IMU_FAULT_COUNT_THRESHOLD) {
                if (m_imu_status[i].functional) {
                    syslog(LOG_WARNING, "[imu_fusion] IMU %d marked as non-functional\n", i);
                }
                m_imu_status[i].functional = false;
            }
        } else {
            if (m_imu_status[i].fault_count > 0) {
                m_imu_status[i].fault_count--;
            }

            m_imu_status[i].recovery_count++;

            if (!m_imu_status[i].functional &&
                m_imu_status[i].recovery_count >= CONFIG_UAV_IMU_RECOVERY_COUNT_THRESHOLD) {
                m_imu_status[i].functional = true;
                m_imu_status[i].fault_count = 0;
                syslog(LOG_INFO, "[imu_fusion] IMU %d recovered\n", i);
            }

            if (m_imu_status[i].functional) {
                m_imu_status[i].selected = true;
            }
        }
    }
}

float ImuFusion::calculate_median(float values[], uint8_t count) const
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
    if (!m_last_imu_data[imu_index].valid) {
        return;
    }

    /* Noise estimate từ biến thiên gyro + accel */
    const float alpha = 0.02f;

    float gyro_diff_sq = 0.0f;
    float accel_diff_sq = 0.0f;

    for (int j = 0; j < 3; j++) {
        float dg = data.gyro[j] - m_last_imu_data[imu_index].gyro[j];
        float da = data.accel[j] - m_last_imu_data[imu_index].accel[j];
        gyro_diff_sq += dg * dg;
        accel_diff_sq += da * da;
    }

    float gyro_rms = sqrtf(gyro_diff_sq / 3.0f);
    float accel_rms = sqrtf(accel_diff_sq / 3.0f);

    /* Quy đổi accel về cùng thang ảnh hưởng với gyro để ra chỉ số noise hỗn hợp */
    float composite_noise = gyro_rms + 0.02f * accel_rms;
    if (composite_noise < 1e-4f) {
        composite_noise = 1e-4f;
    }

    m_imu_status[imu_index].noise_estimate =
        (1.0f - alpha) * m_imu_status[imu_index].noise_estimate + alpha * composite_noise;
}

void ImuFusion::update_weights()
{
    float quality_sum = 0.0f;

    for (uint8_t i = 0; i < m_num_imus; i++) {
        if (m_imu_status[i].selected) {
            float noise = m_imu_status[i].noise_estimate;
            if (noise < 1e-4f) {
                noise = 1e-4f;
            }

            float noise_quality = 1.0f / noise;
            float health_quality = 1.0f / (1.0f + 0.25f * m_imu_status[i].fault_count);
            float quality = noise_quality * health_quality;

            m_imu_status[i].weight = quality;
            quality_sum += quality;
        } else {
            m_imu_status[i].weight = 0.0f;
        }
    }

    if (quality_sum <= kEps) {
        /* fallback uniform cho các IMU selected */
        uint8_t selected_count = 0;
        for (uint8_t i = 0; i < m_num_imus; i++) {
            if (m_imu_status[i].selected) {
                selected_count++;
            }
        }

        if (selected_count > 0) {
            float uniform = 1.0f / selected_count;
            for (uint8_t i = 0; i < m_num_imus; i++) {
                if (m_imu_status[i].selected) {
                    m_imu_status[i].weight = uniform;
                }
            }
        }

        return;
    }

    for (uint8_t i = 0; i < m_num_imus; i++) {
        if (m_imu_status[i].selected) {
            m_imu_status[i].weight /= quality_sum;
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
        m_imu_status[i].recovery_count = 0;
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
        printf("    IMU %d: %s/%s (samples=%lu, errors=%lu, fcnt=%lu, rcnt=%lu, noise=%.4f, weight=%.2f)\n",
               i,
               s.present ? "PRESENT" : "ABSENT",
               s.functional ? "OK" : "FAIL",
               (unsigned long)s.total_samples,
               (unsigned long)s.error_samples,
               (unsigned long)s.fault_count,
               (unsigned long)s.recovery_count,
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
    uint64_t last_ts = samples[0].timestamp_us;

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
