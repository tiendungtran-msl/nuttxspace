/****************************************************************************
 * apps/uav/lib/sensor_processing/imu_fusion.hpp
 *
 * MULTI-IMU FUSION - Kết hợp dữ liệu từ nhiều IMU
 *
 * MỤC ĐÍCH:
 * - Kết hợp dữ liệu từ 4 IMU để tăng độ chính xác
 * - Phát hiện và loại trừ IMU bị lỗi (voting)
 * - Cung cấp redundancy cho an toàn bay
 * - Ước lượng noise của từng IMU để weighted averaging
 *
 * THIẾT KẾ:
 * - Ba chế độ fusion: VOTING, WEIGHTED, PRIMARY
 * - VOTING: Sử dụng median - robust với 1 sensor lỗi
 * - WEIGHTED: Average có trọng số theo noise estimate
 * - PRIMARY: Dùng IMU chính, fallback khi lỗi
 *
 * FAULT DETECTION:
 * - So sánh mỗi IMU với median
 * - IMU lệch quá ngưỡng bị đánh dấu faulty
 * - Sau N samples liên tiếp lệch → disable IMU đó
 *
 ****************************************************************************/

#ifndef __UAV_LIB_SENSOR_PROCESSING_IMU_FUSION_HPP
#define __UAV_LIB_SENSOR_PROCESSING_IMU_FUSION_HPP

#include <stdint.h>
#include <string.h>
#include <math.h>

#ifndef CONFIG_UAV_NUM_IMUS
#define CONFIG_UAV_NUM_IMUS 4
#endif

namespace uav {
namespace sensor_processing {

/****************************************************************************
 * Configuration
 ****************************************************************************/

/* Ngưỡng phát hiện lỗi (rad/s cho gyro, m/s² cho accel) */
#ifndef CONFIG_UAV_IMU_FAULT_THRESHOLD_GYRO
#define CONFIG_UAV_IMU_FAULT_THRESHOLD_GYRO     0.1f    /* rad/s */
#endif

#ifndef CONFIG_UAV_IMU_FAULT_THRESHOLD_ACCEL
#define CONFIG_UAV_IMU_FAULT_THRESHOLD_ACCEL    2.0f    /* m/s² */
#endif

/* Số samples liên tiếp lệch để disable IMU */
#ifndef CONFIG_UAV_IMU_FAULT_COUNT_THRESHOLD
#define CONFIG_UAV_IMU_FAULT_COUNT_THRESHOLD    10
#endif

/* Số samples liên tiếp tốt để recover IMU sau khi bị loại */
#ifndef CONFIG_UAV_IMU_RECOVERY_COUNT_THRESHOLD
#define CONFIG_UAV_IMU_RECOVERY_COUNT_THRESHOLD 30
#endif

/* Chênh lệch timestamp tối đa giữa các IMU được phép fusion (us) */
#ifndef CONFIG_UAV_IMU_MAX_TIMESTAMP_SKEW_US
#define CONFIG_UAV_IMU_MAX_TIMESTAMP_SKEW_US    5000
#endif

/* Ngưỡng inlier cho robust fusion (score chuẩn hóa) */
#ifndef CONFIG_UAV_IMU_INLIER_GATE
#define CONFIG_UAV_IMU_INLIER_GATE              2.5f
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @brief Chế độ fusion
 */
enum class FusionMode : uint8_t {
    VOTING,     /* Median voting - robust với 1 sensor lỗi */
    WEIGHTED,   /* Weighted average theo noise estimate */
    PRIMARY     /* Dùng primary IMU, fallback khi lỗi */
};

/**
 * @brief Dữ liệu IMU đã xử lý
 */
struct ImuData {
    float gyro[3];          /**< Angular rate (rad/s) */
    float accel[3];         /**< Acceleration (m/s²) */
    float temperature;      /**< Temperature (°C) */
    uint64_t timestamp_us;  /**< Capture timestamp */
    uint8_t instance;       /**< IMU index (0-3) */
    bool valid;             /**< Data validity flag */
};

/**
 * @brief Trạng thái của một IMU
 */
struct ImuStatus {
    bool present;           /**< IMU có tồn tại không */
    bool functional;        /**< IMU có hoạt động không */
    bool selected;          /**< IMU có được chọn để fusion không */
    uint32_t fault_count;   /**< Số samples liên tiếp lệch */
    uint32_t recovery_count;/**< Số samples liên tiếp tốt */
    uint32_t total_samples; /**< Tổng samples đã nhận */
    uint32_t error_samples; /**< Tổng samples lỗi */
    float noise_estimate;   /**< Ước lượng noise level */
    float weight;           /**< Trọng số trong weighted average */
};

/**
 * @brief Kết quả fusion
 */
struct FusedImuData {
    float gyro[3];          /**< Fused angular rate (rad/s) */
    float accel[3];         /**< Fused acceleration (m/s²) */
    float temperature;      /**< Average temperature (°C) */
    uint64_t timestamp_us;  /**< Fusion timestamp */
    uint8_t num_imus_used;  /**< Số IMU được dùng trong fusion */
    uint8_t healthy_mask;   /**< Bitmask của IMU healthy */
    bool valid;             /**< Kết quả có valid không */
};

/****************************************************************************
 * ImuFusion Class
 ****************************************************************************/

class ImuFusion {
public:
    ImuFusion();
    ~ImuFusion() = default;

    /* Delete copy/move */
    ImuFusion(const ImuFusion&) = delete;
    ImuFusion& operator=(const ImuFusion&) = delete;

    /**
     * @brief Khởi tạo fusion module
     *
     * @param num_imus Số lượng IMU (1-4)
     * @return 0 nếu thành công
     */
    int init(uint8_t num_imus = CONFIG_UAV_NUM_IMUS);

    /**
     * @brief Set chế độ fusion
     *
     * @param mode Fusion mode
     */
    void set_mode(FusionMode mode);

    /**
     * @brief Lấy chế độ fusion hiện tại
     * @return FusionMode
     */
    FusionMode get_mode() const { return m_mode; }

    /**
     * @brief Set primary IMU cho PRIMARY mode
     *
     * @param imu_index Index của primary IMU (0-3)
     */
    void set_primary_imu(uint8_t imu_index);

    /**
     * @brief Đánh dấu IMU là present
     *
     * @param imu_index IMU index
     * @param present true nếu IMU có mặt
     */
    void set_imu_present(uint8_t imu_index, bool present);

    /**
     * @brief Cập nhật dữ liệu từ một IMU
     *
     * Gọi hàm này mỗi khi có sample mới từ một IMU.
     *
     * @param imu_index IMU index (0-3)
     * @param data Dữ liệu IMU
     */
    void update_imu(uint8_t imu_index, const ImuData& data);

    /**
     * @brief Thực hiện fusion và lấy kết quả
     *
     * Gọi hàm này sau khi đã update tất cả IMU.
     *
     * @param result Output fused data
     * @return 0 nếu thành công, -1 nếu không có IMU nào valid
     */
    int fuse(FusedImuData& result);

    /**
     * @brief Lấy trạng thái của một IMU
     *
     * @param imu_index IMU index
     * @return ImuStatus struct
     */
    ImuStatus get_imu_status(uint8_t imu_index) const;

    /**
     * @brief Lấy số IMU đang hoạt động
     * @return Số IMU functional
     */
    uint8_t get_healthy_count() const;

    /**
     * @brief Lấy healthy IMU bitmask
     * @return Bitmask
     */
    uint8_t get_healthy_mask() const;

    /**
     * @brief Reset trạng thái fault detection
     */
    void reset_fault_detection();

    /**
     * @brief In status debug
     */
    void print_status() const;

private:
    /* Internal fusion methods */
    void fuse_voting(FusedImuData& result);
    void fuse_weighted(FusedImuData& result);
    void fuse_primary(FusedImuData& result);
    void compute_reference_median(float gyro_ref[3], float accel_ref[3], float &temp_ref) const;
    float compute_residual_score(uint8_t imu_index,
                                 const float gyro_ref[3],
                                 const float accel_ref[3]) const;

    /* Fault detection */
    void check_faults();
    float calculate_median(float values[], uint8_t count) const;

    /* Noise estimation */
    void update_noise_estimate(uint8_t imu_index, const ImuData& data);
    void update_weights();

    /* State */
    ImuData m_imu_data[CONFIG_UAV_NUM_IMUS];
    ImuStatus m_imu_status[CONFIG_UAV_NUM_IMUS];
    ImuData m_last_imu_data[CONFIG_UAV_NUM_IMUS];  /* Cho noise estimation */

    FusionMode m_mode;
    uint8_t m_num_imus;
    uint8_t m_primary_imu;
    bool m_initialized;
};

/****************************************************************************
 * Coning/Sculling Compensation
 *
 * Bù lỗi do rotation/vibration trong quá trình tích phân IMU.
 * Quan trọng cho dead reckoning chính xác.
 *
 ****************************************************************************/

/**
 * @brief Delta state cho integration
 */
struct DeltaState {
    float delta_angle[3];       /**< Tích phân góc (rad) */
    float delta_velocity[3];    /**< Tích phân vận tốc (m/s) */
    float dt;                   /**< Delta time (s) */
    uint64_t timestamp_us;
};

/**
 * @brief Integrate IMU data với coning/sculling compensation
 *
 * @param samples   Array of IMU samples để integrate
 * @param count     Số samples
 * @param output    Output delta state
 */
void integrate_imu_samples(const ImuData* samples, uint8_t count, DeltaState& output);

} /* namespace sensor_processing */
} /* namespace uav */

#endif /* __UAV_LIB_SENSOR_PROCESSING_IMU_FUSION_HPP */
