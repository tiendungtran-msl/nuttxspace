/****************************************************************************
 * apps/uav/uorb/orb_defines.hpp
 *
 * uORB - Định nghĩa macros và constants
 *
 * MỤC ĐÍCH:
 * - Định nghĩa macros để đăng ký topic
 * - Cung cấp type-safe API
 * - Tương thích với cách dùng của PX4
 *
 * CÁCH DÙNG:
 *   // Trong header định nghĩa message
 *   ORB_DECLARE(sensor_imu);
 *   
 *   // Trong file .cpp
 *   ORB_DEFINE(sensor_imu, sensor_imu_s);
 *
 ****************************************************************************/

#pragma once

#include <cstdint>
#include <cstddef>

namespace uorb
{

/****************************************************************************
 * orb_id_t - Topic identifier
 *
 * Mỗi topic có một ID duy nhất, được dùng để:
 * - Tìm topic trong registry
 * - Type checking tại compile time
 ****************************************************************************/

struct orb_metadata {
    const char* name;           // Tên topic (ví dụ: "sensor_imu")
    size_t size;                // Kích thước message (bytes)
    uint16_t queue_size;        // Số phần tử trong ring buffer
};

using orb_id_t = const orb_metadata*;

/****************************************************************************
 * orb_advert_t - Advertiser handle
 *
 * Trả về từ orb_advertise(), dùng cho orb_publish()
 ****************************************************************************/

using orb_advert_t = void*;

/****************************************************************************
 * Topic Registry
 *
 * Hệ thống hỗ trợ tối đa MAX_TOPICS topics đồng thời.
 * Mỗi topic có thể có MAX_INSTANCES instances (ví dụ: sensor_imu[0..3])
 ****************************************************************************/

static constexpr int ORB_MAX_TOPICS = 32;
static constexpr int ORB_MAX_INSTANCES = 4;
static constexpr int ORB_DEFAULT_QUEUE_SIZE = 8;

/****************************************************************************
 * Macros để định nghĩa topic
 *
 * ORB_DECLARE: Khai báo topic (trong header)
 * ORB_DEFINE:  Định nghĩa topic metadata (trong cpp)
 * ORB_ID:      Lấy orb_id_t từ tên topic
 ****************************************************************************/

// Khai báo extern cho metadata
#define ORB_DECLARE(name) \
    extern const uorb::orb_metadata __orb_##name

// Định nghĩa metadata
#define ORB_DEFINE(name, struct_type) \
    const uorb::orb_metadata __orb_##name = { \
        #name, \
        sizeof(struct_type), \
        uorb::ORB_DEFAULT_QUEUE_SIZE \
    }

// Định nghĩa với queue size tùy chỉnh
#define ORB_DEFINE_QUEUE(name, struct_type, queue_size) \
    const uorb::orb_metadata __orb_##name = { \
        #name, \
        sizeof(struct_type), \
        queue_size \
    }

// Lấy pointer tới metadata
#define ORB_ID(name) (&__orb_##name)

} // namespace uorb
