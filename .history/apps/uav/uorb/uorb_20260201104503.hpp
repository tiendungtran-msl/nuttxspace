/****************************************************************************
 * apps/uav/uorb/uorb.hpp
 *
 * uORB - Hệ thống Publish/Subscribe cho UAV
 *
 * MỤC ĐÍCH:
 * - Cung cấp API giao tiếp giữa các app
 * - Thread-safe với mutex protection
 * - Zero-copy không khả thi trên MCU, dùng copy semantics
 *
 * THIẾT KẾ:
 * - Topic được lưu trong registry toàn cục
 * - Mỗi topic có ring buffer chứa N samples gần nhất
 * - Sequence number để detect cập nhật
 * - File descriptor based API (tương thích poll())
 *
 * API STYLE:
 * - orb_* functions giống PX4 để dễ port code
 * - Trả về -1 và set errno khi lỗi
 *
 ****************************************************************************/

#pragma once

#include "orb_defines.hpp"
#include <pthread.h>

namespace uorb
{

/****************************************************************************
 * orb_advertise - Đăng ký publisher cho topic
 *
 * @param meta      Pointer tới metadata (dùng ORB_ID(name))
 * @param data      Dữ liệu ban đầu (có thể nullptr)
 * @return          Handle để publish, nullptr nếu lỗi
 *
 * VÍ DỤ:
 *   sensor_imu_s imu = {};
 *   orb_advert_t pub = orb_advertise(ORB_ID(sensor_imu), &imu);
 ****************************************************************************/

orb_advert_t orb_advertise(orb_id_t meta, const void* data);

/****************************************************************************
 * orb_advertise_multi - Đăng ký publisher cho topic instance cụ thể
 *
 * @param meta      Pointer tới metadata
 * @param data      Dữ liệu ban đầu
 * @param instance  Instance index (0..ORB_MAX_INSTANCES-1)
 * @return          Handle để publish
 *
 * VÍ DỤ:
 *   // Publish cho IMU thứ 2 (index 1)
 *   orb_advert_t pub = orb_advertise_multi(ORB_ID(sensor_imu), &imu, 1);
 ****************************************************************************/

orb_advert_t orb_advertise_multi(orb_id_t meta, const void* data, int instance);

/****************************************************************************
 * orb_unadvertise - Hủy đăng ký publisher
 *
 * @param handle    Handle từ orb_advertise
 * @return          0 nếu thành công, -1 nếu lỗi
 ****************************************************************************/

int orb_unadvertise(orb_advert_t handle);

/****************************************************************************
 * orb_publish - Publish dữ liệu mới lên topic
 *
 * @param meta      Pointer tới metadata
 * @param handle    Handle từ orb_advertise
 * @param data      Pointer tới dữ liệu mới
 * @return          0 nếu thành công, -1 nếu lỗi
 *
 * QUAN TRỌNG:
 * - Data được copy vào ring buffer
 * - Thread-safe, có thể gọi từ nhiều thread
 * - Không block, luôn trả về ngay
 ****************************************************************************/

int orb_publish(orb_id_t meta, orb_advert_t handle, const void* data);

/****************************************************************************
 * orb_subscribe - Đăng ký subscriber cho topic (instance 0)
 *
 * @param meta      Pointer tới metadata
 * @return          File descriptor >= 0, hoặc -1 nếu lỗi
 *
 * VÍ DỤ:
 *   int sub = orb_subscribe(ORB_ID(sensor_imu));
 *   if (sub >= 0) {
 *       // Success
 *   }
 ****************************************************************************/

int orb_subscribe(orb_id_t meta);

/****************************************************************************
 * orb_subscribe_multi - Đăng ký subscriber cho topic instance cụ thể
 *
 * @param meta      Pointer tới metadata
 * @param instance  Instance index
 * @return          File descriptor >= 0, hoặc -1 nếu lỗi
 ****************************************************************************/

int orb_subscribe_multi(orb_id_t meta, int instance);

/****************************************************************************
 * orb_unsubscribe - Hủy đăng ký subscriber
 *
 * @param fd        File descriptor từ orb_subscribe
 * @return          0 nếu thành công, -1 nếu lỗi
 ****************************************************************************/

int orb_unsubscribe(int fd);

/****************************************************************************
 * orb_copy - Copy dữ liệu mới nhất từ topic
 *
 * @param meta      Pointer tới metadata
 * @param fd        File descriptor từ orb_subscribe
 * @param buffer    Buffer để copy data vào
 * @return          0 nếu thành công, -1 nếu lỗi
 *
 * QUAN TRỌNG:
 * - Buffer phải đủ lớn (>= meta->size)
 * - Luôn copy sample mới nhất
 * - Cập nhật internal sequence để orb_check biết đã đọc
 ****************************************************************************/

int orb_copy(orb_id_t meta, int fd, void* buffer);

/****************************************************************************
 * orb_check - Kiểm tra có dữ liệu mới không
 *
 * @param fd        File descriptor từ orb_subscribe
 * @param updated   Output: true nếu có data mới
 * @return          0 nếu thành công, -1 nếu lỗi
 *
 * VÍ DỤ:
 *   bool updated;
 *   if (orb_check(sub, &updated) == 0 && updated) {
 *       sensor_imu_s data;
 *       orb_copy(ORB_ID(sensor_imu), sub, &data);
 *   }
 ****************************************************************************/

int orb_check(int fd, bool* updated);

/****************************************************************************
 * orb_exists - Kiểm tra topic đã được advertise chưa
 *
 * @param meta      Pointer tới metadata
 * @param instance  Instance index
 * @return          true nếu topic tồn tại
 ****************************************************************************/

bool orb_exists(orb_id_t meta, int instance);

/****************************************************************************
 * orb_group_count - Đếm số instances của topic
 *
 * @param meta      Pointer tới metadata
 * @return          Số instances đã advertise
 ****************************************************************************/

int orb_group_count(orb_id_t meta);

} // namespace uorb

// Expose vào global namespace để dễ dùng
using uorb::orb_advertise;
using uorb::orb_advertise_multi;
using uorb::orb_unadvertise;
using uorb::orb_publish;
using uorb::orb_subscribe;
using uorb::orb_subscribe_multi;
using uorb::orb_unsubscribe;
using uorb::orb_copy;
using uorb::orb_check;
using uorb::orb_exists;
using uorb::orb_group_count;
