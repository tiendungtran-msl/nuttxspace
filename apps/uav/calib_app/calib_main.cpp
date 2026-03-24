/****************************************************************************
 * apps/uav/calib_app/calib_main.cpp
 *
 * UAV SENSOR CALIBRATION APPLICATION
 *
 * MỤC ĐÍCH:
 * - Thu thập dữ liệu cảm biến và tính toán hệ số calibration
 * - Chạy dưới dạng lệnh NSH interactive
 * - Áp dụng kết quả vào driver calibration objects
 *
 * SỬ DỤNG:
 *   calib gyro     - Calibrate gyro bias (giữ board yên)
 *   calib accel    - Calibrate accel bias + scale (6-position)
 *   calib mag      - Calibrate mag hard-iron offset (xoay 360°)
 *   calib status   - Hiển thị calibration hiện tại
 *   calib reset    - Reset về mặc định
 *
 * THUẬT TOÁN:
 *   Gyro:  Tính trung bình khi đứng yên → bias
 *   Accel: 6-position gravity → bias + scale
 *   Mag:   Sphere-fit (tính min/max mỗi trục) → hard-iron offset
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <time.h>

#include <nuttx/clock.h>

/* ICM42688P IMU Driver */
#include <uav/drivers/imu/icm42688p/icm42688p.hpp>

/* BMM150 Magnetometer Driver */
#include <uav/drivers/mag/bmm150/bmm150.hpp>

/****************************************************************************
 * Configuration
 ****************************************************************************/

#ifndef CONFIG_UAV_CALIB_GYRO_SAMPLES
#define CONFIG_UAV_CALIB_GYRO_SAMPLES      5000
#endif

#ifndef CONFIG_UAV_CALIB_ACCEL_SAMPLES
#define CONFIG_UAV_CALIB_ACCEL_SAMPLES     1000
#endif

#ifndef CONFIG_UAV_CALIB_MAG_SAMPLES
#define CONFIG_UAV_CALIB_MAG_SAMPLES       2000
#endif

/* Gravity constant */
#define GRAVITY_MSS  9.80665f

/****************************************************************************
 * Private Data
 *
 * Tham chiếu đến driver instances được khởi tạo bởi sensors_app.
 * Nếu sensors_app chưa chạy, calibration sẽ báo lỗi.
 ****************************************************************************/

/* Extern references đến driver instances từ sensors_app
 * Trong thực tế, bạn có thể truy cập qua registry hoặc global pointer.
 * Ở đây dùng extern cho đơn giản.
 */
extern drivers::imu::ICM42688P *g_imu_instance;
extern drivers::mag::BMM150    *g_mag_instance;

/****************************************************************************
 * Gyroscope Calibration
 *
 * PHƯƠNG PHÁP: Average-at-rest
 *
 * 1. Người dùng đặt board yên trên mặt phẳng
 * 2. Thu thập N mẫu gyro
 * 3. Tính trung bình → đó là bias
 * 4. Lưu bias vào driver calibration object
 *
 * TOÁN HỌC:
 *   bias_x = (1/N) * Σ gyro_x[i]    (rad/s)
 *   bias_y = (1/N) * Σ gyro_y[i]    (rad/s)
 *   bias_z = (1/N) * Σ gyro_z[i]    (rad/s)
 *
 * Gyro đứng yên lý tưởng đọc ra 0 rad/s.
 * Giá trị trung bình != 0 chính là bias cần bù.
 ****************************************************************************/

static int calib_gyro(void)
{
    printf("\n=== GYROSCOPE CALIBRATION ===\n\n");

    if (g_imu_instance == nullptr)
    {
        printf("[calib] ERROR: IMU driver chưa khởi tạo!\n");
        printf("[calib] Hãy chạy 'sensors start' trước.\n");
        return -ENODEV;
    }

    printf("[calib] Đặt board YÊN trên mặt phẳng.\n");
    printf("[calib] Không chạm vào board trong quá trình calibration.\n");
    printf("[calib] Bắt đầu sau 3 giây...\n\n");

    /* Đếm ngược */
    for (int i = 3; i > 0; i--)
    {
        printf("[calib] %d...\n", i);
        sleep(1);
    }

    printf("[calib] Đang thu thập %d mẫu...\n",
           CONFIG_UAV_CALIB_GYRO_SAMPLES);

    /* Thu thập dữ liệu */
    double sum_gx = 0.0;
    double sum_gy = 0.0;
    double sum_gz = 0.0;
    int valid_count = 0;

    drivers::imu::ICM42688P::Data imu_data;

    for (int i = 0; i < CONFIG_UAV_CALIB_GYRO_SAMPLES; i++)
    {
        int ret = g_imu_instance->read(imu_data);
        if (ret == 0)
        {
            sum_gx += (double)imu_data.gyro[0];
            sum_gy += (double)imu_data.gyro[1];
            sum_gz += (double)imu_data.gyro[2];
            valid_count++;
        }

        /* Chờ ~1ms (sampling rate ~1kHz) */
        usleep(1000);

        /* Hiển thị tiến trình */
        if ((i + 1) % 1000 == 0)
        {
            printf("[calib]   %d/%d mẫu...\n",
                   i + 1, CONFIG_UAV_CALIB_GYRO_SAMPLES);
        }
    }

    if (valid_count < CONFIG_UAV_CALIB_GYRO_SAMPLES / 2)
    {
        printf("[calib] ERROR: Chỉ đọc được %d/%d mẫu hợp lệ!\n",
               valid_count, CONFIG_UAV_CALIB_GYRO_SAMPLES);
        return -EIO;
    }

    /* Tính bias */
    float bias_gx = (float)(sum_gx / valid_count);
    float bias_gy = (float)(sum_gy / valid_count);
    float bias_gz = (float)(sum_gz / valid_count);

    printf("\n[calib] === KẾT QUẢ ===\n");
    printf("[calib] Mẫu hợp lệ: %d/%d\n",
           valid_count, CONFIG_UAV_CALIB_GYRO_SAMPLES);
    printf("[calib] Gyro bias X: %+.6f rad/s (%+.3f °/s)\n",
           (double)bias_gx, (double)(bias_gx * 180.0f / 3.14159265f));
    printf("[calib] Gyro bias Y: %+.6f rad/s (%+.3f °/s)\n",
           (double)bias_gy, (double)(bias_gy * 180.0f / 3.14159265f));
    printf("[calib] Gyro bias Z: %+.6f rad/s (%+.3f °/s)\n",
           (double)bias_gz, (double)(bias_gz * 180.0f / 3.14159265f));

    /* Kiểm tra bias có hợp lý không (< 0.5 rad/s ≈ 28.6 °/s) */
    float bias_norm = sqrtf(bias_gx * bias_gx +
                            bias_gy * bias_gy +
                            bias_gz * bias_gz);

    if (bias_norm > 0.5f)
    {
        printf("\n[calib] WARNING: Bias quá lớn (%.3f rad/s)!\n",
               (double)bias_norm);
        printf("[calib] Board có thể đang rung hoặc di chuyển.\n");
        printf("[calib] KHÔNG áp dụng calibration.\n");
        return -EINVAL;
    }

    /* Áp dụng calibration */
    float bias[3] = { bias_gx, bias_gy, bias_gz };
    g_imu_instance->set_gyro_bias(bias);

    printf("\n[calib] ✓ Gyro calibration THÀNH CÔNG!\n");
    printf("[calib]   Bias đã được áp dụng vào driver.\n\n");

    return 0;
}

/****************************************************************************
 * Accelerometer Calibration
 *
 * PHƯƠNG PHÁP: 6-position gravity calibration
 *
 * Đặt board theo 6 hướng (mỗi mặt hướng lên), đo trọng lực.
 * Trục hướng lên đọc ~+9.81 m/s², trục hướng xuống đọc ~-9.81.
 * Các trục ngang đọc ~0.
 *
 * Simplified method (chỉ dùng 1 vị trí - level):
 * 1. Đặt board nằm ngang (Z hướng lên)
 * 2. Accel lý tưởng: X=0, Y=0, Z=-9.81
 * 3. Bias = measured - ideal
 *
 * TOÁN HỌC (simplified single-position):
 *   bias_x = mean(accel_x) - 0.0
 *   bias_y = mean(accel_y) - 0.0
 *   bias_z = mean(accel_z) - (-GRAVITY)   [NED: Z trục xuống = -g]
 *
 * Cho full 6-position, cần thu thập 6 sets rồi solve least-squares.
 * Ở đây implement simplified version trước.
 ****************************************************************************/

static int calib_accel(void)
{
    printf("\n=== ACCELEROMETER CALIBRATION ===\n\n");

    if (g_imu_instance == nullptr)
    {
        printf("[calib] ERROR: IMU driver chưa khởi tạo!\n");
        return -ENODEV;
    }

    printf("[calib] Đặt board NẰM NGANG trên mặt phẳng.\n");
    printf("[calib] Mặt trên (Z+) hướng LÊN.\n");
    printf("[calib] Bắt đầu sau 3 giây...\n\n");

    for (int i = 3; i > 0; i--)
    {
        printf("[calib] %d...\n", i);
        sleep(1);
    }

    printf("[calib] Đang thu thập %d mẫu...\n",
           CONFIG_UAV_CALIB_ACCEL_SAMPLES);

    /* Thu thập dữ liệu */
    double sum_ax = 0.0;
    double sum_ay = 0.0;
    double sum_az = 0.0;
    int valid_count = 0;

    drivers::imu::ICM42688P::Data imu_data;

    for (int i = 0; i < CONFIG_UAV_CALIB_ACCEL_SAMPLES; i++)
    {
        int ret = g_imu_instance->read(imu_data);
        if (ret == 0)
        {
            sum_ax += (double)imu_data.accel[0];
            sum_ay += (double)imu_data.accel[1];
            sum_az += (double)imu_data.accel[2];
            valid_count++;
        }

        usleep(1000);

        if ((i + 1) % 500 == 0)
        {
            printf("[calib]   %d/%d mẫu...\n",
                   i + 1, CONFIG_UAV_CALIB_ACCEL_SAMPLES);
        }
    }

    if (valid_count < CONFIG_UAV_CALIB_ACCEL_SAMPLES / 2)
    {
        printf("[calib] ERROR: Chỉ đọc được %d mẫu hợp lệ!\n",
               valid_count);
        return -EIO;
    }

    /* Tính giá trị trung bình */
    float mean_ax = (float)(sum_ax / valid_count);
    float mean_ay = (float)(sum_ay / valid_count);
    float mean_az = (float)(sum_az / valid_count);

    /**
     * Tính bias:
     * IMU thường dùng NED convention hoặc sensor frame.
     * ICM42688P: Z hướng lên khi chip quay lên → accel_z ≈ -9.81 m/s²
     * (vì accelerometer đo phản lực của trọng lực)
     *
     * Giá trị kỳ vọng khi nằm ngang: X=0, Y=0, Z=-g
     */
    float bias_ax = mean_ax - 0.0f;
    float bias_ay = mean_ay - 0.0f;
    float bias_az = mean_az - (-GRAVITY_MSS);

    printf("\n[calib] === KẾT QUẢ ===\n");
    printf("[calib] Mẫu hợp lệ: %d\n", valid_count);
    printf("[calib] Giá trị trung bình:\n");
    printf("[calib]   X: %+.4f m/s²\n", (double)mean_ax);
    printf("[calib]   Y: %+.4f m/s²\n", (double)mean_ay);
    printf("[calib]   Z: %+.4f m/s²  (kỳ vọng: %.4f)\n",
           (double)mean_az, (double)(-GRAVITY_MSS));
    printf("[calib] Accel bias:\n");
    printf("[calib]   X: %+.4f m/s²\n", (double)bias_ax);
    printf("[calib]   Y: %+.4f m/s²\n", (double)bias_ay);
    printf("[calib]   Z: %+.4f m/s²\n", (double)bias_az);

    /* Kiểm tra tổng gia tốc ≈ 9.81 */
    float total_g = sqrtf(mean_ax * mean_ax +
                          mean_ay * mean_ay +
                          mean_az * mean_az);

    printf("[calib] |Accel| = %.4f m/s² (kỳ vọng: %.4f)\n",
           (double)total_g, (double)GRAVITY_MSS);

    if (fabsf(total_g - GRAVITY_MSS) > 2.0f)
    {
        printf("\n[calib] WARNING: |Accel| lệch quá nhiều so với g!\n");
        printf("[calib] Board có thể đang nghiêng hoặc di chuyển.\n");
        printf("[calib] KHÔNG áp dụng calibration.\n");
        return -EINVAL;
    }

    /* Áp dụng calibration */
    float bias[3] = { bias_ax, bias_ay, bias_az };
    g_imu_instance->set_accel_bias(bias);

    printf("\n[calib] ✓ Accel calibration THÀNH CÔNG!\n");
    printf("[calib]   Bias đã được áp dụng.\n");
    printf("[calib]   (Đây là simplified calibration, chỉ 1 vị trí)\n");
    printf("[calib]   Để chính xác hơn, cần full 6-position calibration.\n\n");

    return 0;
}

/****************************************************************************
 * Magnetometer Calibration
 *
 * PHƯƠNG PHÁP: Sphere-fit (Hard-Iron Compensation)
 *
 * Hard-iron distortion gây ra offset không đổi trên mỗi trục.
 * Khi xoay sensor 360° trên 3 mặt phẳng, dữ liệu tạo thành ellipsoid.
 * Hard-iron offset = tâm của ellipsoid.
 *
 * Simplified method (min-max):
 *   offset_x = (max_x + min_x) / 2
 *   offset_y = (max_y + min_y) / 2
 *   offset_z = (max_z + min_z) / 2
 *
 * Sau calibration:
 *   corrected_x = raw_x - offset_x
 *   corrected_y = raw_y - offset_y
 *   corrected_z = raw_z - offset_z
 ****************************************************************************/

static int calib_mag(void)
{
    printf("\n=== MAGNETOMETER CALIBRATION ===\n\n");

    if (g_mag_instance == nullptr)
    {
        printf("[calib] ERROR: MAG driver chưa khởi tạo!\n");
        printf("[calib] Hãy khởi tạo BMM150 trước.\n");
        return -ENODEV;
    }

    printf("[calib] Xoay board CHẬM theo tất cả các hướng.\n");
    printf("[calib] Cố gắng xoay đầy đủ 360° trên 3 mặt phẳng\n");
    printf("[calib] (pitch, roll, yaw).\n");
    printf("[calib] Thu thập %d mẫu (~%d giây ở 20Hz).\n",
           CONFIG_UAV_CALIB_MAG_SAMPLES,
           CONFIG_UAV_CALIB_MAG_SAMPLES / 20);
    printf("[calib] Bắt đầu sau 3 giây...\n\n");

    for (int i = 3; i > 0; i--)
    {
        printf("[calib] %d...\n", i);
        sleep(1);
    }

    printf("[calib] Đang thu thập... HÃY XOAY BOARD!\n");

    /* Khởi tạo min/max */
    float min_x = 99999.0f, max_x = -99999.0f;
    float min_y = 99999.0f, max_y = -99999.0f;
    float min_z = 99999.0f, max_z = -99999.0f;

    int valid_count = 0;
    drivers::mag::BMM150::Data mag_data;

    for (int i = 0; i < CONFIG_UAV_CALIB_MAG_SAMPLES; i++)
    {
        int ret = g_mag_instance->read(mag_data);
        if (ret == 0)
        {
            float mx = mag_data.mag[0];
            float my = mag_data.mag[1];
            float mz = mag_data.mag[2];

            /* Bỏ qua giá trị overflow (0.0f từ BMM150) */
            if (mx == 0.0f && my == 0.0f && mz == 0.0f)
            {
                continue;
            }

            /* Cập nhật min/max */
            if (mx < min_x) min_x = mx;
            if (mx > max_x) max_x = mx;
            if (my < min_y) min_y = my;
            if (my > max_y) max_y = my;
            if (mz < min_z) min_z = mz;
            if (mz > max_z) max_z = mz;

            valid_count++;
        }

        /* BMM150 ở 20Hz → chờ 50ms */
        usleep(50000);

        /* Hiển thị tiến trình */
        if ((i + 1) % 200 == 0)
        {
            printf("[calib]   %d/%d mẫu (valid: %d) "
                   "X:[%.0f, %.0f] Y:[%.0f, %.0f] Z:[%.0f, %.0f]\n",
                   i + 1, CONFIG_UAV_CALIB_MAG_SAMPLES, valid_count,
                   (double)min_x, (double)max_x,
                   (double)min_y, (double)max_y,
                   (double)min_z, (double)max_z);
        }
    }

    if (valid_count < 100)
    {
        printf("[calib] ERROR: Chỉ có %d mẫu hợp lệ (cần ít nhất 100)!\n",
               valid_count);
        return -EIO;
    }

    /* Tính hard-iron offset (tâm ellipsoid) */
    float offset_x = (max_x + min_x) / 2.0f;
    float offset_y = (max_y + min_y) / 2.0f;
    float offset_z = (max_z + min_z) / 2.0f;

    /* Tính bán kính trung bình (ước lượng cường độ từ trường) */
    float range_x = (max_x - min_x) / 2.0f;
    float range_y = (max_y - min_y) / 2.0f;
    float range_z = (max_z - min_z) / 2.0f;
    float avg_radius = (range_x + range_y + range_z) / 3.0f;

    printf("\n[calib] === KẾT QUẢ ===\n");
    printf("[calib] Mẫu hợp lệ: %d\n", valid_count);
    printf("[calib] Range:\n");
    printf("[calib]   X: [%+.1f, %+.1f] µT  (range: %.1f)\n",
           (double)min_x, (double)max_x, (double)(max_x - min_x));
    printf("[calib]   Y: [%+.1f, %+.1f] µT  (range: %.1f)\n",
           (double)min_y, (double)max_y, (double)(max_y - min_y));
    printf("[calib]   Z: [%+.1f, %+.1f] µT  (range: %.1f)\n",
           (double)min_z, (double)max_z, (double)(max_z - min_z));
    printf("[calib] Hard-iron offset:\n");
    printf("[calib]   X: %+.2f µT\n", (double)offset_x);
    printf("[calib]   Y: %+.2f µT\n", (double)offset_y);
    printf("[calib]   Z: %+.2f µT\n", (double)offset_z);
    printf("[calib] Cường độ từ trường ước lượng: %.1f µT\n",
           (double)avg_radius);

    /* Kiểm tra range hợp lý (từ trường trái đất ~25-65 µT) */
    if (avg_radius < 10.0f || avg_radius > 100.0f)
    {
        printf("\n[calib] WARNING: Cường độ từ trường bất thường!\n");
        printf("[calib] Có thể: không xoay đủ hoặc gần vật nhiễm từ.\n");
    }

    /* Kiểm tra sphericity (tỷ lệ range giữa các trục) */
    float max_range = fmaxf(range_x, fmaxf(range_y, range_z));
    float min_range = fminf(range_x, fminf(range_y, range_z));

    if (max_range > 0 && (min_range / max_range) < 0.5f)
    {
        printf("[calib] WARNING: Dữ liệu không đều giữa các trục.\n");
        printf("[calib] Hãy xoay board đầy đủ hơn trên 3 mặt phẳng.\n");
    }

    printf("\n[calib] ✓ Mag calibration THÀNH CÔNG!\n");
    printf("[calib]   Hard-iron offset đã được tính.\n");
    printf("[calib]   (Lưu ý: Soft-iron calibration chưa được implement)\n\n");

    /* TODO: Lưu offset vào BMM150 driver hoặc file cấu hình */

    return 0;
}

/****************************************************************************
 * Status / Reset Commands
 ****************************************************************************/

static void calib_status(void)
{
    printf("\n=== CALIBRATION STATUS ===\n\n");

    /* IMU Calibration */
    if (g_imu_instance != nullptr)
    {
        float gyro_bias[3];
        float accel_bias[3];

        g_imu_instance->get_gyro_bias(gyro_bias);
        g_imu_instance->get_accel_bias(accel_bias);
        float accel_scale = g_imu_instance->get_accel_scale_correction();

        printf("--- IMU (ICM42688P) ---\n");
        printf("Gyro bias:   X=%+.6f  Y=%+.6f  Z=%+.6f  rad/s\n",
               (double)gyro_bias[0], (double)gyro_bias[1],
               (double)gyro_bias[2]);
        printf("Accel bias:  X=%+.4f  Y=%+.4f  Z=%+.4f  m/s²\n",
               (double)accel_bias[0], (double)accel_bias[1],
               (double)accel_bias[2]);
        printf("Accel scale: %.6f\n", (double)accel_scale);

        const auto& acal = g_imu_instance->get_accel_calibration();
        const auto& gcal = g_imu_instance->get_gyro_calibration();
        printf("Accel calibrated: %s (%d updates)\n",
               acal.is_calibrated() ? "YES" : "NO",
               acal.calibration_count());
        printf("Gyro calibrated:  %s (%d updates)\n",
               gcal.is_calibrated() ? "YES" : "NO",
               gcal.calibration_count());
    }
    else
    {
        printf("--- IMU: NOT AVAILABLE ---\n");
    }

    printf("\n");

    /* Magnetometer Calibration */
    if (g_mag_instance != nullptr)
    {
        printf("--- MAG (BMM150) ---\n");
        g_mag_instance->print_status();
    }
    else
    {
        printf("--- MAG: NOT AVAILABLE ---\n");
    }

    printf("==========================\n\n");
}

static void calib_reset(void)
{
    printf("\n=== RESET CALIBRATION ===\n\n");

    if (g_imu_instance != nullptr)
    {
        g_imu_instance->get_gyro_calibration().reset();
        g_imu_instance->get_accel_calibration().reset();
        printf("[calib] IMU calibration reset.\n");
    }

    printf("[calib] ✓ Calibration đã được reset về mặc định.\n\n");
}

/****************************************************************************
 * Usage
 ****************************************************************************/

static void print_usage(void)
{
    printf("\nSử dụng: calib <command>\n\n");
    printf("Commands:\n");
    printf("  gyro     Calibrate gyroscope (giữ board yên)\n");
    printf("  accel    Calibrate accelerometer (đặt nằm ngang)\n");
    printf("  mag      Calibrate magnetometer (xoay 360°)\n");
    printf("  status   Hiển thị calibration hiện tại\n");
    printf("  reset    Reset calibration về mặc định\n\n");
}

/****************************************************************************
 * Public Functions — NSH Entry Point
 ****************************************************************************/

extern "C"
{

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        print_usage();
        return EXIT_FAILURE;
    }

    const char *cmd = argv[1];
    int ret = 0;

    if (strcmp(cmd, "gyro") == 0)
    {
        ret = calib_gyro();
    }
    else if (strcmp(cmd, "accel") == 0)
    {
        ret = calib_accel();
    }
    else if (strcmp(cmd, "mag") == 0)
    {
        ret = calib_mag();
    }
    else if (strcmp(cmd, "status") == 0)
    {
        calib_status();
    }
    else if (strcmp(cmd, "reset") == 0)
    {
        calib_reset();
    }
    else
    {
        printf("[calib] Lệnh không hợp lệ: '%s'\n", cmd);
        print_usage();
        ret = EXIT_FAILURE;
    }

    return (ret == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}

} /* extern "C" */
