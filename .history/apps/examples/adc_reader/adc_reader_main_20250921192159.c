/****************************************************************************
 * apps/examples/adc_reader/adc_reader_main.c
 *
 * Ứng dụng đọc điện áp từ chân PA1 trên STM32F411-minimum
 * ADC1 Channel 1 - PA1
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <nuttx/analog/adc.h>

/****************************************************************************
 * Định nghĩa
 ****************************************************************************/

#define ADC_DEVICE_PATH   "/dev/adc0"
#define ADC_VREF          3.3f        /* Điện áp tham chiếu 3.3V */
#define ADC_RESOLUTION    4095.0f     /* 12-bit ADC: 2^12 - 1 */
#define SAMPLE_COUNT      1           /* Đọc 1 mẫu mỗi lần */

/****************************************************************************
 * Biến toàn cục
 ****************************************************************************/

static struct adc_msg_s adc_sample[SAMPLE_COUNT];

/****************************************************************************
 * Hàm chuyển đổi giá trị ADC sang điện áp
 ****************************************************************************/

static float adc_to_voltage(int32_t adc_value)
{
  return (adc_value * ADC_VREF) / ADC_RESOLUTION;
}

/****************************************************************************
 * Hàm đọc và hiển thị điện áp
 ****************************************************************************/

static int read_voltage_from_pa1(int fd)
{
  ssize_t nbytes;
  float voltage;
  int samples_count;

  /* Đọc dữ liệu từ ADC */
  nbytes = read(fd, adc_sample, sizeof(adc_sample));
  
  if (nbytes < 0)
    {
      printf("❌ Lỗi đọc ADC: %ld\n", (long)nbytes);
      return -1;
    }
  
  if (nbytes == 0)
    {
      printf("⚠️  Không có dữ liệu ADC\n");
      return 0;
    }

  /* Tính số mẫu đã đọc được */
  samples_count = nbytes / sizeof(struct adc_msg_s);
  
  if (samples_count > 0)
    {
      /* Chuyển đổi giá trị ADC sang điện áp */
      voltage = adc_to_voltage(adc_sample[0].am_data);
      
      /* Hiển thị kết quả */
      printf("📊 PA1 - Giá trị thô: %4ld | Điện áp: %.3fV | Phần trăm: %.1f%%\n",
             (long)adc_sample[0].am_data, 
             voltage,
             (voltage / ADC_VREF) * 100.0f);
      
      return 1;
    }

  return 0;
}

/****************************************************************************
 * Hàm hiển thị thông tin hệ thống
 ****************************************************************************/

static void print_system_info(void)
{
  printf("╔══════════════════════════════════════════════╗\n");
  printf("║        🔋 ĐỌC ĐIỆN ÁP TỪ CHÂN PA1           ║\n");
  printf("║              STM32F411-minimum               ║\n");
  printf("╠══════════════════════════════════════════════╣\n");
  printf("║ • ADC: 12-bit (0-4095)                      ║\n");
  printf("║ • Điện áp tham chiếu: %.1fV                  ║\n", ADC_VREF);
  printf("║ • Chân đọc: PA1 (ADC1 Channel 1)            ║\n");
  printf("║ • Độ phân giải: %.1fmV                       ║\n", (ADC_VREF * 1000.0f) / ADC_RESOLUTION);
  printf("╚══════════════════════════════════════════════╝\n\n");
}

/****************************************************************************
 * Hàm chính
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int fd;
  int sample_count = 0;
  int max_samples = 0;
  int read_interval = 1; /* Giây */
  
  /* Hiển thị thông tin hệ thống */
  print_system_info();

  /* Xử lý tham số dòng lệnh */
  if (argc >= 2)
    {
      max_samples = atoi(argv[1]);
      if (max_samples > 0)
        {
          printf("🎯 Sẽ đọc %d mẫu\n", max_samples);
        }
    }
  
  if (argc >= 3)
    {
      read_interval = atoi(argv[2]);
      if (read_interval < 1) read_interval = 1;
      printf("⏱️  Khoảng cách đọc: %d giây\n", read_interval);
    }

  if (max_samples == 0)
    {
      printf("🔄 Đọc liên tục... (Nhấn Ctrl+C để dừng)\n");
    }
  
  printf("\n");

  /* Mở thiết bị ADC */
  fd = open(ADC_DEVICE_PATH, O_RDONLY);
  if (fd < 0)
    {
      printf("❌ Không thể mở thiết bị ADC: %s\n", ADC_DEVICE_PATH);
      printf("💡 Kiểm tra:\n");
      printf("   - CONFIG_STM32_ADC1=y\n");
      printf("   - CONFIG_ADC=y\n");
      printf("   - ADC driver đã được khởi tạo\n");
      return -1;
    }

  printf("✅ Đã mở thiết bị ADC: %s\n\n", ADC_DEVICE_PATH);

  /* Vòng lặp đọc điện áp */
  while (1)
    {
      int result = read_voltage_from_pa1(fd);
      
      if (result < 0)
        {
          printf("❌ Lỗi đọc dữ liệu, dừng chương trình\n");
          break;
        }
      
      if (result > 0)
        {
          sample_count++;
        }

      /* Kiểm tra điều kiện dừng */
      if (max_samples > 0 && sample_count >= max_samples)
        {
          printf("\n✅ Đã đọc đủ %d mẫu, kết thúc chương trình\n", sample_count);
          break;
        }

      /* Chờ trước khi đọc mẫu tiếp theo */
      sleep(read_interval);
    }

  /* Đóng thiết bị ADC */
  close(fd);
  
  printf("\n📋 Tổng kết:\n");
  printf("   - Đã đọc: %d mẫu\n", sample_count);
  printf("   - Thiết bị: %s\n", ADC_DEVICE_PATH);
  printf("   - Chân đọc: PA1\n");
  printf("\n🏁 Chương trình kết thúc\n");
  
  return 0;
}
