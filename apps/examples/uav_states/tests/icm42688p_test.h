/****************************************************************************
 * apps/examples/icm42688p_test/icm42688p_test.h
 *
 * Test đơn giản cho cảm biến ICM-42688-P.
 * Khai báo prototype cho chương trình test.
 *
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_ICM42688P_TEST_H
#define __APPS_EXAMPLES_ICM42688P_TEST_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @brief   Hàm main để chạy test ICM-42688-P.
 *
 * @param   argc   Số lượng tham số dòng lệnh
 * @param   argv   Mảng tham số dòng lệnh
 *
 * @return  0 nếu thành công, ngược lại trả mã lỗi âm
 */
int icm42688p_test_main(int argc, char *argv[]);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_EXAMPLES_ICM42688P_TEST_H */
