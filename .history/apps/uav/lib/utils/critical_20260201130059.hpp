/****************************************************************************
 * apps/uav/lib/utils/critical.hpp
 *
 * RAII wrapper cho critical sections trong NuttX
 ****************************************************************************/

#pragma once

#include <nuttx/irq.h>

namespace utils {

/**
 * @brief RAII wrapper cho critical section
 * 
 * Tự động enter/leave critical section khi tạo/hủy object.
 * Dùng để bảo vệ code khỏi bị interrupt trong thời gian ngắn.
 */
class CriticalSection {
public:
    CriticalSection() {
        _flags = enter_critical_section();
    }

    ~CriticalSection() {
        leave_critical_section(_flags);
    }

    /* Không cho phép copy/move */
    CriticalSection(const CriticalSection&) = delete;
    CriticalSection& operator=(const CriticalSection&) = delete;

private:
    irqstate_t _flags;
};

/**
 * @brief Kiểm tra xem đang chạy trong interrupt context không
 * @return true nếu đang trong ISR
 */
inline bool in_interrupt() {
    return up_interrupt_context();
}

} // namespace utils
