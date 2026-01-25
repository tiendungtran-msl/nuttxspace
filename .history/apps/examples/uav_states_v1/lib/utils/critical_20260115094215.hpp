#pragma once

#include <nuttx/irq.h>

namespace utils {

/**
 * RAII wrapper for critical sections
 */
class CriticalSection {
public:
    CriticalSection() {
        _flags = enter_critical_section();
    }
    
    ~CriticalSection() {
        leave_critical_section(_flags);
    }
    
    // Delete copy/move
    CriticalSection(const CriticalSection&) = delete;
    CriticalSection& operator=(const CriticalSection&) = delete;
    
private: 
    irqstate_t _flags;
};

/**
 * Check if currently in interrupt context
 */
inline bool in_interrupt() {
    return up_interrupt_context();
}

} // namespace utils