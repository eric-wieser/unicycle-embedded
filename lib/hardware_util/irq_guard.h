#include <Arduino.h>

/**
 * @brief RAII helper to lock out a single interrupt
 *
 * \rst
 * For example, in the following code::
 *
 *     volatile int my_data;
 *     char my_function() {
 *         irq_guard g(TIMER_IRQ);
 *
 *         if(my_data == 1)
 *             return 'A';
 *         if(my_data == 2)
 *             return 'B';
 *         return 'C';
 *     }
 *
 * The volatile variable ``my_data`` is guarded from modification by interrupts
 * on the timer. The interrupts are re-enabled after any return  is executed.
 * \endrst
 */
class irq_guard {
  const uint32_t state;
  const int irq;
public:
  irq_guard(int irq) : irq(irq), state(clearIntEnable(irq)) {}
  ~irq_guard() { restoreIntEnable(irq, state); }
};
