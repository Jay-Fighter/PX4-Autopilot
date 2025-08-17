/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "pwm_cap.h"
#include <px4_arch/io_timer.h>

int AS5600PWMIN::task_spawn(int argc, char *argv[]) {
  auto *pwmin = new AS5600PWMIN();

  if (!pwmin) {
    PX4_ERR("driver allocation failed");
    return PX4_ERROR;
  }

  _object.store(pwmin);
  _task_id = task_id_is_work_queue;

  pwmin->start();

  return PX4_OK;
}

void AS5600PWMIN::start() {
  // NOTE: must first publish here, first publication cannot be in interrupt
  // context
  _pwm_input_pub.update();

  // Initialize the timer isr for measuring pulse widths. Publishing is done
  // inside the isr.
  //   timer_init();
  timer_init_t1c3();
}

void AS5600PWMIN::timer_init(void) {
  /* TODO
   * - use gpio+irq directly instead of timer (if accurate enough)
   * - make pin configurable
   */

  /* reserve the pin + timer */
  for (int i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; ++i) {
    if ((GPIO_INPUT_CAP1 & (GPIO_PORT_MASK | GPIO_PIN_MASK)) ==
        (timer_io_channels[i].gpio_out & (GPIO_PORT_MASK | GPIO_PIN_MASK))) {
      int ret1 = io_timer_allocate_channel(i, IOTimerChanMode_PWMIn);
      int ret2 = io_timer_allocate_timer(timer_io_channels[i].timer_index,
                                         IOTimerChanMode_PWMIn);

      if (ret1 != 0 || ret2 != 0) {
        PX4_ERR("timer/channel alloc failed (%i %i)", ret1, ret2);
        return;
      }
    }
  }

  /* run with interrupts disabled in case the timer is already
   * setup. We don't want it firing while we are doing the setup */
  irqstate_t flags = px4_enter_critical_section();

  /* configure input pin */
  px4_arch_configgpio(GPIO_INPUT_CAP1);

  /* claim our interrupt vector */
  irq_attach(PWMIN_TIMER1_C2_VECTOR, AS5600PWMIN::pwmin_tim_isr, NULL);

  /* Clear no bits, set timer enable bit.*/
  modifyreg32(PWMIN_TIMER1_C2_POWER_REG, 0, PWMIN_TIMER1_C2_POWER_BIT);

  /* disable and configure the timer */
  rCR1 = 0;
  rCR2 = 0;
  rSMCR = 0;
  rDIER = DIER_PWMIN_A_C2;
  rCCER = 0; /* unlock CCMR* registers */
  rCCMR1 = CCMR1_PWMIN_C2;
  rCCMR2 = CCMR2_PWMIN_C2;
  rSMCR = SMCR_PWMIN_1_C2; /* Set up mode */
  rSMCR = SMCR_PWMIN_2_C2; /* Enable slave mode controller */
  rCCER = CCER_PWMIN_C2;
  rDCR = 0;

  /* for simplicity scale by the clock in MHz. This gives us
   * readings in microseconds which is typically what is needed
   * for a PWM input driver */
  uint32_t prescaler = PWMIN_TIMER1_C2_CLOCK / 1000000UL;

  /*
   * define the clock speed. We want the highest possible clock
   * speed that avoids overflows.
   */
  rPSC = prescaler - 1;

  /* run the full span of the counter. All timers can handle
   * uint16 */
  rARR = UINT16_MAX;

  /* generate an update event; reloads the counter, all registers */
  rEGR = ATIM_EGR_UG;

  /* enable the timer */
  rCR1 = ATIM_CR1_CEN;

  px4_leave_critical_section(flags);

  /* enable interrupts */
  up_enable_irq(PWMIN_TIMER1_C2_VECTOR);
}

void AS5600PWMIN::timer_init_t1c3(void) {
  /* TODO
   * - use gpio+irq directly instead of timer (if accurate enough)
   * - make pin configurable
   */

  /* reserve the pin + timer */
  for (int i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; ++i) {
    if ((GPIO_INPUT_CAP2 & (GPIO_PORT_MASK | GPIO_PIN_MASK)) ==
        (timer_io_channels[i].gpio_out & (GPIO_PORT_MASK | GPIO_PIN_MASK))) {
      int ret1 = io_timer_allocate_channel(i, IOTimerChanMode_PWMIn);
      int ret2 = io_timer_allocate_timer(timer_io_channels[i].timer_index,
                                         IOTimerChanMode_PWMIn);

      if (ret1 != 0 || ret2 != 0) {
        PX4_ERR("timer/channel alloc failed (%i %i)", ret1, ret2);
        return;
      }
    }
  }

  /* run with interrupts disabled in case the timer is already
   * setup. We don't want it firing while we are doing the setup */
  irqstate_t flags = px4_enter_critical_section();

  /* configure input pin */
  px4_arch_configgpio(GPIO_INPUT_CAP2);

  /* claim our interrupt vector */
  irq_attach(PWMIN_TIMER1_C2_VECTOR, AS5600PWMIN::pwmin_tim_isr_t1c3, NULL);

  /* Clear no bits, set timer enable bit.*/
  modifyreg32(PWMIN_TIMER1_C2_POWER_REG, 0, PWMIN_TIMER1_C2_POWER_BIT);

  /* disable and configure the timer */
  rCR1 = 0;
  rCR2 = 0;
  rSMCR = 0;
  rDIER = DIER_PWMIN_A_C3;
  rCCER = 0; /* unlock CCMR* registers */
  rCCMR1 = CCMR1_PWMIN_C3;
  rCCMR2 = CCMR2_PWMIN_C3;
  rSMCR = SMCR_PWMIN_1_C3; /* Set up mode */
  rSMCR = SMCR_PWMIN_2_C3; /* Enable slave mode controller */
  rCCER = CCER_PWMIN_C3;
  rDCR = 0;

  /* for simplicity scale by the clock in MHz. This gives us
   * readings in microseconds which is typically what is needed
   * for a PWM input driver */
  uint32_t prescaler = PWMIN_TIMER1_C2_CLOCK / 9000000UL;

  /*
   * define the clock speed. We want the highest possible clock
   * speed that avoids overflows.
   */
  rPSC = prescaler - 1;

  /* run the full span of the counter. All timers can handle
   * uint16 */
  rARR = UINT16_MAX;

  /* generate an update event; reloads the counter, all registers */
  rEGR = GTIM_EGR_UG;

  /* enable the timer */
  rCR1 = GTIM_CR1_CEN;

  px4_leave_critical_section(flags);

  /* enable interrupts */
  up_enable_irq(PWMIN_TIMER1_C2_VECTOR);
}

int AS5600PWMIN::pwmin_tim_isr(int irq, void *context, void *arg) {
  uint16_t status = rSR;
  uint32_t period = rCCR_PWMIN_A_C2;
  uint32_t pulse_width = rCCR_PWMIN_B_C2;

  /* ack the interrupts we just read */
  rSR = 0;

  auto obj = get_instance();

  if (obj != nullptr) {
    obj->publish(status, period, pulse_width);
  }

  return PX4_OK;
}

int AS5600PWMIN::pwmin_tim_isr_t1c3(int irq, void *context, void *arg) {
  static uint32_t last_ccr3 = 0;  // 保存上一次上升沿的捕获值
  uint16_t status = rSR;
  uint32_t period = 0;
  uint32_t pulse_width = 0;

  // 上升沿（通道3）：计算周期（当前值 - 上一次值，处理溢出）
  if (status & ATIM_SR_CC3IF) {
    uint32_t current_ccr3 = rCCR3;
    if (current_ccr3 >= last_ccr3) {
      period = current_ccr3 - last_ccr3;  // 正常情况
    } else {
      period = (UINT16_MAX - last_ccr3) + current_ccr3;  // 溢出情况
    }
    last_ccr3 = current_ccr3;  // 更新上一次值
    rSR &= ~ATIM_SR_CC3IF;
  }

  // 下降沿（通道4）：计算脉宽（当前值 - 最近一次上升沿值）
  if (status & ATIM_SR_CC4IF) {
    uint32_t current_ccr4 = rCCR4;
    if (current_ccr4 >= last_ccr3) {
      pulse_width = current_ccr4 - last_ccr3;  // 正常情况
    } else {
      pulse_width = (UINT16_MAX - last_ccr3) + current_ccr4;  // 溢出情况
    }
    rSR &= ~ATIM_SR_CC4IF;
  }

  // 仅在周期和脉宽有效时发布
  auto obj = get_instance();
  if (obj != nullptr && period > 0 && pulse_width > 0) {
    obj->publish(status, period, pulse_width);
  }

  return PX4_OK;
}
void AS5600PWMIN::publish(uint16_t status, uint32_t period,
                          uint32_t pulse_width) {
  // if we missed an edge, we have to give up
  if (status & SR_OVF_PWMIN_C3) {
    _error_count++;
    return;
  }

  _pwm.timestamp = hrt_absolute_time();
  _pwm.error_count = _error_count;
  _pwm.period = period;
  _pwm.pulse_width = pulse_width;

  _pwm_input_pub.publish(_pwm);

  // update statistics
  _last_period = period;
  _last_width = pulse_width;
  _pulses_captured++;
}

int AS5600PWMIN::print_status() {
  PX4_INFO(
      "count=%u period=%u width=%u", static_cast<unsigned>(_pulses_captured),
      static_cast<unsigned>(_last_period), static_cast<unsigned>(_last_width));
  return 0;
}

int AS5600PWMIN::print_usage(const char *reason) {
  if (reason) {
    printf("%s\n\n", reason);
  }

  PRINT_MODULE_DESCRIPTION(
      R"DESCR_STR(
### Description
Measures the PWM input on AUX5 (or MAIN5) via a timer capture ISR and publishes via the uORB 'pwm_input` message.

)DESCR_STR");

  PRINT_MODULE_USAGE_NAME("pwm_input", "system");
  PRINT_MODULE_USAGE_COMMAND("start");
  PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

  return PX4_OK;
}

int AS5600PWMIN::custom_command(int argc, char *argv[]) {
  return print_usage();
}

extern "C" __EXPORT int pwm_cap_main(int argc, char *argv[]) {
  return AS5600PWMIN::main(argc, argv);
}
