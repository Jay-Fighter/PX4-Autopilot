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

#include <drivers/drv_hrt.h>
#include <drivers/device/device.h>

#include <px4_platform_common/module.h>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/pwm_input.h>

#if HRT_TIMER == PWMIN_TIMER
#error cannot share timer between HRT and PWMIN
#endif

#if !defined(GPIO_PWM_IN) || !defined(PWMIN_TIMER) || !defined(PWMIN_TIMER_CHANNEL)
#error PWMIN defines are needed in board_config.h for this board
#endif

/* Get the timer defines */
#define PWMIN_TIMER1_C2_BASE	STM32_TIM1_BASE
#define PWMIN_TIMER1_C2_CLOCK	STM32_APB2_TIM1_CLKIN
#define PWMIN_TIMER1_C2_POWER_REG	STM32_RCC_APB2ENR
#define PWMIN_TIMER1_C2_POWER_BIT	RCC_APB2ENR_TIM1EN
#define PWMIN_TIMER1_C2_VECTOR	STM32_IRQ_TIM1CC

/*
 * HRT clock must be at least 1MHz
 */
#if PWMIN_TIMER1_C2_CLOCK <= 1000000
# error PWMIN_TIMER1_C2_CLOCK must be greater than 1MHz
#endif

/*
 * Timer register accessors
 */
#define REG(_reg)	(*(volatile uint32_t *)(PWMIN_TIMER1_C2_BASE + _reg))

#define rCR1		REG(STM32_ATIM_CR1_OFFSET)
#define rCR2		REG(STM32_ATIM_CR2_OFFSET)
#define rSMCR		REG(STM32_ATIM_SMCR_OFFSET)
#define rDIER		REG(STM32_ATIM_DIER_OFFSET)
#define rSR		REG(STM32_ATIM_SR_OFFSET)
#define rEGR		REG(STM32_ATIM_EGR_OFFSET)
#define rCCMR1		REG(STM32_ATIM_CCMR1_OFFSET)
#define rCCMR2		REG(STM32_ATIM_CCMR2_OFFSET)
#define rCCER		REG(STM32_ATIM_CCER_OFFSET)
#define rCNT		REG(STM32_ATIM_CNT_OFFSET)
#define rPSC		REG(STM32_ATIM_PSC_OFFSET) //Input Capture Prescaler
#define rARR		REG(STM32_ATIM_ARR_OFFSET)
#define rCCR1		REG(STM32_ATIM_CCR1_OFFSET)
#define rCCR2		REG(STM32_ATIM_CCR2_OFFSET)
#define rCCR3		REG(STM32_ATIM_CCR3_OFFSET)
#define rCCR4		REG(STM32_ATIM_CCR4_OFFSET)
#define rDCR		REG(STM32_ATIM_DCR_OFFSET)
#define rDMAR		REG(STM32_ATIM_DMAR_OFFSET)

/*
 * Specific registers and bits used by HRT sub-functions
 */
//配置1
#define rCCR_PWMIN_A_C2		rCCR2			/* compare register for PWMIN */
#define DIER_PWMIN_A_C2		(ATIM_DIER_CC2IE)	/* interrupt enable for PWMIN */
#define SR_INT_PWMIN_A_C2	ATIM_SR_CC2IF		/* interrupt status for PWMIN */
#define rCCR_PWMIN_B_C2		rCCR1			/* compare register for PWMIN */
#define DIER_PWMIN_B_C2		ATIM_DIER_CC1IE		/* interrupt enable for PWMIN */
#define SR_INT_PWMIN_B_C2	ATIM_SR_CC1IF		/* interrupt status for PWMIN */
#define CCMR1_PWMIN_C2		((0x01 << ATIM_CCMR1_CC2S_SHIFT) | (0x02 << ATIM_CCMR1_CC1S_SHIFT))
#define CCMR2_PWMIN_C2		0
#define CCER_PWMIN_C2		(ATIM_CCER_CC1P | ATIM_CCER_CC1E | ATIM_CCER_CC2E) //同时开启通道 1 和通道 2 的捕获功能，并让通道 1 捕获下降沿、通道 2 捕获上升沿，两者协同测量同一 PWM 信号
#define SR_OVF_PWMIN_C2		(ATIM_SR_CC1OF | ATIM_SR_CC2OF)
#define SMCR_PWMIN_1_C2		(0x06 << ATIM_SMCR_TS_SHIFT)
#define SMCR_PWMIN_2_C2		((0x04 << ATIM_SMCR_SMS_SHIFT) | SMCR_PWMIN_1_C2)

//配置2
#define rCCR_PWMIN_A_C3		rCCR3			/* compare register for PWMIN */
#define DIER_PWMIN_A_C3		(ATIM_DIER_CC3IE)	/* interrupt enable for PWMIN */
#define SR_INT_PWMIN_A_C3	ATIM_SR_CC3IF		/* interrupt status for PWMIN */
#define rCCR_PWMIN_B_C3		rCCR4			/* compare register for PWMIN */
#define DIER_PWMIN_B_C3		ATIM_DIER_CC4IE		/* interrupt enable for PWMIN */
#define SR_INT_PWMIN_B_C3	ATIM_SR_CC4IF		/* interrupt status for PWMIN */
#define CCMR1_PWMIN_C3		0
#define CCMR2_PWMIN_C3		((0x01 << ATIM_CCMR2_CC3S_SHIFT) | (0x02 << ATIM_CCMR2_CC4S_SHIFT))
#define CCER_PWMIN_C3		(ATIM_CCER_CC4P | ATIM_CCER_CC3E | ATIM_CCER_CC4E)
#define SR_OVF_PWMIN_C3		(ATIM_SR_CC4OF | ATIM_SR_CC3OF)
#define SMCR_PWMIN_1_C3 	0
#define SMCR_PWMIN_2_C3  	0
// #define SMCR_PWMIN_1_C3		(0x00 << ATIM_SMCR_TS_SHIFT)
// #define SMCR_PWMIN_2_C3		((0x04 << ATIM_SMCR_SMS_SHIFT) | SMCR_PWMIN_1_C3)


class AS5600PWMIN : public ModuleBase<AS5600PWMIN>
{
public:
	void start();
	void publish(uint16_t status, uint32_t period, uint32_t pulse_width);
	int print_status() override;

	static int pwmin_tim_isr(int irq, void *context, void *arg);
	static int pwmin_tim_isr_t1c3(int irq, void *context, void *arg);

	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	static int task_spawn(int argc, char *argv[]);


private:
	void timer_init(void);
	void timer_init_t1c3(void);

	uint32_t _error_count {};
	uint32_t _pulses_captured {};
	uint32_t _last_period {};
	uint32_t _last_width {};

	bool _timer_started {};

	pwm_input_s _pwm {};

	uORB::PublicationData<pwm_input_s> _pwm_input_pub{ORB_ID(pwm_input)};

};
