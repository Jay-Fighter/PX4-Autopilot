/**
 * @file pwm_cap.h
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/log.h>
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

/*
 * Timer1 register accessors
 */

// TIM1
#define CAP_TIM1_BASE            STM32_TIM1_BASE
#define CAP_TIM1_CLOCK           STM32_APB2_TIM1_CLKIN
#define CAP_TIM1_POWER_REG       STM32_RCC_APB2ENR
#define CAP_TIM1_POWER_BIT       RCC_APB2ENR_TIM1EN
#define CAP_TIM1_IRQ             STM32_IRQ_TIM1CC

#define REG_TIM1(_reg)        (*(volatile uint32_t *)(CAP_TIM1_BASE + _reg))

#define rCR1_TIM1             REG_TIM1(STM32_ATIM_CR1_OFFSET)
#define rCR2_TIM1             REG_TIM1(STM32_ATIM_CR2_OFFSET)
#define rSMCR_TIM1            REG_TIM1(STM32_ATIM_SMCR_OFFSET)
#define rDIER_TIM1            REG_TIM1(STM32_ATIM_DIER_OFFSET)
#define rSR_TIM1              REG_TIM1(STM32_ATIM_SR_OFFSET)
#define rEGR_TIM1             REG_TIM1(STM32_ATIM_EGR_OFFSET)
#define rCCMR1_TIM1           REG_TIM1(STM32_ATIM_CCMR1_OFFSET)
#define rCCMR2_TIM1           REG_TIM1(STM32_ATIM_CCMR2_OFFSET)
#define rCCER_TIM1            REG_TIM1(STM32_ATIM_CCER_OFFSET)
#define rCNT_TIM1             REG_TIM1(STM32_ATIM_CNT_OFFSET)
#define rPSC_TIM1             REG_TIM1(STM32_ATIM_PSC_OFFSET)
#define rARR_TIM1             REG_TIM1(STM32_ATIM_ARR_OFFSET)
#define rCCR1_TIM1            REG_TIM1(STM32_ATIM_CCR1_OFFSET)
#define rCCR2_TIM1            REG_TIM1(STM32_ATIM_CCR2_OFFSET)
#define rCCR3_TIM1            REG_TIM1(STM32_ATIM_CCR3_OFFSET)
#define rCCR4_TIM1            REG_TIM1(STM32_ATIM_CCR4_OFFSET)
#define rDCR_TIM1             REG_TIM1(STM32_ATIM_DCR_OFFSET)
#define rDMAR_TIM1            REG_TIM1(STM32_ATIM_DMAR_OFFSET)


/* Channel A (TIM1, Channel 2) */
#define rCCR_PWMIN_A        rCCR2_TIM1           /* Compare register for PWMIN_A */
#define DIER_PWMIN_A        (GTIM_DIER_CC2IE)    /* Interrupt enable for PWMIN_A */
#define SR_INT_PWMIN_A      GTIM_SR_CC2IF        /* Interrupt status for PWMIN_A */
#define rCCR_PWMIN_B        rCCR1_TIM1           /* Compare register for PWMIN_B */
#define DIER_PWMIN_B        (GTIM_DIER_CC1IE)    /* Interrupt enable for PWMIN_B */
#define SR_INT_PWMIN_B      GTIM_SR_CC1IF        /* Interrupt status for PWMIN_B */
#define CCMR1_PWMIN         ((0x02 << GTIM_CCMR1_CC2S_SHIFT) | (0x01 << GTIM_CCMR1_CC1S_SHIFT)) /* Configure input capture mode */
#define CCMR2_PWMIN         0
#define CCER_PWMIN          (GTIM_CCER_CC1E | GTIM_CCER_CC2E)  /* Enable capture on both channels */
#define SR_OVF_PWMIN        (GTIM_SR_CC1OF | GTIM_SR_CC2OF)     /* Overflow status flag */
#define SMCR_PWMIN_1        (0x05 << GTIM_SMCR_TS_SHIFT)          /* Trigger selection */
#define SMCR_PWMIN_2        ((0x04 << GTIM_SMCR_SMS_SHIFT) | SMCR_PWMIN_1)  /* Configure slave mode and trigger */

/* Channel B (TIM1, Channel 3) */
#define rCCR_PWMIN_C        rCCR3_TIM1           /* Compare register for PWMIN_C */
#define DIER_PWMIN_C        (GTIM_DIER_CC3IE)    /* Interrupt enable for PWMIN_C */
#define SR_INT_PWMIN_C      GTIM_SR_CC3IF        /* Interrupt status for PWMIN_C */
#define rCCR_PWMIN_D        rCCR4_TIM1           /* Compare register for PWMIN_D */
#define DIER_PWMIN_D        (GTIM_DIER_CC4IE)    /* Interrupt enable for PWMIN_D */
#define SR_INT_PWMIN_D      GTIM_SR_CC4IF        /* Interrupt status for PWMIN_D */
#define CCMR1_PWMIN_C       ((0x02 << GTIM_CCMR1_CC2S_SHIFT) | (0x01 << GTIM_CCMR1_CC1S_SHIFT)) /* Configure input capture mode */
#define CCMR2_PWMIN_C       0
#define CCER_PWMIN_C        (GTIM_CCER_CC3E | GTIM_CCER_CC4E)  /* Enable capture on both channels */
#define SR_OVF_PWMIN_C      (GTIM_SR_CC3OF | GTIM_SR_CC4OF)     /* Overflow status flag */
#define SMCR_PWMIN_C_1      (0x05 << GTIM_SMCR_TS_SHIFT)          /* Trigger selection */
#define SMCR_PWMIN_C_2      ((0x04 << GTIM_SMCR_SMS_SHIFT) | SMCR_PWMIN_C_1)  /* Configure slave mode and trigger */
/*
 * HRT clock must be at least 1MHz
 */
#if CAP_TIM1_CLOCK <= 1000000
# error PWMIN_TIMER_CLOCK must be greater than 1MHz
#endif

/*
 * Timer4 register accessors
 */
#define CAP_TIM4_BASE            STM32_TIM4_BASE // TIM4 基地址
#define CAP_TIM4_CLOCK           STM32_APB1_TIM4_CLKIN
#define CAP_TIM4_POWER_REG       STM32_RCC_APB1ENR
#define CAP_TIM4_POWER_BIT       RCC_APB1ENR_TIM4EN
#define CAP_TIM4_IRQ             STM32_IRQ_TIM4

#define REG_TIM4(_reg)        (*(volatile uint32_t *)(CAP_TIM4_BASE + _reg))

#define rCR1_TIM4             REG_TIM4(STM32_GTIM_CR1_OFFSET)     // 控制寄存器1
#define rCR2_TIM4             REG_TIM4(STM32_GTIM_CR2_OFFSET)     // 控制寄存器2
#define rSMCR_TIM4            REG_TIM4(STM32_GTIM_SMCR_OFFSET)    // 从模式控制寄存器
#define rDIER_TIM4            REG_TIM4(STM32_GTIM_DIER_OFFSET)    // 中断使能寄存器
#define rSR_TIM4              REG_TIM4(STM32_GTIM_SR_OFFSET)      // 状态寄存器
#define rEGR_TIM4             REG_TIM4(STM32_GTIM_EGR_OFFSET)     // 事件生成寄存器
#define rCCMR1_TIM4           REG_TIM4(STM32_GTIM_CCMR1_OFFSET)   // 捕获比较模式寄存器1（CH1/CH2）
#define rCCMR2_TIM4           REG_TIM4(STM32_GTIM_CCMR2_OFFSET)   // 捕获比较模式寄存器2（CH3/CH4）
#define rCCER_TIM4            REG_TIM4(STM32_GTIM_CCER_OFFSET)    // 捕获比较输出使能寄存器
#define rCNT_TIM4             REG_TIM4(STM32_GTIM_CNT_OFFSET)     // 计数器
#define rPSC_TIM4             REG_TIM4(STM32_GTIM_PSC_OFFSET)     // 预分频器
#define rARR_TIM4             REG_TIM4(STM32_GTIM_ARR_OFFSET)     // 自动重载寄存器
#define rCCR1_TIM4            REG_TIM4(STM32_GTIM_CCR1_OFFSET)    // 捕获/比较寄存器1
#define rCCR2_TIM4            REG_TIM4(STM32_GTIM_CCR2_OFFSET)    // 捕获/比较寄存器2
#define rCCR3_TIM4            REG_TIM4(STM32_GTIM_CCR3_OFFSET)    // 捕获/比较寄存器3
#define rCCR4_TIM4            REG_TIM4(STM32_GTIM_CCR4_OFFSET)    // 捕获/比较寄存器4
#define rDCR_TIM4             REG_TIM4(STM32_GTIM_DCR_OFFSET)     // DMA 控制寄存器
#define rDMAR_TIM4            REG_TIM4(STM32_GTIM_DMAR_OFFSET)    // DMA 地址寄存器

/*
 * HRT clock must be at least 1MHz
 */
#if CAP_TIM4_CLOCK <= 1000000
# error PWMIN_TIMER_CLOCK must be greater than 1MHz
#endif


class AS5600PWMIN : public ModuleBase<AS5600PWMIN>
{
public:
	void start();
	void publish(uint16_t status, uint32_t period, uint32_t pulse_width);
	int print_status() override;

	static int pwmin_tim_isr(int irq, void *context, void *arg);

	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	static int task_spawn(int argc, char *argv[]);


private:
	void timer_init(void);

	uint32_t _error_count {};
	uint32_t _pulses_captured {};
	uint32_t _last_period {};
	uint32_t _last_width {};

	bool _timer_started {};

	pwm_input_s _pwm {};

	uORB::PublicationData<pwm_input_s> _pwm_input_pub{ORB_ID(pwm_input)};

	uint32_t _periods[4] {};       // 存储4路PWM周期
	uint32_t _pulse_widths[4] {};  // 存储4路PWM脉冲宽度
	uint32_t _pulse_counts[4] {};  // 存储4路PWM捕获计数

};
