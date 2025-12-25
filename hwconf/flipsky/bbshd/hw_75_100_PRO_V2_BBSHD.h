#ifndef HW_75_100_PRO_V2_BBSHD_H_
#define HW_75_100_PRO_V2_BBSHD_H_

#include "mcconf_BBSHD.h"
#include "appconf_BBSHD.h"
#include "../../trampa/75_300/hw_75_300_r2.h"

//BBSHD
#define HW_HAS_LUNA_SERIAL_DISPLAY
#define HW_HAS_WHEEL_SPEED_SENSOR
#define HW_HAS_BRAKE_OVERRIDE

//PAS
#define HW_PAS1_PORT		GPIOA
#define HW_PAS1_PIN			6
#define HW_PAS2_PORT		GPIOC
#define HW_PAS2_PIN			5

#define HW_PAS1_EXTI_PORTSRC	EXTI_PortSourceGPIOC
#define HW_PAS1_EXTI_PINSRC		EXTI_PinSource5
#define HW_PAS1_EXTI_CH			EXTI9_5_IRQn
#define HW_PAS1_EXTI_LINE		EXTI_Line5
#define HW_PAS2_EXTI_PORTSRC	EXTI_PortSourceGPIOA
#define HW_PAS2_EXTI_PINSRC		EXTI_PinSource6
#define HW_PAS2_EXTI_CH			EXTI9_5_IRQn
#define HW_PAS2_EXTI_LINE		EXTI_Line6

//override ADC
#ifdef ADC_IND_SENS1
#undef ADC_IND_SENS1
#define ADC_IND_SENS1			0
#endif
#ifdef ADC_IND_SENS2
#undef ADC_IND_SENS2
#define ADC_IND_SENS2			1
#endif
#ifdef ADC_IND_SENS3
#undef ADC_IND_SENS3
#define ADC_IND_SENS3			2
#endif
#ifdef ADC_IND_CURR1
#undef ADC_IND_CURR1
#define ADC_IND_CURR1			3
#endif
#ifdef ADC_IND_CURR2
#undef ADC_IND_CURR2
#define ADC_IND_CURR2			4
#endif
#ifdef ADC_IND_CURR3
#undef ADC_IND_CURR3
#define ADC_IND_CURR3			5
#endif

// Component parameters (can be overridden)
#ifdef V_REG
#undef V_REG
#define V_REG					3.458 //pro v2 seems to like it
#endif

// used by pas, no brake cuttof sorry
#ifdef HW_ADC_EXT2_GPIO
#undef HW_ADC_EXT2_GPIO
#endif
#ifdef HW_ADC_EXT2_PIN
#undef HW_ADC_EXT2_PIN
#endif


// assign ICU to unused pin to allow to connect speed sensor to servo input
#ifdef HW_ICU_GPIO
#undef HW_ICU_GPIO
#define HW_ICU_GPIO				GPIOB
#endif
#ifdef HW_ICU_PIN
#undef HW_ICU_PIN
#define HW_ICU_PIN				4
#endif
#define HW_SPEED_SENSOR_PORT	GPIOB
#define HW_SPEED_SENSOR_PIN		6


#ifdef HW_LIM_VIN
#undef HW_LIM_VIN
#define HW_LIM_VIN				11.0, 90.0
#endif

// HW-specific functions
float hw75_300_get_temp(void);
void hw_update_speed_sensor(void);
float hw_get_speed(void);
float hw_get_distance(void);
float hw_get_distance_abs(void);
void hw_brake_override(float *brake);
float hw_read_motor_temp(float beta);
bool hw_bbshd_has_fixed_throttle_level(void);

#endif /* HW_75_100_PRO_V2_H_ */