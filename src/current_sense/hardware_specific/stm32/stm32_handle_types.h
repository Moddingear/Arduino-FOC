#pragma once

#ifndef ADC_HANDLE_TYPE
#ifdef USE_HALV2_DRIVER
#define ADC_HANDLE_TYPE hal_adc_handle_t
#define ADC_INSTANCE(handle) (reinterpret_cast<ADC_TypeDef*>((handle)->instance))
#define ADC_CHANNEL_TYPE hal_adc_channel_t
#else
#define ADC_HANDLE_TYPE ADC_HandleTypeDef
#define ADC_INSTANCE(handle) ((handle)->Instance)
#define ADC_CHANNEL_TYPE uint32_t
#endif
#endif
