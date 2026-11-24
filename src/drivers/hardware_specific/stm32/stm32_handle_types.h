#pragma once

#ifndef TIMER_HANDLE_TYPE
#ifdef USE_HALV2_DRIVER
#define TIMER_HANDLE_TYPE hal_tim_handle_t
#define TIMER_INSTANCE(handle) (reinterpret_cast<TIM_TypeDef*>(handle->instance))
#else
#define TIMER_HANDLE_TYPE TIM_HandleTypeDef
#define TIMER_INSTANCE(handle) (handle->Instance)
#endif
#endif
