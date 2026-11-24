
#include "../../hardware_api.h"
#include "./stm32_mcu.h"
#include "./stm32_timerutils.h"
#include "./stm32_searchtimers.h"

#if defined(ARDUINO_ARCH_STM32) // if stm32duino or portenta

#pragma message("")
#pragma message("SimpleFOC: compiling for STM32")
#pragma message("")


/*
 * Timer management
 * SimpleFOC manages the timers using only STM32 HAL and LL APIs, and does not use the HardwareTimer API.
 * This is because the HardwareTimer API is not available on all STM32 boards, and does not provide all
 * the functionality that SimpleFOC requires anyway.
 * By using the HAL and LL APIs directly, we can ensure that SimpleFOC works on all STM32 boards, specifically
 * also those that use MBED with Arduino (Portenta H7, Giga, Nicla).
 * 
 * When using stm32duino, the HardwareTimer API is available, and can be used in parallel with SimpleFOC,
 * provided you don't use the same timers for both.
 */

// track timers initialized via SimpleFOC
int numTimersUsed = 0;
TIMER_HANDLE_TYPE* timersUsed[SIMPLEFOC_STM32_MAX_TIMERSUSED];

// reserve timers for other uses, so SimpleFOC doesn't use them for motors
int numTimersReserved = 0;
TIM_TypeDef* reservedTimers[SIMPLEFOC_STM32_MAX_TIMERSRESERVED];

// track drivers initialized via SimpleFOC - used to know which timer channels are used
int numMotorsUsed = 0;
STM32DriverParams* motorsUsed[SIMPLEFOC_STM32_MAX_MOTORSUSED];

// query functions to check which timers are used
int stm32_getNumTimersUsed() {
  return numTimersUsed;
}
int stm32_getNumMotorsUsed() {
  return numMotorsUsed;
}
int stm32_getNumTimersReserved() {
  return numTimersReserved;
}
bool stm32_isTimerReserved(TIM_TypeDef* timer) {
  for (int i=0; i<numTimersReserved; i++) {
    if (reservedTimers[i] == timer)
      return true;
  }
  return false;
}
bool stm32_isTimerUsed(TIMER_HANDLE_TYPE* timer) {
  for (int i=0; i<numTimersUsed; i++) {
    if (timersUsed[i] == timer)
      return true;
  }
  return false;
}
STM32DriverParams* stm32_getMotorUsed(int index) {
  return motorsUsed[index];
}
bool stm32_isChannelUsed(PinMap* pin) {
  if (stm32_isTimerReserved((TIM_TypeDef*)pin->peripheral)) {
    return true;
  }
  for (int i=0; i<numMotorsUsed; i++) {
    for (int j=0; j<6; j++) {
      if (motorsUsed[i]->timers_handle[j] == NULL) break;
      if (motorsUsed[i]->channels[j] == STM_PIN_CHANNEL(pin->function) && (reinterpret_cast<TIM_TypeDef*>(pin->peripheral)) == TIMER_INSTANCE(motorsUsed[i]->timers_handle[j]))
        return true;
    }
  }
  return false;
}
TIMER_HANDLE_TYPE* stm32_getTimer(PinMap* timer) {
  for (int i=0; i<numTimersUsed; i++) {
    if (TIMER_INSTANCE(timersUsed[i]) == reinterpret_cast<TIM_TypeDef*>(timer->peripheral))
      return timersUsed[i];
  }
  return NULL;
}
bool stm32_reserveTimer(TIM_TypeDef* timer) {
  if (numTimersReserved >= SIMPLEFOC_STM32_MAX_TIMERSRESERVED) {
    SIMPLEFOC_DEBUG("STM32-DRV: ERR: too many timers reserved");
    return false;
  }
  reservedTimers[numTimersReserved++] = timer;
  return true;
}
// function to get a timer handle given the pinmap entry of the pin you want to use
// after calling this function, the timer is marked as used by SimpleFOC
TIMER_HANDLE_TYPE* stm32_useTimer(PinMap* timer) {
  TIMER_HANDLE_TYPE* handle = stm32_getTimer(timer);
  if (handle != NULL) return handle;
  if (numTimersUsed >= SIMPLEFOC_STM32_MAX_TIMERSUSED) {
    SIMPLEFOC_DEBUG("STM32-DRV: ERR: too many timers used");
    return NULL;
  }
  if (stm32_isTimerReserved((TIM_TypeDef*)timer->peripheral)) {
    SIMPLEFOC_DEBUG("STM32-DRV: ERR: timer reserved");
    return NULL;
  }
  #ifdef USE_HALV2_DRIVER
  handle = new hal_tim_handle_t;
  HAL_TIM_Init(handle, static_cast<hal_tim_t>(reinterpret_cast<uintptr_t>(timer->peripheral)));
  hal_tim_config_t config = {
    .prescaler = 0,
    .counter_mode = HAL_TIM_COUNTER_CENTER_UP,
    .period = (1<<16)-1,
    .repetition_counter = 1,
    .clock_sel = {
      .clock_source = HAL_TIM_CLK_INTERNAL,
    },
  };
  HAL_TIM_SetConfig(handle, &config);
  #else
  handle = new TIM_HandleTypeDef();
  handle->Instance = (TIM_TypeDef*)timer->peripheral;
  handle->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
  handle->Lock = HAL_UNLOCKED;
  handle->State = HAL_TIM_STATE_RESET;
  handle->hdma[0] = NULL;
  handle->hdma[1] = NULL;
  handle->hdma[2] = NULL;
  handle->hdma[3] = NULL;
  handle->hdma[4] = NULL;
  handle->hdma[5] = NULL;
  handle->hdma[6] = NULL;
  handle->Init.Prescaler = 0;
  handle->Init.Period = ((1 << 16) - 1);
  handle->Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED2;
  handle->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  handle->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  #if defined(TIM_RCR_REP)
  handle->Init.RepetitionCounter = 1;
  #endif
  #if STM32_CORE_VERSION_MAJOR >= 3
  enableTimerClock(handle->Instance);
  #else
  enableTimerClock(handle);
  #endif
  HAL_TIM_Base_Init(handle);
  #endif
  stm32_pauseTimer(handle);
  timersUsed[numTimersUsed++] = handle;
  return handle;
}




bool _getPwmState(void* params) {
  // assume timers are synchronized and that there's at least one timer
  bool dir = LL_TIM_GetDirection(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[0]));
  return dir;
}




void stm32_pause(STM32DriverParams* params) {
  if (params->master_timer != NULL) {
    stm32_pauseTimer(params->master_timer);
  }
  else {
    for (int i=0; i<params->num_timers; i++) {
      stm32_pauseTimer(params->timers_handle[i]);
    }
  }
}



void stm32_resume(STM32DriverParams* params) {
  if (params->master_timer != NULL) {
    stm32_resumeTimer(params->master_timer);
  }
  else {
    for (int i=0; i<params->num_timers; i++) {
      stm32_resumeTimer(params->timers_handle[i]);
    }
  }
}



// init pin pwm
#ifdef USE_HALV2_DRIVER
TIMER_HANDLE_TYPE* stm32_initPinPWM(uint32_t PWM_freq, PinMap* timer, hal_tim_oc_mode_t mode = HAL_TIM_OC_PWM1, hal_tim_oc_polarity_t polarity = HAL_TIM_OC_HIGH, hal_tim_oc_polarity_t Npolarity = HAL_TIM_OC_HIGH) 
#else
TIMER_HANDLE_TYPE* stm32_initPinPWM(uint32_t PWM_freq, PinMap* timer, uint32_t mode = TIM_OCMODE_PWM1, uint32_t polarity = TIM_OCPOLARITY_HIGH, uint32_t Npolarity = TIM_OCNPOLARITY_HIGH) 
#endif
{
  // sanity check
  if (timer==NULL)
    return NULL;
  TIMER_HANDLE_TYPE* handle = stm32_getTimer(timer);
  uint32_t channel = STM_PIN_CHANNEL(timer->function);
  if (handle==NULL) {
    handle = stm32_useTimer(timer);
    #if  defined(SIMPLEFOC_STM32_DEBUG) && !defined(SIMPLEFOC_DISABLE_DEBUG) 
    SIMPLEFOC_DEBUG("STM32-DRV: Initializing TIM", (int)stm32_getTimerNumber(TIMER_INSTANCE(handle)));
    #endif
    uint32_t arr = stm32_setClockAndARR(handle, PWM_freq);
    if (arr<SIMPLEFOC_STM32_MIN_RESOLUTION) {
      SIMPLEFOC_DEBUG("STM32-DRV: WARN timer resolution too low (<8bit): ", (int)arr+1);
    }
    else {
      #if  defined(SIMPLEFOC_STM32_DEBUG) && !defined(SIMPLEFOC_DISABLE_DEBUG) 
      SIMPLEFOC_DEBUG("STM32-DRV: Timer resolution set to: ", (int)arr+1);
      #endif
    }
  }
  #ifdef USE_HALV2_DRIVER
  bool n_channel = STM_PIN_INVERTED(timer->function);
  hal_tim_oc_channel_config_t ch_config = {
    .polarity = n_channel ? Npolarity : polarity,
    .idle_state = HAL_TIM_OC_IDLE_STATE_RESET,
    .override_state = HAL_TIM_OC_OVERRIDE_RESET,
    .break_mode = HAL_TIM_OC_BREAKMODE_IMMEDIATE,
  };
  hal_tim_channel_t hal_channel = static_cast<hal_tim_channel_t>(channel + (n_channel ? HAL_TIM_CHANNEL_1N : 0));
  HAL_TIM_OC_GetConfigChannel(handle, hal_channel, &ch_config);
  hal_tim_oc_compare_unit_config_t oc_config = {
    .mode = mode,
    .pulse = 0,
  };
  hal_tim_oc_compare_unit_t hal_unit = static_cast<hal_tim_oc_compare_unit_t>(channel);
  HAL_TIM_OC_SetConfigCompareUnit(handle, hal_unit, &oc_config);
  #else
  TIM_OC_InitTypeDef channelOC;
  channelOC.OCMode = mode;
  channelOC.Pulse = 0; //__HAL_TIM_GET_COMPARE(handle, channel);
  channelOC.OCPolarity = polarity;
  channelOC.OCFastMode = TIM_OCFAST_DISABLE;
#if defined(TIM_CR2_OIS1)
  channelOC.OCIdleState = TIM_OCIDLESTATE_RESET; //(polarity==TIM_OCPOLARITY_HIGH)?TIM_OCIDLESTATE_RESET:TIM_OCIDLESTATE_SET;
#endif
#if defined(TIM_CCER_CC1NE)
  channelOC.OCNPolarity = Npolarity;
#if defined(TIM_CR2_OIS1)
  channelOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; //(Npolarity==TIM_OCNPOLARITY_HIGH)?TIM_OCNIDLESTATE_RESET:TIM_OCNIDLESTATE_SET;
#endif
#endif
  HAL_TIM_PWM_ConfigChannel(handle, &channelOC, stm32_getHALChannel(channel));
  #endif
  pinmap_pinout(timer->pin, PinMap_TIM);
  LL_TIM_CC_EnableChannel(TIMER_INSTANCE(handle), stm32_getLLChannel(timer));
  if (IS_TIM_BREAK_INSTANCE(TIMER_INSTANCE(handle))) {
    LL_TIM_EnableAllOutputs(TIMER_INSTANCE(handle));
  }
  #if defined(SIMPLEFOC_STM32_DEBUG) && !defined(SIMPLEFOC_DISABLE_DEBUG) 
    SimpleFOCDebug::print("STM32-DRV: Configured TIM");
    SimpleFOCDebug::print((int)stm32_getTimerNumber(TIMER_INSTANCE(handle)));
    SIMPLEFOC_DEBUG("_CH", (int)channel);
  #endif
  return handle;
}




#ifdef USE_HALV2_DRIVER
#define OCPOL HAL_TIM_OC_HIGH : HAL_TIM_OC_LOW
#define PWM1 HAL_TIM_OC_PWM1
#define PWM2 HAL_TIM_OC_PWM2
#else
#define OCPOL TIM_OCPOLARITY_HIGH : TIM_OCPOLARITY_LOW
#define PWM1 TIM_OCMODE_PWM1
#define PWM2 TIM_OCMODE_PWM2
#endif

/**
0110: PWM mode 1 - In upcounting, channel 1 is active as long as TIMx_CNT<TIMx_CCR1
else inactive. In downcounting, channel 1 is inactive (OC1REF=‘0’) as long as
TIMx_CNT>TIMx_CCR1 else active (OC1REF=’1’).
0111: PWM mode 2 - In upcounting, channel 1 is inactive as long as
TIMx_CNT<TIMx_CCR1 else active. In downcounting, channel 1 is active as long as
TIMx_CNT>TIMx_CCR1 else inactive
 */
// init high side pin
TIMER_HANDLE_TYPE* _stm32_initPinPWMHigh(uint32_t PWM_freq, PinMap* timer) {
  auto polarity = SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH ? OCPOL ;
  TIMER_HANDLE_TYPE* handle = stm32_initPinPWM(PWM_freq, timer, PWM1, polarity);
  LL_TIM_OC_EnablePreload(TIMER_INSTANCE(handle), stm32_getLLChannel(timer));
  return handle;
}

// init low side pin
TIMER_HANDLE_TYPE* _stm32_initPinPWMLow(uint32_t PWM_freq, PinMap* timer) {
  auto polarity = SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH ? OCPOL;
  TIMER_HANDLE_TYPE* handle = stm32_initPinPWM(PWM_freq, timer, PWM2, polarity);
  LL_TIM_OC_EnablePreload(TIMER_INSTANCE(handle), stm32_getLLChannel(timer));
  return handle;
}

#undef OCPOL
#undef PWM1
#undef PWM2






// // align the timers to end the init
// void _startTimers(TIM_HandleTypeDef *timers_to_start[], int timer_num) {
//   stm32_alignTimers(timers_to_start, timer_num);
// }

// void _stopTimers(TIM_HandleTypeDef **timers_to_stop, int timer_num) {
//   TIM_HandleTypeDef* timers_distinct[6];
//   timer_num = stm32_distinctTimers(timers_to_stop, timer_num, timers_distinct);
//   for (int i=0; i < timer_num; i++) {
//     if(timers_distinct[i] == NULL) return;
//     stm32_pauseTimer(timers_distinct[i]);
//     stm32_refreshTimer(timers_distinct[i]);
//     #ifdef SIMPLEFOC_STM32_DEBUG
//       SIMPLEFOC_DEBUG("STM32-DRV: Stopping timer ", stm32_getTimerNumber(timers_distinct[i]->Instance));
//     #endif
//   }
// }






// Basically a sanity check to avoid complex scenarios where the user is using the same timers for multiple purposes.
int stm32_checkTimerFrequency(long pwm_frequency, TIMER_HANDLE_TYPE *timers[], uint8_t num_timers){
  TIMER_HANDLE_TYPE* timers_distinct[6];
  uint8_t timer_num = stm32_distinctTimers(timers, num_timers, timers_distinct);
  float common_pwm_freq = 0.0f;
  for (int i=0; i<timer_num; i++) {
    uint32_t freq = stm32_getTimerClockFreq(timers_distinct[i]);
    uint32_t arr = TIMER_INSTANCE(timers_distinct[i])->ARR;
    uint32_t prescaler = TIMER_INSTANCE(timers_distinct[i])->PSC;
    float pwm_freq = (float)freq/(prescaler+1.0f)/(arr+1.0f)/2.0f;
    if (i==0) {
      common_pwm_freq = pwm_freq;
    } else {
      if (pwm_freq != common_pwm_freq) {
        #if  defined(SIMPLEFOC_STM32_DEBUG) && !defined(SIMPLEFOC_DISABLE_DEBUG) 
        SimpleFOCDebug::print("STM32-DRV: ERR: Timer frequency different: TIM");
        SimpleFOCDebug::print(stm32_getTimerNumber(TIMER_INSTANCE(timers_distinct[0])));
        SimpleFOCDebug::print(" freq: ");
        SimpleFOCDebug::print(common_pwm_freq);
        SimpleFOCDebug::print(" != TIM");
        SimpleFOCDebug::print(stm32_getTimerNumber(TIMER_INSTANCE(timers_distinct[i])));
        SimpleFOCDebug::println(" freq: ", pwm_freq);
        #endif
        return -1;
      }
    }
  }
  if ( (common_pwm_freq - (float)pwm_frequency) > 1.0f) {
      #if  defined(SIMPLEFOC_STM32_DEBUG) && !defined(SIMPLEFOC_DISABLE_DEBUG) 
      SIMPLEFOC_DEBUG("STM32-DRV: ERR: Common timer frequency unexpected: ", common_pwm_freq);
      #endif
      return -1;
  }
  return 0;
}


// configure hardware 6pwm for a complementary pair of channels
STM32DriverParams* _stm32_initHardware6PWMPair(long PWM_freq, float dead_zone, PinMap* pinH, PinMap* pinL, STM32DriverParams* params, int paramsPos) {
  // sanity check
  if (pinH==NULL || pinL==NULL)
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
#if defined(STM32L0xx) // L0 boards dont have hardware 6pwm interface 
  return SIMPLEFOC_DRIVER_INIT_FAILED; // return nothing
#endif

  uint32_t channel1 = STM_PIN_CHANNEL(pinH->function);
  uint32_t channel2 = STM_PIN_CHANNEL(pinL->function);

  // more sanity check
  if (channel1!=channel2 || pinH->peripheral!=pinL->peripheral)
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;

  #ifdef USE_HALV2_DRIVER
  hal_tim_oc_polarity_t polarity = SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH ? HAL_TIM_OC_HIGH : HAL_TIM_OC_LOW;
  hal_tim_oc_polarity_t Npolarity = SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH ? HAL_TIM_OC_HIGH : HAL_TIM_OC_LOW;
  TIMER_HANDLE_TYPE* handle = stm32_initPinPWM(PWM_freq, pinH, HAL_TIM_OC_PWM1, polarity, Npolarity);
  #else
  uint32_t polarity = SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH ? TIM_OCPOLARITY_HIGH : TIM_OCPOLARITY_LOW;
  uint32_t Npolarity = SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH ? TIM_OCNPOLARITY_HIGH : TIM_OCNPOLARITY_LOW;
  TIMER_HANDLE_TYPE* handle = stm32_initPinPWM(PWM_freq, pinH, TIM_OCMODE_PWM1, polarity, Npolarity);
  #endif

  // also init the low side pin
  #ifdef USE_HALV2_DRIVER
  {
    bool n_channel = STM_PIN_INVERTED(pinL->function);
    hal_tim_oc_channel_config_t ch_config = {
      .polarity = n_channel ? Npolarity : polarity,
      .idle_state = HAL_TIM_OC_IDLE_STATE_RESET,
      .override_state = HAL_TIM_OC_OVERRIDE_RESET,
      .break_mode = HAL_TIM_OC_BREAKMODE_IMMEDIATE,
    };
    hal_tim_channel_t hal_channel = static_cast<hal_tim_channel_t>(channel2 + (n_channel ? HAL_TIM_CHANNEL_1N : 0));
    HAL_TIM_OC_GetConfigChannel(handle, hal_channel, &ch_config);
  }
  #endif
  pinmap_pinout(pinL->pin, PinMap_TIM);


  // dead time is set in nanoseconds
  uint32_t dead_time_ns = (float)(1e9f/PWM_freq)*dead_zone;
  #ifdef USE_HALV2_DRIVER
  uint32_t dead_time = LL_TIM_CALC_DEADTIME(stm32_getTimerClockFreq(handle), LL_TIM_GetClockDivision(TIMER_INSTANCE(handle)), dead_time_ns);
  #else
  uint32_t dead_time = __LL_TIM_CALC_DEADTIME(stm32_getTimerClockFreq(handle), LL_TIM_GetClockDivision(TIMER_INSTANCE(handle)), dead_time_ns);
  #endif
  if (dead_time>255) dead_time = 255;
  if (dead_time==0 && dead_zone>0) {
    dead_time = 255; // LL_TIM_CALC_DEADTIME returns 0 if dead_time_ns is too large
    SIMPLEFOC_DEBUG("STM32-DRV: WARN: dead time too large, setting to max");
  }
  // make sure timer output goes to LOW when timer channels are temporarily disabled
  // TODO why init this here, and not generally in the initPWM or init timer functions?
  //      or, since its a rather specialized and hardware-speific setting, why not move it to
  //      another function?
  LL_TIM_SetOffStates(TIMER_INSTANCE(handle), LL_TIM_OSSI_DISABLE, LL_TIM_OSSR_ENABLE);
  LL_TIM_OC_SetDeadTime(TIMER_INSTANCE(handle), dead_time); // deadtime is non linear!
  LL_TIM_CC_EnableChannel(TIMER_INSTANCE(handle), stm32_getLLChannel(pinH) | stm32_getLLChannel(pinL));
  params->timers_handle[paramsPos] = handle;
  params->timers_handle[paramsPos+1] = handle;
  params->channels[paramsPos] = channel1;
  params->channels[paramsPos+1] = channel2;
  params->llchannels[paramsPos] = stm32_getLLChannel(pinH);
  params->llchannels[paramsPos+1] = stm32_getLLChannel(pinL);
  return params;
}




STM32DriverParams* _stm32_initHardware6PWMInterface(long PWM_freq, float dead_zone, PinMap* pinA_h, PinMap* pinA_l, PinMap* pinB_h, PinMap* pinB_l, PinMap* pinC_h, PinMap* pinC_l) {
  STM32DriverParams* params = new STM32DriverParams {
    .timers_handle = { NULL, NULL, NULL, NULL, NULL, NULL },
    .channels = { 0, 0, 0, 0, 0, 0 },
    .llchannels = { 0, 0, 0, 0, 0, 0 },
    .pwm_frequency = PWM_freq,
    .num_timers = 0,
    .master_timer = NULL,
    .dead_zone = dead_zone,
    .interface_type = _HARDWARE_6PWM
  };
  if (_stm32_initHardware6PWMPair(PWM_freq, dead_zone, pinA_h, pinA_l, params, 0) == SIMPLEFOC_DRIVER_INIT_FAILED)
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
  if(_stm32_initHardware6PWMPair(PWM_freq, dead_zone, pinB_h, pinB_l, params, 2) == SIMPLEFOC_DRIVER_INIT_FAILED)
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
  if (_stm32_initHardware6PWMPair(PWM_freq, dead_zone, pinC_h, pinC_l, params, 4) == SIMPLEFOC_DRIVER_INIT_FAILED)
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
  params->num_timers = stm32_countTimers(params->timers_handle, 6);
  return params;
}



#ifdef USE_HALV2_DRIVER
  #define InitPinParam HAL_TIM_OC_PWM1, (SIMPLEFOC_PWM_ACTIVE_HIGH)?HAL_TIM_OC_HIGH:HAL_TIM_OC_LOW
#else
  #define InitPinParam TIM_OCMODE_PWM1, (SIMPLEFOC_PWM_ACTIVE_HIGH)?TIM_OCPOLARITY_HIGH:TIM_OCPOLARITY_LOW
#endif

void* _configure1PWM(long pwm_frequency, const int pinA) {
  if (numMotorsUsed+1 > SIMPLEFOC_STM32_MAX_MOTORSUSED) {
    SIMPLEFOC_DEBUG("STM32-DRV: ERR: too many drivers used");
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
  }

  if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = SIMPLEFOC_STM32_PWM_FREQUENCY; // default frequency 25khz

  int pins[1] = { pinA };
  PinMap* pinTimers[1] = { NULL };
  if (stm32_findBestTimerCombination(1, pins, pinTimers)<0)
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;

  TIMER_HANDLE_TYPE* HT1 = stm32_initPinPWM(pwm_frequency, pinTimers[0], InitPinParam);
  uint32_t channel1 = STM_PIN_CHANNEL(pinTimers[0]->function);

  STM32DriverParams* params = new STM32DriverParams {
    .timers_handle = { HT1 },
    .channels = { channel1 },
    .llchannels = { stm32_getLLChannel(pinTimers[0]) },
    .pwm_frequency = pwm_frequency,
    .num_timers = 1,
    .master_timer = NULL
  };
  // align the timers (in this case, just start them)
  params->master_timer = stm32_alignTimers(params->timers_handle, 1);
  motorsUsed[numMotorsUsed++] = params;
  return params;
}





// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 2PWM setting
// - hardware speciffic
void* _configure2PWM(long pwm_frequency, const int pinA, const int pinB) {
  if (numMotorsUsed+1 > SIMPLEFOC_STM32_MAX_MOTORSUSED) {
    SIMPLEFOC_DEBUG("STM32-DRV: ERR: too many drivers used");
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
  }

  if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = SIMPLEFOC_STM32_PWM_FREQUENCY; // default frequency 25khz

  int pins[2] = { pinA, pinB };
  PinMap* pinTimers[2] = { NULL, NULL };
  if (stm32_findBestTimerCombination(2, pins, pinTimers)<0)
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;

  TIMER_HANDLE_TYPE* HT1 = stm32_initPinPWM(pwm_frequency, pinTimers[0], InitPinParam);
  TIMER_HANDLE_TYPE* HT2 = stm32_initPinPWM(pwm_frequency, pinTimers[1], InitPinParam);
  TIMER_HANDLE_TYPE *timers[2] = {HT1, HT2};
  stm32_checkTimerFrequency(pwm_frequency, timers, 2); 
  
  uint32_t channel1 = STM_PIN_CHANNEL(pinTimers[0]->function);
  uint32_t channel2 = STM_PIN_CHANNEL(pinTimers[1]->function);

  STM32DriverParams* params = new STM32DriverParams {
    .timers_handle = { HT1, HT2 },
    .channels = { channel1, channel2 },
    .llchannels = { stm32_getLLChannel(pinTimers[0]), stm32_getLLChannel(pinTimers[1]) },
    .pwm_frequency = pwm_frequency, // TODO set to actual frequency
    .num_timers = stm32_countTimers(timers, 2),
    .master_timer = NULL
  };
  // align the timers
  params->master_timer = stm32_alignTimers(timers, 2);
  motorsUsed[numMotorsUsed++] = params;
  return params;
}




// function setting the high pwm frequency to the supplied pins
// - BLDC motor - 3PWM setting
// - hardware speciffic
void* _configure3PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC) {
  if (numMotorsUsed+1 > SIMPLEFOC_STM32_MAX_MOTORSUSED) {
    SIMPLEFOC_DEBUG("STM32-DRV: ERR: too many drivers used");
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
  }
  if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = SIMPLEFOC_STM32_PWM_FREQUENCY; // default frequency 25khz

  int pins[3] = { pinA, pinB, pinC };
  PinMap* pinTimers[3] = { NULL, NULL, NULL };
  if (stm32_findBestTimerCombination(3, pins, pinTimers)<0)
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;

  TIMER_HANDLE_TYPE* HT1 = stm32_initPinPWM(pwm_frequency, pinTimers[0], InitPinParam);
  TIMER_HANDLE_TYPE* HT2 = stm32_initPinPWM(pwm_frequency, pinTimers[1], InitPinParam);
  TIMER_HANDLE_TYPE* HT3 = stm32_initPinPWM(pwm_frequency, pinTimers[2], InitPinParam);

  TIMER_HANDLE_TYPE *timers[3] = {HT1, HT2, HT3};
  stm32_checkTimerFrequency(pwm_frequency, timers, 3);

  uint32_t channel1 = STM_PIN_CHANNEL(pinTimers[0]->function);
  uint32_t channel2 = STM_PIN_CHANNEL(pinTimers[1]->function);
  uint32_t channel3 = STM_PIN_CHANNEL(pinTimers[2]->function);

  STM32DriverParams* params = new STM32DriverParams {
    .timers_handle = { HT1, HT2, HT3 },
    .channels = { channel1, channel2, channel3 },
    .llchannels = { stm32_getLLChannel(pinTimers[0]), stm32_getLLChannel(pinTimers[1]), stm32_getLLChannel(pinTimers[2]) },
    .pwm_frequency = pwm_frequency,
    .num_timers = stm32_countTimers(timers, 3),
    .master_timer = NULL
  };
  params->master_timer = stm32_alignTimers(timers, 3);
  motorsUsed[numMotorsUsed++] = params;
  return params;
}



// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 4PWM setting
// - hardware speciffic
void* _configure4PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC, const int pinD) {
  if (numMotorsUsed+1 > SIMPLEFOC_STM32_MAX_MOTORSUSED) {
    SIMPLEFOC_DEBUG("STM32-DRV: ERR: too many drivers used");
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
  }
  if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = SIMPLEFOC_STM32_PWM_FREQUENCY; // default frequency 25khz

  int pins[4] = { pinA, pinB, pinC, pinD };
  PinMap* pinTimers[4] = { NULL, NULL, NULL, NULL };
  if (stm32_findBestTimerCombination(4, pins, pinTimers)<0)
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;

  TIMER_HANDLE_TYPE* HT1 = stm32_initPinPWM(pwm_frequency, pinTimers[0], InitPinParam);
  TIMER_HANDLE_TYPE* HT2 = stm32_initPinPWM(pwm_frequency, pinTimers[1], InitPinParam);
  TIMER_HANDLE_TYPE* HT3 = stm32_initPinPWM(pwm_frequency, pinTimers[2], InitPinParam);
  TIMER_HANDLE_TYPE* HT4 = stm32_initPinPWM(pwm_frequency, pinTimers[3], InitPinParam);
  TIMER_HANDLE_TYPE *timers[4] = {HT1, HT2, HT3, HT4};
  stm32_checkTimerFrequency(pwm_frequency, timers, 4);

  uint32_t channel1 = STM_PIN_CHANNEL(pinTimers[0]->function);
  uint32_t channel2 = STM_PIN_CHANNEL(pinTimers[1]->function);
  uint32_t channel3 = STM_PIN_CHANNEL(pinTimers[2]->function);
  uint32_t channel4 = STM_PIN_CHANNEL(pinTimers[3]->function);

  STM32DriverParams* params = new STM32DriverParams {
    .timers_handle = { HT1, HT2, HT3, HT4 },
    .channels = { channel1, channel2, channel3, channel4 },
    .llchannels = { stm32_getLLChannel(pinTimers[0]), stm32_getLLChannel(pinTimers[1]), stm32_getLLChannel(pinTimers[2]), stm32_getLLChannel(pinTimers[3]) },
    .pwm_frequency = pwm_frequency,
    .num_timers = stm32_countTimers(timers, 4),
    .master_timer = NULL
  };
  params->master_timer = stm32_alignTimers(timers, 4);
  motorsUsed[numMotorsUsed++] = params;
  return params;
}



// function setting the pwm duty cycle to the hardware
// - DC motor - 1PWM setting
// - hardware specific
void _writeDutyCycle1PWM(float dc_a, void* params){
  uint32_t duty = dc_a*(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[0])->ARR+1);
  stm32_setPwm(((STM32DriverParams*)params)->timers_handle[0], ((STM32DriverParams*)params)->channels[0], duty);
}



// function setting the pwm duty cycle to the hardware
// - Stepper motor - 2PWM setting
//- hardware specific
void _writeDutyCycle2PWM(float dc_a,  float dc_b, void* params){
  uint32_t duty1 = dc_a*(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[0])->ARR+1);
  uint32_t duty2 = dc_b*(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[1])->ARR+1);
  if (((STM32DriverParams*)params)->num_timers == 1) LL_TIM_DisableUpdateEvent(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[0]));
  stm32_setPwm(((STM32DriverParams*)params)->timers_handle[0], ((STM32DriverParams*)params)->channels[0], duty1);
  stm32_setPwm(((STM32DriverParams*)params)->timers_handle[1], ((STM32DriverParams*)params)->channels[1], duty2);
  if (((STM32DriverParams*)params)->num_timers == 1) LL_TIM_EnableUpdateEvent(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[0]));
}

// function setting the pwm duty cycle to the hardware
// - BLDC motor - 3PWM setting
//- hardware specific
void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, void* params){
  uint32_t duty1 = dc_a*(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[0])->ARR+1);
  uint32_t duty2 = dc_b*(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[1])->ARR+1);
  uint32_t duty3 = dc_c*(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[2])->ARR+1);
  if (((STM32DriverParams*)params)->num_timers == 1) LL_TIM_DisableUpdateEvent(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[0]));
  stm32_setPwm(((STM32DriverParams*)params)->timers_handle[0], ((STM32DriverParams*)params)->channels[0], duty1);
  stm32_setPwm(((STM32DriverParams*)params)->timers_handle[1], ((STM32DriverParams*)params)->channels[1], duty2);
  stm32_setPwm(((STM32DriverParams*)params)->timers_handle[2], ((STM32DriverParams*)params)->channels[2], duty3);
  if (((STM32DriverParams*)params)->num_timers == 1) LL_TIM_EnableUpdateEvent(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[0]));
}


// function setting the pwm duty cycle to the hardware
// - Stepper motor - 4PWM setting
//- hardware specific
void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, void* params){
  uint32_t duty1 = dc_1a*(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[0])->ARR+1);
  uint32_t duty2 = dc_1b*(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[1])->ARR+1);
  uint32_t duty3 = dc_2a*(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[2])->ARR+1);
  uint32_t duty4 = dc_2b*(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[3])->ARR+1);
  if (((STM32DriverParams*)params)->num_timers == 1) LL_TIM_DisableUpdateEvent(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[0]));
  stm32_setPwm(((STM32DriverParams*)params)->timers_handle[0], ((STM32DriverParams*)params)->channels[0], duty1);
  stm32_setPwm(((STM32DriverParams*)params)->timers_handle[1], ((STM32DriverParams*)params)->channels[1], duty2);
  stm32_setPwm(((STM32DriverParams*)params)->timers_handle[2], ((STM32DriverParams*)params)->channels[2], duty3);
  stm32_setPwm(((STM32DriverParams*)params)->timers_handle[3], ((STM32DriverParams*)params)->channels[3], duty4);
  if (((STM32DriverParams*)params)->num_timers == 1) LL_TIM_EnableUpdateEvent(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[0]));
}




// Configuring PWM frequency, resolution and alignment
// - BLDC driver - 6PWM setting
// - hardware specific
void* _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l, const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l){
  if (numMotorsUsed+1 > SIMPLEFOC_STM32_MAX_MOTORSUSED) {
    SIMPLEFOC_DEBUG("STM32-DRV: ERR: too many drivers used");
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
  }
  if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = SIMPLEFOC_STM32_PWM_FREQUENCY; // default frequency 25khz

  // find configuration
  int pins[6] = { pinA_h, pinA_l, pinB_h, pinB_l, pinC_h, pinC_l };
  PinMap* pinTimers[6] = { NULL, NULL, NULL, NULL, NULL, NULL };
  int score = stm32_findBestTimerCombination(6, pins, pinTimers);

  STM32DriverParams* params;
  // configure accordingly
  if (score<0)
    params = (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
  else if (score<10)  // hardware pwm
    params = _stm32_initHardware6PWMInterface(pwm_frequency, dead_zone, pinTimers[0], pinTimers[1], pinTimers[2], pinTimers[3], pinTimers[4], pinTimers[5]);
  else {  // software pwm
    TIMER_HANDLE_TYPE* HT1 = _stm32_initPinPWMHigh(pwm_frequency, pinTimers[0]);
    TIMER_HANDLE_TYPE* HT2 = _stm32_initPinPWMLow(pwm_frequency, pinTimers[1]);
    TIMER_HANDLE_TYPE* HT3 = _stm32_initPinPWMHigh(pwm_frequency, pinTimers[2]);
    TIMER_HANDLE_TYPE* HT4 = _stm32_initPinPWMLow(pwm_frequency, pinTimers[3]);
    TIMER_HANDLE_TYPE* HT5 = _stm32_initPinPWMHigh(pwm_frequency, pinTimers[4]);
    TIMER_HANDLE_TYPE* HT6 = _stm32_initPinPWMLow(pwm_frequency, pinTimers[5]);
    TIMER_HANDLE_TYPE *timers[6] = {HT1, HT2, HT3, HT4, HT5, HT6};
    stm32_checkTimerFrequency(pwm_frequency, timers, 6); 
    uint32_t channel1 = STM_PIN_CHANNEL(pinTimers[0]->function);
    uint32_t channel2 = STM_PIN_CHANNEL(pinTimers[1]->function);
    uint32_t channel3 = STM_PIN_CHANNEL(pinTimers[2]->function);
    uint32_t channel4 = STM_PIN_CHANNEL(pinTimers[3]->function);
    uint32_t channel5 = STM_PIN_CHANNEL(pinTimers[4]->function);
    uint32_t channel6 = STM_PIN_CHANNEL(pinTimers[5]->function);
    params = new STM32DriverParams {
      .timers_handle = { HT1, HT2, HT3, HT4, HT5, HT6 },
      .channels = { channel1, channel2, channel3, channel4, channel5, channel6 },
      .llchannels = { stm32_getLLChannel(pinTimers[0]), stm32_getLLChannel(pinTimers[1]), stm32_getLLChannel(pinTimers[2]), stm32_getLLChannel(pinTimers[3]), stm32_getLLChannel(pinTimers[4]), stm32_getLLChannel(pinTimers[5]) },
      .pwm_frequency = pwm_frequency,
      .num_timers = stm32_countTimers(timers, 6),
      .master_timer = NULL,
      .dead_zone = dead_zone,
      .interface_type = _SOFTWARE_6PWM
    };
  }
  if (score>=0) {
    params->master_timer = stm32_alignTimers(params->timers_handle, 6);
    motorsUsed[numMotorsUsed++] = params;
  }
  return params; // success
}



void _setSinglePhaseState(PhaseState state, TIMER_HANDLE_TYPE *HT, int llchannel_hi, int llchannel_lo) {
  switch (state) {
    case PhaseState::PHASE_OFF:
      stm32_pauseChannel(HT, llchannel_hi | llchannel_lo);
      break;
    case PhaseState::PHASE_HI:
      stm32_pauseChannel(HT, llchannel_lo);
      stm32_resumeChannel(HT, llchannel_hi);
      break;
    case PhaseState::PHASE_LO:
      stm32_pauseChannel(HT, llchannel_hi);
      stm32_resumeChannel(HT, llchannel_lo);
      break;
    case PhaseState::PHASE_ON:
    default:
      stm32_resumeChannel(HT, llchannel_hi | llchannel_lo);
      break;
  }
}


// Function setting the duty cycle to the pwm pin (ex. analogWrite())
// - BLDC driver - 6PWM setting
// - hardware specific
void _writeDutyCycle6PWM(float dc_a, float dc_b, float dc_c, PhaseState* phase_state, void* params){
  uint32_t duty1;
  uint32_t duty2;
  uint32_t duty3;
  switch(((STM32DriverParams*)params)->interface_type){
    case _HARDWARE_6PWM:
      duty1 = dc_a*(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[0])->ARR+1);
      duty2 = dc_b*(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[2])->ARR+1);
      duty3 = dc_c*(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[4])->ARR+1);
      if(phase_state[0] == PhaseState::PHASE_OFF) duty1 = 0;
      if(phase_state[1] == PhaseState::PHASE_OFF) duty2 = 0;
      if(phase_state[2] == PhaseState::PHASE_OFF) duty3 = 0;
      if (((STM32DriverParams*)params)->num_timers == 1) LL_TIM_DisableUpdateEvent(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[0]));
      _setSinglePhaseState(phase_state[0], ((STM32DriverParams*)params)->timers_handle[0], ((STM32DriverParams*)params)->llchannels[0], ((STM32DriverParams*)params)->llchannels[1]);
      stm32_setPwm(((STM32DriverParams*)params)->timers_handle[0], ((STM32DriverParams*)params)->channels[0], duty1);
      _setSinglePhaseState(phase_state[1], ((STM32DriverParams*)params)->timers_handle[2], ((STM32DriverParams*)params)->llchannels[2], ((STM32DriverParams*)params)->llchannels[3]);
      stm32_setPwm(((STM32DriverParams*)params)->timers_handle[2], ((STM32DriverParams*)params)->channels[2], duty2);
      _setSinglePhaseState(phase_state[2], ((STM32DriverParams*)params)->timers_handle[4], ((STM32DriverParams*)params)->llchannels[4], ((STM32DriverParams*)params)->llchannels[5]);
      stm32_setPwm(((STM32DriverParams*)params)->timers_handle[4], ((STM32DriverParams*)params)->channels[4], duty3);
      if (((STM32DriverParams*)params)->num_timers == 1) LL_TIM_EnableUpdateEvent(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[0]));
      break;
    case _SOFTWARE_6PWM:
      float dead_zone = ((STM32DriverParams*)params)->dead_zone  / 2.0f;
      duty1 = _constrain(dc_a - dead_zone, 0.0f, 1.0f)*(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[0])->ARR+1);
      duty2 = _constrain(dc_b - dead_zone, 0.0f, 1.0f)*(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[2])->ARR+1);
      duty3 = _constrain(dc_c - dead_zone, 0.0f, 1.0f)*(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[4])->ARR+1);
      uint32_t duty1N = _constrain(dc_a + dead_zone, 0.0f, 1.0f)*(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[1])->ARR+1);
      uint32_t duty2N = _constrain(dc_b + dead_zone, 0.0f, 1.0f)*(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[3])->ARR+1);
      uint32_t duty3N = _constrain(dc_c + dead_zone, 0.0f, 1.0f)*(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[5])->ARR+1);

      LL_TIM_DisableUpdateEvent(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[0])); // timers for high and low side assumed to be the same timer
      if (phase_state[0] == PhaseState::PHASE_ON || phase_state[0] == PhaseState::PHASE_HI)
        stm32_setPwm(((STM32DriverParams*)params)->timers_handle[0], ((STM32DriverParams*)params)->channels[0], duty1);
      else
        stm32_setPwm(((STM32DriverParams*)params)->timers_handle[0], ((STM32DriverParams*)params)->channels[0], 0);
      if (phase_state[0] == PhaseState::PHASE_ON || phase_state[0] == PhaseState::PHASE_LO)
        stm32_setPwm(((STM32DriverParams*)params)->timers_handle[1], ((STM32DriverParams*)params)->channels[1], duty1N);
      else
        stm32_setPwm(((STM32DriverParams*)params)->timers_handle[1], ((STM32DriverParams*)params)->channels[1], 0);

      LL_TIM_DisableUpdateEvent(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[2])); // timers for high and low side assumed to be the same timer
      if (phase_state[1] == PhaseState::PHASE_ON || phase_state[1] == PhaseState::PHASE_HI)
        stm32_setPwm(((STM32DriverParams*)params)->timers_handle[2], ((STM32DriverParams*)params)->channels[2], duty2);
      else
        stm32_setPwm(((STM32DriverParams*)params)->timers_handle[2], ((STM32DriverParams*)params)->channels[2], 0);
      if (phase_state[1] == PhaseState::PHASE_ON || phase_state[1] == PhaseState::PHASE_LO)
        stm32_setPwm(((STM32DriverParams*)params)->timers_handle[3], ((STM32DriverParams*)params)->channels[3], duty2N);
      else
        stm32_setPwm(((STM32DriverParams*)params)->timers_handle[3], ((STM32DriverParams*)params)->channels[3], 0);

      LL_TIM_DisableUpdateEvent(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[4])); // timers for high and low side assumed to be the same timer
      if (phase_state[2] == PhaseState::PHASE_ON || phase_state[2] == PhaseState::PHASE_HI)
        stm32_setPwm(((STM32DriverParams*)params)->timers_handle[4], ((STM32DriverParams*)params)->channels[4], duty3);
      else
        stm32_setPwm(((STM32DriverParams*)params)->timers_handle[4], ((STM32DriverParams*)params)->channels[4], 0);
      if (phase_state[2] == PhaseState::PHASE_ON || phase_state[2] == PhaseState::PHASE_LO)
        stm32_setPwm(((STM32DriverParams*)params)->timers_handle[5], ((STM32DriverParams*)params)->channels[5], duty3N);
      else
        stm32_setPwm(((STM32DriverParams*)params)->timers_handle[5], ((STM32DriverParams*)params)->channels[5], 0);
      
      LL_TIM_EnableUpdateEvent(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[0]));
      LL_TIM_EnableUpdateEvent(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[2]));
      LL_TIM_EnableUpdateEvent(TIMER_INSTANCE(((STM32DriverParams*)params)->timers_handle[4]));
      break;
  }
}



#endif
