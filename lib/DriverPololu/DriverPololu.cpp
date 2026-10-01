
#include "DriverPololu.h"
#include "Hw171Manager.h"

#include "nrf.h"
#include "nrf_gpiote.h"
#include "nrf_ppi.h"
#include "nrf_timer.h"


void initPWMMotors() {

  NRF_PWM0->PSEL.OUT[0] = (PIN_PWMA << PWM_PSEL_OUT_PIN_Pos) |
                          (1UL << PWM_PSEL_OUT_PORT_Pos) | 
                          (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);


  NRF_PWM0->PSEL.OUT[1] = (PIN_PWMB << PWM_PSEL_OUT_PIN_Pos) |
                          (1UL << PWM_PSEL_OUT_PORT_Pos) |  
                          (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);


  NRF_PWM0->PSEL.OUT[2] = (PWM_PSEL_OUT_CONNECT_Disconnected << PWM_PSEL_OUT_CONNECT_Pos);
  NRF_PWM0->PSEL.OUT[3] = (PWM_PSEL_OUT_CONNECT_Disconnected << PWM_PSEL_OUT_CONNECT_Pos);

 
  NRF_PWM0->PRESCALER = PWM_PRESCALER_PRESCALER_DIV_16;


  NRF_PWM0->MODE = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
  NRF_PWM0->COUNTERTOP = PWM_TOP_VALUE; 


  NRF_PWM0->DECODER = (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) |
                      (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);


  NRF_PWM0->SEQ[0].PTR = (uint32_t)pwmDutyCycleBuffer;
  NRF_PWM0->SEQ[0].CNT = 2; 
  NRF_PWM0->SEQ[0].REFRESH = 0;
  NRF_PWM0->SEQ[0].ENDDELAY = 0;

  NRF_PWM0->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
  NRF_PWM0->TASKS_SEQSTART[0] = 1;
}

void updatePWM(float porcent, u_int8_t pin) {
  porcent = constrain(porcent, 0.0f, 100.0f);

  uint16_t duty1 = (uint16_t)((porcent / 100.0f) * PWM_TOP_VALUE);

  pwmDutyCycleBuffer[pin] = duty1;

  NRF_PWM0->TASKS_SEQSTART[0] = 1;
}



DriverPololu::DriverPololu(MOTORID idMotor,  Hw171Manager &hw171manager): _idMotor(idMotor), _Hw171manager(&hw171manager)
{
    _velocity = 0;
    _direction = 0;
    _counterCurrentEncoder = 0;
    _couterLastEnconder = 0;


    if(_idMotor == DriverPololu::MOTORID::LEFT){
        _HW_TIMER = NRF_TIMER1;
        _CHANNEL_GPIOTE = CHANNEL_GPIOTE_A;
        _CHANNEL_PPI = CHANNEL_PPI_A;
        _PIN_ENCODER = PIN_ENCODER_A;

    }else if(_idMotor == DriverPololu::MOTORID::RIGHT){
        _HW_TIMER = NRF_TIMER2;
        _CHANNEL_GPIOTE = CHANNEL_GPIOTE_B;
        _CHANNEL_PPI = CHANNEL_PPI_B;
        _PIN_ENCODER = PIN_ENCODER_B;
    }
    
    DriverPololu::initCounterEncoder();
    initPWMMotors();
}

DriverPololu::~DriverPololu()
{
}


void DriverPololu::initCounterEncoder(){
    _HW_TIMER->TASKS_STOP = 1;
    _HW_TIMER->TASKS_CLEAR = 1;
    _HW_TIMER->MODE = TIMER_MODE_MODE_Counter;
    _HW_TIMER->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    _HW_TIMER->TASKS_START = 1;

    NRF_P0->PIN_CNF[_PIN_ENCODER] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                          (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos);

    NRF_GPIOTE->CONFIG[_CHANNEL_GPIOTE] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
                                            (_PIN_ENCODER << GPIOTE_CONFIG_PSEL_Pos) |
                                            (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos);

    // 3. Conectar evento GPIOTE -> Tarea TIMER1 vía PPI Canal 0
    NRF_PPI->CH[_CHANNEL_PPI].EEP = (uint32_t)&NRF_GPIOTE->EVENTS_IN[_CHANNEL_PPI];
    NRF_PPI->CH[_CHANNEL_PPI].TEP = (uint32_t)&_HW_TIMER->TASKS_COUNT;
    NRF_PPI->CHENSET = (1UL << _CHANNEL_PPI);
}


u_int32_t DriverPololu::currentCounter(){

    _HW_TIMER->TASKS_CAPTURE[0] = 1; 
    _counterCurrentEncoder =  _HW_TIMER->CC[0];
    return _counterCurrentEncoder;
}


float DriverPololu::updateVelocity(){

    _HW_TIMER->TASKS_CAPTURE[0] = 1; 
    _counterCurrentEncoder =  _HW_TIMER->CC[0];

    if(FREQVELOCITY <= 0) return 0.0f;

    _velocity = ((_counterCurrentEncoder - _couterLastEnconder)/PULSE_PER_REV)*(60.0f*FREQVELOCITY);
    _couterLastEnconder = _counterCurrentEncoder;
    return _velocity;
}

void DriverPololu::setVelocity(float porcent){
    if(_idMotor == DriverPololu::MOTORID::LEFT){
        updatePWM(porcent, (uint8_t)0);
    }else if(_idMotor == DriverPololu::MOTORID::RIGHT){
        updatePWM(porcent, (uint8_t)1);
    }
}