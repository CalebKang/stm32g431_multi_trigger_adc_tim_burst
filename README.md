# stm32g431_multi_trigger_adc_tim_burst

DMA Burst 기능을 구현, ADC Trigger는 Timer8의 업데이트

main.h의 #define BURST_EVERY_UPDATE 을 제거하면 Upcouting에서 두번의 CCR값을 지정하여

Upcounting 때 CCR 값 변경, Downcounting 때 CCR 값 변경이 자동으로 이루어 지며

#define BURST_EVERY_UPDATE를 그대로 두면

Upcounting 때 CCR 값 변경, Downcounting 때 CCR 값 변경을 코드에서 해주어야 
