#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "TM4C123.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stdio.h"

// --------------------- Defines ---------------------
#define TRIGGER_PIN (1U << 6)
#define ECHO_PIN    (1U << 7)
#define BUZZER_PIN  (1U << 2)

#define LCD_RS (1U << 0)
#define LCD_E  (1U << 1)
#define LCD_D0 (1U << 0)
#define LCD_D1 (1U << 1)
#define LCD_D2 (1U << 4)
#define LCD_D3 (1U << 5)
#define LCD_D4 (1U << 3)
#define LCD_D5 (1U << 4)
#define LCD_D6 (1U << 5)
#define LCD_D7 (1U << 5)

#define LIMIT_SWITCH (1U << 0)
#define IGNITION_BTN (1U << 6)
#define GEAR_P (1U << 4)
#define GEAR_R (1U << 5)
#define GEAR_N (1U << 6)
#define GEAR_D (1U << 7)

#define RGB_RED    (1U << 1)  // PF1
#define RGB_GREEN  (1U << 3)  // PF3
#define RGB_BLUE   (1U << 2)  // PF2

// --------------------- Globals ---------------------
volatile uint32_t distanceValue = 0, speedValue = 0;
volatile bool doorsLocked = false, ignitionOn = false;
volatile bool doorEvent = false, ignitionEvent = false, gearEvent = false;
char gearState[2] = "N";

// --------------------- LCD ---------------------
void LCD_DelayUs(uint32_t us) { for (volatile uint32_t i = 0; i < (us * 4); i++); }
void LCD_PulseEnable(void) { GPIOD->DATA |= LCD_E; LCD_DelayUs(2); GPIOD->DATA &= ~LCD_E; LCD_DelayUs(2); }

void LCD_SendByte(uint8_t data) {
    GPIOB->DATA = (GPIOB->DATA & ~(LCD_D0|LCD_D1|LCD_D2|LCD_D3)) |
                  ((data & 0x01) ? LCD_D0 : 0) | ((data & 0x02) ? LCD_D1 : 0) |
                  ((data & 0x04) ? LCD_D2 : 0) | ((data & 0x08) ? LCD_D3 : 0);
    GPIOE->DATA = (GPIOE->DATA & ~(LCD_D4|LCD_D5|LCD_D6)) |
                  ((data & 0x10) ? LCD_D4 : 0) | ((data & 0x20) ? LCD_D5 : 0) |
                  ((data & 0x40) ? LCD_D6 : 0);
    GPIOA->DATA = (GPIOA->DATA & ~LCD_D7) | ((data & 0x80) ? LCD_D7 : 0);
    LCD_PulseEnable();
}

void LCD_Command(uint8_t cmd) { GPIOD->DATA &= ~LCD_RS; LCD_SendByte(cmd); LCD_DelayUs(50); }
void LCD_Data(uint8_t data) { GPIOD->DATA |= LCD_RS; LCD_SendByte(data); LCD_DelayUs(50); }
void LCD_Print(char *str) { while (*str) LCD_Data(*str++); }

void LCD_Init(void) {
    SYSCTL->RCGCGPIO |= (1U<<0)|(1U<<1)|(1U<<4)|(1U<<5)|(1U<<3);
    while (!(SYSCTL->PRGPIO & (1U<<0))); while (!(SYSCTL->PRGPIO & (1U<<1)));
    while (!(SYSCTL->PRGPIO & (1U<<4))); while (!(SYSCTL->PRGPIO & (1U<<5))); while (!(SYSCTL->PRGPIO & (1U<<3)));
    GPIOB->DIR |= LCD_D0|LCD_D1|LCD_D2|LCD_D3; GPIOB->DEN |= LCD_D0|LCD_D1|LCD_D2|LCD_D3;
    GPIOE->DIR |= LCD_D4|LCD_D5|LCD_D6;        GPIOE->DEN |= LCD_D4|LCD_D5|LCD_D6;
    GPIOA->DIR |= LCD_D7;                      GPIOA->DEN |= LCD_D7;
    GPIOD->DIR |= LCD_RS|LCD_E;               GPIOD->DEN |= LCD_RS|LCD_E;

    LCD_DelayUs(50000);
    LCD_Command(0x38); LCD_Command(0x0C); LCD_Command(0x06); LCD_Command(0x01);
    LCD_DelayUs(2000);
}

// --------------------- RGB ---------------------
void InitRGB(void) {
    SYSCTL->RCGCGPIO |= (1U << 5);
    while (!(SYSCTL->PRGPIO & (1U << 5)));
    GPIOF->LOCK = 0x4C4F434B;
    GPIOF->CR |= 0x1F;
    GPIOF->DIR |= (RGB_RED | RGB_GREEN | RGB_BLUE);
    GPIOF->DEN |= (RGB_RED | RGB_GREEN | RGB_BLUE);
    GPIOF->DATA &= ~(RGB_RED | RGB_GREEN | RGB_BLUE);
}

// --------------------- Ultrasonic ---------------------
void InitUltrasonic(void) {
    SYSCTL->RCGCGPIO |= (1U<<0);
    while (!(SYSCTL->PRGPIO & (1U<<0)));
    GPIOA->DIR |= TRIGGER_PIN;
    GPIOA->DIR &= ~ECHO_PIN;
    GPIOA->DEN |= TRIGGER_PIN | ECHO_PIN;
}

uint32_t MeasureDistance(void) {
    uint32_t d = 0, t = 0;
    GPIOA->DATA &= ~TRIGGER_PIN; for (volatile int i = 0; i < 10; i++);
    GPIOA->DATA |= TRIGGER_PIN;  for (volatile int i = 0; i < 160; i++);
    GPIOA->DATA &= ~TRIGGER_PIN;
    while (!(GPIOA->DATA & ECHO_PIN)) if (++t > 60000) return 0;
    t = 0;
    while (GPIOA->DATA & ECHO_PIN) { d++; if (++t > 120000) break; }
    return d / 58;
}

// --------------------- ADC ---------------------
void InitADC(void) {
    SYSCTL->RCGCADC |= (1U<<0);
    SYSCTL->RCGCGPIO |= (1U<<4);
    while (!(SYSCTL->PRGPIO & (1U<<4)));
    GPIOE->DIR &= ~(1U<<2);
    GPIOE->AFSEL |= (1U<<2);
    GPIOE->DEN &= ~(1U<<2);
    GPIOE->AMSEL |= (1U<<2);
    ADC0->ACTSS &= ~(1U<<3);
    ADC0->EMUX &= ~(0xF<<12);
    ADC0->SSMUX3 = 1;
    ADC0->SSCTL3 = 0x06;
    ADC0->ACTSS |= (1U<<3);
}

uint32_t ReadADC(void) {
    ADC0->PSSI = (1U<<3);
    while ((ADC0->RIS & (1U<<3)) == 0);
    uint32_t r = ADC0->SSFIFO3 & 0xFFF;
    ADC0->ISC = (1U<<3);
    return r;
}

// --------------------- Buzzer ---------------------
void InitBuzzer(void) {
    SYSCTL->RCGCGPIO |= (1U<<1);
    while (!(SYSCTL->PRGPIO & (1U<<1)));
    GPIOB->DIR |= BUZZER_PIN;
    GPIOB->DEN |= BUZZER_PIN;
    GPIOB->DATA &= ~BUZZER_PIN;
}

void BeepBuzzer(uint32_t on_ms, uint32_t off_ms) {
    GPIOB->DATA |= BUZZER_PIN; vTaskDelay(pdMS_TO_TICKS(on_ms));
    GPIOB->DATA &= ~BUZZER_PIN; vTaskDelay(pdMS_TO_TICKS(off_ms));
}

// --------------------- Inputs ---------------------
void InitInputs(void) {
    SYSCTL->RCGCGPIO |= (1U<<2)|(1U<<3)|(1U<<4);
    while (!(SYSCTL->PRGPIO & (1U<<2))); while (!(SYSCTL->PRGPIO & (1U<<3))); while (!(SYSCTL->PRGPIO & (1U<<4)));

    GPIOE->DIR &= ~LIMIT_SWITCH; GPIOE->DEN |= LIMIT_SWITCH; GPIOE->PUR |= LIMIT_SWITCH;
    GPIOD->DIR &= ~IGNITION_BTN; GPIOD->DEN |= IGNITION_BTN; GPIOD->PUR |= IGNITION_BTN;
    GPIOC->DIR &= ~(GEAR_P|GEAR_R|GEAR_N|GEAR_D); GPIOC->DEN |= (GEAR_P|GEAR_R|GEAR_N|GEAR_D); GPIOC->PDR |= (GEAR_P|GEAR_R|GEAR_N|GEAR_D);

    GPIOE->IS &= ~LIMIT_SWITCH; GPIOE->IBE &= ~LIMIT_SWITCH; GPIOE->IEV &= ~LIMIT_SWITCH; GPIOE->IM |= LIMIT_SWITCH;
    GPIOD->IS &= ~IGNITION_BTN; GPIOD->IBE &= ~IGNITION_BTN; GPIOD->IEV &= ~IGNITION_BTN; GPIOD->IM |= IGNITION_BTN;
    GPIOC->IS &= ~(GEAR_P|GEAR_R|GEAR_N|GEAR_D); GPIOC->IBE &= ~(GEAR_P|GEAR_R|GEAR_N|GEAR_D);
    GPIOC->IEV |= (GEAR_P|GEAR_R|GEAR_N|GEAR_D); GPIOC->IM |= (GEAR_P|GEAR_R|GEAR_N|GEAR_D);

    NVIC_EnableIRQ(GPIOE_IRQn); NVIC_EnableIRQ(GPIOD_IRQn); NVIC_EnableIRQ(GPIOC_IRQn);
}

void GPIOE_Handler(void) { GPIOE->ICR = LIMIT_SWITCH; doorsLocked ^=1; doorEvent = true; }
void GPIOD_Handler(void) { GPIOD->ICR = IGNITION_BTN; ignitionOn ^=1; ignitionEvent = true; }
void GPIOC_Handler(void) {
    uint8_t gears = GPIOC->DATA & (GEAR_P|GEAR_R|GEAR_N|GEAR_D);
    int count = !!(gears&GEAR_P)+!!(gears&GEAR_R)+!!(gears&GEAR_N)+!!(gears&GEAR_D);
    if(count>1) strcpy(gearState,"N");
    else if(gears&GEAR_P) strcpy(gearState,"P");
    else if(gears&GEAR_R) strcpy(gearState,"R");
    else if(gears&GEAR_N) strcpy(gearState,"N");
    else if(gears&GEAR_D) strcpy(gearState,"D");
    gearEvent = true;
    GPIOC->ICR = GEAR_P|GEAR_R|GEAR_N|GEAR_D;
}

// --------------------- FreeRTOS Tasks ---------------------
void vUltrasonicTask(void *p) { while(1){ if(ignitionOn) distanceValue=MeasureDistance(); vTaskDelay(pdMS_TO_TICKS(100)); } }

void vSpeedTask(void *p) {
    while(1){
        if(ignitionOn){
            uint32_t a=ReadADC(); speedValue=(a*100)/4095;
            if(speedValue>20&&!doorsLocked){doorsLocked=true;doorEvent=true;}
        } else {
            if (doorsLocked) { doorsLocked = false; doorEvent = true; }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void vBuzzerTask(void *p) {
    while(1){
        if(ignitionOn){
            if(distanceValue<30){ BeepBuzzer(100,100); GPIOF->DATA = (GPIOF->DATA & ~(RGB_GREEN | RGB_BLUE)) | RGB_RED; }
            else if(distanceValue<100){ BeepBuzzer(500,500); GPIOF->DATA = (GPIOF->DATA & ~(RGB_RED | RGB_BLUE)) | RGB_GREEN; }
            else { GPIOF->DATA = (GPIOF->DATA & ~(RGB_RED | RGB_GREEN)) | RGB_BLUE; GPIOB->DATA&=~BUZZER_PIN; vTaskDelay(pdMS_TO_TICKS(500)); }
        } else {
            GPIOF->DATA &= ~(RGB_RED | RGB_GREEN | RGB_BLUE);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void vDisplayTask(void *p) {
    char buf[32];
    while(1){
        if(doorEvent){ LCD_Command(0x80); LCD_Print(doorsLocked?"Doors Locked   ":"Doors Unlocked "); BeepBuzzer(200,0); vTaskDelay(pdMS_TO_TICKS(2000)); doorEvent=false;}
        else if(ignitionEvent){ LCD_Command(0x80); LCD_Print(ignitionOn?"Car Started    ":"Car Off        "); BeepBuzzer(200,0); vTaskDelay(pdMS_TO_TICKS(2000)); ignitionEvent=false;}
        else if(gearEvent){ LCD_Command(0x80); snprintf(buf,sizeof(buf),"Gear: %s        ",gearState); LCD_Print(buf); vTaskDelay(pdMS_TO_TICKS(2000)); gearEvent=false;}
        else if(ignitionOn){ LCD_Command(0x80); snprintf(buf,sizeof(buf),"Dist:%3d cm     ",distanceValue); LCD_Print(buf); LCD_Command(0xC0); snprintf(buf,sizeof(buf),"Speed:%2d km/h  ",speedValue); LCD_Print(buf);}
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// --------------------- Main ---------------------
int main(void) {
    LCD_Init(); InitADC(); InitUltrasonic(); InitBuzzer(); InitInputs(); InitRGB();
    xTaskCreate(vUltrasonicTask,"Ultra",256,NULL,3,NULL);
    xTaskCreate(vSpeedTask,"Speed",256,NULL,3,NULL);
    xTaskCreate(vBuzzerTask,"Buzzer",256,NULL,2,NULL);
    xTaskCreate(vDisplayTask,"Display",256,NULL,1,NULL);
    vTaskStartScheduler();
    while(1);
}
