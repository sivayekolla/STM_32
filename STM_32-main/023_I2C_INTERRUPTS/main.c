#include "stm32l476xx.h"
#include "i2c_master.h"
#include "button.h"


// Example: pressing button will send RGB bytes [255,0,0] (red) to ESP32


void SystemClock_Config(void) {
// Minimal: assume device default clock set by bootloader or startup code.
// If you need an explicit clock config (HSI/MSI/HSE), add it here.
}


int main(void) {
SystemClock_Config();


// init
I2C1_MasterInit();
Button_Init();


// sample payload
uint8_t payload[3] = {255, 0, 0};


while (1) {
// Button ISR will initiate transfer — main can sleep or blink an LED
__WFI(); // wait for interrupt to reduce CPU usage
}
}


// Button ISR calls this function (we provide weak symbol in button.c style below)
void OnUserButtonPressed(void) {
// send data: simple example — cycle colors each press could be implemented
static uint8_t color_idx = 0;
uint8_t payloads[][3] = {{255,0,0},{0,255,0},{0,0,255}};
uint8_t *p = payloads[color_idx % 3];
color_idx++;
I2C1_MasterSendBytes(p, 3);
}