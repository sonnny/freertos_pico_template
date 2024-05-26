/*
 * 
freertos demo for rp2040
works for version 11.1.0
uses uart0 on gpio_0 and gpio_1
use a 3.3v usb to uart convert

if word received in uart is led on then turn on gpio 4 led
if word received in uart is led of then turn off gpio 4 led
uses only 6 character for comparison

not good use of global but enough for demo
uses global variable char array to store incoming uart character
uses global variable index to increment array

to use picoprobe:

sudo openocd
  -f interface/cmsis-dap.cfg
  -f target/rp2040.cfg
  -c "adapter speed 5000"
  -c "program blink.elf verify reset exit"

*/

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h" /* Must come first. */
#include "task.h"     /* RTOS task related API prototypes. */
#include "semphr.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

static char buffer[50];
static int bufferIndex = 0;
volatile int state = 0;

void blink_task(void *pvParameters){
  gpio_init(7); gpio_set_dir(7, 1);

  while(true){
      gpio_put(7, !gpio_get(7));
      vTaskDelay(500 / portTICK_PERIOD_MS);}}

static void on_cli_uart_rx(){
  if (uart_is_readable(uart0)) {
    buffer[bufferIndex++] = uart_getc(uart0);}}
    
void uart_task(void *pvParameters){
  gpio_init(4);  gpio_set_dir(4, 1);
  for(;;){
    if (strncmp(buffer, "led on", 6) == 0) {
      gpio_put(4,1);
      memset(buffer, 0x00, sizeof(buffer));
      bufferIndex = 0;}
  
    if (strncmp(buffer, "led of", 6) == 0) {
      gpio_put(4,0);
      memset(buffer, 0x00, sizeof(buffer));
      bufferIndex = 0;}
  
  vTaskDelay(100 / portTICK_PERIOD_MS); }}
  
int main() {
    stdio_init_all();
    gpio_set_function(0, GPIO_FUNC_UART);  // set gpio 0 to uart functions
    gpio_set_function(1, GPIO_FUNC_UART);  // set gpio 1 to uart functions
    uart_set_baudrate(uart0, 115200);      // set baudrate
    uart_set_hw_flow(uart0, false, false); // no hardware flow control
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart0, false);   // no fifo, processed each character
    irq_set_exclusive_handler(UART0_IRQ, on_cli_uart_rx); // set interrupt function
    irq_set_enabled(UART0_IRQ, true);      // enable interrupt
    uart_set_irq_enables(uart0, true, false); // receive interrupt
    
    memset(buffer, 0x00, sizeof(buffer));
    bufferIndex = 0;
    
    xTaskCreate(uart_task, "UART receive task", 1024 * 3, NULL, 1, NULL);
    xTaskCreate(blink_task, "Blink task", 1024 * 3, NULL, 1, NULL);
	vTaskStartScheduler();
	for(;;){}
	return 0;}


