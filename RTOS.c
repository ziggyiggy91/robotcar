#include <stdint.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "Registers.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "ADC.h"
#include "PWM.h"
#include "LCD.h"
#include "GPIO.h"
#include "UART.h"
#include "PLL.h"
#include "I2C.h"
#include "projdefs.h"

typedef enum{
	LADC,
	RADC,
	POT,
	FADC
}DataSource_t;

typedef struct{
	uint32_t value;
	DataSource_t source;
}Data_t;

static Data_t d[4];//Struct array of 4 

uint32_t adc_data[4];
uint32_t adc_Threshold = 1700; //Value for ~20cm

QueueHandle_t xQueue,xLADCQueue,xRADCQueue,xFADCQueue;
BaseType_t pdPass;
xSemaphoreHandle ledSemaphore;


void
vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}
 
/****************************************************************************
*
* This task retrieves ADC data from PE2 for Front IR Sensor
*
* @param	pvParameter is a pointer for task descriptor
*
* @return	void
*
* @note		None.
*
*****************************************************************************/

void AdcTask3(void *pvParameter){
	BaseType_t xStatus;
	portTickType ui16LastTime;
	uint32_t ui32SwitchDelay = 25;
	static Data_t adc;
	uint32_t adc_data[4];
	adc.source = FADC;
	ui16LastTime = xTaskGetTickCount();
	while(1){
 		xSemaphoreTake(ledSemaphore, 20);
  	ADC_Conversion(adc_data);
		adc.value = adc_data[3];	
			UART_OutString("FADC: ");
			UART_OutUDec(adc.value);
			OutCRLF();
 		xStatus = xQueueSendToBack(xFADCQueue,&adc,portMAX_DELAY);
 		if(xStatus != pdPass){
			UART_OutString("Could not queue values from FADC\n\r");
		}
		GPIO_PORTF_DATA_R = 0x00;
		xSemaphoreGive(ledSemaphore);
		vTaskDelayUntil(&ui16LastTime, ui32SwitchDelay / portTICK_RATE_MS);
	}
}

/****************************************************************************
*
* This task retrieves ADC data from PE3 for potentiometer
*
* @param	pvParameter is a pointer for task descriptor
*
* @return	void
*
* @note		None.
*
*****************************************************************************/

void AdcTask2(void *pvParameter){
	BaseType_t xStatus;
	portTickType ui16LastTime;
	uint32_t ui32SwitchDelay = 25;
	static Data_t adc;
	uint32_t adc_data[4];
	adc.source = POT;
	ui16LastTime = xTaskGetTickCount();
	while(1){
		xSemaphoreTake(ledSemaphore, 100);
		GPIO_PORTF_DATA_R = 0x02;
 		ADC_Conversion(adc_data);
		adc.value = adc_data[2];	
		UART_OutString("POT: ");
			UART_OutUDec(adc.value);
			OutCRLF();
 		xStatus = xQueueSendToBack(xQueue,&adc,portMAX_DELAY); 
		//Change priority
		if(xStatus != pdPass){
			UART_OutString("Could not queue values from Pot\n\r");
		}
				GPIO_PORTF_DATA_R = 0x00;
				xSemaphoreGive(ledSemaphore);
		
		vTaskDelayUntil(&ui16LastTime, ui32SwitchDelay / portTICK_RATE_MS);
	}
}

/****************************************************************************
*
* This task retrieves ADC data from PE4 for right IR sensor
*
* @param	pvParameter is a pointer for task descriptor
*
* @return	void
*
* @note		None.
*
*****************************************************************************/

void AdcTask1(void *pvParameter){
	BaseType_t xStatus;
	portTickType ui16LastTime;
	uint32_t ui32SwitchDelay = 25;
	static Data_t adc;
	uint32_t adc_data[4];
	adc.source = RADC;
	ui16LastTime = xTaskGetTickCount();
	while(1){
		xSemaphoreTake(ledSemaphore, 100);
		GPIO_PORTF_DATA_R = 0x04;
		ADC_Conversion(adc_data);
		adc.value = adc_data[1];		
		UART_OutString("RADC: ");
			UART_OutUDec(adc.value);
			OutCRLF();
 		xStatus = xQueueSendToBack(xRADCQueue,&adc,portMAX_DELAY);
 	if(xStatus != pdPass){
			UART_OutString("Could not queue values from RADC\n\r");
		}
		GPIO_PORTF_DATA_R = 0x00;
		xSemaphoreGive(ledSemaphore);
		
		vTaskDelayUntil(&ui16LastTime, ui32SwitchDelay / portTICK_RATE_MS);
	}
}

/****************************************************************************
*
* This task retrieves ADC data from PE5 for left IR sensor
*
* @param	pvParameter is a pointer for task descriptor
*
* @return	void
*
* @note		None.
*
*****************************************************************************/

void AdcTask0(void *pvParameter){
	BaseType_t xStatus;
	portTickType ui16LastTime;
	uint32_t ui32SwitchDelay = 25;
	static Data_t adc;
   adc.source = LADC;	 
	ui16LastTime = xTaskGetTickCount();
	while(1){
 		xSemaphoreTake(ledSemaphore, 50);
		GPIO_PORTF_DATA_R = 0x08;
		ADC_Conversion(adc_data);
		adc.value = adc_data[0];		
		UART_OutString("LADC: ");
			UART_OutUDec(adc.value);
			OutCRLF();
 		xStatus = xQueueSendToBack(xLADCQueue,&adc,portMAX_DELAY); 
		//Change priority
		if(xStatus != pdPass){
			UART_OutString("Could not queue values from LADC\n\r");
		}
		GPIO_PORTF_DATA_R = 0x00;
		xSemaphoreGive(ledSemaphore);
		
		vTaskDelayUntil(&ui16LastTime, ui32SwitchDelay / portTICK_RATE_MS);
	}
}

/****************************************************************************
*
* This task handles DC Motors based on sensor data.
*
* @param	pvParameter is a pointer for task descriptor
*
* @return	void
*
* @note		None.
*
*****************************************************************************/

void MotorTask(void *pvParameter){	
	int32_t leftSensor,rightSensor,frontSensor,potValue,dacData;
	uint32_t ui32SwitchDelay = 25;
  uint32_t xLCD = 20,yLCD = 16,lcdString = 5;
	Data_t dataReceived,dataReceived1,dataReceived2,dataReceived3;
	BaseType_t xStatus;	
	portTickType ui32WakeTime;
 	
  ui32WakeTime = xTaskGetTickCount();
	while(1){
		xSemaphoreTake(ledSemaphore, 100);
  		if(xQueueReceive(xQueue, &dataReceived, 0) == pdPASS){};
  		if(xQueueReceive(xLADCQueue, &dataReceived1, 0) == pdPASS){};
			if(xQueueReceive(xRADCQueue, &dataReceived2, 0) == pdPASS){};
  		if(xQueueReceive(xFADCQueue, &dataReceived3, 0) == pdPASS){};
 
		 
 			leftSensor  = dataReceived1.value;
			//d[0].value = leftSensor;
			//d[0].source = LADC;
			UART_OutString("LADC: ");
 			UART_OutUDec(leftSensor);
			OutCRLF();
			 
 			rightSensor = dataReceived2.value;
			//d[1].value = rightSensor;
			//d[1].source = RADC;
						UART_OutString("RADC: ");
			UART_OutUDec(rightSensor);
			OutCRLF();
			 
 			potValue    = dataReceived.value;
			//d[2].value = potValue;
			//d[2].source = POT;
						UART_OutString("POT: ");

			UART_OutUDec(potValue);
			 			OutCRLF();

 			frontSensor = dataReceived3.value;
			//d[3].value = frontSensor;
			//d[3].source = FADC;
			UART_OutString("FADC: ");
			UART_OutUDec(frontSensor);
			OutCRLF();
			 
		
		//dacData = (frontSensor | leftSensor | rightSensor)*2.4 ;
		//Address 1100_000
		//I2C_Send2(0xC0, (dacData&(0xF00)), (dacData&0x0FF));
		
		/*
		Wait until all three values from the queue have been received, then proceed.
		*/
		if(potValue < 500){
 			PWM0A_Duty(19999);
			PWM0B_Duty(19999);
			GPIO_PORTF_DATA_R = 0x02; //Display red led when switch_count = 0. 
			}                                    
			/*
			Enter when left sensor value is higher than threshold, and reads 
			object at either 20cm or 30cm.It'll make a right turn by turn left motor, 
			and keeping right motor on. 
			*/
		else if((leftSensor > adc_Threshold) && (potValue >= 500)){
			//GPIO_PORTA_DATA_R = 0x0E;//PA3 and PA2 set high for forward direction
			PWM0A_Duty(19999);
			PWM0B_Duty(19999-potValue);
			ST7735_DrawString(xLCD/2, yLCD/2, "<-", ST7735_GREEN,lcdString);;

			GPIO_PORTF_DATA_R = 0x08;
		}
			/*
			Enter when right sensor value is higher threshold, and reads 
		  object at either 20cm or 30cm.It'll make a right turn by turn left motor,
   		and keeping right motor on.*/ 
		else if((rightSensor > adc_Threshold) && (potValue >= 500)){
			//GPIO_PORTA_DATA_R = 0x0C;//PA3 and PA2 set high for forward direction
  		PWM0A_Duty(19999-potValue);
			PWM0B_Duty(19999);
			ST7735_DrawString(xLCD/2, yLCD/2, "->", ST7735_GREEN,lcdString);
						ST7735_DrawString(xLCD/2, yLCD/2, "->", ST7735_BLACK,lcdString);;

			GPIO_PORTF_DATA_R = 0x04;
		}
		else if((frontSensor > adc_Threshold) && (leftSensor < adc_Threshold) && (rightSensor < adc_Threshold)){
			//GPIO_PORTA_DATA_R = 0x08;//PA3 and PA2 complementary of each other for 180 turn.
			PWM0A_Duty(19999-potValue);
			PWM0B_Duty(19999-potValue);
			ST7735_DrawString(xLCD/2, yLCD/2, "|", ST7735_GREEN,lcdString);
			ST7735_DrawString(xLCD/2, (yLCD/2)+1, "v", ST7735_GREEN,lcdString);
			ST7735_DrawString(xLCD/2, yLCD/2, "|", ST7735_BLACK,lcdString);
			ST7735_DrawString(xLCD/2, (yLCD/2)+1, "v", ST7735_BLACK,lcdString);

		}
		else{
			/*
			Enter when neither left,right, and front sensor detect a wall. 
			*/
			PWM0A_Duty(19999-potValue);
			PWM0B_Duty(19999-potValue);
			GPIO_PORTF_DATA_R = 0x00;
		 }
 		 xSemaphoreGive(ledSemaphore);
		 vTaskDelayUntil(&ui32WakeTime, ui32SwitchDelay / portTICK_RATE_MS);


	  }
}

/****************************************************************************
*
* This task displays sensor data.
*
* @param	pvParameter is a pointer for task descriptor
*
* @return	void
*
* @note		None.
*
*****************************************************************************/

void LcdTask(void *pvParameter){
			BaseType_t xStatus;
			portTickType ui32WakeTime;
	    uint32_t ui32SwitchDelay = 25;
 	
      ui32WakeTime = xTaskGetTickCount();
 	while(1){
			xSemaphoreTake(ledSemaphore, 100);
 		  UART_OutString("-------------------------- \n\r");		
			UART_OutString("LADC DATA: \n\r");		
			UART_OutUDec(d[0].value);
			OutCRLF();		
 			UART_OutString("RADC DATA: \n\r");	
			UART_OutUDec(d[1].value);	
			OutCRLF();			
 			UART_OutString("POT DATA: \n\r");	
			UART_OutUDec(d[2].value);		
			OutCRLF();			
		 	UART_OutString("FADC DATA: \n\r");	
			UART_OutUDec(d[3].value);		
			OutCRLF();
		  UART_OutString("-------------------------- \n\r");		
 		  xSemaphoreGive(ledSemaphore);		
		  vTaskDelayUntil(&ui32WakeTime, ui32SwitchDelay / portTICK_RATE_MS);
	}
}

int main(){
	  uint32_t xLCD = 20,yLCD = 16,lcdString = 5;

	//initialize gpio ports,etc..
	PLL_Init();
	ADC_Init();
	//portA_init();
	portF_init();
	UART0_Init();
	ST7735_InitR(INITR_REDTAB);
	ST7735_FillScreen(ST7735_BLACK);
	ST7735_SetCursor(0,0);
	I2C_Init();//DAC MCP4725
	PWM0A_Init(19999,19999);
	PWM0B_Init(19999,19999);
	ST7735_DrawString(xLCD/2, yLCD/2, "->", ST7735_GREEN,lcdString);;
	ST7735_DrawString(xLCD/2, yLCD/2, "->", ST7735_BLACK,lcdString);;

	UART_OutString("Real Time OS System \n\r");
 	/*
	xQueue created for six items of 32 bit data.
	three queues for separate adc conversions
	*/
	xQueue = xQueueCreate(6,sizeof(Data_t));
	xLADCQueue = xQueueCreate(6,sizeof(Data_t));
	xRADCQueue = xQueueCreate(6,sizeof(Data_t));
	xFADCQueue = xQueueCreate(6,sizeof(Data_t));

	ledSemaphore = xSemaphoreCreateMutex();
	
	//xTaskCreate(LcdTask,  "LCD DATA"     ,1000,NULL,1,NULL);
	xTaskCreate(AdcTask0,  "LADC DATA"   ,1000,NULL,1,NULL);
	xTaskCreate(AdcTask1,  "RADC DATA"   ,1000,NULL,2,NULL);
	xTaskCreate(AdcTask2,  "POT DATA"    ,1000,NULL,5,NULL);
	xTaskCreate(AdcTask3,  "FADC DATA"   ,1000,NULL,3,NULL);
	xTaskCreate(MotorTask,"Motor Control",1000,NULL,4,NULL);
	/*
	Start the scheduler for the created tasks
	*/
	vTaskStartScheduler();
	
	while(1){};
}
