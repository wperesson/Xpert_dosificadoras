
//https://shiroku.net/robotics/simple-finite-state-machine-for-hmi-of-adas-using-freertos/

/* Standard includes. */
#include <stdio.h>
#include <conio.h>
#include <stdbool.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Priorities at which the tasks are created. */
#define tskReadButton_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define tskShowState_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define tskController_PRIORITY		( tskIDLE_PRIORITY + 3 )

/* The rate at which show state is sent from ShowState to Controller.
 The times are converted from milliseconds to ticks using the pdMS_TO_TICKS() macro. */
#define ShowState_FREQUENCY_MS	pdMS_TO_TICKS( 500UL )

/* The rate at which button input is checked by ReadButton.
 The times are converted from milliseconds to ticks using the pdMS_TO_TICKS() macro. */
#define ReadButton_FREQUENCY_MS	pdMS_TO_TICKS( 100UL )

/* The number of items the queue can hold at once. */
#define QUEUE_LENGTH	( 2 )

/* The values sent to the queueController. */
#define BUTTON_NONE			( -1L )
#define BUTTON_LKS			( 1UL )
#define BUTTON_SET			( 2UL )
#define BUTTON_CANCEL		( 3UL )
#define LANE_DETECT_0		( 4UL )
#define LANE_DETECT_1		( 5UL )

/*-----------------------------------------------------------*/

/* The tasks as described in the Clock.pdf file. */
static void tskReadButton(void *pvParameters);
static void tskShowState(void *pvParameters);
static void tskController(void *pvParameters);

/*-----------------------------------------------------------*/

/* The queue used by ReadButton, ShowState, and Controller tasks. */
static QueueHandle_t queueController = NULL;

/* interrupt number and handler*/
static unsigned long showStateInterruptNumber = 3;
static unsigned long showStateInterruptHandler(void) {
	uint32_t ulQueuedValue;
	ulQueuedValue = "interrupt";
	xQueueSend(queueController, &ulQueuedValue, 0U);
	return pdTRUE;
}
/*-----------------------------------------------------------*/

void main_blinky(void) {
	/* Create the queues. */
	queueController = xQueueCreate(QUEUE_LENGTH, sizeof(uint32_t));

	if (queueController != NULL) {
		/* Create the tasks. */
		xTaskCreate(tskReadButton, /* The function that implements the task. */
		"ReadButton", /* The text name assigned to the task - for debug only as it is not used by the kernel. */
		configMINIMAL_STACK_SIZE, /* The size of the stack to allocate to the task. */
		NULL, /* The parameter passed to the task - not used in this simple case. */
		tskReadButton_PRIORITY, /* The priority assigned to the task. */
		NULL); /* The task handle is not required, so NULL is passed. */

		xTaskCreate(tskShowState, "ShowState", configMINIMAL_STACK_SIZE, NULL,
		tskShowState_PRIORITY, NULL);

		xTaskCreate(tskController, "Controller", configMINIMAL_STACK_SIZE, NULL,
		tskController_PRIORITY, NULL);

		/* Create time-tick interrupt handler */
		vPortSetInterruptHandler(showStateInterruptNumber,
				showStateInterruptHandler);

		/* Start the tasks running. */
		vTaskStartScheduler();
	}

	for (;;)
		;
}

/*-----------------------------------------------------------*/

static void tskReadButton(void *pvParameters) {
	TickType_t xNextWakeTime;
	const TickType_t xBlockTime = ReadButton_FREQUENCY_MS;
	uint32_t ulReceivedValue;
	uint32_t ulQueuedValue;

	/* Prevent the compiler warning about the unused parameter. */
	(void) pvParameters;

	/* Initialise xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();

	for (;;) {
		/* Wait until a key has been pressed. */
		if (_kbhit() != 0) {
			/* Remove the key from the input buffer. */
			ulReceivedValue = _getch();
			/* parse input. */
			ulQueuedValue = "none";
			switch (ulReceivedValue) {
			case (uint32_t) 'l':
				ulQueuedValue = BUTTON_LKS;
				break;
			case (uint32_t) 's':
				ulQueuedValue = BUTTON_SET;
				break;
			case (uint32_t) 'c':
				ulQueuedValue = BUTTON_CANCEL;
				break;
			case (uint32_t) '0':
				ulQueuedValue = LANE_DETECT_0;
				break;
			case (uint32_t) '1':
				ulQueuedValue = LANE_DETECT_1;
				break;
			}
			/* queue input. */
			xQueueSend(queueController, &ulQueuedValue, 0U);
		}

		vTaskDelayUntil(&xNextWakeTime, xBlockTime);
	}
}

/*-----------------------------------------------------------*/

static void tskShowState(void *pvParameters) {
	TickType_t xNextWakeTime;
	const TickType_t xBlockTime = ShowState_FREQUENCY_MS;

	/* Prevent the compiler warning about the unused parameter. */
	(void) pvParameters;

	/* Initialise xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();

	for (;;) {
		/* generate interrupt. */
		vPortGenerateSimulatedInterrupt(showStateInterruptNumber);

		/* Place this task in the blocked state until it is time to run again.
		 While in the Blocked state this task will not consume any CPU time. */
		vTaskDelayUntil(&xNextWakeTime, xBlockTime);
	}
}

/*-----------------------------------------------------------*/

struct main_state;
typedef void main_state_fn(struct main_state*);
struct main_state {
	main_state_fn *next;
	uint32_t button;
	bool led_lks;
	bool led_lane;
	bool led_steering;
	bool servo;
};

main_state_fn start, lks_mode_off, lks_mode_on, detect_lane, wait_detect,
		follow_lane;

void start(struct main_state *state) {
	state->next = lks_mode_off;
}

void lks_mode_off(struct main_state *state) {
	if (state->button == BUTTON_LKS) {
		state->led_lks = true;
		state->next = lks_mode_on;
	} else {
		state->next = lks_mode_off;
	}
}

void lks_mode_on(struct main_state *state) {
	if (state->button == BUTTON_LKS) {
		state->led_lks = false;
		state->next = lks_mode_off;
	} else if (state->button == LANE_DETECT_1) {
		state->led_lane = true;
		state->next = detect_lane;
	} else {
		state->next = lks_mode_on;
	}
}

void detect_lane(struct main_state *state) {
	if (state->button == BUTTON_LKS) {
		state->led_lks = false;
		state->led_lane = false;
		state->next = lks_mode_off;
	} else if (state->button == LANE_DETECT_0) {
		state->led_lane = false;
		state->next = wait_detect;
	} else if (state->button == BUTTON_SET) {
		state->led_steering = true;
		state->servo = true;
		state->next = follow_lane;
	} else {
		state->next = detect_lane;
	}
}

void wait_detect(struct main_state *state) {
	if (state->button == BUTTON_LKS) {
		state->led_lks = false;
		state->next = lks_mode_off;
	} else if (state->button == LANE_DETECT_1) {
		state->led_lane = true;
		state->next = detect_lane;
	} else {
		state->next = wait_detect;
	}
}

void follow_lane(struct main_state *state) {
	if (state->button == BUTTON_LKS) {
		state->led_lks = false;
		state->led_lane = false;
		state->led_steering = false;
		state->servo = false;
		state->next = lks_mode_off;
	} else if (state->button == BUTTON_CANCEL) {
		state->led_steering = false;
		state->servo = false;
		state->next = detect_lane;
	} else if (state->button == LANE_DETECT_0) {
		state->led_lane = false;
		state->led_steering = false;
		state->servo = false;
		state->next = wait_detect;
	} else {
		state->next = follow_lane;
	}
}

/*-----------------------------------------------------------*/

static void updateLCD(uint32_t ulReceivedValue,
bool led_lks,
bool led_lane,
bool led_steering,
bool servo) {
	printf("led_lks:%d, ", led_lks);
	printf("led_lane:%d, ", led_lane);
	printf("led_steering:%d, ", led_steering);
	printf("servo:%d", servo);

	if (ulReceivedValue == BUTTON_LKS)
		printf(", BUTTON_LKS");
	else if (ulReceivedValue == BUTTON_SET)
		printf(", BUTTON_SET");
	else if (ulReceivedValue == BUTTON_CANCEL)
		printf(", BUTTON_CANCEL");
	else if (ulReceivedValue == LANE_DETECT_0)
		printf(", LANE_DETECT_0");
	else if (ulReceivedValue == LANE_DETECT_1)
		printf(", LANE_DETECT_1");

	printf("\r\n");
}

/*-----------------------------------------------------------*/

static void tskController(void *pvParameters) {
	uint32_t ulReceivedValue;

	/* init FSM */
	struct main_state main_state = { start, "", false, false, false, false };

	for (;;) {
		/* Wait until something arrives in the queue - this task will block indefinitely.
		 It will not use any CPU time while it is in the Blocked state. */
		xQueueReceive(queueController, &ulReceivedValue, portMAX_DELAY);

		/* update FSM */
		main_state.button = ulReceivedValue;
		main_state.next(&main_state);

		/* update LCD */
		updateLCD(ulReceivedValue, main_state.led_lks, main_state.led_lane,
				main_state.led_steering, main_state.servo);
	}
}

/*-----------------------------------------------------------*/
