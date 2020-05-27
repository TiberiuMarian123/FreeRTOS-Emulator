#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <inttypes.h>

#include <SDL2/SDL_scancode.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

#include "TUM_Ball.h"
#include "TUM_Draw.h"
#include "TUM_Event.h"
#include "TUM_Sound.h"
#include "TUM_Utils.h"
#include "TUM_Font.h"

#include "AsyncIO.h"

#define mainGENERIC_PRIORITY (tskIDLE_PRIORITY)
#define mainGENERIC_STACK_SIZE ((unsigned short)2560)

#define STATE_QUEUE_LENGTH 1
#define STATE_COUNT 3

#define STATE_ONE 0
#define STATE_TWO 1
#define STATE_THREE 2

#define NEXT_TASK 0
#define PREV_TASK 1

#define STARTING_STATE STATE_ONE
#define STATE_DEBOUNCE_DELAY 300

#define KEYCODE(CHAR) SDL_SCANCODE_##CHAR

#define FPS_AVERAGE_COUNT 50
#define FPS_FONT "IBMPlexSans-Bold.ttf"

static QueueHandle_t StateQueue = NULL;

static StackType_t xStack[mainGENERIC_STACK_SIZE * 6];
static StaticTask_t xTaskBuffer;

static TaskHandle_t DemoTask1 = NULL;
static TaskHandle_t DemoTask2 = NULL;
static TaskHandle_t DemoTask3 = NULL;
static TaskHandle_t StateMachine = NULL;
static TaskHandle_t BufferSwap = NULL;

TimerHandle_t xTimer = NULL; // create timer

const unsigned char next_state_signal = NEXT_TASK;
const unsigned char prev_state_signal = PREV_TASK;

static SemaphoreHandle_t ScreenLock = NULL;
static SemaphoreHandle_t DrawSignal = NULL;

// define a handle named buttons_buffer_t. It contains the SCANCODE and Semaphore Lock  
typedef struct buttons_buffer {
    unsigned char buttons[SDL_NUM_SCANCODES];
    SemaphoreHandle_t lock;
} buttons_buffer_t;

static buttons_buffer_t buttons = { 0 };

void changeState(volatile unsigned char *state, unsigned char forwards)
{
    switch (forwards) {
        case NEXT_TASK:
            if (*state == STATE_COUNT - 1) {
                *state = 0;
            }
            else {
                (*state)++;
            }
            break;
        case PREV_TASK:
            if (*state == 0) {
                *state = STATE_COUNT - 1;
            }
            else {
                (*state)--;
            }
            break;
        default:
            break;
    }
}

void basicSequentialStateMachine(void *pvParameters)
{
    unsigned char current_state = STARTING_STATE; // Default state
    unsigned char state_changed = 1; // Only re-evaluate state if it has changed
    unsigned char input = 0;

    const int state_change_period = STATE_DEBOUNCE_DELAY;

    TickType_t last_change = xTaskGetTickCount();

    while (1) {

        if (state_changed) {
            goto initial_state;
        }

        // Handle state machine input
        if (StateQueue)
            if (xQueueReceive(StateQueue, &input, portMAX_DELAY) == pdTRUE)
                if (xTaskGetTickCount() - last_change > state_change_period) {
                    changeState(&current_state, input);
                    state_changed = 1;
                    last_change = xTaskGetTickCount();
                }

initial_state:
        // Handle current state
        if (state_changed) {
            switch (current_state) {
                case STATE_ONE:  // current_state = STARTING_STATE = 0
                    if (DemoTask2) {
                        vTaskSuspend(DemoTask2);
                    }
                    if (DemoTask3) {
                        vTaskSuspend(DemoTask3);
                    }
                    if (DemoTask1) {
                        vTaskResume(DemoTask1);
                    }
                    break;
                case STATE_TWO: // current_state = 1
                    if (DemoTask1) {
                        vTaskSuspend(DemoTask1);
                    }
                    if (DemoTask3) {
                        vTaskSuspend(DemoTask3);
                    }
                    if (DemoTask2) {
                        vTaskResume(DemoTask2);
                    }
                    break;
                case STATE_THREE:
                    if (DemoTask1) {
                        vTaskSuspend(DemoTask1);
                    }
                    if (DemoTask2) {
                        vTaskSuspend(DemoTask2);
                    }
                    if (DemoTask3) {
                        vTaskResume(DemoTask3);
                    }
                    break;   

                default:
                    break;
            }
            state_changed = 0;
        }
    }
}

void vSwapBuffers(void *pvParameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t frameratePeriod = 20;

    tumDrawBindThread(); // Setup Rendering handle with correct GL context

    while (1) {
        if (xSemaphoreTake(ScreenLock, portMAX_DELAY) == pdTRUE) {
            tumDrawUpdateScreen();
            tumEventFetchEvents();
            xSemaphoreGive(ScreenLock);
            xSemaphoreGive(DrawSignal);
            vTaskDelayUntil(&xLastWakeTime,
                            pdMS_TO_TICKS(frameratePeriod));
        }
    }
}

void xGetButtonInput(void)
{   // if the semaphore is not used (lock == 0), then take it and use it to input button.
    if (xSemaphoreTake(buttons.lock, 0) == pdTRUE) {
        xQueueReceive(buttonInputQueue, &buttons.buttons, 0);
        xSemaphoreGive(buttons.lock);
    }
}

void vDrawHelpText(void)
{
    static char str[100] = { 0 };
    static int text_width;
    ssize_t prev_font_size = tumFontGetCurFontSize();

    tumFontSetSize((ssize_t)20);

    sprintf(str, "[Q]uit, [E]=New_State");

    if (!tumGetTextSize((char *)str, &text_width, NULL))
        tumDrawText(str, SCREEN_WIDTH - text_width - 10,
                    DEFAULT_FONT_SIZE * 0.5, Black);
                  
    tumFontSetSize(prev_font_size);
}

static int vCheckStateInput(void)
{
    if (xSemaphoreTake(buttons.lock, 0) == pdTRUE) {
        if (buttons.buttons[KEYCODE(E)]) {
            buttons.buttons[KEYCODE(E)] = 0;
            if (StateQueue) {
                xSemaphoreGive(buttons.lock);
                xQueueSend(StateQueue, &next_state_signal, 0);
                return -1;
            }
        }
        xSemaphoreGive(buttons.lock);
    }

    return 0;
}

// just a more intuitive format 
#define KEYCODE(CHAR) SDL_SCANCODE_##CHAR

void vDrawTriangle(signed short x_mouse, signed short y_mouse){

    static coord_t triangle_points[3];
    signed short Xtri = 250;
    signed short Ytri = SCREEN_HEIGHT-200;
    signed short triangle_width = 100;
    signed short triangle_height = 100;

    triangle_points[0].x = Xtri + x_mouse;
    triangle_points[0].y = Ytri + y_mouse;
    triangle_points[1].x = Xtri + triangle_width / 2 + x_mouse;
    triangle_points[1].y = Ytri - triangle_height + y_mouse;
    triangle_points[2].x = Xtri + triangle_width + x_mouse;
    triangle_points[2].y = Ytri + y_mouse;

    tumDrawTriangle(triangle_points, TUMBlue);
}

signed short Xcenter = SCREEN_WIDTH/5;
signed short Ycenter = SCREEN_HEIGHT/2 - 10;
signed short radius = 35;

void vDrawCircle(signed short x_mouse, signed short y_mouse,
                    signed short Xcenter, signed short Ycenter, unsigned int color){

    tumDrawCircle(Xcenter + x_mouse, Ycenter + y_mouse, radius, color);
}

signed short sqare_size = 70;
signed short Xsquare = 4 * SCREEN_WIDTH/6;
signed short Ysquare = SCREEN_HEIGHT/2 - 60;

void vDrawSquare(signed short x_mouse, signed short y_mouse){

    tumDrawFilledBox(Xsquare + x_mouse, Ysquare + y_mouse, sqare_size, sqare_size, Green);
}

float angle = 0;
signed short route_radius = 140;

// By one iteration, it modifies the position of the square and circle, forming in while-loop a circular route
void UpdatePositionFigure(signed short x_mouse, signed short y_mouse){

    Xsquare = SCREEN_WIDTH/2 + route_radius * cos(angle) - 35 + x_mouse;
    Ysquare = SCREEN_HEIGHT/2 + route_radius * sin(angle) - 35 + y_mouse;
    Xcenter = SCREEN_WIDTH/2 - route_radius * cos(-angle) + x_mouse;
    Ycenter = SCREEN_HEIGHT/2 - route_radius * sin(angle) + y_mouse;
    angle = angle + 0.1;
}


// It works if you don't press the ONE button consecutively (like A, then A once more) 

void PressingCounter(int *count_A, int *count_B, int *count_C, int *count_D, struct timespec the_time,
                     unsigned long lastDebounceTime, unsigned long debounceDelay){

    static unsigned char lastbuttonState[SDL_NUM_SCANCODES]; // initially nothing is being pressed                      
    if (xSemaphoreTake(buttons.lock, 0) == pdTRUE) {

            if (buttons.buttons[KEYCODE(A)] && (the_time.tv_sec - lastDebounceTime > debounceDelay))
            {
                (*count_A)++; 
                memcpy(lastbuttonState,buttons.buttons,SDL_NUM_SCANCODES * sizeof(unsigned char));
                lastDebounceTime = the_time.tv_sec;
            }
            if (buttons.buttons[KEYCODE(B)] && ! lastbuttonState[KEYCODE(B)])
            {
                (*count_B)++;
                memcpy(lastbuttonState,buttons.buttons,SDL_NUM_SCANCODES * sizeof(unsigned char));
            }   
            if (buttons.buttons[KEYCODE(C)] && ! lastbuttonState[KEYCODE(C)])
            {
                memcpy(lastbuttonState,buttons.buttons,SDL_NUM_SCANCODES * sizeof(unsigned char));
                (*count_C)++;
            }
            if (buttons.buttons[KEYCODE(D)] && ! lastbuttonState[KEYCODE(D)])
            {
                memcpy(lastbuttonState,buttons.buttons,SDL_NUM_SCANCODES * sizeof(unsigned char));
                (*count_D)++;   
            }

            xSemaphoreGive(buttons.lock); 
        }
    
    if(tumEventGetMouseLeft()) // if left mouse button is PRESSED
        {                      // reset the counters
            *count_A = 0;    
            *count_B = 0;
            *count_C = 0;
            *count_D = 0;
        }  
    
}

void vDrawFPS(void)
{
    static unsigned int periods[FPS_AVERAGE_COUNT] = { 0 };
    static unsigned int periods_total = 0;
    static unsigned int index = 0;
    static unsigned int average_count = 0;
    static TickType_t xLastWakeTime = 0, prevWakeTime = 0;
    static char str[10] = { 0 };
    static int text_width;
    int fps = 0;
    font_handle_t cur_font = tumFontGetCurFontHandle();

    xLastWakeTime = xTaskGetTickCount();

    if (prevWakeTime != xLastWakeTime) {
        periods[index] =
            configTICK_RATE_HZ / (xLastWakeTime - prevWakeTime);
        prevWakeTime = xLastWakeTime;
    }
    else {
        periods[index] = 0;
    }

    periods_total += periods[index];

    if (index == (FPS_AVERAGE_COUNT - 1)) {
        index = 0;
    }
    else {
        index++;
    }

    if (average_count < FPS_AVERAGE_COUNT) {
        average_count++;
    }
    else {
        periods_total -= periods[index];
    }

    fps = periods_total / average_count;

    tumFontSelectFontFromName(FPS_FONT);

    sprintf(str, "FPS: %2d", fps);

    if (!tumGetTextSize((char *)str, &text_width, NULL))
        tumDrawText(str, SCREEN_WIDTH - text_width - 10,
                              SCREEN_HEIGHT - DEFAULT_FONT_SIZE * 1.5,
                              Black);

    tumFontSelectFontFromHandle(cur_font);
    tumFontPutFontHandle(cur_font);
}


void vDemoTask1(void *pvParameters)
{
    // structure to store time retrieved from Linux kernel
    static struct timespec the_time;

    static char our_time_string[100];
    static int our_time_strings_width = 0;
    static char random_string[100];
    static int random_strings_width = 0;
    static char mouse_string[100];
    static int mouse_strings_width = 0;
    static char press_string[100];
    static int press_strings_width = 0;

    static unsigned long lastDebounceTime = 0;
    static unsigned long debounceDelay = 1; // 1 s delay between buttons

    static int i = 0;
    static int counter = 0;

    static signed short x_mouse;
    static signed short y_mouse; 

    int count_A = 0;
    int count_B = 0;
    int count_C = 0;
    int count_D = 0;



    // Needed such that Gfx library knows which thread controlls drawing
    // Only one thread can call tumDrawUpdateScreen while and thread can call
    // the drawing functions to draw objects. This is a limitation of the SDL
    // backend.
    tumDrawBindThread();

    while (1) {

        tumEventFetchEvents(); // Query events backend for new events, ie. button presses
        xGetButtonInput(); // Update global input

        xSemaphoreTake(ScreenLock, portMAX_DELAY);

        // `buttons` is a global shared variable and as such needs to be
        // guarded with a mutex, mutex must be obtained before accessing the
        // resource and given back when you're finished. If the mutex is not
        // given back then no other task can access the reseource.
        if (xSemaphoreTake(buttons.lock, 0) == pdTRUE) {
            if (buttons.buttons[KEYCODE(Q)]) { // If button Q is pressed
                exit(EXIT_SUCCESS); // File buffers are flushed, streams are closed, and temporary files are deleted.
            }
            xSemaphoreGive(buttons.lock);
        }

        tumDrawClear(White); // Clear screen

        x_mouse = tumEventGetMouseX();
        y_mouse = tumEventGetMouseY();

        sprintf(mouse_string,
                "X_mouse : %d   Y_mouse : %d",
               x_mouse, y_mouse);

        if (!tumGetTextSize((char *)mouse_string,
                            &mouse_strings_width, NULL))
            tumDrawText(mouse_string,
                        SCREEN_WIDTH / 5 - mouse_strings_width / 2 + x_mouse/5,
                        SCREEN_HEIGHT * 1/13 - DEFAULT_FONT_SIZE / 2 + y_mouse/5,
                        TUMBlue);


        clock_gettime(CLOCK_REALTIME, &the_time); // Get kernel real time
        
        vDrawTriangle(x_mouse/5, y_mouse/5); 
        vDrawCircle(x_mouse/5, y_mouse/5, Xcenter, Ycenter, Red);
        vDrawSquare(x_mouse/5, y_mouse/5);
        UpdatePositionFigure(x_mouse/5, y_mouse/5);

        sprintf(our_time_string,
                "There has been %ld seconds since the Epoch. Press Q to quit",
               (long int)the_time.tv_sec);

        if (!tumGetTextSize((char *)our_time_string,
                            &our_time_strings_width, NULL))
            tumDrawText(our_time_string,
                        SCREEN_WIDTH /2  - our_time_strings_width / 2 + x_mouse/5,
                        SCREEN_HEIGHT * 14/15 - DEFAULT_FONT_SIZE / 2 + y_mouse/5,
                        TUMBlue); 	

        sprintf(random_string, "Insert text here");  

        if (!tumGetTextSize((char *)random_string,
                            &random_strings_width, NULL)){  

            tumDrawText(random_string,
                        SCREEN_WIDTH /2  - random_strings_width / 2 + i - 20 + x_mouse/5, // moving horizontally by 'i' pixels
                        SCREEN_HEIGHT * 1/15 - DEFAULT_FONT_SIZE / 2 + y_mouse/5,
                        Red);  	
            }

        // Moving text back and forth on 100 Pixels Horizontally 
        i++; counter++;

        if(counter > 100) 
            i-=2; // i = i-2 because up there i++ is still incrementing no matter what counter is
        if(i == 0) // if we are back where we started
            counter = 0;

        PressingCounter(&count_A, &count_B, &count_C, &count_D, the_time, lastDebounceTime, debounceDelay);
        sprintf(press_string,
                "A:  %d, B:  %d, C:  %d, D:  %d",
               count_A, count_B, count_C, count_D);

        if (!tumGetTextSize((char *)press_string,
                            &press_strings_width, NULL)){  
            tumDrawText(press_string,
                        SCREEN_WIDTH /5  - random_strings_width / 2 - 20 + x_mouse/5, // moving horizontally by 'i' pixels
                        SCREEN_HEIGHT * 1/9 - DEFAULT_FONT_SIZE / 2 + y_mouse/5,
                        Red);  	
            }


        vDrawHelpText();
        vDrawFPS();

        tumDrawUpdateScreen(); // Refresh the screen to draw string

        xSemaphoreGive(ScreenLock);

        vCheckStateInput();

        vTaskDelay((TickType_t)20);
    }
}


void vDemoTask2(){ // BLUE CIRCLE LEFT, 1Hz Circle frequency, 1Hz Task
tumDrawBindThread();
int counter = 0;

    while (1) {

        tumEventFetchEvents(); // Query events backend for new events, ie. button presses
        xGetButtonInput(); // Update global input

        xSemaphoreTake(ScreenLock, portMAX_DELAY);

        if (xSemaphoreTake(buttons.lock, 0) == pdTRUE) {
            if (buttons.buttons[KEYCODE(Q)])  // If button Q is pressed
                    exit(EXIT_SUCCESS);  // File buffers are flushed, streams are closed, and temporary files are deleted.
                    xSemaphoreGive(buttons.lock);
        }

        tumDrawClear(White); // Clear screen

        if(counter < 25) // Draws circle for the first 25 loops => 500 ms (vTaskDelay = 20ms) 
            vDrawCircle(0,0,250,240, TUMBlue);
        counter ++;
        if(counter == 50) //if the circle drew for 500 ms, then nothing for another 500 ms => 50 loops in total
            counter = 0;    // reset counter and begin drawing again

        tumDrawUpdateScreen(); // Refresh the screen to draw string

        xSemaphoreGive(ScreenLock);

        vCheckStateInput();

        vTaskDelay((TickType_t)20); 
    }

}

void vDemoTask3(){  // RED CIRCLE RIGHT , 2Hz circle frequency, 2Hz task
tumDrawBindThread();
int counter = 0;

    while (1) {

        tumEventFetchEvents(); // Query events backend for new events, ie. button presses
        xGetButtonInput(); // Update global input

        xSemaphoreTake(ScreenLock, portMAX_DELAY);

        if (xSemaphoreTake(buttons.lock, 0) == pdTRUE) {
            if (buttons.buttons[KEYCODE(Q)])  // If button Q is pressed
                    exit(EXIT_SUCCESS);  // File buffers are flushed, streams are closed, and temporary files are deleted.
                    xSemaphoreGive(buttons.lock);
        }

        tumDrawClear(White); // Clear screen
        
        if(counter < 12.5) // Draws circle for the first 25 loops => 500 ms (vTaskDelay = 20ms) 
            vDrawCircle(0,0,350,240, Red);
        counter ++;
        if(counter == 25) //if the circle drew for 500 ms, then nothing for another 500 ms => 50 loops in total
            counter = 0;    // reset counter and begin drawing again

        tumDrawUpdateScreen(); // Refresh the screen to draw string;

        xSemaphoreGive(ScreenLock);

        vCheckStateInput();

        vTaskDelay((TickType_t)20);
    }

}

void vTimerCallback(TimerHandle_t xTimer) {

	/* Optionally do something if the pxTimer parameter is NULL. */
	configASSERT(xTimer);
	
}

void vDemoTask4(){
tumDrawBindThread();

    while (1) {

        tumEventFetchEvents(); // Query events backend for new events, ie. button presses
        xGetButtonInput(); // Update global input

        //xTimerStart(xTimer, 0);

        xSemaphoreTake(ScreenLock, portMAX_DELAY);

        if (xSemaphoreTake(buttons.lock, 0) == pdTRUE) {
            if (buttons.buttons[KEYCODE(UP)])  // If ARROW_UP is pressed
                exit(EXIT_SUCCESS);  // File buffers are flushed, streams are closed, and temporary files are deleted.
            xSemaphoreGive(buttons.lock);
        }

    }

}

void vDemoTask5( ){
tumDrawBindThread();

    while (1) {

        tumEventFetchEvents(); // Query events backend for new events, ie. button presses
        xGetButtonInput(); // Update global input

        //xTimerStart(xTimer, 0);

        xSemaphoreTake(ScreenLock, portMAX_DELAY);

        if (xSemaphoreTake(buttons.lock, 0) == pdTRUE) {
            if (buttons.buttons[KEYCODE(DOWN)])  // If ARROW_DOWN is pressed
                exit(EXIT_SUCCESS);  // File buffers are flushed, streams are closed, and temporary files are deleted.
            xSemaphoreGive(buttons.lock);
        }

    }

}

#define PRINT_TASK_ERROR(task) PRINT_ERROR("Failed to print task ##task");
int main(int argc, char *argv[])
{
    char *bin_folder_path = tumUtilGetBinFolderPath(argv[0]);

    printf("Initializing: ");

    if (tumDrawInit(bin_folder_path)) {
        PRINT_ERROR("Failed to initialize drawing");
        goto err_init_drawing;
    }

    if (tumEventInit()) {
        PRINT_ERROR("Failed to initialize events");
        goto err_init_events;
    }

    if (tumSoundInit(bin_folder_path)) {
        PRINT_ERROR("Failed to initialize audio");
        goto err_init_audio;
    }

    atexit(aIODeinit);

    buttons.lock = xSemaphoreCreateMutex(); // Enable the locking mechanism
    if (!buttons.lock) {  // if lock is used by sth. else
        PRINT_ERROR("Failed to create buttons lock");
        goto err_buttons_lock;
    }

    ScreenLock = xSemaphoreCreateMutex();
    if (!ScreenLock) {
        PRINT_ERROR("Failed to create screen lock");
        goto err_screen_lock;
    }

    StateQueue = xQueueCreate(STATE_QUEUE_LENGTH, sizeof(unsigned char));
    if (!StateQueue) {
        PRINT_ERROR("Could not open state queue");
        goto err_state_queue;
    }

    if (xTaskCreate(basicSequentialStateMachine, "StateMachine",
                    mainGENERIC_STACK_SIZE * 2, NULL,
                    configMAX_PRIORITIES - 1, StateMachine) != pdPASS) {
        PRINT_TASK_ERROR("StateMachine");
        goto err_statemachine;
    }

    // Create Timer
    /*xTimer = xTimerCreate("ResetTimer", 15000 / portTICK_PERIOD_MS, pdTRUE,
			(void *) 0, vTimerCallback);
	configASSERT(xTimer != NULL); */

    if (xTaskCreate(vDemoTask1, "DemoTask1", mainGENERIC_STACK_SIZE * 2,
                    NULL, mainGENERIC_PRIORITY, &DemoTask1) != pdPASS) {
        PRINT_TASK_ERROR("DemoTask1");
        goto err_demotask1;
    }

    // Allocate Dynamically, priority -1
    // Set configSUPPORT_DYNAMIC_ALLOCATION to 1
    if (xTaskCreate(vDemoTask2, "DemoTask2", mainGENERIC_STACK_SIZE * 2,
                    NULL, mainGENERIC_PRIORITY - 1, &DemoTask2) != pdPASS) {
        PRINT_TASK_ERROR("DemoTask2");
        goto err_demotask2;
    }

    // Allocate Statically, priority -2              DOES NOT WORK PROPERLY (But it is very close !!!!!!!!)
    // Set configSUPPORT_STATIC_ALLOCATION to 1
    // xTaskCreateStatic(vDemoTask3, "DemoTask3", mainGENERIC_STACK_SIZE * 2,
    //                NULL, mainGENERIC_PRIORITY - 2, xStack, &xTaskBuffer);
    
    if (xTaskCreate(vDemoTask3, "DemoTask3", mainGENERIC_STACK_SIZE * 2,
                    NULL, mainGENERIC_PRIORITY - 2, &DemoTask3) != pdPASS) {
        PRINT_TASK_ERROR("DemoTask3");
        goto err_demotask3;
    }

    vTaskSuspend(DemoTask1);
    vTaskSuspend(DemoTask2); 
    vTaskSuspend(DemoTask3);

    vTaskStartScheduler();

    return EXIT_SUCCESS;

    err_statemachine:
        vQueueDelete(StateQueue);
    err_state_queue:
        vSemaphoreDelete(StateQueue);
    err_demotask1:
        vTaskDelete(BufferSwap);
    err_demotask2:
        vTaskDelete(DemoTask1);
    err_demotask3:
        vTaskDelete(DemoTask2);
    err_screen_lock:
        vSemaphoreDelete(DrawSignal);
    err_buttons_lock:
        tumSoundExit();
    err_init_audio:
        tumEventExit();
    err_init_events:
        tumDrawExit();
    err_init_drawing:
        return EXIT_FAILURE;

}

// cppcheck-suppress unusedFunction
__attribute__((unused)) void vMainQueueSendPassed(void)
{
    /* This is just an example implementation of the "queue send" trace hook. */
}

// cppcheck-suppress unusedFunction
__attribute__((unused)) void vApplicationIdleHook(void)
{
#ifdef __GCC_POSIX__
    struct timespec xTimeToSleep, xTimeSlept;
    /* Makes the process more agreeable when using the Posix simulator. */
    xTimeToSleep.tv_sec = 1;
    xTimeToSleep.tv_nsec = 0;
    nanosleep(&xTimeToSleep, &xTimeSlept);
#endif
}

// Missing data for Static Allocation because vTaskCreateStatic won't work

#define configMINIMAL_STACK_SIZE                128
#define configTIMER_TASK_STACK_DEPTH            configMINIMAL_STACK_SIZE
/* A header file that defines trace macro can be included here. */


/* configSUPPORT_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
/* If the buffers to be provided to the Idle task are declared inside this
function then they must be declared static – otherwise they will be allocated on
the stack and so not exists after this function exits. */
static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task’s
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task’s stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/*———————————————————–*/

/* configSUPPORT_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
application must provide an implementation of vApplicationGetTimerTaskMemory()
to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize )
{
/* If the buffers to be provided to the Timer task are declared inside this
function then they must be declared static – otherwise they will be allocated on
the stack and so not exists after this function exits. */
static StaticTask_t xTimerTaskTCB;
static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

    /* Pass out a pointer to the StaticTask_t structure in which the Timer
    task’s state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task’s stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configTIMER_TASK_STACK_DEPTH is specified in words, not bytes. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}