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

#include "TUM_Ball.h"
#include "TUM_Draw.h"
#include "TUM_Event.h"
#include "TUM_Sound.h"
#include "TUM_Utils.h"
#include "TUM_Font.h"

#include "AsyncIO.h"

#define mainGENERIC_PRIORITY (tskIDLE_PRIORITY)
#define mainGENERIC_STACK_SIZE ((unsigned short)2560)

#define STATE_ONE 0
#define STATE_TWO 1

#define STARTING_STATE STATE_ONE
#define STATE_DEBOUNCE_DELAY 300

static QueueHandle_t StateQueue = NULL;

static TaskHandle_t DemoTask = NULL;

static TaskHandle_t StateMachine = NULL;

// define a handle named buttons_buffer_t. It contains the SCANCODE and Semaphore Lock  
typedef struct buttons_buffer {
    unsigned char buttons[SDL_NUM_SCANCODES];
    SemaphoreHandle_t lock;
} buttons_buffer_t;

static buttons_buffer_t buttons = { 0 };

void xGetButtonInput(void)
{   // if the semaphore is not used (lock == 0), then take it and use it to input button.
    if (xSemaphoreTake(buttons.lock, 0) == pdTRUE) {
        xQueueReceive(buttonInputQueue, &buttons.buttons, 0);
        xSemaphoreGive(buttons.lock);
    }
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

void vDrawCircle(signed short x_mouse, signed short y_mouse){

    tumDrawCircle(Xcenter + x_mouse, Ycenter + y_mouse, radius, Red);
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

// It does work WITHOUT DEBOUNCING. 
// I tried to store the previous status (button press) in lastbuttonState
// Then I compared the lastbuttonState with the current state.
// If the current state is different from the last state, then increment counter
// Otherwise do nothing till a new current state comes

static unsigned char lastbuttonState[SDL_NUM_SCANCODES];
void PressingCounter(int *count_A, int *count_B, int *count_C, int *count_D){

    if (xSemaphoreTake(buttons.lock, 0) == pdTRUE) {

            if (buttons.buttons[KEYCODE(A)] && !strcmp(buttons.buttons[KEYCODE(A)] , lastbuttonState) )
            {
                (*count_A)++; 
                memcpy(lastbuttonState , buttons.buttons[KEYCODE(A)], strlen(buttons.buttons[KEYCODE(A)]));
            }
            if (buttons.buttons[KEYCODE(B)] && !strcmp(buttons.buttons[KEYCODE(B)] , lastbuttonState) )
            {
                (*count_B)++;
                memcpy(lastbuttonState , buttons.buttons[KEYCODE(B)], strlen(buttons.buttons[KEYCODE(B)]));
            }   
            if (buttons.buttons[KEYCODE(C)] && !strcmp(buttons.buttons[KEYCODE(C)] , lastbuttonState) )
            {
                memcpy(lastbuttonState , buttons.buttons[KEYCODE(C)], strlen(buttons.buttons[KEYCODE(C)]));
                (*count_C)++;
            }
            if (buttons.buttons[KEYCODE(D)] && !strcmp(buttons.buttons[KEYCODE(D)] , lastbuttonState) )
            {
                memcpy(lastbuttonState , buttons.buttons[KEYCODE(D)], strlen(buttons.buttons[KEYCODE(D)]));
                (*count_D)++;   
            }
            xSemaphoreGive(buttons.lock); 
        }
    
    if(tumEventGetMouseLeft()) // if left mouse button is PRESSED
        {                       // reset the counters
            *count_A = 0;    
            *count_B = 0;
            *count_C = 0;
            *count_D = 0;
        }  
    
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
        vDrawCircle(x_mouse/5, y_mouse/5);
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

        PressingCounter(&count_A, &count_B, &count_C, &count_D);

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
       
        tumDrawUpdateScreen(); // Refresh the screen to draw string

        vTaskDelay((TickType_t)20);
    }
}

void vDemoTask2(){


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
            if (xQueueReceive(StateQueue, &input, portMAX_DELAY) ==
                pdTRUE)
                if (xTaskGetTickCount() - last_change >
                    state_change_period) {
                    changeState(&current_state, input);
                    state_changed = 1;
                    last_change = xTaskGetTickCount();
                }

initial_state:
        // Handle current state
        if (state_changed) {
            switch (current_state) {
                case STATE_ONE:
                    if (vDemoTask2) {
                        vTaskSuspend(vDemoTask2);
                    }
                    if (vDemoTask1) {
                        vTaskResume(vDemoTask1);
                    }
                    break;
                case STATE_TWO:
                    if (vDemoTask1) {
                        vTaskSuspend(vDemoTask1);
                    }
                    if (vDemoTask2) {
                        vTaskResume(vDemoTask2);
                    }
                    break;
                default:
                    break;
            }
            state_changed = 0;
        }
    }
}
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


    buttons.lock = xSemaphoreCreateMutex(); // Enable the locking mechanism
    if (!buttons.lock) {  // if lock is used by sth. else
        PRINT_ERROR("Failed to create buttons lock");
        goto err_buttons_lock;
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



    vTaskStartScheduler();

    return EXIT_SUCCESS;

err_statemachine:
    vQueueDelete(StateQueue);
err_state_queue:
    vSemaphoreDelete(StateQueue);
err_demotask:
    vSemaphoreDelete(buttons.lock);
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
