/*
 * FreeRTOS V202212.00
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

/******************************************************************************
 * NOTE 1:  This project provides two demo applications.  A simple blinky
 * style project, and a more comprehensive test and demo application.  The
 * mainCREATE_SIMPLE_BLINKY_DEMO_ONLY setting in main.c is used to select
 * between the two.  See the notes on using mainCREATE_SIMPLE_BLINKY_DEMO_ONLY
 * in main.c.  This file implements the simply blinky style version.
 *
 * NOTE 2:  This file only contains the source code that is specific to the
 * blinky demo.  Generic functions, such FreeRTOS hook functions, and functions
 * required to configure the hardware are defined in main.c.
 ******************************************************************************
 *
 * blinky() creates one queue, and two tasks.  It then starts the
 * scheduler.
 *
 * The Queue Send Task:
 * The queue send task is implemented by the prvQueueSendTask() function in
 * this file.  prvQueueSendTask() sits in a loop that causes it to repeatedly
 * block for 3000 milliseconds, before sending the value 100 to the queue that
 * was created within blinky().  Once the value is sent, the task loops
 * back around to block for another 3000 milliseconds...and so on.
 *
 * The Queue Receive Task:
 * The queue receive task is implemented by the prvQueueReceiveTask() function
 * in this file.  prvQueueReceiveTask() sits in a loop where it repeatedly
 * blocks on attempts to read data from the queue that was created within
 * blinky().  When data is received, the task checks the value of the
 * data, and if the value equals the expected 100, toggles an LED.  The 'block
 * time' parameter passed to the queue receive function specifies that the task
 * should be held in the Blocked state indefinitely to wait for data to be
 * available on the queue.  The queue receive task will only leave the Blocked
 * state when the queue send task writes to the queue.  As the queue send task
 * writes to the queue every 3000 milliseconds, the queue receive task leaves
 * the Blocked state every 3000 milliseconds, and therefore toggles the LED
 * every 200 milliseconds.
 */

/* Standard includes. */
#include <string.h>
#include <unistd.h>


/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <neorv32.h>

/* Platform UART configuration */
#define UART_BAUD_RATE (19200)         // transmission speed
#define UART_HW_HANDLE (NEORV32_UART0) // use UART0 (primary UART)

/* Priorities used by the tasks. */
#define mainQUEUE_SEND_TASK_PRIORITY            ( tskIDLE_PRIORITY + 1 )

/* The rate at which data is sent to the queue.  The 500ms value is converted
 * to ticks using the pdMS_TO_TICKS() macro. */
#define mainQUEUE_SEND_FREQUENCY_MS             pdMS_TO_TICKS( 500 )

/* The maximum number items the queue can hold.  The priority of the receiving
 * task is above the priority of the sending task, so the receiving task will
 * preempt the sending task and remove the queue items each time the sending task
 * writes to the queue.  Therefore the queue will never have more than one item in
 * it at any time, and even with a queue length of 1, the sending task will never
 * find the queue full. */
#define mainQUEUE_LENGTH                        ( 1 )


#define SIZE 45 // Size of the matrices
#define NUM_TASKS 4 // Number of parallel tasks

// Define matrices
int matrix1[SIZE][SIZE];
int matrix2[SIZE][SIZE];
int result[SIZE][SIZE];

uint64_t inicio = 0;

uint64_t tempo  = 0;

/*-----------------------------------------------------------*/

SemaphoreHandle_t xSemaphore; // Variável global

void longPrint(const char * string, double valor){
  int inteiro = (int) valor;
  int fracionario = (int)((valor - (int) valor)*1000000000);
  neorv32_uart0_printf("%s %u.%u", string, inteiro, fracionario);
}



void postExecutionTask(void *pvParameters) {

    tempo = neorv32_mtime_get_time() - inicio;
    longPrint("TEMPO SOFTWARE: ", ((double)tempo)/50000000);
    neorv32_uart_printf(UART_HW_HANDLE, "\n");

    // Wait for tasks to finish
    for (int i = 0; i < 4; i++) {
        xSemaphoreTake(xSemaphore, portMAX_DELAY);
    }

    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            neorv32_uart_printf(UART_HW_HANDLE, "%d ", (int)matrix1[i][j]);
        }
        neorv32_uart_printf(UART_HW_HANDLE, "\n");
    }

    neorv32_uart_printf(UART_HW_HANDLE, "\n\n");
    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            neorv32_uart_printf(UART_HW_HANDLE, "%d ", (int)matrix2[i][j]);
        }
        neorv32_uart_printf(UART_HW_HANDLE, "\n");
    }
    neorv32_uart_printf(UART_HW_HANDLE, "\n\n");
    neorv32_uart_printf(UART_HW_HANDLE, "Resultado Matriz\n");
    // Print an impression of the result matrix
    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            neorv32_uart_printf(UART_HW_HANDLE, "%d ", (int)result[i][j]);
        }
        neorv32_uart_printf(UART_HW_HANDLE, "\n");
    }

    // Delete this task when done
    vTaskDelete(NULL);
}

/**
 * Called by main when mainCREATE_SIMPLE_BLINKY_DEMO_ONLY is set to 1 in
 * main.c.
 */
void mult_matrix();
// Function to initialize a matrix with random values
void initializeMatrix(int matrix[][SIZE]) {
    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            matrix[i][j] = rand() % 5; // Random values between 0 and 99
        }
    }
}

// Function to perform matrix multiplication in a task
void multiplyMatrixTask(void *pvParameters) {
    int task_id = (int)pvParameters;
    int rows_per_task = SIZE / NUM_TASKS;
    int extra_rows = SIZE % NUM_TASKS;  // Remaining rows to distribute
    int start_row = task_id * rows_per_task + (task_id < extra_rows ? task_id : extra_rows);
    int end_row = start_row + rows_per_task + (task_id < extra_rows ? 1 : 0);

    for (int i = start_row; i < end_row; i++) {
        for (int j = 0; j < SIZE; j++) {
            result[i][j] = 0;  // Initialize the element to 0
            for (int k = 0; k < SIZE; k++) {
                result[i][j] += matrix1[i][k] * matrix2[k][j];
            }
        }
    }
    xSemaphoreGive(xSemaphore); // Signal completion
    vTaskDelete(NULL);
}



/*-----------------------------------------------------------*/

void mult_matrix( void ) {

    neorv32_uart_printf(UART_HW_HANDLE, "Mult Matrix\n");

    // Initialize matrices
    initializeMatrix(matrix1);
    //initializeMatrix(matrix2);

    for(int i = 0; i < SIZE; i++){
        for(int j = 0; j < SIZE; j++){
            if(j == i)
                matrix2[i][j] = 1;
            else
                matrix2[i][j] = 0;
        }
    } 
    
    xSemaphore = xSemaphoreCreateCounting(4, 0); // Na função de multiplicar matriz, inicializa

    // Create tasks for parallel matrix multiplication
    xTaskCreate(multiplyMatrixTask, "MatrixTask1", configMINIMAL_STACK_SIZE, (void *)0, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(multiplyMatrixTask, "MatrixTask2", configMINIMAL_STACK_SIZE, (void *)1, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(multiplyMatrixTask, "MatrixTask3", configMINIMAL_STACK_SIZE, (void *)2, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(multiplyMatrixTask, "MatrixTask4", configMINIMAL_STACK_SIZE, (void *)3, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(postExecutionTask,  "FinalTask"  , configMINIMAL_STACK_SIZE, (void *)3, tskIDLE_PRIORITY + 1, NULL);
    // (StackType_t)500
    
    

    
    neorv32_uart_printf(UART_HW_HANDLE, "Task Criadas\n");
    inicio = neorv32_mtime_get_time();
    // Start the scheduler
    vTaskStartScheduler();
    
    
}




