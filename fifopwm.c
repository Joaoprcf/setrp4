/**
 * @file fifopwm.c
 * @author João Paulo (joaoprcf@ua.pt)
 * @brief Fifo implementation
 * @version 0.1
 * @date 2022-06-01
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/adc.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <string.h>
#include <timing/timing.h>
#include <stdlib.h>
#include <stdio.h>
#include <hal/nrf_saadc.h>
#include <drivers/pwm.h>

#define PWM_LED0_NODE DT_ALIAS(pwm_led0)

#if DT_NODE_HAS_STATUS(PWM_LED0_NODE, okay)
#define PWM_CTLR DT_PWMS_CTLR(PWM_LED0_NODE)
#define PWM_CHANNEL DT_PWMS_CHANNEL(PWM_LED0_NODE)
#define PWM_FLAGS DT_PWMS_FLAGS(PWM_LED0_NODE)
#else
#error "Unsupported board: pwm-led0 devicetree alias is not defined"
#define PWM_CTLR DT_INVALID_NODE
#define PWM_CHANNEL 0
#define PWM_FLAGS 0
#endif

#define ADC_NID DT_NODELABEL(adc)
#define ADC_RESOLUTION 10
#define ADC_GAIN ADC_GAIN_1_4
#define ADC_REFERENCE ADC_REF_VDD_1_4
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)
#define ADC_CHANNEL_ID 1
#define ADC_CHANNEL_INPUT NRF_SAADC_INPUT_AIN1

#define BUFFER_SIZE 1 ///< buffer size
#define ARRAY_SIZE 10 ///< array size

/* Other defines */
#define TIMER_INTERVAL_MSEC 1000 /* Interval between ADC samples */

/* Size of stack area used by each thread (can be thread specific, if necessary)*/
#define STACK_SIZE 1024

/* Thread scheduling priority */
#define thread_A_prio 1 ///< Thread A priority
#define thread_B_prio 1 ///< Thread B priority
#define thread_C_prio 1 ///< Thread C priority

/* Therad periodicity (in ms)*/
#define thread_A_period 1000 ///< Thread A period

/* Create thread stack space */
K_THREAD_STACK_DEFINE(thread_A_stack, STACK_SIZE); ///< Thread A stack
K_THREAD_STACK_DEFINE(thread_B_stack, STACK_SIZE); ///< Thread B stack
K_THREAD_STACK_DEFINE(thread_C_stack, STACK_SIZE); ///< Thread C stack

/* Create variables for thread data */
struct k_thread thread_A_data; ///< Thread A data
struct k_thread thread_B_data; ///< Thread B data
struct k_thread thread_C_data; ///< Thread C data

/* Create task IDs */
k_tid_t thread_A_tid; ///< ID thread A
k_tid_t thread_B_tid; ///< ID thread B
k_tid_t thread_C_tid; ///< ID thread C

/* Global vars */
uint16_t adcbuffer[ARRAY_SIZE]; ///< adc buffer
uint8_t ptr = 0;                ///< adc buffer ptr

/* Create fifos*/
struct k_fifo fifo_ab; ///< fifo between A and B
struct k_fifo fifo_bc; ///< fifo between B and C

/* Create fifo data structure and variables */
struct data_item_t
{
    void *fifo_reserved; ///< 1st word reserved for use by FIFO
    uint16_t data;       ///< Actual data
};

/* Thread code prototypes */
void thread_A(void *argA, void *argB, void *argC);
void thread_B(void *argA, void *argB, void *argC);
void thread_C(void *argA, void *argB, void *argC);

/* ADC channel configuration */
static const struct adc_channel_cfg my_channel_cfg = { ///< adc config
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id = ADC_CHANNEL_ID,
    .input_positive = ADC_CHANNEL_INPUT};

/* Global vars */
const struct device *adc_dev = NULL; ///< adc device config
const struct device *pwm = NULL;     ///< pwm config

/**
 * @brief Retrieve an ADC sample a buffer
 *
 * @return int
 */
static int adc_sample(uint16_t *adc_sample_buffer)
{
    int ret;
    const struct adc_sequence sequence = {
        .channels = BIT(ADC_CHANNEL_ID),
        .buffer = adc_sample_buffer,
        .buffer_size = sizeof(adc_sample_buffer),
        .resolution = ADC_RESOLUTION,
    };

    if (adc_dev == NULL)
    {
        printk("adc_sample(): error, must bind to adc first \n\r");
        return -1;
    }

    ret = adc_read(adc_dev, &sequence);
    if (ret)
    {
        printk("adc_read() failed with code %d\n", ret);
    }
    else if (adc_sample_buffer[0] > 1023)
    {
        return -1;
    }
    else
    {
        adcbuffer[ptr] = adc_sample_buffer[0];
        ptr = (ptr + 1) % ARRAY_SIZE;
    }

    return ret;
}

/**
 * @brief Filter array values from the adc buffer
 *
 * @return int16_t
 */
int16_t filter(uint16_t *arr)
{
    int i;
    uint16_t sum = 0;

    for (i = 0; i < ARRAY_SIZE; i++)
    {
        sum += arr[i];
        printf("%d - \n", arr[i]);
    }
    float mean = sum / (float)ARRAY_SIZE;

    sum = 0;
    int ctn = 0;
    for (i = 0; i < ARRAY_SIZE; i++)
    {
        if (mean * 1.1 > (float)arr[i] && mean * 0.9 < (float)arr[i])
        {
            sum += arr[i];
            ctn += 1;
        }
    }
    return ctn ? (uint16_t)(sum / (float)ctn) : -1;
}

/**
 * @brief Main function where the fifos and threads are initialized
 *
 * @return void
 */
void main(void)
{

    int err = 0;

    pwm = DEVICE_DT_GET(PWM_CTLR);
    if (!device_is_ready(pwm))
    {
        printk("Error: PWM device %s is not ready\n", pwm->name);
        return;
    }

    /* Welcome message */
    printk("\n\r Simple adc demo for  \n\r");
    printk(" Reads an analog input connected to AN%d and prints its raw and mV value \n\r", ADC_CHANNEL_ID);
    printk(" *** ASSURE THAT ANx IS BETWEEN [0...3V]\n\r");

    /* ADC setup: bind and initialize */
    adc_dev = device_get_binding(DT_LABEL(ADC_NID));
    if (!adc_dev)
    {
        printk("ADC device_get_binding() failed\n");
    }
    err = adc_channel_setup(adc_dev, &my_channel_cfg);
    if (err)
    {
        printk("adc_channel_setup() failed with error code %d\n", err);
    }

    /* It is recommended to calibrate the SAADC at least once before use, and whenever the ambient temperature has changed by more than 10 Â°C */
    NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;

    /* Welcome message */
    printf("\n\r Illustration of the use of fifos\n\r");

    /* Create and init fifo */
    k_fifo_init(&fifo_ab);
    k_fifo_init(&fifo_bc);

    /* Create tasks */
    thread_A_tid = k_thread_create(&thread_A_data, thread_A_stack,
                                   K_THREAD_STACK_SIZEOF(thread_A_stack), thread_A,
                                   NULL, NULL, NULL, thread_A_prio, 0, K_NO_WAIT);

    thread_B_tid = k_thread_create(&thread_B_data, thread_B_stack,
                                   K_THREAD_STACK_SIZEOF(thread_B_stack), thread_B,
                                   NULL, NULL, NULL, thread_B_prio, 0, K_NO_WAIT);

    thread_B_tid = k_thread_create(&thread_C_data, thread_C_stack,
                                   K_THREAD_STACK_SIZEOF(thread_C_stack), thread_C,
                                   NULL, NULL, NULL, thread_C_prio, 0, K_NO_WAIT);

    return;
}

/**
 * @brief Thread responsible to get adc values
 *
 * @return void
 */
void thread_A(void *argA, void *argB, void *argC)
{
    static uint16_t adc_sample_buffer[BUFFER_SIZE];

    /* Timing variables to control task periodicity */
    int64_t fin_time = 0, release_time = 0;

    /* Compute next release instant */
    release_time = k_uptime_get() + thread_A_period;

    /* Thread loop */
    struct data_item_t data_ab;
    while (true)
    {
        /* Get one sample, checks for errors and prints the values */
        int err = adc_sample(adc_sample_buffer);
        if (err)
        {
            printk("adc_sample() failed with error code %d\n\r", err);
        }
        else
        {
            if (adc_sample_buffer[0] > 1023)
            {
                printk("Adc reading out of range\n\r");
            }
            else
            {
                /* ADC is set to use gain of 1/4 and reference VDD/4, so input range is 0...VDD (3 V), with 10 bit resolution */
                printk("ADC reading: raw:%4u / %4u mV: \n\r", adc_sample_buffer[0], (uint16_t)(1000 * adc_sample_buffer[0] * ((float)3 / 1023)));
            }
        }

        /* Sleep a while ... */
        k_fifo_put(&fifo_ab, &data_ab);

        /* Wait for next release instant */
        fin_time = k_uptime_get();
        if (fin_time < release_time)
        {
            k_msleep(release_time - fin_time);
            release_time += thread_A_period;
        }
    }
}

/**
 * @brief Thread responsible to filter and average adc buffered values
 *
 * @return void
 */
void thread_B(void *argA, void *argB, void *argC)
{
    struct data_item_t data_bc;
    uint16_t avg_value = 0;
    while (1)
    {
        k_fifo_get(&fifo_ab, K_FOREVER);

        avg_value = filter(adcbuffer);
        if ((int16_t)avg_value == -1)
        {
            printk("Task B could not transform into a valid value\n", avg_value);
        }
        else
        {
            printk("Task B filter average into: %u\n", avg_value);
        }
        data_bc.data = avg_value;
        k_fifo_put(&fifo_bc, &data_bc);
    }
}

/**
 * @brief Thread responsible to assign the respective pwm power to the led given by thread B
 *
 * @return void
 */
void thread_C(void *argA, void *argB, void *argC)
{
    struct data_item_t *data_bc;
    uint16_t avg_value = 0;
    while (1)
    {
        data_bc = k_fifo_get(&fifo_bc, K_FOREVER);
        avg_value = data_bc->data;
        if ((int16_t)avg_value == -1)
        {
            printk("Read error!\n");
        }
        else
        {
            int ret = pwm_pin_set_usec(pwm, PWM_CHANNEL, 1023U, avg_value, PWM_FLAGS);
            printk("Read avg value: %u\n", avg_value);
        }
    }
}