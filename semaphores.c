/*
 * Paulo Pedreiras, 2022/02
 * Zephyr: Simple thread creation example (2)
 * 
 * One of the tasks is periodc, the other two are activated via a semaphore. Data communicated via sharem memory 
 *
 * Base documentation:
 *      https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/zephyr/reference/kernel/index.html
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

#define PWM_LED0_NODE	DT_ALIAS(pwm_led0)

#if DT_NODE_HAS_STATUS(PWM_LED0_NODE, okay)
#define PWM_CTLR	DT_PWMS_CTLR(PWM_LED0_NODE)
#define PWM_CHANNEL	DT_PWMS_CHANNEL(PWM_LED0_NODE)
#define PWM_FLAGS	DT_PWMS_FLAGS(PWM_LED0_NODE)
#else
#error "Unsupported board: pwm-led0 devicetree alias is not defined"
#define PWM_CTLR	DT_INVALID_NODE
#define PWM_CHANNEL	0
#define PWM_FLAGS	0
#endif

#define MIN_PERIOD_USEC	(USEC_PER_SEC / 64U)
#define MAX_PERIOD_USEC	USEC_PER_SEC


#define ADC_NID DT_NODELABEL(adc) 
#define ADC_RESOLUTION 10
#define ADC_GAIN ADC_GAIN_1_4
#define ADC_REFERENCE ADC_REF_VDD_1_4
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)
#define ADC_CHANNEL_ID 1  

/* This is the actual nRF ANx input to use. Note that a channel can be assigned to any ANx. In fact a channel can */
/*    be assigned to two ANx, when differential reading is set (one ANx for the positive signal and the other one for the negative signal) */  
/* Note also that the configuration of differnt channels is completely independent (gain, resolution, ref voltage, ...) */
#define ADC_CHANNEL_INPUT NRF_SAADC_INPUT_AIN1 

#define BUFFER_SIZE 1
#define ARRAY_SIZE 10

/* Other defines */
#define TIMER_INTERVAL_MSEC 1000 /* Interval between ADC samples */


/* Size of stack area used by each thread (can be thread specific, if necessary)*/
#define STACK_SIZE 1024

/* Thread scheduling priority */
#define thread_A_prio 1
#define thread_B_prio 1
#define thread_C_prio 1

/* Therad periodicity (in ms)*/
#define thread_A_period 1000

/* Create thread stack space */
K_THREAD_STACK_DEFINE(thread_A_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_B_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_C_stack, STACK_SIZE);
  
/* Create variables for thread data */
struct k_thread thread_A_data;
struct k_thread thread_B_data;
struct k_thread thread_C_data;

/* Create task IDs */
k_tid_t thread_A_tid;
k_tid_t thread_B_tid;
k_tid_t thread_C_tid;

/* Global vars (shared memory between tasks A/B and B/C, resp) */
uint16_t avg_value = 0;
uint16_t adcbuffer[ARRAY_SIZE];
uint8_t ptr = 0;

/* Semaphores for task synch */
struct k_sem sem_ab;
struct k_sem sem_bc;

/* Thread code prototypes */
void thread_A_code(void *argA, void *argB, void *argC);
void thread_B_code(void *argA, void *argB, void *argC);
void thread_C_code(void *argA, void *argB, void *argC);

/* ADC channel configuration */
static const struct adc_channel_cfg my_channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id = ADC_CHANNEL_ID,
	.input_positive = ADC_CHANNEL_INPUT
};

/* Global vars */
const struct device *adc_dev = NULL;
const struct device *pwm = NULL;
static uint16_t adc_sample_buffer[BUFFER_SIZE];

/* Takes one sample */
static int adc_sample(void)
{
	int ret;
	const struct adc_sequence sequence = {
		.channels = BIT(ADC_CHANNEL_ID),
		.buffer = adc_sample_buffer,
		.buffer_size = sizeof(adc_sample_buffer),
		.resolution = ADC_RESOLUTION,
	};

	if (adc_dev == NULL) {
            printk("adc_sample(): error, must bind to adc first \n\r");
            return -1;
	}

	ret = adc_read(adc_dev, &sequence);
	if (ret) {
          printk("adc_read() failed with code %d\n", ret);
        } else if(adc_sample_buffer[0] > 1023) {
          return -1;
        } else {
          adcbuffer[ptr] = adc_sample_buffer[0];
          ptr = (ptr+1)%ARRAY_SIZE;
        }
        
	return ret;
}

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

/* Main function */
void main(void) {

    int err=0;
    

    pwm = DEVICE_DT_GET(PWM_CTLR);
	if (!device_is_ready(pwm)) {
		printk("Error: PWM device %s is not ready\n", pwm->name);
		return;
	}

    /* Welcome message */
    printk("\n\r Simple adc demo for  \n\r");
    printk(" Reads an analog input connected to AN%d and prints its raw and mV value \n\r", ADC_CHANNEL_ID);
    printk(" *** ASSURE THAT ANx IS BETWEEN [0...3V]\n\r");
         
    /* ADC setup: bind and initialize */
    adc_dev = device_get_binding(DT_LABEL(ADC_NID));
	if (!adc_dev) {
        printk("ADC device_get_binding() failed\n");
    } 
    err = adc_channel_setup(adc_dev, &my_channel_cfg);
    if (err) {
        printk("adc_channel_setup() failed with error code %d\n", err);
    }
    
    /* It is recommended to calibrate the SAADC at least once before use, and whenever the ambient temperature has changed by more than 10 Â°C */
    NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;

   
    
    /* Welcome message */
    printf("\n\r Illustration of the use of shmem + semaphores\n\r");
    
    /* Create and init semaphores */
    k_sem_init(&sem_ab, 0, 1);
    k_sem_init(&sem_bc, 0, 1);
    
    /* Create tasks */
    thread_A_tid = k_thread_create(&thread_A_data, thread_A_stack,
        K_THREAD_STACK_SIZEOF(thread_A_stack), thread_A_code,
        NULL, NULL, NULL, thread_A_prio, 0, K_NO_WAIT);

    thread_B_tid = k_thread_create(&thread_B_data, thread_B_stack,
        K_THREAD_STACK_SIZEOF(thread_B_stack), thread_B_code,
        NULL, NULL, NULL, thread_B_prio, 0, K_NO_WAIT);

    thread_B_tid = k_thread_create(&thread_C_data, thread_C_stack,
        K_THREAD_STACK_SIZEOF(thread_C_stack), thread_C_code,
        NULL, NULL, NULL, thread_C_prio, 0, K_NO_WAIT);

    
    return;

} 

/* Thread code implementation */
void thread_A_code(void *argA , void *argB, void *argC)
{
    /* Timing variables to control task periodicity */
    int64_t fin_time=0, release_time=0;

    /* Compute next release instant */
    release_time = k_uptime_get() + thread_A_period;

    /* Thread loop */

    while(true){
        /* Get one sample, checks for errors and prints the values */
        int err=adc_sample();
        if(err) {
            printk("adc_sample() failed with error code %d\n\r",err);
        }
        else {
            if(adc_sample_buffer[0] > 1023) {
                printk("Adc reading out of range\n\r");
            }
            else {
                /* ADC is set to use gain of 1/4 and reference VDD/4, so input range is 0...VDD (3 V), with 10 bit resolution */
                printk("ADC reading: raw:%4u / %4u mV: \n\r",adc_sample_buffer[0],(uint16_t)(1000*adc_sample_buffer[0]*((float)3/1023)));
            }
        }

        /* Sleep a while ... */
        k_sem_give(&sem_ab);
       
        /* Wait for next release instant */ 
        fin_time = k_uptime_get();
        if( fin_time < release_time) {
            k_msleep(release_time - fin_time);
            release_time += thread_A_period;
        }
    }

}

void thread_B_code(void *argA , void *argB, void *argC)
{

    while(1) {
        k_sem_take(&sem_ab,  K_FOREVER);
        
        avg_value = filter(adcbuffer);
        if((int16_t)avg_value==-1) {
          printk("Task B could not transform into a valid value\n", avg_value);
        } else {
          printk("Task B filter average into: %u\n", avg_value);
        }
        
        k_sem_give(&sem_bc);

        
  }
}

void thread_C_code(void *argA , void *argB, void *argC)
{

    while(1) {
        k_sem_take(&sem_bc, K_FOREVER);         
        if((int16_t)avg_value==-1) {
            printk("Read error!\n");
        } else {
            int ret = pwm_pin_set_usec(pwm, PWM_CHANNEL,  1023U, avg_value, PWM_FLAGS);
            printk("Read avg value: %u\n", avg_value);
        }

        
  }
}