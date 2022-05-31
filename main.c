#include <stdio.h>
#include <bits/stdc++.h>

#define AVG_N 10

float filter(int *arr)
{
    int i;
    int sum = 0;

    for (i = 0; i < AVG_N; i++)
    {
        sum += arr[i];
    }
    float mean = sum / (float)AVG_N;

    sum = 0;
    int ctn = 0;
    for (i = 0; i < AVG_N; i++)
    {
        if (mean * 1.1 > (float)arr[i] && mean * 0.9 < (float)arr[i])
        {
            sum += arr[i];
            ctn += 1;
        }
    }
    return ctn ? sum / (float)ctn : mean;
}

#define SW4_NODE 0x1c
/*
int outret;
gpio0_dev[btidx] = device_get_binding(DT_LABEL(GPIO0_NID));
outret = gpio_pin_configure(gpio0_dev[btidx], SWNODE[btidx], GPIO_INPUT | GPIO_PULL_UP);

outret = gpio_pin_interrupt_configure(gpio0_dev[btidx], SWNODE[btidx], GPIO_INT_EDGE_TO_ACTIVE);
*/

int main()
{
    int results[] = {48, 48, 48, 48, 51, 51, 51, 51, 48, 55};
    printf("avg: %.2f\n", filter(results));
}