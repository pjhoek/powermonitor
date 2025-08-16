#include <stdio.h>
#include "sdkconfig.h"

int data[100];

void app_main()
{
    for (int i = 0; (data[i] >= 0) && i < 100; i++)
    {
        printf("%d\n", i);
        printf("0x%08X\n", (unsigned int) data[i]);
    }
}