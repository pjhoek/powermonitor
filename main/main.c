#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "sdkconfig.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

//(3.43760743 kilohertz) / (50 Hz) = 68.7521486
#define CONVST_PIN GPIO_NUM_4 // GPIO4 - Start conversion
#define BUSY_PIN GPIO_NUM_27  // GPIO27 - Conversion status
#define CS_PIN GPIO_NUM_15    // GPIO15 - Chip select
#define RESET_PIN GPIO_NUM_25 // GPIO25 - Hardware reset

#define SCK_PIN GPIO_NUM_5   // GPIO5 - SCK
#define MISO_PIN GPIO_NUM_21 // GPIO21 - MISO
#define MOSI_PIN GPIO_NUM_19 // GPIO19 - MOSI (unused)

#define NUM_SAMPLES 3000ULL
const double LSB_10V = 20.0 / 262144.0;
const double I_FS_15AMP = LSB_10V * 15;

// Channel we determine the sample period from (use voltage channel)
#define VOLTAGE_CH 0

static uint8_t rawBytes[18];            // Store raw bytes
static int32_t adcData[NUM_SAMPLES][8]; // ADC values
static double calcRms[8];
static double calcPwr[8];

static void setup_pin_input(int pin)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

static void setup_pin_output(int pin)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

void loop(spi_device_handle_t spi);

void app_main()
{
    printf("Hello world!\n");

    setup_pin_output(CONVST_PIN);
    setup_pin_input(BUSY_PIN);
    setup_pin_output(CS_PIN);
    setup_pin_input(RESET_PIN);

    gpio_set_level(CONVST_PIN, 1);
    gpio_set_level(CS_PIN, 1);
    gpio_set_level(RESET_PIN, 0);

    // Hardware reset pulse
    gpio_set_level(RESET_PIN, 1);
    esp_rom_delay_us(1);
    gpio_set_level(RESET_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(1));

    // --- 1. Configure SPI bus ---
    spi_bus_config_t buscfg = {
        .mosi_io_num = MOSI_PIN,
        .miso_io_num = MISO_PIN,
        .sclk_io_num = SCK_PIN,
        .quadwp_io_num = -1, // Not used
        .quadhd_io_num = -1, // Not used
        .max_transfer_sz = 0 // Default
    };

    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // --- 2. Add SPI device (sets clock, mode, bit order) ---
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 25 * 1000 * 1000, // 25 MHz
        .mode = 1,                          // SPI_MODE2
        .spics_io_num = -1, //CS_PIN,         // Hardware CS
        .queue_size = 1
    };

    spi_device_handle_t spi;
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &spi));

    while (true)
    {
        loop(spi);
        
        // FIXME
        
        vTaskDelay(1);
        // taskYIELD();
    }
}

static void calcRmsAndPower(int ch, int startSample, int endSample)
{
    uint64_t sumRms = 0;
    int64_t sumPwr = 0;
    for (int i = startSample; i < endSample; i++)
    {
        int32_t val = adcData[i][ch];
        sumRms += val * val;
        sumPwr += val * adcData[i][VOLTAGE_CH];
    }

    uint32_t numSamples = endSample - startSample;
    calcRms[ch] = sqrt(((double)sumRms) / ((double)numSamples));
    calcPwr[ch] = ((double)sumPwr) / ((double)numSamples);
}

static bool waitForBusy(int state, uint32_t timeoutMicros)
{
    int64_t then = esp_timer_get_time();
    while (gpio_get_level(BUSY_PIN) != state)
    {
        if (esp_timer_get_time() - then > timeoutMicros)
        {
            // Timeout
            return false;
        }
    }

    // BUSY is now in state `state`
    return true;
}

static void doRead(spi_device_handle_t spi)
{
    if (!waitForBusy(0, 10 * 1000))
    {
        printf("low wait timeout\n");
    }

    gpio_set_level(CONVST_PIN, 0);
    uint32_t then = esp_cpu_get_cycle_count();
    while (esp_cpu_get_cycle_count() - then < 10)
        ;
    gpio_set_level(CONVST_PIN, 1);

    if (!waitForBusy(1, 10))
    {
        printf("high wait timeout\n");
    }

    gpio_set_level(CS_PIN, 0);
    spi_transaction_t t = {
        .length = 18 * 8,  // bits
        .tx_buffer = NULL, // send nothing
        .rx_buffer = rawBytes,
        .flags = 0
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &t));
    gpio_set_level(CS_PIN, 1);
}

void readAllChannels(spi_device_handle_t spi, int32_t *data)
{
    doRead(spi);

    for (int ch = 0; ch < 8; ch++)
    {
        int startBit = ch * 18;
        int startByte = startBit / 8;

        uint32_t buf = 0;
        for (int i = 0; i < 3; i++)
        {
            buf = (buf << 8) | rawBytes[startByte + i];
        }

        int shift = ((startByte + 3) * 8) - (startBit + 18);
        uint32_t val = (uint32_t)((buf >> shift) & 0x3FFFF);

        if (val & 0x20000)
        {
            val |= 0xFFFC0000;
        }

        data[ch] = val;
    }
}

void printStored()
{
    printf("RMS: %.6lf\n", calcRms[0] * LSB_10V);
    printf("RMS1: %.6lf\n", calcRms[1] * LSB_10V);
    printf("PWR1: %.6lf\n", calcPwr[1]);

    // for (int c = 0; c < NUM_SAMPLES; c++) {
    //   printf("%ld\n", adcData[c][0]);
    // }

    printf("---------------------------------------------------------------------------------------------------------------------------------\n");
}

void loop(spi_device_handle_t spi) {
    // Throw away the old conversion result (if any)
    doRead(spi);

    for (int c = 0; c < NUM_SAMPLES; c++)
    {
        readAllChannels(spi, adcData[c]);
    }

    uint64_t start_time = esp_timer_get_time();

    bool dcTest = false; // set true for DC testing, false for AC

    int startSample = 0;
    int endSample = NUM_SAMPLES;

    if (!dcTest)
    {
        bool sign = adcData[0][VOLTAGE_CH] >= 0;

        //  Advance `startSample` to the first sample which differs in sign from the literal zeroth sample (this has to be a zero crossing)
        while (((adcData[startSample][VOLTAGE_CH] >= 0) == sign) && startSample < NUM_SAMPLES)
        {
            startSample++;
            // Serial.println(startSample);
            // Serial.println(NUM_SAMPLES);
            // Serial.println(((uint32_t) startSample) < ((uint32_t) NUM_SAMPLES));
        }
        //  Serial.println("YO2");
        // Serial.println(startSample);

        if (startSample == NUM_SAMPLES)
        {
            printf("sample overflow finding zero crossing\n");
            return;
        }
        // Serial.println("YO3");
        // Serial.flush();

        int halfPeriods = 0;
        int endSample = startSample;
        bool endSign = adcData[endSample][VOLTAGE_CH] >= 0;

        printf("%d\n", startSample + 1);

        // Scan over all of the samples after `startSample` until the end
        for (int currentSample = startSample + 1; ((uint32_t) currentSample) < NUM_SAMPLES; currentSample++)
        {
            printf("%d\n", currentSample);
            printf("%llu\n", NUM_SAMPLES);
            printf("%d\n\n", currentSample < NUM_SAMPLES);
            fflush(stdout);

            bool currentSign = adcData[currentSample][VOLTAGE_CH] >= 0;

            // Serial.println((uint32_t) currentSample);
            // Serial.println((uint32_t) NUM_SAMPLES);
            // Serial.println(((uint32_t) currentSample) < ((uint32_t) NUM_SAMPLES));
            // Serial.println(((uint32_t) currentSample) < ((uint32_t) 2000));
            // Serial.println(((uint32_t) 8841) < ((uint32_t) NUM_SAMPLES));
            // Serial.println(((uint32_t) 8841) < ((uint32_t) 2000));

            // volatile int myValue = 8841;
            // myValue++;
            //  Serial.println("LLLLLLLLLLLLL");
            // Serial.println(((uint32_t) myValue) < ((uint32_t) NUM_SAMPLES));
            // Serial.println(((uint32_t) myValue) < ((uint32_t) 2000));

            // If the current sample now has a different sign to the last saved "end sample", updated
            // the saved "end sample" to this position (we have just found another zero crossing).
            if (currentSign != endSign)
            {
                endSample = currentSample;
                endSign = currentSign;
                halfPeriods++;
            }
        }
        
        printf("%d\n", halfPeriods);

        if (!halfPeriods)
        {
            printf("didn't find any half periods!?!\n");
            return;
        }

        if (false && abs((halfPeriods * 134 *4) - (endSample - startSample)) > 50)
        { //(halfPeriods < 10 || halfPeriods >20) {
            printf("half periods and range disagree! halfPeriods=%d: %d - %d\n", halfPeriods, startSample, endSample);
            return;
        }
        
        // printf("halfPeriods=%d: %d = %d - %d\n", halfPeriods, endSample - startSample, startSample, endSample);
        uint64_t end_time = esp_timer_get_time();
        uint64_t duration_us = end_time - start_time;
        printf("Code section took %llu microseconds (%llu milliseconds)\n", duration_us, duration_us / 1000);
    }

    for (int ch = 0; ch < 8; ch++)
    {
        calcRmsAndPower(ch, startSample, endSample);
    }

    printStored();
}

