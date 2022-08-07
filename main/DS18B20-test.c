#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define GPIO_DS18B20_0      (33)       //gpio33
#define MAX_DEVICES         (8)
#define RESOLUTION          (DS18B20_RESOLUTION_12_BIT)
#define SAMPLE_PERIOD       (1000)     // milliseconds

#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"

void DS18B20_task(){
    for(;;){    
        // Override global log level
        esp_log_level_set("*", ESP_LOG_INFO);

        // To debug, use 'make menuconfig' to set default Log level to DEBUG, then uncomment:
        //esp_log_level_set("owb", ESP_LOG_DEBUG);
        //esp_log_level_set("ds18b20", ESP_LOG_DEBUG);

        // Stable readings require a brief period before communication
        vTaskDelay(3000.0 / portTICK_PERIOD_MS);

        // Create a 1-Wire bus, using the RMT timeslot driver
        OneWireBus * owb;
        owb_rmt_driver_info rmt_driver_info;
        owb = owb_rmt_initialize(&rmt_driver_info, GPIO_DS18B20_0, RMT_CHANNEL_1, RMT_CHANNEL_0);
        owb_use_crc(owb, true);  // enable CRC check for ROM code

        // Find all connected devices
        printf("Find devices:\n");
        OneWireBus_ROMCode device_rom_codes[MAX_DEVICES] = {0};
        int num_devices = 0;
        OneWireBus_SearchState search_state = {0};
        bool found = false;
        owb_search_first(owb, &search_state, &found);
        while (found)
        {
            char rom_code_s[17];
            owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));
            printf("  %d : %s\n", num_devices, rom_code_s);
            device_rom_codes[num_devices] = search_state.rom_code;
            ++num_devices;
            owb_search_next(owb, &search_state, &found);
        }
        printf("Found %d device%s\n", num_devices, num_devices == 1 ? "" : "s");

        // In this example, if a single device is present, then the ROM code is probably
        // not very interesting, so just print it out. If there are multiple devices,
        // then it may be useful to check that a specific device is present.

        if (num_devices == 1)
        {
            // For a single device only:
            OneWireBus_ROMCode rom_code;
            owb_status status = owb_read_rom(owb, &rom_code);
            if (status == OWB_STATUS_OK)
            {
                char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
                owb_string_from_rom_code(rom_code, rom_code_s, sizeof(rom_code_s));
                printf("Single device %s present\n", rom_code_s);
            }
            else
            {
                printf("An error occurred reading ROM code: %d", status);
            }
        }
        else
        {
            // Search for a known ROM code (LSB first):
            // For example: 0x1502162ca5b2ee28
            OneWireBus_ROMCode known_device = {
                .fields.family = { 0x28 },
                .fields.serial_number = { 0xee, 0xb2, 0xa5, 0x2c, 0x16, 0x02 },
                .fields.crc = { 0x15 },
            };
            char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
            owb_string_from_rom_code(known_device, rom_code_s, sizeof(rom_code_s));
            bool is_present = false;

            owb_status search_status = owb_verify_rom(owb, known_device, &is_present);
            if (search_status == OWB_STATUS_OK)
            {
                printf("Device %s is %s\n", rom_code_s, is_present ? "present" : "not present");
            }
            else
            {
                printf("An error occurred searching for known device: %d", search_status);
            }
        }

        // Create DS18B20 devices on the 1-Wire bus
        DS18B20_Info * devices[MAX_DEVICES] = {0};
        for (int i = 0; i < num_devices; ++i)
        {
            DS18B20_Info * ds18b20_info = ds18b20_malloc();  // heap allocation
            devices[i] = ds18b20_info;

            if (num_devices == 1)
            {
                printf("Single device optimisations enabled\n");
                ds18b20_init_solo(ds18b20_info, owb);          // only one device on bus
            }
            else
            {
                ds18b20_init(ds18b20_info, owb, device_rom_codes[i]); // associate with bus and device
            }
            ds18b20_use_crc(ds18b20_info, true);           // enable CRC check on all reads
            ds18b20_set_resolution(ds18b20_info, RESOLUTION);
        }

        // Read temperatures from all sensors sequentially
        if(num_devices>0)
        {
            printf("\nTemperature readings (degrees C):\n");
            for (int i = 0; i < num_devices; ++i)
            {
                float temp;
                ds18b20_convert_and_read_temp(devices[i], &temp);
                printf("  %d: %.3f\n", i, temp);
            }
        }
        else
        {
            printf("\nNo DS18B20 devices detected!\n");
        }

        // clean up dynamically allocated data
        for (int i = 0; i < num_devices; ++i)
        {
            ds18b20_free(&devices[i]);
        }
        owb_uninitialize(owb);
    }
}

void app_main(void)
{   
    xTaskCreate(&DS18B20_task, "DS18B20_task", 4096, NULL, 5, NULL);
}

