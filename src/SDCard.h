/* SD card and FAT filesystem example.
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

static const char *TAG1 = "sd";

#define MOUNT_POINT "/sdcard"

#define PIN_NUM_MISO  GPIO_NUM_19
#define PIN_NUM_MOSI  GPIO_NUM_23
#define PIN_NUM_CLK   GPIO_NUM_18
#define PIN_NUM_CS    GPIO_NUM_5

void sdcard_init(void);
bool sdcard_openFile(const char* file_name);
void sdcard_writeFile(const char* line);
void sdcard_writeFile(uint32_t t, float yaw, float pitch, float ay, float vy, float py);
void sdcard_closeFile(void);
void sdcard_end(void);
