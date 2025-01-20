#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Configuration constants
#define N_GRAINS     100    // Number of grains of sand
#define WIDTH        128     // Display width in pixels
#define HEIGHT       64      // Display height in pixels
#define MAX_FPS      45      // Maximum redraw rate, frames/second
#define OLED_RESET   -1      // Reset pin # (or -1 if sharing Arduino reset pin)
#define MAX_X        (WIDTH  * 256 - 1)  // Maximum X coordinate in grain space
#define MAX_Y        (HEIGHT * 256 - 1)  // Maximum Y coordinate

// Grain structure definition
struct Grain {
    int16_t  x,  y;    // Position
    int16_t vx, vy;    // Velocity
    uint16_t pos;      // Current position in display buffer
};

// Global variables
Adafruit_SSD1306 display(WIDTH, HEIGHT, &Wire, OLED_RESET);
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

Grain grain[N_GRAINS];

uint8_t img[WIDTH * HEIGHT];  // Changed to uint8_t to save memory

float xOffset, yOffset;

volatile bool physicsReady = false;
const int GRAINS_PER_FRAME = 200;  // Increased for smoother animation

// Mutex for thread safety
portMUX_TYPE physicsMux = portMUX_INITIALIZER_UNLOCKED;

// MPU Setup
void MPU_setup() {
  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("initialized OK!");
  delay(100);
}

void updatePhysics() {
    // Loop through each grain
    for (int i = 0; i < N_GRAINS; i++) {
        // Apply accelerometer-based "shake" effect to velocity
        // These values will act as forces applied to the grains.
        grain[i].vx += (a.acceleration.x * 0.1);  // Scale the x-axis acceleration to adjust velocity
        grain[i].vy += (a.acceleration.y * 0.1);  // Scale the y-axis acceleration similarly

        // Update grain positions based on velocity
        grain[i].x += grain[i].vx;
        grain[i].y += grain[i].vy;

        // Boundary check to keep grains within display area, with wraparound
        if (grain[i].x >= WIDTH) grain[i].x = 0;
        if (grain[i].x < 0) grain[i].x = WIDTH - 1;
        if (grain[i].y >= HEIGHT) grain[i].y = 0;
        if (grain[i].y < 0) grain[i].y = HEIGHT - 1;

        // Store the current position in the buffer
        grain[i].pos = grain[i].y * WIDTH + grain[i].x;
    }
}


// Global variables for task synchronization
SemaphoreHandle_t displayMutex;
volatile bool dataReady = false;

void pixelTask(void *param) {
    // Configure watchdog
    esp_task_wdt_init(10, true);
    esp_task_wdt_add(NULL);

    MPU_setup();  // Initialize the MPU sensor
    sensors_event_t a, g, temp;

    while (true) {
        // Reset watchdog timer
        esp_task_wdt_reset();

        // Get accelerometer data
        mpu.getEvent(&a, &g, &temp);

        // Update the physics based on accelerometer values
        updatePhysics();

        // Signal that new data is ready for display
        xSemaphoreTake(displayMutex, portMAX_DELAY);
        dataReady = true;
        xSemaphoreGive(displayMutex);

        // Add delay to prevent task starvation and control the update frequency
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void initializeGrains() {
    for (int i = 0; i < N_GRAINS; i++) {
        // Random initial position and velocity for grains
        grain[i].x = random(WIDTH);
        grain[i].y = random(HEIGHT);
        grain[i].vx = random(-1, 2);  // Small initial velocity
        grain[i].vy = random(-1, 2);  // Small initial velocity
        grain[i].pos = grain[i].y * WIDTH + grain[i].x;
    }
}

void setup() {
    Serial.begin(115200);

    // Create mutex for display synchronization
    displayMutex = xSemaphoreCreateMutex();

    // Initialize display with error handling
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("SSD1306 initialization failed!");
        delay(1000);
        ESP.restart();
    }

	initializeGrains();

    // Increase stack size and lower priority for the pixel task
    TaskHandle_t xHandle = NULL;
    xTaskCreatePinnedToCore(
        pixelTask,
        "PixelTask1",
        8192,           // Increased stack size for pixel task
        NULL,
        1,              // Lower priority for pixel task
        &xHandle,
        0               // Run on Core 0
    );

    // Enable core 0 watchdog
    esp_task_wdt_init(10, true);
}

void loop() {
    static uint32_t lastUpdate = 0;
    const uint32_t UPDATE_INTERVAL = 40; // 50 -> 20 FPS, 40 -> 25 FPS

    // Check if new data is ready for display
    if (xSemaphoreTake(displayMutex, 0) == pdTRUE) {
        if (dataReady && (millis() - lastUpdate >= UPDATE_INTERVAL)) {
            display.clearDisplay();

            // Draw grains as pixels
            for (int i = 0; i < N_GRAINS; i++) {
                int yPos = grain[i].pos / WIDTH;
                int xPos = grain[i].pos % WIDTH;
                display.drawPixel(xPos, yPos, WHITE);
            }

            // Update the display
            display.display();
            dataReady = false;
            lastUpdate = millis();

            // Debug output (rate-limited)
            static uint32_t lastPrint = 0;
            if (millis() - lastPrint >= 1000) {
                Serial.printf("FPS: %d\n", 1000 / UPDATE_INTERVAL);
                lastPrint = millis();
            }
        }
        xSemaphoreGive(displayMutex);
    }

    // Yield to other tasks (ensures FreeRTOS tasks can run)
    yield();
}
