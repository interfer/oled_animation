#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu_setup.h"

#define GRAINS_NUM         50

#define SCREEN_WIDTH       128
#define SCREEN_HEIGHT      64
#define OLED_RESET         -1
#define UPDATE_INTERVAL    40
#define MOVEMENT_THRESHOLD 0.6

// Physics Parameters X(y),Y(z)
#define AXIS_X             y
#define AXIS_Y             z
#define GRAVITY_SCALE      0.6  // Adjust gravity impact
#define DEFAULT_FRICTION   0.95 // Default grain friction
#define DAMPING_FACTOR     0.5
#define COLLISION_PUSH     1    // Push intensity for overlapping grains
#define GRAIN_MIN_SIZE     2
#define GRAIN_MAX_SIZE     2
#define GRAIN_SIZE         2

struct Grain {
    int16_t x, y;
    float vx, vy;
    uint8_t size;
    float friction;
};

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_MPU6050 mpu;

Grain grains[GRAINS_NUM];
float baseX = 0, baseY = 0;
// Gravity variables (dynamic values to be updated based on tilt)
float gravityX = GRAVITY_SCALE;
float gravityY = GRAVITY_SCALE;

SemaphoreHandle_t displayMutex;
volatile bool dataReady = false;

// Update gravity based on accelerometer (dummy example, replace with real sensor readings)
void updateGravity() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Normalize accelerometer values for gravity scaling
    gravityX = (GRAVITY_SCALE * (a.acceleration.AXIS_X - baseX));
    gravityY = -(GRAVITY_SCALE * (a.acceleration.AXIS_Y - baseY));
}

void calibrateSensor() {
    int calibration_factor = 20;
    sensors_event_t a, g, temp;
    baseX = baseY = 0;
    for (int i = 0; i < calibration_factor; i++) {
        mpu.getEvent(&a, &g, &temp);
        baseX += a.acceleration.AXIS_X;
        baseY += -a.acceleration.AXIS_Y;
        delay(10);
    }
    baseX /= calibration_factor;
    baseY /= calibration_factor;
}

void initializeGrains() {
    for (int i = 0; i < GRAINS_NUM; i++) {
        bool placed;
        do {
            placed = true;
            grains[i].x = random(0, SCREEN_WIDTH);
            grains[i].y = random(0, SCREEN_HEIGHT);
            // grains[i].size = random(GRAIN_MIN_SIZE, GRAIN_MAX_SIZE);
            grains[i].size = GRAIN_SIZE;
            grains[i].vx = 0;
            grains[i].vy = 0;
            grains[i].friction = DEFAULT_FRICTION;

            for (int j = 0; j < i; j++) {
                int dx = abs(grains[i].x - grains[j].x);
                int dy = abs(grains[i].y - grains[j].y);
                if (dx < (grains[i].size + grains[j].size) &&
                    dy < (grains[i].size + grains[j].size)) {
                    placed = false;
                    break;
                }
            }
        } while (!placed);
    }
}

void resolveCollision(Grain &g1, Grain &g2) {
    float dx = g1.x - g2.x;
    float dy = g1.y - g2.y;
    float distance = sqrt(dx * dx + dy * dy);
    float minDist = (g1.size + g2.size);

    if (distance < minDist) {
        // Vertical stacking priority
        if (abs(dy) > abs(dx)) {
            // If vertical overlap is more significant, stack vertically
            if (g1.y > g2.y) {
                g1.y = g2.y + minDist;
            } else {
                g2.y = g1.y + minDist;
            }
            // Zero out vertical velocity to simulate resting
            g1.vy = 0;
            g2.vy = 0;
        } else {
            // Existing horizontal collision logic
            float push = COLLISION_PUSH * (minDist - distance) / minDist;
            g1.x += push * (dx / distance);
            g1.y += push * (dy / distance);
            g2.x -= push * (dx / distance);
            g2.y -= push * (dy / distance);
        }

        // Dampen velocities
        g1.vx *= DAMPING_FACTOR;
        g1.vy *= DAMPING_FACTOR;
        g2.vx *= DAMPING_FACTOR;
        g2.vy *= DAMPING_FACTOR;
    }
}

// Update physics with wall collision and gravity
void updatePhysics(Grain &grain) {
    // Apply gravity
    grain.vx += gravityX * 0.2; 
    grain.vy += gravityY * 0.2;

    // Apply friction
    grain.vx *= grain.friction;
    grain.vy *= grain.friction;

    // Update position
    grain.x += grain.vx;
    grain.y += grain.vy;

    // Corner protection with explicit corner zone handling
    // Left edge
    if (grain.x < 0) {
        grain.x = 0;
        grain.vx = abs(grain.vx) * 0.5;
    } 
    // Right edge
    else if (grain.x + grain.size > SCREEN_WIDTH) {
        grain.x = SCREEN_WIDTH - grain.size;
        grain.vx = -abs(grain.vx) * 0.5;
    }

    // Top edge
    if (grain.y < 0) {
        grain.y = 0;
        grain.vy = abs(grain.vy) * 0.5;
    }
    // Bottom edge
    else if (grain.y + grain.size > SCREEN_HEIGHT) {
        grain.y = SCREEN_HEIGHT - grain.size;
        grain.vy = -abs(grain.vy) * 0.5;
    }

    // Additional corner stabilization
    // Top-left corner
    if (grain.x < 0 && grain.y < 0) {
        grain.x = 0;
        grain.y = 0;
        grain.vx = max(0.0f, grain.vx);
        grain.vy = max(0.0f, grain.vy);
    }
    // Top-right corner
    else if (grain.x + grain.size > SCREEN_WIDTH && grain.y < 0) {
        grain.x = SCREEN_WIDTH - grain.size;
        grain.y = 0;
        grain.vx = min(0.0f, grain.vx);
        grain.vy = max(0.0f, grain.vy);
    }
    // Bottom-left corner
    else if (grain.x < 0 && grain.y + grain.size > SCREEN_HEIGHT) {
        grain.x = 0;
        grain.y = SCREEN_HEIGHT - grain.size;
        grain.vx = max(0.0f, grain.vx);
        grain.vy = min(0.0f, grain.vy);
    }
    // Bottom-right corner
    else if (grain.x + grain.size > SCREEN_WIDTH && grain.y + grain.size > SCREEN_HEIGHT) {
        grain.x = SCREEN_WIDTH - grain.size;
        grain.y = SCREEN_HEIGHT - grain.size;
        grain.vx = min(0.0f, grain.vx);
        grain.vy = min(0.0f, grain.vy);
    }
}

void pixelTask(void *param) {
    MPU_setup(&mpu);
    calibrateSensor();

    while (true) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        updateGravity();

        // Update physics for all grains
        for (int i = 0; i < GRAINS_NUM; i++) {
            updatePhysics(grains[i]);
        }

        // Handle collisions between grains
        for (int i = 0; i < GRAINS_NUM - 1; i++) {
            for (int j = i + 1; j < GRAINS_NUM; j++) {
                resolveCollision(grains[i], grains[j]);
            }
        }

        xSemaphoreTake(displayMutex, portMAX_DELAY);
        dataReady = true;
        xSemaphoreGive(displayMutex);

        vTaskDelay(pdMS_TO_TICKS(UPDATE_INTERVAL));
    }
}

void setup() {
    delay(1000);
    Serial.begin(115200);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("OLED initialization failed!");
        while (true) delay(10);
    }

    initializeGrains();
    displayMutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(pixelTask, "PixelTask", 8192, NULL, 1, NULL, 0);
}

void loop() {
    static uint32_t lastUpdate = 0;

    if (xSemaphoreTake(displayMutex, 0) == pdTRUE) {
        if (dataReady && (millis() - lastUpdate >= UPDATE_INTERVAL)) {
            display.clearDisplay();
            for (int i = 0; i < GRAINS_NUM; i++) {
                int x = grains[i].x;
                int y = grains[i].y;
                for (int dx = 0; dx < grains[i].size; dx++) {
                    for (int dy = 0; dy < grains[i].size; dy++) {
                        if (x + dx < SCREEN_WIDTH && y + dy < SCREEN_HEIGHT)
                            display.drawPixel(x + dx, y + dy, WHITE);
                    }
                }
            }
            display.display();
            dataReady = false;

            lastUpdate = millis();
        }
        xSemaphoreGive(displayMutex);
    }
    yield();
}
