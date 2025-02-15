#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define NUM_GRAINS 100

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_MPU6050 mpu;

struct Grain {
    float x, y;  // Position
    float vx, vy; // Velocity
    int size;    // Grain size (diameter in pixels)
};

Grain grains[NUM_GRAINS];
float ax = 0.0, ay = 0.0;

// Initialize grains with random positions, zero velocity, and random sizes
void initializeGrains() {
    for (int i = 0; i < NUM_GRAINS; i++) {
        grains[i].x = random(0, SCREEN_WIDTH);
        grains[i].y = random(0, SCREEN_HEIGHT);
        grains[i].vx = 0;
        grains[i].vy = 0;
        grains[i].size = random(2, 5); // Grain size between 2 and 4 pixels
    }
}

// Update the accelerometer readings
void updateGravity() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    ax = a.acceleration.y;
    ay = -a.acceleration.z;
}

// Update the physics for each grain
void updateGrains() {
    for (int i = 0; i < NUM_GRAINS; i++) {
        grains[i].vx += ax * 0.1; // Adjust sensitivity
        grains[i].vy += ay * 0.1;

        grains[i].x += grains[i].vx;
        grains[i].y += grains[i].vy;

        // Wall collisions
        if (grains[i].x < 0) {
            grains[i].x = 0;
            grains[i].vx = 0;
        }
        if (grains[i].x >= SCREEN_WIDTH - grains[i].size) {
            grains[i].x = SCREEN_WIDTH - grains[i].size - 1;
            grains[i].vx = 0;
        }
        if (grains[i].y < 0) {
            grains[i].y = 0;
            grains[i].vy = 0;
        }
        if (grains[i].y >= SCREEN_HEIGHT - grains[i].size) {
            grains[i].y = SCREEN_HEIGHT - grains[i].size - 1;
            grains[i].vy = 0;
        }

        // Collision with other grains
        for (int j = 0; j < NUM_GRAINS; j++) {
            if (i == j) continue;

            float dx = grains[i].x - grains[j].x;
            float dy = grains[i].y - grains[j].y;
            float distance = sqrt(dx * dx + dy * dy);
            float minDist = (grains[i].size + grains[j].size) / 2;

            if (distance < minDist) {
                // Simple collision response: repel grains
                float overlap = minDist - distance;
                grains[i].x += (dx / distance) * overlap / 2;
                grains[i].y += (dy / distance) * overlap / 2;
                grains[j].x -= (dx / distance) * overlap / 2;
                grains[j].y -= (dy / distance) * overlap / 2;

                // Damp velocity after collision
                grains[i].vx *= 0.8;
                grains[i].vy *= 0.8;
                grains[j].vx *= 0.8;
                grains[j].vy *= 0.8;
            }
        }
    }
}

// Render the grains to the display
void renderGrains() {
    display.clearDisplay();
    for (int i = 0; i < NUM_GRAINS; i++) {
        display.fillCircle(grains[i].x, grains[i].y, grains[i].size / 2, WHITE);
    }
    display.display();
}

void setup() {
    Serial.begin(115200);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("Failed to initialize OLED!");
        while (true);
    }
    if (!mpu.begin()) {
        Serial.println("Failed to initialize MPU6050!");
        while (true);
    }
    initializeGrains();
    display.clearDisplay();
    display.display();
}

void loop() {
    updateGravity();
    updateGrains();
    renderGrains();
    delay(20); // Slow down simulation for smoother rendering
}
