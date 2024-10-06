// Sketch requires Adafruit MPU6050 and PrintEx library

// Basic demo for accelerometer readings from Adafruit MPU6050

#include <PrintEx.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified mag;

typedef struct {
  float x;
  float y;
  float z;
} V3;

static V3 v3_add(const V3 x, const V3 y) { return (V3){ x.x + y.x, x.y + y.y, x.z + y.z }; }
static V3 v3_sub(const V3 x, const V3 y) { return (V3){ x.x - y.x, x.y - y.y, x.z - y.z }; }
static V3 v3_mul(const V3 x, const V3 y) { return (V3){ x.x * y.x, x.y * y.y, x.z * y.z }; }
static V3 v3_div(const V3 x, const V3 y) { return (V3){ x.x / y.x, x.y / y.y, x.z / y.z }; }
static V3 v3_cross(const V3 x, const V3 y) { return (V3){ x.y*y.z-x.z*y.y, x.z*y.x-x.x*y.z, x.x*y.y-x.y*y.x }; }
static V3 v3_scale(const V3 x, float y) { return (V3){ x.x * y, x.y * y, x.z * y }; }
static V3 v3_norm(const V3 x) { return v3_scale(x, 1.0f / v3_len(x)); };
static V3 v3_norm_bias(const V3 x) { return v3_scale(x, 1.0f / (0.001f + v3_len(x))); };
static float v3_dot(const V3 x, const V3 y) { return x.x * y.x + x.y * y.y + x.z * y.z; };
static float v3_len(const V3 x) { return sqrtf(x.x * x.x + x.y * x.y + x.z * x.z); };
static float v3_len2(const V3 x) { return x.x * x.x + x.y * x.y + x.z * x.z; };
static float v3_dist(const V3 x, const V3 y) { return v3_len(v3_sub(x, y)); };
static float v3_dist2(const V3 x, const V3 y) { return v3_len2(v3_sub(x, y)); };

#define ACCEL_BUFFER_SIZE         8
#define ACCEL_GUESS_AGGRESSIVNESS 0.002f
#define ACCEL_GUESS_MAX_LEN       6.0f
#define ACCEL_ESTIMATED_GRAVITY   9.85f

StreamEx serial = Serial;
/*
accel calibrate -0.654,  0.070,  7.055
gyro  calibrate -0.026,  0.036, -0.020
accel guess     0.648,  0.576, -2.755
*/
const V3 accel_offset = {0.648,  0.576, -2.755};
const V3 gyro_offset = {-0.026,  0.036, -0.020};

V3 accel_data[ACCEL_BUFFER_SIZE];
float accel_progress = 0.0;
float linear_movement = 0.0;
float angular_movement = 0.0;
V3 accel_raw = {0, 0, 0};
V3 accel = {0, 0, 0};
V3 gyro_raw = {0, 0, 0};
V3 gyro = {0, 0, 0};
V3 accel_max = {0, 0, 0};
V3 accel_min = {0, 0, 0};
V3 accel_guess = {0, 0, 0};
V3 RX = {1, 0, 0};
V3 RY = {0, 1, 0};
V3 RZ = {0, 0, 1};
V3 magnet = {0, 0, 0};
int n = 0;
unsigned long next_tick;

void setup(void) {
  Serial.begin(115200);

  if (!mpu.begin()) {
    serial.printf("Failed to find MPU6050 chip\n");
    while (1) {}
  }
  if(!mag.begin()) {
    serial.printf("Failed to find HMC5883 chip\n");
    while (1) {}
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  mpu.setHighPassFilter(MPU6050_HIGHPASS_5_HZ);
  next_tick = micros() + 10000;
}

void write_accel(V3 data) {
  // Find the vector in accel_data that has the highest dot product with data when both vectors are normalized
  // This effectively finds the most "samey" vector (atleast in terms of direction) and replaces it
  if (v3_len(accel_data[0]) < 0.0001f) {
    for (int i = 0; i < ACCEL_BUFFER_SIZE; i++) {
      accel_data[i] = data;
    }
    return;
  }
  float highest_dot = -1.0f;
  int highest_index = 0;
  V3 data_norm = v3_norm_bias(v3_add(data, accel_guess));
  for (int i = 0; i < ACCEL_BUFFER_SIZE; i++) {
    V3 accel_norm = v3_norm_bias(accel_data[i]);
    float dot = v3_dot(accel_norm, data_norm);
    if (dot > highest_dot) {
      highest_dot = dot;
      highest_index = i;
    }
  }
  // Make sure data points aren't too close to each other
  if (highest_dot > 0.97) return;
  float old_dot = 0.0;
  float new_dot = 0.0;
  V3 old_norm = v3_norm_bias(accel_data[highest_index]);
  for (int i = 0; i < ACCEL_BUFFER_SIZE; i++) {
    if (i == highest_index) continue;
    V3 norm = v3_norm_bias(accel_data[i]);
    old_dot += v3_dot(old_norm, norm);
    new_dot += v3_dot(data_norm, norm);
  }
  if (new_dot > old_dot) return;
  accel_data[highest_index] = data;
}

void guess_accel() {
  // Adjust the guess vector
  V3 guess_adjust = {0,0,0};
  for (int i = 0; i < ACCEL_BUFFER_SIZE; i++) {
    V3 d = accel_data[i];
    float delta = powf(ACCEL_ESTIMATED_GRAVITY - v3_dist(d, accel_guess), 3.0f);
    guess_adjust.x -= delta * (d.x > 0.0f ? 1.0f : -1.0f);
    guess_adjust.y -= delta * (d.y > 0.0f ? 1.0f : -1.0f);
    guess_adjust.z -= delta * (d.z > 0.0f ? 1.0f : -1.0f);
  }
  accel_guess = v3_add(accel_guess, v3_scale(guess_adjust, ACCEL_GUESS_AGGRESSIVNESS));
  if (v3_len(accel_guess) > ACCEL_GUESS_MAX_LEN) {
    accel_guess = v3_scale(v3_norm(accel_guess), ACCEL_GUESS_MAX_LEN);
  }
}

void compute_linear_movement() {
  V3 diffmax = v3_sub(accel_raw, accel_max);
  V3 diffmin = v3_sub(accel_raw, accel_min);
  linear_movement = 
    fabsf(diffmax.x) + fabsf(diffmax.y) + fabsf(diffmax.z) + 
    fabsf(diffmin.x) + fabsf(diffmin.y) + fabsf(diffmin.z);
  accel_max = v3_add(v3_scale(v3_sub(accel_max, accel_raw), 0.986232704493), accel_raw); // Half-life of roughly 0.5 seconds
  accel_min = v3_add(v3_scale(v3_sub(accel_min, accel_raw), 0.986232704493), accel_raw);
}

void compute_angular_movement() {
  angular_movement = fabsf(gyro.x) + fabsf(gyro.y) + fabsf(gyro.z);
}

void compute_calibration_progress() {
  float total_dot = 0.0;
  V3 normalized[ACCEL_BUFFER_SIZE];
  for (int i = 0; i < ACCEL_BUFFER_SIZE; i++) {
    normalized[i] = v3_norm_bias(accel_data[i]);
  }
  for (int i = 0; i < ACCEL_BUFFER_SIZE; i++) {
    for (int j = i + 1; j < ACCEL_BUFFER_SIZE; j++) {
      total_dot += v3_dot(normalized[i], normalized[j]);
    }
  }
  accel_progress = 0.5-total_dot/(float)(ACCEL_BUFFER_SIZE * (ACCEL_BUFFER_SIZE-1));
}

void compute_angles() {
  float delta = 1.0 / 100.0f;
  float gx = gyro.x * delta;
  float gy = gyro.y * delta;
  float gz = gyro.z * delta;
  V3 newRX = {
    RX.x + RY.x * gz - RZ.x * gy,
    RX.y + RY.y * gz - RZ.y * gy,
    RX.z + RY.z * gz - RZ.z * gy
  };
  V3 newRY = {
    -RX.x * gz + RY.x + RZ.x * gx,
    -RX.y * gz + RY.y + RZ.y * gx,
    -RX.z * gz + RY.z + RZ.z * gx
  };
  V3 newRZ = {
    RX.x * gy - RY.x * gx + RZ.x,
    RX.y * gy - RY.y * gx + RZ.y,
    RX.z * gy - RY.z * gx + RZ.z
  };

  float error = v3_dot(newRX, newRY);

  V3 orthRX = v3_sub(newRX, v3_scale(newRY, error*0.5f));
  V3 orthRY = v3_sub(newRY, v3_scale(newRX, error*0.5f));
  V3 orthRZ = v3_cross(orthRX, orthRY);

  RX = v3_scale(orthRX, 0.5f*(3.0f-v3_len2(orthRX)));
  RY = v3_scale(orthRY, 0.5f*(3.0f-v3_len2(orthRY)));
  RZ = v3_scale(orthRZ, 0.5f*(3.0f-v3_len2(orthRZ)));
}

void print_stats() {
  serial.printf("accel   %6.3f, %6.3f, %6.3f  %6.3f\n", accel, v3_len(accel));
  serial.printf("accelr  %6.3f, %6.3f, %6.3f  %6.3f\n", accel_raw, v3_len(accel_raw));
  serial.printf("accel+  %6.3f, %6.3f, %6.3f\n", accel_max);
  serial.printf("accel-  %6.3f, %6.3f, %6.3f\n", accel_min);
  serial.printf("gyror   %6.3f, %6.3f, %6.3f\n", gyro_raw);
  serial.printf("gyro    %6.3f, %6.3f, %6.3f\n", gyro);
  serial.printf("accel guess    %6.3f, %6.3f, %6.3f\n", accel_guess);
  serial.printf("lin mov %6.3f\n", linear_movement);
  serial.printf("ang mov %6.3f\n", angular_movement);
  serial.printf("calibration progress %d%%\n", (int)(100.0f*accel_progress));
  serial.printf("RX  %6.3f, %6.3f, %6.3f\n", RX);
  serial.printf("RY  %6.3f, %6.3f, %6.3f\n", RY);
  serial.printf("RZ  %6.3f, %6.3f, %6.3f\n", RZ);
  serial.printf("magnet  %6.3f, %6.3f, %6.3f  %6.3f\n", magnet, v3_len(magnet));
  serial.printf("\n");
}

void read_accel() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accel_raw = {a.acceleration.x, a.acceleration.y, a.acceleration.z};
  gyro_raw = {g.gyro.x, g.gyro.y, g.gyro.z};

  accel_max.x = accel_raw.x > accel_max.x ? accel_raw.x : accel_max.x;
  accel_max.y = accel_raw.y > accel_max.y ? accel_raw.y : accel_max.y;
  accel_max.z = accel_raw.z > accel_max.z ? accel_raw.z : accel_max.z;

  accel_min.x = accel_raw.x < accel_min.x ? accel_raw.x : accel_min.x;
  accel_min.y = accel_raw.y < accel_min.y ? accel_raw.y : accel_min.y;
  accel_min.z = accel_raw.z < accel_min.z ? accel_raw.z : accel_min.z;

  accel = v3_sub(accel_raw, accel_guess);
  gyro = v3_sub(gyro_raw, gyro_offset);
}

void read_magnet() {
  sensors_event_t event;
  mag.getEvent(&event);
  magnet = { event.magnetic.x, event.magnetic.y, event.magnetic.z };
}

void loop() {
  if (micros() <= next_tick) return;
  next_tick += 10000;

  read_accel();
  read_magnet();

  if (linear_movement < 1.5f && angular_movement < 0.2f) {
    write_accel(accel_raw);
  }

  compute_angles();
  compute_linear_movement();
  compute_angular_movement();

  n ++;
  if (n % 50 == 0) {
    compute_calibration_progress();
    print_stats();
  }
  if (accel_progress > 0.15f) {
    guess_accel();
  }
}