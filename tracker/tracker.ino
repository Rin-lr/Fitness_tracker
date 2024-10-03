// Sketch requires Adafruit MPU6050 and PrintEx library

// Basic demo for accelerometer readings from Adafruit MPU6050

#include <PrintEx.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

typedef struct {
  float x;
  float y;
  float z;
} V3;

static inline V3 v3_add(const V3 x, const V3 y) { return (V3){ x.x + y.x, x.y + y.y, x.z + y.z }; }
static inline V3 v3_sub(const V3 x, const V3 y) { return (V3){ x.x - y.x, x.y - y.y, x.z - y.z }; }
static inline V3 v3_mul(const V3 x, const V3 y) { return (V3){ x.x * y.x, x.y * y.y, x.z * y.z }; }
static inline V3 v3_div(const V3 x, const V3 y) { return (V3){ x.x / y.x, x.y / y.y, x.z / y.z }; }
static inline V3 v3_scale(const V3 x, float y) { return (V3){ x.x * y, x.y * y, x.z * y }; }
static inline V3 v3_norm(const V3 x) { return v3_scale(x, 1.0f / v3_len(x)); };
static inline V3 v3_norm_bias(const V3 x) { return v3_scale(x, 1.0f / (0.001f + v3_len(x))); };
static inline float v3_dot(const V3 x, const V3 y) { return x.x * y.x + x.y * y.y + x.z * y.z; };
static inline float v3_len(const V3 x) { return sqrtf(x.x * x.x + x.y * x.y + x.z * x.z); };
static inline float v3_len2(const V3 x) { return x.x * x.x + x.y * x.y + x.z * x.z; };
static inline float v3_dist(const V3 x, const V3 y) { return v3_len(v3_sub(x, y)); };
static inline float v3_dist2(const V3 x, const V3 y) { return v3_len2(v3_sub(x, y)); };

#define ACCEL_BUFFER_SIZE         16
#define ACCEL_GUESS_AGGRESSIVNESS 0.02f
#define ACCEL_GUESS_MAX_LEN       6.0f
#define ACCEL_ESTIMATED_GRAVITY   9.81f

StreamEx serial = Serial;
/*
accel calibrate -0.654,  0.070,  7.055
gyro  calibrate -0.026,  0.036, -0.020
accel guess     0.648,  0.576, -2.755
*/
const V3 accel_offset = {0.648,  0.576, -2.755};
const V3 gyro_offset = {-0.026,  0.036, -0.020};

V3 accel_data[ACCEL_BUFFER_SIZE];
float accel_fullness = 0.0;
V3 accel_guess = {0, 0, 0};
V3 accel_total = {0, 0, 0};
V3 gyro_total = {0, 0, 0};
float num_samples = 0;
int n = 0;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  serial.printf("Adafruit MPU6050 test!\n");

  // Try to initialize!
  if (!mpu.begin()) {
    serial.printf("Failed to find MPU6050 chip\n");
    while (1) {
      delay(10);
    }
  }
  serial.println("MPU6050 Found!\n");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  mpu.setHighPassFilter(MPU6050_HIGHPASS_5_HZ);
  delay(100);
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
  if (highest_dot > 0.9) return;
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
    float delta = ACCEL_GUESS_AGGRESSIVNESS * (ACCEL_ESTIMATED_GRAVITY - v3_len(v3_sub(d, accel_guess)));
    guess_adjust.x -= delta * (d.x > 0.0f ? 1.0f : -1.0f);
    guess_adjust.y -= delta * (d.y > 0.0f ? 1.0f : -1.0f);
    guess_adjust.z -= delta * (d.z > 0.0f ? 1.0f : -1.0f);
  }
  accel_guess = v3_add(accel_guess, guess_adjust);
  if (v3_len(accel_guess) > ACCEL_GUESS_MAX_LEN) {
    accel_guess = v3_scale(v3_norm(accel_guess), ACCEL_GUESS_MAX_LEN);
  }
}

void print_fullness() {
  float total_dot = 0.0;
  for (int i = 0; i < ACCEL_BUFFER_SIZE; i++) {
    for (int j = i + 1; j < ACCEL_BUFFER_SIZE; j++) {
      total_dot += 0.5f - v3_dot(v3_norm_bias(accel_data[i]), v3_norm_bias(accel_data[j])) * 0.5f;
    }
  }
  accel_fullness = total_dot/(float)(ACCEL_BUFFER_SIZE * (ACCEL_BUFFER_SIZE-1) * 0.5f);
  serial.printf("fullness %10.3f %10.3f\n", total_dot/(float)(ACCEL_BUFFER_SIZE * (ACCEL_BUFFER_SIZE-1) * 0.5f), total_dot);
}

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  V3 accel = {a.acceleration.x, a.acceleration.y, a.acceleration.z};
  V3 gyro = {g.gyro.x, g.gyro.y, g.gyro.z};

  write_accel(accel);

  accel_total = v3_add(accel_total, accel);
  gyro_total = v3_add(gyro_total, gyro);
  num_samples += 1.0f;

  accel = v3_sub(accel, accel_offset);
  gyro = v3_sub(gyro, gyro_offset);

  n ++;
  if (n % 100 == 0) {
    serial.printf("accel %6.3f, %6.3f, %6.3f  %6.3f\n", accel, v3_len(accel));
    serial.printf("gyro  %6.3f, %6.3f, %6.3f\n", gyro);

    serial.printf("accel calibrate %6.3f, %6.3f, %6.3f\n", v3_scale(accel_total, 1.0f/num_samples));
    serial.printf("gyro  calibrate %6.3f, %6.3f, %6.3f\n", v3_scale(gyro_total, 1.0f/num_samples));
    serial.printf("accel guess     %6.3f, %6.3f, %6.3f\n", accel_guess);

    print_fullness();

    // Give some sort of half-life to calibration data
    num_samples *= 0.5;
    accel_total = v3_scale(accel_total, 0.5);
    gyro_total = v3_scale(gyro_total, 0.5);
    Serial.println("");
  }
  if (accel_fullness > 0.1f) {
    guess_accel();
  }
  delay(10);
}