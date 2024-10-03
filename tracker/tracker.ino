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
static inline float v3_dot(const V3 x, const V3 y) { return x.x * y.x + x.y * y.y + x.z * y.z; };
static inline float v3_len(const V3 x) { return sqrtf(x.x * x.x + x.y * x.y + x.z * x.z); };
static inline float v3_len2(const V3 x) { return x.x * x.x + x.y * x.y + x.z * x.z; };
static inline float v3_dist(const V3 x, const V3 y) { return v3_len(v3_sub(x, y)); };
static inline float v3_dist2(const V3 x, const V3 y) { return v3_len2(v3_sub(x, y)); };

StreamEx serial = Serial;

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

/*
accel calibrate -0.654,  0.070,  7.055
gyro  calibrate -0.026,  0.036, -0.020
*/

const V3 accel_offset = {-0.654,  0.070,  7.055 - 9.805};
const V3 gyro_offset = {-0.026,  0.036, -0.020};

V3 accel_total = {0, 0, 0};
V3 gyro_total = {0, 0, 0};
float num_samples = 0;
int n = 0;

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  V3 accel = {a.acceleration.x, a.acceleration.y, a.acceleration.z};
  V3 gyro = {g.gyro.x, g.gyro.y, g.gyro.z};

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

    // Give some sort of half-life to calibration data
    num_samples *= 0.5;
    accel_total = v3_scale(accel_total, 0.5);
    gyro_total = v3_scale(gyro_total, 0.5);
    Serial.println("");
  }
  delay(10);
}