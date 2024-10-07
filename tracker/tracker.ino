// Sketch requires Adafruit MPU6050 and PrintEx library

// Basic demo for accelerometer readings from Adafruit MPU6050

#include <PrintEx.h>
/*#include <QMC5883LCompass.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>*/
#include <Wire.h>

//static Adafruit_MPU6050 mpu;
//static QMC5883LCompass mag;
static StreamEx serial = Serial;

// How many bits in the fixed point format come after the decimal point
#define FIX_SHIFT 9
#define FIX_ONEF ((float)(1 << FIX_SHIFT))
#define FIX_ONE (1 << FIX_SHIFT)
#define FIX_HALF (1 << (FIX_SHIFT-1))

typedef int16_t fixed16;

typedef struct {
  fixed16 x;
  fixed16 y;
  fixed16 z;
} V3;

typedef struct {
  float x[3];
} V3_floats;

typedef struct {
  float x[4];
} V4_floats;

// Fixed point utils
static fixed16 to_fixed16(float value) { return value * FIX_ONEF;}
static float to_float(fixed16 value) { return value / FIX_ONEF;}
static fixed16 fixed_mul(fixed16 a, fixed16 b) { return ((int32_t)a * (int32_t)b) >> FIX_SHIFT; }
static fixed16 fixed_div(fixed16 a, fixed16 b) { return ((int32_t)a * (int32_t)FIX_ONE) / (int32_t)b; }

// Fixed point vector utils
static V3 v3_add(const V3 x, const V3 y) { return (V3){ x.x + y.x, x.y + y.y, x.z + y.z }; }
static V3 v3_sub(const V3 x, const V3 y) { return (V3){ x.x - y.x, x.y - y.y, x.z - y.z }; }
static V3 v3_mul(const V3 x, const V3 y) { return (V3){ fixed_mul(x.x, y.x), fixed_mul(x.y, y.y), fixed_mul(x.z, y.z) }; }
static V3 v3_div(const V3 x, const V3 y) { return (V3){ fixed_div(x.x, y.x), fixed_div(x.y, y.y), fixed_div(x.z, y.z) }; }
static V3 v3_cross(const V3 x, const V3 y) { return (V3){ (x.y*y.z-x.z*y.y) >> FIX_SHIFT, (x.z*y.x-x.x*y.z) >> FIX_SHIFT, (x.x*y.y-x.y*y.x) >> FIX_SHIFT }; }
static V3 v3_scale(const V3 x, fixed16 y) { return (V3){ fixed_mul(x.x, y), fixed_mul(x.y, y), fixed_mul(x.z, y) }; }
static V3 v3_norm(const V3 x) { return v3_scale(x, fixed_div(FIX_ONE, v3_len(x))); };
static V3 v3_norm_bias(const V3 x) { return v3_scale(x, fixed_div(FIX_ONE, (int32_t)1 + (int32_t)v3_len(x))); };
static fixed16 v3_dot(const V3 x, const V3 y) { return ((int32_t)x.x * (int32_t)y.x + (int32_t)x.y * (int32_t)y.y + (int32_t)x.z * (int32_t)y.z) >> FIX_SHIFT; };
static fixed16 v3_len(const V3 x) { return sqrtf(fabsf(((int32_t)x.x * (int32_t)x.x + (int32_t)x.y * (int32_t)x.y + (int32_t)x.z * (int32_t)x.z) / (FIX_ONEF*FIX_ONEF))) * FIX_ONEF; };
static fixed16 v3_len2(const V3 x) { return ((int32_t)x.x * (int32_t)x.x + (int32_t)x.y * (int32_t)x.y + (int32_t)x.z * (int32_t)x.z) >> FIX_SHIFT; };
static fixed16 v3_dist(const V3 x, const V3 y) { return v3_len(v3_sub(x, y)); };
static fixed16 v3_dist2(const V3 x, const V3 y) { return v3_len2(v3_sub(x, y)); };

// Fixed point printing utils
static V4_floats v3_to_floats_and_length(const V3 x) { return { to_float(x.x), to_float(x.y), to_float(x.z), to_float(v3_len(x)) }; }
static V3_floats v3_to_floats(const V3 x) { return { to_float(x.x), to_float(x.y), to_float(x.z) }; }

// Roguhly 0.5^(1/50)
#define HALF_LIFE_0_5             to_fixed16(0.986232704493f)

// How many entries to keep in the guess algorihm
#define ACCEL_BUFFER_SIZE         16

// Multiplier for the guess algorithm
#define ACCEL_GUESS_AGGRESSIVNESS to_fixed16(0.02f)

// Accelerometer guess vector will not exeed this magnitude
#define ACCEL_GUESS_MAX_LEN       to_fixed16(6.0f)

// Estimated gravity magnitude used by acceleration offset guessing algorihm
#define ACCEL_ESTIMATED_GRAVITY   to_fixed16(9.85f)

// If the dot product between a normalized acceleration and a normalized buffer vector is higher than
// this then don't bother writing the entry
#define ACCEL_GUESS_CLOSENESS     to_fixed16(0.97f)

/*
accel calibrate -0.654,  0.070,  7.055
gyro  calibrate -0.026,  0.036, -0.020
accel guess     0.648,  0.576, -2.755
*/
const static V3 accel_offset = {to_fixed16( 0.648),  to_fixed16(0.576), to_fixed16(-2.755)};
const static V3 gyro_offset =  {to_fixed16(-0.026),  to_fixed16(0.036), to_fixed16(-0.020)};

static V3 accel_data[ACCEL_BUFFER_SIZE];
static fixed16 accel_progress = 0;
static fixed16 linear_movement = 0;
static fixed16 angular_movement = 0;
static V3 accel_raw = {0, 0, 0};
static V3 accel = {0, 0, 0};
static V3 gyro_raw = {0, 0, 0};
static V3 gyro = {0, 0, 0};
static V3 accel_max = {0, 0, 0};
static V3 accel_min = {0, 0, 0};
static V3 accel_guess = {to_fixed16(0.648f), to_fixed16(-0.031), to_fixed16(-3.676f)};
static V3 RX = {FIX_ONE, 0, 0};
static V3 RY = {0, FIX_ONE, 0};
static V3 RZ = {0, 0, FIX_ONE};
static V3 magnet = {0, 0, 0};
static V3 magnet_max = {0, 0, 0};
static V3 magnet_min = {0, 0, 0};
static uint8_t n = 0;
static unsigned long next_tick;

#define I2C_ID_MPU6050  0x68
#define I2C_ID_QMC5883L 0x0D

#define I2C_ADDR_MPU6050_ACCEL  0x3B
#define I2C_ADDR_MPU6050_GYRO   0x43
#define I2C_ADDR_MPU6050_RESET  0x6B
#define I2C_ADDR_MPU6050_WHOAMI 0x75
#define I2C_ADDR_MPU6050_GYRO_CONFIG  0x1B
#define I2C_ADDR_MPU6050_ACCEL_CONFIG 0x1C

static int i2c_detect(int id) {
  Wire.beginTransmission(id);
  return Wire.endTransmission() == 0;
}

static V3 i2c_read_v3(int id, int address) {
  V3 result;
  Wire.beginTransmission(id);
  Wire.write(address);
  int error = Wire.endTransmission(false);
  Wire.requestFrom(id, 6, true);
  if (error != 0) {
    serial.printf("I2C error\n");
    delay(1000);
  }
  result.x = Wire.read() << 8;
  result.x |= Wire.read();
  result.y = Wire.read() << 8;
  result.y |= Wire.read();
  result.z = Wire.read() << 8;
  result.z |= Wire.read();
  return result;
}

static int i2c_read_byte(int id, int address) {
  Wire.beginTransmission(id);
  Wire.write(address);
  int error = Wire.endTransmission(false);
  Wire.requestFrom(id, 1, true);
  if (error != 0) {
    serial.printf("I2C error\n");
    delay(1000);
  }
  return Wire.read();
}

static void i2c_write_byte(int id, int address, int value) {
  Wire.beginTransmission(id);
  Wire.write(address);
  Wire.write(value);
  int error = Wire.endTransmission(false);
  if (error != 0) {
    serial.printf("I2C error\n");
    delay(1000);
  }
}

static void(*reset)() = 0;

static void init_mpu6050() {
  if (!i2c_detect(I2C_ID_MPU6050)) {
    serial.printf("Failed to find MPU6050 chip\n");
    delay(1000);
    reset();
  }
  i2c_write_byte(I2C_ID_MPU6050, I2C_ADDR_MPU6050_RESET, 0); // Reset the device
}

void setup(void) {
  Serial.begin(115200);
  Wire.begin();

  serial.printf("Init\n");

  init_mpu6050();

  if (!i2c_detect(I2C_ID_QMC5883L)) {
    serial.printf("Failed to find QMC5883L chip\n");
    delay(1000);
    reset();
  }

  /*if (!mpu.begin()) {
    serial.printf("Failed to find MPU6050 chip\n");
    while (1) {}
  }
  mag.init(); // Detect QMC5883L presence?

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  mpu.setHighPassFilter(MPU6050_HIGHPASS_5_HZ);*/
  next_tick = micros() + 10000;
  serial.printf("Done\n");
}

static void write_accel(V3 data) {
  // Find the vector in accel_data that has the highest dot product with data when both vectors are normalized
  // This effectively finds the most "samey" vector (atleast in terms of direction) and replaces it

  // If this is the first entry then override all vectors
  if (v3_len(accel_data[0]) < 1) {
    for (uint8_t i = 0; i < ACCEL_BUFFER_SIZE; i++) {
      accel_data[i] = data;
    }
    return;
  }
  fixed16 highest_dot = -FIX_ONE;
  uint8_t highest_index = 0;
  V3 data_norm = v3_norm_bias(v3_add(data, accel_guess));
  for (uint8_t i = 0; i < ACCEL_BUFFER_SIZE; i++) {
    V3 accel_norm = v3_norm_bias(accel_data[i]);
    fixed16 dot = v3_dot(accel_norm, data_norm);
    // Make sure data points aren't too close to each other
    if (highest_dot > ACCEL_GUESS_CLOSENESS) return;
    if (dot > highest_dot) {
      highest_dot = dot;
      highest_index = i;
    }
  }
  fixed16 old_dot = 0;
  fixed16 new_dot = 0;
  V3 old_norm = v3_norm_bias(accel_data[highest_index]);
  for (uint8_t i = 0; i < ACCEL_BUFFER_SIZE; i++) {
    if (i == highest_index) continue;
    V3 norm = v3_norm_bias(accel_data[i]);
    old_dot += v3_dot(old_norm, norm);
    new_dot += v3_dot(data_norm, norm);
  }
  if (new_dot > old_dot) return;
  accel_data[highest_index] = data;
}

static void guess_accel() {
  // Adjust the guess vector
  V3 guess_adjust = { 0, 0, 0 };
  for (uint8_t i = 0; i < ACCEL_BUFFER_SIZE; i++) {
    V3 d = accel_data[i];
    fixed16 delta = fixed_mul(ACCEL_ESTIMATED_GRAVITY - v3_dist(d, accel_guess), ACCEL_GUESS_AGGRESSIVNESS);
    guess_adjust.x -= delta * (d.x > 0 ? FIX_ONE : -FIX_ONE);
    guess_adjust.y -= delta * (d.y > 0 ? FIX_ONE : -FIX_ONE);
    guess_adjust.z -= delta * (d.z > 0 ? FIX_ONE : -FIX_ONE);
  }
  serial.printf("guess adjust %6.3f %6.3f %6.3f\n", v3_to_floats(accel_guess));
  accel_guess = v3_add(accel_guess, guess_adjust);
  if (v3_len(accel_guess) > ACCEL_GUESS_MAX_LEN) {
    accel_guess = v3_scale(v3_norm(accel_guess), ACCEL_GUESS_MAX_LEN);
  }
}

static void compute_linear_movement() {
  V3 diffmax = v3_sub(accel_raw, accel_max);
  V3 diffmin = v3_sub(accel_raw, accel_min);
  linear_movement = 
    fabsf(diffmax.x) + fabsf(diffmax.y) + fabsf(diffmax.z) + 
    fabsf(diffmin.x) + fabsf(diffmin.y) + fabsf(diffmin.z);
  //accel_max = v3_add(v3_scale(v3_sub(accel_max, accel_raw), HALF_LIFE_0_5), accel_raw);
  //accel_min = v3_add(v3_scale(v3_sub(accel_min, accel_raw), HALF_LIFE_0_5), accel_raw);
}

static void compute_angular_movement() {
  angular_movement = fabsf(gyro.x) + fabsf(gyro.y) + fabsf(gyro.z);
}

static V3 normalized[ACCEL_BUFFER_SIZE];

static void compute_calibration_progress() {
  fixed16 total_dot = 0;
  for (uint8_t i = 0; i < ACCEL_BUFFER_SIZE; i++) {
    normalized[i] = v3_norm_bias(accel_data[i]);
  }
  // TODO: This breaks everything
  /*for (uint8_t i = 0; i < ACCEL_BUFFER_SIZE; i++) {
    for (uint8_t j = i + 1; j < ACCEL_BUFFER_SIZE; j++) {
      total_dot += v3_dot(normalized[i], normalized[j]);
    }
  }*/
  accel_progress = FIX_HALF-fixed_div(total_dot, fixed_mul(to_fixed16(ACCEL_BUFFER_SIZE), to_fixed16(ACCEL_BUFFER_SIZE-1)));
}

// https://web.archive.org/web/20120602094634/http://gentlenav.googlecode.com/files/DCMDraft2.pdf
static void compute_angles() {
  fixed16 delta = fixed_div(FIX_ONE, to_fixed16(100.0f));
  fixed16 gx = fixed_mul(gyro.x, delta);
  fixed16 gy = fixed_mul(gyro.y, delta);
  fixed16 gz = fixed_mul(gyro.z, delta);
  V3 newRX = {
    RX.x + (RY.x * gz - RZ.x * gy) >> FIX_SHIFT,
    RX.y + (RY.y * gz - RZ.y * gy) >> FIX_SHIFT,
    RX.z + (RY.z * gz - RZ.z * gy) >> FIX_SHIFT
  };
  V3 newRY = {
    RY.x + (RZ.x * gx - RX.x * gz) >> FIX_SHIFT,
    RY.y + (RZ.y * gx - RX.y * gz) >> FIX_SHIFT,
    RY.z + (RZ.z * gx - RX.z * gz) >> FIX_SHIFT
  };
  V3 newRZ = {
    RZ.x + (RX.x * gy - RY.x * gx) >> FIX_SHIFT,
    RZ.y + (RX.y * gy - RY.y * gx) >> FIX_SHIFT,
    RZ.z + (RX.z * gy - RY.z * gx) >> FIX_SHIFT
  };

  fixed16 error = v3_dot(newRX, newRY);

  V3 orthRX = v3_sub(newRX, v3_scale(newRY, error >> 1));
  V3 orthRY = v3_sub(newRY, v3_scale(newRX, error >> 1));
  V3 orthRZ = v3_cross(orthRX, orthRY);

  RX = v3_scale(orthRX, (to_fixed16(3.0f) - v3_len2(orthRX)) >> 1);
  RY = v3_scale(orthRY, (to_fixed16(3.0f) - v3_len2(orthRY)) >> 1);
  RZ = v3_scale(orthRZ, (to_fixed16(3.0f) - v3_len2(orthRZ)) >> 1);
}

/*

accel    2.766, -0.406, -41.875  41.967
accelr   2.766, -0.406, -41.875
accel+  40.375, 38.117, 26.750
accel-  -34.992, -38.672, -64.000
gyror   -0.361,  0.506, -0.277

*/

static void print_stats() {
  serial.printf("accel   %6.3f, %6.3f, %6.3f  %6.3f\n", v3_to_floats_and_length(accel));
  serial.printf("accelr  %6.3f, %6.3f, %6.3f\n", v3_to_floats(accel_raw));
  serial.printf("accel+  %6.3f, %6.3f, %6.3f\n", v3_to_floats(accel_max));
  serial.printf("accel-  %6.3f, %6.3f, %6.3f\n", v3_to_floats(accel_min));
  serial.printf("accel guess    %6.3f, %6.3f, %6.3f\n", v3_to_floats(accel_guess));
  serial.printf("gyror   %6.3f, %6.3f, %6.3f\n", v3_to_floats(gyro_raw));
  serial.printf("gyro    %6.3f, %6.3f, %6.3f\n", v3_to_floats(gyro));
  /*serial.printf("lin mov %6.3f\n", to_float(linear_movement));
  serial.printf("ang mov %6.3f\n", to_float(angular_movement));
  serial.printf("calibration progress %d%%\n", (int)(100.0f*to_float(accel_progress)));
  serial.printf("RX  %6.3f, %6.3f, %6.3f\n", v3_to_floats(RX));
  serial.printf("RY  %6.3f, %6.3f, %6.3f\n", v3_to_floats(RY));
  serial.printf("RZ  %6.3f, %6.3f, %6.3f\n", v3_to_floats(RZ));
  serial.printf("magnet  %6.3f, %6.3f, %6.3f  %6.3f\n", v3_to_floats_and_length(magnet));
  serial.printf("magnet+ %6.3f, %6.3f, %6.3f\n", v3_to_floats(magnet_max));
  serial.printf("magnet- %6.3f, %6.3f, %6.3f\n", v3_to_floats(magnet_min));*/
  serial.printf("\n");
}

static void read_accel() {
  static uint32_t stable_frames = 0;

  accel_raw = i2c_read_v3(I2C_ID_MPU6050, I2C_ADDR_MPU6050_ACCEL);
  gyro_raw = i2c_read_v3(I2C_ID_MPU6050, I2C_ADDR_MPU6050_GYRO);

  accel_raw = v3_scale(accel_raw, to_fixed16(FIX_ONEF * 9.85f * 2.0f/32768.0f));
  gyro_raw = v3_scale(gyro_raw, to_fixed16(FIX_ONEF * 0.01745329f * 250.0f/32768.0f));

  accel = v3_sub(accel_raw, accel_guess);
  gyro = v3_sub(gyro_raw, gyro_offset);

  if (v3_len(gyro) < to_fixed16(0.05)) {
    stable_frames ++;
  } else {
    stable_frames = 0;
  }
  if (stable_frames > 3) {
    accel_max.x = max(accel_raw.x, accel_max.x);
    accel_max.y = max(accel_raw.y, accel_max.y);
    accel_max.z = max(accel_raw.z, accel_max.z);

    accel_min.x = min(accel_raw.x, accel_min.x);
    accel_min.y = min(accel_raw.y, accel_min.y);
    accel_min.z = min(accel_raw.z, accel_min.z);

    if (accel_max.x > 0 && accel_max.y > 0 && accel_max.z > 0 &&
        accel_min.x < 0 && accel_min.y < 0 && accel_min.z < 0) {
        accel_guess = v3_scale(v3_add(accel_max, accel_min), to_fixed16(0.5f));
    }
  }
}

static void read_magnet() {

}

void loop() {
  unsigned long t = micros();
  if (t <= next_tick) return;
  next_tick += 10000;

  read_accel();
  read_magnet();

  compute_angles();
  compute_angular_movement();

  n ++;
  if (n % 50 == 0) {
    print_stats();
  }
}