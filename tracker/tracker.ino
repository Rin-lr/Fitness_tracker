// Sketch requires Adafruit MPU6050 and PrintEx library

// Basic demo for accelerometer readings from Adafruit MPU6050

#include <PrintEx.h>
#include <Wire.h>

static StreamEx serial = Serial;

// How many bits in the fixed point format come after the decimal point
#define FIX_SHIFT 9
#define FIX_ONEF ((float)(1 << FIX_SHIFT))
#define FIX_ONE (1 << FIX_SHIFT)
#define FIX_HALF (1 << (FIX_SHIFT-1))
#define FIX_MIN (-32768)
#define FIX_MAX (32767)

typedef int16_t fixed16;

typedef struct {
  fixed16 x;
  fixed16 y;
  fixed16 z;
} V3;

typedef struct {
  float x;
  float y;
  float z;
} V3f;

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

//

static V3f v3f_add(const V3f x, const V3f y) { return (V3f){ x.x + y.x, x.y + y.y, x.z + y.z }; }
static V3f v3f_sub(const V3f x, const V3f y) { return (V3f){ x.x - y.x, x.y - y.y, x.z - y.z }; }
static V3f v3f_mul(const V3f x, const V3f y) { return (V3f){ x.x * y.x, x.y * y.y, x.z * y.z }; }
static V3f v3f_div(const V3f x, const V3f y) { return (V3f){ x.x / y.x, x.y / y.y, x.z / y.z }; }
static V3f v3f_cross(const V3f x, const V3f y) { return (V3f){ x.y*y.z-x.z*y.y, x.z*y.x-x.x*y.z, x.x*y.y-x.y*y.x }; }
static V3f v3f_scale(const V3f x, float y) { return (V3f){ x.x * y, x.y * y, x.z * y }; }
static float v3f_dot(const V3f x, const V3f y) { return x.x * y.x + x.y * y.y + x.z * y.z; };
static float v3f_len(const V3f x) { return sqrtf(x.x * x.x + x.y * x.y + x.z * x.z); };
static float v3f_len2(const V3f x) { return x.x * x.x + x.y * x.y + x.z * x.z; };
static float v3f_dist(const V3f x, const V3f y) { return v3f_len(v3f_sub(x, y)); };
static float v3f_dist2(const V3f x, const V3f y) { return v3f_len2(v3f_sub(x, y)); };

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

static fixed16 accel_progress = 0;
static fixed16 linear_movement = 0;
static fixed16 angular_movement = 0;
static V3 accel_raw = {0, 0, 0};
static V3 accel = {0, 0, 0};
static V3 accel_max = {0, 0, 0};
static V3 accel_min = {0, 0, 0};
static V3 accel_guess = {to_fixed16(0.648f), to_fixed16(-0.031), to_fixed16(-3.676f)};
static V3 gyro_raw = {0, 0, 0};
static V3 gyro = {0, 0, 0};
static V3f RX = {1, 0, 0};
static V3f RY = {0, 1, 0};
static V3f RZ = {0, 0, 1};
static V3 magnet_raw = {0, 0, 0};
static V3 magnet = {0, 0, 0};
static V3 magnet_max = {FIX_MIN, FIX_MIN, FIX_MIN};
static V3 magnet_min = {FIX_MAX, FIX_MAX, FIX_MAX};
static V3 magnet_guess = {0, 0, 0};
static uint8_t n = 0;
static uint32_t next_tick; // TODO: Upgrade to 64 bit?

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

static V3 i2c_read_v3_rev(int id, int address) {
  V3 result;
  Wire.beginTransmission(id);
  Wire.write(address);
  int error = Wire.endTransmission(false);
  Wire.requestFrom(id, 6, true);
  if (error != 0) {
    serial.printf("I2C error\n");
    delay(1000);
  }
  result.x = Wire.read();
  result.x |= Wire.read() << 8;
  result.y = Wire.read();
  result.y |= Wire.read() << 8;
  result.z = Wire.read();
  result.z |= Wire.read() << 8;
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

#define I2C_ID_MPU6050  0x68
#define I2C_ADDR_MPU6050_ACCEL  0x3B
#define I2C_ADDR_MPU6050_GYRO   0x43
#define I2C_ADDR_MPU6050_RESET  0x6B
#define I2C_ADDR_MPU6050_WHOAMI 0x75
#define I2C_ADDR_MPU6050_GYRO_CONFIG  0x1B
#define I2C_ADDR_MPU6050_ACCEL_CONFIG 0x1C

//https://nettigo.pl/attachments/440
#define I2C_ID_QMC5883L 0x0D
#define I2C_ADDR_QMC5883L_MAGNET 0x00
#define I2C_ADDR_QMC5883L_CONFIG 0x09

static void init_mpu6050() {
  if (!i2c_detect(I2C_ID_MPU6050)) {
    serial.printf("Failed to find MPU6050 chip\n");
    delay(1000);
    reset();
  }
  i2c_write_byte(I2C_ID_MPU6050, I2C_ADDR_MPU6050_RESET, 0); // Reset the device
}

static void init_qmc5883l() {
  if (!i2c_detect(I2C_ID_QMC5883L)) {
    serial.printf("Failed to find QMC5883L chip\n");
    delay(1000);
    reset();
  }
  // Highest oversampling, 2 gauss, continious mode, 200 Hz
  i2c_write_byte(I2C_ID_QMC5883L, I2C_ADDR_QMC5883L_CONFIG, 0b00001101);
}

void setup(void) {
  Serial.begin(115200);
  Wire.begin();

  serial.printf("Init\n");

  init_mpu6050();
  init_qmc5883l();

  next_tick = micros() + 10000;
  serial.printf("Done\n");
}

static void compute_angular_movement() {
  angular_movement = fabsf(gyro.x) + fabsf(gyro.y) + fabsf(gyro.z);
}

// https://web.archive.org/web/20120602094634/http://gentlenav.googlecode.com/files/DCMDraft2.pdf
static void compute_angles() {
  const float delta = 1.0f / 100.0f;
  float gx = to_float(gyro.x) * delta;
  float gy = to_float(gyro.y) * delta;
  float gz = to_float(gyro.z) * delta;

  V3f newRX = {
    RX.x + RY.x * gz - RZ.x * gy,
    RX.y + RY.y * gz - RZ.y * gy,
    RX.z + RY.z * gz - RZ.z * gy
  };
  V3f newRY = {
    -RX.x * gz + RY.x + RZ.x * gx,
    -RX.y * gz + RY.y + RZ.y * gx,
    -RX.z * gz + RY.z + RZ.z * gx
  };
  V3f newRZ = {
    RX.x * gy - RY.x * gx + RZ.x,
    RX.y * gy - RY.y * gx + RZ.y,
    RX.z * gy - RY.z * gx + RZ.z
  };
  float error = v3f_dot(newRX, newRY);
  V3f orthRX = v3f_sub(newRX, v3f_scale(newRY, error*0.5f));
  V3f orthRY = v3f_sub(newRY, v3f_scale(newRX, error*0.5f));
  V3f orthRZ = v3f_cross(orthRX, orthRY);
  
  RX = v3f_scale(orthRX, 0.5f*(3.0f-v3f_len2(orthRX)));
  RY = v3f_scale(orthRY, 0.5f*(3.0f-v3f_len2(orthRY)));
  RZ = v3f_scale(orthRZ, 0.5f*(3.0f-v3f_len2(orthRZ)));
}

static void print_stats() {
  /*serial.printf("accel   %6.3f, %6.3f, %6.3f  %6.3f\n", v3_to_floats_and_length(accel));
  serial.printf("accelr  %6.3f, %6.3f, %6.3f\n", v3_to_floats(accel_raw));
  serial.printf("accel+  %6.3f, %6.3f, %6.3f\n", v3_to_floats(accel_max));
  serial.printf("accel-  %6.3f, %6.3f, %6.3f\n", v3_to_floats(accel_min));
  serial.printf("accel guess    %6.3f, %6.3f, %6.3f\n", v3_to_floats(accel_guess));
  serial.printf("gyror   %6.3f, %6.3f, %6.3f\n", v3_to_floats(gyro_raw));
  serial.printf("gyro    %6.3f, %6.3f, %6.3f\n", v3_to_floats(gyro));
  serial.printf("ang mov %6.3f\n", to_float(angular_movement));*/
  serial.printf("RX  %6.3f, %6.3f, %6.3f\n", RX);
  serial.printf("RY  %6.3f, %6.3f, %6.3f\n", RY);
  serial.printf("RZ  %6.3f, %6.3f, %6.3f\n", RZ);
  serial.printf("magnet  %6.3f, %6.3f, %6.3f  %6.3f\n", v3_to_floats_and_length(magnet));
  serial.printf("magnetr  %6.3f, %6.3f, %6.3f\n", v3_to_floats(magnet_raw));
  serial.printf("magnet+ %6.3f, %6.3f, %6.3f\n", v3_to_floats(magnet_max));
  serial.printf("magnet- %6.3f, %6.3f, %6.3f\n", v3_to_floats(magnet_min));
  serial.printf("\n");
}

static void read_accel() {
  static uint32_t stable_frames = 0;

  accel_raw = i2c_read_v3(I2C_ID_MPU6050, I2C_ADDR_MPU6050_ACCEL);
  gyro_raw = i2c_read_v3(I2C_ID_MPU6050, I2C_ADDR_MPU6050_GYRO);

  accel_raw = v3_scale(accel_raw, to_fixed16(FIX_ONEF * 9.85f * 2.0f/32768.0f));
  gyro_raw = v3_scale(gyro_raw, to_fixed16(FIX_ONEF * 0.01745329f * 250.0f/32768.0f));

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
  accel = v3_sub(accel_raw, accel_guess);
}

static void read_magnet() {
  magnet_raw = i2c_read_v3_rev(I2C_ID_QMC5883L, I2C_ADDR_QMC5883L_MAGNET);

  magnet_max.x = max(magnet_raw.x, magnet_max.x);
  magnet_max.y = max(magnet_raw.y, magnet_max.y);
  magnet_max.z = max(magnet_raw.z, magnet_max.z);

  magnet_min.x = min(magnet_raw.x, magnet_min.x);
  magnet_min.y = min(magnet_raw.y, magnet_min.y);
  magnet_min.z = min(magnet_raw.z, magnet_min.z);

  magnet_guess = v3_scale(v3_add(magnet_max, magnet_min), to_fixed16(0.5f));
  magnet = v3_sub(magnet_raw, magnet_guess);
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