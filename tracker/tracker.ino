// Sketch requires Adafruit MPU6050 and PrintEx library

// Basic demo for accelerometer readings from Adafruit MPU6050

#include <PrintEx.h>
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <avr/io.h>
#include <avr/sleep.h>

static StreamEx serial = Serial;

typedef struct {
  float x;
  float y;
  float z;
} V3f;

// Only used as quaterions
typedef struct {
  float w;
  float i;
  float j;
  float k;
} V4f;

//

static V3f v3f_add(const V3f x, const V3f y) { return (V3f){ x.x + y.x, x.y + y.y, x.z + y.z }; }
static V3f v3f_sub(const V3f x, const V3f y) { return (V3f){ x.x - y.x, x.y - y.y, x.z - y.z }; }
static V3f v3f_mul(const V3f x, const V3f y) { return (V3f){ x.x * y.x, x.y * y.y, x.z * y.z }; }
static V3f v3f_div(const V3f x, const V3f y) { return (V3f){ x.x / y.x, x.y / y.y, x.z / y.z }; }
static V3f v3f_cross(const V3f x, const V3f y) { return (V3f){ x.y*y.z-x.z*y.y, x.z*y.x-x.x*y.z, x.x*y.y-x.y*y.x }; }
static V3f v3f_scale(const V3f x, float y) { return (V3f){ x.x * y, x.y * y, x.z * y }; }
static V3f v3f_norm(const V3f x) { return v3f_scale(x, 1.0f / v3f_len(x)); };
static float v3f_dot(const V3f x, const V3f y) { return x.x * y.x + x.y * y.y + x.z * y.z; };
static float v3f_len(const V3f x) { return sqrtf(x.x * x.x + x.y * x.y + x.z * x.z); };
static float v3f_len2(const V3f x) { return x.x * x.x + x.y * x.y + x.z * x.z; };
static float v3f_dist(const V3f x, const V3f y) { return v3f_len(v3f_sub(x, y)); };
static float v3f_dist2(const V3f x, const V3f y) { return v3f_len2(v3f_sub(x, y)); };

// Roguhly 0.5^(1/50)
#define HALF_LIFE_0_5             0.986232704493f

// How many entries to keep in the guess algorihm
#define ACCEL_BUFFER_SIZE         16

// Multiplier for the guess algorithm
#define ACCEL_GUESS_AGGRESSIVNESS 0.02f

// Accelerometer guess vector will not exeed this magnitude
#define ACCEL_GUESS_MAX_LEN       6.0f

// Estimated gravity magnitude used by acceleration offset guessing algorihm
#define ACCEL_ESTIMATED_GRAVITY   9.85f

/*
accel calibrate -0.654,  0.070,  7.055
gyro  calibrate -0.026,  0.036, -0.020
accel guess     0.648,  0.576, -2.755
*/
const static V3f accel_offset = { 0.648, 0.576, -2.755};
const static V3f gyro_offset =  {-0.026, 0.036, -0.021};

static V3f accel_raw = {0, 0, 0};
static V3f accel = {0, 0, 0};
static V3f accel_max = {0, 0, 0};
static V3f accel_min = {0, 0, 0};
static V3f accel_guess = {0.648f, -0.031, -3.676f};
static V3f gyro_raw = {0, 0, 0};
static V3f gyro = {0, 0, 0};
static uint32_t n = 0;
static uint32_t next_tick; // TODO: Upgrade to 64 bit?

static int i2c_detect(int id) {
  Wire.beginTransmission(id);
  return Wire.endTransmission() == 0;
}

static V3f i2c_read_v3(int id, int address) {
  V3f result;
  Wire.beginTransmission(id);
  Wire.write(address);
  int error = Wire.endTransmission(false);
  Wire.requestFrom(id, 6, 1);
  if (error) {
    serial.printf("I2C error %d\n", error);
    delay(100);
  }
  int val;
  val = Wire.read() << 8;
  val |= Wire.read();
  result.x = val;
  val = Wire.read() << 8;
  val |= Wire.read();
  result.y = val;
  val = Wire.read() << 8;
  val |= Wire.read();
  result.z = val;
  error = Wire.endTransmission();
  if (error) {
    serial.printf("I2C error %d\n", error);
    delay(100);
  }
  return result;
}

static V3f i2c_read_v3_rev(int id, int address) {
  V3f result;
  Wire.beginTransmission(id);
  Wire.write(address);
  int error = Wire.endTransmission(false);
  Wire.requestFrom(id, 6, 1);
  if (error) {
    serial.printf("I2C error %d\n", error);
    delay(100);
  }
  int val;
  val = Wire.read();
  val = Wire.read() << 8;
  result.x = val;
  val = Wire.read();
  val = Wire.read() << 8;
  result.y = val;
  val = Wire.read();
  val = Wire.read() << 8;
  result.z = val;
  error = Wire.endTransmission();
  if (error) {
    serial.printf("I2C error %d\n", error);
    delay(100);
  }
  return result;
}

static int i2c_read_byte(int id, int address) {
  Wire.beginTransmission(id);
  Wire.write(address);
  int error = Wire.endTransmission(false);
  Wire.requestFrom(id, 1, 1);
  if (error) {
    serial.printf("I2C error %d\n", error);
    delay(100);
  }
  int result = Wire.read();
  error = Wire.endTransmission();
  if (error) {
    serial.printf("I2C error %d\n", error);
    delay(100);
  }
  return result;
}

static int i2c_read_bcd6(int id, int address) {
  int data = i2c_read_byte(id, address);
  return (data & 0x0f) + ((data & 0x40)>>4) * 10;
}

static int i2c_read_bcd7(int id, int address) {
  int data = i2c_read_byte(id, address);
  return (data & 0x0f) + ((data & 0x70)>>4) * 10;
}

static int i2c_read_bcd8(int id, int address) {
  int data = i2c_read_byte(id, address);
  return (data & 0x0f) + ((data & 0xf0)>>4) * 10;
}

static void i2c_write_byte(int id, int address, int value) {
  Wire.beginTransmission(id);
  Wire.write(address);
  Wire.write(value);
  int error = Wire.endTransmission();
  if (error) {
    serial.printf("I2C error %d\n", error);
    delay(100);
  }
}

static void(*reset)() = 0;

#define I2C_ID_MPU6050                0x69
#define I2C_ADDR_MPU6050_ACCEL        0x3B
#define I2C_ADDR_MPU6050_GYRO         0x43
#define I2C_ADDR_MPU6050_RESET        0x6B
#define I2C_ADDR_MPU6050_WHOAMI       0x75
#define I2C_ADDR_MPU6050_GYRO_CONFIG  0x1B
#define I2C_ADDR_MPU6050_ACCEL_CONFIG 0x1C

#define I2C_ID_DS1307                 0x68
#define I2C_ADDR_DS1307_SECOND_BCD7   0x00
#define I2C_ADDR_DS1307_MINUTE_BCD7   0x01
#define I2C_ADDR_DS1307_HOUR_BCD6     0x02
#define I2C_ADDR_DS1307_DAY           0x03
#define I2C_ADDR_DS1307_DATE_BCD7     0x04
#define I2C_ADDR_DS1307_MONTH_BCD7    0x05
#define I2C_ADDR_DS1307_YEAR_BCD8     0x06
#define I2C_ADDR_DS1307_CONTROL       0x07

static void init_mpu6050() {
  if (!i2c_detect(I2C_ID_MPU6050)) {
    serial.printf("Failed to find MPU6050 chip\n");
    delay(3000);
    reset();
  }
  i2c_write_byte(I2C_ID_MPU6050, I2C_ADDR_MPU6050_RESET, 0x00); // Reset the device
}

static void init_ds1307() {
  if (!i2c_detect(I2C_ID_DS1307)) {
    serial.printf("Failed to find DS1307 chip\n");
    delay(3000);
    reset();
  }
  i2c_write_byte(I2C_ID_DS1307, I2C_ADDR_DS1307_SECOND_BCD7, 0x00); // Enable oscillator, seconds = 0
  i2c_write_byte(I2C_ID_DS1307, I2C_ADDR_DS1307_MINUTE_BCD7, 0x01); // minutes = 1
  i2c_write_byte(I2C_ID_DS1307, I2C_ADDR_DS1307_HOUR_BCD6, 0x00); // hours = 0
  i2c_write_byte(I2C_ID_DS1307, I2C_ADDR_DS1307_DAY, 0x00); // days = 1
  i2c_write_byte(I2C_ID_DS1307, I2C_ADDR_DS1307_DATE_BCD7, 0x00); // date = 1
  i2c_write_byte(I2C_ID_DS1307, I2C_ADDR_DS1307_MONTH_BCD7, 0x00); // months = 1
  i2c_write_byte(I2C_ID_DS1307, I2C_ADDR_DS1307_YEAR_BCD8, 0x00); // years = 0
  //i2c_write_byte(I2C_ID_DS1307, I2C_ADDR_DS1307_CONTROL, 0x10); // Enable square wave, 32 kHz
}

int sleep = 0;

ISR(TIMER2_OVF_vect) //update time
{
  /*TCNT2 = 0; //preload timer 256 - 32,768kHz/32/4Hz
  sleep ++;*/
}

static void init_counter() {
  TCCR2A = 0x00; // Wave Form Generation Mode 0: Normal Mode, OC2A disconnected
  TCCR2B = (1<<CS22)|(1<<CS21)|(1<<CS20); // prescaler = 1024
  TIMSK2 = (1<<TOIE2); // Enable interrupt
}

SdFat sd;
SdFile myFile;

void init_sd() {
  if (!sd.begin(16, SPI_HALF_SPEED)) {
    sd.initErrorHalt();
  }

  // re-open the file for reading:
  if (!myFile.open("test.txt", O_READ)) {
    sd.errorHalt("opening test.txt for read failed");
  }
  Serial.println("test.txt:");

  // read from the file until there's nothing else in it:
  int data;
  while ((data = myFile.read()) >= 0) {
    Serial.write(data);
  }
  // close the file:
  myFile.close();
  if (!myFile.open("test.txt", O_RDWR | O_CREAT | O_TRUNC | O_SYNC)) {
    sd.errorHalt("opening test.txt for write failed");
  }
}

void setup(void) {
  Serial.begin(115200);
  serial.printf("Init\n");
  delay(100);

  Wire.begin();

  init_sd();
  init_mpu6050();
  init_ds1307();
  init_counter();

  delay(100);

  next_tick = micros() + 100000;

  serial.printf("Done\n");
  serial.flush();
}

static int timestamp_seconds() {
  int data = i2c_read_byte(I2C_ID_DS1307, I2C_ADDR_DS1307_SECOND_BCD7);
  return (data & 0xF) + (data >> 4) * 10;
}

int activity_accel = 0;
int activity_gyro = 0;
int next_write = 10;

static void write_file() {
  serial.printf("WRITING FILE");
  myFile.println("activity:");
  myFile.println(activity_accel + activity_gyro);
  activity_accel = 0;
  activity_gyro = 0;
  myFile.close();
  if (!myFile.open("test.txt", O_RDWR | O_CREAT | O_APPEND | O_SYNC)) {
    sd.errorHalt("opening test.txt for write failed");
  }
  serial.flush();
}

static uint64_t timestamp() {
  return 
    i2c_read_byte(I2C_ID_DS1307, I2C_ADDR_DS1307_SECOND_BCD7)
    | (i2c_read_bcd7(I2C_ID_DS1307, I2C_ADDR_DS1307_MINUTE_BCD7) << 8)
    | (i2c_read_bcd6(I2C_ID_DS1307, I2C_ADDR_DS1307_HOUR_BCD6) << 16)
    | (i2c_read_bcd7(I2C_ID_DS1307, I2C_ADDR_DS1307_DATE_BCD7) << 24)
    | (i2c_read_bcd7(I2C_ID_DS1307, I2C_ADDR_DS1307_MONTH_BCD7) << 32)
    | (i2c_read_bcd8(I2C_ID_DS1307, I2C_ADDR_DS1307_YEAR_BCD8) << 40);
}

static void print_stats() {
  serial.printf("\n");
  serial.printf("activity %d %d\n", fabsf(v3f_len(accel)-9.85f) > 2.5f, v3f_len(gyro) > 0.15f);
  uint64_t ts = timestamp();
  serial.printf("timestamp 0x%lx%lx %d %d\n", (uint32_t)(ts >> 32), (uint32_t)timestamp(), timestamp_seconds(), next_write);
  //serial.printf("count %d %d\n", (int)(TCNT2), sleep);
  //serial.printf("accelr  %6.3f, %6.3f, %6.3f\n", accel_raw);
  //serial.printf("accel   %6.3f, %6.3f, %6.3f  %6.3f\n", accel, v3f_len(accel));
  /*serial.printf("accel+  %6.3f, %6.3f, %6.3f\n", v3_to_floats(accel_max));
  serial.printf("accel-  %6.3f, %6.3f, %6.3f\n", v3_to_floats(accel_min));
  serial.printf("accel guess    %6.3f, %6.3f, %6.3f\n", v3_to_floats(accel_guess));*/
  //serial.printf("gyror   %6.3f, %6.3f, %6.3f\n", gyro_raw);
  //serial.printf("gyro    %6.3f, %6.3f, %6.3f\n", gyro);
  //serial.printf("gyro error  %6.3f, %6.3f, %6.3f\n", gx_err, gy_err, gz_err);
  //serial.printf("gyro adj to  %6.3f, %6.3f, %6.3f\n", g_adj);
}

static void compute_accel() {
  activity_accel += fabsf(v3f_len(accel)-9.85f) > 2.5f;
  activity_gyro += v3f_len(gyro) > 0.15f;
}

static void read_accel() {
  static uint32_t stable_frames = 0;

  accel_raw = i2c_read_v3(I2C_ID_MPU6050, I2C_ADDR_MPU6050_ACCEL);
  gyro_raw = i2c_read_v3(I2C_ID_MPU6050, I2C_ADDR_MPU6050_GYRO);
  accel_raw = v3f_scale(accel_raw, 9.85f * 2.0f/32768.0f);
  gyro_raw = v3f_scale(gyro_raw, 256.0f * 0.01745329f / 32768.0f); // TODO: Why is this constant like this?

  gyro = v3f_sub(gyro_raw, gyro_offset);

  if (v3f_len(gyro) < 0.05) {
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
        accel_guess = v3f_scale(v3f_add(accel_max, accel_min), 0.5f);
    }
  }
  accel = v3f_sub(accel_raw, accel_guess);
}

void loop() {
  TIMSK2 = (1<<TOIE2); // interrupt when TCNT2 is overflowed
  TCNT2 = 0;
  set_sleep_mode(SLEEP_MODE_PWR_SAVE); // choose power down mode
  sleep_mode(); // sleep now!
  sleep++;

  read_accel();
  compute_accel();

 // if ((sleep % 61) == 0) {
  if (timestamp_seconds() == next_write) {
    write_file();
    next_write += 10;
    next_write %= 60;
  }
  if ((sleep % 31) == 0) {
    print_stats();
    serial.flush();
    sleep = 0;
  }
}
