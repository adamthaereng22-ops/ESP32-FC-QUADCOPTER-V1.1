#ifndef ESP32
#error "Select an ESP32 board: Tools → Board → ESP32 Arduino → ESP32 Dev Module"
#endif

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP32Servo.h>
#include <TinyGPS++.h>
#include <math.h>

/***************** Pins *****************/
constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;

constexpr int ESC_M1_PIN = 13; // Rear Right  CCW
constexpr int ESC_M2_PIN = 14; // Front Right CW
constexpr int ESC_M3_PIN = 25; // Rear Left   CW
constexpr int ESC_M4_PIN = 26; // Front Left  CCW

constexpr int IBUS_RX_PIN = 16;
constexpr int IBUS_TX_PIN = 27;

constexpr int GPS_RX_PIN = 17;
constexpr int GPS_TX_PIN = 5;

/************** ESC pulse limits **************/
constexpr int PWM_MIN  = 1000;
constexpr int PWM_IDLE = 1050;
constexpr int PWM_MAX  = 2000;

/***************** Objects *****************/
Adafruit_MPU6050 mpu;
Adafruit_BME280 bme;
Servo esc1, esc2, esc3, esc4;
HardwareSerial IBUS_S(2);
HardwareSerial GPS_Serial(1);
TinyGPSPlus gps;

/**************** System State ***************/
bool armed = false;
uint32_t lastMotorUpdate = 0;

/************ Complementary filter ************/
float roll_deg  = 0.0f;
float pitch_deg = 0.0f;
float yawRate_dps = 0.0f;
float yawAngle_deg = 0.0f; // تكامل yaw لحل الانحراف
const float alpha = 0.95f;

float gx_offset = 0.0f;
float gy_offset = 0.0f;
float gz_offset = 0.0f;

/******** BME280 ************/
bool  bme_ok       = false;
float bme_temp_C   = 0.0f;
float bme_pres_hPa = 0.0f;
float bme_alt_m    = 0.0f;
float bme_alt0_m   = 0.0f;
const float SEALEVEL_HPA = 1013.25f;

/***************** PID *****************/
struct PIDState { float integral = 0.0f; float prevMeas = 0.0f; };

// زاوية (خارجي) لِـ Roll/Pitch
PIDState pidAngR, pidAngP;
// معدل (داخلي) لِـ Roll/Pitch
PIDState pidRateR, pidRateP;
// معدل Yaw
PIDState pidYaw;

// معاملات زاوية (خارجي)
float KpAngR=1.2f, KiAngR=0.00f, KdAngR=0.00f;
float KpAngP=1.2f, KiAngP=0.00f, KdAngP=0.00f;

// معاملات معدل (داخلي)
float KpRateR=0.32f, KiRateR=0.04f, KdRateR=0.025f;
float KpRateP=0.32f, KiRateP=0.04f, KdRateP=0.025f;

// Yaw كمعدل
float KpYaw=0.70f, KiYaw=0.06f, KdYaw=0.0f;

/************ GPS Hold PID ************/
PIDState pidGpsX, pidGpsY;
float KpGps = 0.8f;
float KiGps = 0.02f;
float KdGps = 0.10f;
bool gpsHoldEnabled = false;
double homeLat = 0.0;
double homeLon = 0.0;

/**************** Mixer ****************/
int baseThrottle = PWM_IDLE;
int m1 = PWM_MIN, m2 = PWM_MIN, m3 = PWM_MIN, m4 = PWM_MIN;
float uRoll=0.0f, uPitch=0.0f, uYaw=0.0f;

/*************** iBUS mapping ***************/
constexpr int CH_ROLL      = 0;
constexpr int CH_PITCH     = 1;
constexpr int CH_THR       = 2;
constexpr int CH_YAW       = 3;
constexpr int CH_ARM       = 4;
constexpr int CH_GPS_HOLD  = 5;

const float MAX_ANGLE_DEG   = 25.0f;
const float MAX_RATE_TARGET = 250.0f;
const float MAX_YAW_RATE    = 120.0f;
const int   DEADBAND        = 15;
const float ARM_ANGLE_LIMIT = 25.0f;

/*************** iBUS state ***************/
uint16_t ibus_ch[14] = {1500,1500,1000,1500,1000};
uint32_t lastIbusFrameMs = 0;

/**************** Helpers ****************/
static inline float fmap(float x,float inMin,float inMax,float outMin,float outMax){
  return (x-inMin)*(outMax-outMin)/(inMax-inMin)+outMin;
}
static inline float fconstrain(float v,float lo,float hi){
  return (v<lo)?lo:((v>hi)?hi:v);
}
static inline int applyDeadband(int v,int center=1500,int db=DEADBAND){
  return (abs(v-center)<=db)?center:v;
}

// PID قياسي
float pidCompute(float setpoint,float measurement,PIDState &s,float dt,
                 float Kp,float Ki,float Kd,float outMin,float outMax){
  float error = setpoint - measurement;
  float dMeas = (measurement - s.prevMeas)/dt;
  s.prevMeas = measurement;
  s.integral += error * dt;
  s.integral = fconstrain(s.integral, -80.0f, 80.0f);
  float out = Kp*error + Ki*s.integral - Kd*dMeas;
  return fconstrain(out,outMin,outMax);
}

/*************** IMU update ***************/
float gx_prev=0, gy_prev=0, gz_prev=0;
const float gyro_lpf = 0.43f;

void updateIMU(float dt){
  sensors_event_t a,g,t;
  mpu.getEvent(&a,&g,&t);

  const float RAD2DEG = 57.29578f;
  float gx = (g.gyro.x - gx_offset) * RAD2DEG;
  float gy = (g.gyro.y - gy_offset) * RAD2DEG;
  float gz = (g.gyro.z - gz_offset) * RAD2DEG;

  gx = gx_prev*gyro_lpf + gx*(1-gyro_lpf); gx_prev=gx;
  gy = gy_prev*gyro_lpf + gy*(1-gyro_lpf); gy_prev=gy;
  gz = gz_prev*gyro_lpf + gz*(1-gyro_lpf); gz_prev=gz;

  roll_deg  += gx * dt;
  pitch_deg += gy * dt;
  yawAngle_deg += gz * dt;
  yawRate_dps = gz;

  float accRoll  = atan2f(a.acceleration.y,a.acceleration.z)*RAD2DEG;
  float accPitch = atan2f(-a.acceleration.x,
                    sqrtf(a.acceleration.y*a.acceleration.y+a.acceleration.z*a.acceleration.z))*RAD2DEG;

  roll_deg  = alpha*roll_deg  + (1-alpha)*accRoll;
  pitch_deg = alpha*pitch_deg + (1-alpha)*accPitch;
}

/***************** Mixer (X) *****************/
void mixMotors(){
  m1 = baseThrottle - uRoll - uPitch + uYaw; // Rear Right
  m2 = baseThrottle - uRoll + uPitch - uYaw; // Front Right
  m3 = baseThrottle + uRoll - uPitch - uYaw; // Rear Left
  m4 = baseThrottle + uRoll + uPitch + uYaw; // Front Left
}

/*************** iBUS *****************/
bool ibusReadFrame(){
  const int FRAME_LEN = 32;
  if(IBUS_S.available() < FRAME_LEN) return false;
  if(IBUS_S.peek() != 0x20){ IBUS_S.read(); return false; }

  uint8_t buf[FRAME_LEN];
  IBUS_S.readBytes(buf, FRAME_LEN);

  if(buf[0]!=0x20 || buf[1]!=0x40) return false;

  uint16_t sum=0;
  for(int i=0;i<FRAME_LEN-2;i++) sum+=buf[i];
  uint16_t expect = 0xFFFF - sum;
  uint16_t got = (uint16_t)buf[FRAME_LEN-2] | ((uint16_t)buf[FRAME_LEN-1]<<8);
  if(got!=expect) return false;

  for(int ch=0;ch<14;ch++){
    int idx = 2 + ch*2;
    ibus_ch[ch] = (uint16_t)buf[idx] | ((uint16_t)buf[idx+1]<<8);
  }

  lastIbusFrameMs = millis();
  return true;
}

uint16_t ibusReadChannel(int ch){
  return (ch<0||ch>=14)?1500:ibus_ch[ch];
}

/******************** SETUP ********************/
void setup(){
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[BOOT] ESP32 Quad X starting...");
  Wire.begin(SDA_PIN, SCL_PIN);

  if(!mpu.begin()){
    while(1){ Serial.println("[ERR] MPU6050 not found!"); delay(1000); }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  const int N=4000;
  float gx_off=0,gy_off=0,gz_off=0;
  for(int i=0;i<N;i++){
    sensors_event_t a,g,t;
    mpu.getEvent(&a,&g,&t);
    gx_off+=g.gyro.x; gy_off+=g.gyro.y; gz_off+=g.gyro.z;
    delay(1);
  }
  gx_offset=gx_off/N; gy_offset=gy_off/N; gz_offset=gz_off/N;

  IBUS_S.begin(115200,SERIAL_8N1,IBUS_RX_PIN,IBUS_TX_PIN);
  IBUS_S.setTimeout(1);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  esc1.setPeriodHertz(50); esc2.setPeriodHertz(50);
  esc3.setPeriodHertz(50); esc4.setPeriodHertz(50);

  esc1.attach(ESC_M1_PIN,PWM_MIN,PWM_MAX);
  esc2.attach(ESC_M2_PIN,PWM_MIN,PWM_MAX);
  esc3.attach(ESC_M3_PIN,PWM_MIN,PWM_MAX);
  esc4.attach(ESC_M4_PIN,PWM_MIN,PWM_MAX);

  esc1.writeMicroseconds(PWM_MIN);
  esc2.writeMicroseconds(PWM_MIN);
  esc3.writeMicroseconds(PWM_MIN);
  esc4.writeMicroseconds(PWM_MIN);

  if(bme.begin(0x76)) bme_ok=true;
  else if(bme.begin(0x77)) bme_ok=true;
  else bme_ok=false;

  if(bme_ok){
    delay(100);
    bme_alt0_m = bme.readAltitude(SEALEVEL_HPA);
  }

  GPS_Serial.begin(9600,SERIAL_8N1,GPS_RX_PIN,GPS_TX_PIN); // يمكن رفع الباود ريت إذا أردت

  Serial.println("[OK] Setup done. (NO PROPS)");
}

/********************* LOOP *********************/
void loop(){
  static uint32_t lastLoop = micros();
  uint32_t now = micros();
  if(now - lastLoop < 500) return;
  float dt = (now - lastLoop) * 1e-6f;
  lastLoop = now;

  updateIMU(dt);
  while(ibusReadFrame()){}

  uint16_t thr=ibusReadChannel(CH_THR);
  uint16_t ail=ibusReadChannel(CH_ROLL);
  uint16_t ele=ibusReadChannel(CH_PITCH);
  uint16_t rud=ibusReadChannel(CH_YAW);
  uint16_t sw =ibusReadChannel(CH_ARM);
  uint16_t swGPS = ibusReadChannel(CH_GPS_HOLD);

  ail=applyDeadband(ail);
  ele=applyDeadband(ele);
  rud=applyDeadband(rud);

  bool wantArm=(sw>1700);
  bool thrLow=(thr<PWM_IDLE);

  gpsHoldEnabled = (swGPS > 1700);

  static float yawOffset = yawAngle_deg;

  if(wantArm && !armed && thrLow &&
     fabs(roll_deg)<=ARM_ANGLE_LIMIT &&
     fabs(pitch_deg)<=ARM_ANGLE_LIMIT){

    armed=true;
    yawOffset = yawAngle_deg;
    if(gps.location.isValid()){
      homeLat = gps.location.lat();
      homeLon = gps.location.lng();
    }
    for(int us=PWM_MIN;us<=PWM_IDLE;us+=5){
      esc1.writeMicroseconds(us);
      esc2.writeMicroseconds(us);
      esc3.writeMicroseconds(us);
      esc4.writeMicroseconds(us);
      delay(3);
    }
    pidAngR.integral=pidAngP.integral=0;
    pidRateR.integral=pidRateP.integral=0;
    pidYaw.integral=0;
    pidGpsX.integral=pidGpsY.integral=0;
  }
  else if(!wantArm && armed){
    armed=false;
    esc1.writeMicroseconds(PWM_MIN);
    esc2.writeMicroseconds(PWM_MIN);
    esc3.writeMicroseconds(PWM_MIN);
    esc4.writeMicroseconds(PWM_MIN);
  }

  baseThrottle=fconstrain(thr,PWM_MIN,PWM_MAX);

  /******** GPS Hold Correction ********/
  static double lastLat=0.0, lastLon=0.0;

  while(GPS_Serial.available()){
    gps.encode(GPS_Serial.read());
  }

  if(gps.location.isValid()){
    lastLat = gps.location.lat();
    lastLon = gps.location.lng();
  }

  float gpsRollCorr=0, gpsPitchCorr=0;
  if(gpsHoldEnabled){
    double dLat = (lastLat - homeLat)*111320.0;
    double dLon = (lastLon - homeLon)*111320.0*cos(homeLat*DEG_TO_RAD);

    gpsPitchCorr = pidCompute(0.0f,dLat,pidGpsX,dt*0.5,KpGps,KiGps,KdGps,-10,10);
    gpsRollCorr  = pidCompute(0.0f,dLon,pidGpsY,dt*0.5,KpGps,KiGps,KdGps,-10,10);
  }

  float setRollDeg  = fmap(ail,1000,2000,-MAX_ANGLE_DEG,+MAX_ANGLE_DEG) + gpsRollCorr;
  float setPitchDeg = fmap(ele,1000,2000,-MAX_ANGLE_DEG,+MAX_ANGLE_DEG) + gpsPitchCorr;
  float yawRate_sp  = fmap(rud,1000,2000,-MAX_YAW_RATE,+MAX_YAW_RATE);

  float rollRate_target  = pidCompute(setRollDeg , roll_deg , pidAngR, dt, KpAngR, KiAngR, KdAngR, -MAX_RATE_TARGET, +MAX_RATE_TARGET);
  float pitchRate_target = pidCompute(setPitchDeg, pitch_deg, pidAngP, dt, KpAngP, KiAngP, KdAngP, -MAX_RATE_TARGET, +MAX_RATE_TARGET);

  float uRoll_raw  = pidCompute(rollRate_target , gx_prev, pidRateR, dt, KpRateR, KiRateR, KdRateR, -150, 150);
  float uPitch_raw = pidCompute(pitchRate_target, gy_prev, pidRateP, dt, KpRateP, KiRateP, KdRateP, -150, 150);

  uRoll  =  uRoll_raw;
  uPitch =-uPitch_raw;

  float yawError = yawAngle_deg - yawOffset;
  uYaw = pidCompute(yawRate_sp - yawError, yawRate_dps, pidYaw, dt, KpYaw, KiYaw, KdYaw, -120, 120);

  mixMotors();

  const int MAX_STEP = 8;
  auto slew = [&](int target, int &cur){
    int delta = target - cur;
    if(delta >  MAX_STEP) delta =  MAX_STEP;
    if(delta < -MAX_STEP) delta = -MAX_STEP;
    cur += delta;
  };
  slew(m1, m1); slew(m2, m2); slew(m3, m3); slew(m4, m4);

  if(micros()-lastMotorUpdate>=1000){
    lastMotorUpdate=micros();
    if(armed){
      esc1.writeMicroseconds((m1<PWM_IDLE)?PWM_IDLE:m1);
      esc2.writeMicroseconds((m2<PWM_IDLE)?PWM_IDLE:m2);
      esc3.writeMicroseconds((m3<PWM_IDLE)?PWM_IDLE:m3);
      esc4.writeMicroseconds((m4<PWM_IDLE)?PWM_IDLE:m4);
    }else{
      esc1.writeMicroseconds(PWM_MIN);
      esc2.writeMicroseconds(PWM_MIN);
      esc3.writeMicroseconds(PWM_MIN);
      esc4.writeMicroseconds(PWM_MIN);
    }
  }

  /******** BME280 Update ********/
  if(bme_ok){
    bme_temp_C   = bme.readTemperature();
    bme_pres_hPa = bme.readPressure()/100.0f;
    bme_alt_m    = bme.readAltitude(SEALEVEL_HPA) - bme_alt0_m;
  }

  /******** Debug Print ********/
  static uint32_t lastPrint=0;
  if(millis()-lastPrint>=80){
    lastPrint=millis();
    Serial.print("ARM="); Serial.print(armed?"ON ":"OFF");
    Serial.print(" THR="); Serial.print(baseThrottle);
    Serial.print(" R/P/Y="); Serial.print(roll_deg,1); Serial.print("/"); Serial.print(pitch_deg,1); Serial.print("/"); Serial.print(yawAngle_deg,1);
    Serial.print(" u="); Serial.print((int)uRoll); Serial.print(","); Serial.print((int)uPitch); Serial.print(","); Serial.print((int)uYaw);
    Serial.print(" M="); Serial.print(m1); Serial.print(","); Serial.print(m2); Serial.print(","); Serial.print(m3); Serial.print(","); Serial.print(m4);
    Serial.print(" RC="); Serial.print(ail); Serial.print(","); Serial.print(ele); Serial.print(","); Serial.print(rud); Serial.print(","); Serial.print(thr);
    if(bme_ok){
      Serial.print(" T="); Serial.print(bme_temp_C,1); Serial.print("C Alt="); Serial.print(bme_alt_m,1);
    }
    if(gps.location.isUpdated()){
      Serial.print(" GPS="); Serial.print(gps.location.lat(),6); Serial.print(","); Serial.print(gps.location.lng(),6);
      Serial.print(" SAT="); Serial.print(gps.satellites.value());
    }
    Serial.print(" GPS_HOLD="); Serial.println(gpsHoldEnabled?"ON":"OFF");
  }
}
