#include <Wire.h>
#include <VL53L0X.h>

#define RADIUS 32.0  // Encoder'in dönen tekerlek yarıçapı (mm)
#define PPR 20       // Encoder'in her turdaki pulse sayısı
#define PI 3.14159265


// MPU6050 ve HMC5883L için I2C adresleri
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define GYRO_CNFG_REG 0x1B
#define ACC_CNFG_REG 0x1C

// HMC5883L register adresleri (Manyetometre)
#define HMC5883L_ADDR 0x1E
#define HMC5883L_REG_CONFIG_A 0x00
#define HMC5883L_REG_CONFIG_B 0x01
#define HMC5883L_REG_MODE 0x02
#define HMC5883L_REG_DATA_X_MSB 0x03


// Arabanın bulunduğu durumları temsil eder
enum NavigationState {
  INITIAL_ROTATION,
  FIRST_STRAIGHT,
  SECOND_ROTATION,
  SECOND_STRAIGHT,
  THIRD_ROTATION,
  THIRD_STRAIGHT,
  FOURTH_ROTATION,
  FOURTH_STRAIGHT,
  FIFTH_ROTATION,
  FIFTH_STRAIGHT,
  COMPLETED
};

// Durum değişkeni
NavigationState currentState = INITIAL_ROTATION;

// Hareket etmesi planlanan Durumlar
const float initialTargetYaw = 0;  // İlk rotasyon hedefi
const float secondTargetYaw = 90;  // İkinci rotasyon hedefi
const float thirdTargetYaw = 180;  // Üçüncü rotasyon hedefi
const float fourthTargetYaw = 90;  // Dördüncü rotasyon hedefi
const float fifthTargetYaw = 180;  // Dördüncü rotasyon hedefi
const float targetDistance = 420;  // mm cinsinden gidilecek ara mesafe
const float yawThreshold = 3;      // derece cinsinden sapma değeri

// Enkoderden pulse sayıcı değişkenleri
volatile unsigned long rightPulseCount = 0;
volatile unsigned long leftPulseCount = 0;
unsigned long previousRightPulseCount = 0;
unsigned long previousLeftPulseCount = 0;
float encoderDistance = 0;


// gerekli pinler tanımlaması
const byte rightEncoderPin = 18;  // Sağ interrupt encoder pini
const byte leftEncoderPin = 19;   // Sol interrupt encoder pini

const byte rightMotorPin1 = 33;
const byte rightMotorPin2 = 32;
const byte rightMotorPWM = 45;
const byte leftMotorPin1 = 36;
const byte leftMotorPin2 = 37;
const byte leftMotorPWM = 44;

// Mikro saniye cinsinden minimum ve maksimum PWM süreleri tanımlaması
const int SERVO_MIN = 555;   // MG90S için 0° (mikrosaniye) /sağ
const int SERVO_MAX = 2520;  // MG90S için 180° (mikrosaniye) /sol
const byte servoPin = 2;     // MG90S servo motor kontrol pini

byte flag = 0;



float yawMag;        // eğim telafisi yapılmış manyetometreden hesaplanan baş açısı
float yawFusion;     // manyetometre ile jiroskop füzyon ederek elde edilen baş açısı
float yaw;           // başlana referans açısından buluna baş açısı
float targetYaw;     // hedef baş açısı
float referenceYaw;  // robotu başlana referans baş açısı

float offsetGyroZ;

unsigned long previousMillis;

// PD Controller
const float Kp = 9.0, Kd = 1.0;
float pdError;
int outputPD = 0;

unsigned long nowTime;
unsigned long lastTime;

// lidar için gerekli tanımlamalar
VL53L0X lidar;
const byte gpioIntPin = 3;  // lidar INT pini
volatile bool measurementReady = false;
int lidarDistance;     // lidar'dan buluna ileri mesafe
float distanceFusion;  // lidar ile enkoder füzyon edilmiş hali mesafe

void setup() {
  Serial.begin(115200);
  Wire.setClock(400000);  // I2C hızını ayarla
  Wire.begin();
  delay(250);

  initLidar();

  initServo();

  initMPU6050();  // MPU6050 Sensörünü Başlat (ivmeölçer)
  CalibrationGyro(&offsetGyroZ);

  initHMC5883L();  // HMC5883L Sensörünü Başlat (manyetometre)

  initEncoder();  // Enkoder Pinleri Tanımlama ve interrupt'larını ayarlama

  initMotorPins();  // Motor Pinleri Tanımlama

  previousMillis = millis();
  lastTime = micros();
}

void loop() {

  int16_t rawAccelX, rawAccelY, rawAccelZ;                     // Ham ivmeölçer verileri
  float calibratedAccelX, calibratedAccelY, calibratedAccelZ;  // Kalibre edilmiş ivmeölçer verileri

  int16_t rawGyroZ;       // Ham jiroskop verileri
  float calibratedGyroZ;  // Kalibre edilmiş jiroskop verileri derece/saniye cinsinden

  int16_t rawMagX, rawMagY, rawMagZ;                     // Ham manyetometre verileri
  float calibratedMagX, calibratedMagY, calibratedMagZ;  // Kalibre edilmiş manyetometre verileri

  // MPU6050 İvmeölçer verilerini oku
  readAccelMPU6050(&rawAccelX, &rawAccelY, &rawAccelZ);
  // Kalibre edilmiş değerleri hesapla
  getCalibratedAccel(rawAccelX, rawAccelY, rawAccelZ,
                     &calibratedAccelX, &calibratedAccelY, &calibratedAccelZ);

  // readHMC5883L manyetometre verilerini oku
  readHMC5883L(&rawMagX, &rawMagY, &rawMagZ);
  // Kalibre edilmiş değerleri hesapla
  getCalibratedMag(rawMagX, rawMagY, rawMagZ,
                   &calibratedMagX, &calibratedMagY, &calibratedMagZ);



  // eğim telafisi ve Yaw açısını hesapla
  calculateYaw(calibratedAccelX, calibratedAccelY, calibratedAccelZ,
               calibratedMagX, calibratedMagY, calibratedMagZ);

  // MPU6050 jiroskop verilerini oku ve kalibre et
  readGyroMPU6050(&rawGyroZ);
  getCalibratedGyro(rawGyroZ, &calibratedGyroZ);

  // robot ilk açıldığında açı düzenlene kadar 6 sanyiye çalış sonra açıyı referans al
  while (flag == 0) {

    // zaman aralığı hesaplama
    nowTime = micros();
    float dt = (float)(nowTime - lastTime) / 1.0e6;
    lastTime = nowTime;

    // tamamlayıcı filtre uygulaması -- manyetometre ile jiroskop füzyonu
    yawFusion = 0.95 * (yawFusion - calibratedGyroZ * dt) + 0.05 * yawMag;

    if (millis() - previousMillis > 6000) {
      flag = 1;
      referenceYaw = yawFusion;

      writeServo(180);
      delay(2000);
      writeServo(90);
      delay(2000);
    }
  };

  // zaman aralığı hesaplama
  nowTime = micros();
  float dt = (float)(nowTime - lastTime) / 1.0e6;
  lastTime = nowTime;

  // tamamlayıcı filtre uygulaması -- manyetometre ile jiroskop füzyonu
  yawFusion = 0.95 * (yawFusion - calibratedGyroZ * dt) + 0.05 * yawMag;

  // PD kontrolör -----------------
  yaw = yawFusion - referenceYaw;

  // robot ilk çalıştığında eksi küçük değer -0. çıkıyor onu istemiyoruz robotun konumu şarşırtıyor
  if (yaw < 0 && yaw > -1) {
    yaw = 0;

  } else if (yaw < 0) {
    yaw += 360;
  }
  ////
  pdError = targetYaw - yaw;
  // derivative = -calibratedGyroZ   -- // yaw açısını Türev terimi yerine jiroskoptan direk açısal hızı kullandım
  outputPD = (int)(Kp * pdError + Kd * calibratedGyroZ);

  float deltaDistance = readEncoders();
  encoderDistance = encoderDistance + deltaDistance;  // mm cinsinden

  // lidar ölçüsü varsa veriyi oku
  if (measurementReady) {
    measurementReady = false;
    int distance = lidar.readRangeSingleMillimeters();  //lidardan mesafe ölçümü mm cinsinden
    lidar.timeoutOccurred();
    lidarDistance = 1100 - distance;
    if (lidarDistance < 0) {
      lidarDistance = 0;
    }

    lidar.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);  // interrupt'ı temizle
  }

  distanceFusion = 0.05 * (lidarDistance) + 0.95 * encoderDistance;






  //Navigasyonu yönet
  handleNavigation();

  // Bilgileri seri monitöre yazdır
  Serial.print(" lidarDistance ");
  Serial.print(lidarDistance);
  Serial.print(" distanceFusion: ");
  Serial.print(distanceFusion);

  Serial.print(" rightPulseCount: ");
  Serial.print(rightPulseCount);
  Serial.print(" leftPulseCount: ");
  Serial.print(leftPulseCount);



  Serial.print(" encoderDistance: ");
  Serial.print(encoderDistance);

  Serial.print(" yaw: ");
  Serial.print(yaw);
  Serial.print("°");
  Serial.print(" yawFusion: ");
  Serial.print(yawFusion);
  Serial.println("°");


  //while (millis() - previousMillis <= 100)
  //;  //Nan statment
  //previousMillis = millis();
}


// lidar için interrupt service routine
void lidarISR() {
  measurementReady = true;
}
void initLidar() {


  pinMode(gpioIntPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(gpioIntPin), lidarISR, FALLING);


  lidar.setTimeout(500);

  if (!lidar.init()) {
    Serial.println("VL53L0X başlatılamadı. Bağlantıları kontrol et.");
    while (1)
      ;  // Dur
  }
  // GPIO1 ayarı: ölçüm tamamlandığında kesme tetikle
  lidar.writeReg(VL53L0X::SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);  // measurement ready
  lidar.writeReg(VL53L0X::GPIO_HV_MUX_ACTIVE_HIGH,
                 lidar.readReg(VL53L0X::GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10);  // active low
  lidar.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);                    // kesmeyi temizle

  //Ölçüm aralığı ayarlanabilir:
  //lidar.setSignalRateLimit(0.1);
  //lidar.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  //lidar.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

  lidar.startContinuous();  // Sürekli ölçüm modu
}
void initServo() {

  pinMode(servoPin, OUTPUT);  // Servo motor pinini çıkış olarak ayarla

  // Servoyu 90° pozisyona döndür
  writeServo(90);
  delay(2000);  // 2 saniye bekle
}

void initEncoder() {
  // Encoder pinleri giriş ve pullup olarak ayarla
  pinMode(rightEncoderPin, INPUT_PULLUP);
  pinMode(leftEncoderPin, INPUT_PULLUP);

  // Encoder interrupt'larını ayarla
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), rightEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), leftEncoderISR, RISING);
}

// Sağ encoder için interrupt servis rutini
void rightEncoderISR() {
  static unsigned long lastInterruptTime = 0;
  unsigned long currentTime = micros();
  if (currentTime - lastInterruptTime > 2000) {  // 2000 µs debounce süresi
    rightPulseCount++;
    lastInterruptTime = currentTime;
  }
}

// Sol encoder için interrupt servis rutini
void leftEncoderISR() {
  static unsigned long lastInterruptTime = 0;
  unsigned long currentTime = micros();
  if (currentTime - lastInterruptTime > 1500) {  // 1500 µs debounce süresi
    leftPulseCount++;
    lastInterruptTime = currentTime;
  }
}

//enkoderden mesafe hesaapla
float readEncoders() {
  int16_t deltaRightPulseCount = rightPulseCount - previousRightPulseCount;
  previousRightPulseCount = rightPulseCount;

  int16_t deltaLeftPulseCount = leftPulseCount - previousLeftPulseCount;
  previousLeftPulseCount = leftPulseCount;

  float deltaRightDistance = (float(deltaRightPulseCount) / PPR) * (2 * PI * RADIUS);
  float deltaLeftDistance = (float(deltaLeftPulseCount) / PPR) * (2 * PI * RADIUS);

  float deltaDistance = (deltaRightDistance + deltaLeftDistance) / 2.0;
  return deltaDistance;
}

void initMotorPins() {
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
}

void moveForward() {
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
  analogWrite(rightMotorPWM, 80);

  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  analogWrite(leftMotorPWM, 80);
}


void stopMotors() {
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);
  analogWrite(rightMotorPWM, 0);

  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);
  analogWrite(leftMotorPWM, 0);
}

void moveRight() {
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);
  analogWrite(rightMotorPWM, outputPD);

  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  analogWrite(leftMotorPWM, outputPD);
}

void moveLeft() {
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
  analogWrite(rightMotorPWM, outputPD);

  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);
  analogWrite(leftMotorPWM, outputPD);
}


// Kalibre edilmiş ivmeölçer değerlerini hesapla
void getCalibratedAccel(int16_t rawAccelX, int16_t rawAccelY, int16_t rawAccelZ,
                        float *calibratedAccelX, float *calibratedAccelY, float *calibratedAccelZ) {
  float temp[3];
  float accelOffset[3] = { 125.057492, -12.818640, -47.637789 };

  float accelAinv[3][3] = { { 1.000456, 0.00059, 0.001449 },
                            { 0.00059, 1.004256, 0.002941 },
                            { 0.001449, 0.002941, 0.994340 } };
  temp[0] = (float)rawAccelX - accelOffset[0];
  temp[1] = (float)rawAccelY - accelOffset[1];
  temp[2] = (float)rawAccelZ - accelOffset[2];
  *calibratedAccelX = accelAinv[0][0] * temp[0] + accelAinv[0][1] * temp[1] + accelAinv[0][2] * temp[2];
  *calibratedAccelY = accelAinv[1][0] * temp[0] + accelAinv[1][1] * temp[1] + accelAinv[1][2] * temp[2];
  *calibratedAccelZ = accelAinv[2][0] * temp[0] + accelAinv[2][1] * temp[1] + accelAinv[2][2] * temp[2];
}


void getCalibratedGyro(int16_t rawGyroZ, float *calibratedGyroZ) {

  *calibratedGyroZ = (rawGyroZ - offsetGyroZ) / 32.8;  // /32.8 derece/saniye cinsinden dönüşümü için gerekli adım
}


void CalibrationGyro(float *offsetGyroZ) {

  Serial.println("jiroskop kalibrasyonu başladı lütfen sensörü hareket ettirmeyiniz...");
  int16_t rawGyroZ;
  for (int i = 0; i < 4000; i++) {

    readGyroMPU6050(&rawGyroZ);
    *offsetGyroZ += rawGyroZ;

    delay(1);
  }
  *offsetGyroZ /= 4000.0;
}

void getCalibratedMag(int16_t rawMagX, int16_t rawMagY, int16_t rawMagZ,
                      float *calibratedMagX, float *calibratedMagY, float *calibratedMagZ) {
  float temp[3];
  float magOffset[3] = { -19.467699, 408.842280, 94.706312 };

  float magAinv[3][3] = { { 1.009179, 0.003661, 0.058009 },
                          { 0.003661, 1.000805, -0.011264 },
                          { 0.058009, -0.011264, 1.095640 } };
  temp[0] = (float)rawMagX - magOffset[0];
  temp[1] = (float)rawMagY - magOffset[1];
  temp[2] = (float)rawMagZ - magOffset[2];
  *calibratedMagX = magAinv[0][0] * temp[0] + magAinv[0][1] * temp[1] + magAinv[0][2] * temp[2];  // LSB/Gauss birimi
  *calibratedMagY = magAinv[1][0] * temp[0] + magAinv[1][1] * temp[1] + magAinv[1][2] * temp[2];
  *calibratedMagZ = magAinv[2][0] * temp[0] + magAinv[2][1] * temp[1] + magAinv[2][2] * temp[2];
  *calibratedMagX *= 100.0f / 1370.0f;  // uT (microtesla) birimi
  *calibratedMagY *= 100.0f / 1370.0f;
  *calibratedMagZ *= 100.0f / 1370.0f;
}

// MPU6050 Sensörünü Başlat
void initMPU6050(void) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1_REG);
  Wire.write(0x00);  // Uyku modundan çık
  Wire.endTransmission();

  // SMPLRT_DIV_REG --> yazarak veri hızını 1KHz olarak ayarla 8KHZ Jiroskop çalışma frekansı için
  //ivme ölçer çıkış frekansı 1khz olarak ayarlanıyor
  //yani Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) ->>>1KHz=8KHz /(1+ SMPLRT_DIV=7)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(SMPLRT_DIV_REG);
  Wire.write(0x07);
  Wire.endTransmission();

  // GYRO CNFG --> +-500 derece/saniye -->08 veya --> +-1000 derece/saniye -->10
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_CNFG_REG);  // jiroskop çözünlüğü ayarlanması
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_ADDR, 6);

  Wire.beginTransmission(MPU6050_ADDR);  //burada 8g'yi ayarlamak (çözünürlüğü ayarlamak)
  Wire.write(ACC_CNFG_REG);              //ivmeölçer çıkışlarını 1C adresinde(register) saklanır bu adresten gelen ölçüm aralığı AFS_SEL'e (JİROSKOP GÜRÜLTÜ PERFORMANSI) göre 2 yani 10 yani 16 bit aralığ o da bizim sonraki satır tanımı
  Wire.write(0x10);                      //0x10 adresi iki tabanlıda gösterimi 0001 0000 o da 16 bit eder //
  Wire.endTransmission();
}

// MPU6050'dan İvmeölçer verilerini oku
void readAccelMPU6050(int16_t *accelX, int16_t *accelY, int16_t *accelZ) {
  Wire.beginTransmission(MPU6050_ADDR);  // bu dört satırı alçak geçiş filtresini aç
  Wire.write(0x1A);
  Wire.write(0x05);  //5 byte yaz
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_ADDR);  //ivmeölçer değerlerinden çekmesini başlamak
  Wire.write(0x3B);                      // ivmeölçer değerleri ilk saklanan değerin adresi AccX
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);  // bu fonkisyon 0x68 dresten iletilen verilerinden 6 baytı talep edeceğiz -- Normalde biz Master olarak biz veri göndeririz ama bu fonkisyonda Slave'i Master yaptık ve Master'ı Slave yaptık

  if (Wire.available() == 6) {
    *accelX = Wire.read() << 8 | Wire.read();  //Yüksek biti (MSB) 8 bit kaydırarak LSB bitinden veri çekmek
    *accelY = Wire.read() << 8 | Wire.read();
    *accelZ = Wire.read() << 8 | Wire.read();
  }
}

void readGyroMPU6050(int16_t *rawGyroZ) {



  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x43);  //jiroskop değerleri hesaplamak için dizinin ilk adresi
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_ADDR, 6);

  if (Wire.available() == 6) {
    int16_t rawGyroX = Wire.read() << 8 | Wire.read();  //Yüksek biti (MSB) 8 bit kaydırarak LSB bitinden veri çekmek
    int16_t rawGyroY = Wire.read() << 8 | Wire.read();
    *rawGyroZ = Wire.read() << 8 | Wire.read();
  }
}
// HMC5883L Sensörünü Başlat ayarları
void initHMC5883L() {
  // Mag CNFG -->  Config A register adresi --> HMC5883L_REG_CONFIG_A ---- Örnekleme oranı: 8G range, 200Hz ölçüm modu -->  0x1C
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(HMC5883L_REG_CONFIG_A);  // Config A register adresi
  Wire.write(0x1C);
  Wire.endTransmission();
  // Mag CNFG --> Config B register adresi --> HMC5883L_REG_CONFIG_B ---- Kazanç ayarı:  ±0.88 Gauss  --> 0x00
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(HMC5883L_REG_CONFIG_B);  // Config B register adresi
  Wire.write(0x00);                   // Kazanç ayarı: ±0.88 Gauss
  Wire.endTransmission();

  // Mag CNFG --> Mode register adresi --> HMC5883L_REG_MODE ---- Sürekli ölçüm modu --> 0x00
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(HMC5883L_REG_MODE);  // Mode register adresi
  Wire.write(0x00);               // Sürekli ölçüm modu
  Wire.endTransmission();
}

// HMC5883L manyetometre verilerini oku
void readHMC5883L(int16_t *magX, int16_t *magY, int16_t *magZ) {
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(HMC5883L_REG_DATA_X_MSB);  // Veri okuma başlangıç adresi (X ekseni MSB register adresi)
  Wire.endTransmission();
  Wire.requestFrom(HMC5883L_ADDR, 6);

  if (Wire.available() == 6) {
    *magX = (Wire.read() << 8) | Wire.read();
    *magZ = (Wire.read() << 8) | Wire.read();
    *magY = (Wire.read() << 8) | Wire.read();
  }
}



// Tilt Compensation (eğem telafisi) ve Yaw hesaplama
void calculateYaw(float accelX, float accelY, float accelZ, float magX, float magY, float magZ) {
  /// Pitch ve Roll açılarını hesapla (Radyan)
  float pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ));  //radyan cinsinden
  float roll = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ));

  //Tilt Combinsation (Eğim telafisi hesapla)
  float Xm = magX * cos(pitch) + magZ * sin(pitch);
  float Ym = magX * sin(roll) * sin(pitch) + magY * cos(roll) - magZ * sin(roll) * cos(pitch);

  // X ve Y eksenlerinden Yaw (heading/baş) açısını hesapla
  yawMag = atan2(Ym, Xm) * 180.0 / PI;

  //if (yawMag < 0) yawMag += 360;  // 0-360 derece aralığına taşı
}

void rotationCar() {
  if (outputPD > 0) {
    moveRight();
    if (outputPD > 255) {
      outputPD = 255;
    }
  } else {
    outputPD = -outputPD;
    moveLeft();
    if (outputPD > 255) {
      outputPD = 255;
    }
  }
}

void handleNavigation() {
  switch (currentState) {

    case INITIAL_ROTATION:
      targetYaw = initialTargetYaw;
      if (abs(targetYaw - yaw) <= yawThreshold) {
        currentState = FIRST_STRAIGHT;
        encoderDistance = 0;  // Düz çizgi hareketi için mesafeyi sıfırla

        writeServo(0);
        delay(2000);
        writeServo(90);
        delay(2000);
        moveForward();
      } else {
        rotationCar();
      }
      break;

    case FIRST_STRAIGHT:
      if (distanceFusion >= targetDistance) {
        currentState = SECOND_ROTATION;
        targetYaw = secondTargetYaw;
        stopMotors();

      } else {
        moveForward();
      }
      break;

    case SECOND_ROTATION:
      if (abs(targetYaw - yaw) <= yawThreshold) {
        currentState = SECOND_STRAIGHT;
        encoderDistance = 0;  // Düz çizgi hareketi için mesafeyi sıfırla

        moveForward();
      } else {

        rotationCar();
      }
      break;

    case SECOND_STRAIGHT:
      if (distanceFusion >= targetDistance) {
        currentState = THIRD_ROTATION;
        targetYaw = thirdTargetYaw;
        stopMotors();


      } else {
        moveForward();
      }
      break;

    case THIRD_ROTATION:
      if (abs(targetYaw - yaw) <= yawThreshold) {
        currentState = THIRD_STRAIGHT;
        encoderDistance = 0;  // Düz çizgi hareketi için mesafeyi sıfırla

        moveForward();
      } else {

        rotationCar();
      }
      break;

    case THIRD_STRAIGHT:
      if (distanceFusion >= targetDistance) {
        currentState = FOURTH_ROTATION;
        targetYaw = fourthTargetYaw;
        stopMotors();


      } else {
        moveForward();
      }
      break;

    case FOURTH_ROTATION:
      if (abs(targetYaw - yaw) <= yawThreshold) {
        currentState = FOURTH_STRAIGHT;
        encoderDistance = 0;  // Düz çizgi hareketi için mesafeyi sıfırla

        moveForward();
      } else {

        rotationCar();
      }
      break;

    case FOURTH_STRAIGHT:
      if (distanceFusion >= targetDistance) {
        currentState = FIFTH_ROTATION;
        targetYaw = fifthTargetYaw;
        stopMotors();


      } else {
        moveForward();
      }
      break;

    case FIFTH_ROTATION:
      if (abs(targetYaw - yaw) <= yawThreshold) {
        currentState = FIFTH_STRAIGHT;
        encoderDistance = 0;  // Düz çizgi hareketi için mesafeyi sıfırla

        moveForward();
      } else {
        rotationCar();
      }
      break;

    case FIFTH_STRAIGHT:
      if (distanceFusion >= targetDistance) {
        currentState = COMPLETED;
        stopMotors();

      } else {
        moveForward();
      }
      break;

    case COMPLETED:
      stopMotors();
      break;
  }
}


// Servo motoru istenen açıya döndürmek için PWM sinyali üretir
void writeServo(int angle) {
  // Açıyı mikrosaniye cinsinden PWM genişliğine çevir
  int pulseWidth = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);

  // PWM sinyali gönder
  for (int i = 0; i < 50; i++) {  // 20 ms döngüsü içinde 50 kez sinyal gönder (50 Hz)
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(pulseWidth);  // PWM sinyalinin "HIGH" süresi
    digitalWrite(servoPin, LOW);
    delayMicroseconds(20000 - pulseWidth);  // PWM sinyalinin "LOW" süresi
  }
}
