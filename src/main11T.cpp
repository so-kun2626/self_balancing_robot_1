#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Encoder.h>
#include <Wire.h>

//==============================
//エンコーダーの値を速度に変換するための定数
//==============================
const float WHEEL_RADIUS = 0.024;          // 半径[m]
const float GEAR_RATIO = 30.0;             // ギヤ比
const float ENCODER_PULSE = 14.0;          // モータ1回転のパルス数

const float WHEEL_CIRCUMFERENCE = 2.0 * PI * WHEEL_RADIUS;
const float DISTANCE_PER_PULSE = WHEEL_CIRCUMFERENCE / (GEAR_RATIO * ENCODER_PULSE);

//==============================
// 角度PD制御
//==============================
float KpAngle = 28.7f;
float KdAngle = 1.4f;

float angle = 0.0f;
float gyro = 0.0f;

float targetAngle = -4.5f;
float angleError = 0.0f;
float angleOutput = 0.0f;

//==============================
// 速度PID制御用の定数
//==============================
float KpSpeed = 10.6f;
float KiSpeed = 0.0f;
float speedIntegral = 0.0f;
float speedAngle = 0.0f;

//==============================
// 位置P制御用
//==============================
float KpPosition = 0.9f;

float targetPosition = 0.0f;
float position = 0.0f;
float positionError = 0.0f;



//==============================
//モーターの制御用ピン
//==============================
const int motorPin1 = 13;
const int motorPin2 = 27;
const int motorPin3 = 25;
const int motorPin4 = 26;

const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;

const int CH1 = 0;
const int CH2 = 1;
const int CH3 = 2;
const int CH4 = 3;

//==============================
//モーターの初期化
//==============================
void initMotor()
{
    ledcSetup(CH1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH2, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH3, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH4, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(motorPin1, CH1);
    ledcAttachPin(motorPin2, CH2);
    ledcAttachPin(motorPin3, CH3);
    ledcAttachPin(motorPin4, CH4);

    Serial.println("Motor Ready");
}

//==============================
// 送信するデータ
//==============================
typedef struct
{
    int64_t leftCount;
    int64_t rightCount;

    int64_t leftDiff;
    int64_t rightDiff;

   float leftSpeed;
    float rightSpeed;

    float position;
    float speed;

    float targetPosition;
    float positionError;
    float targetSpeed;

    float speedError;
    float speedIntegral;
    float speedOutput;

    float angle;
    float gyro;
    float angleOutput;
    float angleError;

    float targetAngle;
    float speedAngle;
    uint32_t time;

} DebugData;

DebugData send_data;
esp_now_peer_info_t peerInfo;
ESP32Encoder encoderL;
ESP32Encoder encoderR;

const int ENCODER_A_L = 34;
const int ENCODER_B_L = 35;
const int ENCODER_A_R = 32;
const int ENCODER_B_R = 33;

int64_t lastCountL = 0;
int64_t lastCountR = 0;

// 受信機のMACアドレス
uint8_t receiverAddress[] =
{
    0x94, 0x51, 0xDC, 0x2D, 0xB4, 0xD8
};

//==============================
// 送信完了時のコールバック
//==============================
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    Serial.print("Send Status : ");

    if(status == ESP_NOW_SEND_SUCCESS)
    {
        Serial.println("Success");
    }
    else
    {
        Serial.println("Fail");
    }
}

void initESPNow()
{
    WiFi.mode(WIFI_STA);

    Serial.print("My MAC Address : ");
    Serial.println(WiFi.macAddress());

    if(esp_now_init() != ESP_OK)
    {
        Serial.println("ESP-NOW Init Failed");
        return;
    }

    memcpy(peerInfo.peer_addr, receiverAddress, 6);

    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if(esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Add Peer Failed");
        return;
    }

    esp_now_register_send_cb(onDataSent);

    Serial.println("Transmitter Ready");
}

void driveMotor(int pwm)
{
    pwm = constrain(pwm, -255, 255);

    if(pwm > 0)
    {
        ledcWrite(CH1, pwm);
        ledcWrite(CH2, 0);

        ledcWrite(CH3, pwm);
        ledcWrite(CH4, 0);
    }
    else if(pwm < 0)
    {
        ledcWrite(CH1, 0);
        ledcWrite(CH2, -pwm);

        ledcWrite(CH3, 0);
        ledcWrite(CH4, -pwm);
    }
    else
    {
        ledcWrite(CH1, 0);
        ledcWrite(CH2, 0);

        ledcWrite(CH3, 0);
        ledcWrite(CH4, 0);
    }
}

#define MPU_ADDR 0x68


void initMPU()
{
    Wire.begin();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);  // PWR_MGMT_1
    Wire.write(0);
    Wire.endTransmission(true);


    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission(true);


    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission(true);


    Serial.println("MPU6050 Ready");
}

float accX, accY, accZ;
float gyroX;

void updateAngle(float dt)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);

    Wire.requestFrom(MPU_ADDR,14,true);
    if (Wire.available() != 14)
    {
        return;
    }

    int16_t ax = Wire.read()<<8 | Wire.read();
    int16_t ay = Wire.read()<<8 | Wire.read();
    int16_t az = Wire.read()<<8 | Wire.read();

    Wire.read();
    Wire.read();

    int16_t gx = Wire.read()<<8 | Wire.read();
    int16_t gy = Wire.read()<<8 | Wire.read();
    int16_t gz = Wire.read()<<8 | Wire.read();


    float accAngle =
    atan2(ax,az)
    *180.0/PI;


    gyro=-gy/65.5;


    


    angle =
    0.98*(angle + gyro*dt)
    +0.02*accAngle;
}

void sendDebugData()
{
    const float dt = 0.005f;
    int64_t currentCountL = encoderL.getCount();
    int64_t currentCountR = encoderR.getCount();

    send_data.leftCount = currentCountL;
    send_data.rightCount = currentCountR;

    send_data.leftDiff = currentCountL - lastCountL;
    send_data.rightDiff = currentCountR - lastCountR;

    send_data.leftSpeed = send_data.leftDiff * DISTANCE_PER_PULSE / dt;
    send_data.rightSpeed = send_data.rightDiff * DISTANCE_PER_PULSE / dt;

    position = (currentCountL + currentCountR ) *0.5f *DISTANCE_PER_PULSE;

    send_data.position = position;
    send_data.speed = (send_data.leftSpeed + send_data.rightSpeed) / 2.0f;
    send_data.time = millis();

    send_data.targetPosition = targetPosition;

    positionError = targetPosition - position;

    send_data.positionError = positionError;

    send_data.targetSpeed = KpPosition * positionError;

    send_data.targetSpeed = constrain( send_data.targetSpeed, -0.2f, 0.2f );

    //==============================
    // 速度PI制御
    //==============================
    send_data.speedError = send_data.targetSpeed - send_data.speed;

    speedIntegral += send_data.speedError * dt;
    speedIntegral = constrain(speedIntegral, -5.0f, 5.0f);

    speedAngle = KpSpeed * send_data.speedError + KiSpeed * speedIntegral;
    speedAngle = constrain(speedAngle, -2.0f, 2.0f);
    
    //==============================
    // 姿勢制御
    //==============================
    updateAngle(dt);

    float finalTargetAngle = targetAngle - speedAngle;

    angleError = finalTargetAngle - angle;
    angleOutput = KpAngle * angleError - KdAngle * gyro;

    send_data.angle = angle;
    send_data.gyro = gyro;
    send_data.angleOutput = angleOutput;
    send_data.angleError = angleError;

    send_data.targetAngle = finalTargetAngle;
    send_data.speedAngle = speedAngle;

    send_data.speedIntegral = speedIntegral;

    send_data.speedOutput = constrain(angleOutput, -255.0f, 255.0f);
    driveMotor((int)send_data.speedOutput);

    lastCountL = currentCountL;
    lastCountR = currentCountR;
    

    esp_now_send(receiverAddress,
                 (uint8_t *)&send_data,
                 sizeof(send_data));
}
void initEncoder()
{
    ESP32Encoder::useInternalWeakPullResistors = puType::up;

    encoderL.attachHalfQuad(
        ENCODER_A_L,
        ENCODER_B_L);

    encoderR.attachHalfQuad(
        ENCODER_A_R,
        ENCODER_B_R);

    encoderL.setCount(0);
    encoderR.setCount(0);

    Serial.println("Encoder Ready");
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("=== ESP-NOW Transmitter ===");

    initESPNow();
    initEncoder();
    initMotor();
    initMPU();
}

const uint32_t CONTROL_PERIOD_US = 5000;   // 5ms = 200Hz

void loop()
{
    static uint32_t nextTime = micros();

    if ((int32_t)(micros() - nextTime) >= 0)
    {
        nextTime += CONTROL_PERIOD_US;
        sendDebugData();
    }
}