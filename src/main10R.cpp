#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>



//==============================
// 受信するデータ
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

DebugData receive_data;

//==============================
// 受信コールバック
//==============================
void onDataRecv(const uint8_t *mac_addr,
                const uint8_t *data,
                int data_len)
{
    memcpy(&receive_data, data, sizeof(receive_data));

Serial.println();
Serial.println("===== Position Loop =====");

Serial.print("Position        : ");
Serial.println(receive_data.position, 4);

Serial.print("Target Position : ");
Serial.println(receive_data.targetPosition, 4);

Serial.print("Position Error  : ");
Serial.println(receive_data.positionError, 4);

Serial.println();

Serial.println("===== Speed Loop =====");

Serial.print("Speed           : ");
Serial.println(receive_data.speed, 4);

Serial.print("Target Speed    : ");
Serial.println(receive_data.targetSpeed, 4);

Serial.print("Speed Error     : ");
Serial.println(receive_data.speedError, 4);

Serial.println();

Serial.println("===== Motor =====");

Serial.print("Integral        : ");
Serial.println(receive_data.speedIntegral, 4);

Serial.print("Output          : ");
Serial.println(receive_data.speedOutput, 4);

Serial.println();

Serial.println("===== Angle Loop =====");

Serial.print("Angle          : ");
Serial.println(receive_data.angle, 4);

Serial.print("Target Angle   : ");
Serial.println(0.0, 4);

Serial.print("Angle Error    : ");
Serial.println(receive_data.angleError, 4);

Serial.print("Gyro           : ");
Serial.println(receive_data.gyro, 4);

Serial.print("Angle Output   : ");
Serial.println(receive_data.angleOutput, 4);

Serial.print("Target Angle   : ");
Serial.println(receive_data.targetAngle, 4);

Serial.print("Speed Angle    : ");
Serial.println(receive_data.speedAngle, 4);

Serial.println("=========================");

}

//==============================
// ESP-NOW初期化
//==============================
void initESPNow()
{
    WiFi.mode(WIFI_STA);

    Serial.print("My MAC Address : ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("ESP-NOW Init Failed");
        return;
    }

    esp_now_register_recv_cb(onDataRecv);

    Serial.println("Receiver Ready");
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("=== ESP-NOW Receiver ===");

    initESPNow();
}

void loop()
{
    // 何もしない
}