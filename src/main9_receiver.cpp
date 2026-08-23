#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// 受信するデータ
typedef struct struct_message {
    int value;
} struct_message;

struct_message receivedData;

// ESP-NOW受信コールバック
void OnDataRecv(const uint8_t *mac_addr,
                const uint8_t *incomingData,
                int len)
{
    // データサイズ確認
    if (len != sizeof(receivedData))
    {
        Serial.println("Received data size error");
        return;
    }

    // データをコピー
    memcpy(&receivedData, incomingData, sizeof(receivedData));

    // 送信元MACアドレスを表示
    Serial.print("Received from: ");

    for (int i = 0; i < 6; i++)
    {
        Serial.printf("%02X", mac_addr[i]);

        if (i < 5)
        {
            Serial.print(":");
        }
    }

    Serial.println();

    // 受信データを表示
    Serial.print("Value: ");
    Serial.println(receivedData.value);

    Serial.println("--------------------");
}

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("ESP-NOW Receiver Start");

    // Wi-Fi STAモード
    WiFi.mode(WIFI_STA);

    // MACアドレス表示
    Serial.print("Receiver MAC Address: ");
    Serial.println(WiFi.macAddress());

    // ESP-NOW初期化
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("ESP-NOW initialization failed!");
        return;
    }

    Serial.println("ESP-NOW initialized successfully");

    // 受信コールバック登録
    esp_now_register_recv_cb(OnDataRecv);

    Serial.println("Receiver is ready!");
}

void loop()
{
}