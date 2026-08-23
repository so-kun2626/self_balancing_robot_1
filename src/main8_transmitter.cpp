#include <WiFi.h>
#include <esp_now.h>

// 受信側ESP32のMACアドレス
uint8_t receiverAddress[] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

int counter = 0;

void setup()
{
    Serial.begin(115200);

    // Wi-Fiをステーションモードにする
    WiFi.mode(WIFI_STA);

    // ESP-NOWを開始
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("ESP-NOW Init Failed");
        return;
    }

    // 送信先を登録
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, receiverAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Failed to add peer");
        return;
    }

    Serial.println("ESP-NOW Sender Ready");
}

void loop()
{
    // 送信
    esp_err_t result = esp_now_send(
        receiverAddress,
        (uint8_t *)&counter,
        sizeof(counter)
    );

    if (result == ESP_OK)
    {
        Serial.print("Send: ");
        Serial.println(counter);
    }
    else
    {
        Serial.println("Send Failed");
    }

    counter++;

    delay(1000);
}