#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <LiquidCrystal.h>


LiquidCrystal lcd(0, 1, 4, 5, 6, 7);

QueueHandle_t xQueue_Lm35;
QueueHandle_t xQueue_Gaz;

bool durum = false;
void setup() {
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);

  xQueue_Gaz = xQueueCreate(5, sizeof(int32_t));
  xQueue_Lm35 = xQueueCreate(5, sizeof(int32_t));

  lcd.begin(16, 2);
  xTaskCreate(vGorevLED1,
              "Gorev1",
              100,
              NULL,
              1,
              NULL);
  xTaskCreate(vGorevLED2,
              "Gorev2",
              100,
              NULL,
              2,
              NULL);
  xTaskCreate(vGorevLED3,
              "Gorev3",
              100,
              NULL,
              3,
              NULL);
  xTaskCreate(vGorevLED4,
              "Gorev4",
              100,
              NULL,
              4,
              NULL);
  xTaskCreate(vSenderLm35,
              "vSenderLm35",
              128,
              NULL,
              5,
              NULL);
  xTaskCreate(vSenderGaz,
              "vSenderGaz",
              128,
              NULL,
              6,
              NULL);
  xTaskCreate(vLCDYazdir,
              "vLCDYazdir",
              200,
              NULL,
              7,
              NULL);
}

void loop() {
  // put your main code here, to run repeatedly:

}
void vGorevLED1(void *pvParameters)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  Serial.begin(9600);
  (void) pvParameters;
  for (;;)
  {
    if (durum == false)
    {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    digitalWrite(2, LOW);
    Serial.println("Görev1");
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    digitalWrite(2, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    durum = true;
  }
}

void vGorevLED2(void *pvParameters)
{

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  Serial.begin(9600);
  (void) pvParameters;
  for (;;)
  {
    if (durum == false)
    {
      vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    digitalWrite(3, LOW);
    Serial.println("Görev2");
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    digitalWrite(3, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    durum = true;
  }
}
void vGorevLED3(void *pvParameters)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  Serial.begin(9600);
  (void) pvParameters;
  for (;;)
  {
    if (durum == false)
    {
      vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
    digitalWrite(8, LOW);
    Serial.println("Görev3");
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    digitalWrite(8, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    durum = true;
  }
}

void vGorevLED4(void *pvParameters)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  Serial.begin(9600);
  (void) pvParameters;
  for (;;)
  {
    if (durum == false)
    {
      vTaskDelay(4000 / portTICK_PERIOD_MS);
    }
    digitalWrite(9, LOW);
    Serial.println("Görev4");
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    digitalWrite(9, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    durum = true;
  }
}
void vSenderLm35(void *pvParameters) {
  int32_t okunanDeger;
  BaseType_t xDurum ;
  for (;;) {
    okunanDeger = analogRead(A15);
    okunanDeger = (okunanDeger / 1024.0) * 500;
    xDurum = xQueueSendToBack(xQueue_Lm35, &okunanDeger, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if (xDurum == pdPASS)
    {
      Serial.print("LM35: ");
      Serial.println(okunanDeger);
    }
    else
    {
      Serial.println("Kuyruğa yazdırılamadı lm 35 ");
    }
    taskYIELD();

  }

}
void vSenderGaz(void *pvParameters) {
  int32_t sensorValue;
  BaseType_t xDurum ;
  for (;;) {
    sensorValue = analogRead(A0);
    xDurum = xQueueSendToBack(xQueue_Gaz, &sensorValue, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if (xDurum == pdPASS)
    {
      if (sensorValue < 300) {
        Serial.print("Gaz Sensor: ");
        Serial.println(sensorValue);
      }
      else{
        Serial.print("Gaz Sensoru Esik Degeri Astı: ");
        Serial.println(sensorValue);
      }
    }
    else
    {
      Serial.println("Kuyruğa yazdırılamadı gaz ");
    }
    taskYIELD();
  }
}
void vLCDYazdir(void *pvParameters)
{
  int32_t ReceivedValue_Gaz = 0;
  int32_t ReceivedValue_Lm35 = 0;
  for (;;)
  {
    if (xQueueReceive (xQueue_Gaz, &ReceivedValue_Gaz, portMAX_DELAY) == pdPASS)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Gaz: ");
      lcd.setCursor(5, 0);
      lcd.print(ReceivedValue_Gaz);
    }
    if (xQueueReceive (xQueue_Lm35, &ReceivedValue_Lm35, portMAX_DELAY) == pdPASS)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("LM35: ");
      lcd.setCursor(6, 1);
      lcd.print(ReceivedValue_Lm35);
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    taskYIELD();
  }
}
