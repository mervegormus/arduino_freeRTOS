#include <Arduino_FreeRTOS.h>

#include <Arduino_FreeRTOS.h>

void setup() {

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);

  xTaskCreate(vGorev1,
              "Gorev1",
              100,
              NULL,
              1,
              NULL
             );

  xTaskCreate(vGorev2,
              "Gorev2",
              100,
              NULL,
              2,
              NULL
             );

  xTaskCreate(vGorev3,
              "Gorev3",
              100,
              NULL,
              3,
              NULL
             );
  xTaskCreate(vGorev4,
              "Gorev4",
              100,
              NULL,
              4,
              NULL
             );

}

void loop() {
  // put your main code here, to run repeatedly:

}
void vGorev1 (void *pvParameters)
{


  for (;;)
  {

    digitalWrite(2, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    for (;;) {
      digitalWrite(2, HIGH);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      digitalWrite(2, LOW);
      vTaskDelay(3000 / portTICK_PERIOD_MS);
    }

  }

}
void vGorev2 (void *pvParameters)
{
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  digitalWrite(3, HIGH);
  for (;;)
  { digitalWrite(3, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    digitalWrite(3, LOW);
    vTaskDelay(3000 / portTICK_PERIOD_MS);

  }
}
void vGorev3 (void *pvParameters)
{
  vTaskDelay(3000 / portTICK_PERIOD_MS);
  digitalWrite(4, HIGH);
  for (;;)
  { digitalWrite(4, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    digitalWrite(4, LOW);
    vTaskDelay(3000 / portTICK_PERIOD_MS);

  }
}
void vGorev4 (void *pvParameters)
{
  vTaskDelay(4000 / portTICK_PERIOD_MS);
  digitalWrite(5, HIGH);
  for (;;)
  { digitalWrite(5, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    digitalWrite(5, LOW);
    vTaskDelay(3000 / portTICK_PERIOD_MS);

  }
}
