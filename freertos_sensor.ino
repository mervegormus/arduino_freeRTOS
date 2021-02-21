#include <Arduino_FreeRTOS.h>
#include <Servo.h>
#include <avr/sleep.h>
#include <queue.h>
#include <MLX90615.h>
#include <I2cMaster.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <LCD.h>

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3 ,  POSITIVE); //kullanılan portları belirliyoruz


#define SDA_PIN 44
#define SCL_PIN 43

SoftI2cMaster i2c(SDA_PIN , SCL_PIN);
MLX90615 mlx90615(DEVICE_ADDR, &i2c);

//kuyruk tanımlaması yapıyoruz şimdi
QueueHandle_t xQueue_Nesne;
QueueHandle_t xQueue_Ortam;
//QueueHandle_t xQueue_Lm35;
QueueHandle_t xQueue_Gaz;


Servo servo1;
Servo servo2;
Servo servo3;

bool durum = false;

//parametre gönderimi
static const char*servoSec1 = "Servo1";
static const char*servoSec2 = "Servo2";

//öncelik değiştirme
TaskHandle_t xTaskLED2Handle = NULL;


//öncelik değiştirme handle ile olur bir ledin önceliğini değiştirelim mesela
//TaskHandle_t xTaskLED2Handle = NULL;


//----boşta göre için
volatile uint32_t ulIdleCycleCount = 0ul;
ISR(INT0_vect) {
  //Boş olmasına rağmen bu interrupt fonksiyonu kaldırılmamalı
}

/*void vGorev1(void *pvParameters);
  void vGorev2(void *pvParameters);
  void vGorev3(void *pvParameters);
  void vGorev4(void *pvParameters);
*/




void setup() {
  pinMode(2, OUTPUT); //led1
  pinMode(3, OUTPUT); //led2
  pinMode(4, OUTPUT); //led3
  pinMode(5, OUTPUT); //led4

  /* pinMode(9,OUTPUT);//servo1
     pinMode(9,OUTPUT);//servo2

  */

  //kuyruk oluşturlım
  xQueue_Nesne = xQueueCreate(5, sizeof(int32_t));
  xQueue_Ortam = xQueueCreate(5, sizeof(int32_t));
  // xQueue_Lm35 = xQueueCreate(5, sizeof(int32_t));
  xQueue_Gaz = xQueueCreate(5, sizeof(int32_t));

  lcd.begin(16, 2); //lcd belirledik
  lcd.backlight();  //lcd nin arka ışığını oluşturmak için

  xTaskCreate(vGorevLED1, //1.ledi kontrol eder..
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
              &xTaskLED2Handle); //BUNA ÖNECELİK KOYDUK BURASI NULL OLMAYACAK , &xTaskLED2Handle); olacak
  //çalışma esnasında
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
  xTaskCreate(vGorevSERVO,
              "Gorev5",
              128,
              (void *)servoSec1,
              5,
              NULL);
  xTaskCreate(vGorevSERVO,
              "Gorev6",
              128,
              (void *)servoSec2,
              6,
              NULL);

  //kuyruğa veri aktaran veri çeken gorev
  xTaskCreate(vSenderSicaklik_Nesne,
              "vSenderSicaklik",
              128,
              NULL,//içerde okuma yapacağı için herhangi bir parametre göndermiyoruz
              7,
              NULL);
  /* xTaskCreate(vSenderLm35,
               "vSenderLm35",
               128,
               NULL,//içerde okuma yapacağı için herhangi bir parametre göndermiyoruz
               8,
               NULL);*/
  xTaskCreate(vSenderSicaklik_Ortam,
              "vSenderSicaklik",
              128,
              NULL,//içerde okuma yapacağı için herhangi bir parametre göndermiyoruz
              9,
              NULL);
  xTaskCreate(vSenderGaz,
              "vSenderGaz",
              128,
              NULL,
              10,
              NULL);
  //LCD gorevİ
  xTaskCreate(vLCDYazdir,
              "vLCDYazdir",
              200,
              NULL,
              11,
              NULL);


  servo1.attach(9);
  servo1.attach(10);


}

void loop() {
  //loop her zaman boş

}

void vApplicationIdleHook(void)
{
  //ulIdleCycleCount++;
  //Serial.println(ulIdleCycleCount);
  //uykuModu();

}



void vGorevLED1(void *pvParameters)
{
  // TickType_t xLastWakeTime1;
  //TickType_t xLastWakeTime2;
  //  unsigned long m1;
  // unsigned long m2;

  Serial.begin(9600);
  (void) pvParameters;
  for (;;)
  { //xLastWakeTime1 = xTaskGetTickCount();
    // m1 = millis();
    if (durum == false)
    {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    digitalWrite(2, LOW);
    Serial.print("Gorev1\t");
    Serial.print("1");
    Serial.println();
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    //vTaskDelayUntil (&xLastWakeTime, 1000/ portTICK_PERIOD_MS);

    digitalWrite(2, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    //vTaskDelayUntil (&xLastWakeTime, 4000/ portTICK_PERIOD_MS);
    durum = true;
    // m2 = millis();
    //Serial.println("led1zaman");
    //Serial.println(m2 - m1);
    /* xLastWakeTime2 = xTaskGetTickCount();
      Serial.println("sensor led1" );
      Serial.println(xLastWakeTime2 - xLastWakeTime1);*/
  }
}

void vGorevLED2(void *pvParameters)
{
  /* öncelik tanımaya devam ediyoruz
    UBaseType_t uxPriority; diye bir değişken tanımlamalıyız
  */
  UBaseType_t uxPriority;
  // TickType_t xLastWakeTime1;
  // TickType_t xLastWakeTime2;
  // unsigned long m1;
  //unsigned long m2;

  uint16_t ui = 0;
  /*öncelik tanımlamış olsak
    uint16_t ui;
  */

  Serial.begin(9600);
  (void) pvParameters;
  for (;;)

  { // xLastWakeTime1 = xTaskGetTickCount();
    // m1 = millis();
    //öncelik tanımlamaya devam ediyoruz   (ui++;)  ekleriz buraya
    ui++;
    if (durum == false)
    {
      vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    digitalWrite(3, LOW);
    Serial.print("Gorev2\t");
    Serial.print("1");
    Serial.println();
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    //vTaskDelayUntil (&xLastWakeTime, 1000/ portTICK_PERIOD_MS);

    digitalWrite(3, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    //vTaskDelayUntil (&xLastWakeTime, 4000/ portTICK_PERIOD_MS);
    durum = true;
    /*xLastWakeTime2 = xTaskGetTickCount();
      Serial.println("sensor led2" );
      Serial.println(xLastWakeTime2 - xLastWakeTime1);*/
    //m2 = millis();
    //Serial.println("led2zaman");
    // Serial.println(m2 - m1);
    /*öncelik tanımlaya devam ediyoruz
      if (ui==5){
      ui=0;
      vTaskPrioritySet(xTaskLED2Handle,(uxPriority+ui)); önceliğini arttırdık böyle
      Serial.println("LED2 önceliği değişti");

    */
  }
}

void vGorevLED3(void *pvParameters)
{
  // TickType_t xLastWakeTime1;
  // TickType_t xLastWakeTime2;
  //unsigned long m1;
  // unsigned long m2;
  Serial.begin(9600);
  (void) pvParameters;
  for (;;)
  { //xLastWakeTime1 = xTaskGetTickCount();
    //m1 = millis();
    if (durum == false)
    {
      vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
    digitalWrite(4, LOW);
    Serial.print("Gorev3\t");;
    Serial.print("1");
    Serial.println();
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    //vTaskDelayUntil (&xLastWakeTime, 1000/ portTICK_PERIOD_MS);

    digitalWrite(4, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    //vTaskDelayUntil (&xLastWakeTime, 4000/ portTICK_PERIOD_MS);
    durum = true; // başa döndüğü zaman durumu tru yapıyoruz  hıgh ile dönüş yapınca low a çıkıyor
    /*  xLastWakeTime2 = xTaskGetTickCount();
      Serial.println("sensor led3" );
      Serial.println(xLastWakeTime2 - xLastWakeTime1);*/
    //m2 = millis();
    //Serial.println("led3zaman");
    //Serial.println(m2 - m1);
  }
}

void vGorevLED4(void *pvParameters)
{
  // unsigned long m1;
  // unsigned long m2;

  Serial.begin(9600);
  (void) pvParameters;
  for (;;)
  { //m1 = millis();
    if (durum == false)
    {
      vTaskDelay(4000 / portTICK_PERIOD_MS);
    }
    digitalWrite(5, LOW);
    Serial.print("Gorev4\t");
    Serial.print("1");
    Serial.println();
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    //vTaskDelayUntil (&xLastWakeTime, 1000/ portTICK_PERIOD_MS);

    digitalWrite(5, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    //vTaskDelayUntil (&xLastWakeTime, 4000/ portTICK_PERIOD_MS);
    durum = true;
    // m2 = millis();
    //  Serial.println("led4zaman");
    // Serial.println(m2 - m1);
  }
}

/*void vGorevServo(void *pvParameters) //parametresiz hali
  {

  for(;;)
  {
    servo1.write(100); //100 DERECELİK AÇIYA GETİRİYOR MOTORU SONRA 20 DERECEYE ÇEKİYOR
    vTaskDelay(1000/portTICK_PERIOD_MS);
    Servo1.write(20); //1 SANİYE SONRA 20 DERECELİK AÇIYLA GELECEK
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  }*/

//şimdi parametre göndererek yapalım bunun bunun için en yukarıya static olarak tanımladık

void vGorevSERVO(void *pvParameters)
{
  // unsigned long m1;
  // unsigned long m2;
  char *pcTaskName;

  Serial.begin(9600);
  pcTaskName = (void*)pvParameters; //tek gorev birden fazla gorev alacak
  int32_t acidegeri = 0;
  for (;;)
  { //m1 = millis();
    Serial.print(pcTaskName);
    Serial.print("\t");
    if (pcTaskName == "Servo1") {
      acidegeri = 100;
      servo1.write(acidegeri);
      Serial.print( acidegeri);
      Serial.println();
    }

    if (pcTaskName == "Servo2") {
      acidegeri = 180;
      servo2.write(acidegeri);
      Serial.print(acidegeri);
      Serial.println();

    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.print(pcTaskName);
    Serial.print("\t");
    if (pcTaskName == "Servo1") {
      acidegeri = 20;
      servo1.write(acidegeri);
      Serial.print(acidegeri);
      Serial.println();
    }
    if (pcTaskName == "Servo2") {
      acidegeri = 0;
      servo2.write(acidegeri);
      Serial.print(acidegeri);
      Serial.println();
    }
    // m2 = millis();
    // Serial.println("servo");
    // Serial.println(m2 - m1);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
/*void vSenderLm35(void *pvParameters) {
  int32_t okunanDeger;
  BaseType_t xDurum ;
  for (;;) {
    okunanDeger = analogRead(A15);
    okunanDeger = (okunanDeger / 1024.0) * 500;
    xDurum = xQueueSendToBack(xQueue_Lm35, &okunanDeger, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if (xDurum == pdPASS)
    {
      Serial.print("LM35 ");
      Serial.println(okunanDeger);
    }
    else
    {
      Serial.println("Kuyruğa yazdırılamadı lm 35 ");
    }
    taskYIELD(); //kuyruğa yazdırıldı artık başka gorevlere git onlar çalışsın mantığı

  }

  }*/
void vSenderGaz(void *pvParameters) {
  // unsigned long m1;
  // unsigned long m2;

  int32_t sensorValue;
  BaseType_t xDurum ;
  for (;;) {
    // m1 = millis();
    sensorValue = analogRead(A0);
    xDurum = xQueueSendToBack(xQueue_Gaz, &sensorValue, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if (xDurum == pdPASS)
    {
      if (sensorValue < 300) {
        Serial.print("GazSensor\t");
        Serial.print(sensorValue);
        Serial.println();
      }
      else {
        Serial.print("GazSensoruEsikDegeriAsti\t");
        Serial.print(sensorValue);
        Serial.println();
      }
    }
    else
    {
      Serial.println("Kuyruga yazdirilamadi gaz ");
    }
    // m2 = millis();
    // Serial.println("gsz");
    //  Serial.println(m2 - m1);
    taskYIELD();
  }
}
void vSenderSicaklik_Nesne(void *pvParameters)
{
  int32_t yazdirilacakDeger;
  BaseType_t xDurum ;
  // TickType_t xLastWakeTime1;
  // TickType_t xLastWakeTime2;

  for (;;)
  { // xLastWakeTime1 = xTaskGetTickCount();
    yazdirilacakDeger = mlx90615.getTemperature(MLX90615_OBJECT_TEMPERATURE); //nesenenin sıcaklığını okuyan fonk
    xDurum = xQueueSendToBack(xQueue_Nesne, &yazdirilacakDeger, 0); //kuyruktan veri alıyoruz
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if (xDurum == pdPASS)
    {
      Serial.print("MLX90615Nesne\t");
      Serial.print(yazdirilacakDeger);
      Serial.println();
    }
    else
    {
      Serial.println("Kuyruga yazdirilamadi ");
    }
    //xLastWakeTime2 = xTaskGetTickCount();
    //Serial.println("sensor nesne" );
    //Serial.println(xLastWakeTime2 - xLastWakeTime1);
    taskYIELD(); //kuyruğa yazdırıldı artık başka gorevlere git onlar çalışsın mantığı
  }
}


void vSenderSicaklik_Ortam(void *pvParameters)
{ //TickType_t xLastWakeTime1;
  //TickType_t xLastWakeTime2;

  int32_t yazdirilacakDeger;
  BaseType_t xDurum ;

  for (;;)
  { //xLastWakeTime1 = xTaskGetTickCount();
    yazdirilacakDeger = mlx90615.getTemperature(MLX90615_AMBIENT_TEMPERATURE); //nesenenin sıcaklığını okuyan fonk
    xDurum = xQueueSendToBack(xQueue_Ortam, &yazdirilacakDeger, 0); //kuyruktan veri alıyoruz
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if (xDurum == pdPASS)
    {
      Serial.print("MLX90615Ortam\t");
      Serial.print(yazdirilacakDeger);
      Serial.println();
    }
    else
    {
      Serial.println("Kuyruga yazdirilamadi ");
    }
    // xLastWakeTime2 = xTaskGetTickCount();
    // Serial.println("sensor ortam" );
    //Serial.println(xLastWakeTime2 - xLastWakeTime1);
    taskYIELD(); //kuyruğa yazdırıldı artık başka gorevlere git onlar çalışsın mantığı
  }
}


void vLCDYazdir(void *pvParameters)
{
  int32_t ReceivedValue_Nesne = 0;
  int32_t ReceivedValue_Ortam = 0;
  //  int32_t ReceivedValue_Lm35 = 0;
  int32_t ReceivedValue_Gaz = 0;
  // unsigned long m1;
  //  unsigned long m2;

  for (;;)
    //setcorsorları ayarlaman lazım değişiklik yapıldı.
  { //m1 = millis();
    if (xQueueReceive (xQueue_Nesne, &ReceivedValue_Nesne, portMAX_DELAY) == pdPASS)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Nesne ");
      lcd.setCursor(7, 0);
      lcd.print(ReceivedValue_Nesne);
    }
    /* if (xQueueReceive (xQueue_Lm35, &ReceivedValue_Lm35, portMAX_DELAY) == pdPASS)
      {
       lcd.clear();
       lcd.setCursor(0, 0);
       lcd.print("LM35 ");
       lcd.setCursor(6, 1);
       lcd.print(ReceivedValue_Lm35);
      }*/
    if (xQueueReceive (xQueue_Ortam, &ReceivedValue_Ortam, portMAX_DELAY) == pdPASS)
    {
      //clear olmaz burda
      lcd.setCursor(0, 0);
      lcd.print("Ortam ");
      lcd.setCursor(10, 0);
      lcd.print(ReceivedValue_Ortam);
    }
    if (xQueueReceive (xQueue_Gaz, &ReceivedValue_Gaz, portMAX_DELAY) == pdPASS)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Gaz ");
      lcd.setCursor(10, 1);
      lcd.print(ReceivedValue_Gaz);
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    // m2 = millis();
    // Serial.println("sensor lcd" );
    //  Serial.println(m2 - m1);
    taskYIELD();
  }
}
