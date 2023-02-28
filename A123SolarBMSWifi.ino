/* A123 BMS */
/*

   |__| \    / o
   |  |  \/\/  o

 GND--- S -----.___      .---- D ---[BATT-]
                |__\_ __|
                 _______                      ___________________
                    |     _________          |    ARDUINO UNO    |         _____________
    +12V          GATE 0 | ULN2803 |  +12V---| VIN               |        |   MCP2515   |
     |	            |    |         |         |               +5V |--------| VCC         |
     0--| R2k2 |----0----| 18    1 |<--------| A0/14          18 |------->| SCK         |_      _
     |                   |         |         |                19 |<-------| MISO   CANH | \/\/\/
     |           GATE 1  |         |         |                23 |------->| MOSI   CANL |_/\/\/\_
     |              |    |         |         |                 5 |------->| CS          |
     0--| R2k2 |----0----| 17    2 |<--------| A1/15          21 |<-------| INT         |
     |                   |         |         |               GND |--------| GND         |
     |           GATE 2  |         |         |                   |        |_____________|
     |              |    |         |         |                   |
     0--| R2k2 |----0----| 16    3 |<--------| A2/16             |                       //RED
     |                   |         |         |                 7 |------->| R560 |----| LED |------0
     |           GATE 3  |         |         |                   |                       //YELLOW  |
     |              |    |         |         |                 6 |------->| R560 |----| LED |------0
     0--| R2k2 |----0----| 15    4 |<--------| A3/17             |                       //GREEN   |
     |                   |         |         |                 5 |------->| R560 |----| LED |------0
     0-------------------| 10    9 |----0----| GND               |                                 |
                         |_________|    |    |                 1 |------->| TX                     |
                                        |    |                 0 |<-------| RX                     |
                                       _|_   |___________________|                                _|_
                                       GND                                                        GND
      __
     (__ \    / o
     ___) \/\/  o

Grobe Funktionsbeschreibung:

StartUp:
Nach den nötigen Initialisierungroutinen wird  über den CAN-Bus eine Nachricht mit der ID 0x50 verschickt.
Das versenden der Nachricht erledigt die Methode "send_balance()" der Klasse "A123Module"
Alle Angeschlossenen A123 Module sollten daraufhin antworten.
Die Methode "Decode_CAN()" decocodiert die NAchrichten und legt alle Module und Signale in der Struktur "modules" ab.

*/


#include "WiFi.h"
#include <Arduino_JSON.h>
#include <EspMQTTClient.h>
#include <ACAN2515.h>
#include "A123.h"
#include "A123SolarBMSWifi.h"
#include <ESP32Time.h>
#include <math.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

#define ABSMAXVOLTAGE 3650
#define ABSMINVOLTAGE 2100
#define BMSNUMBER 0  //edit vor every BMS
#define MYMQTTTOPIC "tele/bms/0/"

#define RedLED 14
#define YellowLED 27
#define GreenLED 16

//define Tasks

void TaskCANSetup(void *pvParameters);
void TaskCANRead(void *pvParameters);
void TaskCANWrite(void *pvParameters);
void TaskPublishMQTT(void *pvParameters);

TaskHandle_t TaskCANSetupHandle;
TaskHandle_t TaskCANReadHandle;
TaskHandle_t TaskCANWriteHandle;
TaskHandle_t TaskPublishMQTTHandle;

QueueHandle_t rxMailbox;
QueueHandle_t txMailbox;



struct canRxMessages_t {
  struct A123_m01_n_std_t module_std[15];
  struct A123_m01_n_xd1_t module_xd1[15];
  struct A123_m01_n_xd2_t module_xd2[15];
  uint8_t modulesFound[15];
  /*struct m02p_t m02;
  struct m03n_t m03;
  struct m04p_t m04;
  struct m05n_t m05;
  struct m06p_t m06;
  struct m07n_t m07;
  struct m08p_t m08;
  struct m09n_t m09;
  struct m10p_t m10;
  struct m11n_t m11;
  struct m12p_t m12;
  struct m13n_t m13;
  struct m14p_t m14;
  struct m15n_t m15;*/
};

struct canTxMessages_t {
  struct A123_bcm_cmd_t bcm_cmd;
  struct A123_bcm_diag_req_t bcm_diag_req;
};

static const byte MCP2515_CS = 5;                                // CS input of MCP2515 (adapt to your design)
static const byte MCP2515_INT = 26;                              // INT output of MCP2515 (adapt to your design)
static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL;  // 8 MHz

ACAN2515 can(
  MCP2515_CS,
  SPI,
  MCP2515_INT);

EspMQTTClient client(
  "192.168.178.168",  // MQTT Broker server ip
  1883,
  "BMS0Client"  // Client name that uniquely identify your device
);

ESP32Time rtc(3600);  // offset in seconds GMT+1


void onConnectionEstablished() {
  // Subscribe to "mytopic/test" and display received message to Serial
  /*client.subscribe("tele/batterie/VOLTAGES", onMQTTbmsReceived);

  client.subscribe("stat/battery/PARAMETER", onMQTTbatteryReceive);

  client.subscribe("cmnd/battery/PARAMETER", onMQTTbatteryReceive);
  */
}

// Replace with your network credentials (STATION)
const char *ssid = "Martin Routher King";
const char *password = "I have a stream!";


void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  //wifi_station_set_hostname( hostname.c_str() );
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    digitalWrite(RedLED, LOW);
    delay(500);
    digitalWrite(RedLED, HIGH);
    delay(500);
  }
  Serial.println(WiFi.localIP());
  digitalWrite(RedLED, LOW);
}

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RedLED, OUTPUT);
  pinMode(YellowLED, OUTPUT);
  pinMode(GreenLED, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  //Alle LEDs an!
  digitalWrite(RedLED, HIGH);
  digitalWrite(YellowLED, HIGH);
  digitalWrite(GreenLED, HIGH);
  delay(1000);
  digitalWrite(GreenLED, LOW);


  Serial.begin(115200);
  rxMailbox = xQueueCreate(1, sizeof(struct canRxMessages_t));
  txMailbox = xQueueCreate(1, sizeof(struct canTxMessages_t));
  initWiFi();
  client.enableDebuggingMessages();  // Enable debugging messages sent to serial output
  client.enableHTTPWebUpdater();     // Enable the web updater. User and password default to values of MQTTUsername and MQTTPassword. These can be overridded with enableHTTPWebUpdater("user", "password").
  client.enableOTA();                // Enable OTA (Over The Air) updates. Password defaults to MQTTPassword. Port is the default OTA port. Can be overridden with enableOTA("password", port).

  // Now set up tasks to run independently.

  xTaskCreatePinnedToCore(
    TaskCANSetup, "TaskCANSetup", 4096  // Stack size
    ,
    NULL, configMAX_PRIORITIES - 10  // Priority
    ,
    &TaskCANSetupHandle, 0);

  xTaskCreatePinnedToCore(
    TaskCANRead, "TaskCANRead", 4096  // Stack size
    ,
    NULL, configMAX_PRIORITIES - 11  // Priority
    ,
    &TaskCANReadHandle, 0);

  xTaskCreatePinnedToCore(
    TaskCANWrite, "TaskCANWrite", 4096  // Stack size
    ,
    NULL, configMAX_PRIORITIES - 12  // Priority
    ,
    &TaskCANWriteHandle, 0);

  xTaskCreatePinnedToCore(
    TaskPublishMQTT, "TaskPublishMQTT", 4096  // Stack size
    ,
    NULL, configMAX_PRIORITIES - 8  // Priority
    ,
    &TaskPublishMQTTHandle, 0);
}
void loop() {
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/


void TaskCANSetup(void *pvParameters) {
  (void)pvParameters;
  //setup
  //init the SPI
  SPI.begin();
  ACAN2515Settings settings(QUARTZ_FREQUENCY, 500UL * 1000UL);  // CAN bit rate 500 kb/s
  settings.mRequestedMode = ACAN2515Settings::NormalMode;       // Select loopback mode
  const uint16_t errorCode = can.begin(settings, [] {
    can.isr();
  });

  if (errorCode == 0) {
    Serial.println("CAN Configuration OK!");
    //Wait 100ms and wake up the CAN Guys
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(YellowLED, LOW);
    vTaskResume(TaskCANReadHandle);
    vTaskResume(TaskCANWriteHandle);

  } else {

    //we have a problem...
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(RedLED, HIGH);
    Serial.print("Configuration error 0x");
    Serial.println(errorCode, HEX);
  }
  //my Work is done here, I go to sleep
  vTaskSuspend(NULL);
  for (;;) {
  }
}

void TaskCANRead(void *pvParameters) {
  (void)pvParameters;
  TickType_t xLastWakeTime;
  struct canRxMessages_t myCanRxMessages;
  static CANMessage frame;
  uint8_t moduleNumber = 0;
  uint8_t i = 0;
  uint16_t timeout = 1000;
  uint16_t counter = 0;
  struct A123_m01_n_std_t module_std[15];
  struct A123_m01_n_xd1_t module_xd1[15];
  struct A123_m01_n_xd2_t module_xd2[15];

  //Wait until CAN is initialized -> TASKCANSetup will wake me up
  vTaskSuspend(NULL);
  memset(&myCanRxMessages, 0, sizeof(myCanRxMessages));
  xLastWakeTime = xTaskGetTickCount();
  for (;;)  // A Task shall never return or exit.
  {
    counter++;
    while (can.available()) {
      can.receive(frame);
      counter=0;
      //entpacke die Frames in die Struktur
      moduleNumber = (frame.id & 0x00F);
      myCanRxMessages.modulesFound[moduleNumber] = 1;
      if ((frame.id & 0xFF0) == 0x200) {
        counter++;
        A123_m01_n_std_unpack(&module_std[moduleNumber], frame.data, 8);
        myCanRxMessages.module_std[moduleNumber] = module_std[moduleNumber];
      }
      if ((frame.id & 0xFF0) == 0x300) {
        A123_m01_n_xd1_unpack(&module_xd1[moduleNumber], frame.data, 8);
        myCanRxMessages.module_xd1[moduleNumber] = module_xd1[moduleNumber];
      }
      if ((frame.id & 0xFF0) == 0x380) {
        A123_m01_n_xd2_unpack(&module_xd2[moduleNumber], frame.data, 8);
        myCanRxMessages.module_xd2[moduleNumber] = module_xd2[moduleNumber];
      }
      //Teil es mit deinen Kumpels
      if (i > 16) {
        xQueueOverwrite(rxMailbox, &myCanRxMessages);
      } else {
        i++;
      }
    }
    if (counter > 1000) {
      digitalWrite(RedLED, HIGH);
      counter=1000;
    }
    else{
      digitalWrite(RedLED, LOW);
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
  }
}

void TaskCANWrite(void *pvParameters) {

  (void)pvParameters;
  int i = 0;
  static uint8_t local_bcm_request_id = 0;
  TickType_t xLastWakeTime;
  static CANMessage frame;
  struct A123_bcm_cmd_t A123_bcm_cmd;
  struct A123_m01_n_std_t A123_m01_n_std;

  //struct canTxMessages_t myCanTxMessages;

  vTaskSuspend(NULL);
  vTaskDelay(100 / portTICK_PERIOD_MS);

  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    //xQueuePeek(txMailbox, &myCanTxMessages, 10);
    local_bcm_request_id++;
    if (local_bcm_request_id > 0xF) {
      local_bcm_request_id = 0;
    }

    frame.ext = A123_BCM_CMD_IS_EXTENDED;
    frame.len = A123_BCM_CMD_LENGTH;
    frame.id = A123_BCM_CMD_FRAME_ID;
    memset(frame.data, 0, sizeof(frame.data));
    A123_bcm_cmd.bcm_request_id = local_bcm_request_id;
    A123_bcm_cmd.bcm_balance_cmd_target = 0xFF;
    A123_bcm_cmd.bcm_request_type_new = 1;
    A123_bcm_cmd.bcm_request_type_old = 1;
    A123_bcm_cmd.bcm_message_format = 1;
    A123_bcm_cmd.bcm_v_balance_target = A123_bcm_cmd_bcm_v_balance_target_encode(3.5);

    A123_bcm_cmd_pack(frame.data, &A123_bcm_cmd, 8);

    if (!can.tryToSend(frame)) {
      digitalWrite(RedLED, HIGH);
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
  }
}

void TaskPublishMQTT(void *pvParameters) {

  (void)pvParameters;
  TickType_t xLastWakeTime;
  static struct canRxMessages_t myCanRxMessages;

  char buf[20];
  double Power;
  int i = 0;
  String timeStamp;
  String formattedDate;
  JSONVar modul;
  JSONVar cells;
  JSONVar balancers;
  JSONVar temps;

  // bms["TotalStartTime"] = rtc.getTime("%Y-%m-%dT%H:%M:%S");

  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    if (xQueueReceive(rxMailbox, &myCanRxMessages, 10)) {
      int j = 0;
      for (i = 0; i < 15; i++) {
        if (myCanRxMessages.modulesFound[i]) {
          digitalWrite(GreenLED, LOW);
          // modul["id"] = i;
          modul["umin"] = round(A123_m01_n_std_mod_v_min_decode(myCanRxMessages.module_std[i].mod_v_min) * 1000);
          modul["umax"] = round(A123_m01_n_std_mod_v_max_decode(myCanRxMessages.module_std[i].mod_v_max) * 1000);
          modul["bcount"] = A123_m01_n_std_m01n_bal_count_decode(myCanRxMessages.module_std[i].m01n_bal_count);
          
          // modul["res_id"] = A123_m01_n_std_m01n_response_id_std_decode(myCanRxMessages.module_std[i].m01n_response_id_std);
          // modul["sw_v"] =
            // String(int(A123_m01_n_std_m01n_sw_ver_major_decode(myCanRxMessages.module_std[i].m01n_sw_ver_major))) + "." + String(int(A123_m01_n_std_m01n_sw_ver_minor_decode(myCanRxMessages.module_std[i].m01n_sw_ver_minor))) + "-" + String(int(A123_m01_n_std_m01n_sw_ver_build_decode(myCanRxMessages.module_std[i].m01n_sw_ver_build)));

          modul["utot"] = round(A123_m01_n_xd1_m01n_v_mbb_total_decode(myCanRxMessages.module_xd1[i].m01n_v_mbb_total) * 1000);
          cells[0] = round(A123_m01_n_xd1_m01n_v_cell_01_decode(myCanRxMessages.module_xd1[i].m01n_v_cell_01) * 1000);
          cells[1] = round(A123_m01_n_xd1_m01n_v_cell_02_decode(myCanRxMessages.module_xd1[i].m01n_v_cell_02) * 1000);
          cells[2] = round(A123_m01_n_xd1_m01n_v_cell_03_decode(myCanRxMessages.module_xd1[i].m01n_v_cell_03) * 1000);
          cells[3] = round(A123_m01_n_xd2_m01n_v_cell_04_decode(myCanRxMessages.module_xd2[i].m01n_v_cell_04) * 1000);
          cells[4] = round(A123_m01_n_xd2_m01n_v_cell_05_decode(myCanRxMessages.module_xd2[i].m01n_v_cell_05) * 1000);
          cells[5] = round(A123_m01_n_xd2_m01n_v_cell_06_decode(myCanRxMessages.module_xd2[i].m01n_v_cell_06) * 1000);
          cells[6] = round(A123_m01_n_xd2_m01n_v_cell_07_decode(myCanRxMessages.module_xd2[i].m01n_v_cell_07) * 1000);
          modul["cells"] = cells;
          balancers[0] = myCanRxMessages.module_xd1[i].m01n_bal_cell_01;
          balancers[1] = myCanRxMessages.module_xd1[i].m01n_bal_cell_02;
          balancers[2] = myCanRxMessages.module_xd1[i].m01n_bal_cell_03;
          balancers[3] = myCanRxMessages.module_xd2[i].m01n_bal_cell_04;
          balancers[4] = myCanRxMessages.module_xd2[i].m01n_bal_cell_05;
          balancers[5] = myCanRxMessages.module_xd2[i].m01n_bal_cell_06;
          balancers[6] = myCanRxMessages.module_xd2[i].m01n_bal_cell_07;
          modul["bal"] = balancers;
          temps[0]=A123_m01_n_std_m01n_t0_decode(myCanRxMessages.module_std[i].m01n_t0);
          temps[1]=A123_m01_n_std_m01n_t1_decode(myCanRxMessages.module_std[i].m01n_t1);
          temps[2]=A123_m01_n_std_m01n_t2_decode(myCanRxMessages.module_std[i].m01n_t2);
          temps[3]=A123_m01_n_std_m01n_t3_decode(myCanRxMessages.module_std[i].m01n_t3);
          modul["t"]=temps;
          // Serial.println(JSON.stringify(modul));
          sprintf(buf, "M%02d", i);

          if (client.publish(MYMQTTTOPIC + String(buf), JSON.stringify(modul), false)) {
            digitalWrite(GreenLED, HIGH);
          } else {
            digitalWrite(GreenLED, LOW);
          }
          j++;
        }
      }
    }
    client.loop();
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10000));
  }
}