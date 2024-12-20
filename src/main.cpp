#include <Arduino.h>
#include <ArduinoBLE.h>
#include <errfs.h>

#include <Arduino.h>
#include <Wire.h>
#include <DS3231.h>

DS3231 rtcne;
RTCDateTime dt;

BLEService batreService("180F");
BLEService sensorService("1815");

BLEUnsignedCharCharacteristic batteryLevelChar("2A19", BLERead | BLENotify);
BLEStringCharacteristic bpmChar("2A39", BLERead | BLENotify, 20);
BLEStringCharacteristic oksiChar("2BF3", BLERead | BLENotify, 20);
BLEStringCharacteristic temperaturChar("2A25", BLERead | BLENotify, 20);
BLEStringCharacteristic timeuid("1805", BLERead | BLENotify, 20);
BLEStringCharacteristic notyChar("2A01", BLERead | BLEWrite | BLENotify, 20);

int oldBatteryLevel = 0; // last battery level reading from analog input
////////////////////////////////////////////////////////////////////////////////////////////
#include <MAX3010x.h>
#include <MAX30105.h>
#include "filters.h"
#include "time.h"
#include <Wire.h>
typedef const uint_fast16_t fast; // ytpedef bikin alias tipe data, shg bisa dinamai sesuai kit mau
typedef uint16_t fastt;
fast izza = 20;
// unsigned long daq_check; unsigned long daq_check_interval = 10 * 100;
// void switchRelay(bool _the_state); uint_fast8_t getTemperature(); void sendData();
// void deviceRelaySwitch(DynamicJsonDocument source_doc); void devicePing(DynamicJsonDocument source_doc); void deviceRandomFunction(DynamicJsonDocument source_doc);
// void autoRelay(); void deviceSwitchConnection(DynamicJsonDocument source_doc);
long lastMsg = 0;
char msg[50];
uint_fast8_t value = 0;
// long t1, t2, t3, t4, t5, t6, t7, t8, t9;
float x;
bool uplod = false;
uint_fast8_t average_bpm, average_spo2;
// inisial json
unsigned int _data_length = 1024;
MAX30105 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS; // default 400 // tipe data auto ut menyesuaikan
const float kSamplingFrequency = 400.0;
// Finger Detection Threshold and Cooldown
const unsigned long kFingerThreshold = 10000;
const unsigned long kFingerCooldownMs = 500; // defalut 500
// Edge Detectioon Threshold (decrease for MAX30100)
const float kEdgeThreshold = -2000.0;
// Filters
const float kLowPassCutoff = 5.0;
const float kHighPassCutoff = 0.5;

// Averaging
bool kEnableAveraging = false;
const uint_fast8_t kAveragingSamples = 5;
uint_fast8_t kSampleThreshold = 2;

// Filter Instances
LowPassFilter low_pass_filter_red(kLowPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter_ir(kLowPassCutoff, kSamplingFrequency);
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<kAveragingSamples> averager_bpm;
MovingAverageFilter<kAveragingSamples> averager_r;
MovingAverageFilter<kAveragingSamples> averager_spo2;

// Statistic for pulse oximetry
MinMaxAvgStatistic stat_red;
MinMaxAvgStatistic stat_ir;

// R value to SpO2 calibration factors
// See https://www.maximintegrated.com/en/design/technical-documents/app-notes/6/6845.html
float kSpO2_A = 1.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 112.6898759;

// Timestamp of the last heartbeat
long last_heartbeat = 0;
// Timestamp for finger detection
long finger_timestamp = 0;
bool finger_detected = false;

// Last diff to detect zero crossing
float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;
#define led 27

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 180      /* Time ESP32 will go to sleep (in seconds) */
#define tosleep 2
unsigned long lastmiliisdeepsleep = 0;
//////////////////////////////////////////////////////////////////////////////////////////
int med(int arr[], int size);
unsigned long lastmillis = 0;
int arrMedian[500];
int arrMedianspo[500];
int arrMediansuhu[500];
int arri;

int arrspo[500];
int arrbpm[500];
int arrsuhu[500];
int arrtime[500];
bool k = 1;

String BLEbpm;
String BLEspo;
String BLEsuhu;
String BLEtime;
///////////////////

int deviceon;
#define nodeId 3

#if nodeId == 1
#define nodeCode "KA-VF4L" // Rover1
#define devicename "Stroke 1"
#elif nodeId == 2
#define nodeCode "KA-D5CW" // Rover2
#define devicename "Stroke 2"
#elif nodeId == 3
#define nodeCode "KA-06NG" // Rover3
#define devicename "Stroke 3"
#else
#define nodeCode "NO NODECODE FOUND"
#define devicename "NO device FOUND"
#endif

void setup()
{

  dt = rtcne.getDateTime();
  k = 1;
  Serial.begin(115200);
  // while (!Serial)
  //   ;
  // rtcne.setDateTime(__DATE__, __TIME__);

  rtcne.begin();

  if (!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED))
  {
    Serial.println("LittleFS Mount Failed");
    return;
  }
  errReadInt("/boot.txt", deviceon);
  deviceon++;
  errfsWriteFsStr("/boot.txt", String(deviceon));
  errReadArr("/spo.txt", arrspo);
  errReadArr("/bpm.txt", arrbpm);
  errReadArr("/suhu.txt", arrsuhu);
  errReadArr("/unixtime.txt", arrtime);

  if (true) ////bluetooth
  {
    // begin initialization
    BLE.begin();

    BLE.setLocalName(devicename);
    BLE.setDeviceName(nodeCode);

    BLE.setAdvertisedService(batreService);           // add the service UUID
    batreService.addCharacteristic(batteryLevelChar); // add the battery level characteristic
    BLE.addService(batreService);                     // Add the battery service

    BLE.setAdvertisedService(sensorService);         // add the service UUID
    sensorService.addCharacteristic(bpmChar);        // add the battery level characteristic
    sensorService.addCharacteristic(oksiChar);       // add the battery level characteristic
    sensorService.addCharacteristic(temperaturChar); // add the battery level characteristic
    sensorService.addCharacteristic(timeuid);        // add the battery level characteristic
    sensorService.addCharacteristic(notyChar);       // add the battery level characteristic
    BLE.addService(sensorService);                   // Add the battery service

    batteryLevelChar.writeValue(oldBatteryLevel); // set initial value for this characteristic
    // bpmChar.writeValue(oldBatteryLevel);          // set initial value for this characteristic
    // oksiChar.writeValue(oldBatteryLevel);         // set initial value for this characteristic
    // temperaturChar.writeValue(oldBatteryLevel);   // set initial value for this characteristic

    // start advertising
    BLE.advertise();

    Serial.println("Bluetooth® device active, waiting for connections...");
  }

  if (sensor.begin() && sensor.setSamplingRate(kSamplingRate)) /// sensor
  {
    Serial.println("Sensor initialized");
  }

  else
  {

    Serial.println("Sensor not found");
    while (1)
      ; // kalo ini jalan berarti program jadi stuck dan tdk eksekusi ke code selanjutnya
  }

  sensor.setLedCurrent(MAX30105::LED_RED, 38); // 28
  sensor.setLedCurrent(MAX30105::LED_IR, 38);  // 28 pangkal jari manis

  lastmiliisdeepsleep = millis();
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  // errWriteArr("/spo.txt", 0, deviceon);
  // errWriteArr("/bpm.txt", 0, deviceon);
  // errWriteArr("/suhu.txt", 0, deviceon);
  // errWriteArr("/suhu.txt", 0, deviceon);
}

void loop()
{

  dt = rtcne.getDateTime();
  // wait for a Bluetooth® Low Energy central
  BLEDevice central = BLE.central();

  x = sensor.readTemperature();
  long rssi = 0;
  auto sample = sensor.readSample(100); // //defalutnya 100200 itu milidetik
  float current_value_red = sample.red;
  float current_value_ir = sample.ir;
  if (sample.red < kFingerThreshold)
  {
    //   if (millis() - finger_timestamp > kFingerCooldownMs)
    //   {
    //     finger_detected = true;
    //   }
    // }
    // else
    // {
    // Reset values if the finger is removed
    differentiator.reset();
    averager_bpm.reset();
    averager_r.reset();
    averager_spo2.reset();
    low_pass_filter_red.reset();
    low_pass_filter_ir.reset();
    high_pass_filter.reset();
    stat_red.reset();
    stat_ir.reset();

    finger_detected = false;

    finger_timestamp = millis();

    // publish data wtihout finger
    average_bpm = 0;
    average_spo2 = 0;
    if (k == 1)
    {
      //////////

      int logarri;
      if (arri != 0)
      {
        logarri = arri;
      }
      BLEbpm = String(med(arrMedian, logarri)) + " BPM";
      BLEspo = String(med(arrMedianspo, logarri)) + "%";
      BLEsuhu = String(med(arrMediansuhu, logarri)) + "℃";
      String timene = rtcne.dateFormat("U", dt);
      Serial.println("Send BLE = Heart Rate: " + BLEbpm + "| SPO2: " + BLEspo + "| suhu: " + BLEsuhu + " | deviceon " + deviceon + " | timeunix " + timene);
      arrbpm[deviceon - 1] = BLEbpm.toInt();
      arrspo[deviceon - 1] = BLEspo.toInt();
      arrsuhu[deviceon - 1] = BLEsuhu.toInt();
      arrtime[deviceon - 1] = timene.toInt();

      errWriteArr("/spo.txt", arrspo, deviceon);
      errWriteArr("/bpm.txt", arrbpm, deviceon);
      errWriteArr("/suhu.txt", arrsuhu, deviceon);
      errWriteArr("/unixtime.txt", arrtime, deviceon);

      ///////////
      Serial.println("no finggers");
      notyChar.writeValue("no finggers");
      k = 0;
    }
    lastmillis = millis();
    arri = 0;
  }

  else
  {
    k = 1;
    current_value_red = low_pass_filter_red.process(current_value_red);
    current_value_ir = low_pass_filter_ir.process(current_value_ir);

    // Statistics for pulse oximetry
    stat_red.process(current_value_red);
    stat_ir.process(current_value_ir);

    // Heart beat detection using value for red LED
    float current_value = high_pass_filter.process(current_value_red);
    float current_diff = differentiator.process(current_value);

    // Valid values?
    if (!isnan(current_diff) && !isnan(last_diff))
    {

      // Detect Heartbeat - Zero-Crossing
      if (last_diff > 0 && current_diff < 0)
      {
        crossed = true;
        crossed_time = millis();
      }

      if (current_diff > 0)
      {
        crossed = false;
      }

      // Detect Heartbeat - Falling Edge Threshold
      if (crossed && current_diff < kEdgeThreshold)
      {
        if (last_heartbeat != 0 && crossed_time - last_heartbeat > 300)
        {

          fastt bpm = 60000 / (crossed_time - last_heartbeat);
          float rred = (stat_red.maximum() - stat_red.minimum()) / stat_red.average();
          float rir = (stat_ir.maximum() - stat_ir.minimum()) / stat_ir.average();
          float r = rred / rir;
          float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;
          if (bpm > 20 && bpm < 250)
          {
            // Average?
            kEnableAveraging = true;
            if (kEnableAveraging == true)
            {
              notyChar.writeValue("scaning");
              average_bpm = averager_bpm.process(bpm);
              int average_r = averager_r.process(r);
              average_spo2 = averager_spo2.process(spo2);

              int average_bpmnow = average_bpm;
              int average_spo2now = average_spo2;

              long avr = averager_bpm.count();
              Serial.println(avr);

              // Show if enough samples have been collected
              if (averager_bpm.count() >= kSampleThreshold)
              {

                if (average_bpm < 0)
                  average_bpm = 0;
                if (average_spo2 < 0)
                  average_spo2 = 0;

                Serial.println(millis());
                Serial.print("Heart Rate (avg, bpm): ");
                Serial.println(average_bpm);
                Serial.print("R-Value (avg): ");
                Serial.println(average_r);
                Serial.print("SpO2 (avg, %): ");
                Serial.println(average_spo2);
                Serial.print("Suhu (℃)");
                Serial.println(x);
                Serial.print("Time (ms): ");
                arrMedian[arri] = average_bpm;
                arrMedianspo[arri] = average_spo2;
                arrMediansuhu[arri] = x;
                arri++;

                if (millis() - lastmillis > 30000)
                {
                  lastmillis = millis();
                  BLEbpm = String(med(arrMedian, arri)) + " BPM";
                  BLEspo = String(med(arrMedianspo, arri)) + "%";
                  BLEsuhu = String(med(arrMediansuhu, arri)) + "℃";
                  String timene = rtcne.dateFormat("U", dt);
                  Serial.println("Send BLE = Heart Rate: " + BLEbpm + "| SPO2: " + BLEspo + "| suhu: " + BLEsuhu + " | deviceon " + deviceon + " | timeunix " + timene);
                  arrbpm[deviceon - 1] = BLEbpm.toInt();
                  arrspo[deviceon - 1] = BLEspo.toInt();
                  arrsuhu[deviceon - 1] = BLEsuhu.toInt();
                  arrtime[deviceon - 1] = timene.toInt();

                  errWriteArr("/spo.txt", arrspo, deviceon);
                  errWriteArr("/bpm.txt", arrbpm, deviceon);
                  errWriteArr("/suhu.txt", arrsuhu, deviceon);
                  errWriteArr("/unixtime.txt", arrtime, deviceon);
                  arri = 0;
                }

                constrain(average_spo2, 0, 100);
              }
              Serial.println("avg on");
            }
            else
            {
              Serial.print("Time (ms): ");
              Serial.println(millis());
              Serial.print("Heart Rate (current, bpm): ");
              Serial.println(bpm);
              Serial.print("R-Value (current): ");
              Serial.println(r);
              Serial.print("SpO2 (current, %): ");
              Serial.println(spo2);
            }
          }
          else
          {
            notyChar.writeValue("detected 3");
          }

          // Reset statistic
          stat_red.reset();
          stat_ir.reset();
        }
        else
        {
          notyChar.writeValue("detected 2");
        }
        crossed = false;
        last_heartbeat = crossed_time;
      }
    }
    else
    {
      notyChar.writeValue("detected 1");
    }

    last_diff = current_diff;
  }

  if (millis() - lastmiliisdeepsleep > (60000 * tosleep))
  {
    Serial.println("sleep");
    esp_deep_sleep_start();
  }

  if (central)
  {

    if (notyChar.written())
    {
      if (notyChar.value() == "sleep")
      {
        Serial.print("sleep");
        notyChar.writeValue("sleep");
        delay(100);
        esp_deep_sleep_start();
      }
    }
    if (notyChar.value() == "erase")
    {
      notyChar.writeValue("erase");
      errfsWriteFsStr("/spo.txt", " ");
      errfsWriteFsStr("/bpm.txt", " ");
      errfsWriteFsStr("/suhu.txt", " ");
      errfsWriteFsStr("/boot.txt", " ");
      errfsWriteFsStr("/unixtime.txt", " ");
      notyChar.writeValue("okee");
      delay(1000);
      errReadInt("/boot.txt", deviceon);
      deviceon++;
      errfsWriteFsStr("/boot.txt", String(deviceon));
      errReadArr("/spo.txt", arrspo);
      errReadArr("/bpm.txt", arrbpm);
      errReadArr("/suhu.txt", arrsuhu);
      errReadArr("/unixtime.txt", arrtime);
    }

    if (notyChar.value() == "send")
    {

      BLEbpm = String(med(arrMedian, arri));
      BLEspo = String(med(arrMedianspo, arri));
      BLEsuhu = String(med(arrMediansuhu, arri));
      String timene = rtcne.dateFormat("U", dt);
      Serial.println("Send BLE = Heart Rate: " + BLEbpm + "| SPO2: " + BLEspo + "| suhu: " + BLEsuhu + " | deviceon " + deviceon + " | timeunix " + timene);
      arrbpm[deviceon - 1] = BLEbpm.toInt();
      arrspo[deviceon - 1] = BLEspo.toInt();
      arrsuhu[deviceon - 1] = BLEsuhu.toInt();
      arrtime[deviceon - 1] = timene.toInt();

      errWriteArr("/spo.txt", arrspo, deviceon);
      errWriteArr("/bpm.txt", arrbpm, deviceon);
      errWriteArr("/suhu.txt", arrsuhu, deviceon);
      errWriteArr("/unixtime.txt", arrtime, deviceon);

      notyChar.writeValue("sending " + String(deviceon) + " data");
      for (int i = 0; i < deviceon; i++)
      {
        delay(50);
        bpmChar.writeValue(String(arrbpm[i]) + " bpm");
        oksiChar.writeValue(String(arrspo[i]) + "%");
        timeuid.writeValue(String(arrtime[i]));
        temperaturChar.writeValue(String(arrsuhu[i]) + "℃");
        Serial.println(String(arrtime[i]));
      }
      notyChar.writeValue("okee");
    }
  }
  // if a central is connected to the peripheral:
  //
  //  if (central)
  //   {
  //     Serial.print("Connected to central: ");
  //     Serial.println(central.address());
  //
  //     while (central.connected())
  //     {  String valuee;
  //
  //       if (Serial.available())
  //       {
  //         valuee = Serial.readString();
  //         Serial.println(valuee);
  //         batteryLevelChar.writeValue(valuee.toInt());
  //       }
  //     }
  //     Serial.print("Disconnected from central: ");
  //     Serial.println(central.address());
  //   }
}

///////////////////

int med(int arr[], int size)
{
  int maxCount = 0;
  int mode = arr[size];
  for (int i = 1; i < size; i++)
  {
    if (arr[i] == 0)
      continue;
    int count = 0;
    for (int j = 0; j < size; j++)
    {
      if (arr[j] == arr[i])
      {
        count++;
      }
    }
    if (count > maxCount)
    {
      maxCount = count;
      mode = arr[i];
    }
  }
  return mode;
}
