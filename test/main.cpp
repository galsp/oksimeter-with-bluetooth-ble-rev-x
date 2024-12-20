#include <Arduino.h>
#include <ArduinoJson.h>
#include <MAX3010x.h>
#include <MAX30105.h>
#include <DS3231.h>
#include <Wire.h>
#include "filters.h"
#include  <math.h>
#include "time.h"

DS3231 coba;
RTCDateTime dt;
RTCAlarmTime dt1;
float  hay,hayy;
uint8_t ledset;
uint8_t map_ir;

#define MAX30102_ADDRESS 0x57
#define FIFO_DATA_REGISTER 0x07

typedef const uint_fast16_t fast;  // ytpedef bikin alias tipe data, shg bisa dinamai sesuai kit mau
typedef uint16_t fastt;
String a;
float izza = 20;
unsigned long daq_check;
unsigned long daq_check_interval = 10 * 100;
long lastMsg = 0;
char msg[50];
uint_fast8_t value = 0;
long t1, t2, t3, t4, t5, t6, t7, t8, t9;
float x;
bool uplod = false;
uint_fast8_t average_bpm, average_spo2;
// inisial json
unsigned int _data_length = 256;
DynamicJsonDocument _doc(_data_length);
String _txt;  // Sensor (adjust to your sensor type)
MAX30105 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;  // default 400 // tipe data auto ut menyesuaikan
const float kSamplingFrequency = 400.0;
// Finger Detection Threshold and Cooldown
const unsigned long kFingerThreshold = 10000;// intensitas chaya utk led red
const unsigned long kFingerCooldownMs = 300;  // defalut 500
// Edge Detectioon Threshold (decrease for MAX30100)
const float kEdgeThreshold = -2000.0;//defaukt 2000
// Filters
const float kLowPassCutoff = 5.0;
const float kHighPassCutoff = 0.5;

// Averaging
bool kEnableAveraging = false;
const uint_fast8_t kAveragingSamples = 5;
uint_fast8_t kSampleThreshold = 3;

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
uint_fast8_t pin[5]={2,3,4,5,6};

uint_fast8_t w;

void setup() {

   Serial.begin(9600);
   Wire.begin();
  
  coba.begin();

  coba.setDateTime(__DATE__, __TIME__);
  // Set from UNIX timestamp
  // clock.setDateTime(1397408400);
  // Manual (YYYY, MM, DD, HH, II, SS
  coba.setDateTime(2024,12,17, 12, 38, 00);
  coba.setAlarm1( 3,10, 40, 0, DS3231_MATCH_M_S,true);
  
    if (sensor.begin() && sensor.setSamplingRate(kSamplingRate)) {
      Serial.println("Sensor initialized");
    }
    else {
      Serial.println("Sensor not found");
      
    }

    sensor.setLedCurrent(MAX30105::LED_RED, 47);//20
    sensor.setLedCurrent(MAX30105::LED_IR, 40);//40


  }


  void loop() {
    // put your main code here, to run repeatedly:
  
// Serial.println(millis());
  dt = coba.getDateTime();
  dt1 = coba.getAlarm1();

if(millis()-t6 >1000){

  Serial.print("format tanggal lengkap: ");
  Serial.println(coba.dateFormat("d F Y H:i:s",  dt));
  //  a = String(coba.dateFormat("d",dt));
  t6 = millis();
}

  // }
  ///
  x = sensor.readTemperature();
  // long rssi = WiFi.RSSI();
  auto sample  = sensor.readSample(200);  // 200 itu milidetik
  float current_value_red = sample.red;
  float current_value_ir = sample.ir;

  if (sample.red > kFingerThreshold) {
    if (millis() - finger_timestamp > kFingerCooldownMs) {
      finger_detected = true;
    }
  } else {
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

    _doc["nodeCode"] = "";
     String _time =" String(pTime())";
    _doc["time"] = _time;
    _doc["0"] = average_bpm;
    _doc["1"] = average_spo2;
    _doc["2"] = x;
    _doc["3"] =" rssi";
    _doc["4"] = "pertama";
    _doc["5"] ="kedua ";
    _doc["5"] ="keduaaaa ";
    serializeJsonPretty(_doc, _txt);

    if (millis() - t2 > 1000) {      //  ptr_MQTT->publish(out_topic,_txt.c_str());
       Serial.print("data without finger = ");
       t2 = millis();
       Serial.println(_txt);

    }
   
  }

  if (finger_detected) {

    Serial.println("p0");
    current_value_red = low_pass_filter_red.process(current_value_red);
    current_value_ir = low_pass_filter_ir.process(current_value_ir);
    // Statistics for pulse oximetry
    stat_red.process(current_value_red);
    stat_ir.process(current_value_ir);
    // Heart beat detection using value for red LED
    float current_value = high_pass_filter.process(current_value_red);
    float current_diff = differentiator.process(current_value);

    // Valid values?
    if (!isnan(current_diff) && !isnan(last_diff)) {
        Serial.println("p1");
        Serial.print("crossed");
        Serial.print(crossed);
        Serial.print("|curr_diff");
        Serial.print(current_diff);
      // Detect Heartbeat - Zero-Crossing
      if (last_diff > 0 && current_diff < 0) {
        crossed = true;
        Serial.println("p1/2");
        crossed_time = millis();
      }

      if (current_diff > 0) {
        crossed = false;
        Serial.println("-p1/2");
        
      }


      // Detect Heartbeat - Falling Edge Threshold
      if (crossed && current_diff < kEdgeThreshold) {
        Serial.println("p2");
        if (last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {  // defalut 300
          // Show Results
          Serial.print("p3");
        
          fastt bpm = 60000 / (crossed_time - last_heartbeat);
          float rred = (stat_red.maximum() - stat_red.minimum()) / stat_red.average();
          float rir = (stat_ir.maximum() - stat_ir.minimum()) / stat_ir.average();
          float r = rred / rir;
          float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;

          if (bpm > 20 && bpm < 250) {
            Serial.print("p4");
            // Average?
            kEnableAveraging = true;
            if (kEnableAveraging == true) {
              Serial.println("p5");
              average_bpm = averager_bpm.process(bpm);
              uint_fast32_t average_r = averager_r.process(r);
              average_spo2 = averager_spo2.process(spo2);
              uint_fast32_t bowl[10];
              uint_fast32_t average_bpmnow = average_bpm;
              uint_fast32_t average_spo2now = average_spo2;

              long avr = averager_bpm.count();
              Serial.println(avr);

              // Show if enough samples have been collected
              if (averager_bpm.count() >= kSampleThreshold) {
                Serial.println("p7");
                Serial.println(millis());
                Serial.print("Heart Rate (avg, bpm): ");
                Serial.println(average_bpm);
                Serial.print("R-Value (avg): ");
                Serial.println(average_r);
                Serial.print("SpO2 (avg, %): ");
                Serial.println(average_spo2);
                if (average_bpm < 0) average_bpm = 0;
                if (average_spo2 < 0) average_spo2 = 0;
                if (average_spo2 > 100)average_spo2 =100; 
                   
                   int bpmnow = average_bpm;
                // for(int a = 0 ; a<= 10 ; a++){
                for( w = 0; w <= 10; w++){
                 
                  if(bpmnow < average_bpm||bpmnow >average_bpm){
                     bowl[w]= average_bpm;
                   hay += bowl[w];
                   hayy = hay/10;
                    if(w == 10){
                Serial.print("bpm rata = ");
                Serial.println(hayy);
                     }      
                  }
                   
                   delay(100);      

                }
               

                Serial.print("Time (ms): ");

                float spo2_batas = constrain(average_spo2, 0, 100);

                //json
                _doc["nodeCode"] = "node_code";
                //  String _time = String(pTime());
                _doc["time"] = "_time";
                _doc["0"] = average_bpm;  //average_bpm
                _doc["1"] = spo2_batas;   //average_spo2
                _doc["2"] = x;
                _doc["3"] =" rssi";
                serializeJsonPretty(_doc, _txt);
                  if (millis() - t3 > 1000) {  // tambahan
                                             // client.publish("flux/general_json/43-6964/tes","tes");
                                             // if( ptr_MQTT->publish(out_topic,_txt.c_str())){
                                             //   Serial.println("succes");
                          Serial.println(_txt);                    //   }
                                          //   else{
                                             //     Serial.println("not now");
                                             //   }

                    t3 = millis();
                  }
                
              }
              Serial.println("avg on");
            } else {
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


          // Reset statistic
          stat_red.reset();
          stat_ir.reset();
        }
        crossed = false;
        last_heartbeat = crossed_time;
      }
    }
    last_diff = current_diff;
  }
  
}


