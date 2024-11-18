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
long t1, t2, t3, t4, t5, t6, t7, t8, t9;
float x;
bool uplod = false;
uint_fast8_t average_bpm, average_spo2;
// inisial json
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

void setup()
{
  Serial.begin(9600);

  if (sensor.begin() && sensor.setSamplingRate(kSamplingRate))
  {
    Serial.println("Sensor initialized");
  }

  else
  {

    Serial.println("Sensor not found");
    // while (1)
    //   ; // kalo ini jalan berarti program jadi stuck dan tdk eksekusi ke code selanjutnya
  }

  sensor.setLedCurrent(MAX30105::LED_RED, 38); // 28
  sensor.setLedCurrent(MAX30105::LED_IR, 38);  // 28 pangkal jari manis
}

void loop()
{

  x = sensor.readTemperature();
  long rssi = 0;
  auto sample = sensor.readSample(100); // //defalutnya 100200 itu milidetik
  float current_value_red = sample.red;
  float current_value_ir = sample.ir;

  if (sample.red > kFingerThreshold)
  {
    if (millis() - finger_timestamp > kFingerCooldownMs)
    {
      finger_detected = true;
    }
  }
  else
  {
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
    String _time = "";
    _doc["time"] = _time;
    _doc["0"] = average_bpm;
    _doc["1"] = average_spo2;
    _doc["2"] = x;
    _doc["3"] = "";
    serializeJsonPretty(_doc, _txt);

    Serial.println(_txt);
  }

  if (finger_detected)
  {

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
