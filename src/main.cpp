#include <Arduino.h>
/* Example implementation of an alarm using DS3231
 *
 * VCC and GND of RTC should be connected to some power source
 * SDA, SCL of RTC should be connected to SDA, SCL of arduino
 * SQW should be connected to CLOCK_INTERRUPT_PIN
 * CLOCK_INTERRUPT_PIN needs to work with interrupts
 */

#include <RTClib.h>
// #include <Wire.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define WAKEUP_PIN (gpio_num_t)33  // Use an RTC GPIO pin (RTC_GPIO8 is GPIO33)

RTC_DS3231 rtc;

// the pin that is connected to SQW
#define CLOCK_INTERRUPT_PIN 4

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
BLECharacteristic* pCharacteristic1 = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t value = 0;
uint8_t ttime[7];


// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

static BLEUUID  BLESERVICE_UUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define TIME_SERVICE_UUID        "1805"

#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID1 "2A08"

void onAlarm();

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      digitalWrite(2, HIGH);
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      digitalWrite(2, LOW);
    }
};

class TimeCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic, esp_ble_gatts_cb_param_t* param) {
        std::string value = pCharacteristic->getValue();

        if (value.length() != 7) {
            Serial.println("Invalid time format received");
            return;
        }

        // Parse received data
        uint16_t year = value[0] | (value[1] << 8);
        uint8_t month = value[2];
        uint8_t day = value[3];
        uint8_t hour = value[4];
        uint8_t minute = value[5];
        uint8_t second = value[6];
        Serial.print(year); Serial.print(month);  Serial.print(day); 
        Serial.print(hour); Serial.print(minute); Serial.print(second);
        DateTime updateTime (year, month, day, hour, minute, second);
        rtc.adjust(updateTime);
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      digitalWrite(2, LOW);
    }
};

void setup() {
    pinMode(2, OUTPUT);
    Serial.begin(115200);

    // Configure the wake-up pin
    esp_sleep_enable_ext0_wakeup(WAKEUP_PIN, 0); // Wake up on LOW signal

    // initializing the rtc
    if(!rtc.begin()) {
        Serial.println("Couldn't find RTC!");
        Serial.flush();
        while (1) delay(10);
    }

    if(rtc.lostPower()) {
        // this will adjust to the date and time at compilation
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    //we don't need the 32K Pin, so disable it
    rtc.disable32K();

    // Making it so, that the alarm will trigger an interrupt
    pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), onAlarm, FALLING);

    // set alarm 1, 2 flag to false (so alarm 1, 2 didn't happen so far)
    // if not done, this easily leads to problems, as both register aren't reset on reboot/recompile
    rtc.clearAlarm(1);
    rtc.clearAlarm(2);

    // stop oscillating signals at SQW Pin
    // otherwise setAlarm1 will fail
    rtc.writeSqwPinMode(DS3231_OFF);

    // turn off alarm 2 (in case it isn't off already)
    // again, this isn't done at reboot, so a previously set alarm could easily go overlooked
    rtc.disableAlarm(2);

    // schedule an alarm 10 seconds in the future
    if(!rtc.setAlarm1(
            rtc.now() + TimeSpan(60),
            DS3231_A1_Second // this mode triggers the alarm when the seconds match. See Doxygen for other options
    )) {
        Serial.println("Error, alarm wasn't set!");
    }else {
        Serial.println("Alarm will happen in 60 seconds!");
    }

    // To sel alarm when minutes and seconds match set DS3231_A1_Minute in setAlarm1() above
    // and uncomment below lines this shift 59 minutes clock in ahead to verify the alarm trigger.
    // DateTime now = rtc.now();
    // uint8_t newMn = (now.minute() + ((uint8_t) (59))) % 60;
    // uint8_t newHr = (now.hour() + ((now.minute() + ((uint8_t) (30))) / 60));
    // DateTime futureTime (now.year(), now.month(), now.day(), newHr, newMn, now.second());
    // rtc.adjust(futureTime);

      // Create the BLE Device
    BLEDevice::init("ESP32");

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);
    // BLEService *pService = pServer->createService(BLESERVICE_UUID, 30, 0);
    BLEService *pServiceTime = pServer->createService(TIME_SERVICE_UUID);


    // Create a BLE Characteristic
    pCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ   |
                        BLECharacteristic::PROPERTY_WRITE  |
                        BLECharacteristic::PROPERTY_NOTIFY |
                        BLECharacteristic::PROPERTY_INDICATE
                        );

  pCharacteristic1 = pServiceTime->createCharacteristic(
                    CHARACTERISTIC_UUID1,
                    BLECharacteristic::PROPERTY_READ   |
                    BLECharacteristic::PROPERTY_WRITE  |
                    BLECharacteristic::PROPERTY_NOTIFY |
                    BLECharacteristic::PROPERTY_INDICATE
                  );

    pCharacteristic->addDescriptor(new BLE2902());     
    pCharacteristic1->addDescriptor(new BLE2902());          

    pCharacteristic1->setCallbacks(new TimeCallback());

    // Start the service
    pService->start();
    pServiceTime->start();

    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
    BLEDevice::startAdvertising();
    Serial.println("Waiting a client connection to notify...");
}

void loop() {
    // print current time
    char date[15] = "YYYY hh:mm:ss";
    rtc.now().toString(date);
    Serial.print(date);

    // the stored alarm value + mode
    DateTime alarm1 = rtc.getAlarm1();
    Ds3231Alarm1Mode alarm1mode = rtc.getAlarm1Mode();
    char alarm1Date[12] = "DD hh:mm:ss";
    alarm1.toString(alarm1Date);
    Serial.print(" [Alarm1: ");
    Serial.print(alarm1Date);
    Serial.print(", Mode: ");
    switch (alarm1mode) {
      case DS3231_A1_PerSecond: Serial.print("PerSecond"); break;
      case DS3231_A1_Second: Serial.print("Second"); break;
      case DS3231_A1_Minute: Serial.print("Minute"); break;
      case DS3231_A1_Hour: Serial.print("Hour"); break;
      case DS3231_A1_Date: Serial.print("Date"); break;
      case DS3231_A1_Day: Serial.print("Day"); break;
    }

    // the value at SQW-Pin (because of pullup 1 means no alarm)
    Serial.print("] SQW: ");
    Serial.print(digitalRead(CLOCK_INTERRUPT_PIN));

    // whether a alarm fired
    Serial.print(" Fired: ");
    Serial.print(rtc.alarmFired(1));

    // Serial.print(" Alarm2: ");
    // Serial.println(rtc.alarmFired(2));
    // control register values (see https://datasheets.maximintegrated.com/en/ds/DS3231.pdf page 13)
    // Serial.print(" Control: 0b");
    // Serial.println(read_i2c_register(DS3231_ADDRESS, DS3231_CONTROL), BIN);

    // resetting SQW and alarm 1 flag
    // using setAlarm1, the next alarm could now be configurated
    if (rtc.alarmFired(1)) {
        rtc.clearAlarm(1);
        Serial.print(" - Alarm cleared");
    }
    Serial.println();

    // notify changed value
    if (deviceConnected) {
        pCharacteristic->setValue((uint8_t*)&value, 1);
        pCharacteristic->notify();
        value++;
        delay(200); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
        DateTime time1= rtc.now();
        ttime[0] = time1.year();
        ttime[1] = time1.year() >> 8;
        ttime[2] = time1.month();
        ttime[3] = time1.day();
        ttime[4] = time1.hour();
        ttime[5] = time1.minute();
        ttime[6] = time1.second();
        ttime[7] = 0;
        ttime[8] = 0;

        // for(int i = 0; i < 8; i++)
        // {
        //     uint8_t temp = ttime[i];
        //     Serial.println(temp);
        // }

        pCharacteristic1->setValue(ttime, 8);
        pCharacteristic1->notify();
        // value++;

        // To send controller in deep sleep, just to test sleep mode
        // At pin 33 when there is interrupt at SQW pin from RTC DS3231 controller wakes up
        Serial.println(millis());
        Serial.println("Kya ho gya bhai?");
        if (millis() > 10000)
        {
            // Enter Hibernation Mode
            Serial.println("Going to Deep Sleep");
            esp_deep_sleep_start();
        }
    }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
    delay(2000);
}

void onAlarm() {
    Serial.println("Alarm occured!");
}

/*static uint8_t read_i2c_register(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write((byte)reg);
    Wire.endTransmission();

    Wire.requestFrom(addr, (byte)1);
    return Wire.read();
}*/