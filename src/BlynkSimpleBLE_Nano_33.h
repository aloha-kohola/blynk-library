/**
 * @file       BlynkSimpleBLE_Nano_33.h
 * @author     Manuel Reichert
 * @license    This project is released under the MIT License (MIT)
 * @copyright  Copyright (c) 2020 BlynkSimpleBLE_Nano_33_h
 * @date       Aug 2020
 * @brief
 *
 */

#ifndef BlynkSimpleBLE_Nano_33_h
#define BlynkSimpleBLE_Nano_33_h

#ifndef BLYNK_INFO_CONNECTION
#define BLYNK_INFO_CONNECTION "RBL_BLE_Nano"
#endif

#define BLYNK_PRINT Serial
#define BLYNK_DEBUG

#define BLYNK_SEND_ATOMIC
#define BLYNK_SEND_CHUNK 20
#define BLYNK_SEND_THROTTLE 20

#include <BlynkApiArduino.h>
#include <Blynk/BlynkProtocol.h>
#include <utility/BlynkFifo.h>
#include <ArduinoBLE.h>

/*
 * The Nordic UART Service
 */
//static const uint8_t uart_base_uuid[]     = {0x71, 0x3D, 0, 0, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const char* uart_base_uuid     = "713D0000-503E-4C75-BA94-3148F18D941E";

//static const uint8_t uart_base_uuid_rev[] = {0x1E, 0x94, 0x8D, 0xF1, 0x48, 0x31, 0x94, 0xBA, 0x75, 0x4C, 0x3E, 0x50, 0, 0, 0x3D, 0x71};
//static const uint8_t uart_tx_uuid[]       = {0x71, 0x3D, 0, 3, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const char* uart_tx_uuid       = "713D0003-503E-4C75-BA94-3148F18D941E";

//static const uint8_t uart_rx_uuid[]       = {0x71, 0x3D, 0, 2, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const char* uart_rx_uuid       = "713D0002-503E-4C75-BA94-3148F18D941E";

static const char* uart_dev_name      = "Blynk";

#define TXRX_BUF_LEN 20
uint8_t txPayload[TXRX_BUF_LEN] = {0,};
uint8_t rxPayload[TXRX_BUF_LEN] = {0,};

/*GattCharacteristic  txCharacteristic (uart_tx_uuid, txPayload, 1, TXRX_BUF_LEN,
                                      GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE |
                                      GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE);

GattCharacteristic  rxCharacteristic (uart_rx_uuid, rxPayload, 1, TXRX_BUF_LEN,
                                      GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);

GattCharacteristic *uartChars[] = {&txCharacteristic, &rxCharacteristic};*/

//GattService         uartService(uart_base_uuid, uartChars, sizeof(uartChars) / sizeof(GattCharacteristic*));

BLEService  uartService(uart_base_uuid);
BLECharacteristic txCharacteristic(uart_tx_uuid, BLEWriteWithoutResponse | BLEWrite, TXRX_BUF_LEN);
BLECharacteristic rxCharacteristic(uart_rx_uuid, BLENotify, TXRX_BUF_LEN);

class BlynkTransportBLE_Nano_33
{
public:
    BlynkTransportBLE_Nano_33()
        : mConn (false)
    {}

    // IP redirect not available
    void begin(char* h, uint16_t p) {}

    void begin() {
        instance = this;

        /*ble.gap().onConnection(connectCallback);
        ble.gap().onDisconnection(disconnectCallback);

        ble.gattServer().addService(uartService);
        ble.gattServer().onDataWritten(writeCallback);
        ble.gattServer().onDataSent(sentCallback);

        // Setup advertising
        ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS,
                                              uart_base_uuid_rev, sizeof(uart_base_uuid));*/

        if (!BLE.begin()) {
            Serial.println("starting BLE failed!");
            while (1);
        }

        BLE.setLocalName(uart_dev_name);
        BLE.setDeviceName(uart_dev_name);
        BLE.setAdvertisedService(uartService);
        uartService.addCharacteristic(txCharacteristic);
        uartService.addCharacteristic(rxCharacteristic);
        BLE.addService(uartService);

        // Bluetooth LE connection handlers.
        BLE.setEventHandler(BLEConnected, connectCallback);
        BLE.setEventHandler(BLEDisconnected, disconnectCallback);

        // Event driven reads.
        txCharacteristic.setEventHandler(BLEWritten, writeCallback);

        BLE.advertise();

        // Print out full UUID and MAC address.
        Serial.println("Peripheral advertising info: ");
        Serial.print("Name: ");
        Serial.println(uart_dev_name);
        Serial.print("MAC: ");
        Serial.println(BLE.address());
        Serial.print("Service UUID: ");
        Serial.println(uartService.uuid());
        Serial.print("rxCharacteristic UUID: ");
        Serial.println(rxCharacteristic);
        Serial.print("txCharacteristics UUID: ");
        Serial.println(txCharacteristic);
        

        Serial.println("Bluetooth device active, waiting for connections...");
    }

    bool connect() {
        mBuffRX.clear();
        Serial.println("connect();");
        return mConn = true;
    }

    void disconnect() {
        Serial.println("disconnect();");
        mConn = false;
    }

    bool connected() {
        //Serial.println("connected();");
        return mConn;
    }

    size_t read(void* buf, size_t len) {
        Serial.println("read();");
        millis_time_t start = BlynkMillis();
        while (BlynkMillis() - start < BLYNK_TIMEOUT_MS) {
            if (available() < len) {
                //ble.waitForEvent();
                BLE.poll(1);
                Serial.println("Poll");
            } else {
                break;
            }
        }

        noInterrupts();
        size_t res = mBuffRX.get((uint8_t*)buf, len);
        interrupts();

        Serial.print("read: " + String((const char*)buf));
        for(uint8_t i = 0; i < len; i++) {
            Serial.print((int)((uint8_t*)buf) + i, 16);
        }
        Serial.println();

        return res;
    }

    size_t write(const void* buf, size_t len) {
        //ble.updateCharacteristicValue(rxCharacteristic.getValueAttribute().getHandle(), (uint8_t*)buf, len);
        rxCharacteristic.writeValue((const unsigned char*)buf, len);
        Serial.println("write: " + String((const char*)buf));
        return len;
    }

    size_t available() {
        noInterrupts();
        size_t rxSize = mBuffRX.size();
        interrupts();
        return rxSize;
    }

private:
    static BlynkTransportBLE_Nano_33* instance;

    static
    void writeCallback(BLEDevice central, BLECharacteristic characteristic)
    {
        const uint8_t* value = {0,};
        uint8_t len = 0;

        Serial.println("Value received.");

        if (!instance)
            return;
      noInterrupts();
      //BLYNK_DBG_DUMP(">> ", params->data, params->len);
      //instance->mBuffRX.put(rxCharacteristic., params->len);
      value = characteristic.value();
      len = characteristic.valueLength();
      instance->mBuffRX.put(value, len);
      interrupts();
      Serial.println("writeCallback: " + String((const char*)value));
    }

    static
    void sentCallback(unsigned count)
    {
      //Serial.print("SENT: ");
      //Serial.println(count);
    }

    static
    void connectCallback(BLEDevice central);

    static
    void disconnectCallback(BLEDevice central);

private:
    bool mConn;

    BlynkFifo<uint8_t, BLYNK_MAX_READBYTES*2> mBuffRX;
};

class BlynkBLE_Nano_33
    : public BlynkProtocol<BlynkTransportBLE_Nano_33>
{
    typedef BlynkProtocol<BlynkTransportBLE_Nano_33> Base;
public:
    BlynkBLE_Nano_33(BlynkTransportBLE_Nano_33& transp)
        : Base(transp)
    {}

    void begin(const char* auth)
    {
        Serial.println("Base::begin();");
        Base::begin(auth);
        state = DISCONNECTED;
        conn.begin();
    }
};

BlynkTransportBLE_Nano_33* BlynkTransportBLE_Nano_33::instance = NULL;

static BlynkTransportBLE_Nano_33 _blynkTransport;
BlynkBLE_Nano_33 Blynk(_blynkTransport);

void BlynkTransportBLE_Nano_33::connectCallback(BLEDevice central)
{
  BLYNK_LOG1("Device connected");
  Serial.println("Device connected");
  Blynk.startSession();
}

void BlynkTransportBLE_Nano_33::disconnectCallback(BLEDevice central)
{
  BLYNK_LOG1("Device disconnected");
  Serial.println("Device disconnected");
  //__disable_irq();
  Blynk.disconnect();
  //__enable_irq();
  BLE.advertise();
}

#include <BlynkWidgets.h>

#endif
