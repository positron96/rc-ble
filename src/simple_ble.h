#pragma once

#include "line_processor.h"
#include "log.h"

static NimBLEServer* pServer;

// NORDIC UART service UUID
#define NRF_UART_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NRF_UART_RX_CHAR_UUID  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NRF_UART_TX_CHAR_UUID  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define NRF_UART_CHAR_SIZE 20
#define NRF_UART_RX_BUFFER_SIZE 128

extern void process_str(const char* buf, size_t len);

using BLELineProcessor = line_processor::LineProcessor<NRF_UART_RX_BUFFER_SIZE>;
BLELineProcessor rx(line_processor::callback_t::create<process_str>());


/**  None of these are required as they will be handled by the library with defaults. **
 **                       Remove as you see fit for your needs                        */
class ServerCallbacks: public NimBLEServerCallbacks {
    /** Alternative onConnect() method to extract details of the connection.
     *  See: src/ble_gap.h for the details of the ble_gap_conn_desc struct.
     */
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
        Serial.print("Client connected: ");
        Serial.println(NimBLEAddress(desc->peer_ota_addr).toString().c_str());
        /** We can use the connection handle here to ask for different connection parameters.
         *  Args: connection handle, min connection interval, max connection interval
         *  latency, supervision timeout.
         *  Units; Min/Max Intervals: 1.25 millisecond increments.
         *  Latency: number of intervals allowed to skip.
         *  Timeout: 10 millisecond increments, try for 5x interval time for best results.
         */
        pServer->updateConnParams(desc->conn_handle, 24, 48, 0, 60);
    };
    void onDisconnect(NimBLEServer* pServer) {
        Serial.println("Client disconnected");
    };
    void onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) {
        Serial.printf("MTU updated: %u for connection ID: %u\n", MTU, desc->conn_handle);
    };

/********************* Security handled here **********************
****** Note: these are the same return values as defaults ********/
    uint32_t onPassKeyRequest(){
        Serial.println("Server Passkey Request");
        /** This should return a random 6 digit number for security
         *  or make your own static passkey as done here.
         */
        return 123456;
    };

    bool onConfirmPIN(uint32_t pass_key){
        Serial.print("The passkey YES/NO number: ");Serial.println(pass_key);
        /** Return false if passkeys don't match. */
        return true;
    };

    void onAuthenticationComplete(ble_gap_conn_desc* desc){
        /** Check that encryption was successful, if not we disconnect the client */
        if(!desc->sec_state.encrypted) {
            NimBLEDevice::getServer()->disconnect(desc->conn_handle);
            Serial.println("Encrypt connection failed - disconnecting client");
            return;
        }
        Serial.println("Starting BLE work!");
    };
};

/** Handler class for characteristic actions */
class CharacteristicCallbacks: public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic* pCharacteristic){
        Serial.print(pCharacteristic->getUUID().toString().c_str());
        Serial.print(": onRead(), value: ");
        Serial.println(pCharacteristic->getValue().c_str());
    };

    void onWrite(NimBLECharacteristic* chr) {
        // Serial.print(pCharacteristic->getUUID().toString().c_str());
        // Serial.print(": onWrite(), value: ");

        NimBLEAttValue val = chr->getValue();
        const char* in = (const char*)(val.data());
        for (size_t i = 0; i < val.size(); i++) {
            char c = in[i];
            if(c == 0) {
                continue;
            } else {
                rx.add(c);
            }
        }
    };
    /** Called before notification or indication is sent,
     *  the value can be changed here before sending if desired.
     */
    void onNotify(NimBLECharacteristic* pCharacteristic) {
        Serial.println("Sending notification to clients");
    };


    /** The status returned in status is defined in NimBLECharacteristic.h.
     *  The value returned in code is the NimBLE host return code.
     */
    void onStatus(NimBLECharacteristic* pCharacteristic, Status status, int code) {
        String str = ("Notification/Indication status code: ");
        str += status;
        str += ", return code: ";
        str += code;
        str += ", ";
        str += NimBLEUtils::returnCodeToString(code);
        Serial.println(str);
    };

    void onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) {
        String str = "Client ID: ";
        str += desc->conn_handle;
        str += " Address: ";
        str += std::string(NimBLEAddress(desc->peer_ota_addr)).c_str();
        if(subValue == 0) {
            str += " Unsubscribed to ";
        }else if(subValue == 1) {
            str += " Subscribed to notfications for ";
        } else if(subValue == 2) {
            str += " Subscribed to indications for ";
        } else if(subValue == 3) {
            str += " Subscribed to notifications and indications for ";
        }
        str += std::string(pCharacteristic->getUUID()).c_str();

        Serial.println(str);
    };
};

/** Handler class for descriptor actions */
class DescriptorCallbacks : public NimBLEDescriptorCallbacks {
    void onWrite(NimBLEDescriptor* pDescriptor) {
        std::string dscVal = pDescriptor->getValue();
        Serial.print("Descriptor witten value:");
        Serial.println(dscVal.c_str());
    };

    void onRead(NimBLEDescriptor* pDescriptor) {
        logf("%s: descr read\n", pDescriptor->getUUID().toString().c_str());
    };
};


/** Define callback instances globally to use for multiple Charateristics \ Descriptors */
static DescriptorCallbacks dscCallbacks;
static CharacteristicCallbacks chrCallbacks;

NimBLECharacteristic* pBatChar;
NimBLECharacteristic* pBatEnergyChar;

void ble_start() {
    /** sets device name */
    NimBLEDevice::init("MicroRC");

    /** Optional: set the transmit power, default is 0db */
    NimBLEDevice::setPower(9); /** +9db */

    /** Set the IO capabilities of the device, each option will trigger a different pairing method.
     *  BLE_HS_IO_DISPLAY_ONLY    - Passkey pairing
     *  BLE_HS_IO_DISPLAY_YESNO   - Numeric comparison pairing
     *  BLE_HS_IO_NO_INPUT_OUTPUT - DEFAULT setting - just works pairing
     */
    //NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY); // use passkey
    //NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_YESNO); //use numeric comparison

    /** 2 different ways to set security - both calls achieve the same result.
     *  no bonding, no man in the middle protection, secure connections.
     *
     *  These are the default values, only shown here for demonstration.
     */
    //NimBLEDevice::setSecurityAuth(false, false, true);
    NimBLEDevice::setSecurityAuth(
        /*BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_MITM |*/
        BLE_SM_PAIR_AUTHREQ_SC
    );

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    pServer->advertiseOnDisconnect(true);

    NimBLEService* pUartSvc = pServer->createService(NRF_UART_SERVICE_UUID);
    NimBLECharacteristic* pRxCharacteristic = pUartSvc->createCharacteristic(
        NRF_UART_RX_CHAR_UUID,
        NIMBLE_PROPERTY::WRITE_NR,
    /** Require a secure connection for read and write access */
    //    NIMBLE_PROPERTY::READ_ENC |  // only allow reading if paired / encrypted
    //    NIMBLE_PROPERTY::WRITE_ENC   // only allow writing if paired / encrypted
        NRF_UART_CHAR_SIZE
    );
    NimBLECharacteristic* pTxCharacteristic = pUartSvc->createCharacteristic(
        NRF_UART_TX_CHAR_UUID,
        NIMBLE_PROPERTY::NOTIFY,
        NRF_UART_CHAR_SIZE
    );

    pRxCharacteristic->setValue(0);
    pRxCharacteristic->setCallbacks(&chrCallbacks);
    pTxCharacteristic->setValue(0);
    pTxCharacteristic->setCallbacks(&chrCallbacks);

    // NimBLEService* pBaadService = pServer->createService("BAAD");
    // NimBLECharacteristic* pFoodCharacteristic = pBaadService->createCharacteristic(
    //                                            "F00D",
    //                                            NIMBLE_PROPERTY::READ |
    //                                            NIMBLE_PROPERTY::WRITE |
    //                                            NIMBLE_PROPERTY::NOTIFY
    //                                           );

    // pFoodCharacteristic->setValue("Fries");
    // pFoodCharacteristic->setCallbacks(&chrCallbacks);

    /** Note a 0x2902 descriptor MUST NOT be created as NimBLE will create one automatically
     *  if notification or indication properties are assigned to a characteristic.
     */

    /** Custom descriptor: Arguments are UUID, Properties, max length in bytes of the value */
    // NimBLEDescriptor* pC01Ddsc = pFoodCharacteristic->createDescriptor(
    //                                            "C01D",
    //                                            NIMBLE_PROPERTY::READ |
    //                                            NIMBLE_PROPERTY::WRITE|
    //                                            NIMBLE_PROPERTY::WRITE_ENC, // only allow writing if paired / encrypted
    //                                            20
    //                                           );
    // pC01Ddsc->setValue("Send it back!");
    // pC01Ddsc->setCallbacks(&dscCallbacks);

    pUartSvc->start();
    // pBaadService->start();

    constexpr uint16_t BLE_SVC_BAS_UUID16 = 0x180F;
    constexpr uint16_t BLE_SVC_BAS_CHR_UUID16_BATTERY_LEVEL = 0x2A19;
    constexpr uint16_t BLE_SVC_BAS_CHR_UUID16_ENERGY_STATUS = 0x2BF0;

    auto pBatSvc = pServer->createService(BLE_SVC_BAS_UUID16);
    pBatChar = pBatSvc->createCharacteristic(BLE_SVC_BAS_CHR_UUID16_BATTERY_LEVEL, NIMBLE_PROPERTY::READ, 1);
    pBatEnergyChar = pBatSvc->createCharacteristic(BLE_SVC_BAS_CHR_UUID16_ENERGY_STATUS, NIMBLE_PROPERTY::READ, 4);
    pBatSvc->start();

    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    /** Add the services to the advertisment data **/
    pAdvertising->addServiceUUID(pUartSvc->getUUID());
    //pAdvertising->addServiceUUID(pBaadService->getUUID());
    /** If your device is battery powered you may consider setting scan response
     *  to false as it will extend battery life at the expense of less data sent.
     */
    //pAdvertising->setScanResponse(true);
    pAdvertising->start();

    logs("Advertising Started");
}

  /** Do your thing here, this just spams notifications to all connected clients */
    // if(pServer->getConnectedCount()) {
    //     NimBLEService* pSvc = pServer->getServiceByUUID("BAAD");
    //     if(pSvc) {
    //         NimBLECharacteristic* pChr = pSvc->getCharacteristic("F00D");
    //         if(pChr) {
    //             pChr->notify(true);
    //         }
    //     }
    // }
