/******************************************************************************
 * Freematics BLE OBD-II Demonstration
 *
 * This sketch gathers a snapshot of common OBD-II parameters and streams the
 * result over a BLE characteristic. The implementation focuses on clarity and
 * robustness:
 *   - BLE messages are chunked to respect MTU limits.
 *   - Serial output is mirrored to BLE via a small circular buffer.
 *   - OBD connection is monitored and re-established when required.
 *
 * Target: Freematics ONE+ (ESP32) with Arduino framework.
 ******************************************************************************/

#include <FreematicsPlus.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

namespace {

constexpr uint8_t LED_PIN = 4;                    // onboard LED
constexpr char BLE_DEVICE_NAME[] = "Freematics";  // BLE name
constexpr char BLE_SERVICE_UUID[] = "12345678-1234-1234-1234-123456789abc";
constexpr char BLE_CHAR_UUID[]    = "87654321-4321-4321-4321-ba0987654321";
constexpr uint8_t API_VERSION = 1;
constexpr size_t MAX_NOTIFY = 180;                // safe chunk payload size

struct PIDPollingInfo {
  uint8_t   pid;
  uint8_t   tier;
  int       value;
  uint32_t  ts;
};

PIDPollingInfo obdData[] = {
  {PID_SPEED,          1},
  {PID_RPM,            1},
  {PID_THROTTLE,       1},
  {PID_ENGINE_LOAD,    1},
  {PID_INTAKE_MAP,     1},
  {PID_MAF_FLOW,       1},
  {PID_COOLANT_TEMP,   2},
  {PID_INTAKE_TEMP,    2},
  {PID_FUEL_PRESSURE,  2},
  {PID_TIMING_ADVANCE, 2},
  {PID_FUEL_LEVEL,     3},
};

constexpr size_t SERIAL_BUFFER_SIZE = 10;
constexpr size_t MAX_SERIAL_MSG_LENGTH = 100;

struct SerialMessage {
  String message;
  unsigned long timestamp;
  bool used;
};

SerialMessage serialBuffer[SERIAL_BUFFER_SIZE];
size_t serialBufferIndex = 0;
size_t serialBufferCount = 0;

BLEServer*         bleServer        = nullptr;
BLECharacteristic* bleCharacteristic = nullptr;
bool bleClientConnected = false;

FreematicsESP32 sys;
COBD obd;

void bleNotifyChunked(const String& s) {
  if (!bleClientConnected || !bleCharacteristic) return;
  int start = 0;
  while (start < (int)s.length()) {
    int end = start + MAX_NOTIFY;
    if (end > (int)s.length()) end = s.length();
    bleCharacteristic->setValue(s.substring(start, end).c_str());
    bleCharacteristic->notify();
    delay(5);
    start = end;
  }
}

void addSerialMessage(const String& message) {
  if (message.isEmpty() || message.length() > MAX_SERIAL_MSG_LENGTH) return;
  serialBuffer[serialBufferIndex] = {message, millis(), false};
  serialBufferIndex = (serialBufferIndex + 1) % SERIAL_BUFFER_SIZE;
  if (serialBufferCount < SERIAL_BUFFER_SIZE) serialBufferCount++;
}

void transmitSerialBuffer() {
  if (!bleClientConnected || !bleCharacteristic) return;
  int sent = 0;
  for (size_t i = 0; i < SERIAL_BUFFER_SIZE && sent < 3; ++i) {
    int index = (serialBufferIndex + SERIAL_BUFFER_SIZE - 1 - i) % SERIAL_BUFFER_SIZE;
    auto& entry = serialBuffer[index];
    if (!entry.used && entry.message.length() > 0) {
      String packet = String(entry.timestamp) + ":" + entry.message;
      bleNotifyChunked(packet);
      entry.used = true;
      ++sent;
    }
  }
}

class BLESerial {
public:
  static void println(const String& message) {
    Serial.println(message);
    addSerialMessage(message);
    if (bleClientConnected) bleNotifyChunked(message);
  }

  static void print(const String& message) {
    Serial.print(message);
    String clean = message;
    clean.replace("\n", "");
    addSerialMessage(clean);
    if (bleClientConnected) bleNotifyChunked(clean);
  }
};

void sendObdSnapshot() {
  String line;
  line.reserve(160);
  line += "[" + String(millis()) + "] ";

  int v;
  if (obd.readPID(PID_RPM, v))            line += "RPM:" + String(v) + ' ';
  if (obd.readPID(PID_SPEED, v))          line += "SPD:" + String(v) + ' ';
  if (obd.readPID(PID_ENGINE_LOAD, v))    line += "LOAD:" + String(v) + ' ';
  if (obd.readPID(PID_THROTTLE, v))       line += "THR:" + String(v) + ' ';
  if (obd.readPID(PID_INTAKE_MAP, v))     line += "MAP:" + String(v) + ' ';
  if (obd.readPID(PID_MAF_FLOW, v))       line += "MAF:" + String(v) + ' ';
  if (obd.readPID(PID_COOLANT_TEMP, v))   line += "ECT:" + String(v) + ' ';
  if (obd.readPID(PID_INTAKE_TEMP, v))    line += "IAT:" + String(v) + ' ';
  if (obd.readPID(PID_TIMING_ADVANCE, v)) line += "ADV:" + String(v) + ' ';
  if (obd.readPID(PID_FUEL_PRESSURE, v))  line += "FP:"  + String(v) + ' ';
  if (obd.readPID(PID_FUEL_LEVEL, v))     line += "FUEL%:" + String(v) + ' ';

  line += "VBAT:" + String(obd.getVoltage(), 2) + 'V';
  line += " CPUT:" + String(readChipTemperature());
  BLESerial::println(line);
}

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    bleClientConnected = true;
    BLESerial::println("=== BLE CLIENT CONNECTED ===");
    BLESerial::println("ConnID: " + String(server->getConnId()));
    BLESerial::println("Starting data collection...");

    if (bleCharacteristic) {
      String msg = "0:" + String(millis()) + ",CONNECT:SUCCESS,API_VERSION:" + String(API_VERSION) + ";";
      bleCharacteristic->setValue(msg.c_str());
      bleCharacteristic->notify();
      BLESerial::println("BLE TX CONNECT: " + msg);
    }
  }

  void onDisconnect(BLEServer* server) override {
    bleClientConnected = false;
    BLESerial::println("=== BLE CLIENT DISCONNECTED ===");
    BLESerial::println("Restarting advertising...");
    server->getAdvertising()->start();
    BLESerial::println("Waiting for new BLE client connection...");
  }
};

void initBle() {
  BLESerial::println("Initialising BLE...");
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEDevice::setMTU(185);

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());

  BLEService* service = bleServer->createService(BLE_SERVICE_UUID);
  bleCharacteristic = service->createCharacteristic(
    BLE_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  auto descriptor = new BLE2902();
  descriptor->setNotifications(true);
  bleCharacteristic->addDescriptor(descriptor);
  service->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x0);
  advertising->start();
  BLESerial::println("BLE ready");
}

bool connectObd() {
  BLESerial::print("Connecting to OBD...");
  if (obd.init()) {
    BLESerial::println("OK");
    return true;
  }
  BLESerial::println("FAILED");
  return false;
}

} // namespace

bool obdConnected = false;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  initBle();

  while (!sys.begin());
  obd.begin(sys.link);
}

void loop() {
  digitalWrite(LED_PIN, HIGH);

  if (!obdConnected) {
    if (connectObd()) {
      obdConnected = true;
    } else {
      digitalWrite(LED_PIN, LOW);
      delay(250);
      return;
    }
  }

  sendObdSnapshot();
  transmitSerialBuffer();

  if (obd.errors > 2) {
    BLESerial::println("OBD disconnected");
    obdConnected = false;
    obd.reset();
  }

  digitalWrite(LED_PIN, LOW);
  delay(100);
}
