/*
 * xiao_imu_ble.ino
 *
 * XIAO nRF52840 Sense IMU telemetry transmitter.
 *
 * Reads the onboard LSM6DS3 accelerometer and gyroscope and sends
 * averaged raw readings to a Raspberry Pi using the Adafruit
 * Bluefruit BLE UART service.
 *
 * The LSM6DS3 is configured for a 208 Hz output data rate. The
 * application reads the IMU approximately every 5 ms and averages
 * pairs of readings, producing one BLE telemetry packet approximately
 * every 10 ms.
 *
 * All six gyro/accelerometer axes are obtained with a single 12-byte
 * I2C transaction beginning at OUTX_L_G. This avoids performing six
 * separate I2C transactions and keeps the six axis measurements
 * together.
 *
 * Raw int16_t IMU values are transmitted. Conversion to acceleration
 * and angular velocity is performed by the Raspberry Pi.
 *
 * Packet format (20 bytes):
 *
 *     Offset  Size  Type       Description
 *     ------  ----  ---------  ----------------------------
 *       0       4   uint8_t    Synchronization bytes "MPU6"
 *       4       4   uint32_t   Timestamp, microseconds
 *       8       2   int16_t    Accelerometer X
 *      10       2   int16_t    Accelerometer Y
 *      12       2   int16_t    Accelerometer Z
 *      14       2   int16_t    Gyroscope X
 *      16       2   int16_t    Gyroscope Y
 *      18       2   int16_t    Gyroscope Z
 *
 * Multi-byte values are transmitted little-endian on the nRF52840.
 */

#include <bluefruit.h>
#include <LSM6DS3.h>
#include <Wire.h>


/*
 * BLE services.
 */
BLEDfu  bledfu;
BLEDis  bledis;
BLEUart bleuart;
BLEBas  blebas;


/*
 * Onboard LSM6DS3.
 */
LSM6DS3 imu(I2C_MODE, 0x6A);


/*
 * Four-byte synchronization marker.
 */
static constexpr uint8_t TELEMETRY_SYNC[4] = {
    0x4D, 0x50, 0x55, 0x36       // "MPU6"
};


/*
 * Two IMU readings are averaged for each transmitted packet.
 */
static constexpr uint32_t SAMPLES_PER_PACKET = 2;


/*
 * Read the IMU every 5 ms.
 *
 * The LSM6DS3 ODR is 208 Hz, corresponding to a new sample
 * approximately every 4.81 ms.
 */
static constexpr uint32_t SAMPLE_INTERVAL_US = 5000;


/*
 * Wire representation of one telemetry packet.
 *
 * The packed attribute guarantees that the compiler cannot insert
 * padding between fields.
 */
struct __attribute__((packed)) telemetry_packet_t {
    uint8_t  sync[4];
    uint32_t time_us;

    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;
};

static_assert(sizeof(telemetry_packet_t) == 20,
              "Telemetry packet must be exactly 20 bytes");


/*
 * 32-bit accumulators prevent overflow when adding int16_t samples.
 */
static int32_t ax_sum = 0;
static int32_t ay_sum = 0;
static int32_t az_sum = 0;

static int32_t gx_sum = 0;
static int32_t gy_sum = 0;
static int32_t gz_sum = 0;

static uint32_t sample_count = 0;
static uint32_t next_sample_us = 0;


/*
 * Configure the onboard LSM6DS3.
 *
 * Accelerometer:
 *      range       +/-2 g
 *      sample rate 208 Hz
 *      bandwidth   50 Hz
 *
 * Gyroscope:
 *      range       +/-500 degrees/sec
 *      sample rate 208 Hz
 *      bandwidth   50 Hz
 */
static bool setup_imu()
{
    imu.settings.accelEnabled    = 1;
    imu.settings.accelRange      = 2;
    imu.settings.accelSampleRate = 208;
    imu.settings.accelBandWidth  = 50;

    imu.settings.gyroEnabled     = 1;
    imu.settings.gyroRange       = 500;
    imu.settings.gyroSampleRate  = 208;
    imu.settings.gyroBandWidth   = 50;

    return imu.begin() == IMU_SUCCESS;
}


/*
 * Read all six gyro/accelerometer axes with one I2C transaction.
 *
 * The LSM6DS3 output registers are contiguous:
 *
 *      0x22 OUTX_L_G
 *      0x23 OUTX_H_G
 *      0x24 OUTY_L_G
 *      0x25 OUTY_H_G
 *      0x26 OUTZ_L_G
 *      0x27 OUTZ_H_G
 *      0x28 OUTX_L_XL
 *      0x29 OUTX_H_XL
 *      0x2A OUTY_L_XL
 *      0x2B OUTY_H_XL
 *      0x2C OUTZ_L_XL
 *      0x2D OUTZ_H_XL
 *
 * Each value is stored little-endian.
 *
 * Returns true if the I2C read succeeds.
 */
static bool read_imu(int16_t &ax, int16_t &ay, int16_t &az,
                     int16_t &gx, int16_t &gy, int16_t &gz)
{
    uint8_t raw[12];

    status_t status = imu.readRegisterRegion(
        raw,
        LSM6DS3_ACC_GYRO_OUTX_L_G,
        sizeof(raw)
    );

    if (status != IMU_SUCCESS) {
        return false;
    }

    gx = (int16_t)(((uint16_t)raw[1]  << 8) | raw[0]);
    gy = (int16_t)(((uint16_t)raw[3]  << 8) | raw[2]);
    gz = (int16_t)(((uint16_t)raw[5]  << 8) | raw[4]);

    ax = (int16_t)(((uint16_t)raw[7]  << 8) | raw[6]);
    ay = (int16_t)(((uint16_t)raw[9]  << 8) | raw[8]);
    az = (int16_t)(((uint16_t)raw[11] << 8) | raw[10]);

    return true;
}


/*
 * Read one complete IMU sample and add it to the accumulation
 * buffer.
 *
 * sample_count is incremented only following a successful I2C read.
 */
static bool accumulate_imu_sample()
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;

    if (!read_imu(ax, ay, az, gx, gy, gz)) {
        return false;
    }

    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;

    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;

    ++sample_count;

    return true;
}


/*
 * Clear the sample accumulators.
 */
static void clear_accumulators()
{
    ax_sum = 0;
    ay_sum = 0;
    az_sum = 0;

    gx_sum = 0;
    gy_sum = 0;
    gz_sum = 0;

    sample_count = 0;
}


/*
 * Generate and transmit a telemetry packet containing the average
 * of the two accumulated IMU samples.
 */
static void send_telemetry()
{
    telemetry_packet_t packet;

    memcpy(
        packet.sync,
        TELEMETRY_SYNC,
        sizeof(packet.sync)
    );

    /*
     * Timestamp the completed averaged measurement.
     *
     * micros() wraps approximately every 71.6 minutes. The Raspberry
     * Pi can handle this using unsigned 32-bit timestamp arithmetic.
     */
    packet.time_us = micros();

    packet.ax = (int16_t)(ax_sum / (int32_t)SAMPLES_PER_PACKET);
    packet.ay = (int16_t)(ay_sum / (int32_t)SAMPLES_PER_PACKET);
    packet.az = (int16_t)(az_sum / (int32_t)SAMPLES_PER_PACKET);

    packet.gx = (int16_t)(gx_sum / (int32_t)SAMPLES_PER_PACKET);
    packet.gy = (int16_t)(gy_sum / (int32_t)SAMPLES_PER_PACKET);
    packet.gz = (int16_t)(gz_sum / (int32_t)SAMPLES_PER_PACKET);

    /*
     * BLEUart TX is implemented using BLE notifications.
     *
     * Only attempt transmission if a central is connected and has
     * enabled notifications on the UART TX characteristic.
     */
    if (Bluefruit.connected() && bleuart.notifyEnabled()) {
        bleuart.write(
            reinterpret_cast<const uint8_t *>(&packet),
            sizeof(packet)
        );
    }

    clear_accumulators();
}


/*
 * Called when a BLE central connects.
 */
static void connect_callback(uint16_t conn_handle)
{
    BLEConnection *connection = Bluefruit.Connection(conn_handle);

    char central_name[32] = { 0 };

    connection->getPeerName(
        central_name,
        sizeof(central_name)
    );

    Serial.print("BLE connected");

    if (central_name[0] != '\0') {
        Serial.print(": ");
        Serial.print(central_name);
    }

    Serial.println();
}


/*
 * Called when the BLE connection is lost.
 */
static void disconnect_callback(uint16_t conn_handle,
                                uint8_t reason)
{
    (void)conn_handle;

    Serial.print("BLE disconnected, reason = 0x");
    Serial.println(reason, HEX);
}


/*
 * Configure and start BLE advertising.
 */
static void start_advertising()
{
    Bluefruit.Advertising.stop();

    Bluefruit.Advertising.clearData();
    Bluefruit.ScanResponse.clearData();

    /*
     * Put flags, transmit power, and the UART service UUID in the
     * advertising packet.
     */
    Bluefruit.Advertising.addFlags(
        BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE
    );

    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(bleuart);

    /*
     * Put the full device name in the scan response.
     */
    Bluefruit.ScanResponse.addName();

    /*
     * Resume advertising automatically following a disconnect.
     */
    Bluefruit.Advertising.restartOnDisconnect(true);

    /*
     * Advertising intervals are in 0.625 ms units:
     *
     *      32  =  20.0 ms
     *      244 = 152.5 ms
     */
    Bluefruit.Advertising.setInterval(32, 244);

    /*
     * Advertise at the fast rate for the first 30 seconds.
     */
    Bluefruit.Advertising.setFastTimeout(30);

    /*
     * Zero means advertise indefinitely.
     */
    Bluefruit.Advertising.start(0);
}


/*
 * Arduino initialization.
 */
void setup()
{
    Serial.begin(115200);

    /*
     * Allow a USB serial terminal to attach during development, but
     * don't prevent standalone operation if USB isn't connected.
     */
    uint32_t serial_wait_start = millis();

    while (!Serial &&
           ((uint32_t)(millis() - serial_wait_start) < 3000)) {
        delay(10);
    }

    Serial.println();
    Serial.println("XIAO nRF52840 Sense IMU BLE telemetry");

    /*
     * Initialize the IMU before making the BLE telemetry service
     * available.
     */
    if (!setup_imu()) {
        Serial.println("ERROR: LSM6DS3 initialization failed");

        while (true) {
            delay(1000);
        }
    }

    Serial.println("LSM6DS3 initialized");

    /*
     * Initialize the Bluefruit BLE stack in peripheral mode.
     */
    Bluefruit.begin();

    Bluefruit.setTxPower(4);
    Bluefruit.setName("XIAO-IMU");

    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

    /*
     * Device Firmware Update service.
     */
    bledfu.begin();

    /*
     * Device Information Service.
     */
    bledis.setManufacturer("Seeed Studio");
    bledis.setModel("XIAO nRF52840 Sense");
    bledis.begin();

    /*
     * Nordic UART Service.
     */
    bleuart.begin();

    /*
     * Battery service.
     *
     * Report 100% until actual battery monitoring is implemented.
     */
    blebas.begin();
    blebas.write(100);

    start_advertising();

    clear_accumulators();

    /*
     * Establish the first scheduled sampling time.
     */
    next_sample_us = micros() + SAMPLE_INTERVAL_US;

    Serial.println("BLE advertising started");
}


/*
 * Main loop.
 *
 * The IMU is read at nominal 5 ms intervals. Two successful readings
 * are accumulated and averaged before a BLE packet is generated.
 *
 * Advancing next_sample_us from its previous scheduled value rather
 * than from the current time prevents normal loop execution time from
 * accumulating as timing drift.
 *
 * Casting the timestamp difference to int32_t makes the comparison
 * safe across micros() rollover.
 */
void loop()
{
    uint32_t now = micros();

    if ((int32_t)(now - next_sample_us) >= 0) {

        next_sample_us += SAMPLE_INTERVAL_US;

        if (accumulate_imu_sample()) {

            if (sample_count >= SAMPLES_PER_PACKET) {
                send_telemetry();
            }
        }
    }
}
