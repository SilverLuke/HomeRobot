/* GY87 READER

- GY-87: Parzialmente rotto si leggono soltanto gli address `0x68` e `0x77`. Sensore fatto da 3 chip:
    - MPU6050: Funge (gyro+acc)
    - HMC5883L: Non Funge  (Magentic)
    - BMP180:  Funge  (Pressione e temperatura)

lib_deps =
    jrowberg/I2Cdevlib-MPU6050@^1.0.0
    jrowberg/I2Cdevlib-HMC5883L@^1.0.0
    lowpowerlab/BMP180
*/
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "SFE_BMP180.h"
#include "Wire.h"

static const char LED = 6;
static const float ACCEL_SENS = 16384.0; // Accel Sensitivity with default +/- 2g scale
static const float GYRO_SENS = 131.0; // Gyro Sensitivity with default +/- 250 deg/s scale

// Magnetometer class default I2C address is 0x1E
// specific I2C addresses may be passed as a parameter here
// this device only supports one I2C address (0x1E)
HMC5883L mag;
int16_t mx, my, mz;

// Accel/Gyro class default I2C address is 0x68 (can be 0x69 if AD0 is high)
// specific I2C addresses may be passed as a parameter here
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;


// MPB180
SFE_BMP180 pressure;
double baseline; // baseline pressure


char getPressure(double *temp, double *pres) {
    char status;

    // You must first get a temperature measurement to perform a pressure reading.

    // Start a temperature measurement:
    // If request is successful, the number of ms to wait is returned.
    // If request is unsuccessful, 0 is returned.

    status = pressure.startTemperature();
    if (status != 0) {
        // Wait for the measurement to complete:

        delay(status);

        // Retrieve the completed temperature measurement:
        // Note that the measurement is stored in the variable T.
        // Use '&T' to provide the address of T to the function.
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getTemperature(*temp);
        if (status != 0) {
            // Start a pressure measurement:
            // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
            // If request is successful, the number of ms to wait is returned.
            // If request is unsuccessful, 0 is returned.

            status = pressure.startPressure(3);
            if (status != 0) {
                // Wait for the measurement to complete:
                delay(status);

                // Retrieve the completed pressure measurement:
                // Note that the measurement is stored in the variable P.
                // Use '&P' to provide the address of P.
                // Note also that the function requires the previous temperature measurement (T).
                // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
                // Function returns 1 if successful, 0 if failure.

                status = pressure.getPressure(*pres, *temp);
                if (status != 0) {
                    return 0;
                }
                Serial.println("error retrieving pressure measurement\n");
            } else
                Serial.println("error starting pressure measurement\n");
        } else
            Serial.println("error retrieving temperature measurement\n");
    } else
        Serial.println("error starting temperature measurement\n");
    return 1;
}

void setup() {
    boolean state = HIGH;
    unsigned int count = 0;

    pinMode(LED, OUTPUT);

    Serial.begin(115200);
    while (!Serial && (count < 30)) {
        delay(200); // Wait for serial port to connect with timeout. Needed for native USB
        digitalWrite(LED, state);
        state = !state;
        count++;
    }

    digitalWrite(LED, HIGH);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin(20, 21);

    // ==================== MPU6050 ============================
    accelgyro.initialize();
    Serial.print("Testing Accel/Gyro... ");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    while ( ! accelgyro.testConnection()) {
        delay(500);
        accelgyro.initialize();
        Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    }
        // Starts up with accel +/- 2 g and gyro +/- 250 deg/s scale
        accelgyro.setI2CBypassEnabled(true); // set bypass mode
    // Now we can talk to the HMC5883l

    // ==================== HMC5883L ============================
    // mag.initialize();
    // Serial.print("Testing Mag...  ");
    // Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
    // while ( ! mag.testConnection()) {
    //     delay(500);
    //     mag.initialize();
    //     Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
    // }
    // ==================== BMP180 ============================

    while ( ! pressure.begin()) {
        Serial.println("BMP180 init fail (disconnected?)\n\n");
        sleep(500);
    }
    Serial.println("BMP180 init success");
    double temp;
    char status = getPressure(&temp, &baseline);
    if (status != 0) {
        Serial.println("error getting pressure measurement\n");
    } else {
        Serial.print("baseline pressure: ");
        Serial.print(baseline);
        Serial.print(" mb ");
        Serial.print(temp);
        Serial.println(" C");
    }
}

void loop() {
    static unsigned long ms = 0;
    static boolean state = HIGH;

    // Serial Output Format
    // === Accel === | === Gyro === | ======= Mag ======= | === Barometer === |
    //   X   Y   Z   |  X   Y   Z   |  X   Y   Z  Heading |  Temp   Pressure  |
    double a, pres, temp;


    if (millis() - ms > 1000) {
        // read raw accel/gyro measurements
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // display tab-separated accel/gyro x/y/z values
        Serial.print("Accel X Y Z: ");
        Serial.print(ax / ACCEL_SENS);
        Serial.print("\t");
        Serial.print(ay / ACCEL_SENS);
        Serial.print("\t");
        Serial.print(az / ACCEL_SENS);
        Serial.print("\t");
        Serial.print("Gyro X Y Z: ");
        Serial.print(gx / GYRO_SENS);
        Serial.print("\t");
        Serial.print(gy / GYRO_SENS);
        Serial.print("\t");
        Serial.print(gz / GYRO_SENS);
        Serial.print("\n");

        // // read raw heading measurements
        // mag.getHeading(&mx, &my, &mz);
        //
        // // display tab-separated mag x/y/z values
        // Serial.print("Heading X Y Z: ");
        // Serial.print(mx);
        // Serial.print("\t");
        // Serial.print(my);
        // Serial.print("\t");
        // Serial.print(mz);
        // Serial.print("\n");
        //
        // // To calculate heading in degrees. 0 degree indicates North
        // float heading = atan2(my, mx);
        // if (heading < 0) heading += 2 * M_PI;
        // Serial.print(heading * 180 / M_PI);
        // Serial.print("\t");


        // Get a new pressure reading:
        double temp, press;
        char status = getPressure(&temp, &press);
        if (status != 0) {
            Serial.println("error getting pressure measurement\n");
        } else {
            Serial.print("pressure: ");
            Serial.print(baseline);
            Serial.print(" mb");
            Serial.print(temp);
            Serial.println(" C");
        }

        // Show the relative altitude difference between
        // the new reading and the baseline reading:
        Serial.print("relative altitude: ");
        if (a >= 0.0)
            Serial.print(" "); // add a space for positive numbers
        Serial.print(a, 1);
        Serial.print(" meters, ");
        if (a >= 0.0)
            Serial.print(" "); // add a space for positive numbers
        Serial.print(a * 3.28084, 0);
        Serial.println(" feet");

        ms = millis();
        digitalWrite(LED, state);
        state = !state;
    }
}