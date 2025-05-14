#include <H4.h>
#include <H4Plugins.h>

class DataPortPlugin : public H4Plugin {
    public:
        DataPortPlugin() : H4Plugin("DataPortPlugin") {
            // Set a timer to check the port every 50 ms
            h4.every(50, [this]() {
                checkPort();
            });
        }

        void checkPort() {
            if (dataAvailableOnPort()) {
                h4.cancelSingleton(H4P_TRIG1); // Cancel the timer if data is available
                handleData(); // Process the data
            }
        }

        bool dataAvailableOnPort() {
            // Implement port-specific logic to check if data is available
            // e.g., Serial.available() for serial, or use SDI-12 or other communication protocols
            return Serial.available();
        }

        void handleData() {
            String data = Serial.readString(); // Example for serial port
            Serial.print("Data received: ");
            Serial.println(data);
            // You can dispatch tasks or callbacks from here
        }
};

// Instantiate your plugin
DataPortPlugin dataPortPlugin;
