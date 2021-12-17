#include <mcp_can_dfs_2.h>
#include <mcp_can_2.h>

#include <SPI.h>
#include <Arduino.h>

#define DEBUG_MODE 1
// Globals for EPS Pump status
bool pump_is_running;
bool vehicle_is_running;
bool override_enabled;

const int INT_PUMP = 6;
const int INT_VEHICLE = 7;
const int SPI_CS_PUMP = 8;
const int SPI_CS_VEHICLE = 9;

MCP_CAN can_vehicle(SPI_CS_VEHICLE);
MCP_CAN can_pump(SPI_CS_PUMP);

short int previous_steering_angle = 0;
short int current_steering_angle = 0;
short int target_steering_angle = 0;
bool current_steering_angle_valid = false;

long last_steering_time = 0;

// Globals for CANBUS debug / testing
uint32_t counter = 0;

void convert_201_values(uint8_t buf[]) {
    // RPM = uint16_t(bytes 0, 1) * 4
    uint16_t engine_speed = ((buf[0] * 256) + buf[1]) / 4;
    //Serial.print("Engine speed from vehicle network: ");
    //Serial.println(engine_speed);

    // Speed in KM/hr = (uint16_t(bytes 4. 5) + 100) * 100
    uint16_t vehicle_speed = (((buf[4] * 256) + buf[5]) / 100) - 100;
    //Serial.print("Vehicle speed from vehicle network: ");
    //Serial.println(vehicle_speed);

    // Mazda 3 pump expects RPM as raw bytes, no scalar like MX5
    // Mazda 3 pump expects km/hr*100, no 100 km offset like MX5
    vehicle_speed *= 100;

    // Set vehicle running flag
    //vehicle_is_running = engine_speed < 500 ? false : true;

    // Conditionally override engine speed if enabled
    if (override_enabled) {
      engine_speed = 1000;
    }
    // Write modified values back into buffer
    buf[0] = engine_speed / 256;
    buf[1] = engine_speed % 256;
    buf[4] = vehicle_speed / 256;
    buf[5] = vehicle_speed % 256;
}

void print_message_contents(uint32_t id, uint8_t len, uint8_t buf[]) {
    Serial.print("0x");
    Serial.print(id, HEX);
    Serial.print(": ");
    for (uint8_t i = 0; i < len; i++) {
        Serial.print("0x");
        Serial.print(buf[i], HEX);
        Serial.print(", ");
    }
    Serial.println();
}

// current_steering_angle and target_steering_able are doubled to allow for .5* resolution without floating point conversion.
void update_steering_outputs() {
  // Don't run too fast
  if (micros() - last_steering_time < 500) return;
  
  if (target_steering_angle > current_steering_angle) {
     current_steering_angle++;
  } else if (target_steering_angle < current_steering_angle) {
     current_steering_angle--;
  }

  digitalWrite(4, abs((current_steering_angle / 6) % 2));
  digitalWrite(5, abs((((current_steering_angle + 3 )/ 6)% 2)));  
  last_steering_time = micros();
}

unsigned char run_pump[8] = {0x23, 0xE8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char stop_pump[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void setup()
{
    Serial.begin(115200);

    // Setup digital outputs for steering angle sensor
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);

    uint8_t ret;

    ret =  can_vehicle.begin(MCP_ANY, 13, MCP_8MHZ);
    if (ret == CAN_OK)
    {
        Serial.println("Vehicle CAN Initialized Successfully!");
        can_vehicle.setMode(MCP_NORMAL);
    } else
    {
        Serial.println("Error Initializing Vehicle CAN...");
        while(1);
    }

    ret = can_pump.begin(MCP_ANY, 13, MCP_8MHZ);
    if (ret == CAN_OK)
    {
        Serial.println("Pump CAN Initialized Successfully!");
        can_pump.setMode(MCP_NORMAL);
    } else
    {
        Serial.println("Error Initializing Pump CAN...");
        while(1);
    }
}

// Main program loop.  Poll both CAN networks.  If 0x201 received from vehicle network, modify and rebroadcast to pump network.

void loop()
{
    update_steering_outputs();
    
    uint32_t id;
    uint8_t len;
    uint8_t buf[24];
    uint8_t ret;

    // Check interrupt pin for vehicle CAN
    if(!digitalRead(INT_VEHICLE)) {
        ret = can_vehicle.readMsgBuf(&id, &len, buf);

        // Check if 0x201 ID
        if (id == 0x201) {
            // 0x201 - modify and rebroadcast to pump
            Serial.print("Received from vehicle CAN: ");
            print_message_contents(id, 8, buf);
            convert_201_values(buf);
            Serial.print("Sending to pump CAN: ");
            print_message_contents(id, len, buf);
            can_pump.sendMsgBuf(0x201, 0, len, buf);
        } else if (id == 0x081) {
            Serial.print("Received from vehicle CAN: ");
            print_message_contents(id, len, buf);
            
            // Convert from steering angle message format to degrees
            int16_t steering_angle;
            unsigned char *ip = (unsigned char *) &steering_angle;
            ip[0] = buf[3];
            ip[1] = buf[2];
            uint16_t u_steering_angle = (steering_angle * 10 ) + 1600;
            ip = (unsigned char *) &u_steering_angle;

            Serial.print("Steering angle from sensor: ");
            Serial.println(steering_angle);  

            // We initialize steering angle to zero.  This conditional makes sure there aren't any wild jumps in output from the first steering message.
            if (!current_steering_angle_valid) {
              current_steering_angle_valid = true;
              current_steering_angle = steering_angle * 2;
              previous_steering_angle = current_steering_angle;       
            } 
            
            // Drive outputs for early version Mazda 3 pump (pre 2009)
            target_steering_angle = steering_angle * 2;
            update_steering_outputs();

            // Send canbus message for late version Mazda 3 pump (2009+)           
            // byte 6 = steering rate of change.  Pump reaches max speed when delta is 0x10. byte 1 must be nonzero? more than 1?
            //unsigned char tx_steering[8] = { 0, 50, 0, 0, abs(current_steering_angle*2 - previous_steering_angle)*10, abs(current_steering_angle*2 - previous_steering_angle)*10, abs(current_steering_angle*2 - previous_steering_angle)*10, 0 };
            //unsigned char tx_steering[8] = { 0, 50, 0, 0, 0, 0, counter / 255, 0 };
            unsigned char tx_steering[8] = { 0, 50, 0, 0, 0, 0, abs(target_steering_angle - previous_steering_angle)*1, 0 };
            print_message_contents(0x82, 8, tx_steering);
            can_pump.sendMsgBuf(0x82, 0, 8, tx_steering);

            previous_steering_angle = current_steering_angle;
        } else {
            //Serial.print("Got something new on vehicle CAN! ");
            //print_message_contents(id, 8, buf);
        }
    } else {
      //Serial.println("No message on vehicle CAN");
    }

    // Check interrupt pin for pump CAN
    if(!digitalRead(INT_PUMP)) {
        ret = can_pump.readMsgBuf(&id, &len, buf);
        
        if (id == 0x4DA) {
            // Only from early Mazda 3 pump
            // Serial.print("Received from pump CAN: ");
            // print_message_contents(id, len, buf);
            
            uint16_t steering_angle;
            unsigned char *ip = (unsigned char *) &steering_angle;
            ip[0] = buf[1];
            ip[1] = buf[0];

            steering_angle = (steering_angle - 32768 ) / 10;
            
            Serial.print("Steering angle from pump: ");
            Serial.println(steering_angle);
        } else if (id == 0x240) {
            // Containes information about pump RPM / current
            Serial.print("Received from pump CAN: ");
            print_message_contents(id, len, buf);

        } else {
            //Serial.print("Recieved from pump CAN: ");
            //print_message_contents(id, len, buf);
        }
    } else {
      //Serial.println("No message on pump CAN");
    }
    
}
