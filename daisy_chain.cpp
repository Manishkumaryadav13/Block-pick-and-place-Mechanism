#include <stdio.h>
#include <math.h>
#include "../../../src/dynamixel2pico/Dynamixel2Pico.h"

#include "pico/bootrom.h"
#define UART_ID1    uart1
#define DXL_SERIAL1  UART_ID1
#define UART_RX_PIN1   5
#define UART_TX_PIN1   4
#define DXL_DIR_PIN1   6


#define UART_ID0    uart0
#define DXL_SERIAL0  UART_ID0
#define UART_RX_PIN0   1
#define UART_TX_PIN0   0
#define DXL_DIR_PIN0   2

using namespace ControlTableItem;
//Please see eManual Control Table section of your DYNAMIXEL.
//This example is written for DYNAMIXEL X series(excluding XL-320)
#define OPERATING_MODE_ADDR         11
#define OPERATING_MODE_ADDR_LEN     1
#define TORQUE_ENABLE_ADDR          64
#define TORQUE_ENABLE_ADDR_LEN      1
#define LED_ADDR                    65
#define LED_ADDR_LEN                1
#define GOAL_POSITION_ADDR          116
#define GOAL_POSITION_ADDR_LEN      4
#define PRESENT_POSITION_ADDR       132
#define PRESENT_POSITION_ADDR_LEN   4
#define POSITION_CONTROL_MODE       3
#define TIMEOUT 10    //default communication timeout 10ms


uint8_t turn_on = 1;
uint8_t turn_off = 0;
uint8_t DXL_ID = 1;
const uint8_t DXL_1_ID = 1;
const uint8_t DXL_2_ID = 2;
const uint8_t DXL_3_ID = 3;
const uint8_t numOfMotors = 3;
const uint8_t DXL_IDS[numOfMotors] = {DXL_1_ID, DXL_2_ID, DXL_3_ID};
const float DXL_PROTOCOL_VERSION = 2.0;
volatile int16_t goal_position;
uint8_t operatingMode = POSITION_CONTROL_MODE;
Dynamixel2Pico dxl1(DXL_SERIAL1, DXL_DIR_PIN1, UART_RX_PIN1, UART_TX_PIN1);
Dynamixel2Pico dxl0(DXL_SERIAL0, DXL_DIR_PIN0, UART_RX_PIN0, UART_TX_PIN0);

float kp = 400;
float ki = 5;
float kd = 0;

uint32_t userInput = 0;
int command = 0;
bool ret = false;
#define SECRET_CODE     64209
#define DEFAULT_VELOCITY 50
// ----------------------------------------------------------------
// User defined functions definitions -----------------------------
void moveTo(Dynamixel2Pico dxl_num, uint8_t ID, uint32_t position);

void moveToAngle(Dynamixel2Pico dxl_num, uint8_t ID, uint32_t angle);

void moveMotorToAngle(Dynamixel2Pico dxl_num, const uint8_t ID, uint32_t angle);

float getMotorPosition(Dynamixel2Pico dxl_num, const uint8_t ID, uint8_t unit);

float getPosition(Dynamixel2Pico dxl_num, uint8_t id, uint8_t unit = (uint8_t)0U);

float getCurrentAngle(Dynamixel2Pico dxl_num, uint8_t id, uint8_t unit = (uint8_t)0U);

uint32_t parse_numeric_value();

void dynamixelSetup(Dynamixel2Pico dxl_num, const uint8_t totalMotors);

float getCurrentVelocity(Dynamixel2Pico dxl_num, uint8_t id, uint8_t unit);

void setVelocity(Dynamixel2Pico dxl_num, uint8_t id, uint32_t velocity);

void new_loop(Dynamixel2Pico dxl_num, uint8_t ID);
// ----------------------------------------------------------------
int main() {
    
    stdio_init_all();
    sleep_ms(2000);
    // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
    dynamixelSetup(dxl0, 2);
    dynamixelSetup(dxl1, 2);
    
    while (true) {   
      uint8_t command = getchar_timeout_us(0);
      if(command =='U'|| command =='u'){
        uint8_t subcommand = getchar_timeout_us(0);
        if(subcommand =='2'){
          printf("DynamixelController\n");
        }
      }
      if(command == 'I'){
        uint8_t subcommand = getchar_timeout_us(0);
        if(subcommand == '0'){
          uint8_t DXL_ID_dummy = parse_numeric_value();
          if(DXL_ID_dummy > 0 && DXL_ID_dummy < 100){
            DXL_ID = DXL_ID_dummy;
            new_loop(dxl0, DXL_ID);
          }
          else if (DXL_ID_dummy == 0){
            for(int id = 0; id < DXL_BROADCAST_ID; id++) {
              //iterate until all ID in each buadrate is scanned.
              if(dxl0.ping(id)) {
                printf("ID : ");
                printf(" %d ",id);
                printf(", Model Number: ");
                printf(" %d \n", dxl0.getModelNumber(id));
              }
            }
          }
        }
        else if(subcommand == '1'){
          uint8_t DXL_ID_dummy = parse_numeric_value();
          if(DXL_ID_dummy > 0 && DXL_ID_dummy < 100){
            DXL_ID = DXL_ID_dummy;
            new_loop(dxl1, DXL_ID);
          }
          else if (DXL_ID_dummy == 0){
            for(int id = 0; id < DXL_BROADCAST_ID; id++) {
              //iterate until all ID in each buadrate is scanned.
              if(dxl1.ping(id)) {
                printf("ID : ");
                printf(" %d ",id);
                printf(", Model Number: ");
                printf(" %d \n", dxl1.getModelNumber(id));
              }
            }
          }
        }
      }
      // else if(command == '?'){
      //   int16_t current_position[numOfMotors];  
      //   for(int i = 0; i < numOfMotors; i++){
      //     current_position[i] = getPosition(dxl0, DXL_IDS[i]);  
      //     printf("%d ", current_position[i]);
      //   }
      //   printf("\n");
      // } 
      else if (command == '#'){   
      command = getchar_timeout_us(0);
      if(command == 'r' || command == 'R'){
          command = getchar_timeout_us(0);
          if(command == 'p' || command == 'P'){
              int dummy_data = parse_numeric_value();
              if(dummy_data == SECRET_CODE){
                  reset_usb_boot(0, 0);
              }
          }
        }
      }
    }
    return 0;
}

//----------------------------------------------------------------
void moveTo(Dynamixel2Pico dxl_num, uint8_t ID, uint32_t position) {
  dxl_num.write(ID, GOAL_POSITION_ADDR, (uint8_t*)&position, GOAL_POSITION_ADDR_LEN, TIMEOUT);
}

void moveToAngle(Dynamixel2Pico dxl_num, uint8_t ID, uint32_t angle){
  dxl_num.setGoalPosition(ID, angle, UNIT_DEGREE);
}

void moveMotorToAngle(Dynamixel2Pico dxl_num, uint8_t ID, uint32_t angle){
  dxl_num.setGoalPosition(ID, angle, UNIT_DEGREE);
  printf("Motor ID: %d \t Position(angle): %d\n", ID, angle);
}

float getCurrentAngle(Dynamixel2Pico dxl_num, uint8_t id, uint8_t unit){
  float position = -1;
  position = dxl_num.getPresentPosition(id,unit);
  sleep_ms(100);
  printf("Present position(angle): %f\n",position);
  return position;
}

float getPosition(Dynamixel2Pico dxl_num, uint8_t id, uint8_t unit){
  float position = -1;
  position = dxl_num.getPresentPosition(id, unit);
  sleep_ms(100);
  // printf("Present position: %f\n", position);

  return position;
}

float getMotorPosition(Dynamixel2Pico dxl_num, const uint8_t ID, uint8_t unit){
  float position = -1;
  position = dxl_num.getPresentPosition(ID, unit);
  sleep_ms(100);
  printf("Present position: %f\n", position);

  return position;
}

float getCurrentVelocity(Dynamixel2Pico dxl_num, uint8_t id, uint8_t unit){
  int velocity = -1;
  velocity = dxl_num.getPresentVelocity(id, unit);
  sleep_ms(100);
  printf("Present velocity(RPM): %d\n", velocity);
  return velocity;
}

void setVelocity(Dynamixel2Pico dxl_num, uint8_t id, uint32_t velocity){
  bool ret = dxl_num.writeControlTableItem(PROFILE_VELOCITY, id, velocity);
  sleep_ms(100);
  if(ret){
    printf("Successfully set goal velocity(RPM): %d\n", velocity);
  }
  else{
    printf("Error: Set goal velocity failed\n");
  }
}

uint32_t parse_numeric_value(){
    int buffer = 0, decimal_power = 0;
    float var = 0;
    bool neg_data = false, first_byte = true, decimal_data = false;
    while (true)
    {   
        buffer = getchar_timeout_us(100);
        if(first_byte && buffer == '-'){
            neg_data = true;
            first_byte = false;
        }
        if(buffer>='0' && buffer<='9'){    
            var = var*10 + (buffer - 48);
            if (decimal_data){
                decimal_power++;
            }
        }
        else if(buffer == '.'){
            decimal_data = true;
        }
        else if(buffer == '\n' || buffer == ' ' || buffer == ','){
            break;
        }    
        else if (buffer <= 0 || buffer == '-'){
            return PICO_ERROR_TIMEOUT;

        }
    }
    if (neg_data == true) var = -var;
    if(decimal_data)    var = var/(pow(10, decimal_power));
    return var;
}
void dynamixelSetup(Dynamixel2Pico dxl_num, uint8_t id){
  // Get DYNAMIXEL information
    dxl_num.begin(57600);
    // dxl_num.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    // Turn off torque when configuring items in EEPROM area
    if(dxl_num.write(id, TORQUE_ENABLE_ADDR, (uint8_t*)&turn_off , TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
      printf("DYNAMIXEL Torque off\n");
    else
      printf("Error: Torque off failed\n");

    // Set Operating Mode
    // if(dxl_num.write(id, OPERATING_MODE_ADDR, (uint8_t*)&operatingMode, OPERATING_MODE_ADDR_LEN, TIMEOUT))
    //   printf("Set operating mode\n");
    // else
    //   printf("Error: Set operating mode failed\n");

    dxl_num.writeControlTableItem(POSITION_P_GAIN, id, kp);
    dxl_num.writeControlTableItem(POSITION_I_GAIN, id, ki);
    dxl_num.writeControlTableItem(POSITION_D_GAIN, id, kd);

    // Turn on torque
    if(dxl_num.write(id, TORQUE_ENABLE_ADDR, (uint8_t*)&turn_on, TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
      printf("Torque on\n");
    else
      printf("Error: Torque on failed\n");
    setVelocity(dxl_num, id, DEFAULT_VELOCITY);
} 

void new_loop(Dynamixel2Pico dxl_type, uint8_t ID){
  uint8_t command = getchar_timeout_us(50);               
  if(command == 'P' || command == 'p') {            
    uint8_t sub_command = getchar_timeout_us(0);
    if(sub_command == '?'){
      float current_position = getPosition(dxl_type, ID);
      printf("current position: %f\n", current_position);
    }
    else if(sub_command == '='){
      userInput = parse_numeric_value();
      if(userInput >= 0){ 
          goal_position = userInput; // Set Goal Position
          float current_position = getPosition(dxl_type, ID);
          printf("Goal Position : %f \n", goal_position);
          printf("current Position : %f \n", current_position);
          moveTo(dxl_type, ID, goal_position);
      }
    }
  }
  else if(command == 'I'){
    uint8_t newID = parse_numeric_value();
    if (newID > 0 && newID <200){
      printf("writing new ID %d to %d\n", newID, DXL_ID);
      dxl_type.torqueOff(DXL_ID);
      printf(dxl_type.setID(DXL_ID, newID)?"Success":"failed");
      dxl_type.torqueOn(newID);
      printf("\n");
    }
  }
  else if(command == 'A' || command == 'a'){
    uint8_t sub_command = getchar_timeout_us(0);
    
    if(sub_command == '?'){
      float current_angle = getCurrentAngle(dxl_type, ID);
      printf("Present angle: %f\n", current_angle);
    }
    else if(sub_command == '='){
      userInput = parse_numeric_value();
      if(userInput >= 0 && userInput <= 360){
          printf("Goal angle : %" PRIu32 "\n", userInput);
          moveToAngle(dxl_type, ID, userInput); // Set Goal Angle
      }
    } 
  }
  else if (command == 'v' || command == 'V'){
    uint8_t sub_command = getchar_timeout_us(0);
    printf("Command: %c sub-command: %c\n",command, sub_command);
    if(sub_command == '?'){
      float current_velocity = getCurrentVelocity(dxl_type, DXL_ID, UNIT_RAW);
      printf("Current Velocity: %f", current_velocity);
    }
    else if (sub_command == '='){
      userInput = parse_numeric_value();
      dxl_type.torqueOff(DXL_ID);
      setVelocity(dxl_type, DXL_ID, userInput);
      dxl_type.torqueOn(DXL_ID);
    }
  }
  else if(command == 'l'){
    printf("Blinking Dynamixel LED\n");
    dxl_type.ledOn(DXL_ID);  // Turn on the LED on DYNAMIXEL
    sleep_ms(200);
    dxl_type.ledOff(DXL_ID); // Turn off the LED on DYNAMIXEL
    sleep_ms(200);
  }
  // else if(command == 'U'|| command == 'u'){
  //   printf("DynamixelController\n");
  //   sleep_ms(200);
  // }
  else if(command == 'k'){
    uint8_t subcommand = getchar_timeout_us(0);
    if(subcommand == '?'){
      uint16_t p = dxl_type.readControlTableItem(POSITION_P_GAIN, DXL_ID);
      uint16_t i = dxl_type.readControlTableItem(POSITION_I_GAIN, DXL_ID);
      uint16_t d = dxl_type.readControlTableItem(POSITION_D_GAIN, DXL_ID);
      printf("pid values: %d %d %d\n", p, i, d);
    }
    else if(subcommand == 'p'){
      uint16_t userInput = parse_numeric_value();
      dxl_type.writeControlTableItem(POSITION_P_GAIN, DXL_ID, userInput);
    }
    else if(subcommand == 'i'){
      uint16_t userInput = parse_numeric_value();
      dxl_type.writeControlTableItem(POSITION_I_GAIN, DXL_ID, userInput); 
    }
    else if(subcommand == 'd'){
      uint16_t userInput = parse_numeric_value();
      dxl_type.writeControlTableItem(POSITION_D_GAIN, DXL_ID, userInput);
    }

  }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
  else if(command == 'r'){
      printf("Command: %c\n",command);
      ret = dxl_type.factoryReset(DXL_ID, 0x02, TIMEOUT);

      if(ret) { 
          printf("Reset successfull !!!\n");
          sleep_ms(1000);
          dynamixelSetup(dxl_type, DXL_ID);
      }
      else {
          printf("Reset failed!!!\n");
      }
  }
}
