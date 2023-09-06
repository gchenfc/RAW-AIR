// Copyright 2022 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <numeric>
#include <Dynamixel2Arduino.h>

#define DXL_BUS Serial1
#define USB Serial
#define BT Serial2
#define DXL_PACKET_BUFFER_LENGTH 1024
enum Interface {
  USB_INTERFACE = 0,
  BT_INTERFACE = 1,
  INTERFACE_COUNT,  // must be last, to count how many enums there are
};

// Pin definitions
#define ADC_BATTERY (33u)
#define PIN_LED (32u)
#define LED_BUILTIN PIN_LED
#define BDPIN_DXL_PWR_EN (31u)

uint8_t packet_buffer[DXL_PACKET_BUFFER_LENGTH];
unsigned long led_update_time = 0;
constexpr long FLASH_DURATION = 50;  // ms

// Code for acting as a slave device, to turn servos on/off.
constexpr uint8_t MY_SLAVE_ID = 0x55;
constexpr uint8_t PACKET_FOR_ME_SLAVE[] = {
    0xff,         // header1
    0xff,         // header2
    MY_SLAVE_ID,  // id
    3,            // length, instruction+param+checksum
    0x03,         // instruction: write
                  // Param should be 0 for off, 1 for on.
                  // Checksum
};

uint8_t checksum(uint8_t* packet, uint8_t len) {
  uint8_t sum = 0;
  for (uint8_t i = 2; i < len - 1; i++) {
    sum += packet[i];
  }
  return ~sum;
}
uint8_t slave_checksum(uint8_t param) {
  // return ~(PACKET_FOR_ME_SLAVE[2] + 3 + 0x03 + param);
  return ~std::accumulate(PACKET_FOR_ME_SLAVE + 2,
                          PACKET_FOR_ME_SLAVE + sizeof(PACKET_FOR_ME_SLAVE),
                          param);
}

void check_byte_for_slave_msg(Interface interface, uint8_t datum) {
  static uint8_t indexes_along_packet[2] = {0};
  static uint8_t actions[2] = {0};
  uint8_t& index = indexes_along_packet[interface];
  uint8_t& action = actions[interface];
  // special cases
  if (index == sizeof(PACKET_FOR_ME_SLAVE) / sizeof(uint8_t)) {  // param
    action = datum;
    ++index;
  } else if (index == 1 + sizeof(PACKET_FOR_ME_SLAVE) / sizeof(uint8_t)) {
    if (slave_checksum(action) == datum) {
      // do the action
      digitalWrite(BDPIN_DXL_PWR_EN, action != 0);
    }
    index = 0;  // finished packet.  Reset
  } else if (PACKET_FOR_ME_SLAVE[index] == datum) {
    ++index;
  } else if (index == 2 && datum == PACKET_FOR_ME_SLAVE[1]) {
    // We can prefix with as many 0xff's as we want.  Do nothing.
  } else {
    index = 0;
  }
}
// End of slave code.

Dynamixel2Arduino dxl(DXL_BUS);

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  // Use UART port of DYNAMIXEL Shield to debug.
  USB.begin(57600);
  // BT.begin(9600);
  BT.begin(1000000);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  // dxl.begin(USB.baud());
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol
  // version.
  dxl.begin(1000000);
}

void loop() {
  // put your main code here, to run repeatedly:
  dataTransceiver();

  // if(USB.baud() != dxl.getPortBaud()) {
  //   dxl.begin(USB.baud());
  // }
}

void dataTransceiver() {
  int length = 0;
  int i = 0;

  // USB -> DXL
  length = USB.available();
  if (length > 0) {
    for (i = 0; i < length; i++) {
      uint8_t b = USB.read();
      DXL_BUS.write(b);
      check_byte_for_slave_msg(USB_INTERFACE, b);
    }
    ledStatus();
  }

  // BT -> DXL
  length = BT.available();
  if (length > 0) {
    for (i = 0; i < length; i++) {
      uint8_t b = BT.read();
      DXL_BUS.write(b);
      check_byte_for_slave_msg(BT_INTERFACE, b);
    }
    ledStatus();
  }

  // DXL -> USB & BT
  length = DXL_BUS.available();
  if (length > 0) {
    if (length > DXL_PACKET_BUFFER_LENGTH) {
      length = DXL_PACKET_BUFFER_LENGTH;
    }
    for (i = 0; i < length; i++) {
      packet_buffer[i] = DXL_BUS.read();
    }
    USB.write(packet_buffer, length);
    BT.write(packet_buffer, length);
    ledStatus();
  }

  // Logic is: For the first FLASH_DURATION/2 ms after ledStatus is first
  // called, turn the LED on.  Then, turn it off for the next FLASH_DURATION/2
  // ms.  If ledStatus keeps getting called after FLASH_DURATION, then reset
  // the led_update_time to now so that the LED will be on again for
  // FLASH_DURATION/2.
  digitalWrite(LED_BUILTIN,
               (millis() - led_update_time) < (FLASH_DURATION / 2));
}

void ledStatus() {
  if ((millis() - led_update_time) > FLASH_DURATION) {
    led_update_time = millis();
  }
}
