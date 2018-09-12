/*
Copyright 2015 - 2017 Andreas Chaitidis Andreas.Chaitidis@gmail.com

This program is free software : you can redistribute it and / or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.If not, see <http://www.gnu.org/licenses/>.
*/

#include "VescUart.h"
#include "buffer.h"
#include "crc.h"

static HardwareSerial* serialPort1;
static HardwareSerial* serialPort2;
static HardwareSerial* serialPort3;
static HardwareSerial* serialPort4;
static HardwareSerial* debugSerialPort = NULL;

bool UnpackPayload(uint8_t* message, int lenMes, uint8_t* payload, int lenPa);
bool ProcessReadPacket(uint8_t* message, struct bldcMeasure& values, int len);

void SetSerialPort(HardwareSerial*  _serialPort1, HardwareSerial*  _serialPort2, HardwareSerial*  _serialPort3, HardwareSerial*  _serialPort4)
{
  serialPort1 = _serialPort1;
  serialPort2 = _serialPort2;
  serialPort3 = _serialPort3;
  serialPort4 = _serialPort4;
}

void SetSerialPort(HardwareSerial* _serialPort)
{
  SetSerialPort(_serialPort, _serialPort, _serialPort, _serialPort);
}

void SetDebugSerialPort(HardwareSerial * _debugSerialPort)
{
  debugSerialPort = _debugSerialPort;
}

enum REC_State 
{
  REC_IDLE = 0,
  REC_LEN,
  REC_PAYLOAD,
};

//HardwareSerial *serial; ///@param num as integer with the serial port in use (0=Serial; 1=Serial1; 2=Serial2; 3=Serial3;)
uint16_t ReceiveUartMessageMC(uint8_t* payloadReceived, int num) {

  //Messages <= 255 start with 2. 2nd byte is length
  //Messages >255 start with 3. 2nd and 3rd byte is length combined with 1st >>8 and then &0xFF

  int counter = 0;
  //uint16_t endMessage = 256;
  uint16_t endMessage = 400;
  bool messageRead = false;
  //uint8_t messageReceived[256];
  uint8_t messageReceived[400];
  
  for(int i = 0; i < 400; i++)
    messageReceived[i]=66;
  
  uint16_t lenPayload = 0;
  HardwareSerial* serial = serialPort1;
  
  REC_State state = REC_IDLE;
  bool longMessage = false;
  
  while (!serial->available())
  ;

  bool fertig = false;
  while (!fertig) 
  {
    while (!serial->available())
    ;
    
    uint8_t c = serial->read();
    switch(state)
    {
      case REC_IDLE:
        if(c == 2 || c == 3)
        {
          
          counter = 0;
          messageReceived[counter] = c;
          state = REC_LEN;
          if(c == 3)
          {
            longMessage = true;
          }
          else 
          {
            longMessage = false;
          }
          counter = 1;
        }
        if(c == 3)
        {
          longMessage = true;
          state = REC_LEN;
        }
      break;
      case REC_LEN:
        
        if(!longMessage)
        {
          messageReceived[counter] = c;
          lenPayload = messageReceived[1];
          endMessage = messageReceived[1] + 5; //Payload size + 2 for sice + 3 for SRC and End.
          state = REC_PAYLOAD;
          counter = 2; //++
        }
        else
        {
          messageReceived[counter] = c;
          if(counter == 2)
          {
            lenPayload = messageReceived[1] << 8 | messageReceived[2];
            endMessage = lenPayload + 6; //Payload size + 3 for sice + 3 for SRC and End.
            state = REC_PAYLOAD;
          }
          counter++; //first 2, then 3
        }
      break;
      case REC_PAYLOAD:
        
        messageReceived[counter] = c;
        counter++;
        if(counter >= endMessage || counter >= 400)
        {
          fertig = true;
        }
        
      break;
    }
    
  }  
  
  /*
  debugSerialPort->print("Fertig. len = "); debugSerialPort->print(lenPayload);
  debugSerialPort->print(" endmessage = "); debugSerialPort->print(endMessage);
  debugSerialPort->print(" fertig = "); debugSerialPort->print(fertig);
  debugSerialPort->print(" counter = "); debugSerialPort->println(counter);
  for(int i = 0; i <= endMessage; i++)
  {
    debugSerialPort->print(i);debugSerialPort->print(": "); debugSerialPort->println(String(messageReceived[i]));
  }*/
  
  bool unpacked = false;
  if (fertig) 
  {
    unpacked = UnpackPayload(messageReceived, endMessage, payloadReceived, messageReceived[1]);
  }
  if (unpacked)
  {
    return lenPayload; //Message was read

  }
  else {
    return 0; //No Message Read
  }
}
/* 
uint16_t ReceiveUartMessage(uint8_t* payloadReceived, int num) {

  //Messages <= 255 start with 2. 2nd byte is length
  //Messages >255 start with 3. 2nd and 3rd byte is length combined with 1st >>8 and then &0xFF

  int counter = 0;
  //uint16_t endMessage = 256;
  uint16_t endMessage = 256;
  bool messageRead = false;
  //uint8_t messageReceived[256];
  uint8_t messageReceived[256];
  uint16_t lenPayload = 0;
  HardwareSerial* serial;
  

  switch (num) {
    case 0:
      serial = serialPort1;
      break;
    case 1:
      serial = serialPort2;
      break;
    case 2:
      serial = serialPort3;
      break;
    case 3:
      serial = serialPort4;
      break;
    default:
      break;

  }

  while (serial->available()) {

    uint8_t c = serial->read();
   
    debugSerialPort->print(String(c));
    debugSerialPort->print(", ");
    
    debugSerialPort->print("counter: ");
    debugSerialPort->println(counter);
    
    messageReceived[counter++] = c;
    
    if (counter == 3) 
    {//case if state of 'counter' with last read 1
      switch (messageReceived[0])
      {
      case 2:
        endMessage = messageReceived[1] + 5; //Payload size + 2 for sice + 3 for SRC and End.
        lenPayload = messageReceived[1];
        break;
      case 3:
        debugSerialPort->println("case 3: ");
        endMessage = messageReceived[1] * 256 + messageReceived[2] + 6; //Payload size + 2 for sice + 3 for SRC and End.
        lenPayload = messageReceived[1] * 256 + messageReceived[2];
        debugSerialPort->print(endMessage);
        debugSerialPort->print(" ");
        debugSerialPort->println(lenPayload);
        //ToDo: Add Message Handling > 255 (starting with 3)
        break;
      default:
        debugSerialPort->println("default switch ");
        break;
      }
    }
    if (counter >= sizeof(messageReceived))
    {
      debugSerialPort->println("counter >= sizeof(messageReceived)");
      break;
    }

    if (counter == endMessage && messageReceived[endMessage - 1] == 3) {//+1: Because of counter++ state of 'counter' with last read = "endMessage"
      messageReceived[endMessage] = 0;
      if (debugSerialPort != NULL) {
        debugSerialPort->println("End of message reached!");
      }
      messageRead = true;
      break; //Exit if end of message is reached, even if there is still more data in buffer.
    }
    
  }
  bool unpacked = false;
  if (messageRead) {
        debugSerialPort->println("if (messageRead) {");
    unpacked = UnpackPayload(messageReceived, endMessage, payloadReceived, messageReceived[1]);
  }
  if (unpacked)
  {
        debugSerialPort->println("if (unpacked) {  Message was read");
    return lenPayload; //Message was read

  }
  else {
    debugSerialPort->println("return 0");
    return 0; //No Message Read
  }
} */

bool UnpackPayload(uint8_t* message, int lenMes, uint8_t* payload, int lenPay) {
  uint16_t crcMessage = 0;
  uint16_t crcPayload = 0;
  //Rebuild src:
  crcMessage = message[lenMes - 3] << 8;
  crcMessage &= 0xFF00;
  crcMessage += message[lenMes - 2];
  if(debugSerialPort!=NULL){
    debugSerialPort->print("SRC received: "); debugSerialPort->println(crcMessage);
  } // DEBUG
  
  int lenPayload;
  //Extract payload:
  if(message[0] == 2)
  {
    debugSerialPort->println("if 2");
    lenPayload = message[1];
    memcpy(payload, &message[2], lenPayload);
  }
  else if(message[0] == 3)
  {
    debugSerialPort->println("if 3");
    lenPayload = message[1] << 8 | message[2];
    
    memcpy(payload, &message[3], lenPayload);
  }
  else 
    debugSerialPort->print("ERROR");
  
  crcPayload = crc16(payload, lenPayload);

  
  if(debugSerialPort!=NULL){
    debugSerialPort->print("SRC calc: "); debugSerialPort->println(crcPayload);
  }
  if (crcPayload == crcMessage)
  {
    if(debugSerialPort!=NULL){
        debugSerialPort->print("Received: "); SerialPrint(message, lenMes); debugSerialPort->println();
        debugSerialPort->print("Payload :      "); SerialPrint(payload, lenPayload - 1); debugSerialPort->println();
    } // DEBUG

    return true;
  }
  else
  {
    return false;
  }
}




int PackSendPayload(uint8_t* payload, int lenPay, int num) {
  uint16_t crcPayload = crc16(payload, lenPay);
  int count = 0;
  uint8_t messageSend[256];

  if (lenPay <= 256)
  {
    messageSend[count++] = 2;
    messageSend[count++] = lenPay;
  }
  else
  {
    messageSend[count++] = 3;
    messageSend[count++] = (uint8_t)(lenPay >> 8);
    messageSend[count++] = (uint8_t)(lenPay & 0xFF);
  }
  memcpy(&messageSend[count], payload, lenPay);

  count += lenPay;
  messageSend[count++] = (uint8_t)(crcPayload >> 8);
  messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
  messageSend[count++] = 3;
  messageSend[count] = NULL;

if(debugSerialPort!=NULL){
  debugSerialPort->print("UART package send: "); SerialPrint(messageSend, count);

} // DEBUG


  HardwareSerial *serial;

  switch (num) {
    case 0:
      serial=serialPort1;
      break;
    case 1:
      serial=serialPort2;
      break;
    case 2:
      serial=serialPort3;
      break;
    case 3:
      serial=serialPort4;
      break;
    default:
      break;
  }

  //Sending package
  serial->write(messageSend, count);


  //Returns number of send bytes
  return count;
}




bool ProcessReadPacket(uint8_t* message, struct bldcMeasure& values, int len) {
  COMM_PACKET_ID packetId;
  int32_t ind = 0;

  packetId = (COMM_PACKET_ID)message[0];
  message++;//Eliminates the message id
  len--;

  switch (packetId)
  {

  case COMM_GET_VALUES:
    values.tempFetFiltered    = buffer_get_float16(message, 1e1, &ind);
    values.tempMotorFiltered  = buffer_get_float16(message, 1e1, &ind);
    values.avgMotorCurrent    = buffer_get_float32(message, 100.0, &ind);
    values.avgInputCurrent    = buffer_get_float32(message, 100.0, &ind);
    values.avgId              = buffer_get_float32(message, 1e2, &ind);
    values.avgIq              = buffer_get_float32(message, 1e2, &ind);
    values.dutyNow            = buffer_get_float16(message, 1000.0, &ind);
    values.rpm                = buffer_get_float32(message, 1.0, &ind);
    values.inpVoltage         = buffer_get_float16(message, 10.0, &ind);
    values.ampHours           = buffer_get_float32(message, 10000.0, &ind);
    values.ampHoursCharged    = buffer_get_float32(message, 10000.0, &ind);
    values.wattHours          = buffer_get_float32(message, 1e4, &ind);
    values.watthoursCharged   = buffer_get_float32(message, 1e4, &ind);
    values.tachometer         = buffer_get_int32(message, &ind);
    values.tachometerAbs      = buffer_get_int32(message, &ind);
    values.faultCode          = message[ind];
    return true;
    break;
  default:
    return false;
    break;
  }
}

/* bool ProcessReadPacketMC(uint8_t* data, mc_configuration& mcconf, int len) {
  COMM_PACKET_ID packetId;
  int32_t ind = 0;

  packetId = (COMM_PACKET_ID)data[0];
  data++;//Eliminates the message id
  len--;

  switch (packetId)
  {

  case COMM_GET_VALUES:
    ind = 0;
		mcconf.pwm_mode = (mc_pwm_mode)data[ind++];
		mcconf.comm_mode = (mc_comm_mode)data[ind++];
		mcconf.motor_type = (mc_motor_type)data[ind++];
		mcconf.sensor_mode = (mc_sensor_mode)data[ind++];

		mcconf.l_current_max = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_current_min = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_in_current_max = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_in_current_min = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_abs_current_max = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_min_erpm = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_max_erpm = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_max_erpm_fbrake = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_max_erpm_fbrake_cc = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_min_vin = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_max_vin = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_battery_cut_start = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_battery_cut_end = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_slow_abs_current = data[ind++];
		//                             mcconf.l_rpm_lim_neg_torque = data[ind++];
		mcconf.l_temp_fet_start = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_temp_fet_end = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_temp_motor_start = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_temp_motor_end = buffer_get_float32(data, 1000.0, &ind);
		mcconf.l_min_duty = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.l_max_duty = buffer_get_float32(data, 1000000.0, &ind);

		mcconf.sl_min_erpm = buffer_get_float32(data, 1000.0, &ind);
		mcconf.sl_min_erpm_cycle_int_limit = buffer_get_float32(data, 1000.0, &ind);
		mcconf.sl_max_fullbreak_current_dir_change = buffer_get_float32(data, 1000.0, &ind);
		mcconf.sl_cycle_int_limit = buffer_get_float32(data, 1000.0, &ind);
		mcconf.sl_phase_advance_at_br = buffer_get_float32(data, 1000.0, &ind);
		mcconf.sl_cycle_int_rpm_br = buffer_get_float32(data, 1000.0, &ind);
		mcconf.sl_bemf_coupling_k = buffer_get_float32(data, 1000.0, &ind);

		memcpy(mcconf.hall_table, data + ind, 8);
		ind += 8;
		mcconf.hall_sl_erpm = buffer_get_float32(data, 1000.0, &ind);

		mcconf.foc_current_kp = buffer_get_float32(data, 1e5, &ind);
		mcconf.foc_current_ki = buffer_get_float32(data, 1e5, &ind);
		mcconf.foc_f_sw = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_dt_us = buffer_get_float32(data, 1e6, &ind);
		mcconf.foc_encoder_inverted = data[ind++];
		mcconf.foc_encoder_offset = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_encoder_ratio = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_sensor_mode = (mc_foc_sensor_mode)data[ind++];
		mcconf.foc_pll_kp = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_pll_ki = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_motor_l = buffer_get_float32(data, 1e8, &ind);
		mcconf.foc_motor_r = buffer_get_float32(data, 1e5, &ind);
		mcconf.foc_motor_flux_linkage = buffer_get_float32(data, 1e5, &ind);
		mcconf.foc_observer_gain = buffer_get_float32(data, 1e0, &ind);
		mcconf.foc_duty_dowmramp_kp = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_duty_dowmramp_ki = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_openloop_rpm = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_sl_openloop_hyst = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_sl_openloop_time = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_sl_d_current_duty = buffer_get_float32(data, 1e3, &ind);
		mcconf.foc_sl_d_current_factor = buffer_get_float32(data, 1e3, &ind);
		memcpy(mcconf.foc_hall_table, data + ind, 8);
		ind += 8;
		//                                         mcconf.foc_hall_sl_erpm = buffer_get_float32(data, 1000.0, &ind);

		mcconf.s_pid_kp = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.s_pid_ki = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.s_pid_kd = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.s_pid_min_erpm = buffer_get_float32(data, 1000.0, &ind);

		mcconf.p_pid_kp = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.p_pid_ki = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.p_pid_kd = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.p_pid_ang_div = buffer_get_float32(data, 1e5, &ind);

		mcconf.cc_startup_boost_duty = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.cc_min_current = buffer_get_float32(data, 1000.0, &ind);
		mcconf.cc_gain = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.cc_ramp_step_max = buffer_get_float32(data, 1000000.0, &ind);

		mcconf.m_fault_stop_time_ms = buffer_get_int32(data, &ind);
		mcconf.m_duty_ramp_step = buffer_get_float32(data, 1000000.0, &ind);
		//  mcconf.m_duty_ramp_step_rpm_lim = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.m_current_backoff_gain = buffer_get_float32(data, 1000000.0, &ind);
		mcconf.m_encoder_counts = buffer_get_uint32(data, &ind);
    return true;
    break;
  default:
    return false;
    break;
  }
} */

/* bool VescUartGetValue(bldcMeasure& values, int num) {
  uint8_t command[1] = { COMM_GET_VALUES };
  uint8_t payload[256];
  PackSendPayload(command, 1, num);
  delay(10); //needed, otherwise data is not read
  int lenPayload = ReceiveUartMessage(payload, num);
  if (lenPayload > 1) {
    bool read = ProcessReadPacket(payload, values, lenPayload); //returns true if sucessful
    return read;
  }
  else
  {
    return false;
  }
}
bool VescUartGetValue(bldcMeasure& values) {
  return VescUartGetValue(values, 0);
} */

bool VescUartGetMC(bldcMeasure& values, int num) {
  uint8_t command[1] = { COMM_GET_MCCONF };  //COMM_GET_MCCONF  COMM_GET_VALUES
  uint8_t payload[350];
  PackSendPayload(command, 1, num);
  //delay(10); //needed, otherwise data is not read
  int lenPayload = ReceiveUartMessageMC(payload, num);/* MC */
  if (lenPayload > 1) {
    bool read = ProcessReadPacket(payload, values, lenPayload); //returns true if sucessful
    return read;
  }
  else
  {
    return false;
  }
}
bool VescUartGetMC(bldcMeasure& values) {
  return VescUartGetMC(values, 0);
}

void VescUartSetCurrent(float current, int num) {
  int32_t index = 0;
  uint8_t payload[5];

  payload[index++] = COMM_SET_CURRENT ;
  buffer_append_int32(payload, (int32_t)(current * 1000), &index);
  PackSendPayload(payload, 5, num);
}
void VescUartSetCurrent(float current){
  VescUartSetCurrent(current, 0);
}

void VescUartSetPosition(float position, int num) {
  int32_t index = 0;
  uint8_t payload[5];

  payload[index++] = COMM_SET_POS ;
  buffer_append_int32(payload, (int32_t)(position * 1000000.0), &index);
  PackSendPayload(payload, 5, num);
}
void VescUartSetPosition(float position) {
  VescUartSetPosition(position, 0);
}

void VescUartSetDuty(float duty, int num) {
  int32_t index = 0;
  uint8_t payload[5];

  payload[index++] = COMM_SET_DUTY ;
  buffer_append_int32(payload, (int32_t)(duty * 100000), &index);
  PackSendPayload(payload, 5, num);
}
void VescUartSetDuty(float duty) {
  VescUartSetDuty(duty, 0);
}


void VescUartSetRPM(float rpm, int num) {
  int32_t index = 0;
  uint8_t payload[5];

  payload[index++] = COMM_SET_RPM ;
  buffer_append_int32(payload, (int32_t)(rpm), &index);
  PackSendPayload(payload, 5, num);
}
void VescUartSetRPM(float rpm) {
  VescUartSetRPM(rpm, 0);
}

void VescUartSetCurrentBrake(float brakeCurrent, int num) {
  int32_t index = 0;
  uint8_t payload[5];

  payload[index++] = COMM_SET_CURRENT_BRAKE;
  buffer_append_int32(payload, (int32_t)(brakeCurrent * 1000), &index);
  PackSendPayload(payload, 5, num);
}
void VescUartSetCurrentBrake(float brakeCurrent) {
  VescUartSetCurrentBrake(brakeCurrent, 0);
}


void VescUartSetNunchukValues(remotePackage& data, int num) {
  int32_t ind = 0;
  uint8_t payload[11];
  payload[ind++] = COMM_SET_CHUCK_DATA;
  payload[ind++] = data.valXJoy;
  payload[ind++] = data.valYJoy;
  buffer_append_bool(payload, data.valLowerButton, &ind);
  buffer_append_bool(payload, data.valUpperButton, &ind);
  //Acceleration Data. Not used, Int16 (2 byte)
  payload[ind++] = 0;
  payload[ind++] = 0;
  payload[ind++] = 0;
  payload[ind++] = 0;
  payload[ind++] = 0;
  payload[ind++] = 0;

if(debugSerialPort!=NULL){
  debugSerialPort->println("Data reached at VescUartSetNunchuckValues:");
  debugSerialPort->print("valXJoy = "); debugSerialPort->print(data.valXJoy); debugSerialPort->print(" valYJoy = "); debugSerialPort->println(data.valYJoy);
  debugSerialPort->print("LowerButton = "); debugSerialPort->print(data.valLowerButton); debugSerialPort->print(" UpperButton = "); debugSerialPort->println(data.valUpperButton);
}

  PackSendPayload(payload, 11, num);
}
void VescUartSetNunchukValues(remotePackage& data) {
  VescUartSetNunchukValues(data, 0);
}


void SerialPrint(uint8_t* data, int len) {

  //  debugSerialPort->print("Data to display: "); debugSerialPort->println(sizeof(data));

  for (int i = 0; i <= len; i++)
  {
    debugSerialPort->print(data[i]);
    debugSerialPort->print(" ");
  }
  debugSerialPort->println("");
}


void SerialPrint(const struct bldcMeasure& values) {
  debugSerialPort->print("tempFetFiltered:  "); debugSerialPort->println(values.tempFetFiltered);
  debugSerialPort->print("tempMotorFiltered:"); debugSerialPort->println(values.tempMotorFiltered);
  debugSerialPort->print("avgMotorCurrent:  "); debugSerialPort->println(values.avgMotorCurrent);
  debugSerialPort->print("avgInputCurrent:  "); debugSerialPort->println(values.avgInputCurrent);
  debugSerialPort->print("avgId:      "); debugSerialPort->println(values.avgId);
  debugSerialPort->print("avgIq:      "); debugSerialPort->println(values.avgIq);
  debugSerialPort->print("dutyNow:      "); debugSerialPort->println(values.dutyNow);
  debugSerialPort->print("rpm:        "); debugSerialPort->println(values.rpm);
  debugSerialPort->print("inpVoltage:    "); debugSerialPort->println(values.inpVoltage);
  debugSerialPort->print("ampHours:    "); debugSerialPort->println(values.ampHours);
  debugSerialPort->print("ampHoursCharged:  "); debugSerialPort->println(values.ampHoursCharged);
  debugSerialPort->print("tachometer:    "); debugSerialPort->println(values.tachometer);
  debugSerialPort->print("tachometerAbs:  "); debugSerialPort->println(values.tachometerAbs);
  debugSerialPort->print("faultCode:    "); debugSerialPort->println(values.faultCode);
}


/* void SerialPrint(const mc_configuration& config) {
  debugSerialPort->print("l_current_max:  "); debugSerialPort->println(config.l_current_max);
  debugSerialPort->print("l_current_min:"); debugSerialPort->println(config.l_current_min);
  debugSerialPort->print("l_in_current_max:  "); debugSerialPort->println(config.l_in_current_max);
  debugSerialPort->print("l_in_current_min:  "); debugSerialPort->println(config.l_in_current_min);
  debugSerialPort->print("l_min_erpm:      "); debugSerialPort->println(config.l_min_erpm);
  debugSerialPort->print("l_max_erpm:      "); debugSerialPort->println(config.l_max_erpm);
  debugSerialPort->print("l_max_erpm_fbrake:      "); debugSerialPort->println(config.l_max_erpm_fbrake);
  debugSerialPort->print("l_max_erpm_fbrake_cc:        "); debugSerialPort->println(config.l_max_erpm_fbrake_cc);
  debugSerialPort->print("l_min_vin:    "); debugSerialPort->println(config.l_min_vin);
  debugSerialPort->print("l_max_vin:    "); debugSerialPort->println(config.l_max_vin);
  debugSerialPort->print("sl_min_erpm:  "); debugSerialPort->println(config.sl_min_erpm);
  debugSerialPort->print("sl_min_erpm_cycle_int_limit:    "); debugSerialPort->println(config.sl_min_erpm_cycle_int_limit);
  //debugSerialPort->print("tachometerAbs:  "); debugSerialPort->println(config.tachometerAbs);
  //debugSerialPort->print("faultCode:    "); debugSerialPort->println(config.faultCode);
} */
