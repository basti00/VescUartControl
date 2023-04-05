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
#include "mempools.h"
#include "packet.h"

static HardwareSerial* serialPort1;
static HardwareSerial* serialPort2;
static HardwareSerial* serialPort3;
static HardwareSerial* serialPort4;
static HardwareSerial* debugSerialPort = NULL;

uint8_t payloadGlobal[PACKET_MAX_PL_LEN];
uint8_t messageGlobal[500]; // 488 MESSAGE Length
static uint8_t send_buffer[PACKET_MAX_PL_LEN];

bool UnpackPayload(uint8_t* message, int lenMes, uint8_t* payload, int lenPa);
bool ProcessReadPacket(uint8_t* message, bldcMeasure& values, int len);

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

void SetDebugSerialPort(HardwareSerial * _Serial)
{
  debugSerialPort = _Serial;
}

enum REC_State 
{
  REC_IDLE = 0,
  REC_LEN,
  REC_PAYLOAD,
};

//https://vscode.dev/github/basti30/VescUartControl/blob/fdeb5bfca8f9c10acc51ff8ae5fef26bb2c129c2/packet.c#L155-L156
uint16_t ReceiveUartMessage2(uint8_t *payloadReceived, int num)
{

  //Messages <= 255 start with 2. 2nd byte is length
  //Messages >255 start with 3. 2nd and 3rd byte is length combined with 1st >>8 and then &0xFF

  int counter = 0;
  //uint16_t endMessage = 256;
  uint16_t endMessage = PACKET_MAX_PL_LEN;
  bool messageRead = false;

  //for(int i = 0; i < 420; i++)
  //  messageReceived[i]=0;

  uint16_t lenPayload = 0;
  HardwareSerial *serial = serialPort1;

  REC_State state = REC_IDLE;
  bool longMessage = false;

  int timeout = 0;
  int pack_length =0;

  do{
    pack_length=serial->available();
    if(pack_length) break;
    timeout += 10;
    delay(10);
    if (timeout > 10000)
    {
      Serial.println("Timeout, cant communicate with vesc");
      return 0; //Timeout, cant communicate with vesc.
    }
  }while(!pack_length);
  timeout = 0;

  Serial.print("Packet length connect ");
  Serial.println(pack_length);
  pack_length=0;


  bool done = false;
  while (!done)
  {
    delay(0);
    do{
      pack_length=serial->available();
      if(pack_length) break;
        timeout += 10;
        delay(10);
        if (timeout > 1000)
        {
          Serial.println("Timeout, cant communicate with vesc");
          return 0; //Timeout, cant communicate with vesc.
        }
    }while(!pack_length);

    timeout = 0;
    //Serial.print(" Packet length ");
    //Serial.println(pack_length);

    uint8_t c = serial->read();
    //Serial.print("read ");
    //Serial.println(c);
    switch (state)
    {
    case REC_IDLE:
      //Serial.print("Receive IDLE ");
      if (c == 2 || c == 3)
      {

        counter = 0;
        messageGlobal[counter] = c;
        state = REC_LEN;
        if (c == 3)
        {
          longMessage = true;
        }
        else
        {
          longMessage = false;
        }
        counter = 1;
      }
      if (c == 3)
      {
        longMessage = true;
        state = REC_LEN;
      }
      break;
    case REC_LEN:
    //Serial.print("Receive LEN ");

      if (!longMessage)
      {
        messageGlobal[counter] = c;
        lenPayload = messageGlobal[1];
        //Serial.print("len not long ");
        //Serial.println(lenPayload);
        endMessage = messageGlobal[1] + 5; //Payload size + 2 for sice + 3 for SRC and End.
        state = REC_PAYLOAD;
        counter = 2; //++
      }
      else if (longMessage && messageGlobal[0] == 3 )
      {
        messageGlobal[counter] = c;
        if (counter == 2)
        {
          lenPayload = messageGlobal[1] << 8 | messageGlobal[2];
          endMessage = lenPayload + 6; //Payload size + 3 for sice + 3 for SRC and End.
          Serial.print("len LONG ");
         Serial.println(lenPayload);
          state = REC_PAYLOAD;
        }
        counter++; //first 2, then 3
      }
      else{
        Serial.print("t LONG ");
        lenPayload =  (unsigned int)messageGlobal[1] << 16 | (unsigned int)messageGlobal[2] << 8 | (unsigned int)messageGlobal[3];
        state = REC_PAYLOAD;
      }
      break;
    case REC_PAYLOAD:

      messageGlobal[counter] = c;
      counter++;
      if (counter >= endMessage || counter >= PACKET_MAX_PL_LEN)
      {
        done = true;
      }
      break;
    }
  }


   bool unpacked = false;
  if (done) 
  {
    unpacked = UnpackPayload(messageGlobal, endMessage, payloadReceived, messageGlobal[1]);
  }
  if (unpacked)
  {
    //return lenPayload; //Message was read
    return counter;
  }
  else {
    return 0; //No Message Read
  }

}


//HardwareSerial *serial; ///@param num as integer with the serial port in use (0=Serial; 1=Serial1; 2=Serial2; 3=Serial3;)
uint16_t ReceiveUartMessage(uint8_t* payloadReceived, int num) {

  //Messages <= 255 start with 2. 2nd byte is length
  //Messages >255 start with 3. 2nd and 3rd byte is length combined with 1st >>8 and then &0xFF

  int counter = 0;
  //uint16_t endMessage = 256;
  uint16_t endMessage = 400;
  bool messageRead = false;
  
  //for(int i = 0; i < 400; i++)
  //  messageReceived[i]=0;
  
  uint16_t lenPayload = 0;
  HardwareSerial* serial = serialPort1;
  
  REC_State state = REC_IDLE;
  bool longMessage = false;
  
  int timeout=0;
  while (!serial->available())
  {
    timeout += 10;
    delay(10);
    if(timeout > 1000)
    {
      Serial.println("Timeout, cant communicate with vesc");
      return 0; //Timeout, cant communicate with vesc.
    }
  }
  timeout=0;

  bool fertig = false;
  while (!fertig) 
  {
    delay(0);
    while (!serial->available())
    {
      timeout += 10;
      delay(10);
      if(timeout > 1000)
      {
        Serial.println("Timeout, cant communicate with vesc");
        return 0; //Timeout, cant communicate with vesc.
      }
    }
    timeout=0;
    
    uint8_t c = serial->read();
    switch(state)
    {
      case REC_IDLE:
        if(c == 2 || c == 3)
        {
          
          counter = 0;
          messageGlobal[counter] = c;
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
          messageGlobal[counter] = c;
          lenPayload = messageGlobal[1];
          endMessage = messageGlobal[1] + 5; //Payload size + 2 for sice + 3 for SRC and End.
          state = REC_PAYLOAD;
          counter = 2; //++
        }
        else
        {
          messageGlobal[counter] = c;
          if(counter == 2)
          {
            lenPayload = messageGlobal[1] << 8 | messageGlobal[2];
            endMessage = lenPayload + 6; //Payload size + 3 for sice + 3 for SRC and End.
            state = REC_PAYLOAD;
          }
          counter++; //first 2, then 3
        }
      break;
      case REC_PAYLOAD:
        
        messageGlobal[counter] = c;
        counter++;
        if(counter >= endMessage || counter >= 500)
        {
          fertig = true;
        }   
      break;
    }



  }  
  
  
  bool unpacked = false;
  if (fertig) 
  {
    unpacked = UnpackPayload(messageGlobal, endMessage, payloadReceived, messageGlobal[1]);
  }
  if (unpacked)
  {
    //return lenPayload; //Message was read
    return counter;
  }
  else {
    return 0; //No Message Read
  }
}



bool UnpackPayload(uint8_t* message, int lenMes, uint8_t* payload, int lenPay)  //FOR ALL MSG
{
  uint16_t crcMessage = 0;
  uint16_t crcPayload = 0;
  //Rebuild src:
  crcMessage = message[lenMes - 3] << 8;
  crcMessage &= 0xFF00;
  crcMessage += message[lenMes - 2];
  //if(debugSerialPort!=NULL){
  //  debugSerialPort->print("SRC received: "); debugSerialPort->println(crcMessage);
  //} // DEBUG
  
  int lenPayload;
  //Extract payload:
  if(message[0] == 2)
  {
    lenPayload = message[1];
    memcpy(payload, &message[2], lenPayload);
  }
  else if(message[0] == 3)
  {
    lenPayload = message[1] << 8 | message[2];
    
    memcpy(payload, &message[3], lenPayload);
  }
  else 
    if(debugSerialPort!=NULL)debugSerialPort->print("ERROR: wrong start byte");
  
  crcPayload = crc16(payload, lenPayload);

  
  //if(debugSerialPort!=NULL){
  //  debugSerialPort->print("SRC calc: "); debugSerialPort->println(crcPayload);
  //}
  if (crcPayload == crcMessage)
  {
    if(debugSerialPort!=NULL){
        //debugSerialPort->print("Received: "); SerialPrint(message, lenMes); debugSerialPort->println();
        //debugSerialPort->print("Payload :      "); SerialPrint(payload, lenPayload - 1); debugSerialPort->println();
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
  //uint8_t messageSend[400]; glob

  if (lenPay <= 256)
  {
    messageGlobal[count++] = 2;
    messageGlobal[count++] = lenPay;
  }
  else
  {
    messageGlobal[count++] = 3;
    messageGlobal[count++] = (uint8_t)(lenPay >> 8);
    messageGlobal[count++] = (uint8_t)(lenPay & 0xFF);
  }
  memcpy(&messageGlobal[count], payload, lenPay);

  count += lenPay;
  messageGlobal[count++] = (uint8_t)(crcPayload >> 8);
  messageGlobal[count++] = (uint8_t)(crcPayload & 0xFF);
  messageGlobal[count++] = 3;
  messageGlobal[count] = 0;

//if(debugSerialPort!=NULL){
//  debugSerialPort->print("UART package send: "); SerialPrint(messageGlobal, count);
 //SerialPrint0(messageGlobal, count);
//} // DEBUG


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
  
  serial->write(messageGlobal, count);


  //Returns number of send bytes
  return count;
}



//https://vscode.dev/github/basti30/VescUartControl/blob/fdeb5bfca8f9c10acc51ff8ae5fef26bb2c129c2/packet.c#L41
int packet_send(uint8_t* payload, int lenPay, int num) {


  if (lenPay == 0 || lenPay > PACKET_MAX_PL_LEN) {
		return 0;
	}

  uint16_t crcPayload = crc16(payload, lenPay);
  int count = 0;

  uint8_t messageSend[PACKET_MAX_PL_LEN]; //glob

  if (lenPay <= 256)
  {
    messageSend[count++] = 2;
    messageSend[count++] = lenPay;
  }
  else if ( lenPay <= 65535)
  {
    messageSend[count++] = 3;
    messageSend[count++] = (uint8_t)(lenPay >> 8);
    messageSend[count++] = (uint8_t)(lenPay & 0xFF);
  }
  else{ 
    messageSend[count++] = 4;
		messageSend[count++] = lenPay >> 16;
		messageSend[count++] = (lenPay >> 8) & 0xFF;
		messageSend[count++] = lenPay & 0xFF;

  }
  memcpy(&messageSend[count], payload, lenPay);

  count += lenPay;
  messageSend[count++] = (uint8_t)(crcPayload >> 8);
  messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
  messageSend[count++] = 3;
  messageSend[count] = 0;



 SerialPrint0(messageSend, count); // DEBUG


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




//https://github.com/vedderb/bldc/blob/fea2c81e55ab68b8476d6b1514595306691f3375/confgenerator.c#L8
int BuildPacket(uint8_t* buffer, mc_configuration& conf)
{
  int32_t ind = 0;

  //buffer[ind++] = COMM_SET_MCCONF; 

	buffer_append_uint32(buffer, MCCONF_SIGNATURE, &ind);

	buffer[ind++] = conf.pwm_mode;
	buffer[ind++] = conf.comm_mode;
	buffer[ind++] = conf.motor_type;
	buffer[ind++] = conf.sensor_mode;
	buffer_append_float32_auto(buffer, conf.l_current_max, &ind);
	buffer_append_float32_auto(buffer, conf.l_current_min, &ind);
	buffer_append_float32_auto(buffer, conf.l_in_current_max, &ind);
	buffer_append_float32_auto(buffer, conf.l_in_current_min, &ind);
	buffer_append_float32_auto(buffer, conf.l_abs_current_max, &ind);
	buffer_append_float32_auto(buffer, conf.l_min_erpm, &ind);
	buffer_append_float32_auto(buffer, conf.l_max_erpm, &ind);
	buffer_append_float16(buffer, conf.l_erpm_start, 10000, &ind);
	buffer_append_float32_auto(buffer, conf.l_max_erpm_fbrake, &ind);
	buffer_append_float32_auto(buffer, conf.l_max_erpm_fbrake_cc, &ind);
	buffer_append_float16(buffer, conf.l_min_vin, 10, &ind);
	buffer_append_float16(buffer, conf.l_max_vin, 10, &ind);
	buffer_append_float16(buffer, conf.l_battery_cut_start, 10, &ind);
	buffer_append_float16(buffer, conf.l_battery_cut_end, 10, &ind);
	buffer_append_float16(buffer, conf.l_battery_regen_cut_start, 10, &ind);
	buffer_append_float16(buffer, conf.l_battery_regen_cut_end, 10, &ind);
	buffer[ind++] = conf.l_slow_abs_current;
	buffer_append_float16(buffer, conf.l_temp_fet_start, 10, &ind);
	buffer_append_float16(buffer, conf.l_temp_fet_end, 10, &ind);
	buffer_append_float16(buffer, conf.l_temp_motor_start, 10, &ind);
	buffer_append_float16(buffer, conf.l_temp_motor_end, 10, &ind);
	buffer_append_float16(buffer, conf.l_temp_accel_dec, 10000, &ind);
	buffer_append_float16(buffer, conf.l_min_duty, 10000, &ind);
	buffer_append_float16(buffer, conf.l_max_duty, 10000, &ind);
	buffer_append_float32_auto(buffer, conf.l_watt_max, &ind);
	buffer_append_float32_auto(buffer, conf.l_watt_min, &ind);
	buffer_append_float16(buffer, conf.l_current_max_scale, 10000, &ind);
	buffer_append_float16(buffer, conf.l_current_min_scale, 10000, &ind);
	buffer_append_float16(buffer, conf.l_duty_start, 10000, &ind);
	buffer_append_float32_auto(buffer, conf.sl_min_erpm, &ind);
	buffer_append_float32_auto(buffer, conf.sl_min_erpm_cycle_int_limit, &ind);
	buffer_append_float32_auto(buffer, conf.sl_max_fullbreak_current_dir_change, &ind);
	buffer_append_float16(buffer, conf.sl_cycle_int_limit, 10, &ind);
	buffer_append_float16(buffer, conf.sl_phase_advance_at_br, 10000, &ind);
	buffer_append_float32_auto(buffer, conf.sl_cycle_int_rpm_br, &ind);
	buffer_append_float32_auto(buffer, conf.sl_bemf_coupling_k, &ind);
	buffer[ind++] = (uint8_t)conf.hall_table[0];
	buffer[ind++] = (uint8_t)conf.hall_table[1];
	buffer[ind++] = (uint8_t)conf.hall_table[2];
	buffer[ind++] = (uint8_t)conf.hall_table[3];
	buffer[ind++] = (uint8_t)conf.hall_table[4];
	buffer[ind++] = (uint8_t)conf.hall_table[5];
	buffer[ind++] = (uint8_t)conf.hall_table[6];
	buffer[ind++] = (uint8_t)conf.hall_table[7];
	buffer_append_float32_auto(buffer, conf.hall_sl_erpm, &ind);
	buffer_append_float32_auto(buffer, conf.foc_current_kp, &ind);
	buffer_append_float32_auto(buffer, conf.foc_current_ki, &ind);
	buffer_append_float32_auto(buffer, conf.foc_f_zv, &ind);
	buffer_append_float32_auto(buffer, conf.foc_dt_us, &ind);
	buffer[ind++] = conf.foc_encoder_inverted;
	buffer_append_float32_auto(buffer, conf.foc_encoder_offset, &ind);
	buffer_append_float32_auto(buffer, conf.foc_encoder_ratio, &ind);
	buffer[ind++] = conf.foc_sensor_mode;
	buffer_append_float32_auto(buffer, conf.foc_pll_kp, &ind);
	buffer_append_float32_auto(buffer, conf.foc_pll_ki, &ind);
	buffer_append_float32_auto(buffer, conf.foc_motor_l, &ind);
	buffer_append_float32_auto(buffer, conf.foc_motor_ld_lq_diff, &ind);
	buffer_append_float32_auto(buffer, conf.foc_motor_r, &ind);
	buffer_append_float32_auto(buffer, conf.foc_motor_flux_linkage, &ind);
	buffer_append_float32_auto(buffer, conf.foc_observer_gain, &ind);
	buffer_append_float32_auto(buffer, conf.foc_observer_gain_slow, &ind);
	buffer_append_float16(buffer, conf.foc_observer_offset, 1000, &ind);
	buffer_append_float32_auto(buffer, conf.foc_duty_dowmramp_kp, &ind);
	buffer_append_float32_auto(buffer, conf.foc_duty_dowmramp_ki, &ind);
	buffer_append_float16(buffer, conf.foc_start_curr_dec, 10000, &ind);
	buffer_append_float32_auto(buffer, conf.foc_start_curr_dec_rpm, &ind);
	buffer_append_float32_auto(buffer, conf.foc_openloop_rpm, &ind);
	buffer_append_float16(buffer, conf.foc_openloop_rpm_low, 1000, &ind);
	buffer_append_float16(buffer, conf.foc_d_gain_scale_start, 1000, &ind);
	buffer_append_float16(buffer, conf.foc_d_gain_scale_max_mod, 1000, &ind);
	buffer_append_float16(buffer, conf.foc_sl_openloop_hyst, 100, &ind);
	buffer_append_float16(buffer, conf.foc_sl_openloop_time_lock, 100, &ind);
	buffer_append_float16(buffer, conf.foc_sl_openloop_time_ramp, 100, &ind);
	buffer_append_float16(buffer, conf.foc_sl_openloop_time, 100, &ind);
	buffer_append_float16(buffer, conf.foc_sl_openloop_boost_q, 100, &ind);
	buffer_append_float16(buffer, conf.foc_sl_openloop_max_q, 100, &ind);
	buffer[ind++] = (uint8_t)conf.foc_hall_table[0];
	buffer[ind++] = (uint8_t)conf.foc_hall_table[1];
	buffer[ind++] = (uint8_t)conf.foc_hall_table[2];
	buffer[ind++] = (uint8_t)conf.foc_hall_table[3];
	buffer[ind++] = (uint8_t)conf.foc_hall_table[4];
	buffer[ind++] = (uint8_t)conf.foc_hall_table[5];
	buffer[ind++] = (uint8_t)conf.foc_hall_table[6];
	buffer[ind++] = (uint8_t)conf.foc_hall_table[7];
	buffer_append_float32_auto(buffer, conf.foc_hall_interp_erpm, &ind);
	buffer_append_float32_auto(buffer, conf.foc_sl_erpm_start, &ind);
	buffer_append_float32_auto(buffer, conf.foc_sl_erpm, &ind);
	buffer[ind++] = conf.foc_sample_v0_v7;
	buffer[ind++] = conf.foc_sample_high_current;
	buffer[ind++] = conf.foc_sat_comp_mode;
	buffer_append_float16(buffer, conf.foc_sat_comp, 1000, &ind);
	buffer[ind++] = conf.foc_temp_comp;
	buffer_append_float16(buffer, conf.foc_temp_comp_base_temp, 100, &ind);
	buffer_append_float16(buffer, conf.foc_current_filter_const, 10000, &ind);
	buffer[ind++] = conf.foc_cc_decoupling;
	buffer[ind++] = conf.foc_observer_type;
	buffer_append_float16(buffer, conf.foc_hfi_voltage_start, 10, &ind);
	buffer_append_float16(buffer, conf.foc_hfi_voltage_run, 10, &ind);
	buffer_append_float16(buffer, conf.foc_hfi_voltage_max, 10, &ind);
	buffer_append_float16(buffer, conf.foc_hfi_gain, 1000, &ind);
	buffer_append_float16(buffer, conf.foc_hfi_hyst, 100, &ind);
	buffer_append_float32_auto(buffer, conf.foc_sl_erpm_hfi, &ind);
	buffer_append_uint16(buffer, conf.foc_hfi_start_samples, &ind);
	buffer_append_float32_auto(buffer, conf.foc_hfi_obs_ovr_sec, &ind);
	buffer[ind++] = conf.foc_hfi_samples;
	buffer[ind++] = conf.foc_offsets_cal_on_boot;
	buffer_append_float32_auto(buffer, conf.foc_offsets_current[0], &ind);
	buffer_append_float32_auto(buffer, conf.foc_offsets_current[1], &ind);
	buffer_append_float32_auto(buffer, conf.foc_offsets_current[2], &ind);
	buffer_append_float16(buffer, conf.foc_offsets_voltage[0], 10000, &ind);
	buffer_append_float16(buffer, conf.foc_offsets_voltage[1], 10000, &ind);
	buffer_append_float16(buffer, conf.foc_offsets_voltage[2], 10000, &ind);
	buffer_append_float16(buffer, conf.foc_offsets_voltage_undriven[0], 10000, &ind);
	buffer_append_float16(buffer, conf.foc_offsets_voltage_undriven[1], 10000, &ind);
	buffer_append_float16(buffer, conf.foc_offsets_voltage_undriven[2], 10000, &ind);
	buffer[ind++] = conf.foc_phase_filter_enable;
	buffer[ind++] = conf.foc_phase_filter_disable_fault;
	buffer_append_float32_auto(buffer, conf.foc_phase_filter_max_erpm, &ind);
	buffer[ind++] = conf.foc_mtpa_mode;
	buffer_append_float32_auto(buffer, conf.foc_fw_current_max, &ind);
	buffer_append_float16(buffer, conf.foc_fw_duty_start, 10000, &ind);
	buffer_append_float16(buffer, conf.foc_fw_ramp_time, 1000, &ind);
	buffer_append_float16(buffer, conf.foc_fw_q_current_factor, 10000, &ind);
	buffer[ind++] = conf.foc_speed_soure;
	buffer_append_int16(buffer, conf.gpd_buffer_notify_left, &ind);
	buffer_append_int16(buffer, conf.gpd_buffer_interpol, &ind);
	buffer_append_float16(buffer, conf.gpd_current_filter_const, 10000, &ind);
	buffer_append_float32_auto(buffer, conf.gpd_current_kp, &ind);
	buffer_append_float32_auto(buffer, conf.gpd_current_ki, &ind);
	buffer[ind++] = conf.sp_pid_loop_rate;
	buffer_append_float32_auto(buffer, conf.s_pid_kp, &ind);
	buffer_append_float32_auto(buffer, conf.s_pid_ki, &ind);
	buffer_append_float32_auto(buffer, conf.s_pid_kd, &ind);
	buffer_append_float16(buffer, conf.s_pid_kd_filter, 10000, &ind);
	buffer_append_float32_auto(buffer, conf.s_pid_min_erpm, &ind);
	buffer[ind++] = conf.s_pid_allow_braking;
	buffer_append_float32_auto(buffer, conf.s_pid_ramp_erpms_s, &ind);
	buffer_append_float32_auto(buffer, conf.p_pid_kp, &ind);
	buffer_append_float32_auto(buffer, conf.p_pid_ki, &ind);
	buffer_append_float32_auto(buffer, conf.p_pid_kd, &ind);
	buffer_append_float32_auto(buffer, conf.p_pid_kd_proc, &ind);
	buffer_append_float16(buffer, conf.p_pid_kd_filter, 10000, &ind);
	buffer_append_float32_auto(buffer, conf.p_pid_ang_div, &ind);
	buffer_append_float16(buffer, conf.p_pid_gain_dec_angle, 10, &ind);
	buffer_append_float32_auto(buffer, conf.p_pid_offset, &ind);
	buffer_append_float16(buffer, conf.cc_startup_boost_duty, 10000, &ind);
	buffer_append_float32_auto(buffer, conf.cc_min_current, &ind);
	buffer_append_float32_auto(buffer, conf.cc_gain, &ind);
	buffer_append_float16(buffer, conf.cc_ramp_step_max, 10000, &ind);
	buffer_append_int32(buffer, conf.m_fault_stop_time_ms, &ind);
	buffer_append_float16(buffer, conf.m_duty_ramp_step, 10000, &ind);
	buffer_append_float32_auto(buffer, conf.m_current_backoff_gain, &ind);
	buffer_append_uint32(buffer, conf.m_encoder_counts, &ind);
	buffer_append_float16(buffer, conf.m_encoder_sin_amp, 1000, &ind);
	buffer_append_float16(buffer, conf.m_encoder_cos_amp, 1000, &ind);
	buffer_append_float16(buffer, conf.m_encoder_sin_offset, 1000, &ind);
	buffer_append_float16(buffer, conf.m_encoder_cos_offset, 1000, &ind);
	buffer_append_float16(buffer, conf.m_encoder_sincos_filter_constant, 1000, &ind);
	buffer_append_float16(buffer, conf.m_encoder_sincos_phase_correction, 1000, &ind);
	buffer[ind++] = conf.m_sensor_port_mode;
	buffer[ind++] = conf.m_invert_direction;
	buffer[ind++] = conf.m_drv8301_oc_mode;
	buffer[ind++] = (uint8_t)conf.m_drv8301_oc_adj;
	buffer_append_float32_auto(buffer, conf.m_bldc_f_sw_min, &ind);
	buffer_append_float32_auto(buffer, conf.m_bldc_f_sw_max, &ind);
	buffer_append_float32_auto(buffer, conf.m_dc_f_sw, &ind);
	buffer_append_float32_auto(buffer, conf.m_ntc_motor_beta, &ind);
	buffer[ind++] = conf.m_out_aux_mode;
	buffer[ind++] = conf.m_motor_temp_sens_type;
	buffer_append_float32_auto(buffer, conf.m_ptc_motor_coeff, &ind);
	buffer_append_float16(buffer, conf.m_ntcx_ptcx_res, 0.1, &ind);
	buffer_append_float16(buffer, conf.m_ntcx_ptcx_temp_base, 10, &ind);
	buffer[ind++] = (uint8_t)conf.m_hall_extra_samples;
	buffer[ind++] = (uint8_t)conf.m_batt_filter_const;
	buffer[ind++] = (uint8_t)conf.si_motor_poles;
	buffer_append_float32_auto(buffer, conf.si_gear_ratio, &ind);
	buffer_append_float32_auto(buffer, conf.si_wheel_diameter, &ind);
	buffer[ind++] = conf.si_battery_type;
	buffer[ind++] = (uint8_t)conf.si_battery_cells;
	buffer_append_float32_auto(buffer, conf.si_battery_ah, &ind);
	buffer_append_float32_auto(buffer, conf.si_motor_nl_current, &ind);
	buffer[ind++] = conf.bms.type;
	buffer[ind++] = conf.bms.limit_mode;
	buffer_append_float16(buffer, conf.bms.t_limit_start, 100, &ind);
	buffer_append_float16(buffer, conf.bms.t_limit_end, 100, &ind);
	buffer_append_float16(buffer, conf.bms.soc_limit_start, 1000, &ind);
	buffer_append_float16(buffer, conf.bms.soc_limit_end, 1000, &ind);
	buffer[ind++] = conf.bms.fwd_can_mode;

	return ind;
}

bool ProcessReadPacket(uint8_t* message, bldcMeasure& values, int len) {
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

// https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L407
bool ProcessReadPacket(uint8_t* buffer, mc_configuration& mcconf, int len) {

  int32_t ind = 0;
  COMM_PACKET_ID packetId;

  packetId = (COMM_PACKET_ID)buffer[0];
  if(packetId == COMM_GET_MCCONF){
    Serial.print("Received COMM_GET_MCCONF");
  }
  buffer++;//Eliminates the message id
  len--;

	uint32_t signature = buffer_get_uint32(buffer, &ind);

	mcconf.pwm_mode = (mc_pwm_mode)buffer[ind++];
	mcconf.comm_mode =  (mc_comm_mode)buffer[ind++];
	mcconf.motor_type = (mc_motor_type)buffer[ind++];
	mcconf.sensor_mode = (mc_sensor_mode)buffer[ind++];
	mcconf.l_current_max = buffer_get_float32_auto(buffer, &ind);
	mcconf.l_current_min = buffer_get_float32_auto(buffer, &ind);
	mcconf.l_in_current_max = buffer_get_float32_auto(buffer, &ind);
	mcconf.l_in_current_min = buffer_get_float32_auto(buffer, &ind);
	mcconf.l_abs_current_max = buffer_get_float32_auto(buffer, &ind);
	mcconf.l_min_erpm = buffer_get_float32_auto(buffer, &ind);
	mcconf.l_max_erpm = buffer_get_float32_auto(buffer, &ind);
	mcconf.l_erpm_start = buffer_get_float16(buffer, 10000, &ind);
	mcconf.l_max_erpm_fbrake = buffer_get_float32_auto(buffer, &ind);
	mcconf.l_max_erpm_fbrake_cc = buffer_get_float32_auto(buffer, &ind);
	mcconf.l_min_vin = buffer_get_float16(buffer, 10, &ind);
	mcconf.l_max_vin = buffer_get_float16(buffer, 10, &ind);
	mcconf.l_battery_cut_start = buffer_get_float16(buffer, 10, &ind);
	mcconf.l_battery_cut_end = buffer_get_float16(buffer, 10, &ind);
	mcconf.l_battery_regen_cut_start = buffer_get_float16(buffer, 10, &ind);
	mcconf.l_battery_regen_cut_end = buffer_get_float16(buffer, 10, &ind);
	mcconf.l_slow_abs_current = buffer[ind++];
	mcconf.l_temp_fet_start = buffer_get_float16(buffer, 10, &ind);
	mcconf.l_temp_fet_end = buffer_get_float16(buffer, 10, &ind);
	mcconf.l_temp_motor_start = buffer_get_float16(buffer, 10, &ind);
	mcconf.l_temp_motor_end = buffer_get_float16(buffer, 10, &ind);
	mcconf.l_temp_accel_dec = buffer_get_float16(buffer, 10000, &ind);
	mcconf.l_min_duty = buffer_get_float16(buffer, 10000, &ind);
	mcconf.l_max_duty = buffer_get_float16(buffer, 10000, &ind);
	mcconf.l_watt_max = buffer_get_float32_auto(buffer, &ind);
	mcconf.l_watt_min = buffer_get_float32_auto(buffer, &ind);
	mcconf.l_current_max_scale = buffer_get_float16(buffer, 10000, &ind);
	mcconf.l_current_min_scale = buffer_get_float16(buffer, 10000, &ind);
	mcconf.l_duty_start = buffer_get_float16(buffer, 10000, &ind);
	mcconf.sl_min_erpm = buffer_get_float32_auto(buffer, &ind);
	mcconf.sl_min_erpm_cycle_int_limit = buffer_get_float32_auto(buffer, &ind);
	mcconf.sl_max_fullbreak_current_dir_change = buffer_get_float32_auto(buffer, &ind);
	mcconf.sl_cycle_int_limit = buffer_get_float16(buffer, 10, &ind);
	mcconf.sl_phase_advance_at_br = buffer_get_float16(buffer, 10000, &ind);
	mcconf.sl_cycle_int_rpm_br = buffer_get_float32_auto(buffer, &ind);
	mcconf.sl_bemf_coupling_k = buffer_get_float32_auto(buffer, &ind);
	mcconf.hall_table[0] = (int8_t)buffer[ind++];
	mcconf.hall_table[1] = (int8_t)buffer[ind++];
	mcconf.hall_table[2] = (int8_t)buffer[ind++];
	mcconf.hall_table[3] = (int8_t)buffer[ind++];
	mcconf.hall_table[4] = (int8_t)buffer[ind++];
	mcconf.hall_table[5] = (int8_t)buffer[ind++];
	mcconf.hall_table[6] = (int8_t)buffer[ind++];
	mcconf.hall_table[7] = (int8_t)buffer[ind++];
	mcconf.hall_sl_erpm = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_current_kp = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_current_ki = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_f_zv = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_dt_us = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_encoder_inverted = buffer[ind++];
	mcconf.foc_encoder_offset = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_encoder_ratio = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_sensor_mode = (mc_foc_sensor_mode)buffer[ind++];
	mcconf.foc_pll_kp = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_pll_ki = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_motor_l = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_motor_ld_lq_diff = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_motor_r = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_motor_flux_linkage = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_observer_gain = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_observer_gain_slow = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_observer_offset = buffer_get_float16(buffer, 1000, &ind);
	mcconf.foc_duty_dowmramp_kp = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_duty_dowmramp_ki = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_start_curr_dec = buffer_get_float16(buffer, 10000, &ind);
	mcconf.foc_start_curr_dec_rpm = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_openloop_rpm = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_openloop_rpm_low = buffer_get_float16(buffer, 1000, &ind);
	mcconf.foc_d_gain_scale_start = buffer_get_float16(buffer, 1000, &ind);
	mcconf.foc_d_gain_scale_max_mod = buffer_get_float16(buffer, 1000, &ind);
	mcconf.foc_sl_openloop_hyst = buffer_get_float16(buffer, 100, &ind);
	mcconf.foc_sl_openloop_time_lock = buffer_get_float16(buffer, 100, &ind);
	mcconf.foc_sl_openloop_time_ramp = buffer_get_float16(buffer, 100, &ind);
	mcconf.foc_sl_openloop_time = buffer_get_float16(buffer, 100, &ind);
	mcconf.foc_sl_openloop_boost_q = buffer_get_float16(buffer, 100, &ind);
	mcconf.foc_sl_openloop_max_q = buffer_get_float16(buffer, 100, &ind);
	mcconf.foc_hall_table[0] = buffer[ind++];
	mcconf.foc_hall_table[1] = buffer[ind++];
	mcconf.foc_hall_table[2] = buffer[ind++];
	mcconf.foc_hall_table[3] = buffer[ind++];
	mcconf.foc_hall_table[4] = buffer[ind++];
	mcconf.foc_hall_table[5] = buffer[ind++];
	mcconf.foc_hall_table[6] = buffer[ind++];
	mcconf.foc_hall_table[7] = buffer[ind++];
	mcconf.foc_hall_interp_erpm = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_sl_erpm_start = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_sl_erpm = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_sample_v0_v7 = buffer[ind++];
	mcconf.foc_sample_high_current = buffer[ind++];
	mcconf.foc_sat_comp_mode = (SAT_COMP_MODE )buffer[ind++];
	mcconf.foc_sat_comp = buffer_get_float16(buffer, 1000, &ind);
	mcconf.foc_temp_comp = buffer[ind++];
	mcconf.foc_temp_comp_base_temp = buffer_get_float16(buffer, 100, &ind);
	mcconf.foc_current_filter_const = buffer_get_float16(buffer, 10000, &ind);
	mcconf.foc_cc_decoupling = (mc_foc_cc_decoupling_mode)buffer[ind++];
	mcconf.foc_observer_type = (mc_foc_observer_type)buffer[ind++];
	mcconf.foc_hfi_voltage_start = buffer_get_float16(buffer, 10, &ind);
	mcconf.foc_hfi_voltage_run = buffer_get_float16(buffer, 10, &ind);
	mcconf.foc_hfi_voltage_max = buffer_get_float16(buffer, 10, &ind);
	mcconf.foc_hfi_gain = buffer_get_float16(buffer, 1000, &ind);
	mcconf.foc_hfi_hyst = buffer_get_float16(buffer, 100, &ind);
	mcconf.foc_sl_erpm_hfi = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_hfi_start_samples = buffer_get_uint16(buffer, &ind);
	mcconf.foc_hfi_obs_ovr_sec = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_hfi_samples = (foc_hfi_sample)buffer[ind++];
	mcconf.foc_offsets_cal_on_boot = buffer[ind++];
	mcconf.foc_offsets_current[0] = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_offsets_current[1] = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_offsets_current[2] = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_offsets_voltage[0] = buffer_get_float16(buffer, 10000, &ind);
	mcconf.foc_offsets_voltage[1] = buffer_get_float16(buffer, 10000, &ind);
	mcconf.foc_offsets_voltage[2] = buffer_get_float16(buffer, 10000, &ind);
	mcconf.foc_offsets_voltage_undriven[0] = buffer_get_float16(buffer, 10000, &ind);
	mcconf.foc_offsets_voltage_undriven[1] = buffer_get_float16(buffer, 10000, &ind);
	mcconf.foc_offsets_voltage_undriven[2] = buffer_get_float16(buffer, 10000, &ind);
	mcconf.foc_phase_filter_enable = buffer[ind++];
	mcconf.foc_phase_filter_disable_fault = buffer[ind++];
	mcconf.foc_phase_filter_max_erpm = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_mtpa_mode = (MTPA_MODE)buffer[ind++];
	mcconf.foc_fw_current_max = buffer_get_float32_auto(buffer, &ind);
	mcconf.foc_fw_duty_start = buffer_get_float16(buffer, 10000, &ind);
	mcconf.foc_fw_ramp_time = buffer_get_float16(buffer, 1000, &ind);
	mcconf.foc_fw_q_current_factor = buffer_get_float16(buffer, 10000, &ind);
	mcconf.foc_speed_soure = (SPEED_SRC)buffer[ind++];
	mcconf.gpd_buffer_notify_left = buffer_get_int16(buffer, &ind);
	mcconf.gpd_buffer_interpol = buffer_get_int16(buffer, &ind);
	mcconf.gpd_current_filter_const = buffer_get_float16(buffer, 10000, &ind);
	mcconf.gpd_current_kp = buffer_get_float32_auto(buffer, &ind);
	mcconf.gpd_current_ki = buffer_get_float32_auto(buffer, &ind);
	mcconf.sp_pid_loop_rate = (PID_RATE)buffer[ind++];
	mcconf.s_pid_kp = buffer_get_float32_auto(buffer, &ind);
	mcconf.s_pid_ki = buffer_get_float32_auto(buffer, &ind);
	mcconf.s_pid_kd = buffer_get_float32_auto(buffer, &ind);
	mcconf.s_pid_kd_filter = buffer_get_float16(buffer, 10000, &ind);
	mcconf.s_pid_min_erpm = buffer_get_float32_auto(buffer, &ind);
	mcconf.s_pid_allow_braking = buffer[ind++];
	mcconf.s_pid_ramp_erpms_s = buffer_get_float32_auto(buffer, &ind);
	mcconf.p_pid_kp = buffer_get_float32_auto(buffer, &ind);
	mcconf.p_pid_ki = buffer_get_float32_auto(buffer, &ind);
	mcconf.p_pid_kd = buffer_get_float32_auto(buffer, &ind);
	mcconf.p_pid_kd_proc = buffer_get_float32_auto(buffer, &ind);
	mcconf.p_pid_kd_filter = buffer_get_float16(buffer, 10000, &ind);
	mcconf.p_pid_ang_div = buffer_get_float32_auto(buffer, &ind);
	mcconf.p_pid_gain_dec_angle = buffer_get_float16(buffer, 10, &ind);
	mcconf.p_pid_offset = buffer_get_float32_auto(buffer, &ind);
	mcconf.cc_startup_boost_duty = buffer_get_float16(buffer, 10000, &ind);
	mcconf.cc_min_current = buffer_get_float32_auto(buffer, &ind);
	mcconf.cc_gain = buffer_get_float32_auto(buffer, &ind);
	mcconf.cc_ramp_step_max = buffer_get_float16(buffer, 10000, &ind);
	mcconf.m_fault_stop_time_ms = buffer_get_int32(buffer, &ind);
	mcconf.m_duty_ramp_step = buffer_get_float16(buffer, 10000, &ind);
	mcconf.m_current_backoff_gain = buffer_get_float32_auto(buffer, &ind);
	mcconf.m_encoder_counts = buffer_get_uint32(buffer, &ind);
	mcconf.m_encoder_sin_amp = buffer_get_float16(buffer, 1000, &ind);
	mcconf.m_encoder_cos_amp = buffer_get_float16(buffer, 1000, &ind);
	mcconf.m_encoder_sin_offset = buffer_get_float16(buffer, 1000, &ind);
	mcconf.m_encoder_cos_offset = buffer_get_float16(buffer, 1000, &ind);
	mcconf.m_encoder_sincos_filter_constant = buffer_get_float16(buffer, 1000, &ind);
	mcconf.m_encoder_sincos_phase_correction = buffer_get_float16(buffer, 1000, &ind);
	mcconf.m_sensor_port_mode = (sensor_port_mode)buffer[ind++];
	mcconf.m_invert_direction = buffer[ind++];
	mcconf.m_drv8301_oc_mode = (drv8301_oc_mode)buffer[ind++];
	mcconf.m_drv8301_oc_adj = buffer[ind++];
	mcconf.m_bldc_f_sw_min = buffer_get_float32_auto(buffer, &ind);
	mcconf.m_bldc_f_sw_max = buffer_get_float32_auto(buffer, &ind);
	mcconf.m_dc_f_sw = buffer_get_float32_auto(buffer, &ind);
	mcconf.m_ntc_motor_beta = buffer_get_float32_auto(buffer, &ind);
	mcconf.m_out_aux_mode = (out_aux_mode)buffer[ind++];
	mcconf.m_motor_temp_sens_type = (temp_sensor_type)buffer[ind++];
	mcconf.m_ptc_motor_coeff = buffer_get_float32_auto(buffer, &ind);
	mcconf.m_ntcx_ptcx_res = buffer_get_float16(buffer, 0.1, &ind);
	mcconf.m_ntcx_ptcx_temp_base = buffer_get_float16(buffer, 10, &ind);
	mcconf.m_hall_extra_samples = buffer[ind++];
	mcconf.m_batt_filter_const = buffer[ind++];
	mcconf.si_motor_poles = buffer[ind++];
	mcconf.si_gear_ratio = buffer_get_float32_auto(buffer, &ind);
	mcconf.si_wheel_diameter = buffer_get_float32_auto(buffer, &ind);
	mcconf.si_battery_type = (BATTERY_TYPE)buffer[ind++];
	mcconf.si_battery_cells = buffer[ind++];
	mcconf.si_battery_ah = buffer_get_float32_auto(buffer, &ind);
	mcconf.si_motor_nl_current = buffer_get_float32_auto(buffer, &ind);
	mcconf.bms.type = (BMS_TYPE)buffer[ind++];
	mcconf.bms.limit_mode = buffer[ind++];
	mcconf.bms.t_limit_start = buffer_get_float16(buffer, 100, &ind);
	mcconf.bms.t_limit_end = buffer_get_float16(buffer, 100, &ind);
	mcconf.bms.soc_limit_start = buffer_get_float16(buffer, 1000, &ind);
	mcconf.bms.soc_limit_end = buffer_get_float16(buffer, 1000, &ind);
	mcconf.bms.fwd_can_mode = (BMS_FWD_CAN_MODE)buffer[ind++];

	return true;
} 

bool VescUartGet(bldcMeasure& values, int num) {
  uint8_t command[1] = { COMM_GET_VALUES };
  //uint8_t payload[256];glob
  PackSendPayload(command, 1, num);
  delay(10); //needed, otherwise data is not read
  int lenPayload = ReceiveUartMessage2(payloadGlobal, num);
  if (lenPayload > 1) {
    bool return_val = ProcessReadPacket(payloadGlobal, values, lenPayload); //returns true if sucessful
    return return_val;
  }
  else
  {
    return false;
  }
}
bool VescUartGet(bldcMeasure& values) {
  return VescUartGet(values, 0);
}




bool VescUartGet(mc_configuration& config, int num) {
  uint8_t command[2] = { COMM_GET_MCCONF };  //COMM_GET_MCCONF  COMM_GET_VALUES
  //uint8_t payload[400];glob
 
  PackSendPayload(command, 1, num);
  delay(10); //needed, otherwise data is not read
  int lenPayload = ReceiveUartMessage2(payloadGlobal, num);// MC
  
  Serial.print("Received");
  Serial.print(lenPayload);
  Serial.println(" Bytes");

  if(lenPayload > 1) {Serial.print("code ");Serial.print(payloadGlobal[0]); Serial.println(payloadGlobal[1]); Serial.println(payloadGlobal[2]); Serial.println(payloadGlobal[3]);};

  if (lenPayload > 1 && payloadGlobal[0] == COMM_GET_MCCONF) {
    bool return_val = ProcessReadPacket(payloadGlobal, config, lenPayload); //returns true if sucessful 
    Serial.print("Received COMM_GET_MCCONFIG");
    Serial.println(return_val);
    return return_val;
  }
 
 
    //Serial.print("no comm with vesc or too short message got");
    return false; //no comm with vesc or too short message got.
 
}

void confgenerator_set_defaults_mcconf(mc_configuration *conf) {
	conf->pwm_mode = MCCONF_PWM_MODE;
	conf->comm_mode = MCCONF_COMM_MODE;
	conf->motor_type = MCCONF_DEFAULT_MOTOR_TYPE;
	conf->sensor_mode = MCCONF_SENSOR_MODE;
	conf->l_current_max = MCCONF_L_CURRENT_MAX;
	conf->l_current_min = MCCONF_L_CURRENT_MIN;
	conf->l_in_current_max = MCCONF_L_IN_CURRENT_MAX;
	conf->l_in_current_min = MCCONF_L_IN_CURRENT_MIN;
	conf->l_abs_current_max = MCCONF_L_MAX_ABS_CURRENT;
	conf->l_min_erpm = MCCONF_L_RPM_MIN;
	conf->l_max_erpm = MCCONF_L_RPM_MAX;
	conf->l_erpm_start = MCCONF_L_RPM_START;
	conf->l_max_erpm_fbrake = MCCONF_L_CURR_MAX_RPM_FBRAKE;
	conf->l_max_erpm_fbrake_cc = MCCONF_L_CURR_MAX_RPM_FBRAKE_CC;
	conf->l_min_vin = MCCONF_L_MIN_VOLTAGE;
	conf->l_max_vin = MCCONF_L_MAX_VOLTAGE;
	conf->l_battery_cut_start = MCCONF_L_BATTERY_CUT_START;
	conf->l_battery_cut_end = MCCONF_L_BATTERY_CUT_END;
	conf->l_battery_regen_cut_start = MCCONF_L_BATTERY_REGEN_CUT_START;
	conf->l_battery_regen_cut_end = MCCONF_L_BATTERY_REGEN_CUT_END;
	conf->l_slow_abs_current = MCCONF_L_SLOW_ABS_OVERCURRENT;
	conf->l_temp_fet_start = MCCONF_L_LIM_TEMP_FET_START;
	conf->l_temp_fet_end = MCCONF_L_LIM_TEMP_FET_END;
	conf->l_temp_motor_start = MCCONF_L_LIM_TEMP_MOTOR_START;
	conf->l_temp_motor_end = MCCONF_L_LIM_TEMP_MOTOR_END;
	conf->l_temp_accel_dec = MCCONF_L_LIM_TEMP_ACCEL_DEC;
	conf->l_min_duty = MCCONF_L_MIN_DUTY;
	conf->l_max_duty = MCCONF_L_MAX_DUTY;
	conf->l_watt_max = MCCONF_L_WATT_MAX;
	conf->l_watt_min = MCCONF_L_WATT_MIN;
	conf->l_current_max_scale = MCCONF_L_CURRENT_MAX_SCALE;
	conf->l_current_min_scale = MCCONF_L_CURRENT_MIN_SCALE;
	conf->l_duty_start = MCCONF_L_DUTY_START;
	conf->sl_min_erpm = MCCONF_SL_MIN_RPM;
	conf->sl_min_erpm_cycle_int_limit = MCCONF_SL_MIN_ERPM_CYCLE_INT_LIMIT;
	conf->sl_max_fullbreak_current_dir_change = MCCONF_SL_MAX_FB_CURR_DIR_CHANGE;
	conf->sl_cycle_int_limit = MCCONF_SL_CYCLE_INT_LIMIT;
	conf->sl_phase_advance_at_br = MCCONF_SL_PHASE_ADVANCE_AT_BR;
	conf->sl_cycle_int_rpm_br = MCCONF_SL_CYCLE_INT_BR;
	conf->sl_bemf_coupling_k = MCCONF_SL_BEMF_COUPLING_K;
	conf->hall_table[0] = MCCONF_HALL_TAB_0;
	conf->hall_table[1] = MCCONF_HALL_TAB_1;
	conf->hall_table[2] = MCCONF_HALL_TAB_2;
	conf->hall_table[3] = MCCONF_HALL_TAB_3;
	conf->hall_table[4] = MCCONF_HALL_TAB_4;
	conf->hall_table[5] = MCCONF_HALL_TAB_5;
	conf->hall_table[6] = MCCONF_HALL_TAB_6;
	conf->hall_table[7] = MCCONF_HALL_TAB_7;
	conf->hall_sl_erpm = MCCONF_HALL_ERPM;
	conf->foc_current_kp = MCCONF_FOC_CURRENT_KP;
	conf->foc_current_ki = MCCONF_FOC_CURRENT_KI;
	conf->foc_f_zv = MCCONF_FOC_F_ZV;
	conf->foc_dt_us = MCCONF_FOC_DT_US;
	conf->foc_encoder_inverted = MCCONF_FOC_ENCODER_INVERTED;
	conf->foc_encoder_offset = MCCONF_FOC_ENCODER_OFFSET;
	conf->foc_encoder_ratio = MCCONF_FOC_ENCODER_RATIO;
	conf->foc_sensor_mode = MCCONF_FOC_SENSOR_MODE;
	conf->foc_pll_kp = MCCONF_FOC_PLL_KP;
	conf->foc_pll_ki = MCCONF_FOC_PLL_KI;
	conf->foc_motor_l = MCCONF_FOC_MOTOR_L;
	conf->foc_motor_ld_lq_diff = MCCONF_FOC_MOTOR_LD_LQ_DIFF;
	conf->foc_motor_r = MCCONF_FOC_MOTOR_R;
	conf->foc_motor_flux_linkage = MCCONF_FOC_MOTOR_FLUX_LINKAGE;
	conf->foc_observer_gain = MCCONF_FOC_OBSERVER_GAIN;
	conf->foc_observer_gain_slow = MCCONF_FOC_OBSERVER_GAIN_SLOW;
	conf->foc_observer_offset = MCCONF_FOC_OBSERVER_OFFSET;
	conf->foc_duty_dowmramp_kp = MCCONF_FOC_DUTY_DOWNRAMP_KP;
	conf->foc_duty_dowmramp_ki = MCCONF_FOC_DUTY_DOWNRAMP_KI;
	conf->foc_start_curr_dec = MCCONF_FOC_START_CURR_DEC;
	conf->foc_start_curr_dec_rpm = MCCONF_FOC_START_CURR_DEC_RPM;
	conf->foc_openloop_rpm = MCCONF_FOC_OPENLOOP_RPM;
	conf->foc_openloop_rpm_low = MCCONF_FOC_OPENLOOP_RPM_LOW;
	conf->foc_d_gain_scale_start = MCCONF_FOC_D_GAIN_SCALE_START;
	conf->foc_d_gain_scale_max_mod = MCCONF_FOC_D_GAIN_SCALE_MAX_MOD;
	conf->foc_sl_openloop_hyst = MCCONF_FOC_SL_OPENLOOP_HYST;
	conf->foc_sl_openloop_time_lock = MCCONF_FOC_SL_OPENLOOP_T_LOCK;
	conf->foc_sl_openloop_time_ramp = MCCONF_FOC_SL_OPENLOOP_T_RAMP;
	conf->foc_sl_openloop_time = MCCONF_FOC_SL_OPENLOOP_TIME;
	conf->foc_sl_openloop_boost_q = MCCONF_FOC_SL_OPENLOOP_BOOST_Q;
	conf->foc_sl_openloop_max_q = MCCONF_FOC_SL_OPENLOOP_MAX_Q;
	conf->foc_hall_table[0] = MCCONF_FOC_HALL_TAB_0;
	conf->foc_hall_table[1] = MCCONF_FOC_HALL_TAB_1;
	conf->foc_hall_table[2] = MCCONF_FOC_HALL_TAB_2;
	conf->foc_hall_table[3] = MCCONF_FOC_HALL_TAB_3;
	conf->foc_hall_table[4] = MCCONF_FOC_HALL_TAB_4;
	conf->foc_hall_table[5] = MCCONF_FOC_HALL_TAB_5;
	conf->foc_hall_table[6] = MCCONF_FOC_HALL_TAB_6;
	conf->foc_hall_table[7] = MCCONF_FOC_HALL_TAB_7;
	conf->foc_hall_interp_erpm = MCCONF_FOC_HALL_INTERP_ERPM;
	conf->foc_sl_erpm_start = MCCONF_FOC_SL_ERPM_START;
	conf->foc_sl_erpm = MCCONF_FOC_SL_ERPM;
	conf->foc_sample_v0_v7 = MCCONF_FOC_SAMPLE_V0_V7;
	conf->foc_sample_high_current = MCCONF_FOC_SAMPLE_HIGH_CURRENT;
	conf->foc_sat_comp_mode = MCCONF_FOC_SAT_COMP_MODE;
	conf->foc_sat_comp = MCCONF_FOC_SAT_COMP;
	conf->foc_temp_comp = MCCONF_FOC_TEMP_COMP;
	conf->foc_temp_comp_base_temp = MCCONF_FOC_TEMP_COMP_BASE_TEMP;
	conf->foc_current_filter_const = MCCONF_FOC_CURRENT_FILTER_CONST;
	conf->foc_cc_decoupling = MCCONF_FOC_CC_DECOUPLING;
	conf->foc_observer_type = MCCONF_FOC_OBSERVER_TYPE;
	conf->foc_hfi_voltage_start = MCCONF_FOC_HFI_VOLTAGE_START;
	conf->foc_hfi_voltage_run = MCCONF_FOC_HFI_VOLTAGE_RUN;
	conf->foc_hfi_voltage_max = MCCONF_FOC_HFI_VOLTAGE_MAX;
	conf->foc_hfi_gain = MCCONF_FOC_HFI_GAIN;
	conf->foc_hfi_hyst = MCCONF_FOC_HFI_HYST;
	conf->foc_sl_erpm_hfi = MCCONF_FOC_SL_ERPM_HFI;
	conf->foc_hfi_start_samples = MCCONF_FOC_HFI_START_SAMPLES;
	conf->foc_hfi_obs_ovr_sec = MCCONF_FOC_HFI_OBS_OVR_SEC;
	conf->foc_hfi_samples = MCCONF_FOC_HFI_SAMPLES;
	conf->foc_offsets_cal_on_boot = MCCONF_FOC_OFFSETS_CAL_ON_BOOT;
	conf->foc_offsets_current[0] = MCCONF_FOC_OFFSETS_CURRENT_0;
	conf->foc_offsets_current[1] = MCCONF_FOC_OFFSETS_CURRENT_1;
	conf->foc_offsets_current[2] = MCCONF_FOC_OFFSETS_CURRENT_2;
	conf->foc_offsets_voltage[0] = MCCONF_FOC_OFFSETS_VOLTAGE_0;
	conf->foc_offsets_voltage[1] = MCCONF_FOC_OFFSETS_VOLTAGE_1;
	conf->foc_offsets_voltage[2] = MCCONF_FOC_OFFSETS_VOLTAGE_2;
	conf->foc_offsets_voltage_undriven[0] = MCCONF_FOC_OFFSETS_VOLTAGE_UNDRIVEN_0;
	conf->foc_offsets_voltage_undriven[1] = MCCONF_FOC_OFFSETS_VOLTAGE_UNDRIVEN_1;
	conf->foc_offsets_voltage_undriven[2] = MCCONF_FOC_OFFSETS_VOLTAGE_UNDRIVEN_2;
	conf->foc_phase_filter_enable = MCCONF_FOC_PHASE_FILTER_ENABLE;
	conf->foc_phase_filter_disable_fault = MCCONF_FOC_PHASE_FILTER_DISABLE_FAULT;
	conf->foc_phase_filter_max_erpm = MCCONF_FOC_PHASE_FILTER_MAX_ERPM;
	conf->foc_mtpa_mode = MCCONF_FOC_MTPA_MODE;
	conf->foc_fw_current_max = MCCONF_FOC_FW_CURRENT_MAX;
	conf->foc_fw_duty_start = MCCONF_FOC_FW_DUTY_START;
	conf->foc_fw_ramp_time = MCCONF_FOC_FW_RAMP_TIME;
	conf->foc_fw_q_current_factor = MCCONF_FOC_FW_Q_CURRENT_FACTOR;
	conf->foc_speed_soure = MCCONF_FOC_SPEED_SOURCE;
	conf->gpd_buffer_notify_left = MCCONF_GPD_BUFFER_NOTIFY_LEFT;
	conf->gpd_buffer_interpol = MCCONF_GPD_BUFFER_INTERPOL;
	conf->gpd_current_filter_const = MCCONF_GPD_CURRENT_FILTER_CONST;
	conf->gpd_current_kp = MCCONF_GPD_CURRENT_KP;
	conf->gpd_current_ki = MCCONF_GPD_CURRENT_KI;
	conf->sp_pid_loop_rate = MCCONF_SP_PID_LOOP_RATE;
	conf->s_pid_kp = MCCONF_S_PID_KP;
	conf->s_pid_ki = MCCONF_S_PID_KI;
	conf->s_pid_kd = MCCONF_S_PID_KD;
	conf->s_pid_kd_filter = MCCONF_S_PID_KD_FILTER;
	conf->s_pid_min_erpm = MCCONF_S_PID_MIN_RPM;
	conf->s_pid_allow_braking = MCCONF_S_PID_ALLOW_BRAKING;
	conf->s_pid_ramp_erpms_s = MCCONF_S_PID_RAMP_ERPMS_S;
	conf->p_pid_kp = MCCONF_P_PID_KP;
	conf->p_pid_ki = MCCONF_P_PID_KI;
	conf->p_pid_kd = MCCONF_P_PID_KD;
	conf->p_pid_kd_proc = MCCONF_P_PID_KD_PROC;
	conf->p_pid_kd_filter = MCCONF_P_PID_KD_FILTER;
	conf->p_pid_ang_div = MCCONF_P_PID_ANG_DIV;
	conf->p_pid_gain_dec_angle = MCCONF_P_PID_GAIN_DEC_ANGLE;
	conf->p_pid_offset = MCCONF_P_PID_OFFSET;
	conf->cc_startup_boost_duty = MCCONF_CC_STARTUP_BOOST_DUTY;
	conf->cc_min_current = MCCONF_CC_MIN_CURRENT;
	conf->cc_gain = MCCONF_CC_GAIN;
	conf->cc_ramp_step_max = MCCONF_CC_RAMP_STEP;
	conf->m_fault_stop_time_ms = MCCONF_M_FAULT_STOP_TIME;
	conf->m_duty_ramp_step = MCCONF_M_RAMP_STEP;
	conf->m_current_backoff_gain = MCCONF_M_CURRENT_BACKOFF_GAIN;
	conf->m_encoder_counts = MCCONF_M_ENCODER_COUNTS;
	conf->m_encoder_sin_amp = MCCONF_M_ENCODER_SIN_AMP;
	conf->m_encoder_cos_amp = MCCONF_M_ENCODER_COS_AMP;
	conf->m_encoder_sin_offset = MCCONF_M_ENCODER_SIN_OFFSET;
	conf->m_encoder_cos_offset = MCCONF_M_ENCODER_COS_OFFSET;
	conf->m_encoder_sincos_filter_constant = MCCONF_M_ENCODER_SINCOS_FILTER;
	conf->m_encoder_sincos_phase_correction = (float)MCCONF_M_ENCODER_SINCOS_PHASE;
	conf->m_sensor_port_mode = MCCONF_M_SENSOR_PORT_MODE;
	conf->m_invert_direction = MCCONF_M_INVERT_DIRECTION;
	conf->m_drv8301_oc_mode = MCCONF_M_DRV8301_OC_MODE;
	conf->m_drv8301_oc_adj = MCCONF_M_DRV8301_OC_ADJ;
	conf->m_bldc_f_sw_min = MCCONF_M_BLDC_F_SW_MIN;
	conf->m_bldc_f_sw_max = MCCONF_M_BLDC_F_SW_MAX;
	conf->m_dc_f_sw = MCCONF_M_DC_F_SW;
	conf->m_ntc_motor_beta = MCCONF_M_NTC_MOTOR_BETA;
	conf->m_out_aux_mode = MCCONF_M_OUT_AUX_MODE;
	conf->m_motor_temp_sens_type = MCCONF_M_MOTOR_TEMP_SENS_TYPE;
	conf->m_ptc_motor_coeff = MCCONF_M_PTC_MOTOR_COEFF;
	conf->m_ntcx_ptcx_res = MCCONF_M_NTCX_PTCX_RES;
	conf->m_ntcx_ptcx_temp_base = MCCONF_M_NTCX_PTCX_BASE_TEMP;
	conf->m_hall_extra_samples = MCCONF_M_HALL_EXTRA_SAMPLES;
	conf->m_batt_filter_const = MCCONF_M_BATT_FILTER_CONST;
	conf->si_motor_poles = MCCONF_SI_MOTOR_POLES;
	conf->si_gear_ratio = MCCONF_SI_GEAR_RATIO;
	conf->si_wheel_diameter = MCCONF_SI_WHEEL_DIAMETER;
	conf->si_battery_type = MCCONF_SI_BATTERY_TYPE;
	conf->si_battery_cells = MCCONF_SI_BATTERY_CELLS;
	conf->si_battery_ah = MCCONF_SI_BATTERY_AH;
	conf->si_motor_nl_current = MCCONF_SI_MOTOR_NL_CURRENT;
	conf->bms.type = MCCONF_BMS_TYPE;
	conf->bms.limit_mode = MCCONF_BMS_LIMIT_MODE;
	conf->bms.t_limit_start = MCCONF_BMS_T_LIMIT_START;
	conf->bms.t_limit_end = MCCONF_BMS_T_LIMIT_END;
	conf->bms.soc_limit_start = MCCONF_BMS_SOC_LIMIT_START;
	conf->bms.soc_limit_end = MCCONF_BMS_SOC_LIMIT_END;
	conf->bms.fwd_can_mode = MCCONF_BMS_FWD_CAN_MODE;
}



void commands_send_mcconf(COMM_PACKET_ID packet_id, mc_configuration* mcconf, void(*reply_func)(unsigned char* data, unsigned int len)) {
	uint8_t *send_buffer_global = mempools_get_packet_buffer();
	send_buffer_global[0] = packet_id;
	int32_t len = BuildPacket(send_buffer_global + 1, *mcconf); //confgenerator_serialize_mcconf

  PackSendPayload(send_buffer_global, len + 1, 0);

	mempools_free_packet_buffer(send_buffer_global);
}



void commands_send_GET_MCCONF(){

  mc_configuration *mcconf = mempools_alloc_mcconf();

  confgenerator_set_defaults_mcconf(mcconf);
 
  commands_send_mcconf(COMM_GET_MCCONF, mcconf, 0);
  
  mempools_free_mcconf(mcconf);

}



bool VescUartGet(mc_configuration& config) {
  return VescUartGet(config, 0);
}

bool VescUartSet(mc_configuration& config, int num) {
  
 uint8_t receivedpayloadGlobal[500];
  //uint8_t payload[360];glob //= {3, 1, 84, 14, 1, 0, 2, 2, 66, 92, 0, 0, 194, 72, 0, 0, 66, 180, 0, 0, 194, 32, 0, 0, 67, 22, 0, 0, 199, 195, 80, 0, 71, 5, 252, 0, 63, 76, 204, 205, 67, 150, 0, 0, 68, 187, 128, 0, 65, 0, 0, 0, 66, 100, 0, 0, 66, 35, 51, 51, 66, 20, 204, 205, 1, 66, 170, 0, 0, 66, 200, 0, 0, 66, 170, 0, 0, 66, 200, 0, 0, 62, 25, 153, 154, 59, 163, 215, 10, 63, 115, 51, 51, 70, 106, 96, 0, 198, 106, 96, 0, 67, 22, 0, 0, 68, 137, 128, 0, 65, 32, 0, 0, 66, 120, 0, 0, 63, 76, 204, 205, 71, 156, 64, 0, 68, 22, 0, 0, 255, 1, 3, 2, 5, 6, 4, 255, 68, 250, 0, 0, 61, 35, 215, 10, 66, 4, 225, 72, 70, 156, 64, 0, 61, 163, 215, 10, 0, 67, 52, 0, 0, 64, 224, 0, 0, 2, 68, 250, 0, 0, 71, 28, 64, 0, 56, 39, 143, 252, 61, 7, 252, 185, 60, 133, 37, 3, 74, 103, 82, 192, 62, 153, 153, 154, 65, 32, 0, 0, 67, 72, 0, 0, 67, 200, 0, 0, 61, 204, 204, 205, 61, 204, 204, 205, 0, 0, 0, 0, 0, 0, 0, 0, 255, 95, 27, 62, 163, 127, 195, 255, 69, 28, 64, 0, 1, 0, 0, 0, 0, 0, 0, 65, 200, 0, 0, 61, 204, 204, 205, 59, 131, 18, 111, 59, 131, 18, 111, 56, 209, 183, 23, 62, 76, 204, 205, 68, 97, 0, 0, 1, 60, 245, 194, 143, 0, 0, 0, 0, 57, 209, 183, 23, 62, 76, 204, 205, 63, 128, 0, 0, 60, 35, 215, 10, 61, 204, 204, 205, 59, 150, 187, 153, 61, 35, 215, 10, 0, 0, 1, 244, 60, 163, 215, 10, 63, 0, 0, 0, 0, 0, 32, 0, 0, 0, 0, 16, 69, 59, 128, 0, 71, 28, 64, 0, 71, 8, 184, 0, 69, 83, 64, 0, 93, 32, 3, 0 };
  //debugSerialPort->print("comp  ");
  //SerialPrint(payload, 360);
  
  for(int i = 0; i<488; i++) // 
  {
    payloadGlobal[i] = 66;
    receivedpayloadGlobal[i]=66;
  }
  
  int lenPayload = BuildPacket(payloadGlobal, config);
  //should be 340 debugSerialPort->print("lenPayload");debugSerialPort->println(lenPayload);
  Serial.print("self build ");
  Serial.println(lenPayload);
  /*
  Serial.print("[");
  for(int i = 0; i<476; i++)
    {
      Serial.print(payloadGlobal[i]);
      Serial.print(", ");
    }

  Serial.print("]");*/
  
  int ret_pack = packet_send(payloadGlobal, lenPayload, num);
  Serial.print("Packed and sent!");
  Serial.println(ret_pack);
  

  lenPayload = ReceiveUartMessage2(receivedpayloadGlobal, num);

  Serial.print("received set response");
  Serial.print(receivedpayloadGlobal[0]);
  Serial.print(" ");
  Serial.print(lenPayload);
  
  if (lenPayload > 1 || lenPayload == 0 || payloadGlobal[0] != 13) {
    Serial.println("wrong answer from Vesc. should be lenPayload = 1 and payload[0] == 13 (COMM_SET_MCCONF)");
    //wrong answer from Vesc. should be lenPayload = 1 and payload[0] == 13 (COMM_SET_MCCONF)
    return false;
  }
  else
  {
    return true; //setting of mc-config was sucessful
  }
}
bool VescUartSet(mc_configuration& config) {
  return VescUartSet(config, 0);
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

  payload[index++] = COMM_SET_POS;
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


void SerialPrint0(uint8_t* data, int len) 
{

  //  debugSerialPort->print("Data to display: "); debugSerialPort->println(sizeof(data));

  for (int i = 0; i <= len; i++)
  {
    Serial.print(data[i]);
    Serial.print(", ");
  }
  Serial.println("");
}

void SerialPrint(uint8_t* data, int len) 
{

  //  debugSerialPort->print("Data to display: "); debugSerialPort->println(sizeof(data));

  for (int i = 0; i <= len; i++)
  {
    if(debugSerialPort!=NULL) debugSerialPort->print(data[i]);
    if(debugSerialPort!=NULL) debugSerialPort->print(", ");
  }
  if(debugSerialPort!=NULL) debugSerialPort->println("");
}


void SerialPrint(const bldcMeasure& values) 
{
  if(debugSerialPort!=NULL){
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
}


void Serial0Print(const mc_configuration& config) 
{
 
    Serial.print("MCCONF_SIGNATURE: "); Serial.println(MCCONF_SIGNATURE);
    Serial.print("pwm_mode: "); Serial.println(config.pwm_mode);
    Serial.print("comm_mode: "); Serial.println(config.comm_mode);
    Serial.print("motor_type: "); Serial.println(config.motor_type);
    Serial.print("sensor_mode: "); Serial.println(config.sensor_mode); 
    Serial.print("l_current_max: "); Serial.println(config.l_current_max);
    Serial.print("l_current_min: "); Serial.println(config.l_current_min);
    Serial.print("l_in_current_max: "); Serial.println(config.l_in_current_max);
    Serial.print("l_in_current_min: "); Serial.println(config.l_in_current_min);
    Serial.print("l_abs_current_max: "); Serial.println(config.l_abs_current_max);
    Serial.print("l_min_erpm: "); Serial.println(config.l_min_erpm);
    Serial.print("l_max_erpm: "); Serial.println(config.l_max_erpm);
    Serial.print("l_max_erpm_fbrake: "); Serial.println(config.l_max_erpm_fbrake);
    Serial.print("l_max_erpm_fbrake_cc: "); Serial.println(config.l_max_erpm_fbrake_cc);
    Serial.print("l_min_vin: "); Serial.println(config.l_min_vin);
    Serial.print("l_max_vin: "); Serial.println(config.l_max_vin);
    Serial.print("l_battery_cut_start: "); Serial.println(config.l_battery_cut_start);
    Serial.print("l_battery_cut_end:      "); Serial.println(config.l_battery_cut_end);
    Serial.print("l_slow_abs_current:      "); Serial.println(config.l_slow_abs_current);
    
    Serial.print("l_temp_fet_start:      "); Serial.println(config.l_temp_fet_start);
    Serial.print("l_temp_fet_end:      "); Serial.println(config.l_temp_fet_end);
    Serial.print("l_temp_motor_start:      "); Serial.println(config.l_temp_motor_start);
    Serial.print("l_temp_motor_end:      "); Serial.println(config.l_temp_motor_end);
    Serial.print("l_temp_accel_dec:      "); Serial.println(config.l_temp_accel_dec);
    Serial.print("l_min_duty:      "); Serial.println(config.l_min_duty);
    Serial.print("l_max_duty:      "); Serial.println(config.l_max_duty);
    Serial.print("l_watt_max:      "); Serial.println(config.l_watt_max);
    Serial.print("l_watt_min:      "); Serial.println(config.l_watt_min);
    
    Serial.print("l_current_max_scale:      "); Serial.println(config.l_current_max_scale);
    Serial.print("l_current_min_scale:      "); Serial.println(config.l_current_min_scale);

    Serial.print("lo_current_max:      "); Serial.println(config.lo_current_max);
    Serial.print("lo_current_min:      "); Serial.println(config.lo_current_min);
    Serial.print("lo_in_current_max:      "); Serial.println(config.lo_in_current_max);
    Serial.print("lo_in_current_min:      "); Serial.println(config.lo_in_current_min);
    Serial.print("lo_current_motor_max_now:      "); Serial.println(config.lo_current_motor_max_now);
    Serial.print("lo_current_motor_min_now:      "); Serial.println(config.lo_current_motor_min_now);
    Serial.print("sl_min_erpm:      "); Serial.println(config.sl_min_erpm);
    Serial.print("sl_min_erpm_cycle_int_limit:      "); Serial.println(config.sl_min_erpm_cycle_int_limit);
    Serial.print("sl_max_fullbreak_current_dir_change:      "); Serial.println(config.sl_max_fullbreak_current_dir_change);
    Serial.print("sl_cycle_int_limit:      "); Serial.println(config.sl_cycle_int_limit);
    Serial.print("sl_phase_advance_at_br:      "); Serial.println(config.sl_phase_advance_at_br);
    Serial.print("sl_cycle_int_rpm_br:      "); Serial.println(config.sl_cycle_int_rpm_br);
    Serial.print("sl_bemf_coupling_k:      "); Serial.println(config.sl_bemf_coupling_k);
    //Serial.print("hall_table:      "); Serial.println(config.hall_table);
    Serial.print("hall_sl_erpm:      "); Serial.println(config.hall_sl_erpm);
    Serial.print("foc_current_kp:      "); Serial.println(config.foc_current_kp);
    Serial.print("foc_current_ki:      "); Serial.println(config.foc_current_ki);
    Serial.print("foc_f_zv:      "); Serial.println(config.foc_f_zv);
    Serial.print("foc_dt_us:      "); Serial.println(config.foc_dt_us);
    Serial.print("foc_encoder_inverted:      "); Serial.println(config.foc_encoder_inverted);
    Serial.print("foc_encoder_offset:      "); Serial.println(config.foc_encoder_offset);
    Serial.print("foc_encoder_ratio:      "); Serial.println(config.foc_encoder_ratio);
    Serial.print("foc_sensor_mode:      "); Serial.println(config.foc_sensor_mode);
    Serial.print("foc_pll_kp:      "); Serial.println(config.foc_pll_kp);
    Serial.print("foc_pll_ki:      "); Serial.println(config.foc_pll_ki);
    Serial.print("foc_motor_l:      "); Serial.println(config.foc_motor_l);
    Serial.print("foc_motor_r:      "); Serial.println(config.foc_motor_r);
    Serial.print("foc_motor_flux_linkage:      "); Serial.println(config.foc_motor_flux_linkage);
    Serial.print("foc_observer_gain:      "); Serial.println(config.foc_observer_gain);
    Serial.print("foc_observer_gain_slow:      "); Serial.println(config.foc_observer_gain_slow);
    Serial.print("foc_duty_dowmramp_kp:      "); Serial.println(config.foc_duty_dowmramp_kp);
    Serial.print("foc_duty_dowmramp_ki:      "); Serial.println(config.foc_duty_dowmramp_ki);
    Serial.print("foc_openloop_rpm:      "); Serial.println(config.foc_openloop_rpm);
    Serial.print("foc_sl_openloop_hyst:      "); Serial.println(config.foc_sl_openloop_hyst);
    Serial.print("foc_sl_openloop_time:      "); Serial.println(config.foc_sl_openloop_time);
    Serial.print("foc_fw_duty_start:      "); Serial.println(config.foc_fw_duty_start);
    Serial.print("foc_fw_q_current_factor:      "); Serial.println(config.foc_fw_q_current_factor);
    Serial.print("foc_fw_current_max:      "); Serial.println(config.foc_fw_current_max);
    Serial.print("foc_sl_erpm:      "); Serial.println(config.foc_sl_erpm);
    Serial.print("foc_sample_v0_v7:      "); Serial.println(config.foc_sample_v0_v7);
    Serial.print("foc_sample_high_current:      "); Serial.println(config.foc_sample_high_current);
    Serial.print("foc_sat_comp:      "); Serial.println(config.foc_sat_comp);
    Serial.print("foc_temp_comp:      "); Serial.println(config.foc_temp_comp);
    Serial.print("foc_temp_comp_base_temp:      "); Serial.println(config.foc_temp_comp_base_temp);
    Serial.print("foc_current_filter_const:      "); Serial.println(config.foc_current_filter_const);
    
    // GPDrive
	/*int gpd_buffer_notify_left;
	int gpd_buffer_interpol;
	float gpd_current_filter_const;
	float gpd_current_kp;
	float gpd_current_ki;*/
    
    Serial.print("s_pid_kp:      "); Serial.println(config.s_pid_kp);
    Serial.print("s_pid_ki:      "); Serial.println(config.s_pid_ki);
    Serial.print("s_pid_kd:      "); Serial.println(config.s_pid_kd);
    Serial.print("s_pid_kd_filter:      "); Serial.println(config.s_pid_kd_filter);
    Serial.print("s_pid_min_erpm:      "); Serial.println(config.s_pid_min_erpm);
    Serial.print("s_pid_allow_braking:      "); Serial.println(config.s_pid_allow_braking);
    Serial.print("p_pid_kp:      "); Serial.println(config.p_pid_kp);
    Serial.print("p_pid_ki:      "); Serial.println(config.p_pid_ki);
    Serial.print("p_pid_kd:      "); Serial.println(config.p_pid_kd);
    Serial.print("p_pid_kd_filter:      "); Serial.println(config.p_pid_kd_filter);
    Serial.print("p_pid_ang_div:      "); Serial.println(config.p_pid_ang_div);
    Serial.print("cc_startup_boost_duty:      "); Serial.println(config.cc_startup_boost_duty);
    Serial.print("cc_min_current:      "); Serial.println(config.cc_min_current);
    Serial.print("cc_gain:      "); Serial.println(config.cc_gain);
    Serial.print("cc_ramp_step_max:      "); Serial.println(config.cc_ramp_step_max);
    Serial.print("m_fault_stop_time_ms:      "); Serial.println(config.m_fault_stop_time_ms);
    Serial.print("m_duty_ramp_step:      "); Serial.println(config.m_duty_ramp_step);
    Serial.print("m_current_backoff_gain:      "); Serial.println(config.m_current_backoff_gain);
    Serial.print("m_encoder_counts:      "); Serial.println(config.m_encoder_counts);
    Serial.print("m_sensor_port_mode:      "); Serial.println(config.m_sensor_port_mode);
    Serial.print("m_invert_direction:      "); Serial.println(config.m_invert_direction);
    Serial.print("m_drv8301_oc_mode:      "); Serial.println(config.m_drv8301_oc_mode);
    Serial.print("m_drv8301_oc_adj:      "); Serial.println(config.m_drv8301_oc_adj);
    Serial.print("m_bldc_f_sw_min:      "); Serial.println(config.m_bldc_f_sw_min);
    Serial.print("m_bldc_f_sw_max:      "); Serial.println(config.m_bldc_f_sw_max);
    Serial.print("m_dc_f_sw:      "); Serial.println(config.m_dc_f_sw);
    Serial.print("m_ntc_motor_beta:      "); Serial.println(config.m_ntc_motor_beta);
    Serial.print("m_out_aux_mode:      "); Serial.println(config.m_out_aux_mode);
    
    Serial.print("si_motor_poles:      "); Serial.println(config.si_motor_poles);
    Serial.print("si_gear_ratio:      "); Serial.println(config.si_gear_ratio);
    Serial.print("si_wheel_diameter:      "); Serial.println(config.si_wheel_diameter);
    Serial.print("si_battery_type:      "); Serial.println(config.si_battery_type);
    Serial.print("si_battery_cells:      "); Serial.println(config.si_battery_cells);
    Serial.print("si_battery_ah:      "); Serial.println(config.si_battery_ah);  
  
}
