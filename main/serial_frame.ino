#define ESCAPE_CHAR 0x05
#define FRAME_START 0x0A
#define FRAME_END 0x0B

uint32_t last_0A0 = 0;
uint32_t last_0A1 = 0;
uint32_t last_0A2 = 0;
uint32_t last_0A5 = 0;
uint32_t last_0A7 = 0;
uint32_t last_0AA = 0;
uint32_t last_0AB = 0;

void send_byte_with_escape(uint8_t b)
{
  if(b == ESCAPE_CHAR)
  {
    Serial2.write(ESCAPE_CHAR);
  }
  Serial2.write(b);
}

void send_frame(uint8_t* buf, uint8_t len)
{
  Serial2.write(ESCAPE_CHAR);
  Serial2.write(FRAME_START);

  for(uint8_t i = 0; i < len; i++)
  {
    send_byte_with_escape(buf[i]);
  }

  Serial2.write(ESCAPE_CHAR);
  Serial2.write(FRAME_END);
}

void send_can_wireless(CAN_message_t msg)
{
  //trying to slow down motor controller messages
  uint32_t t = millis();
  switch(msg.id){
    case 0x0A0: 
      if((t-last_0A0) < 1000)
        return;
      else
        last_0A0 = t; 
      break;
    case 0x0A1: 
      if((t-last_0A1) < 1000)
        return;
      else
        last_0A1 = t; 
      break;
    case 0x0A2: 
      if((t-last_0A2) < 1000)
        return;
      else
        last_0A2 = t; 
      break;
    case 0x0A5: 
      if((t-last_0A5) < 1000)
        return;
      else
        last_0A5 = t; 
      break;
    case 0x0A7: 
      if((t-last_0A7) < 1000)
        return;
      else
        last_0A7 = t; 
      break;
    case 0x0AA: 
      if((t-last_0AA) < 1000)
        return;
      else
        last_0AA = t; 
      break;
    case 0x0AB: 
      if((t-last_0AB) < 1000)
        return;
      else
        last_0AB = t; 
      break;
    case 0x0C0: 
      return; 
      break;
  }

  uint8_t frame[10] = {};
  frame[0] = msg.id&0xFF;
  frame[1] = (msg.id>>8)&0xFF;
  for(uint8_t i = 0; i < msg.len; i++)
  {
    frame[i+2] = msg.buf[i]; 
  }

  send_frame(frame, msg.len+2); 
}


/*void send_can_wirless(CAN_message_t msg)
{
  uint8_t msg_vcu[8] = {};
  msg_vcu[0] = MSG_VCU_ID;

  uint8_t msg_bms[8] = {};
  msg_bms[0] = MSG_BMS_ID;
  
  switch(msg.id)
  {
    case 0x766:  // VCU status
      msg_vcu[1] = msg.buf[4];  // vcu state
      break;
    case 0x0C0:  // torque request
      msg_vcu[2] = msg.buf[0]; //torque low
      msg_vcu[3] = msg.buf[1]; //torque high
      send_frame(&msg_vcu, 4);
      break;
    case 0x380:  // BMS status
      msg_bms[1] = msg.buf[1];
      msg_bms[2] = msg.buf
      break;
    /*
    case 0x0A0: // MC temps 1
      temp_phaseA = ((msg.buf[1] << 8) + msg.buf[0])/10;
      temp_phaseB = ((msg.buf[3] << 8) + msg.buf[2])/10;
      temp_phaseC = ((msg.buf[5] << 8) + msg.buf[4])/10;
      temp_gate_driver = ((msg.buf[7] << 8) + msg.buf[6])/10;
      Serial2.printf("Phase A: %iC, Phase B: %iC, Phase C: %iC, Gate Driver: %iC\n", temp_phaseA, temp_phaseB, temp_phaseC, temp_gate_driver);
      break;
    */
    /*  
    case 0x0A2: // MC temps 3
      temp_phaseA = ((msg.buf[5] << 8) + msg.buf[4])/10;
      Serial2.printf("Motor Temp: %iC\n", temp_phaseA);
    */
    /*case 0x0AA:  // MC internal states
      if(msg.buf[0] == last_mc_state)
      {
        return;
      }
      last_mc_state = msg.buf[0];
      
      Serial2.print("MC State: ");
      switch(msg.buf[0])
      {
        case 0: 
          Serial2.print("Start\n");
          break;
        case 1: 
          Serial2.print("Precharge Init\n");
          break;
        case 2: 
          Serial2.print("Precharge Active\n");
          break;
        case 3:
          Serial2.print("Precharge Complete\n");
          break;
        case 4: 
          Serial2.print("Wait\n");
          break;
        case 5: 
          Serial2.print("Ready\n");
          break;
        case 6: 
          Serial2.print("Motor Running\n");
          break;
        case 7: 
          Serial2.print("Blink Fault Code\n");
          break;
        case 14: 
          Serial2.print("Shutdown in progress\n");
          break;
        case 15: 
          Serial2.print("Recycle power\n");
          break;
        default: 
          Serial2.print("Invalid state!\n");
          break;
      }
      break;
    case 0x400:
      Serial2.printf("Temps: MC In/Out: %uC/%uC, MTR In/Out: %uC/%uC\n", 
                    msg.buf[0],msg.buf[1],msg.buf[2],msg.buf[3]);
      Serial2.printf("Pressures: MC In/Out: %upsi/%upsi, MTR In/Out: %upsi/%upsi\n", 
                    msg.buf[4],msg.buf[5],msg.buf[6],msg.buf[7]);
                    
      break;
    
    case 0x401:
      Serial2.printf("Air Temps: MC In/Out: %uC/%uC, MTR In/Out: %uC/%uC\n", 
                    msg.buf[0],msg.buf[1],msg.buf[2],msg.buf[3]);
                    
      break;
    case 0x500: // front wheel speed
      torque = ((msg.buf[0] << 8) + msg.buf[1]);
      Serial2.printf("FWS: %irpm\n", torque);
      break;
    case 0x0A5: // MC motor position
      torque = ((msg.buf[3] << 8) + msg.buf[2]);
      Serial2.printf("RWS: %irpm\n", torque);
      break;
  }
}*/
