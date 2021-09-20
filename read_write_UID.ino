  #include <SPI.h>
  #include <PN532_SPI.h>
  #include "PN532.h"
  #include<EEPROM.h>

  PN532_SPI pn532spi(SPI, 15);
  PN532 nfc(pn532spi);

  uint8_t uidbuffer0[16]={0};
  uint8_t uidbuffer1[16]={0};
  uint8_t uidbuffer2[16]={0};
  uint8_t uidbuffer3[16]={0};
  uint8_t la=27;
  uint8_t lb=25;
  uint8_t lc=33;
  uint8_t ld=32;
  uint8_t butpin=12;
  uint8_t modepin=13;
  uint8_t total = 64;
  uint8_t ReadorWrite=0; //0 write 1 read
  uint8_t statebuff[3]={0,0,0}; //前一個動作的狀態 用來檢視現在變了哪些
  /*[0] for card state buffer,
       0 for initial the Variable,
       1 for standby,
       2 for done,wait for taking card away, 
    [1] for uidbuffer changing buffer.
       0 is in uidbuffer0,
       1 is in uidbuffer1,
       2 is in uidbuffer2,
       3 is in uidbuffer3,
    [2] for Mode changing buffer.
  */
  //boolean test;
  uint8_t ledPin = 2;
  
void setup() {
  Serial.begin(115200);
  pinMode(modepin, INPUT_PULLUP); // read or write;LOW is write,HIGH is read
  pinMode(butpin, INPUT_PULLUP); // button,for change UIDbuffer
  pinMode(ledPin,OUTPUT);
  pinMode(la, OUTPUT);
  pinMode(lb, OUTPUT);
  pinMode(lc, OUTPUT);
  pinMode(ld, OUTPUT);  

  nfc.begin();
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1); // halt
  }
  nfc.SAMConfig();

  EEPROM.begin(total);
  for(uint8_t addr=0;addr<total;addr++)//read
  {
    uint8_t data=EEPROM.read(addr);
    if(addr<16){
      uidbuffer0[addr] = data ;//將讀卡完成的資料輸入進EEPROM
    }
    else if(addr>=16&&addr<32){
      uidbuffer1[addr-16] =data;//將讀卡完成的資料輸入進EEPROM
    }
    else if(addr>=32&&addr<48){
      uidbuffer2[addr-32]=data ;//將讀卡完成的資料輸入進EEPROM  
    }
    else{
      uidbuffer3[addr-48]=data ;
    }

  }
  
  Serial.println("Waiting for an ISO14443A Card ...");
}


void loop() {
  uint8_t pin_ReadorWrite = digitalRead(modepin);  // read or write;1 is write,0 is read
  uint8_t pin_UIDbuf = digitalRead(butpin);   // UIDbuffer A or B;0 is A,1 is B 

  if(pin_UIDbuf == LOW){        //按了卡槽按鈕
    statebuff[1]++;
    if(statebuff[1]==4){statebuff[1]=0;}
    Serial.println(statebuff[1]);
    delay(200);
    while(pin_UIDbuf == LOW){
      pin_UIDbuf = digitalRead(butpin);
    }
    PrintTheChoosingBuf(statebuff[1]);
  }

  if(pin_ReadorWrite == LOW){        //按了mode按鈕
    ReadorWrite++;
    if(ReadorWrite==2){ReadorWrite=0;}
    delay(200);
    while(pin_ReadorWrite == LOW){
      pin_ReadorWrite = digitalRead(butpin);
    }
  }

  if(statebuff[2]!=ReadorWrite){ // If Mode change,initial the Variable 
    statebuff[0]=0;
  }
  
  if(ReadorWrite==HIGH) // Read mode
  {
    switch(statebuff[0]){
      case 2:
        WritetoROM();
        break;
      case 0:
        Serial.println("Reading mode.");
        PrintTheChoosingBuf(statebuff[1]);
        statebuff[0]=1;
        break;
      case 1:
        
        break;
    }
    switch(statebuff[1]){
      case 0:
        GetBlock0AndGetUID(uidbuffer0);
        break;
      case 1:
        GetBlock0AndGetUID(uidbuffer1);
        break;
      case 2:
        GetBlock0AndGetUID(uidbuffer2);
        break;
      case 3:
        GetBlock0AndGetUID(uidbuffer3);
        break;
    }
    
    /*   
    if (pin_UIDbufAorB==0){ // put data in uidbufA
      GetBlock0AndGetUID(uidbuffer0);
    }
    else{ // put data in uidbufB
      GetBlock0AndGetUID(uidbuffer1);
    }
    */
  }
  else // Write mode
  {
    
    switch(statebuff[0]){
      case 2:
        break;
      case 0:
        Serial.println("Writeing mode.");
        PrintTheChoosingBuf(statebuff[1]);
        statebuff[0]=1;
  
        break;
      case 1:
        break;
    }

    switch(statebuff[1]){
      case 0:
        writeToBlock0(uidbuffer0);
        break;
      case 1:
        writeToBlock0(uidbuffer1);
        break;
      case 2:
        writeToBlock0(uidbuffer2);
        break;
      case 3:
        writeToBlock0(uidbuffer3);
        break;
    }
    /*
    if (pin_UIDbufAorB==0){ // write uidbufA into whitecard
      writeToBlock0(uidbuffer0);
    }
    else{ // write uidbufB into whitecard
      writeToBlock0(uidbuffer1);
    }
    */
  }
  
  statebuff[2] = ReadorWrite;  //Update the Mode statu to buff

}


void GetBlock0AndGetUID(uint8_t *uidbuffer){
  uint8_t success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength,100);

  if(success==0 && statebuff[0]==2)
  {
    statebuff[0]=1;

    delay(10);
  }
  if (success && statebuff[0]!=2) {
    // Display some basic information about the card
    Serial.println("Found an ISO14443A card");
    Serial.print("  UID Length: ");
    Serial.print(uidLength, DEC);
    Serial.println(" bytes");
    Serial.print("  UID Value: ");
    nfc.PrintHex(uid, uidLength);
    Serial.println("");
    
    //Serial.println("Seems to be a Mifare Classic card (4 byte UID)");
    //Serial.println("Trying to authenticate block 4 with default KEYA value");
    uint8_t keya[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
 
    success = nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 0, 0, keya);
      
    if (success)
    {
      //Serial.println("Sector 1 (Blocks 4..7) has been authenticated");
      uint8_t data[16];
      
      success = nfc.mifareclassic_ReadDataBlock(0, data);
    
      if (success)
      {
        Serial.println("Reading Block 0:");
        nfc.PrintHexChar(data, 16);
        Serial.println("");
        //test=true;
        delay(10);
        for (uint8_t i=0 ;i<16;i++){
          uidbuffer[i]=data[i],HEX;
        }


        statebuff[0]=2; // change state to 2
        
      }
      else
      {
        Serial.println("Ooops ... unable to read the requested block.  Try another key?");
      }
    }
    else
    {
      Serial.println("Ooops ... authentication failed: Try another key?");
    }
  }
}

void writeToBlock0(uint8_t data[]){
  int result;
  uint8_t success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
  

  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength,100);
  //Serial.print(success);Serial.print(" ");Serial.println(statebuff[0]);
  if(success==0 && statebuff[0]==2)
  {
    statebuff[0]=1;

    delay(10);
  }
  if (success && statebuff[0]!=2) {  
    //uint8_t changeuid[]= { 0xCB, 0xA6, 0xBC, 0x77};
    // FF 00 00 00 08 D4 08 63 02 00 63 03 00
    // Defines the transmission data rate and framing during transmission.
    // Disables CRC,106kbits, 14443A
    result=nfc.writeRegister(0x6302,0);
    if(!result){
      Serial.println("Write 6302 00 Failed.");
      return;
    } 
    delay(10);
    Serial.println("Write 6302 00 Success.");  
  
    // Defines the transmission data rate and framing during reciving.
    // Disables CRC,106kbits, 14443A
    result=nfc.writeRegister(0x6303,0);
    if(!result){
      Serial.println("Write 6303 00 Failed.");
      return;
    }
    delay(10);
    Serial.println("Write 6303 00 Success.");
  
    
    // Send MIFARE Halt command, since CRC is disabled during transmission, CRC of 57 CD is sent in the command
     
    // FF 00 00 00 06 D4 42 50 00 57 CD
    uint8_t cmd[] = {0x50,0,0x57,0xCD};
    result=nfc.inCommunicateThru(cmd,4);
    if(!result){
      Serial.println("50 00 57 CD Failed.");
      return;
    }
    delay(10);
    Serial.println("50 00 57 CD Success.");

    // FF 00 00 00 05 D4 08 63 3D 07
    // Adjustments for bit oriented frames
    // 07 means only 7 bits of last bytes is transmitted
    result=nfc.writeRegister(0x633D,7);
    if(!result){
      Serial.println("Write 633D 07 Failed.");
      return;
    }
    delay(10);
    Serial.println("Write 633D 07 Success.");
  
    // FF 00 00 00 03 D4 42 40
    // BackDoor Command
    cmd[0] = 0x40;
    result=nfc.inCommunicateThru(cmd,1);
    if(!result){
      Serial.println("40 Failed.");
      return;
    }
    delay(10);
    Serial.println("40 Success.");
  
    // FF 00 00 00 05 D4 08 63 3D 00
    // 00 means all 8 bits of last type is transmitted
    result=nfc.writeRegister(0x633D,0);
    if(!result){
      Serial.println("Write 633D 00 Failed.");
      return;
    }
    delay(10);
    Serial.println("Write 633D 00 Success.");

    // FF 00 00 00 03 D4 42 43
    // BackDoor Command
    cmd[0] = 0x43;
    result=nfc.inCommunicateThru(cmd,1);
    if(!result){
      Serial.println("43 Failed.");
      return;
    }
    delay(10);
    Serial.println("43 Success.");  

    // FF 00 00 00 08 D4 08 63 02 80 63 03 80
    // Enables CRC,106kbits, 14443A/Mifare Framing for transmission
    result=nfc.writeRegister(0x6302,0x80);
    if(!result){
      Serial.println("Write 6302 80 Failed.");
      return;
    }
    delay(10);
    Serial.println("Write 6302 80 Success.");  
  
    // Enables CRC,106kbits, 14443A/Mifare Framing for reception
    result=nfc.writeRegister(0x6303,0x80);
    if(!result){
      Serial.println("Write 6303 80 Failed.");
      return;
    }
    delay(10);
    Serial.println("Write 6303 80 Success.");

    // Write to Block 0
    //uint8_t data[] = {0xCB,0xA6,0xBC,0x77,0xA6,0x08,0x04,0,0x99,0x81,0x85,0x65,0x78,0x82,0x65,0x89};
    result=nfc.mifareclassic_WriteDataBlock(0,data);
    if(!result){
      Serial.println("Write Failed.");
      return;
    }

    Serial.println("Write Success.");
    statebuff[0]=2;

  }
}



void PrintTheChoosingBuf(uint8_t buf){
  switch(buf){ // put data in uidbufA
    case 0:
      digitalWrite(la, HIGH);
      digitalWrite(lb, LOW);
      digitalWrite(lc, LOW);
      digitalWrite(ld, LOW);
      Serial.print("Using buffer 0 : ");
      printbufferdata(uidbuffer0);
      break;
    case 1:
      digitalWrite(la, LOW);
      digitalWrite(lb, HIGH);
      digitalWrite(lc, LOW);
      digitalWrite(ld, LOW);
      Serial.print("Using buffer 1 : ");
      printbufferdata(uidbuffer1);
      break;
    case 2:
      digitalWrite(la, LOW);
      digitalWrite(lb, LOW);
      digitalWrite(lc, HIGH);
      digitalWrite(ld, LOW);
      Serial.print("Using buffer 2 : ");
      printbufferdata(uidbuffer2);
      break;
    case 3:
      digitalWrite(la, LOW);
      digitalWrite(lb, LOW);
      digitalWrite(lc, LOW);
      digitalWrite(ld, HIGH);
      Serial.print("Using buffer 3 : ");
      printbufferdata(uidbuffer3);
      break;
  }

}

void printbufferdata(uint8_t *data){
  for(int i=0;i<16;i++){
    Serial.print(data[i],HEX);
    Serial.print(" ");
  } 
  Serial.println("");
}

void WritetoROM(){
  uint8_t data;
  for(uint8_t addr=0;addr<total;addr++)//write
  {
    if(addr<16){
      data = uidbuffer0[addr];//將讀卡完成的資料輸入進EEPROM
    }
    else if(addr>=16&&addr<32){
      data = uidbuffer1[addr-16];//將讀卡完成的資料輸入進EEPROM
    }
    else if(addr>=32&&addr<48){
      data = uidbuffer2[addr-32];//將讀卡完成的資料輸入進EEPROM  
    }
    else{
      data = uidbuffer3[addr-48];
    }
      EEPROM.write(addr,data);

  }
  EEPROM.commit();//儲存 
}
