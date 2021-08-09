  #include <SPI.h>
  #include <PN532_SPI.h>
  #include "PN532.h"

  PN532_SPI pn532spi(SPI, 15);
  PN532 nfc(pn532spi);

  uint8_t uidbufferA[16]={0};
  uint8_t uidbufferB[16]={0};
  uint8_t statuebuff[3]={0,2,0}; 
  /*[0] for loop statue buffer,
       0 for initial,
       1 for standby,
       2 for done,wait for taking card away, 
    [1] for uidbuffer changing buffer.
       0 is in uidbufferB,
       1 is in uidbufferA,
  */
  //boolean test;
  uint8_t ledPin = 2;
  
void setup() {
  Serial.begin(115200);
  pinMode(13, INPUT); // read or write;1 is write,0 is read
  pinMode(12, INPUT); // UIDbuffer A or B;1 is A,0 is B 
  pinMode(ledPin,OUTPUT);  

  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1); // halt
  }
  
  nfc.SAMConfig();
  Serial.println("Waiting for an ISO14443A Card ...");
}


void loop() {
  uint8_t ReadorWrite = digitalRead(13); // read or write;1 is write,0 is read
  uint8_t UIDbufAorB = digitalRead(12); // UIDbuffer A or B;1 is A,0 is B 

  if(statuebuff[2]!=ReadorWrite){
    statuebuff[0]=0;
  }
  
  if(ReadorWrite==0) // Read mode
  {
    switch(statuebuff[0]){
      case 2:
        break;
      case 0:
        Serial.println("Reading mode."); 
        TheChoosingBuf(UIDbufAorB);
        nowisAorBinLED(UIDbufAorB); 
        statuebuff[0]=1;
        break;
      case 1:
        if (UIDbufAorB != statuebuff[1]){
          TheChoosingBuf(UIDbufAorB);
          nowisAorBinLED(UIDbufAorB); 
        }
        break;
    }
       
    if (UIDbufAorB==1){ // put data in uidbufA
      GetBlock0AndGetUID(uidbufferA);
    }
    else{ // put data in uidbufB
      GetBlock0AndGetUID(uidbufferB);
    }
    
  }
  else // Write mode
  {
    
    switch(statuebuff[0]){
      case 2:
        break;
      case 0:
        Serial.println("Writeing mode.");
        TheChoosingBuf(UIDbufAorB);
        nowisAorBinLED(UIDbufAorB); 
        statuebuff[0]=1;
        break;
      case 1:
        if (UIDbufAorB != statuebuff[1]){
          TheChoosingBuf(UIDbufAorB);
          nowisAorBinLED(UIDbufAorB); 
        }
        break;
    }
    

    if (UIDbufAorB==1){ // write uidbufA into whitecard
      writeToBlock0(uidbufferA);
    }
    else{ // write uidbufB into whitecard
      writeToBlock0(uidbufferB);
    }
  }
  
  statuebuff[1] = UIDbufAorB;
  statuebuff[2] = ReadorWrite;

  /*if(test){    
      Serial.print("A  ");
      
      Serial.println("");
      Serial.print("B  ");
      for(int i=0;i<16;i++){
        Serial.print(uidbufferB[i],HEX);
        Serial.print(" ");
      } 
    Serial.println("");
    test=false;    
  }*/
}


void GetBlock0AndGetUID(uint8_t *uidbuffer){
  uint8_t success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
  Serial.println(success);
  if(success==0 && statuebuff[0]==2)
  {
    statuebuff[0]=1;
    digitalWrite(ledPin, LOW);
    delay(100);
  }
  if (success && statuebuff[0]!=2) {
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
        delay(1000);
        for (uint8_t i=0 ;i<16;i++){
          uidbuffer[i]=data[i],HEX;
        }

        digitalWrite(ledPin, HIGH);
        statuebuff[0]=2; // change statue to 2
        
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

  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
  Serial.println(success);
  if(success==0 && statuebuff[0]==2)
  {
    statuebuff[0]=1;
    digitalWrite(ledPin, LOW);
    delay(100);
  }
  if (success && statuebuff[0]!=2) {  
    //uint8_t changeuid[]= { 0xCB, 0xA6, 0xBC, 0x77};
    // FF 00 00 00 08 D4 08 63 02 00 63 03 00
    // Defines the transmission data rate and framing during transmission.
    // Disables CRC,106kbits, 14443A
    result=nfc.writeRegister(0x6302,0);
    if(!result){
      Serial.println("Write 6302 00 Failed.");
      return;
    } 
    delay(100);
    Serial.println("Write 6302 00 Success.");  
  
    // Defines the transmission data rate and framing during reciving.
    // Disables CRC,106kbits, 14443A
    result=nfc.writeRegister(0x6303,0);
    if(!result){
      Serial.println("Write 6303 00 Failed.");
      return;
    }
    delay(100);
    Serial.println("Write 6303 00 Success.");
  
    /*
     * Send MIFARE Halt command, since CRC is disabled during transmission, CRC of 57 CD is sent in the command
     */
    // FF 00 00 00 06 D4 42 50 00 57 CD
    uint8_t cmd[] = {0x50,0,0x57,0xCD};
    result=nfc.inCommunicateThru(cmd,4);
    if(!result){
      Serial.println("50 00 57 CD Failed.");
      return;
    }
    delay(100);
    Serial.println("50 00 57 CD Success.");

    // FF 00 00 00 05 D4 08 63 3D 07
    // Adjustments for bit oriented frames
    // 07 means only 7 bits of last bytes is transmitted
    result=nfc.writeRegister(0x633D,7);
    if(!result){
      Serial.println("Write 633D 07 Failed.");
      return;
    }
    delay(100);
    Serial.println("Write 633D 07 Success.");
  
    // FF 00 00 00 03 D4 42 40
    // BackDoor Command
    cmd[0] = 0x40;
    result=nfc.inCommunicateThru(cmd,1);
    if(!result){
      Serial.println("40 Failed.");
      return;
    }
    delay(100);
    Serial.println("40 Success.");
  
    // FF 00 00 00 05 D4 08 63 3D 00
    // 00 means all 8 bits of last type is transmitted
    result=nfc.writeRegister(0x633D,0);
    if(!result){
      Serial.println("Write 633D 00 Failed.");
      return;
    }
    delay(100);
    Serial.println("Write 633D 00 Success.");

    // FF 00 00 00 03 D4 42 43
    // BackDoor Command
    cmd[0] = 0x43;
    result=nfc.inCommunicateThru(cmd,1);
    if(!result){
      Serial.println("43 Failed.");
      return;
    }
    delay(100);
    Serial.println("43 Success.");  

    // FF 00 00 00 08 D4 08 63 02 80 63 03 80
    // Enables CRC,106kbits, 14443A/Mifare Framing for transmission
    result=nfc.writeRegister(0x6302,0x80);
    if(!result){
      Serial.println("Write 6302 80 Failed.");
      return;
    }
    delay(100);
    Serial.println("Write 6302 80 Success.");  
  
    // Enables CRC,106kbits, 14443A/Mifare Framing for reception
    result=nfc.writeRegister(0x6303,0x80);
    if(!result){
      Serial.println("Write 6303 80 Failed.");
      return;
    }
    delay(100);
    Serial.println("Write 6303 80 Success.");

    // Write to Block 0
    //uint8_t data[] = {0xCB,0xA6,0xBC,0x77,0xA6,0x08,0x04,0,0x99,0x81,0x85,0x65,0x78,0x82,0x65,0x89};
    result=nfc.mifareclassic_WriteDataBlock(0,data);
    if(!result){
      Serial.println("Write Failed.");
      return;
    }
    delay(100);
    Serial.println("Write Success.");
    statuebuff[0]=2;
    digitalWrite(ledPin,HIGH);
  }
}

void nowisAorBinLED(uint8_t buf){
  digitalWrite(ledPin, HIGH);
  delay(250);
  digitalWrite(ledPin, LOW);
  delay(250);
  if(buf==0){
    digitalWrite(ledPin, HIGH);
    delay(250);
    digitalWrite(ledPin, LOW);
    delay(250);
  }
}

void TheChoosingBuf(uint8_t buf){
  if (buf==1){ // put data in uidbufA
    Serial.print("Using buffer A : ");
    printbufferdata(uidbufferA);
  }
  else{ // put data in uidbufB
    Serial.print("Using buffer B : ");
    printbufferdata(uidbufferB);
  }
}

void printbufferdata(uint8_t *data){
  for(int i=0;i<16;i++){
    Serial.print(data[i],HEX);
    Serial.print(" ");
  } 
  Serial.println("");
}
