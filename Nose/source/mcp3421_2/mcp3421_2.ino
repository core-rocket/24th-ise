
#include  <Wire.h>
#include  <MCP342X.h>

// Instantiate objects used in this project
MCP342X myADC;
double voltage;
void setup() {
  Wire.begin();  // join I2C bus
//   TWBR = 12;  // 400 kHz (maximum)
  
  Serial.begin(9600); // Open serial connection to send info to the host
  Serial.println("Starting up");
  Serial.println("Testing device connection...");
    Serial.println(myADC.testConnection() ? "MCP342X connection successful" : "MCP342X connection failed");
    
  myADC.configure( MCP342X_MODE_CONTINUOUS |
                   MCP342X_CHANNEL_1 |
                   MCP342X_SIZE_18BIT |
                   MCP342X_GAIN_1X
                 );
  delay(300);
  
}  // End of setup()

void loop() {
  static int32_t  result;
  byte bytes[4];
  myADC.startConversion();
  myADC.getResult(&result);
  // Serial.println(result, HEX);
  ConvertToVoltage(&result,&voltage);
  Serial.print("voltage:");
  Serial.println(voltage);
}  // End of loop()

void ConvertToVoltage(int32_t* _result,double* voltage){
  byte bytes[4];
  bytes[0] = (char)(*_result & 0xFF);
  bytes[1] = (char)((*_result >> 8) & 0xFF);
  bytes[2] = (char)((*_result >> 16) & 0xFF); 
  bytes[3] = (char)((*_result >> 24) & 0xFF); 
  Serial.print("bytes[0]:");
  Serial.println(bytes[0],HEX);
  Serial.print("bytes[1]:");
  Serial.println(bytes[1],HEX);
  Serial.print("bytes[2]:");
  Serial.println(bytes[2],HEX);
  Serial.print("bytes[3]:");
  Serial.println(bytes[3],HEX);
  double pga=1;
  double lsb=2*2.048/pow(2,18);
  uint32_t outputcode=bytes[0]+(bytes[1]<<8)+(bytes[2]<<16)+(bytes[3]<<24);
  byte msb=(bytes[3]>>7)&0xFF;
  // if(msb=0x00){
    *voltage=(double)(outputcode)*lsb/pga;
  // }else{
  //   *voltage=-(double)(((~outputcode)+1)&0x3FFF)*lsb/pga;
  // }
}