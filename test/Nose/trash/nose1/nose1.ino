#include <Wire.h>
#include <Adafruit_BME280.h>
#include  <MCP342X.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; 
MCP342X myADC;

unsigned long delayTime;
const int clockFrequency = 400000;//I2C bus speed

void setup() {
    Serial.begin(115200);
    unsigned status;
    Wire.setSDA(6);
    Wire.setSCL(7);
    Wire.setClock(clockFrequency);
    Wire.begin();
    status = bme.begin(0x76);
    myADC.configure( MCP342X_MODE_CONTINUOUS |
                MCP342X_CHANNEL_1 |
                MCP342X_SIZE_18BIT |
                MCP342X_GAIN_1X
                );
}


void loop() { 
    static int32_t result;
    static float  temperature;
    static float  barometic_pressure;
    static char  adc_bytes[3];
    static double voltage;

    //差圧センサ関連
    myADC.startConversion();
    myADC.getResult(&result);
    DevideBytes(&result, adc_bytes); //CAN送信用
    ConvertToVoltage(adc_bytes, &voltage); //3つのバイトを電圧に変換
    //BME280関連
    GetBME280Data(&temperature, &barometic_pressure);

    Serial.print("adc_bytes:");
    Serial.print(adc_bytes[0],HEX);
    Serial.print(adc_bytes[1],HEX);
    Serial.println(adc_bytes[2],HEX);
    Serial.print("temperature:");
    Serial.println(temperature,10);
    Serial.print("barometic_pressure:");
    Serial.println(barometic_pressure,10);
    Serial.print("voltage:");
    Serial.println(voltage,10);
    
    delay(5);
}

void GetBME280Data(float* temperature,float* barometic_pressure){
  *temperature=bme.readTemperature();
  *barometic_pressure=bme.readPressure();
}

void DevideBytes(int32_t* _result,char* bytes){
    bytes[2] = (char)(*_result & 0xFF);
    bytes[1] = (char)((*_result >> 8) & 0xFF);
    bytes[0] = (char)((*_result >> 16) & 0xFF); 
}

void ConvertToVoltage(char* bytes,double* voltage){
    double pga=1;
    double lsb=2*2.048/pow(2,18);

    byte msb=(bytes[0]>>6)&0x01;
    uint32_t outputcode=bytes[2]|(bytes[1]<<8)|((bytes[0]*0x01)<<16);
    if(msb==0x00){//正の値
    *voltage=(double)(outputcode)*lsb/pga;
    }else{//負の値
    outputcode=((~outputcode)&0x01FFFF)+1;//2の補数
    *voltage=-(double)(outputcode)*lsb/pga;
    }
}


