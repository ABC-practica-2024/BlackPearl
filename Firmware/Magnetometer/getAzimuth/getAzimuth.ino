
#include <Wire.h>
#include "qmc5883l.h"


boolean error;

QMC5883L qmc5883l(&Wire);

void setup()
{
   Serial.begin(9600);
   while(!Serial);
   
   Wire.begin();
   Wire.setClock(400000); // 400kHz I2C clock

   if (!qmc5883l.begin()) error = true;
   else
   {
      qmc5883l.setConfig({ QMC5883L::Mode::continuous,
                           QMC5883L::OutputDataRate::odr_10hz,
                           QMC5883L::FullScaleRange::rng_8g,
                           QMC5883L::OverSampleRate::osr_512,
                           false                              });

      if (!qmc5883l.writeConfig()) error = true;
      else
      {
         qmc5883l.setCalibration({ {255.120624, 
                                    -449.527586,
                                     9.694871 },
                                  {{   1.274925,   -0.053847,  -0.006444},
                                   {  -0.053847,    1.206629,   0.056813},
                                   {  -0.006444,    0.056813,   1.318348} }});

         error = false;
      }
   }

    
}

void loop()
{
   //if (error) return;
   

   if (qmc5883l.readData())
   {
      Serial.println(qmc5883l.azimuthZUp());
   }
}


void printError(QMC5883L::Error error, char *error_location)
{
   Serial.print("Error: ");
   switch(error)
   {
      case QMC5883L::i2c_buffer_overflow: Serial.print("I2C buffer overflow");          break;
      case QMC5883L::i2c_address_nack:    Serial.print("I2C address not acknowledged"); break;
      case QMC5883L::i2c_data_nack:       Serial.print("I2C data not acknowledged");    break;
      case QMC5883L::i2c_other:           Serial.print("I2C other");                    break;
      case QMC5883L::i2c_request_partial: Serial.print("I2C request partial");          break;
      case QMC5883L::qmc_data_overflow:   Serial.print("QMC5883L data overflow");       break;
   }
   Serial.print(error);
   Serial.print(" in ");
   Serial.println(error_location);
}



