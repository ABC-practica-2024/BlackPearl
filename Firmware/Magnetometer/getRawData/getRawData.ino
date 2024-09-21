// -------------------------------------------------------------------------------------------------- //
// Robert's Smorgasbord 2022                                                                          //
// https://robertssmorgasbord.net                                                                     //
// https://www.youtube.com/channel/UCGtReyiNPrY4RhyjClLifBA                                           //
// QST QMC5883L 3-Axis Digital Compass and Arduino MCU – The Details (3) https://youtu.be/O3ZtVkj6CvU //
// -------------------------------------------------------------------------------------------------- //

#include <Wire.h>
#include "qmc5883l.h"


boolean error;

QMC5883L qmc5883l(&Wire);

void setup()
{
   Serial.begin(57600);
   while(!Serial);
   
   Wire.begin();
   Wire.setClock(400000); // 400kHz I2C clock

   if (!qmc5883l.begin())
   {
      error = true;
   }
   else
   {
      qmc5883l.setConfig({ QMC5883L::Mode::continuous,
                           QMC5883L::OutputDataRate::odr_10hz,
                           QMC5883L::FullScaleRange::rng_8g,
                           QMC5883L::OverSampleRate::osr_512,
                           false                              });

      if (!qmc5883l.writeConfig())
      {
         error = true;
      }
      else
      {
         //qmc5883l.setCalibration({{{-790, 1387}, {-2097, 601}, {-695, 2153}}}); // From "The Basics"
         //qmc5883l.setCalibration({{{-883, 1185}, {-2005, 766}, {-936, 1965}}}); // New
         qmc5883l.setCalibration({{{-842, 306}, {-1342, -96}, {-1117, -886}}}); // New, Z rotation only

         error = false;
      }
   }

      
}

void loop()
{
   //if (error) return;

   if (qmc5883l.readData())
   {
      //printRawData(qmc5883l.rawDataAxes());
      //printCalibratedData(qmc5883l.calibratedDataAxes());
      //printStatus(qmc5883l.dataReady(), qmc5883l.dataOverflow(), qmc5883l.dataSkipped());
      //plotData(qmc5883l.rawDataX(), qmc5883l.rawDataY(), qmc5883l.rawDataZ());
      //plotData(qmc5883l.calibratedDataX(), qmc5883l.calibratedDataY(), qmc5883l.calibratedDataZ());
      //plotData(qmc5883l.rawDataAxes(), qmc5883l.calibratedDataAxes());
      plotData(qmc5883l.rawDataAxes());
      //Serial.println(qmc5883l.azimuthZUp());
   }
   delay(150);
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

void printConfig(const QMC5883L::Config& config)
{
   Serial.println("Config:");
   Serial.print(  "  Mode               = "); 
   switch (config.mode)
   {
      case QMC5883L::Mode::standby:    Serial.println("Standby"); break;
      case QMC5883L::Mode::continuous: Serial.println("Continuous"); break;
   }
   Serial.print(  "  Output Data Rate   = ");
   switch (config.output_data_rate)
   {
      case QMC5883L::OutputDataRate::odr_10hz:  Serial.println("10Hz");  break;
      case QMC5883L::OutputDataRate::odr_50hz:  Serial.println("50Hz");  break;
      case QMC5883L::OutputDataRate::odr_100hz: Serial.println("100Hz"); break;
      case QMC5883L::OutputDataRate::odr_200hz: Serial.println("200Hz"); break;
   }
   Serial.print(  "  Full Scale Range   = ");
   switch (config.full_scale_range)
   {
      case QMC5883L::FullScaleRange::rng_2g: Serial.println("2G"); break;
      case QMC5883L::FullScaleRange::rng_8g: Serial.println("5G"); break;
   }
   Serial.print(  "  Over Sample Rate   = ");
   switch (config.over_sample_rate)
   {
      case QMC5883L::OverSampleRate::osr_64:  Serial.println("64");  break;
      case QMC5883L::OverSampleRate::osr_128: Serial.println("128"); break;
      case QMC5883L::OverSampleRate::osr_256: Serial.println("256"); break;
      case QMC5883L::OverSampleRate::osr_512: Serial.println("512"); break;
   }
   Serial.print(  "  Interrupt Disabled = ");
   Serial.println(config.interrupt_disabled ? "True" : "False");  
}

void printStatus(boolean data_ready, boolean data_overflow, boolean data_skipped)
{
   Serial.print("Status: Data Ready = ");
   Serial.print(data_ready     ? "True " : "False");
   Serial.print(", Data Overflow = ");
   Serial.print(data_overflow  ? "True " : "False");
   Serial.print(", Data Skipped = ");
   Serial.println(data_skipped ? "True " : "False");
}

void printRawData(QMC5883L::RawDataAxes rawDataAxes)
{
   char buffer[strlen("Raw Data: X = -12345; Y = -12345; Z = -12345") + 1];

   sprintf(buffer, 
           "Raw Data: X = % 5d; Y = % 5d; Z = % 5d",
           rawDataAxes.x,
           rawDataAxes.y,
           rawDataAxes.z                            );

   Serial.println(buffer);  
}

void printCalibratedData(QMC5883L::CalibratedDataAxes calibratedDataAxes)
{
   Serial.print("Calibrated Data: X = ");
   Serial.print(calibratedDataAxes.x, 5);
   Serial.print("; Y = ");
   Serial.print(calibratedDataAxes.y, 5);
   Serial.print("; Z = ");
   Serial.println(calibratedDataAxes.z, 5);
}

void plotData(float x, float y, float z)
{
   Serial.print(x);
   Serial.print("\t");
   Serial.print(y);
   Serial.print("\t");
   Serial.print(z);
   Serial.println();
}

void plotData(QMC5883L::RawDataAxes rawDataAxes)
{
   Serial.print(rawDataAxes.x);
   Serial.print("\t");
   Serial.print(rawDataAxes.y);
   Serial.print("\t");
   Serial.print(rawDataAxes.z);
   Serial.print("\t");
   Serial.println();
}

void plotData(QMC5883L::RawDataAxes        rawDataAxes,
              QMC5883L::CalibratedDataAxes calibratedDataAxes)
{
   Serial.print(rawDataAxes.x);
   Serial.print("\t");
   Serial.print(calibratedDataAxes.x);
   Serial.print("\t");
   Serial.print(rawDataAxes.y);
   Serial.print("\t");
   Serial.print(calibratedDataAxes.y);
   Serial.print("\t");
   Serial.print(rawDataAxes.z);
   Serial.print("\t");
   Serial.print(calibratedDataAxes.z);
   Serial.println();
}
