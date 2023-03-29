/*
  Modified from arduinoFFT example:
	Example of use of the FFT libray to compute FFT for a signal sampled through the ADC.
        Copyright (C) 2018 Enrique Condés and Ragnar Ranøyen Homb

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "arduinoFFT.h"
#include <ADC.h>
#include <SPI.h>
#include <String.h>

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
/*
These values can be changed in order to evaluate the functions
*/
#define CHANNEL A0
//#define CHANNEL A13
//#define CHANNEL 15
const uint16_t samples = 16384; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 40000; //Hz, must be less than 10000 due to ADC

unsigned int sampling_period_us;
unsigned long microseconds;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

void setup()
{
  
  digitalWrite(SS, HIGH);  // ensure SS stays high for now

  // Put SCK, MOSI, SS pins into output mode
  // also put SCK, MOSI into LOW state, and SS into HIGH state.
  // Then put SPI hardware into Master mode and turn SPI on
  SPI.begin ();

  // Slow down the master a bit
  //SPI.setClockDivider(SPI_CLOCK_DIV8);
  SPI.setClockDivider(SPI_CLOCK_DIV128);

  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  //Serial.begin(115200);
  //while(!Serial);
  //Serial.println("Ready");
  //analogReadResolution(12);


  
}

void loop()
{
  //need something to trigger sampling
  //int run_doppler=0;
  
  /*SAMPLING*/
  microseconds = micros();
  for(int i=0; i<samples; i++)
  {
      vReal[i] = analogRead(CHANNEL);
      //vReal[i] = adc_read_fast(CHANNEL);
      vImag[i] = 0;
      while(micros() - microseconds < sampling_period_us){
        //empty loop
      }
      microseconds += sampling_period_us;
  }
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); 
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  double f_rx = FFT.MajorPeak(vReal, samples, samplingFrequency);

  //printData();
  //Serial.println();

  //Serial.print("Freq: ");
  //Serial.println(f_rx, 6); 
  
  
  float vel = dopplerVel(f_rx);
  //Serial.print("\nDoppler Shift: ");
  //Serial.println(vel);
  float spin = calcSpinRate(vReal, (samples >> 1), f_rx);
  //while(1); /* Run Once */
  //delay(10000); /* Repeat after delay */
 
  //char c;

  //float vel2 = 85.7934;
  //float spin2 = 1513.543;
  char* vel_buff = (char*)&vel;
  char* spin_buff = (char*)&spin;

  

  // enable Slave Select
  digitalWrite(SS, LOW);    // SS is pin 10
  delay(500);

  // send test string
  for (int i = 0; i < sizeof(float); i++)
    SPI.transfer (vel_buff[i]);

  for(int i = 0; i < sizeof(float); i++)
    SPI.transfer(spin_buff[i]);

  

  // disable Slave Select
  digitalWrite(SS, HIGH);

  delay (500);  // 1 seconds delay
  
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa = 0.0;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
	break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
	break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
	break;
    }
    //Serial.print(abscissa, 6);
    //if(scaleType==SCL_FREQUENCY)
      //Serial.print("Hz");
    //Serial.print(" ");
    //Serial.println(vData[i], 4);
  }
  //Serial.println();
}

void printData() {
  /* Print the results of the sampling according to time */
  //Serial.println("Data:");
  PrintVector(vReal, samples, SCL_TIME);
  //FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  //Serial.println("Weighed data:");
  PrintVector(vReal, samples, SCL_TIME);
  //FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  //Serial.println("Computed Real values:");
  PrintVector(vReal, samples, SCL_INDEX);
  //Serial.println("Computed Imaginary values:");
  PrintVector(vImag, samples, SCL_INDEX);
  //FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  //Serial.println("Computed magnitudes:");
  PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
  //double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
  //Serial.println(x, 6); //Print out what frequency is the most dominant.
}

float dopplerVel(double f_rx) {
  float intermediate = 0.00625; //c/(2*f_tx) done outside so that we don't worry about overflow.
  float v_obj = f_rx * intermediate;
  return v_obj;
}

////model call after (but no SCL_FREQ necessary) PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
float calcSpinRate(double *vData, uint16_t bufferSize, double f_rx) {
  double max_bandwidth = 1400;
  uint16_t max_idx = (uint16_t)(((f_rx + max_bandwidth) * ((double)samples))/samplingFrequency);
  uint16_t min_idx = (uint16_t)(((f_rx + 10) * ((double)samples))/samplingFrequency); //10 is to avoid main hump
  if (min_idx > max_idx) {
    max_idx = 65000;
    //Serial.println("OVERFLOW!!!");
  }

  if (max_idx > bufferSize) {
    max_idx = bufferSize - 1;
  }
  
  //Serial.println("Max search:");
  //Serial.println(((max_idx * 1.0 * samplingFrequency) / ((double)samples)));
  //Serial.println("Min search:");
  //Serial.println(((min_idx * 1.0 * samplingFrequency) / ((double)samples)));
  double freq_band = 0.0;
  double freq_band_mag = 0.0;
  uint16_t freq_band_idx = 0;
  for (uint16_t i = max_idx; i > min_idx; i--){
     if (vData[i] > 5000 && vData[i] > freq_band_mag) // TODO make better
     {
        freq_band_mag = vData[i];
        freq_band_idx = i;
     }
  }
  freq_band = ((freq_band_idx * 1.0 * samplingFrequency) / ((double)samples));
  //Serial.println("Found bandwidth");
  //Serial.println(freq_band - f_rx);

  float spin_rate = (freq_band-f_rx) *60.0;
  //Serial.println("\nSpin Rate:");
  //Serial.println(spin_rate);
 //return (freq_band - f_rx);
  return spin_rate;

}
