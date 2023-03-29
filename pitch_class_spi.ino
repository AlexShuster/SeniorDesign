#include <SPI.h>

char buf [100];
volatile byte pos;
volatile bool process_it;

uint8_t predict_two_pitch(float, float);

void setup (void)
{
  Serial.begin (115200);   // debugging

  // turn on SPI in slave mode
  SPCR |= bit (SPE);

  // have to send on master in, *slave out*
  pinMode (MISO, OUTPUT);

  // get ready for an interrupt
  pos = 0;   // buffer empty
  process_it = false;

  // now turn on interrupts
  SPI.attachInterrupt();

}  // end of setup


// SPI interrupt routine
ISR (SPI_STC_vect)
{
byte c = SPDR;  // grab byte from SPI Data Register

  // add to buffer if room
  if (pos < sizeof buf)
    {
    buf [pos++] = c;

    // example: newline means time to process buffer
    if (pos == 2*sizeof(float))
      process_it = true;

    }  // end of room available
}  // end of interrupt routine SPI_STC_vect

// main loop - wait for flag set in interrupt routine
void loop (void)
{
  if (process_it)
    {
    //buf [pos] = 0;
    float vel = *(float*)buf;
    float spin = *(float*)(buf+sizeof(float));
    //Serial.print("Velocity: %f, Spin Rate: %f \n", vel, spin);
    Serial.print("Velocity: ");
    Serial.print(vel);
    Serial.print(" Spin Rate: ");
    Serial.print(spin);
    Serial.println();

    float vel_mph = 2.237 * vel;
    
    uint8_t prediction_output = predict_two_pitch(vel_mph,spin);
    //Serial.print("Classification: ");
    //Serial.println(prediction_output);
    
    pos = 0;
    process_it = false;
    }  // end of flag set

}  // end of loop


uint8_t predict_two_pitch(float velocity, float spin_rate) {
  //W = weight vector from pretrained SVM Model
  float w[2] = {2.68913904, 0.02047686};
  //Bias from pretrained SVM Model
  float b = -269.20533180465014;
  //Feature Vector X = [Velocity, Spin Rate]
  float X[2] = {velocity, spin_rate};  
  //Linear SVM equation to get predicted value
  float y = X[0]*w[0] + X[1]*w[1] + b;  
  uint8_t pred = 0;
  if (y > 1){
    pred = 1;
  } else {
    pred = 0;
  }
  Serial.print("Classification: ");
  Serial.println(pred);
  return pred;
}
