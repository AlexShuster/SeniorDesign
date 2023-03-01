const int velocity_input = 2;
const int spin_rate_input = 3;
//Output prediction to a pin, 0 = Offspeed and 1 = Fastball
const int prediction_output = 4;

void setup() {
  // put your setup code here, to run once:
  pinMode(velocity_input,INPUT);
  pinMode(spin_rate_input,INPUT);
  pinMode(prediction_output,OUTPUT);
  Serial.begin(115200);
}

void loop() {
  // Commented out function call for now
  //u_int8_t prediction_output = predict_two_pitch(velocity_input,spin_rate_input);
  //digitalWrite(prediction_output,pred);
}


u_int8_t predict_two__pitch(float velocity, float spin_rate) {
  //W = weight vector from pretrained SVM Model
  float w[2] = {2.68913904, 0.02047686};
  //Bias from pretrained SVM Model
  float b = -269.20533180465014;
  //Feature Vector X = [Velocity, Spin Rate]
  float X[2] = {velocity, spin_rate};  
  //Linear SVM equation to get predicted value
  float y = X[0]*w[0] + X[1]*w[1] + b;  
  u_int8_t pred = 0;
  if (y > 1){
    pred = 1;
  } else {
    pred = 0;
  }
  Serial.print(pred);
  return pred;
}
