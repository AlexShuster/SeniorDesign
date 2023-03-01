void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}


int predict_two__pitch(float velocity, float spin_rate) {
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
  digitalWrite(prediction_output,pred);
}