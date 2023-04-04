const int velocity_input = 2;
const int spin_rate_input = 3;
//Output prediction to a pin, 0 = Offspeed and 1 = Fastball
const int prediction_output = 4;

float X[100][3] = {{1 , 86.8 , 1868}, 
{1 , 89.4 , 2359}, 
{1 , 93.3 , 2194}, 
{1 , 91.9 , 2653}, 
{1 , 89.9 , 1943}, 
{1 , 95.6 , 2191}, 
{1 , 76.9 , 2223}, 
{1 , 88.7 , 2185}, 
{1 , 84.8 , 2032}, 
{1 , 94.0 , 2409}, 
{1 , 94.8 , 2199}, 
{0 , 79.2 , 2495}, 
{1 , 95.5 , 2425}, 
{1 , 97.3 , 2328}, 
{0 , 91.4 , 2072}, 
{0 , 92.6 , 2018}, 
{1 , 88.6 , 2140}, 
{1 , 87.8 , 1810}, 
{1 , 83.5 , 2671}, 
{0 , 90.9 , 2240}, 
{0 , 91.7 , 2173}, 
{0 , 78.2 , 2401}, 
{1 , 88.2 , 2366}, 
{1 , 95.2 , 2151}, 
{1 , 95.4 , 2225}, 
{1 , 87.8 , 1810}, 
{0 , 89.8 , 2035}, 
{1 , 95.5 , 1877}, 
{0 , 83.8 , 2080}, 
{1 , 92.9 , 2417}, 
{1 , 79.3 , 2183}, 
{1 , 95.4 , 2225}, 
{1 , 89.0 , 2203}, 
{0 , 87.6 , 1642}, 
{1 , 84.0 , 2215}, 
{1 , 88.2 , 1876}, 
{0 , 85.5 , 2349}, 
{1 , 87.7 , 1845}, 
{1 , 93.8 , 2341}, 
{1 , 85.5 , 2189}, 
{1 , 97.9 , 2184}, 
{0 , 92.9 , 2274}, 
{1 , 83.0 , 2065}, 
{0 , 93.2 , 2066}, 
{1 , 93.8 , 2357}, 
{1 , 77.2 , 2307}, 
{1 , 88.9 , 2441}, 
{1 , 99.1 , 2352}, 
{0 , 88.7 , 1634}, 
{1 , 85.9 , 1484}, 
{1 , 95.4 , 2404}, 
{0 , 88.8 , 1939}, 
{1 , 92.3 , 2000}, 
{1 , 87.4 , 2356}, 
{1 , 83.3 , 2293}, 
{0 , 93.2 , 2066}, 
{1 , 95.8 , 2023}, 
{0 , 95.0 , 2122}, 
{1 , 93.3 , 2055}, 
{1 , 84.4 , 2291}, 
{1 , 91.5 , 2316}, 
{1 , 82.6 , 2483}, 
{1 , 92.6 , 2128}, 
{1 , 92.4 , 2075}, 
{1 , 93.2 , 2283}, 
{0 , 89.3 , 1959}, 
{0 , 83.9 , 2320}, 
{1 , 90.2 , 1707}, 
{1 , 88.2 , 1701}, 
{1 , 88.2 , 2233}, 
{1 , 89.2 , 2125}, 
{0 , 91.4 , 2255}, 
{1 , 85.5 , 2519}, 
{1 , 96.4 , 2298}, 
{1 , 82.7 , 1705}, 
{1 , 84.6 , 2941}, 
{1 , 87.4 , 2263}, 
{1 , 93.6 , 2173}, 
{0 , 83.4 , 2585}, 
{0 , 98.7 , 2370}, 
{1 , 92.4 , 2140}, 
{0 , 86.3 , 1894}, 
{1 , 93.6 , 2300}, 
{1 , 88.6 , 2330}, 
{1 , 76.5 , 2834}, 
{1 , 81.2 , 1332}, 
{1 , 88.1 , 2136}, 
{0 , 79.3 , 2338}, 
{0 , 89.2 , 2736}, 
{1 , 90.0 , 2150}, 
{1 , 91.7 , 2122}, 
{1 , 94.7 , 2452}, 
{1 , 83.8 , 2060}, 
{1 , 79.7 , 2627}, 
{0 , 81.3 , 2368}, 
{1 , 89.6 , 2160}, 
{1 , 93.6 , 2196}, 
{1 , 82.9 , 2422}, 
{1 , 95.6 , 2518}, 
{1 , 83.5 , 1142}};
uint8_t y_two_pitch[100] = {0.0, 1.0,1.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,1.0,0.0,1.0,1.0,1.0,1.0
,1.0,0.0,0.0,1.0,1.0,0.0,0.0,1.0,1.0,0.0,1.0,1.0,0.0,1.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,1.0,0.0
,1.0,1.0,0.0,1.0,1.0,0.0,0.0,1.0,0.0,0.0,1.0,1.0,1.0,1.0,0.0,1.0,1.0,1.0,1.0,0.0,0.0,0.0,1.0,1.0
,1.0,1.0,0.0,0.0,0.0,0.0,1.0,1.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,1.0,1.0,0.0,1.0,1.0,0.0,0.0,1.0,0.0
,0.0,1.0,1.0,1.0,0.0,0.0,0.0,1.0,1.0,0.0,1.0
,0.0};
uint8_t y_seven_pitch[100] = {0,2,5,6,0,5,6,6,0,3,3,6,3,5,5,5,2,0,6,3,3,1,6,3,5,0,5,5,
6,2,1,5,6,0,1,0,2,0,3,6,5,3,0,3,3,1,6,3,0,0,3,5,5,2,6,3,5,3,3,1,6,6,5,3,3,3,6,0,0,6,2,
3,6,3,0,1,6,3,6,3,3,0,3,2,1,0,2,6,6,2,5,3,0,1,6,2,5,6,3,4};


void setup() {
  // put your setup code here, to run once:
  pinMode(velocity_input,INPUT);
  pinMode(spin_rate_input,INPUT);
  pinMode(prediction_output,OUTPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t y_pred[100] = {};
  
  // for(int i = 0; i < 1; i++) {
  //   y_pred[i] = predict_seven_pitch(X[i][0],X[i][1],X[i][2]);
  // }
  delay(1000);
  accuracy(y_pred,y_seven_pitch,100);
}

void accuracy(uint8_t y_pred[],uint8_t y_test[],int N) {
  float count = 0;
  for(int i = 0; i < N; i++) {
    if(y_pred[i] == y_test[i]) {
      count += 1.0;
    }
  }
  float accuracy = (count/N)*100;
  Serial.print("Accuracy: ");
  Serial.print(accuracy);
  Serial.print("%");
  Serial.println();
  delay(1000);
}


//1 = right, 0 = left
//updated with offspeed = 0(Channgeup, Curveball, Slider, Splitter) and Fastball = 1(4-seam Fastball, Cutter, and Sinker)
uint8_t predict_two_pitch(int throwing_arm, float velocity, float spin_rate) {
  // Serial.print("Velocity:");
  // Serial.print(velocity);
  // delay(1000);
  //W = weight vector from pretrained SVM Model
  float w[3] = {-3.77248428, 3.11702946,  0.00617956};
  float b = -286.35451119231914;
  //Feature Vector X = [Velocity, Spin Rate]
  float X[3] = {throwing_arm, velocity, spin_rate};  
  //Linear SVM equation to get predicted value
  float y = X[0]*w[0] + X[1]*w[1] + X[2]*w[2] + b;  
  // Serial.print("Y:");
  // Serial.print(y);
  // delay(1000);
  uint8_t pred = 0;
  if (y > 1){
    pred = 1;
  } else {
    pred = 0;
  }
  // Serial.print("Pred: ");
  // Serial.print(pred);
  // delay(1000);
  return pred;
  // digitalWrite(prediction_output,pred);
}

//ages 10-18, 19 == college, else is MLB
uint8_t predict_experience_two_pitch(int throwing_arm, float velocity, float spin_rate, int experience_level) {
  float w[3] = {};
  float b = 0.0;
  if (experience_level == 10){
    //accuracy = 70%
    w[0] = 0.34136446;
    w[1] = -0.854808;
    w[2] = 0.00666127;
    b = 37.561499850603695;
  } else if(experience_level == 11) {
    //accuracy = 60%
    w[0] = 0.49015269;
    w[1] = 0.34162832;
    w[2] = -0.0088187;
    b = -6.8652997075552;
  } else if(experience_level == 12) {
    //accuracy = 73%
    w[0] = -1.20213472;
    w[1] = 0.97455178;
    w[2] = -0.00164458;
    b = -55.007939723149335;
  } else if(experience_level == 13) {
    //accuracy = 86%
    w[0] = -1.88227197;
    w[1] = -1.32382397;
    w[2] =  0.00413271;
    b = -89.11690499174325;
  } else if(experience_level == 14) {
    //accuracy = 88%
    w[0] = -1.50177066;
    w[1] =  1.19553603;
    w[2] = 0.00380762;
    b = -87.05943299468039;
  } else if(experience_level == 15) {
    //accuracy = 89%
    w[0] = -1.53755173;
    w[1] = 1.08828407;
    w[2] = 0.00543559;
    b = -86.65224626873662;
  } else if(experience_level == 16) {
    //accuracy = 88%
    w[0] = -1.28979595;
    w[1] =  1.09714135;
    w[2] =  0.00251726;
    b = -85.02884363760495;
  } else if(experience_level == 17) {
    //accuracy = 85%
     w[0] = -1.74418036;
     w[1] = 1.27264135;
     w[2] = 0.00460643;
     b = -104.92057142090434;
  } else if(experience_level == 18) {
    //accuracy = 86%
    w[0] = -1.47897291;
    w[1] =  1.24876061;
    w[2] =  0.0037658;
    b = -105.31950113774215;    
  } else if(experience_level == 19) {
    //accuracy = 87%
    w[0] = -1.93802168;
    w[1] = 1.36263619;
    w[2] = 0.00571239;
    b = -118.44628437551006;
  } else {
    //MLB
    //accuracy = 96%
    w[0] = -3.77248428;
    w[1] = 3.11702946;
    w[2] = 0.00617956;
    b = -286.35451119231914;
  }
  //Linear SVM equation to get predicted value
  float y = w[0]*throwing_arm + w[1]*velocity + w[2]*spin_rate + b;  
  Serial.print("y: ");
  Serial.println(y);
  delay(1000);
  uint8_t pred = 0;
  if (y > 1) {
    pred = 1;
  } else {
    pred = 0;
  }
  Serial.print("Pred: ");
  Serial.println(pred);
  delay(1000);
  return pred;
}
*/

uint8_t predict_seven_pitch(int throwing_arm, float velocity, float spin_rate) {
  int votes[7] = {};
  float w[21][3] = {{-5.53687822e-01,  6.07595178e-01, -7.08212091e-03},
 { 2.42403226e+00, -7.94967065e-01, -1.13656390e-02},
 { 1.33509133e+00, -9.84782409e-01, -8.02153088e-03},
 {-8.65496000e-01, -2.10310170e-01,  5.96280231e-03},
 { 1.33287822e+00, -9.48333970e-01, -5.35347263e-03},
 { 4.97903002e-01,  1.84521701e-01, -1.49712591e-02},
 { 2.49433471e+00, -1.06920800e+00,  2.97406047e-03},
 { 4.27132600e-01, -7.08176986e-01,  5.27634307e-03},
 {-4.44089210e-16, -2.22248945e-01,  4.40724944e-03},
 { 4.44089210e-15, -9.28050819e-01,  9.67616322e-03},
 { 2.16320081e+00, -2.03894284e+00,  7.16791168e-04},
 { 1.16119181e+00, -1.44816637e+00,  1.73191049e-02},
 {-5.93341211e-04,  1.42401891e-03,  3.44137902e-02},
 { 1.06255320e+00, -1.37779046e+00,  1.93009688e-02},
 {-1.98014179e+00,  1.86624103e+00, -3.77155171e-03},
 {-2.07952565e-01,  8.55437708e-01,  3.24837696e-01},
 { 5.26366139e-01, -3.12180856e-01,  3.88841103e-02},
 {-1.24045808e+00,  1.60826307e+00, -1.19519517e-02},
 { 6.85321906e-01, -1.17403656e-01, -1.39033387e-02},
 { 3.64232382e-01,  5.83512361e-0,1 -7.88401053e-03},
 {-9.47652855e-01,  1.40350125e+00, -1.15943129e-02}};
  float b[21] = {-34.67567594,   91.67355429,  103.27184256,   11.56615811,   94.43161805,
  14.04751384,   81.83168503,   48.72559804,   10.72821916,   58.92118838,
  162.22771893,   89.47132755,  -66.47297999,   80.58588467, -154.48706709
 -690.81156033,  -53.77953048, -115.71840327,   34.44554411,  -36.67600083,
  -99.30674084}; 
  float pred[21] = {};
  // for(int i = 0; i < 21; i++) {
  //   pred[i] = w[i][0]*throwing_arm + w[i][1]*velocity + w[i][2]*spin_rate + b[i];
  // }
  for(int j = 0; j < 21; j++) { 
    pred[j] = w[j][0]*throwing_arm + w[j][1]*velocity + w[j][2]*spin_rate + b[j];   
    if(pred[j] > 0 and j < 6){
      votes[0] += 1;
    } else if(pred[j] < 0 and j == 0) {
        votes[1] += 1;
    } else if(pred[j] < 0 and j == 1) {
        votes[2] += 1;
    } else if(pred[j] < 0 and j == 2) {
        votes[3] += 1;
    } else if(pred[j] < 0 and j == 3) {
        votes[4] += 1;
    } else if(pred[j] < 0 and j == 4) {
        votes[5] += 1;
    } else if(pred[j] < 0 and j == 5) {
        votes[6] += 1;
    } else if(pred[j] > 0 and j >= 6 and j < 11) {
        votes[1] += 1;
    } else if(pred[j] < 0 and j == 6) {
        votes[2] += 1;
    } else if(pred[j] < 0 and j == 7) {
        votes[3] += 1;
    } else if(pred[j] < 0 and j == 8) {
        votes[4] += 1;
    } else if(pred[j] < 0 and j == 9) {
        votes[5] += 1;
    } else if(pred[j] < 0 and j == 10) {
        votes[6] += 1;
    } else if(pred[j] > 0 and j >= 11 and j < 15) {
        votes[2] += 1;
    } else if(pred[j] < 0 and j == 11) {
        votes[3] += 1;
    } else if(pred[j] < 0 and j == 12) {
        votes[4] += 1;
    } else if(pred[j] < 0 and j == 13) {
        votes[5] += 1;
    } else if(pred[j] < 0 and j == 14) {
        votes[6] += 1;
    } else if(pred[j] > 0 and j >= 15 and j < 18) {
        votes[3] += 1;
    } else if(pred[j] < 0 and j == 15) {
        votes[4] += 1;
    } else if(pred[j] < 0 and j == 16) {
        votes[5] += 1;
    } else if(pred[j] < 0 and j == 17) {
        votes[6] += 1;
    } else if(pred[j] > 0 and j >= 18 and j < 20) {
        votes[4] += 1;
    } else if(pred[j] < 0 and j == 18) {
        votes[5] += 1;
    } else if(pred[j] < 0 and j == 19) {
        votes[6] += 1;
    } else if(pred[j] > 0 and j == 20) {
        votes[5] += 1;
    } else {
        votes[6] += 1;
    }
  }
  int max_votes = 0;
  uint8_t class_label = 0;
  for(int i = 0; i < 7; i++) {
    if(votes[i] > max_votes) {
      max_votes = votes[i];
      class_label = i;
    }
  }
  Serial.print("Class Label: ");
  Serial.println(class_label);
  delay(1000);
  return class_label;
}

/*
uint8_t predict_experience_seven_pitch(int throwing_arm, float velocity, float spin_rate, int experience_level) {
  int votes[21] = {};
  float w[21][3] = {{}};
  float b[21] = {};
  if (experience_level == 10){
    float w[21][3] = {{-3.14321274e-02, -9.39399279e-01, 4.94412281e-03},
    { 0.00000000e+00,  1.25495704e-03,  2.54144837e-02},
    {-3.29715011e-01,  3.55836538e+00,  7.03227481e-02},
    { 0.00000000e+00,  1.25495704e-03,  2.54144837e-02},
    { 0.00000000e+00,  1.25495704e-03,  2.54144837e-02},
    { 1.39655835e-02, -1.51856744e+00,  7.37943676e-02},
    { 0.00000000e+00,  5.33420182e-03,  3.89636064e-02},
    { 2.05025757e-02,  6.94611384e-02, -1.89103429e-01},
    { 0.00000000e+00,  5.33420182e-03,  3.89636064e-02},
    { 0.00000000e+00,  5.33420182e-03,  3.89636064e-02},
    {-1.82342838e-01,  2.49657323e+00, -4.61061391e-03},
    { 2.11094114e-04, -7.28459565e-04, -2.05332174e-02},
    { 0.00000000e+00,  0.00000000e+00,  0.00000000e+00},
    { 0.00000000e+00,  0.00000000e+00,  0.00000000e+00},
    { 7.01162493e-04, -3.48996213e-03, -3.72780568e-02},
    {-2.11094111e-04,  7.28459553e-04,  2.05332170e-02},
    {-1.24684610e-04,  7.28536448e-04,  2.05353845e-02},
    { 0.00000000e+00, -1.21125783e-01,  4.08654778e-01},
    { 0.00000000e+00,  0.00000000e+00,  0.00000000e+00},
    { 7.01162493e-04, -3.48996213e-03, -3.72780568e-02},
    { 7.01162493e-04, -3.48996213e-03, -3.72780568e-02}};
    float b[21] = {4.28517098,  -0.99999999, -22.51406769,  -0.99999999,
    -0.99999999,  -0.49309521,  -1.00000001,  17.15446904,
    -1.00000001,  -1.00000001, -13.77036364,   0.99978893,
    1.0        ,  -1.0        ,   0.9992988 ,  -0.99978892,
    -0.99995845, -38.33202627,  -1.0      ,   0.9992988 ,
    0.9992988};

  } else if(experience_level == 11) {
    float w[21][3] = {{5.55111512e-17, -8.91081708e-01,  1.10928836e-01},
    {1.11774652e-04,  1.54002077e-03,  3.45118353e-02},
    {-2.83189538e-01,  2.14360170e+00,  1.50102214e-01},
    {1.11774652e-04,  1.54002077e-03,  3.45118353e-02},
    {1.11774652e-04,  1.54002077e-03,  3.45118353e-02},
    {3.15945580e-02, -1.53752029e+00,  5.75482066e-02},
    {0.00000000e+00,  8.60304153e-03,  5.73159123e-02},
    {5.43910628e-02,  1.54294078e-01, -2.86387211e-01},
    {0.00000000e+00,  8.60304153e-03,  5.73159123e-02},
    {0.00000000e+00,  8.60304153e-03,  5.73159123e-02},
    {-1.33226763e-15,  2.73909857e+00, -4.25373845e-02},
    {4.74177854e-04, -1.06289306e-03, -3.07733833e-02},
    {0.00000000e+00,  0.00000000e+00,  0.00000000e+00},
    {0.00000000e+00,  0.00000000e+00,  0.00000000e+00},
    {7.01162493e-04, -3.48996213e-03, -3.72780568e-02},
    {-2.11094111e-04,  7.28459553e-04,  2.05332170e-02},
    {-1.24684610e-04,  7.28536448e-04,  2.05353845e-02},
    {0.00000000e+00, -1.21125783e-01,  4.08654778e-01},
    {0.00000000e+00,  0.00000000e+00,  0.00000000e+00},
    {7.01162493e-04, -3.48996213e-03, -3.72780568e-02},
    {7.01162493e-04, -3.48996213e-03, -3.72780568e-02}
  };
  float b[21] = {-3.11623185,  -1.00007451, -18.77413025,  -1.00007451,
  -1.00007451,   0.36465671,  -1.00000001,  17.24020743,
  -1.00000001,  -1.00000001,  -9.46418468,   0.99952582,
  1.0        ,  -1.0        ,   0.99999996,  -1.0        ,
  -1.0        ,   0.64824689,  -1.0        ,   0.99999996,
    0.99999996};
  } else if(experience_level == 12) {
    float w[21][3] = {{0.000000, -0.022381, 0.399793},
    {0.000000, 0.006148, 0.310140},
    {0.000000, 0.025969, 0.978196},
    {0.000000, 0.006148, 0.310140},
    {0.000000, 0.006148, 0.310140},
    {0.000000, -0.012441, 0.524657},
    {0.000000, 0.926895, 1.903927},
    {0.027609, 0.100163, -0.800790},
    {0.000000, 0.926895, 1.903927},
    {0.000000, 0.926895, 1.903927},
    {0.255339, 1.862035, -3.780831},
    {0.000000, -0.012987, -0.514691},
    {0.000000, 0.000000, 0.000000},
    {0.000000, 0.000000, 0.000000},
    {0.000000, -0.192399, -1.277032},
    {0.000000, 0.012987, 0.514691},
    {0.000000, 0.012987, 0.514691},
    {-0.112178, -0.132687, 1.428899},
    {0.000000, 0.000000, 0.000000},
    {0.000000, -0.192399, -1.277032},
    {0.000000, -0.192399, -1.277032}
    };
    float b[21] = {-1.57427653, -1.0        , -6.34836769, -1.0        , -1.0        ,
    -2.38044409, -1.0        ,  2.10008311, -1.0        , -1.0        ,
    5.04533849,  1.0        ,  1.0        , -1.0      ,  1.00000004,
    -1.0        , -1.0        , -4.53592404, -1.0        ,  1.00000004,
    1.00000004};
  } else if(experience_level == 13) {
    float w[21][3] = {
    {0.0, -0.0162818, 0.36519408},
    {0.0, -0.03216325, 0.68107553},
    {0.0, 0.01772613, 0.48038381},
    {0.0, -0.03216325, 0.68107553},
    {0.0, -0.03216325, 0.68107553},
    {0.0, -0.0162818, 0.36519408},
    {0.0, 0.9243589, 1.84486368},
    {0.02617895, -0.11339091, -0.7824636},
    {0.0, 0.9243589, 1.84486368},
    {0.0, 0.9243589, 1.84486368},
    {0.23871426, 1.78320146, -3.62062117},
    {0.0, -0.00767613, -0.52361876},
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0},
    {0.0, -0.17300014, -1.22010207},
    {0.0, -0.0001096, 0.51707081},
    {0.0, -0.0001096, 0.51707081},
    {-0.0823712, 0.12948513, 1.18077545},
    {0.0, 0.0, 0.0},
    {0.0, -0.17300014, -1.22010207},
    {0.0, -0.17300014, -1.22010207}
    };
    float b[21] = {-6.35205742,  -1.00000002, -14.11813032,  -1.00000002,
    -1.00000002,  -6.52266445,  -1.0        ,   5.90381414,
    -1.0        ,  -1.0        ,  -4.89215339,   0.99999895,
    1.0        ,  -1.0        ,   1.0        ,  -0.99999999,
    -0.99999999, -12.65418924,  -1.0        ,   1.0        ,
    1.0};
  } else if(experience_level == 14) {
    float w[21][3] = {{ 0.0, 0.000000, 0.000000 },
    { 0.0, 0.000000, 0.000000 },
    { 0.0, 0.000000, 0.000000 },
    { 0.0, 0.006148, 0.310140 },
    { 0.0, -0.022381, 0.399793 },
    { 0.0, -0.012441, 0.524657 },
    { 0.0, 0.025969, 0.978196 },
    { 0.0, -0.012987, -0.514691 },
    { 0.0, 0.926895, 1.903927 },
    { 0.0, 0.926895, 1.903927 },
    { 0.0, 0.926895, 1.903927 },
    { 0.0, 0.006148, 0.310140 },
    { 0.0, 0.000000, 0.000000 },
    { 0.0, 0.000000, 0.000000 },
    { 0.0, 0.000000, 0.000000 },
    { 0.0, -0.192399, -1.277032 },
    { 0.0, 0.012987, 0.514691 },
    { 0.0, 0.012987, 0.514691 },
    { 0.0, -0.132687, 1.428899 },
    { 0.0, -0.192399, -1.277032 },
    { 0.0, -0.192399, -1.277032 }
    };
    float b[21] = {-4.17462816,  -1.00044695, -10.59836037,  -1.00044695,
    -1.00044695, -21.25525824,  -1.0        ,   4.46814985,
    -1.0        ,  -1.0        ,  -2.00380038,   0.99999731,
    1.0        ,  -1.0        ,   1.0        ,  -1.00000001,
    -1.00000001, -21.0528144 ,  -1.0        ,   1.0        ,
    1.0};
  } else if(experience_level == 15) {
    float w[18][3] = {
    {0.0, -0.014, 0.467},
    {0.0, 0.0, 0.0},
    {0.0, -0.014, 0.467},
    {0.0, -0.003, 0.388},
    {0.0, 0.005, 0.272},
    {0.0, 0.0, 0.0},
    {0.0, -0.003, 0.388},
    {0.0, -0.003, 0.388},
    {0.0, 0.005, 0.272},
    {0.0, -0.014, 0.467},
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0},
    {0.0, -0.014, 0.467},
    {0.0, -0.014, 0.467},
    {0.0, 0.0, 0.0},
    {0.0, 0.005, 0.272},
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0}
    };
    float b[21] = {-1.31007333, -1.0        , -3.81047964, -1.0        , -1.0        ,
    -1.85788695, -1.0        ,  1.64285022, -0.94730924, -1.0        ,
    3.20855837,  1.0        ,  1.0        , -1.0        ,  1.0        ,
    -1.0        , -1.0        , -3.41452246, -1.0        ,  0.98182492,
    1.0};
  } else if(experience_level == 16) {
    float w[21][3] = {{0.00000000e+00, -2.52167352e-02, 6.64504408e-01},
    {0.00000000e+00, 8.36435872e-03, 5.88719067e-01},
    {0.00000000e+00, 1.03409051e-02, 1.10443404e+00},
    {0.00000000e+00, 8.36435872e-03, 5.88719067e-01},
    {0.00000000e+00, 8.36435872e-03, 5.88719067e-01},
    {3.05359597e-02, -1.10970628e-02, 7.96270461e-01},
    {0.00000000e+00, 3.49470199e+00, 5.53851541e+00},
    {3.07393453e-02, 1.56048796e-01, -1.57339004e+00},
    {-2.21515703e-01, 3.41091933e+00, 5.48352924e+00},
    {0.00000000e+00, 3.49470199e+00, 5.53851541e+00},
    {9.51104902e-02, 2.01132224e+00, -6.65968791e+00},
    {7.11944207e-04, -2.75171756e-02, -1.22756883e+00},
    {0.00000000e+00, 0.00000000e+00, 0.00000000e+00},
    {0.00000000e+00, 0.00000000e+00, 0.00000000e+00},
    {0.00000000e+00, -5.49688656e-01, -3.43727662e+00},
    {-1.79357925e-02, 2.75589188e-02, 1.21702899e+00},
    {0.00000000e+00, 2.75269745e-02, 1.22800596e+00},
    {-3.28339430e-02, -1.62388581e-01, 2.53775861e+00},
    {0.00000000e+00, 0.00000000e+00, 0.00000000e+00},
    {1.62250688e-01, -5.28928091e-01, -3.35422957e+00},
    {0.00000000e+00, -5.49688656e-01, -3.43727662e+00},
    };
    float b[21] = {-1.25578595, -1.0        , -3.53041935, -1.0      , -1.0        ,
    -1.73443116, -1.0        ,  1.55576354, -0.77846527, -1.0        ,
    2.83857901,  0.99952537,  1.0        , -1.0        ,  1.0        ,
    -0.98208694, -1.0        , -3.14563168, -1.0        ,  0.83784837,
    1.0};
  } else if(experience_level == 17) {
    float w[21][3] = {{2.77555756e-17, -2.44532001e-02,  6.96387697e-01},
    { 2.77555756e-17,  8.35904613e-03,  6.28815745e-01},
    { 0.00000000e+00,  9.56938947e-03,  1.14873760e+00},
    { 2.77555756e-17,  8.35904613e-03,  6.28815745e-01},
    { 2.77555756e-17,  8.35904613e-03,  6.28815745e-01},
    { 2.65644762e-02, -9.93349077e-03,  8.19555635e-01},
    { 0.00000000e+00,  4.40424014e+00,  6.50665825e+00},
    { 2.68627975e-02,  1.65077118e-01, -1.72571241e+00},
    {-8.00790539e-02,  3.46434313e+00,  5.07336676e+00},
    { 0.00000000e+00,  4.40424014e+00,  6.50665825e+00},
    { 1.30709695e-01,  2.20594365e+00, -7.33821014e+00},
    { 5.55111512e-17, -3.02319089e-02, -1.38571111e+00},
    { 0.00000000e+00,  0.00000000e+00,  0.00000000e+00},
    { 0.00000000e+00,  0.00000000e+00,  0.00000000e+00},
    { 0.00000000e+00, -6.77196089e-01, -4.10321098e+00},
    { 0.00000000e+00,  3.02319089e-02,  1.38571111e+00},
    { 0.00000000e+00,  3.02319089e-02,  1.38571111e+00},
    {-2.92092062e-02, -1.59083040e-01,  2.69838579e+00},
    { 0.00000000e+00,  0.00000000e+00,  0.00000000e+00},
    { 1.78314723e-01, -6.46540309e-01, -3.96887002e+00},
    { 9.60461783e-04, -6.76869719e-01, -4.10124068e+00}}; 
    float b[21] = {-1.2134931 , -1.00000001, -3.33240461, -1.00000001, -1.00000001,
    -1.63235727, -0.99999998,  1.48433481, -0.48318587, -0.99999998,
      2.52774475,  1.0        ,  1.0        , -1.0        ,  1.0        ,
    -1.0        , -1.0        , -2.90866174, -1.0        ,  0.82192817,
      0.99923163};
  } else if(experience_level == 18) {
    float w[21][3] = {{ 0.00000000e+00, -6.83735951e-03,  1.47347252e-01},
    { 0.00000000e+00,  2.57710976e-03,  1.00379180e-01},
    {-2.68767897e-01,  9.45955196e-02,  3.58687253e-01},
    { 0.00000000e+00,  2.57710976e-03,  1.00379180e-01},
    { 0.00000000e+00,  2.57710976e-03,  1.00379180e-01},
    { 2.95645963e-02, -6.63094804e-03,  2.41269878e-01},
    { 0.00000000e+00,  1.57498492e-01,  4.99213581e-01},
    { 2.49597990e-02,  1.76627983e-02, -2.21324717e-01},
    { 0.00000000e+00,  1.57498492e-01,  4.99213581e-01},
    { 0.00000000e+00,  1.57498492e-01,  4.99213581e-01},
    { 2.90493509e-01,  1.12577691e+00, -1.66776714e+00},
    { 3.67541105e-05, -3.72005786e-03, -1.31174287e-01},
    { 0.00000000e+00,  0.00000000e+00,  0.00000000e+00},
    { 0.00000000e+00,  0.00000000e+00,  0.00000000e+00},
    { 0.00000000e+00, -3.49859692e-02, -3.01814044e-01},
    {-1.73472348e-18,  3.72012619e-03,  1.31176697e-01},
    {-1.73472348e-18,  3.72012619e-03,  1.31176697e-01},
    { 0.00000000e+00, -4.96271754e-02,  5.69763457e-01},
    { 0.00000000e+00,  0.00000000e+00,  0.00000000e+00},
    { 0.00000000e+00, -3.49859692e-02, -3.01814044e-01},
    {-2.29587716e-05, -3.49855665e-02, -3.01810570e-01}}; 
    float b[21] = {-1.93038386, -1.00000001, -8.45891457, -1.00000001, -1.00000001,
    -3.83017812, -1.0        ,  2.36410892, -1.0        , -1.0        ,
    9.77541151,  0.9999755 ,  1.0        , -1.0        ,  1.00000003,
    -0.99999999, -0.99999999, -7.65853764, -1.0        ,  1.00000003,
    1.00000765};
  } else if(experience_level == 19) {
    float w[21][3] = {{ 0.00000000e+00, -6.22385920e-03,  1.46189001e-01},
    { 0.00000000e+00,  2.61467124e-03,  1.05932132e-01},
    {-1.63806467e-01,  7.30017743e-02,  3.35657870e-01},
    { 0.00000000e+00,  2.61467124e-03,  1.05932132e-01},
    { 0.00000000e+00,  2.61467124e-03,  1.05932132e-01},
    { 2.56569060e-02, -5.01468978e-03,  2.25011967e-01},
    { 0.00000000e+00,  2.10967381e-01,  5.95808559e-01},
    { 2.66258584e-02,  1.74684299e-02, -2.28555541e-01},
    { 0.00000000e+00,  2.10967381e-01,  5.95808559e-01},
    { 0.00000000e+00,  2.10967381e-01,  5.95808559e-01},
    { 2.82448899e-01,  9.63142743e-01, -1.82691580e+00},
    { 5.45417428e-05, -4.10559537e-03, -1.44845879e-01},
    { 0.00000000e+00,  0.00000000e+00,  0.00000000e+00},
    { 0.00000000e+00,  0.00000000e+00,  0.00000000e+00},
    { 0.00000000e+00, -4.25303103e-02, -3.50751671e-01},
    { 0.00000000e+00,  4.10570737e-03,  1.44849831e-01},
    { 0.00000000e+00,  4.10570737e-03,  1.44849831e-01},
    { 0.00000000e+00, -3.92993342e-02,  5.37985459e-01},
    { 0.00000000e+00,  0.00000000e+00,  0.00000000e+00},
    { 0.00000000e+00, -4.25303103e-02, -3.50751671e-01},
    { 0.00000000e+00, -4.25303103e-02, -3.50751671e-01}};
    float b[21] = {-1.75547109, -1.0       , -7.37869444, -1.0        , -1.0        ,
    -3.26896372, -0.99999998,  2.14639408, -0.99999998, -0.99999998,
    9.00623992,  0.99996364,  1.0        , -1.0        ,  1.00000002,
    -1.00000001, -1.00000001, -6.40685225, -1.0        ,  1.00000002,
    1.00000002};
  } else {
    float w[21][3] = {{0.00000000e+00, -9.02715621e-01,  9.77491538e-02},
    { 4.94073640e-05,  8.14352677e-04,  1.61067184e-02},
    {-5.55835831e-01,  1.81048063e+00,  7.26009077e-02},
    { 4.94073640e-05,  8.14352677e-04,  1.61067184e-02},
    { 4.94073640e-05,  8.14352677e-04,  1.61067184e-02},
    { 3.20928416e-01, -1.25234128e+00,  3.83802961e-02},
    { 0.00000000e+00,  3.27482718e-03,  2.63506881e-02},
    { 2.09213132e-03,  6.71216087e-03, -6.43026232e-02},
    { 0.00000000e+00,  3.27482718e-03,  2.63506881e-02},
    { 0.00000000e+00,  3.27482718e-03,  2.63506881e-02},
    { 2.08232829e-01,  3.08854999e+00, -7.23585217e-02},
    { 7.67145641e-05, -4.60388263e-04, -1.23778546e-02},
    { 0.00000000e+00,  0.00000000e+00,  0.00000000e+00},
    { 0.00000000e+00,  0.00000000e+00,  0.00000000e+00},
    { 0.00000000e+00, -1.51181495e-03, -1.99164649e-02},
    {-4.65142726e-05,  4.60405933e-04,  1.23783297e-02},
    {-4.65142726e-05,  4.60405933e-04,  1.23783297e-02},
    { 1.99216265e-01, -2.24701324e+00,  5.75598824e-02},
    { 0.00000000e+00,  0.00000000e+00,  0.00000000e+00},
    { 0.00000000e+00, -1.51181495e-03, -1.99164649e-02},
    { 0.00000000e+00, -1.51181495e-03, -1.99164649e-02}};
    float b[21] = {-5.45398808,  -1.00003293, -26.88877061,  -1.00003293,
    -1.00003293,   1.32495099,  -1.00000001,   9.33491218,
    -1.00000001,  -1.00000001, -17.18962364,   0.99992325,
    1.0        ,  -1.0        ,   1.00000001,  -0.99998449,
    -0.99998449,   5.19839461,  -1.0        ,   1.00000001,
    1.00000001};
  } 
  float pred[21] = {};
  for(int i = 0; i < 21; i++) {
    pred[i] = w[i][0]*throwing_arm + w[i][1]*velocity + w[i][2]*spin_rate + b[i];
  }
  for(int j = 0; j < 21; j++) {    
    if(pred[j] > 0 and j < 6){
      votes[0] += 1;
    } else if(pred[j] < 0 and j == 0) {
        votes[1] += 1;
    } else if(pred[j] < 0 and j == 1) {
        votes[2] += 1;
    } else if(pred[j] < 0 and j == 2) {
        votes[3] += 1;
    } else if(pred[j] < 0 and j == 3) {
        votes[4] += 1;
    } else if(pred[j] < 0 and j == 4) {
        votes[5] += 1;
    } else if(pred[j] < 0 and j == 5) {
        votes[6] += 1;
    } else if(pred[j] > 0 and j >= 6 and j < 11) {
        votes[1] += 1;
    } else if(pred[j] < 0 and j == 6) {
        votes[2] += 1;
    } else if(pred[j] < 0 and j == 7) {
        votes[3] += 1;
    } else if(pred[j] < 0 and j == 8) {
        votes[4] += 1;
    } else if(pred[j] < 0 and j == 9) {
        votes[5] += 1;
    } else if(pred[j] < 0 and j == 10) {
        votes[6] += 1;
    } else if(pred[j] > 0 and j >= 11 and j < 15) {
        votes[2] += 1;
    } else if(pred[j] < 0 and j == 11) {
        votes[3] += 1;
    } else if(pred[j] < 0 and j == 12) {
        votes[4] += 1;
    } else if(pred[j] < 0 and j == 13) {
        votes[5] += 1;
    } else if(pred[j] < 0 and j == 14) {
        votes[6] += 1;
    } else if(pred[j] > 0 and j >= 15 and j < 18) {
        votes[3] += 1;
    } else if(pred[j] < 0 and j == 15) {
        votes[4] += 1;
    } else if(pred[j] < 0 and j == 16) {
        votes[5] += 1;
    } else if(pred[j] < 0 and j == 17) {
        votes[6] += 1;
    } else if(pred[j] > 0 and j >= 18 and j < 20) {
        votes[4] += 1;
    } else if(pred[j] < 0 and j == 18) {
        votes[5] += 1;
    } else if(pred[j] < 0 and j == 19) {
        votes[6] += 1;
    } else if(pred[j] > 0 and j == 20) {
        votes[5] += 1;
    } else {
        votes[6] += 1;
    }
  }
  int max_votes = 0;
  uint8_t class_label = 0;
  for(int i = 0; i < 21; i++) {
    // Serial.print("Votes: ");
    // Serial.println(votes[i]);
    // delay(1000);
    if(votes[i] > max_votes) {
      max_votes = votes[i];
      class_label = i;
    }
  }
  Serial.print("Pred: ");
  Serial.println(class_label);
  delay(1000);
  return class_label;
}
*/
