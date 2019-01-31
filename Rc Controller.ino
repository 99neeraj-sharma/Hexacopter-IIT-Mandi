int aileron=2, elevator=3, throttle=4, rudder=5, aux=6;
int ch[5];
int stime;
void setup() {
  pinMode(aileron,INPUT);
  pinMode(elevator,INPUT);
  pinMode(throttle,INPUT);
  pinMode(rudder,INPUT);
  pinMode(aux,INPUT);
  Serial.begin(9600);
}

void loop() {
    while(digitalRead(aileron) == LOW){}
    stime = micros();
    ch[0] = calc(stime, 0);
    Serial.print("ch1: ");
    Serial.println(ch[0]);
    
    while(digitalRead(elevator) == LOW){}
    stime = micros();
    ch[1] = calc(stime, 1);
    Serial.print("ch2: ");
    Serial.println(ch[1]);
    
    while(digitalRead(throttle) == LOW){}
    stime = micros();
    ch[2] = calc(stime, 2);
    Serial.print("ch3: ");
    Serial.println(ch[2]);
    
    while(digitalRead(rudder) == LOW){}
    stime = micros();
    ch[3] = calc(stime, 3);
    Serial.print("ch4: ");
    Serial.println(ch[3]);
    
    while(digitalRead(aux) == LOW){}
    stime = micros();
    ch[4] = calc(stime, 4);
    Serial.print("ch5: ");
    Serial.println(ch[5]);
    delay(1000);

}
int calc(int stime, int var){
  if(var == 0){
    while(digitalRead(aileron) == HIGH){}
    return int(micros() - stime);
  }
  else if(var == 1){
    while(digitalRead(elevator) == HIGH){}
    return int(micros() - stime);
  }
  if(var == 2){
    while(digitalRead(throttle) == HIGH){}
    return int(micros() - stime);
  }
  if(var == 3){
    while(digitalRead(rudder) == HIGH){}
    return int(micros() - stime);
  }
  if(var == 4){
    while(digitalRead(aux) == HIGH){}
    return int(micros() - stime);
  }
}
