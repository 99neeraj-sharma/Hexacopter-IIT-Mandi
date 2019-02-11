const int trigPin1 = 9;
const int echoPin1 = 10;
const int trigPin2 = 1;
const int echoPin2 = 2;
const int trigPin3 = 3;
const int echoPin3 = 4;
const int trigPin4 = 5;
const int echoPin4 = 6;
long duration;
int distance;

void setup() {
pinMode(trigPin, OUTPUT);
pinMode(echoPin, INPUT);
Serial.begin(9600);
}

void loop() {
digitalWrite(trigPin1, LOW);
delayMicroseconds(2);
digitalWrite(trigPin1, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin1, LOW);
duration = pulseIn(echoPin1, HIGH);
distance1= duration*0.034/2;
Serial.print("Distance1: ");
Serial.println(distance1);

digitalWrite(trigPin2, LOW);
delayMicroseconds(2);
digitalWrite(trigPin2, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin2, LOW);
duration = pulseIn(echoPin2, HIGH);
distance2= duration*0.034/2;
Serial.print("Distance1: ");
Serial.println(distance2);

digitalWrite(trigPin3, LOW);
delayMicroseconds(2);
digitalWrite(trigPin3, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin3, LOW);
duration = pulseIn(echoPin3, HIGH);
distance3= duration*0.034/2;
Serial.print("Distance3: ");
Serial.println(distance3);


digitalWrite(trigPin4, LOW);
delayMicroseconds(2);
digitalWrite(trigPin4, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin4, LOW);
duration = pulseIn(echoPin4, HIGH);
distance4= duration*0.034/2;
Serial.print("Distance4: ");
Serial.println(distance4);

}
