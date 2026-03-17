const int enablePin = 2;
const int directionPin= 3;

void setup() {
  pinMode(enablePin, OUTPUT);
  pinMode(directionPin, OUTPUT);
}

void loop() {
  digitalWrite(enablePin, HIGH);
  digitalWrite(directionPin, HIGH);
  delay(1000);
  digitalWrite(enablePin, LOW);
  delay(1000);

}
