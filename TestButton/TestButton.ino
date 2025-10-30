int button1 = 6;
int buttonRead;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(button1,INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
  buttonRead = digitalRead(button1);
  Serial.print("is pressed?: ");
  Serial.println(buttonRead);
  delay(200);
}
