void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial.println("I am receiver teensy");
}

void loop() {
  //does nothing until data is detected
  while (Serial1.available() == 0) {
    Serial.println("Waiting for data");
    delay(1000);
  }
  readAndSpit();
}

void readAndSpit() {
//Serial.available will return an int if it has received data
  if (Serial1.available() > 0) {
    Serial.println("Received!");
    String message = Serial1.readStringUntil('\n');  // Read the incoming message
    Serial.print("Message from SenderTeensy: ");
    Serial.println(message);
    // creating a char[] to write back
    int length = message.length();
    char str[length];
    for (int i = 0; i < length; i++) {
      str[i] = message.charAt(i);
    }
    Serial1.write(str, length);
    delay(1000);
  }
}
