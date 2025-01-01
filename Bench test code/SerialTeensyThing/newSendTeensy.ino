void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.println("I am sender teensy");
}

void loop() {
  //Serial1.write("Hello receiverTeensy");
  String message = "Hello receiverTeensy";
  // creating a char[] to write back
    int length = message.length() + 1;
    char str[length];
    for (int i = 0; i < length; i++) {
      str[i] = message.charAt(i);
    }
    str[message.length()] = '\n';
    
    Serial1.write(str, length);
  Serial1.end(); 
  Serial1.begin(115200);
  //does nothing until data is detected
  while (Serial1.available() == 0) {
    Serial.println("Waiting for data");
    delay(500);
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
  }
  delay(500);
}