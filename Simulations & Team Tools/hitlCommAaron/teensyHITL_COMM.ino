void setup() {
  Serial.begin(115200);          // Start serial communication
}

void loop() {
  // reads the string that .py sends
  String data = Serial.readStringUntil('\n');
  String ex, why, zee;

  // uses substring to get first value
  int firstStart = 0;
  int firstEnd = data.indexOf(',');
  ex = data.substring(firstStart, firstEnd);

  // uses substring to get second value
  int secondStart = firstEnd + 1;
  int secondEnd = data.indexOf(',', secondStart);
  why = data.substring(secondStart, secondEnd);
  
  // uses substring to get third value
  int thirdStart = secondEnd + 1;
  int thirdEnd = data.indexOf(' ', thirdStart);
  zee = data.substring(thirdStart, thirdEnd);

  /* 
  can use .toFloat(), but keeping substring as a string for the sake of this form of output
  once we start using an actual simulation, we can change the output type from string to whatever is needed
  */
  String output = "Current x pos: " + ex + ", Current y pos: " + why + ", Current z pos: " + zee;
  Serial.println(output);
  delay(95);
}