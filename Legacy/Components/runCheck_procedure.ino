void runCheck(){
  while (digitalRead(7) == LOW){}
  if (digitalRead(8) == HIGH){
    Serial.write("Run 1");
    turnBias = 0;}
  else if (digitalRead(12) == HIGH){
    Serial.write("Run 2");
    turnBias = 1;}
  else if (digitalRead(13) == HIGH){
    Serial.write("Run 1");
    if(deadEnd == 0){
      turnBias = 1;}
    else if(deadEnd == 1){
      turnBias = 0;}}
  }
