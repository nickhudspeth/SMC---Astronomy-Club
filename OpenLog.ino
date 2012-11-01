//OPENLOG FUNCTIONS

/*
  OpenLog device is on Serial3. Serial(0) is the IDE debug window.

*/

void createFile(char *fname)
{
    //Reset OpenLog
  digitalWrite(openlog_reset, LOW);
  delay(20);
  digitalWrite(openlog_reset, HIGH);

  //char dateb[]= "11/11/11";
  char buff[50]; //Buffer to store both 8-character file name, .xxx extension, and OpenLog system command
  
  //Serial.println("in!");
  Serial3.write(26); // sending ASCII character ctrl+z three times to datalogger puts it into command mode so that new files may be created.
  Serial3.write(26); 
  Serial3.write(26); 
  
 char dbuff[80]; // Buffer to store date string
  
while(1) {
    if(Serial3.available())
      if(Serial3.read() == '>'){
      Serial.println("Openlog responsive.");
           break; }
//      else
//      {
//          Serial3.write(26); // sending ASCII character ctrl+z three times to datalogger puts it into command mode so that new files may be created.
//          Serial3.write(26); 
//          Serial3.write(26);
//      } 
  }  
  
  sprintf(buff, "new %s.txt\r", fname);
  Serial.println(buff);
  Serial3.print(buff);
  
   while(1) {
    if(Serial3.available())
      if(Serial3.read() == '>') {
        Serial.println("Openlog ready for append command.");
        break;
      }
  }
  
  Serial3.flush();
  sprintf(buff, "append %s.txt\r", fname);
  Serial3.print(buff);
  Serial.println(buff);
  
while(1) {
    if(Serial3.available()){     
      if(Serial3.read() == '<'){
        Serial.println("Openlog appending...");
           break; 
      }
  }  
}

  Serial.println(" created.");

//Serial.print("out!");
 Serial3.println("H.A.R.M. High Altitude Research Module:");
 sprintf(dbuff,"Flight Date %s",dateb);
 Serial3.print(dbuff);
 Serial3.println();
 Serial3.println();
 
 //Serial3.write(26); //close append mode
}

void openLogInit(){
  //Initialize datalogger; create logging files. See createFile().
  createFile("INTETEMP");
  createFile("EXTETEMP");
  createFile("HUMIDITY");
  createFile("PRESSURE");
  createFile("AMBILGHT");
  createFile("SYSEVENT");
  createFile("ALTITUDE");
  createFile("GPS");
  digitalWrite(datalogger_not_ready_indicator, LOW);
  }

void appendFile(char *logData, char *editFile, int timestamp, int newline){
  Serial.println("in appendFile.");
  char buff[50];
  char dbuff[200];
  sprintf(buff,"append %s",editFile);
  
  if(newline){
    sprintf(dbuff,"%s ",logData);
  }
  else
  {
    sprintf(dbuff,"%s \n",logData);
  }
//  gps.get_datetime(&date, &time, &fix_age); //Refresh current date and time 
//  gps.crack_datetime(&year, &month, &day,
//  &hour, &minutes, &second, &hundredths, &fix_age);

      //Reset OpenLog
  digitalWrite(openlog_reset, LOW);
  delay(100);
  digitalWrite(openlog_reset, HIGH);
  while(1){
    if(Serial.available())
    {
      if(Serial.read() == '<')break; // Wait for openlog to indicate that it is alive and recording to a file
    }
  }
 // Serial.println("Openlog reset.");
  Serial3.write(26); // sending ASCII character ctrl+z three times to datalogger puts it into command mode so that new files may be created.
  Serial3.write(26); 
  Serial3.write(26); 
  Serial.println("Escape chars sent.");
  
  
while(1) {
    if(Serial3.available()){
      if(Serial3.read() == '>'){
      Serial.println("Openlog ready for append command.");
           break; }
//      else
//      {
//          Serial3.write(26); // sending ASCII character ctrl+z three times to datalogger puts it into command mode so that new files may be created.
//          Serial3.write(26); 
//          Serial3.write(26);
//      } 
  }  
}
  //Serial3.flush();
  Serial.print(buff);
   Serial3.println(buff);
   
  
while(1) {
    if(Serial3.available()){     
      if(Serial3.read() == '<'){
        Serial.println("Openlog appending..."); // wait for openlog to indicate file is open and ready for writing
           break; 
      }
  }  
}
 
  //Optional timestamp
//  if(timestamp){
//    char sz[32];
//    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d   ",
//        month, day, year, hour, minutes, second);
//    Serial3.print(sz);
//  }
  
  Serial3.print(logData);

  }


void recordEvent(char* eventData){
  Serial.print("Recording event ");
  Serial.println(eventData);
  appendFile(eventData, "SYSEVENT.txt", 1,1);
}
