//OPENLOG FUNCTIONS

void createFile(char* fname)
{
  Serial.print("in!");
  Serial3.write(26); // sending ASCII character ctrl+z three times to datalogger puts it into command mode so that new files may be created.
  Serial3.write(26); 
  Serial3.write(26); 
  char buff[30]; //Buffer to store both 8-character file name, .xxx extension, and OpenLog system command
 // char dbuff[80]; // Buffer to store date string
  
  Serial.print("in while");
  while(1) {
    //Serial.print("in while");
    if(Serial3.available())
      if(Serial3.read() == '>') break;
  }
  Serial.print("1");

  sprintf(buff,"new %s.txt\r",fname);
  Serial3.print(buff);
  
   while(1) {
    if(Serial3.available())
      if(Serial3.read() == '>') break;
  }
  Serial.print("2");
  
  sprintf(buff, "append %s.txt\r", fname);
  Serial3.print(buff);
  
  while(1) {
    if(Serial3.available())
      if(Serial3.read() == '<') break;
  }

Serial.print("out!");
  
 // sprintf(dbuff,"H.A.R.M. High Altitude Research Module: \r\nFlight Date %s",dateb);
}


//Initialize datalogger; create logging files. See createFile().
void openLogInit(){
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
  //Serial.print("in AF");
  
  char buff[200];
  
  gps.get_datetime(&date, &time, &fix_age); //Refresh current date and time 
  gps.crack_datetime(&year, &month, &day,
  &hour, &minutes, &second, &hundredths, &fix_age);

  Serial3.write(26); // sending ASCII character ctrl+z three times to datalogger puts it into command mode so that new files may be created.
  Serial3.write(26); 
  Serial3.write(26); 
  
  while(1) {
    if(Serial3.available())
      if(Serial3.read() == '>') break;
  }
  
  sprintf(buff,"append %s",editFile);
  Serial3.print(buff);
  
  while(1) {
    if(Serial3.available())
      if(Serial3.read() == '<') break;
  }
 
  //Optional timestamp
  if(timestamp){
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d   ",
        month, day, year, hour, minutes, second);
    Serial3.print(sz);
  }
  
  Serial3.print(logData);
  Serial3.print(" ");
  
  //Optional newline character
  if(newline){
    Serial3.print("\r\n");
  }
    
}

void recordEvent(char* eventData){
  //Serial.print("in RE");
  appendFile(eventData, "SYSEVENT.txt", 1,1);
}
