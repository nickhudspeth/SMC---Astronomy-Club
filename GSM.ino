void gsmInit(){
  int gsm = 0;
  while(gsm = 0){
//  if (SerialReadUntil("+SIND: 4", 45*1000)) 
//  {
//    digitalWrite(gsm_radio_not_ready_indicator, HIGH);
//  } 
//  else 
//    {
//    digitalWrite(gsm_radio_not_ready_indicator,LOW);
//    gsm = 1;
    }
  }



void gsm_send_message(char* message)
{
  char buffer[600];
  sprintf(buffer,"AT+CMGS=%s%s",number,message); 
//  Serial.print("AT+CMGS=");
//  Serial.print(number); //Phone number
//  Serial.print(message);
  Serial.write(26); //Append CTRL-Z Character
  Serial.flush();
}  

//void gsm_get_signal_strength()
//{
//  char inputArray[15];
//  int index = 0;
//  
//  Serial.print("AT+CHQ");
//  if(Serial.available() > 0 ){
//  
//    inputArray[index] = Serial.read();
//    index++;
//  while(inputArray[(index-1)] != '\n' && index < 15)
//  {
//    //YOU MAY NEED TO CHANGE (index-1) to (index-2)
//    inputArray[index] = Serial.read();
//    index++;
//  }
//  
//    sscanf(second,"+CSQ: %d,99",&signal_strength);
//    
//  }
//}
  
  


void gsm_send_param(int param)
{
  //Create lookup table to associate incoming SMS string with integers
  String data;
  char buff1[300];
  //char buff2[500];
  
  switch(param){
    case 1:
       // "Location?" Command sent
      sprintf(buff1,"Current Location: Latitude = %f Longitude = %f Altitude = %f",flat,flon,falt);
      break;
      
    case 2:
      // "Latitude" Command sent
      sprintf(buff1,"Current Latitude = %f",flat);
      break;
          
    case 3:
      //"Longitude" Command sent
      sprintf(buff1,"Current Longitude = %f",flon);
      break;
      
    case 4:
      //"Altitude" Command sent
      sprintf(buff1,"Current Altitude = %f",falt);
      break;
    
    case 5:
      //"Distance" Command sent
      sprintf(buff1,"Current distance to launch point = %f meters. %c",(unsigned long)TinyGPS::distance_between(flat, flon, HOME_LAT, HOME_LON),TinyGPS::cardinal(TinyGPS::course_to(flat, flon, HOME_LAT, HOME_LON)));
      break;
      
    case 6:
      //"Environment" Command sent
      sprintf(buff1,"Current Environmental Conditions: \n External Temp = %f\nHumidity= %d\nPressure= %f\nAmbient Light Level= %d\nCarbon Monoxide Level= %d\n",e_temp_f,humidity,pressure,ambient_light,carbon_monoxide);
      break;
      
    case 7:
      //"External Temperature" Command sent
      sprintf(buff1,"External temperature = %d degrees F.",e_temp_f);
      break;
      
    case 8:
      //"Humidity" Command sent
      sprintf(buff1,"Humidity = %d percent RF.",humidity);
      break;
      
    case 9:
      //"Pressure" Command sent
      sprintf(buff1,"Current Pressure = %d kPa.",pressure);
      break;
      
    case 10:
      //"Ambient Light" Command sent
      sprintf(buff1,"Current ambient light level = %d/1024.",ambient_light);
      break;
    
    case 11:
      //"Carbon Monoxide Level" Command sent
      sprintf(buff1,"Current Carbon Monoxide Level = %d/1024.",carbon_monoxide);
      break;
    case 12:
      //"System" Command sent
      sprintf(buff1,"Current System Conditions: \n Internal Temp= %f\n Flight Mode = %d\n GSM Signal Strength = %d\n Smoke Detect = %d",i_temp_f,current_mode,(signal_strength/31)*100,smoke);
      //data = String("Current System Conditions:" + "\nInternal Temperature = " + i_temp_f + "degrees F" + "\nFlight Mode = " + current_mode + "\nGSM Signal Strength = " + ((signal_strength/31)*100) + " percent" + "\nSmoke Detect = " + smoke);
      break;
    case 13:
      //"Summary" Command sent
      break;
    
    case 14:
      //"Cutdown" Command sent
      gps.crack_datetime(&year, &month, &day, &hour, &minutes, &second, &hundredths, &fix_age);
      char sz[100];
      sprintf(sz, "User initiated cutdown sequence started at %02d/%02d/%02d %02d:%02d:%02d   ",
        month, day, year, hour, minutes, second);
      recordEvent(sz);
      tetherRelease();
      break;
      
    case 15:
      //"Internal Temperature" Command sent
      getITemp();
      sprintf(buff1,"Internal temperature = %f degrees F",i_temp_f);
      break;
    
    case 16:
      data = String("Current flight mode = " + current_mode);
      break;
      
    case 17:
     // gsm_get_signal_strength();
      int db_level = map(signal_strength,0,31,-117,-48);
      sprintf(buff1,"Current GSM Signal Strength = %d dB",db_level);
      break;
      
  }
  
  gsm_send_message(buff1);
  
}

  
  void transmit(){
    
    
  }
  
  void gsmCheck(){
  }
  
