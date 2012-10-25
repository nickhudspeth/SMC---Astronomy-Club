



static void gpsdump(TinyGPS &gps);
static bool feedgps();
static void print_float(float val, float invalid, int len, int prec);
static void print_int(unsigned long val, unsigned long invalid, int len);
static void print_date(TinyGPS &gps);
static void print_str(const char *str, int len); //MAY NEED TO REDECLARE CHAR AS CONST CHAR


//GPS
void gpsInit(){
  
  gpsRefresh();
  
  while(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES)
  {
    digitalWrite(gps_no_fix_indicator, HIGH); //Turn on indicator while acquiring GPS fix
  }
  
  digitalWrite(gps_no_fix_indicator, LOW);
  HOME_LAT = flat;
  HOME_LON = flon;
  
  char obj_str [40];
  sprintf(obj_str,"Sizeof(gpsobject) = %d",sizeof(TinyGPS));
  appendFile(obj_str,"GPS.txt",0,1); 
  appendFile("Sats HDOP Latitude Longitude Fix  Date       Time       Date Alt     Course Speed Card  Distance Course Card  Chars Sentences Checksum","GPS.txt",0,1);
  appendFile("          (deg)    (deg)     Age                        Age  (m)     --- from GPS ----  ---- to Home    ----  RX    RX        Fail","GPS.txt",0,1);
  appendFile("--------------------------------------------------------------------------------------------------------------------------------------","GPS.txt",0,1);
}

void gpsRefresh(){
   bool newdata = false;
   
   if (feedgps()){
      newdata = true;
   }
   
   gpsdump(gps);
  
   gps.get_datetime(&date, &time, &fix_age);
   gps.crack_datetime(&year, &month, &day,&hour, &minutes, &second, &hundredths, &fix_age);
   gps.get_position(&lat, &lon, NULL);
   falt = gps.f_altitude(); // +/- altitude in meters
   fc = gps.f_course(); // course in degrees
   fk = gps.f_speed_knots(); // speed in knots
   fmph = gps.f_speed_mph(); // speed in miles/hr
   fmps = gps.f_speed_mps(); // speed in m/sec
   fkmph = gps.f_speed_kmph(); // speed in km/hr
   
   fdist = TinyGPS::distance_between(flat, flon, HOME_LAT, HOME_LON);
  //cardinal = TinyGPS::cardinal(TinyGPS::course_to(flat, flon, HOME_LAT, HOME_LON)); UNCOMMENT AND FIX ME
   
   if(fmph > 60)
   {
     current_mode = freefall_mode;
   }
}





//void loop()
//{
//  bool newdata = false;
//  unsigned long start = millis();
//  
//  // Every second we print an update
//  while (millis() - start < 1000)
//  {
//    if (feedgps())
//      newdata = true;
//  }
//  
//  gpsdump(gps);
//}

static void gpsdump(TinyGPS &gps)
{
  
  
  print_int(gps.satellites(), TinyGPS::GPS_INVALID_SATELLITES, 5);
  print_int(gps.hdop(), TinyGPS::GPS_INVALID_HDOP, 5);
  gps.f_get_position(&flat, &flon, &fix_age);
  print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 9, 5);
  print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 10, 5);
  print_int(fix_age, TinyGPS::GPS_INVALID_AGE, 5);

  print_date(gps);

  print_float(gps.f_altitude(), TinyGPS::GPS_INVALID_F_ALTITUDE, 8, 2);
  print_float(gps.f_course(), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2);
  print_float(gps.f_speed_kmph(), TinyGPS::GPS_INVALID_F_SPEED, 6, 2);
  print_str(gps.f_course() == TinyGPS::GPS_INVALID_F_ANGLE ? "*** " : TinyGPS::cardinal(gps.f_course()), 6);
  print_int(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0UL : (unsigned long)TinyGPS::distance_between(flat, flon, HOME_LAT, HOME_LON) / 1000, 0xFFFFFFFF, 9);
  print_float(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : TinyGPS::course_to(flat, flon, 51.508131, -0.128002), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2);
  print_str(flat == TinyGPS::GPS_INVALID_F_ANGLE ? "*** " : TinyGPS::cardinal(TinyGPS::course_to(flat, flon, HOME_LAT, HOME_LON)), 6);

  gps.stats(&chars, &sentences, &failed_checksum);
  print_int(chars, 0xFFFFFFFF, 6);
  print_int(sentences, 0xFFFFFFFF, 10);
  print_int(failed_checksum, 0xFFFFFFFF, 9);
  appendFile("\n","GPS.txt",0,0); // Print newline character at the end of the GPS dump.
  //Serial.println();
}

static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  appendFile(sz,"GPS.txt",0,0);
  feedgps();
}

static void print_float(float val, float invalid, int len, int prec)
{
  char sz[32];
  if (val == invalid)
  {
    strcpy(sz, "*******");
    sz[len] = 0;
        if (len > 0) 
          sz[len-1] = ' ';
    for (int i=7; i<len; ++i)
        sz[i] = ' ';
    
  }
  else
  {
    sprintf(sz,"%f",val);
    appendFile(sz,"GPS.txt",0,0);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1);
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      appendFile(" ","GPS.txt",0,0);
  }
  feedgps();
}

static void print_date(TinyGPS &gps)
{

  gps.crack_datetime(&year, &month, &day, &hour, &minutes, &second, &hundredths, &fix_age);
  if (fix_age == TinyGPS::GPS_INVALID_AGE)
    appendFile("*******    *******    ","GPS.txt",0,0);
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d   ",
        month, day, year, hour, minutes, second);
    appendFile(sz,"GPS.txt",0,0);
  }
  print_int(fix_age, TinyGPS::GPS_INVALID_AGE, 5);
  feedgps();
}

static void print_str(const char *str, int len) //MAY NEED TO REDECLARE CHAR IN ARUMENT DESCRIPTION AS CONST CHAR
{
  char buff[500];
  strcpy(buff,str);
  appendFile(buff,"GPS.txt",0,0);
  appendFile(" ","GPS.txt",0,0);
//  int slen = strlen(str);
//  for (int i=0; i<len; ++i)
//    Serial.print(i<slen ? str[i] : ' ');
  feedgps();
}

static bool feedgps()
{
  while (Serial2.available())
  {
    if (gps.encode(Serial2.read()))
      return true;
  }
  return false;
}

