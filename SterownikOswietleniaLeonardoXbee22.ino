#include <Time.h>
#include <XBee.h>
#include <TimeAlarms.h>
#include <math.h>
#include <Wire.h>
#include <DS1307RTC.h>

#define Long 20.7971600
#define Lat 52.2103900
#define Req -0.833
#define XBee_Data_len 30
#define DIGITAL_PINS 13
#define ANALOG_PINS 5
#define DIGITAL_START_PIN 0x10
#define DIGITAL_END_PIN 0x1F
#define ANALOG_START_PIN 0x30
#define ANALOG_END_PIN 0x3F

XBee xbee = XBee();
XBeeResponse  response = XBeeResponse();


long XBee_Addr64_MS;
long XBee_Addr64_LS;
int XBee_Addr16;
XBeeAddress64 XBee_Addr64;


int dummy; 
int count=0;
int tLed1=4; //dioda czerwona
int tLed2=5; //dioda zolta
int tLed3=6; //dioda zielona
int swtch1 = 7; //przelacznik 1
int swtch2 = 8; //przelacznik 2
int swtch3 = 9; // do przyszlego wykorzystania
int swtch4 = 10; // do przyszlego wykorzystania
int swtch5 = 11; // do przyszłego wykorzystania
int in1 = 12; //do przyszłego wykorzystania
int in2 = 13; // do przyszlego wykorzystania

int charsReceived = 0;

String dsep="-";
String tsep=":";
String loffStr;
String lonStr;
String strDT="";

char XBee_Data[30];

boolean DEBUG=true;

//Reset urzadzenia
void(* resetFunc) (void) = 0;//declare reset function at address 0


////myBlink
void myBlink(int idx = 1, int times =1, int period = 100){

  if (times<=0){
    times=1;
  }
  for (int i=0; i<times; i++){
    myDigitalWrite(idx,HIGH);
    delay(period/times);
    myDigitalWrite(idx,LOW);
    delay(period/times);
  }
}


ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();
ZBTxStatusResponse txStatus = ZBTxStatusResponse();


//definicja funkcji debug
void debug(String str){
  if (DEBUG){
     Serial.println(str);
  }
}

void replay(String str){
   debug("replay");
   str=str+"="+strDT;
   debug(str);
   char myData[35];
   str.toCharArray(myData,sizeof(myData));

   XBeeAddress64 XBee_Addr64 = XBeeAddress64(0x0000000, 0x00000000);
   ZBTxRequest zbTx = ZBTxRequest(XBee_Addr64, XBee_Addr16, ZB_BROADCAST_RADIUS_MAX_HOPS, ZB_TX_UNICAST, (uint8_t*) myData, strlen(myData), xbee.getNextFrameId());
   xbee.send(zbTx);
   if (xbee.readPacket(500)) {
            // got a response!

            // should be a znet tx status            	
    	    if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
    	       xbee.getResponse().getZBTxStatusResponse(txStatus);
    	    	
    	       // get the delivery status, the fifth byte
               if (txStatus.getDeliveryStatus() == SUCCESS) {
                	// success.  time to celebrate
                 	debug("Delivery status SUCCESS");
               } else {
                	// the remote XBee did not receive our packet. is it powered on?
                 	debug("Delivery status FAILURE");
               }
            }      
        } else {
          // local XBee did not provide a timely TX Status Response -- should not happen
          debug("No responce from coordinator");
        }
        debug("end of replay");
}

void parseReceivedText(char *XBee_Data,int len){
  // look at first character and decide what to do
  debug("parseReceivedText");
  
  if (XBee_Data[0] == 0x00){
    resetFunc();
  }
    else if (XBee_Data[0] == 0x01){
      setDebug();
  }
  else if (XBee_Data[0] == 0x64){
    hardwareClockDisplay();
  }
  else if (XBee_Data[0] == 0x65){
    digitalClockDisplay();
  }
  else if (XBee_Data[0] == 0x66){
    showSun();
  }
  else if(XBee_Data[0] >= DIGITAL_START_PIN && XBee_Data[0] <= DIGITAL_END_PIN){
    if (len>1){
      writeDigitalPin(&XBee_Data[0]);
    }
    else{
      readDigitalPins(&XBee_Data[0]);
    }
  }
  else if(XBee_Data[0] >= ANALOG_START_PIN && XBee_Data[0] <= ANALOG_END_PIN){
     if (len>1){
      writeAnalogPin(&XBee_Data[0]);
    }
    else{
      readAnalogPins(&XBee_Data[0]);
    }
  }
  else {
    printErrorMessage("01"); 
  }
}

void setDebug(){
  if (DEBUG){
        DEBUG=false;
        replay("DB=OFF");
  }
  else{
        DEBUG=true;
        debug("Debug enabled");
        replay("DB=ON");
  }
}


void readDigitalPins(char *XBee_Data){
  // if we got here, XBee_Data[0] = 'd' and XBee_Data[1] = 'r'
  debug("readDigitalPins");
  int pin = XBee_Data[0]-DIGITAL_START_PIN;
  if (XBee_Data[0] == DIGITAL_END_PIN) {
  // output the valueof each digital pin
    for (int i = 0; i < DIGITAL_PINS; i++){ 
      PrintDigitalStatus(i);
    }
  }
  else {
    PrintDigitalStatus(pin);
  }
} 

void writeDigitalPin(char *XBee_Data){
  // if we got here, XBee_Data[0] = 'd' and XBee_Data[1] = 'w'
  debug("writeDigitalPin");
  int pin = XBee_Data[0]-DIGITAL_START_PIN;
        if(XBee_Data[1] == 0x00) {
          digitalWrite(pin, LOW);
          PrintDigitalStatus(pin);
        }
        else {
          digitalWrite(pin, HIGH);
          PrintDigitalStatus(pin);
        }
}

void readAnalogPins(char *XBee_Data){
  // if we got here, XBee_Data[0] = 'a' and XBee_Data[1] = 'r'
  // check XBee_Data[2] is a CR then
  // output the value of each analog input pin
  debug("readAnalogPins");
  int pin = XBee_Data[0]-ANALOG_START_PIN;
  if(XBee_Data[0] == ANALOG_END_PIN) {
    for (int i = 0; i < ANALOG_PINS; i++) {
      PrintAnalogStatus(i);
    }
  }
  else{
     PrintAnalogStatus(pin);
   }
}

void PrintAnalogStatus(int pin){
    replay("A"+String(pin)+"="+String(analogRead(pin)));
}


void writeAnalogPin(char *XBee_Data){
  // if we got here, XBee_Data[0] = 'a' and XBee_Data[1] = 'w'
  debug("writeAnalogPin");
  int pin = XBee_Data[0]-ANALOG_START_PIN;
  int pwmSetting = XBee_Data[0];
  if(pwmSetting >= 0 && pwmSetting <= 255) {
    analogWrite(pin,pwmSetting);
    PrintAnalogStatus(pin);
  }
  else {
    printErrorMessage("08"); //pwm value out of range
  }
}

void printErrorMessage(String errmsg)
{
  myBlink(1,1);
  replay("ER="+errmsg);
}


void PrintDigitalStatus(int pin){
    replay("D"+String(pin)+"="+digitalRead(pin));
}



void myDigitalWrite(int idx,int state){
  //tLed1=4 dioda czerwona
  //tLed2=5 dioda zolta
  //tLed3=6 dioda zielona
  switch(idx){
    case 1: //czerwona
      digitalWrite(tLed1,state);
      break;
     case 2: //zolta
      digitalWrite(tLed2,state);
      break;
     case 3: //czerwona & zolta
      digitalWrite(tLed1,state);
      digitalWrite(tLed2,state);
      break;
     case 4: // zielona
       digitalWrite(tLed3,state);
       break;
     case 5: //czerwona i zielona
      digitalWrite(tLed1,state);
      digitalWrite(tLed3,state);
      break;
     case 6: //zolta i zielona
      digitalWrite(tLed2,state);
      digitalWrite(tLed3,state);
     case 7: // wszystkie
      digitalWrite(tLed1,state);
      digitalWrite(tLed2,state);
      digitalWrite(tLed3,state);
      break;
      default:
        break;
  }
}


int * sunInt2TimeDigits(float sunR){
  //Converts time in int to array of ret[]={hh,mm,ss}
  static int ret[3];
  int Sec;
  
  ret[0]=(int)sunR;
  Sec=(sunR-ret[0])*3600;
  ret[1]=Sec/60;
  ret[2]=(Sec-(ret[1]*60));
  
  return ret;
  
}

float * calcSunInt(int Rok, int M, int D){
  //Calculates sun rise ret[0] and sun set ret[1] as float
  static float ret[2];

  float J=367L*Rok-int(7*(Rok+int((M+9)/12))/4)+int(275*M/9)+D-730531.5;
  float Cent=J/36525;
  float L=fmod((4.8949504201433+628.331969753199*Cent),6.28318530718);
  float G = fmod((6.2400408+628.3019501*Cent),6.28318530718);
  float O = 0.409093-0.0002269*Cent;
  float F = 0.033423*sin(G)+0.00034907*sin(2*G);
  float E = 0.0430398*sin(2*(L+F)) - 0.00092502*sin(4*(L+F)) - F;
  float A = asin(sin(O)*sin(L+F));
  float C = (sin(0.017453293*Req)-sin(0.017453293*Lat)*sin(A))/(cos(0.017453293*Lat)*cos(A));
  
  ret[0] = (PI - (E+0.017453293*Long+1*acos(C)))*57.29577951/15; // Sun Rise
  ret[1] = (PI - (E+0.017453293*Long+(-1)*acos(C)))*57.29577951/15; //Sun Set
  
  return ret;
}

void setTimeRTC(){
  time_t t=0;
   
  while (t<=0){
    printErrorMessage("11"); 
    //myBlink(1,3);
    delay(1000);
    t=RTC.get();
  }
  setTime(t);
  //myBlink(4,3, 500); // zielona 3 razy
  setAlarm();
}

void digitalClockDisplay(){
  debug("digitalClockDisplay");
  String msg="TD="+String(year())+dsep+print2digits(month())+dsep+print2digits(day())+" "+hour()+tsep+print2digits(minute())+tsep+print2digits(second());
  replay(msg); 
}

void hardwareClockDisplay(){
  debug("hardwareClockDisplay");
  tmElements_t tm;
  if (RTC.read(tm)){
    //String msg="msgdt"+tmYearToCalendar(tm.Year)+dsep+print2digits(tm.Month)+dsep+print2digits(tm.Day)+" "+tm.Hour+tsep+print2digits(tm.Minute)+tsep+print2digits(tm.Second);
    String msg="TH="+String(tmYearToCalendar(tm.Year))+dsep+print2digits(tm.Month)+dsep+print2digits(tm.Day)+" "+tm.Hour+tsep+print2digits(tm.Minute)+tsep+print2digits(tm.Second);
    replay(msg);
  }
  else{
    printErrorMessage("12"); //Unable to read RTC
  }
}

String print2digits(int number) {
  String str="";
  if (number >= 0 && number < 10) {
    str="0"+String(number);
  }
  else{
    str=String(number);
  }
  return str;
}

void setAlarm(){
  
  float *SunSR;
  int *sunRise,*sunSet;
  int HR,HS,MR,MS,SR,SS;
  
  SunSR=calcSunInt(year(),month(),day());
  
  sunRise=sunInt2TimeDigits(*SunSR);
  
  HR=*sunRise;
  MR=*(sunRise+1);
  SR=*(sunRise+2);
  
 sunSet=sunInt2TimeDigits(*(SunSR+1)); 
  HS=*sunSet;
  MS=*(sunSet+1);
  SS=*(sunSet+2);
  
  Alarm.alarmOnce(HR,MR,SR, lightsOff);
  Alarm.alarmOnce(HS,MS,SS, lightsOn);
  Alarm.alarmRepeat(0,0,10,setAlarm);
  //Alarm.timerRepeat(5, pulseGreen);
  loffStr=print2digits(HR)+":"+print2digits(MR)+":"+print2digits(SR);
  lonStr=print2digits(HS)+":"+print2digits(MS)+":"+print2digits(SS);
  showSun();
}

void pulseGreen(){
  myBlink(4,1,100);
}

void showSun(){
  debug("showSun");
  hardwareClockDisplay();
  digitalClockDisplay();
  replay("LF="+loffStr);
  replay("LO="+lonStr);
}

void lightsOff(){
  debug("lightsOff");
  digitalWrite(swtch1, LOW);
}

void lightsOn(){
  debug("lightsOn");
  digitalWrite(swtch1, HIGH);
}


void setup() {
  Serial.begin(9600);
  Serial1.begin(38400);
  xbee.begin(Serial1);
  if(DEBUG){
    while(!Serial){
    }
    debug("started");
  }
  pinMode(swtch1, OUTPUT);  
  pinMode(swtch2, OUTPUT);
  pinMode(tLed1, OUTPUT);
  pinMode(tLed2, OUTPUT);
  pinMode(tLed3, OUTPUT);
  pinMode(swtch3, OUTPUT);
  pinMode(swtch4, OUTPUT);
  pinMode(swtch5, OUTPUT);
  pinMode(in1, INPUT);
  pinMode(in2, INPUT);
  myBlink(7,1,500); // Zapal 3x wszystkie diody 
  if (timeStatus() == timeNotSet){
    setTimeRTC();
  }  
}

void loop() {
  memset(&XBee_Data[0], 0, sizeof(XBee_Data));
  xbee.readPacket();
  if (xbee.getResponse().isAvailable()){
    debug("*");
    if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE){
      myBlink(2,1);
      xbee.getResponse().getZBRxResponse(rx);
      /*
      XBee_Addr64_MS=(uint32_t(rx.getFrameData()[0]) << 24) + (uint32_t(rx.getFrameData()[1]) << 16) + (uint16_t(rx.getFrameData()[2]) << 8) + rx.getFrameData()[3];
      XBee_Addr64_LS=(uint32_t(rx.getFrameData()[4]) << 24) + (uint32_t(rx.getFrameData()[5]) << 16) + (uint16_t(rx.getFrameData()[6]) << 8) + rx.getFrameData()[7];
      XBee_Addr64 = XBeeAddress64(XBee_Addr64_MS, XBee_Addr64_LS);
      XBee_Addr16=rx.getRemoteAddress16();
      */

      if (rx.getDataLength()>=XBee_Data_len){
        dummy=XBee_Data_len;
      }
      else {
        dummy=rx.getDataLength();
      }
      if (dummy>=19){
        for (count=0;count<=dummy-1;count++){
              XBee_Data[count]=rx.getData(count);
            }
         for (count=0;count<=18;count++){
           strDT+=XBee_Data[count];
         }
         parseReceivedText(&XBee_Data[19],dummy);
      }
      else{
        debug("Payload to short");
        printErrorMessage("99");
      }
    }
    else if (xbee.getResponse().getApiId() == MODEM_STATUS_RESPONSE) {
        xbee.getResponse().getModemStatusResponse(msr);
        // the local XBee sends this response on certain events, like association/dissociation
        
        if (msr.getStatus() == ASSOCIATED) {
          // yay this is great.  flash led
          debug("ACCOCIATED");
        } else if (msr.getStatus() == DISASSOCIATED) {
          // this is awful.. flash led to show our discontent
          debug("DISACCOCIATED");
        } 
     else {
          // another status
          debug("Another status");
        }
      } 
   else {
      	// not something we were expecting
        debug("Unexpected XBee error");    
    }
  }
  myBlink(2,1,1);
  Alarm.delay(1);
}


