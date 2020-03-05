#include <Wire.h>                   // I2C interface library
#include <Adafruit_BMP280.h>        // Piezo Tone Library
#include "U8glib.h"
#include <EEPROM.h>

#include <VoltageReference.h>
#include <SimpleKalmanFilter.h>

const char *hex = "0123456789ABCDEF";
char t_check[3];
char newl[2] = "\n";

float measurementConstant = 0.2;
float estimationConstant = 0.2;
float noiseConstant = 0.05;
float tempo = 0.0;

// for reading non important sensors for every second 
unsigned long get_time_second = millis();

bool beepOn = true;
bool refAltitudeOn = false;

SimpleKalmanFilter varioKalmanFilter(measurementConstant, estimationConstant, noiseConstant);
//SimpleKalmanFilter altitudeKalmanFilter(measurementConstant, estimationConstant, noiseConstant);

VoltageReference vRef;

#define tone_out1 8
#define tone_out2 9

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NO_ACK);  // Display which does not send AC
Adafruit_BMP280 sensor_bmp;

float vario_down = -2.3;              
float vario_emergence = -5.1;  
float vario_up = 0.25;
float alt[51];
float tim[51];
float beep;
float Beep_period;
float mux = 0.25;
float muxTmp = 0.25;

float vario = 0.0;

float altitude = 0.0;
float altitude2 = 0.0;
float refAltitude2 = 0.0;
float temperature = 0.0;
unsigned long bounseInput4P = 0UL;
unsigned long time;

unsigned char samples=10;
unsigned char maxsamples=50;
unsigned char countPressVal = 0;
unsigned char screen = 0;
unsigned char menu = 2;
unsigned char memo = 1;


//button
unsigned char buttonState;
unsigned long duration;
unsigned long resettime;
unsigned char buttonStatus=0;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 20; //debounce button


//clock
unsigned char m_clock = 0;
unsigned char m_second = 0, m_minute = 0, m_hour = 0;
unsigned long UtlTime;
unsigned long lastMillis;


//variables to confirm takeoff
unsigned long temp_alt;
unsigned char temp_verif_alt = 0;
unsigned char ult_altitude;

//Variables Memory Recording
int num_memo = 0;
float Mvarioup = 00.0;
float Mvariodown = 00.0;
float MaltitudeMax = 0000.0;
float MaltitudeDec = 0000.0;
float m_gain = 0000.0;
float m_landing = 0000.0;
int endfly;
int end_memo = 1;
unsigned long m_savetime;
int memory = 1;

//Variables to Read Memories
float Rvarioup=00.0;
float Rvariodown=00.0;
float RaltitudeMax=0000.0;
float Rgain=0000.0;
unsigned char Rm_hour;
unsigned char Rm_minute;
int Rend_memo=1; //read memory
int Rend_memo_display=1;
unsigned char Nmeno_display=1;

int  m_takeoff = 0;
boolean  thermalling = false;
boolean  getAltitude2 = false;

void play_welcome_beep()
{
    if(beepOn)
    {
        for (int aa=100;aa<=800;aa=aa+100)
        {
            tone(tone_out1,aa,400);
            tone(tone_out2,aa+3,400);
            delay(40);
        }
    }
}

const uint8_t rook_bitmap[] PROGMEM = {
                                        0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X38,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X07,0XF0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X7F,0XE0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X00,0X00,0X00,0X07,0XFF,0XE0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X00,0X00,0X00,0X3F,0XFF,0XC0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X00,0X00,0X01,0XFF,0XFF,0X80,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X00,0X00,0X0F,0XFF,0XFF,0X80,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X00,0X00,0X7F,0XFF,0XFF,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X00,0X03,0XFF,0XFF,0XFE,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X00,0X0F,0XFF,0XFF,0XFE,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X00,0X3F,0XFF,0XFF,0XFC,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X00,0X00,0X3F,0XFF,0XF8,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X00,0X0F,0XFF,0XFF,0XFF,0XFE,0X7F,0XFF,0XFE,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X00,0XEF,0XFF,0XFF,0XFF,0XFE,0X7F,0XFF,0XFF,0XE0,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X00,0XAF,0XFF,0XFF,0XFF,0XFE,0X7F,0XFF,0XFF,0XF8,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X00,0XAF,0XFF,0XFF,0XFF,0XFE,0X7F,0XFF,0XFF,0XFE,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X7F,0XAF,0XFF,0XFF,0XFF,0XFE,0X7F,0XFF,0XFF,0XFF,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X40,0X2F,0XFF,0XFF,0XFF,0XFE,0X7F,0XFF,0XFF,0XFF,0X80,0X00,0X00,
                                        0X00,0X00,0X00,0X40,0X3F,0XFF,0XFF,0XFF,0XFE,0X7F,0XFF,0XFF,0XFF,0X80,0X00,0X00,
                                        0X00,0X00,0X00,0X7F,0XFF,0XFF,0XE7,0X9F,0XFE,0X7F,0XFF,0XFF,0XFF,0XC0,0X00,0X00,
                                        0X00,0X00,0X00,0X7F,0XFF,0XFF,0X87,0XBF,0XFE,0X7F,0XFF,0XFF,0XFF,0XC0,0X00,0X00,
                                        0X00,0X00,0X00,0X49,0XBF,0XFF,0X87,0XBF,0XFE,0X7F,0XF8,0X0F,0XFF,0XC0,0X00,0X00,
                                        0X00,0X00,0X00,0X49,0XBF,0XFF,0X87,0X3F,0XFE,0X7F,0XF8,0X07,0XFF,0XC0,0X00,0X00,
                                        0X00,0X00,0X00,0X49,0XBF,0XFF,0X87,0X3F,0XFE,0X7F,0XF8,0X03,0XFF,0XC0,0X00,0X00,
                                        0X00,0X00,0X00,0X40,0X3F,0XFF,0X86,0X3F,0XFE,0X7F,0XF8,0X03,0XFF,0XC0,0X00,0X00,
                                        0X00,0X00,0X00,0X40,0X3F,0XFF,0X86,0X3F,0XFE,0X7F,0XF8,0X03,0XFF,0XC0,0X00,0X00,
                                        0X00,0X00,0X00,0X7F,0XFF,0XFF,0X80,0X3F,0XFE,0X7F,0XF8,0X03,0XFF,0XC0,0X00,0X00,
                                        0X00,0X00,0X00,0X7F,0XFF,0XFF,0X80,0X3F,0XFE,0X7F,0XF8,0X07,0XFF,0XC0,0X00,0X00,
                                        0X00,0X00,0X00,0X43,0XFF,0XFF,0X80,0X3F,0XFE,0X7F,0XF8,0X1F,0XFF,0X80,0X00,0X00,
                                        0X00,0X00,0X00,0X60,0X3F,0XFF,0X80,0X3F,0XFE,0X7F,0XFF,0XFF,0XFF,0X80,0X00,0X00,
                                        0X00,0X00,0X00,0X3E,0X3F,0XFF,0X80,0X3F,0XFE,0X7F,0XFF,0XFF,0XFF,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X3C,0X3F,0XFF,0X80,0X3F,0XFE,0X7F,0XFF,0XFF,0XFF,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X40,0X6F,0XFF,0X80,0X3F,0XFE,0X7F,0XFF,0XFF,0XFE,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X47,0XCF,0XFF,0X80,0X3F,0XFE,0X7F,0XFF,0XFF,0XFC,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0XFF,0XEF,0XFF,0X80,0X3F,0XFE,0X7F,0XFF,0XFF,0XF8,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0XEF,0XAF,0XFF,0XFE,0X3F,0XFE,0X7F,0XFF,0XFF,0XE0,0X00,0X00,0X00,
                                        0X00,0X00,0X01,0XC9,0XAF,0XFF,0XFF,0X3F,0XFE,0X7F,0XFF,0XFF,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X01,0XC9,0XA7,0XFF,0XDF,0XFF,0XFE,0X7F,0XFC,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X01,0XC9,0XA7,0XFF,0XFF,0XFF,0XFE,0X7F,0XFC,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X03,0XC0,0X27,0XFF,0XFF,0XFF,0XFC,0X7F,0XFC,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X03,0XC0,0X27,0XFF,0XFF,0XFF,0XFC,0X7F,0XFC,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X01,0XFF,0XE3,0XFF,0XFF,0XFF,0XF8,0X7F,0XFC,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X01,0XE0,0XA1,0XFF,0XFF,0XFF,0XF8,0X7F,0XFC,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0XF0,0XA1,0XFF,0XFF,0XFF,0XF0,0X7F,0XFC,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X7F,0XA0,0XFF,0XFF,0XFF,0XE0,0X7F,0XFC,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X7F,0X20,0X3F,0XFF,0XFF,0XC0,0X7F,0XFC,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X40,0X20,0X1F,0XFF,0XFF,0X00,0X7F,0XFC,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X40,0X20,0X07,0XFF,0XFC,0X00,0X7F,0XFC,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X3F,0XE0,0X0F,0XFF,0XC0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X00,0X00,0X1F,0XE0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X00,0X00,0X7F,0X80,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X00,0X00,0XFE,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X00,0X03,0XF8,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X00,0X0F,0XC0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X00,0X3F,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X01,0XF0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X0F,0X80,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X70,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                                        0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                                      };




void draw(void) 
{    
    u8g.setColorIndex(1); // Instructs the display to draw with a pixel on.
    u8g.setFont(u8g_font_8x13Br); //para o alfabeto completo com caracteres tirar o r do font_8x13Br
    if(screen == 0)
    {
        //u8g.drawBitmapP( 0, 0, 16, 64, rook_bitmap);
    }
    else if(screen == 1)
    {
        u8g.setColorIndex(0);
        u8g.setColorIndex(1);
        u8g.drawStr( 13, 30, "**  LET'S  **"); 
        u8g.drawStr( 38, 50, "  FLY  ");
    }
    
    if(screen == 2)
    {
        u8g.setColorIndex(0);
        u8g.setColorIndex(1);
        
        unsigned char posVar = 0;
        unsigned char posAlt1 = 0;
        unsigned char posAlt2 = 0;
        //temperatura
        
        u8g.setPrintPos(0, 16);
        u8g.print(temperature,1);
        u8g.drawStr( 35, 16, "C"); 
        
        //box and m / s vario
        
        u8g.drawFrame(46,3,82,35); // square vario
        u8g.drawStr(100, 33, "M\057s"); //m/s
        u8g.drawStr(120, 64, "M"); //M altimeter       
        
        //submenu and clock
        if(m_clock == 1)
        {          
            if(countPressVal == 2 || countPressVal == 3)
            {
                u8g.drawStr(0, 30, "v");
                u8g.setPrintPos(10, 30);
                u8g.print(muxTmp);
            }
            else if(countPressVal == 0)
            {
                u8g.drawStr(0, 30, "Alt1");
            }
            else if(countPressVal == 1)
            {
                u8g.drawStr(0, 30, "Alt2");
            }     
            else if(countPressVal == 4)
            {
                if(!beepOn)
                {                  
                    u8g.drawStr(0, 30, "S_ON");
                }
                else
                {
                    u8g.drawStr(0, 30, "S_OFF");
                }                
            }                        
        }
        else{
            String timex = "";
            if(m_hour < 10){
                timex += "0";
            }
            timex += m_hour;
            if (m_second%2==0)
            {
                timex += ":";
            }
            else{
                timex += " ";
            }
            if(m_minute < 10){
                timex += "0";
            }
            timex += m_minute;
            
            u8g.setPrintPos(0, 34);
            u8g.print(timex);
            
            u8g.setFont(u8g_font_5x8r); 
            u8g.setPrintPos(15, 62);   
            u8g.print(mux);
            
            u8g.setPrintPos(15, 50);
            
            if(beepOn)
            {              
                u8g.print("S_ON");
            }
            else
            {
                u8g.print("S_OFF");
            }
        }        
     
        u8g.setFont(u8g_font_fub20n); //if it's only numbers use n at the end u8g.setFont (u8g_font_fub20n);
        
        if(refAltitudeOn)
        {
            if(altitude2 >= 1000)
            {
                posAlt2 = 44;
            }
            else if(altitude2 >= 100 && altitude2 < 1000)
            {
                posAlt2 = 60;
            }
            else if(altitude2 >= 10 && altitude2 < 100)
            {
                posAlt2 = 76;
            }
            else if(altitude2 >= 0 && altitude2 < 10)
            {
                posAlt2 = 92;
            }

            u8g.setPrintPos(posAlt2, 64);       
            u8g.print(altitude2, 0);  
        } 
        else
        {
           if(altitude >= 1000)
            {
                posAlt1 = 44;
            }
            else if(altitude >= 100 && altitude < 1000)
            {
                posAlt1 = 60;
            }
            else if(altitude >= 10 && altitude < 100)
            {
                posAlt1 = 76;
            }
            else if(altitude >= 0 && altitude < 10)
            {
                posAlt1 = 92;
            }           

            u8g.setPrintPos(posAlt1, 64);       
            u8g.print(altitude, 0);  
        } 
        
        //vario
        if (vario < 0)
        {
            posVar = 48;
            if(vario < -0.1){
                u8g.drawTriangle(111,15, 120,7, 103,7);
            }
        }
        else
        {
            posVar = 57;
            if(vario > 0.1){
                u8g.drawTriangle(111,7, 120,15, 103,15);
            }
        }
        u8g.setPrintPos(posVar, 30);
        u8g.print(vario,1);        
        
        //read battery value                 
        int vcc = vRef.readVcc();
        
        u8g.drawFrame(4,42,4,2);
        u8g.drawFrame(2,44,8,18);
        if(vcc > 2600){
            u8g.drawStr( 1, 60, ".");
        }
        if(vcc > 2900){
            u8g.drawStr( 1, 55, ".");
        }
        if(vcc > 3000){
            u8g.drawStr( 1, 50, ".");
        }    
    }
    
    //-------------------menu-------------
    
    if(screen == 3)
    {  
        u8g.setColorIndex(0);
        u8g.setColorIndex(1);        
        
        //retrieves data from eeprom
        
        if(Rend_memo_display == 1){ //indicates which memory the display is requesting
            Rend_memo=2;
        }
        if(Rend_memo_display == 2){
            Rend_memo= 22;
        }
        if(Rend_memo_display > 2){
            Rend_memo = (Rend_memo_display*20) + 2;
        }
        
        if(memory == 0)
        {
            EEPROM.get(Rend_memo, Rm_hour);
            delay(10);
            Rend_memo = Rend_memo +1;
            EEPROM.get(Rend_memo, Rm_minute);
            delay(10);
            Rend_memo = Rend_memo + 2;
            EEPROM.get(Rend_memo, Rvarioup);
            delay(10);
            Rend_memo = Rend_memo + 4;
            EEPROM.get(Rend_memo, Rvariodown);
            delay(10);
            Rend_memo = Rend_memo + 4;
            EEPROM.get(Rend_memo, RaltitudeMax);
            delay(10);
            Rend_memo = Rend_memo + 4;
            EEPROM.get(Rend_memo, Rgain);
            delay(10);
            
            memory = 1;
        }
        
        u8g.drawRFrame(0, 0, 128, 16, 4);
        u8g.setPrintPos(4, 13);
        u8g.print(Nmeno_display);// flight number
        u8g.drawStr(20, 13, "\tMEMORY");        
        //Shows flights        
        
        String Rtimex = ""; //arranges flight hr
        if(Rm_hour < 10){
            Rtimex += "0";
        }
        Rtimex += Rm_hour;
        Rtimex += ":";
        if(Rm_minute < 10){
            Rtimex += "0";
        }
        Rtimex += Rm_minute;
        
        u8g.drawStr(45, 13, ".");
        u8g.setPrintPos(80, 13);
        u8g.print(Rtimex); //eeprom memory here
        //u8g.drawStr(22, 31, ":");
        u8g.drawTriangle(10,22, 20,31, 0,31);
        u8g.setPrintPos(26, 31);
        u8g.print(Rvarioup,1); //eeprom memory here
        //u8g.drawStr(82, 31, ":");
        u8g.drawTriangle(70,30, 79,22, 62,22);
        u8g.setPrintPos(82, 31);
        u8g.print(Rvariodown,1); //eeprom memory here
        u8g.drawStr(0, 47, "Alt.Max:");
        u8g.setPrintPos(68, 47);
        u8g.print(RaltitudeMax,1); //eeprom memory here
        u8g.drawStr(0, 64, "Alt.Gain:"); //gain from takeoff
        u8g.setPrintPos(71, 64);
        u8g.print(Rgain,1); //eeprom memory here
    }
    
    if(screen==4){
        
        u8g.setColorIndex(0);
        u8g.setColorIndex(1);
        u8g.drawStr( 25, 30, "MEMORY");
        u8g.drawStr( 37, 50, "Cleared");
        
        delay(250);
        menu=5;
    }
    
    if(screen==5){
        
        u8g.setColorIndex(0);
        u8g.setColorIndex(1);
        u8g.drawStr( 25, 35, "OK :)");
        delay(250);
        menu=2;        
    }    
}

void setnmeaVarioLXWP0(float alt, float vario) { //in m/s
  // $LXWP0,logger_stored, airspeed, airaltitude,
  //   v1[0],v1[1],v1[2],v1[3],v1[4],v1[5], hdg, windspeed*CS<CR><LF>
  //
  // 0 loger_stored : [Y|N] (not used in LX1600)
  // 1 IAS [km/h] ----> Condor uses TAS!
  // 2 baroaltitude [m]
  // 3-8 vario values [m/s] (last 6 measurements in last second)
  // 9 heading of plane (not used in LX1600)
  // 10 windcourse [deg] (not used in LX1600)
  // 11 windspeed [km/h] (not used in LX1600)
  //
  // e.g.:
  // $LXWP0,Y,222.3,1665.5,1.71,,,,,,239,174,10.1

  char t_nmeaVarioLXWP0[60] = "$LXWP0,N,,";
  char t_vario[5];
  char t_alt[9];

  char t_tail[3] = ",,,,,,,";
  char t_comma[2] = ",";

  dtostrf(alt, 5, 2, t_alt);
  strcat(t_nmeaVarioLXWP0, t_alt);
  strcat(t_nmeaVarioLXWP0, t_comma);
  
  dtostrf(vario, 2, 2, t_vario);
  strcat(t_nmeaVarioLXWP0, t_vario);
  strcat(t_nmeaVarioLXWP0, t_comma);
  
  strcat(t_nmeaVarioLXWP0, t_tail);
  strcat(t_nmeaVarioLXWP0, "*");

  getCRC(t_nmeaVarioLXWP0);

  strcat(t_nmeaVarioLXWP0, t_check);

  if (Serial.available())      
  {
    Serial.print(t_nmeaVarioLXWP0); 
    Serial.print(newl);
  }
}

void getCRC(char *buff) {
  // NMEA CRC: XOR each byte with previous for all chars between '$' and '*'
  char c;
  byte i;
  byte start_with = 0;
  byte end_with = 0;
  char CRC = 0;

  for (i = 0; i < 128; i++) {
    c = buff[i];
    if (c == 0x24) {
      start_with = i;
    }
    if (c == 0x2a) {
      end_with = i;
      break;
    }
  }
  if (end_with > start_with) {
    for (i = start_with + 1; i < end_with; i++) { // XOR every character between '$' and '*'
      CRC = CRC ^ buff[i] ;  // compute CRC
    }
  }

  //Single threaded, so this is allowed
  t_check[0] = hex[(CRC >> 4) & 0xF];
  t_check[1] = hex[(CRC) & 0xF];
  t_check[2] = '\0';
}

void setup()
{
    Serial.begin(115200);  
    
    Wire.begin();                //  Initialize i2c
    pinMode(4, INPUT);
    digitalWrite(4, HIGH);
    pinMode(tone_out1, OUTPUT);  // Dynamic pin8 output -
    pinMode(tone_out2, OUTPUT);  // Speaker pin9 output  +
    sensor_bmp.begin(); // Sensitivity of pressure sensor
    vRef.begin(); //initiates reading of battery voltage
    
    //read memory to record
    num_memo = EEPROM.read(1); //last recording before being turned off   
    delay(10);    
    
    num_memo ++;
    if(num_memo > 20){num_memo = 1;}
    if(num_memo < 1){num_memo = 1;}
    
    Rend_memo_display = EEPROM.read(1);
    
    play_welcome_beep();
}

void loop(void)
{   
    //oled display
    time = millis() / 300;
    u8g.firstPage();
    
    if (time <= 5)//start screen
    {
        screen = 0;       
    }
    else if(time >5 && time <= 25)//Lets fly screen
    {
        screen = 1;
    }
    else
    {       
        screen = menu;
        
        //--------------save data-----------
        
        if(vario > Mvarioup){Mvarioup=vario;} //writes the largest integer variable
        if(vario < Mvariodown){Mvariodown=vario;} //writes the largest descendant variable
        
        if(altitude > MaltitudeMax){
            MaltitudeMax = altitude;
        }
        
        MaltitudeDec = altitude;
        
        if(millis() - temp_alt < 0)
        {
            temp_alt = millis();
        }
        else
        {
            temp_verif_alt = int((millis() - temp_alt)/1000); 
        }
        if(temp_verif_alt > 20 && m_takeoff == 0)
        {
            temp_verif_alt = 0;
            MaltitudeDec = altitude;
            
            if((altitude - ult_altitude < -5.0 || altitude - ult_altitude > 5.0) && menu == 2) 
            {
                m_takeoff = 1;
            }
        }
        ult_altitude = MaltitudeDec;
        m_gain = MaltitudeMax - MaltitudeDec;
        
        //Flight ended.   
        if((m_landing - altitude < 2 || m_landing - altitude > -2) && menu == 2 && endfly == 0 && m_takeoff == 1)
        {
            m_savetime = millis();
            endfly = 1;
        }
        
        m_landing = altitude;
        if(m_landing - altitude < 2 && m_landing - altitude > -2 && menu == 2 && endfly == 1)
        {
            if (millis() - m_savetime > 5000) 
            { 
                //save the whole on eeprom!!!  
                
                if(num_memo == EEPROM.length())  //check if address counter has reached the end of EEPROM
                {
                    num_memo = 1;              //if yes: reset address counter
                }                  
                
                EEPROM.write(1, num_memo); //writing the last record memory adress.
                delay(10);
                
                if(num_memo == 1){ 
                    end_memo=2;
                }
                if(num_memo == 2){
                    end_memo= 22;
                    
                }
                if(num_memo > 2){
                    end_memo = (num_memo*20) + 2;
                }
                
                EEPROM.put(end_memo, m_hour);
                delay(10);
                end_memo = end_memo + 1;
                EEPROM.put(end_memo, m_minute);
                delay(10);
                end_memo = end_memo + 2;
                
                EEPROM.put(end_memo, Mvarioup);
                delay(10);
                end_memo = end_memo + 4;
                EEPROM.put(end_memo, Mvariodown);
                delay(10);
                end_memo = end_memo + 4;
                EEPROM.put(end_memo, MaltitudeMax);
                delay(10);
                end_memo = end_memo + 4;
                EEPROM.put(end_memo, m_gain);
                delay(10);               
                
                endfly = 0;                
            }
        }      
    }   
    
    int currentState = (digitalRead (4));

    if (currentState != buttonState)
    {
        duration = millis();
        resettime = millis();
    }
    
    if(currentState == LOW)
    {   
        if ((millis() - duration > 5) && (millis() - duration < 2000) && menu == 2 && buttonStatus == 0)
        {
            countPressVal++;
            if(beepOn)
                tone(tone_out1,1800,40);           
            
            delay(10);
            
            if(countPressVal > 4){countPressVal = 0;}
            
            lastMillis = millis();
            m_clock = 1;
            buttonStatus = 1;            
        }       
        
        if (millis() - duration > 2000 && menu == 2 && buttonStatus == 1)
        {           
            memory =0;
            menu = 3;
            duration = millis();
        }
        
        if (millis() - duration > 2000 && menu == 3 && buttonStatus == 1) 
        {
            menu = 2;
            Nmeno_display = 1;
            Rend_memo_display = EEPROM.read(1);
            duration = millis();
        }
        
        if (millis() - resettime > 5000 && (menu == 2 || menu == 3) && buttonStatus == 1) 
        {            
            menu = 4;
            
            for (int i = 0; i < EEPROM.length(); i++)
            {
                if(EEPROM.read(i) != 0) //skip already "empty" addresses
                {
                    EEPROM.write(i, 0);  
                }
            } 
            
            if( countPressVal > 1)
                countPressVal = 0;
            
            num_memo = 1;
            
            EEPROM.write(1, num_memo); //writing the last record memory adress.
            delay(10);                
            
            Nmeno_display = 1;       
            
            Rend_memo_display = EEPROM.read(1);
            resettime = millis();           
        }
        
        if ((millis() - duration > 5) && (millis() - duration < 2000)&& menu == 3 && buttonStatus==0) 
        {
            memory = 0;
            Nmeno_display++; // Show only the flight number
            Rend_memo_display--; // pulls the memory of flight from the oldest to the oldest
            if(Rend_memo_display < 1){Rend_memo_display = 20;}
            if(Nmeno_display > 20){Nmeno_display = 1;}
            buttonStatus = 1;
            if(beepOn)
                tone(tone_out1,1800,40);    
            
            delay(10);
            
            if( countPressVal > 1)
                countPressVal = 0;
        }        
    }
    if(currentState ==HIGH)
    {
        buttonStatus = 0;       
    }
    
    buttonState = currentState;
    
    //SENSITIVITY 
    
    if((countPressVal) == 2) {muxTmp = 0.25;} // 3 sinal
    if((countPressVal) == 3) {muxTmp = 0.5;} // 4 sinal  
    
    //mostra m_clock e variaÃ§aÃµ
    if (lastMillis > 0 && (millis() - lastMillis > 2500))
    {
        m_clock = 0;
        lastMillis = 0;
        
        if(menu == 2)
        {
            if((countPressVal) == 0) {refAltitudeOn = false;} // 1 sinal
            if((countPressVal) == 1) {refAltitudeOn = true;} // 2 sinal        
            if((countPressVal) == 2) {mux = 0.25;} // 3 sinal
            if((countPressVal) == 3) {mux = 0.5;} // 4 sinal
            if((countPressVal) == 4) 
            {
                if(beepOn)
                    beepOn = false;
                else
                    beepOn = true;
            } //5 sinal   
            
            vario_up = mux;
        }  
        
        if(countPressVal > 1)
        {          
            if(!refAltitudeOn)
            {
                countPressVal = 0;
            }
            else
            {
                countPressVal = 1;
            }
        }     
    }
    
    tempo = millis();
    float N1 = 0;
    float N2 = 0;
    float N3 = 0;
    float D1 = 0;
    float D2 = 0;
    
    //every second get temperature and battery level 
    if (millis() >= (get_time_second + 1000))      
    {
        setnmeaVarioLXWP0(altitude, vario);       
        temperature = (sensor_bmp.readTemperature());
        get_time_second = millis();
    }
    
    auto tmpAlt  = (sensor_bmp.readAltitude(1013.25)); // calculate altitude
    
    for(int cc=1;cc<=maxsamples;cc++)
    {                                   
        alt[(cc-1)] = alt[cc];
        tim[(cc-1)] = tim[cc];
    };
    
    alt[maxsamples] = tmpAlt;
    tim[maxsamples] = tempo;
    float stime = tim[maxsamples-samples];
    for(int cc = (maxsamples-samples); cc<maxsamples; cc++)
    {
        N1 += (tim[cc]-stime)*alt[cc];
        N2 += (tim[cc]-stime);
        N3 += (alt[cc]);
        D1 += (tim[cc]-stime)*(tim[cc]-stime);
        D2 += (tim[cc]-stime);
    };    
    
    altitude = round(tmpAlt*100)/100.0;//altitudeKalmanFilter.updateEstimate(round(tmpAlt*100)/100.0);   //apply kalman filter    
    if(altitude < 0) altitude = 0.0;
    
    if(getAltitude2)
    {
         altitude2 = altitude - refAltitude2;
         if(altitude2 < 0) altitude2 = 0.0;
    }
    else
    {
        refAltitude2 = round(tmpAlt*100)/100.0;
        getAltitude2 = true;
    }  
   
    auto tmpVario  = 1000.0*(float)((samples*N1)-N2*N3)/(float)(samples*D1-D2*D2); // calculate vario
    
    vario = varioKalmanFilter.updateEstimate(round(tmpVario*100)/100.0);   //apply kalman filter 
    
    if(vario > 9.9)vario = 9.9;
       
    auto delayBeep = 300;
    
    if ((tempo - beep) > Beep_period && screen == 2)
    {
        beep = tempo;
        
        if (vario > vario_up && vario <= 9.9)
        {          
            Beep_period = delayBeep + 50 - (vario*10);
            
            if(beepOn)
            {
                tone(tone_out1, (1000 + (100*vario)), delayBeep - (vario*10)); 
                tone(tone_out2, (1003 + (100*vario)), delayBeep - (vario*10));
            }
            thermalling = true;
        }
        else if ((vario < 0 ) && (thermalling == true))
        {
            thermalling = false;
        }
        else if (vario < vario_down && vario > vario_emergence)
        {         
            Beep_period = 200;
            if(beepOn)
            {
                tone(tone_out1,(300 - (vario)), 340);
                tone(tone_out2,(303 - (vario)), 340);
            }
            thermalling = false;
        }
        else if (vario < vario_emergence)
        {         
            Beep_period=200;
            if(beepOn)
            {
                tone(tone_out1, 2000, 250);
                tone(tone_out2, 2003, 250);
                delay(200);
                tone(tone_out1, 1000, 250);
                tone(tone_out2, 1003, 250);
            }
            thermalling = false;
        }
    }
    
    //clock
    if(m_takeoff == 1){ //inicia o cronometro quando detectada a m_takeoff
        if(millis() - UtlTime<0)
        {
            UtlTime = millis();
        }
        else{
            m_second = int((millis()-UtlTime)/1000);
        }
        if(m_second > 59)
        {
            m_second = 0;
            m_minute++;
            UtlTime = millis();
            if(m_minute > 59)
            {
                m_hour++;
                m_minute=0;
            }
        }
    }
    else{
        m_second = 00;
        m_minute = 00;
        m_hour = 00;
    }
    
    //-----------mostrar screens-----------
    do 
    {
        draw();       
    } while( u8g.nextPage() );
    //end oled display
}
