#include "mbed.h"
#include "Callback.h"
#include "Mutex.h"
#include "ThisThread.h"
#include "Thread.h"
#include <HTS221Sensor.h>
#include "DFRobot_RGBLCD.h"
#include <stm32l4xx_hal_iwdg.h>


#define I2C2_SCL    PB10  //clock
#define I2C2_SDA    PB11  //data

DevI2C i2c(PB_11, PB_10);
HTS221Sensor sensor(&i2c);

typedef struct{
    uint16_t seconds; //unsigned 16-bit integer
    uint16_t minutes;

    Mutex mutex;
}minsec_t;

typedef struct{
   float temp;
   float hum;
}temphum_t;

typedef struct{
    uint16_t seconds;
    uint16_t minutes;

}alarm_s;
alarm_s alarm_start;
DFRobot_RGBLCD lcd(16,2,D14,D15);
const int colorR = 255;
const int colorG = 0;
const int colorB = 255;

DigitalIn enable_alarm(BUTTON1); //enable alarm
DigitalIn start_off(A0); //start/stop alarm permanently
DigitalIn set_alarm_sec(A1); //set seconds 
DigitalIn set_alarm(A2);  //set minutes
DigitalIn humtemp_snooze(A3);  //used as both humtemp and snooze in separate thread
DigitalIn disable_alarm(A4);  //disable alarm
PwmOut alarm(D6);


void min_thread(minsec_t *data){
    while(true){
    ThisThread::sleep_for(1s);    
    data->mutex.lock();
    if(data->seconds==59)
    data->minutes++;
    else if(data->minutes>59)
    data->minutes=0;  
    data->mutex.unlock();
    }
}

void sec_thread(minsec_t *data){
    bool state = true;
    bool alarmstate = true;
    humtemp_snooze.mode(PullUp);
    start_off.mode(PullUp);
    disable_alarm.mode(PullUp);
    enable_alarm.mode(PullUp);

    while(state==true){
    ThisThread::sleep_for(1s);
    data->mutex.lock();
    data->seconds++;

    if(data->seconds>59)
    data->seconds=0;
    lcd.setCursor(1,0);
    lcd.clear();
    lcd.printf("%i:%i", data->minutes, data->seconds);
    data->mutex.unlock();

    if(start_off.read()==0){
    thread_sleep_for(100);
    alarm.write(0.0f); 
    state=false;
    lcd.clear();
    lcd.setCursor(1,0);
    lcd.printf(" Alarm turned");
    lcd.setCursor(0,1);
    lcd.printf("       off");

    }
    if (disable_alarm.read()==0){
    alarmstate = false;
    lcd.clear();
    lcd.setCursor(1,0);
    lcd.printf(" Alarm disabled.");
    }
    if (enable_alarm.read()==0){
    alarmstate = true;
    lcd.clear();
    lcd.setCursor(1,0);
    lcd.printf(" Alarm enabled.");

    }
    if (alarm_start.minutes==data->minutes && alarm_start.seconds==data->seconds &&
    alarmstate==true){
    alarm.write(0.5f);}
    else if (humtemp_snooze.read()==0){
    lcd.clear();
    lcd.setCursor(1,0);
    lcd.printf("5 min snooze");
    alarm.write(0.0f); 
    thread_sleep_for(100);
    data->mutex.lock();
    data->minutes = 0;
    data->seconds = 0;
    data->mutex.unlock();
    alarm_start.minutes = 5; 
    alarm_start.seconds = 0;
        }
    }
    
    }

temphum_t temphum;

int main(){
minsec_t data;
alarm.pulsewidth_ms(1); //1ms
alarm.write(0);

start_off.mode(PullUp);
set_alarm_sec.mode(PullUp);
set_alarm.mode(PullUp);
humtemp_snooze.mode(PullUp);

while(true){

lcd.init();
lcd.setRGB(colorR, colorG, colorB);
lcd.setCursor(1,0);
lcd.printf("%i:%i", alarm_start.minutes, alarm_start.seconds);

if (humtemp_snooze.read()==0){
sensor.init(nullptr);
sensor.enable();   
sensor.get_temperature(&temphum.temp);
sensor.get_humidity(&temphum.hum);
lcd.setCursor(1,0);
lcd.printf("Temp:     %f", temphum.temp);
lcd.setCursor(0,1);
lcd.printf(" Humidity: %f", temphum.hum);
ThisThread::sleep_for(3s);
}
else if (set_alarm.read()==0){  
alarm_start.minutes = alarm_start.minutes +1;
thread_sleep_for(100);
}
else if (set_alarm_sec.read()==0){ 
alarm_start.seconds = alarm_start.seconds +5;
thread_sleep_for(100);
}
else if (start_off.read()==0){
Thread thread_min;
thread_min.start(callback(min_thread, &data));
Thread thread_sec;
thread_sec.start(callback(sec_thread, &data));
thread_min.join();
thread_sec.join(); 
}
}


}

