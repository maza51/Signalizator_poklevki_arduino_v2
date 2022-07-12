#include <Wire.h>
#include <RCSwitch.h>
#include <SoftwareSerial.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#define PIN_RX 3
#define PIN_TX 4
#define PIN_RC433 2
#define PIN_ADXL_VCC 1
#define PIN_433_VCC 3

byte buff[6];
int old_x = 0;
int old_y = 0;
int old_z = 0;
int diff_max = 5;
int diff_max_podsechka = 70;
unsigned long int time_tap = 0;

RCSwitch mySwitch = RCSwitch();
SoftwareSerial mySerial(PIN_RX, PIN_TX);

void setup()
{
    //pinMode(PIN_ADXL_VCC, OUTPUT);
    //pinMode(PIN_433_VCC, OUTPUT);
    //digitalWrite(PIN_433_VCC, LOW);
    
    Wire.begin();
    mySerial.begin(9600);
    mySwitch.enableTransmit(PIN_RC433);

    adxl345_init();

    ADCSRA &= ~_BV(ADEN);
}

void adxl345_init() {
    digitalWrite(PIN_ADXL_VCC, HIGH);
    
    writeTo(0x53, 0x2D, 0);
    writeTo(0x53, 0x2D, 16);
    writeTo(0x53, 0x2D, 8);

    writeTo(0x53, 0x2C, 0b1010); // Hz
    writeTo(0x53, 0x31, 0b00); //   g

    delay(500);
}

void loop()
{
    readFrom(0x53, 0x32, 6, buff);

    int x = (((int)buff[1]) << 8) | buff[0];
    int y = (((int)buff[3]) << 8) | buff[2];
    int z = (((int)buff[5]) << 8) | buff[4];

    int diff_x = abs(old_x - x);
    int diff_y = abs(old_y - y);
    int diff_z = abs(old_z - z);

    old_x = x;
    old_y = y;
    old_z = z;

    if (diff_x > diff_max_podsechka || diff_y > diff_max_podsechka || diff_z > diff_max_podsechka) {
        mySerial.println("Podsechka");
        system_sleep(2);
        return;
    }

    if (z < -200) {
        calibration();
        return;
    }

    if (diff_x > diff_max || diff_y > diff_max || diff_z > diff_max) {
        if (millis() - time_tap > 10) {
            mySerial.println("Poklevka");
            send_for_resiver(B0001, 4);
            time_tap = millis();
        }
    }

    //delay(50);
    delay_wdt(WDTO_60MS);
}

void delay_wdt(int wdto_ms) {
    wdt_reset();
    wdt_enable(wdto_ms);
    WDTCR |= _BV(WDIE);
    
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_mode();
    wdt_disable();
}

void system_sleep(int t) {
    //digitalWrite(PIN_ADXL_VCC, LOW);

    while(t > 0) {
        delay_wdt(WDTO_8S);
        t--;
    }

    //adxl345_init();
}

void calibration() {
    delay(500);
    diff_max += diff_max < 10 ? 1 : -5;
    send_for_resiver(diff_max, 4);
    mySerial.print("diff_max change to ");
    mySerial.println(diff_max);
    delay(5000);
}

void send_for_resiver(int data, int b) {
    //digitalWrite(PIN_433_VCC, HIGH);
    mySwitch.send(data, b);
    //digitalWrite(PIN_433_VCC, LOW);
}

ISR (WDT_vect) {
    
}

void writeTo(int device, byte address, byte val) {
   Wire.beginTransmission(device);
   Wire.write(address);
   Wire.write(val);
   Wire.endTransmission();
}

void readFrom(int device, byte address, int num, byte buff[]) {
    Wire.beginTransmission(device);
    Wire.write(address);
    Wire.endTransmission();

    Wire.beginTransmission(device);
    Wire.requestFrom(device, num);

    int i = 0;
    while(Wire.available())
    {
        buff[i] = Wire.read();
        i++;
    }
    Wire.endTransmission();
}

long readVcc() {
    ADCSRA |= _BV(ADEN);
  
    ADMUX = _BV(MUX3) | _BV(MUX2);
  
    delay(75);
    ADCSRA |= _BV(ADSC);
    while (bit_is_set(ADCSRA,ADSC));

    uint8_t low = ADCL;
    uint8_t high = ADCH;

    long result = (high<<8) | low;

    result = 1125300L / result;

    ADCSRA &= ~_BV(ADEN);
  
    return result; // Vcc в милливольтах
}
