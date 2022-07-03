// Import the Liquid Crystal library
#include <LiquidCrystal.h>
//Initialise the LCD with the arduino. LiquidCrystal(rs, enable, d4, d5, d6, d7)
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

char inputByte;
void setup() {
  // Switch on the LCD screen
  lcd.begin(16, 2); // 2x16 LCD Ekran başlatıldı
  Serial.begin(9600); // 9600 Baud Rate hızında seri haberleşme başlatıldı
}

void loop() {
  // put your main code here, to run repeatedly:
  while(Serial.available()>0){        // Haberleşmenin olup olmadığı kontrolü 
        inputByte = Serial.read();  // Seri haberleşme ile HC-06’dan gelen değerlerin okunması
        if(inputByte == 'y'){  // X koordinatının tamamlandığı
           
           lcd.print(" ");
        }
          if(inputByte == 'x'){ // Y koordinatının tamamlandığı
           delay(300); // 300ms delay eklendi
          lcd.clear(); 
          
            }
          lcd.print(inputByte); // Koordinat verisinin ekrana bastırılması
          } 
          
        }
