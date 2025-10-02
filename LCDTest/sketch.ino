#include <LiquidCrystal.h>

//Setup for LCD
int rs = 3;
int en = 6;
int d4 = 10;
int d5 = 11;
int d6 = 12;
int d7 = 13;
LiquidCrystal lcd(rs,en,d4,d5,d6,d7);

int previousPress = 1; //Keeps track of the last press of the button
int WireframeNum = 1;
int sectionBreak = 0; //Display number for break;
int section = 0;
int secondsPassed = 0;


unsigned long tNow;
unsigned long tPast;

bool isDisplayed = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(8, INPUT_PULLUP);
  lcd.begin(16,2); //16 columns and 2 rows

  tNow = millis();
  tPast = tNow;
}

void loop() {
  // put your main code here, to run repeatedly:
  int sensorVal = digitalRead(8);
  // Serial.print("Current Val:");
  // Serial.println(sensorVal);

  // Serial.print("Previous Val:");
  // Serial.println(previousPress);

  // 1 is off, 0 is on - If the button is released, switch to next section
  if(previousPress == 0 and sensorVal == 1){
    Serial.print("Section: ");
    Serial.println(WireframeNum);
    if (WireframeNum == 1){
      WireframeNum = 2;
    }
    else{
      WireframeNum = 1;
    }
  }

  previousPress = sensorVal;

  tNow = millis();
  isDisplayed = false; //Ensures the screens don't stack up.

  //Check the amount of milliseconds that passes from the last time 
  // the arduino pings. If it's more than 1s, then update the display.
  //https://www.reddit.com/r/arduino/comments/11rhepb/how_do_i_start_and_stop_a_loop_with_the_press_of/
  //Code adapted from above reddit -> Used the millisecond strat but changed the output from a LED
  //light into a LCD display

  if (tNow - tPast >= 1000ul){
    tPast = tNow;
    Serial.println("A Second has passed.");
    Serial.println(section); //The current section being display in the 2nd wireframe
    Serial.println(WireframeNum); //Which set of display screens is being shown (1: breaktime, 2: Leaves in use)
    lcd.clear();
    secondsPassed++;

    if(WireframeNum == 1){
      //change the current section of the break screen.
      if(sectionBreak == 0 && isDisplayed == false && secondsPassed == 2){
        changeTopRowLCD("Place items in");
        changeBottomRowLCD("leaves to start!");
        sectionBreak = 1;
        isDisplayed = true;
        secondsPassed = 0;
      }
      if(sectionBreak == 1 && isDisplayed == false  && secondsPassed == 2){
        changeTopRowLCD("Current Study");
        changeBottomRowLCD("Timer: 0hr20min");
        sectionBreak = 0;
        isDisplayed = true;
        secondsPassed = 0;
      }
    }

    if(WireframeNum == 2){
      //change the current section of the Studying screen.
      if(section == 0 && isDisplayed == false){
        changeTopRowLCD("Studying");
        changeBottomRowLCD("1/4 Leaves used");
        section = 1;
        isDisplayed = true;
      }
      if(section == 1 && isDisplayed == false){
        changeTopRowLCD("Studying.");
        changeBottomRowLCD("1/4 Leaves used");
        section = 2;
        isDisplayed = true;
      }
      if(section == 2 && isDisplayed == false){
        changeTopRowLCD("Studying..");
        changeBottomRowLCD("1/4 Leaves used");
        section = 3;
        isDisplayed = true;
      }
      if(section == 3 && isDisplayed == false){
        changeTopRowLCD("Studying...");
        changeBottomRowLCD("1/4 Leaves used");
        section = 0;
        isDisplayed = true;
      }
      secondsPassed = 0;
    }

    
  }

}

void breakMode(){
  changeTopRowLCD("Place items in");
  changeBottomRowLCD("leaves to start!");

  delay(2000);
  lcd.clear();

  changeTopRowLCD("Current Study");
  changeBottomRowLCD("Timer: 0hr20min");

  delay(2000);
  lcd.clear();
}


void studyMode(){
  changeTopRowLCD("Studying");
  changeBottomRowLCD("1/4 Leaves used");
  delay(1000);

  changeTopRowLCD("Studying.");
  delay(1000);

  changeTopRowLCD("Studying..");
  delay(1000);
  
  changeTopRowLCD("Studying...");
  delay(1000);

  lcd.clear();
}


void changeTopRowLCD(String sentence){
  lcd.setCursor(0,0); //column, row
  lcd.print(sentence);
}

void changeBottomRowLCD(String sentence){
  lcd.setCursor(0,1); //column, row
  lcd.print(sentence);
}

