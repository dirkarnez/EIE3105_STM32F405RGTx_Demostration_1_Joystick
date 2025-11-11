/* Funduino Joystick Shield Example

This program simply captures all of the inputs of the Funduino Joystick Shield buttons and
joystick every second and displays them in the Serial Monitor Window.  
The Arduino pins below are defined by the shield and cannot be changed.
*/
int const UP_BTN = 2;
int const DOWN_BTN = 4;
int const LEFT_BTN = 5;
int const RIGHT_BTN = 3;
int const E_BTN = 6;
int const F_BTN = 7;
int const JOYSTICK_BTN = 8;
int const JOYSTICK_AXIS_X = A0;
int const JOYSTICK_AXIS_Y = A1;
int buttons[] = {UP_BTN, DOWN_BTN, LEFT_BTN, RIGHT_BTN, E_BTN, F_BTN, JOYSTICK_BTN};
//===============================================================================
//  Initialization
//===============================================================================
void setup() {
  //Set all button pins as inputs with internal pullup resistors enabled.
  for (int i; i < 7; i++)  pinMode(buttons[i], INPUT_PULLUP);
  Serial.begin(9600);
}
//===============================================================================
//  Main
//===============================================================================
void loop() {
  // Check each button input and print the status to the Serial Monitor Window
  Serial.print("UP="),Serial.print(digitalRead(UP_BTN));
  Serial.print("\tDOWN="),Serial.print(digitalRead(DOWN_BTN));
  Serial.print("\tLEFT="),Serial.print(digitalRead(LEFT_BTN));
  Serial.print("\tRIGHT="),Serial.print(digitalRead(RIGHT_BTN));
  Serial.print("\tE="),Serial.print(digitalRead(E_BTN));
  Serial.print("\tF="),Serial.print(digitalRead(F_BTN));
  Serial.print("\tJOYSTICK BTN="),Serial.print(digitalRead(JOYSTICK_BTN));
      
  // Print the full X/Y joystick values (0-1023)
  Serial.print("\tX="),Serial.print(analogRead(JOYSTICK_AXIS_X));
  Serial.print("\tY="),Serial.println(analogRead(JOYSTICK_AXIS_Y)); 
  //delay(250);
 }
