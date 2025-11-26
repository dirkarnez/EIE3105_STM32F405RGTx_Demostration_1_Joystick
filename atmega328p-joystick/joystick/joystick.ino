/* Funduino Joystick Shield Example

https://protosupplies.com/product/funduino-joystick-shield-v1-a/

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
  Serial.begin(115200);
}
//===============================================================================
//  Main
//===============================================================================
void loop() {
  // Check each button input and print the status to the Serial Monitor Window
  Serial.print(" u="),Serial.print(digitalRead(UP_BTN) == 1 ? 0 : 1);
  Serial.print(" d="),Serial.print(digitalRead(DOWN_BTN) == 1 ? 0 : 1);
  Serial.print(" l="),Serial.print(digitalRead(LEFT_BTN) == 1 ? 0 : 1);
  Serial.print(" r="),Serial.print(digitalRead(RIGHT_BTN) == 1 ? 0 : 1);
  Serial.print(" e="),Serial.print(digitalRead(E_BTN) == 1 ? 0 : 1);
  Serial.print(" f="),Serial.print(digitalRead(F_BTN) == 1 ? 0 : 1);
  Serial.print(" j="),Serial.print(digitalRead(JOYSTICK_BTN) == 1 ? 0 : 1);
      
  // Print the full X/Y joystick values (0-1023)
  Serial.print(" x="),Serial.print(analogRead(JOYSTICK_AXIS_X));
  Serial.print(" y="),Serial.print(analogRead(JOYSTICK_AXIS_Y)); 
  Serial.print("\n"); 
  //delay(250);
 }
