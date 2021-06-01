/*
 This is a version of the code made for use with a 16x2 LCD screen with an I2C adapter (easier wiring)
 This code controls a remote controlled AC powered motorized sliding gate.
 This is a retrofit for a formerly magnet limited sliding gate motor. 
 The opening time and autoclose wait time are set by the user. Settings are stored in EEPROM for restart after power outages.
 There is amperage overage monitoring to detect if an object blocks the gate.
 The gate can be partially opened or closed.
 Hardware includes an arduino nano, ACS712 current detector, a 2 AC controlled switch, remote with 2 buttons, LCD display, and a DIP switch. 
 
 Initialization requires use of the remote.
*/
 
//Defined variables that may need changing
float current_limit = 4.0;  //Amperage limit as read by the ACS712 before cutoff is engaged
float timing = 9000;  //default gate moving time in ms. Currently 9sec. It will be changed and stored in EEPROM during setup
float timeout_Until_Gate_Closes = 10000; //time until gate begins autoclosing.  Currently 10sec. It will be changed and stored in EEPROM during setup
float reading_Amp_Lockout_Period = 1000; //time delay before a high amp reading will trigger stop gate. Currently 1 second
unsigned long dip_Setup_Delay = 6000; // used to count down 6 seconds after turning on the dip to see if the user wants to reset

//ACS712 related variables
float current;  //raw data values from the 712
float test_Frequency = 50;   //test signal frequency (Hz) for the 712 (unit specific)
float windowLength = 40.0/test_Frequency; //how long to average the signal from the 712
float intercept = 0; //calibrates at 0 for this particular 712
float slope = 0.0551; //adjusted for our acs712 to match multimeter reading
float amps; //estimated actual current in amps from the 712
unsigned long reading712Period = 1000; //tracks time in milliseconds since last reading on the 712
unsigned long previous712Millis = 0; // for the 712
unsigned long previous_High_Amp_Millis = 0;

//gate movement related variables
int state_gate_flag = 13;  //0 is gate is closed not moving, 1 is opening, 2 is open not moving,3 is closing,
                           //4 is stopped while opening, 5 is stopped while closing, 6 is awaiting autoclose, 7 is initialize, 
                           //8 is gate is opening, 9 is waiting for autoclose wait period to setup, 10 is setup gate closing, 
                           //11 is stopping the setup autoclose, 12 is confirming setup, 13 is setup not started
const char *gateStatus[] = {"Closed    ", "Opening   ", "Open      ", "Closing    ", "Open Stop  ", "Close Stop ", 
                            "AutoClose  ","7","8","9","10","AutoClose  ","12","13"};//the states with numbers don't display on the LCD
int remote_B_action_flag = 0; //0 is the remote B button is inactive, 1 is the B button is active.
int gate_was_user_stopped_flag = 0; // becomes 1 if the user stopped the gate while closing.  It will prevent the autoclose from starting
int setup_flag = 1; //becomes 2 if setup is in progress , 3 if setup is complete. It is stored in EEPROM
int dip_flag = 0; //becomes 1 if the dip reset switch is on
float time_Left_To_Move = 00.00; //the main timekeeping variable.  Will count down.
float initial_Time_Left_To_Move = 00.00; //used to store the total time the gate will move
float time_Autoclose_Initiates = 0; //stores the time autoclose timer starts
unsigned long time_A_Pressed_Millis = 0; //will store the absolute time when remote button A was pushed
unsigned long time_Initiated_Millis = 0; // for setting timers

//arduino pin assignments
const int relay_open_pin_off = 5; //Controls the relay to open the gate (NO1 on the relay board) ***HIGH is off, otherwise both the open and close motors will run when power is restored
const int relay_close_pin_off = 6; //Controls the relay to close the gate (NO2 on the relay board)***HIGH is off, otherwise both the open and close motors will run when power is restored
const float amperage_pin = A0; //Receives raw level corresponding to amps used by the system from the 712 chip. Analog
const int buttonA_pin = 4; //Receives signal from remote button A
const int buttonB_pin = 3; //Receives signal from remote button B
const int dip_pin = 10; //Receives signal from DIP pin

//button or dip change monitor variables
int previous_State_A = 0; //0 not pressed, 1 is pressed
int previous_State_B = 0;
int previous_State_Dip = 0; //0 is off, 1 is on
long when_Button_Changed = 0; //will be used to wait 100ms to make sure a button press is real
int button_Status = 0;  //0 no change, 1 or 3 is A or B has been released, 2 or 4 is A or B is pressed, 5 is dip turned to off, 6 is dip turned to on
unsigned long dip_Setup_Millis = 0; // keeps track of timer if asking about resetting the system

//libraries
#include <Filters.h> //this corrects AC readings from the AC712 current reader
RunningStatistics inputStats; //create statistics to look at the raw test signal from the AC712

#include <EEPROM.h> //will save setup_flag, timing, and timeout_Until_Gate_Closes in case of power outage

#include <LiquidCrystal_PCF8574.h> //library which works well with JANSANE 16x2 1602 LCD I2C module interface adapter available on Amazon
#include <Wire.h> //necessary for the LCD library to work
LiquidCrystal_PCF8574 lcd(0x27); // set the LCD address to 0x27 for a 16 chars and 2 line display

void setup()
{
  Serial.begin(9600);
  
  inputStats.setWindowSecs(windowLength);  //set the window length for sampling 712
  
  pinMode(relay_open_pin_off, OUTPUT);
  pinMode(relay_close_pin_off, OUTPUT);
  pinMode(amperage_pin, INPUT);  
  pinMode(buttonA_pin, INPUT);
  pinMode(buttonB_pin, INPUT);
  pinMode(dip_pin, INPUT);

  digitalWrite(relay_close_pin_off, HIGH);  //make sure the gates are set to not move
  digitalWrite(relay_open_pin_off, HIGH);


  //initialize LCD
  Wire.begin();
  Wire.beginTransmission(0x27);
  lcd.begin(16, 2); // initialize the lcd
  lcd.setBacklight(255);
  
  EEPROM.get(0, setup_flag);  //see if system has already been setup
}

void loop()
{
  powerup(); //handles first time nano setup and power out sequence
  remoteAmonitor(); //checks for remote A push, initiates movement changes, finishes setup
  remoteBmonitor(); //checks for remote B push and handles all movement based on B, when B is active
  gatestop(); //determines when to stop gate motion and initiates autoclose wait period
  autoclose();  //handles the autoclose pause time
  lcdupdate(); //handles the screen
  currentmonitoring(); //checks the amperage ACS712 and handles overages
  changechecker(); //monitors A, B and the DIP for changes 
}

void powerup()
{
  if (state_gate_flag == 13) //if the power has just been restored.  This variable has been set to 13 by the restart.
  {
    if (setup_flag == 3) // if setup has been completed (this is done first to limit the number of EEPROM puts and will run after any power outage)
    {
      EEPROM.get(10, timing);
      EEPROM.get(20, timeout_Until_Gate_Closes);
      setup_flag = 2;  //keeps the get from running again
    }
    if (setup_flag == 1) // if setup has not been completed.  This should only run once per arduino, even if there is a power outage
    {
      EEPROM.put(10, timing); //puts default settings into the system
      EEPROM.put(20, timeout_Until_Gate_Closes);
      setup_flag = 2; //keeps the put from running again
    }
    lcd.display(); //screen turns on
    lcd.setBacklight(255);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("B to acknowledge");
    lcd.setCursor(0,1);
    lcd.print("power was off   ");
    if (button_Status == 4) //will not go forward until B is pushed and released. This is determined by changechecker()
    { 
      button_Status = 0; //resets the variable
      state_gate_flag = 7; //continues the recent power up sequence
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("must close w/ B ");
      lcd.setCursor(0,1);
      lcd.print("A when closed   ");
      remote_B_action_flag = 1; //activate monitorBbutton()
    }
  }
}

void remoteAmonitor()
{
  if (button_Status == 1) //if button A is newly pressed and released
    {
      button_Status = 0;
      time_A_Pressed_Millis = millis(); //marks the absolute time the button was released

      switch (state_gate_flag) //this is the main logic tree that determines what to do with a button A press and release based on prior gate status
        {
          case 2: //fallthrough if the gate is open and not moving (will do the same as case 6)
          
          case 6: //if the gate is open and waiting for autoclose timeout
            time_Left_To_Move = timing;
            initial_Time_Left_To_Move = time_Left_To_Move;
            time_Initiated_Millis = millis();
            digitalWrite(relay_open_pin_off, HIGH);
            digitalWrite(relay_close_pin_off, LOW); // sends a signal to the close pin to start closing 
            state_gate_flag = 3; //the gate is closing
            break;

          case 0: //if the gate is closed and not moving and the button is pushed
            time_Left_To_Move = timing;
            initial_Time_Left_To_Move = time_Left_To_Move;
            time_Initiated_Millis = millis();
            digitalWrite(relay_close_pin_off, HIGH);
            digitalWrite(relay_open_pin_off, LOW); //sends a brief signal to the open timer to open and start its timer
            state_gate_flag = 1; //the gate is opening
            break;

          case 4: //if the gate was stopped while opening and the button is pushed
            digitalWrite(relay_open_pin_off, HIGH);
            digitalWrite(relay_close_pin_off, LOW);  //the gate will start closing. It already knows for how long
            state_gate_flag = 3; //the gate is closing
            break;
            
          case 5: //if the gate was stopped while closing and the button is pushed
            digitalWrite(relay_close_pin_off, HIGH);
            digitalWrite(relay_open_pin_off, LOW);  //the gate will start opening. It already knows for how long
            state_gate_flag = 1; //the gate is opening
            gate_was_user_stopped_flag = 1; //sets a flag so the gate wont go into autoclose mode
            break;

          case 1:  //if the gate is opening and someone pushes the button
            time_Left_To_Move = millis() - time_Initiated_Millis; //store the time the gate has to close once the button is pushed again
            initial_Time_Left_To_Move = time_Left_To_Move;
            time_A_Pressed_Millis = 0;  //resets time_A_Pressed
            digitalWrite(relay_open_pin_off, HIGH);  //the gate will stop opening
            digitalWrite(relay_close_pin_off, HIGH);
            state_gate_flag = 4; 
            break;
        
          case 3: //if the gate is closing and someone pushes the button
            time_Left_To_Move = timing - time_Left_To_Move;  //the gate will reverse and start opening when the button is pushed
            initial_Time_Left_To_Move = time_Left_To_Move;
            time_A_Pressed_Millis = 0;  //resets time_A_Pressed
            digitalWrite(relay_close_pin_off, HIGH);  //the gate will stop closing
            digitalWrite(relay_open_pin_off, HIGH);
            state_gate_flag = 5;
            break;
       
          //the rest of these cases only happen during setup
  
          case 7: //user has just acknowledged that the gate is all the way closed by pushing A
            remote_B_action_flag = 0; //turn off the B button
            digitalWrite(relay_close_pin_off, HIGH); //gate stops closing (should do this anyway when B is released)
            digitalWrite(relay_open_pin_off, HIGH);
            dip_flag = digitalRead(dip_pin);
            if (dip_flag == LOW) //if the setup dip is off
              {
                lcd.noDisplay();
                lcd.setBacklight(0);
                //digitalWrite(LCD_bright_pin, LOW);
                state_gate_flag = 0; //go to normal function
              }
            if (dip_flag == HIGH)
              {
                lcd.clear();
                lcd.setBacklight(255);
                lcd.setCursor(0,0);
                lcd.print("Press A To      ");
                lcd.setCursor(0,1);
                lcd.print("Start Gate      ");
                state_gate_flag = 8;
              }
            break;
   
          case 8: //if the gate is being setup and the A button is pushed
            time_Initiated_Millis = millis();  
            digitalWrite(relay_close_pin_off, HIGH); //gate begins to open
            digitalWrite(relay_open_pin_off, LOW);
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("Press A To      ");
            lcd.setCursor(0,1);
            lcd.print("Stop Gate       ");
            state_gate_flag = 9;
            break;

          case 9: //A has been pressed to stop the opening gate
            digitalWrite(relay_close_pin_off, HIGH);
            digitalWrite(relay_open_pin_off, HIGH); //gate stops opening
            timing = millis() - time_Initiated_Millis; // set how long the gate opens and closes
            time_Initiated_Millis = millis(); //now set this variable to the start of the autoclose period setup
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("Press A To Start");
            lcd.setCursor(0,1);
            lcd.print("Closing         ");
            state_gate_flag = 10;
            break;
             
          case 10: //A has been pressed to start closing the gate again
            timeout_Until_Gate_Closes = millis() - time_Initiated_Millis; //set the time before autoclose initiates
            time_Left_To_Move = timing;
            initial_Time_Left_To_Move = time_Left_To_Move;
            time_Initiated_Millis = millis();
            digitalWrite(relay_open_pin_off, HIGH);
            digitalWrite(relay_close_pin_off, LOW); // sends a signal to the close pin to start closing 
            state_gate_flag = 11;
            break; 

          //case 11 is handled in the gatestop() function, and moves on to case 12 if the new settings are confirmed

          case 12: //finishes setup if A is pressed and released.  If B was pressed it will be picked up in the remoteBmonitor() function
            remote_B_action_flag = 0;
            EEPROM.put(0, 3);  //puts 3 in slot 0 of the EEPROM (the setup_flag), meaning that after a power outage, the arduino will detect that setup has been completed
            EEPROM.put(10, timing); //puts selected settings into the system
            EEPROM.put(20, timeout_Until_Gate_Closes);
            state_gate_flag = 0;
            break;
      }
        
    }
}

void remoteBmonitor()
{
  if (remote_B_action_flag == 1)  //if button B is live
  {
    if (digitalRead(buttonB_pin) == LOW)
    {
      digitalWrite(relay_close_pin_off, HIGH); //gate stops closing
      digitalWrite(relay_open_pin_off, HIGH);
    }
    if (digitalRead(buttonB_pin) == HIGH)
    {
      if(state_gate_flag == 7)  //setup sequence monitoring
      {
       digitalWrite(relay_close_pin_off, LOW); //gate begins to close
       digitalWrite(relay_open_pin_off, HIGH);
      }
      if (state_gate_flag == 12) //if user wants to reset
      {
        state_gate_flag = 13;
        remote_B_action_flag = 0;
      }
      if (dip_Setup_Millis >= 1) //if the dip switch has recently been turned on
      {
        dip_Setup_Millis = 0; //reset this variable
        remote_B_action_flag = 0;
        state_gate_flag = 13; //reset the system  
      } 
    }
  }
}

void gatestop()
{
  if ((state_gate_flag == 1) or (state_gate_flag == 3) or (state_gate_flag == 11)) //if the gate is in motion
  {
    time_Left_To_Move = (initial_Time_Left_To_Move) - (millis() - time_A_Pressed_Millis); //calculate time left to move
    if (time_Left_To_Move <= 0) // if the timer has run out
    {
      digitalWrite(relay_open_pin_off, HIGH); //gate stops, fully open
      digitalWrite(relay_close_pin_off, HIGH); //for safety
      time_A_Pressed_Millis = 0;
      time_Initiated_Millis = 0;
      
      if (state_gate_flag == 1) //if the gate is opening, setup variables for autoclose
      {  
        if (gate_was_user_stopped_flag == 0) //if the gate has finished opening normally
        {
          time_Autoclose_Initiates = millis();  //autoclose initiates
          time_Left_To_Move = timeout_Until_Gate_Closes;
          initial_Time_Left_To_Move = time_Left_To_Move;
          state_gate_flag = 6; //the gate is pending autoclose
        }
        if (gate_was_user_stopped_flag == 1) //if the gate finished opening as a result of the user stopping closure, autoclose won't initiate
        {
          state_gate_flag = 2; //the gate is all the way open
          gate_was_user_stopped_flag = 0; //reset flag
        }
      }
      if (state_gate_flag == 3) //if the gate is closing
      {
        time_Left_To_Move = timing;  //resets the initial variables
        initial_Time_Left_To_Move = timing;
        state_gate_flag = 0;  //the gate stops moving, fully closed
      }
      if (state_gate_flag == 11) //if the gate autoclose is ending during initial setup
      {
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("A Confirm ac");
          lcd.print(timeout_Until_Gate_Closes/1000, 1);
          lcd.setCursor(0,1);
          lcd.print("B Restart tm");
          lcd.print(timing/1000, 1);
          remote_B_action_flag = 1; //B button is on again
          state_gate_flag = 12;   
      }
    }  
  }
  if ((state_gate_flag == 2) or (state_gate_flag == 0)) //if the gate is fully open or closed
  {
    time_Left_To_Move = timing; //sets the time left to move so the display shows correctly
    initial_Time_Left_To_Move = time_Left_To_Move;
    time_Initiated_Millis = 0;
  }
}

void autoclose() //keeps the gate open while the autoclose period passes
{
  if (state_gate_flag == 6) //if the autoclose waiting period has started
  {
    time_Left_To_Move = (initial_Time_Left_To_Move) - (millis() - time_Autoclose_Initiates);
    if (time_Left_To_Move <= 0) //if the autoclose waiting period has ended, put gate in closing mode
    {
      time_Left_To_Move = timing;
      initial_Time_Left_To_Move = time_Left_To_Move;
      time_Initiated_Millis = 0;
      time_Autoclose_Initiates = 0;
      time_A_Pressed_Millis = millis(); // simulates that the remote button was pressed
      digitalWrite(relay_close_pin_off, LOW); // sends a signal to the close pin to start closing 
      state_gate_flag = 3;  //gate will begin standard closing
    }
  }
}

void lcdupdate()
{
  dip_flag = digitalRead(dip_pin);
  if ((dip_flag == HIGH) or (state_gate_flag >= 7)) //if dip is on or setup is ongoing
  {
    lcd.display();
    lcd.setBacklight(255); 
  }
  if ((dip_flag == LOW) and (state_gate_flag <= 6)) //if the dip is off and setup is done
  {
    lcd.noDisplay();
    lcd.setBacklight(0);
  }  
  if (((state_gate_flag <= 6) or (state_gate_flag == 11)) and dip_Setup_Millis == 0)  //normal operation or during setup autoclose and if DIP not recently turned on
  {
    lcd.setCursor(0,0);
    lcd.print("T:        A:");
    lcd.setCursor(0,1);
    lcd.print(gateStatus[state_gate_flag]); 
    lcd.setCursor(12,0);
    lcd.print(amps, 2);
    lcd.setCursor(2,0);
    lcd.print(timing/1000, 1);
    lcd.print("s ");
    lcd.setCursor(11, 1); // column, row
    if (time_Left_To_Move <= 9999) //keeps the columns from jumping
    {
      lcd.print(" ");
    }
    lcd.print(time_Left_To_Move / 1000, 1); //Time gate has been in motion
    lcd.print("s ");
  }
}

void currentmonitoring()
{
  current = analogRead(amperage_pin); //read the analog in value
  inputStats.input(current);//log to Stats function

  if((unsigned long)(millis() - previous712Millis) >= reading712Period)//every second it calculates the reading
  {
    previous712Millis = millis(); //start a new time interval, also prevents rollover (i think)
    amps = intercept + slope * inputStats.sigma();
  }

  if ((amps >= current_limit) and (millis()>4000)) //High current more than 4 seconds after system startup
  {
    if ((unsigned long)(millis() - previous_High_Amp_Millis) >= reading_Amp_Lockout_Period)// and longer than the lockout
    {
      previous_High_Amp_Millis = millis();//start a new time interval, and prevents rollover, i think
      digitalWrite(relay_open_pin_off, HIGH);
      digitalWrite(relay_close_pin_off, HIGH);
      lcd.setCursor(0,1);
      lcd.print("HighCurrnt");
      delay(2000); //leave msg on for 2 seconds
      if(state_gate_flag == 1) //if the gate is moving open
      {
        digitalWrite(relay_open_pin_off, HIGH);
        digitalWrite(relay_close_pin_off, LOW);
        state_gate_flag = 3;
      }
      else //if the gate is moving closed ie state_gate_flag is 3 and the prior if did not just make it 3
      {
        digitalWrite(relay_close_pin_off, HIGH);
        digitalWrite(relay_open_pin_off, LOW);
        state_gate_flag = 1;
      } 
    }
  }
}

void changechecker()
{
  if (digitalRead(dip_pin) != previous_State_Dip)
  {   
    if (digitalRead(dip_pin) == 1)
    {
      dip_Setup_Millis = dip_Setup_Delay + millis(); //sets the timer to the end time
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Press B to setup");
      remote_B_action_flag = 1;
      previous_State_Dip = 1;
    }
    if (digitalRead(dip_pin) == 0)
    {
      previous_State_Dip = 0;
      remote_B_action_flag = 0;    
    }
  }
  if (dip_Setup_Millis >= millis()) //if the dip waiting period is active, continue the timer
  {
    lcd.setCursor(7,1);
    lcd.print((dip_Setup_Millis - millis())/1000, 1);
  }
  if ((dip_Setup_Millis > 0) and (dip_Setup_Millis <= millis())) //the dip waiting period has ended
  {
    dip_Setup_Millis = 0;
    remote_B_action_flag = 0;
  }
  
  if ((digitalRead(buttonA_pin) != previous_State_A) or (digitalRead(buttonB_pin) != previous_State_B)) //if there has been a change
  {
    if (digitalRead(buttonA_pin) != previous_State_A)
    {   
      button_Status = previous_State_A + 1; //returns 1 if button A is released, 2 if pressed
    }
    if (digitalRead(buttonB_pin) != previous_State_B)
    {
      button_Status = previous_State_B + 3; //returns 3 if button B is released, 4 if pressed
    }
    
    previous_State_A = digitalRead(buttonA_pin);
    previous_State_B = digitalRead(buttonB_pin);
  }
}
