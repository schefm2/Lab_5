/*  
	Names: Sydney Bahs, Tom Saad, Matthew Scheffer
    Section: 2
    Date: 12/13/17
    File name: lab5.c
    Description: 
*/
#include <stdio.h>
#include <stdlib.h>
#include <c8051_SDCC.h>
#include <i2c.h>

#define RANGER_ADDR 0xE0
#define COMPASS_ADDR 0xC0
#define ACCEL_ADDR 0x3A
#define PING_CM 0x51
#define PCA_START 28672     //PCA0 value for a pulse of ~20 ms

//Max left and right pulse widths set so the servo isn't strained
#define SERVO_LEFT_PW 2425
#define SERVO_CENTER_PW 2895
#define SERVO_RIGHT_PW 3245

//Max forward and reverse pulse widths set so the motor isn't strained
#define MOTOR_REVERSE_PW 2027 
#define MOTOR_NEUTRAL_PW 2765
#define MOTOR_FORWARD_PW 3502

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------

//Initialization Functions
void Port_Init(void);
void PCA_Init (void);
void XBR0_Init(void);
void Interrupt_Init(void);
void ADC_Init(void);
void SMB_Init(void);

//High Level Functions
void Set_Neutral(void);
void Set_Motion(void);
void Start_Parameters(void);
void Print_Data(void);

//Low Level Functions
void BeginDrive(void);
void StopDrive(void);
void Read_Accel(void);
void Set_Servo_PWM(void);
void Set_Motor_PWM(void);
void Pause(void);
void Wait(void);
unsigned int pow(unsigned int a, unsigned char b);
unsigned int calibrate(void);
unsigned char parallel_input(void);
unsigned char read_AD_input(unsigned char pin_number);
void Calibrate_Accel(void);
void Buzzer_Sound(void);
unsigned char Calculate_Gain(void);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

unsigned char Data[5];  //Data array used to read and write to I2C Bus slaves
signed long xaccel, yaccel, xoffset=0, yoffset=0, error_sum=0;
unsigned int Servo_PW, Motor_PW;
unsigned char kdx, kdy, ks, ki; //Feedback gains for x-axis of car, y-axis of car, and steering
unsigned char keyboard, keypad, accel_count, print_count, wait_count;
float xpercent, ypercent; //Time is in tenths of a second

__bit servo_stop, motor_stop, accel_flag, print_flag, post_start=0; //flags

//sbits
__sbit __at 0xB5 BUZZ;  //P3.5 (pin 34 on EVB connector); buzzer for running up slope
__sbit __at 0xB6 SS1;   //P3.6 (pin 31 on EVB connector); slideswitch run/stop for Servo
__sbit __at 0xB7 SS2;   //P3.7 (pin 32 on EVB connector); slideswitch run/stop for Drive Motor

//-----------------------------------------------------------------------------
// Main Function
//-----------------------------------------------------------------------------
void main(void)
{
    //Initialize board
    Sys_Init();
    putchar(' '); //The quotes in this line may not format correctly
    Port_Init();
    XBR0_Init();
    Interrupt_Init();
    PCA_Init();
    SMB_Init();
    ADC_Init();	//Must come after PCA_Init to allow capacitors to charge
    Accel_Init_C();
    printf("\r\nStart.\r\n");

    
    Start_Parameters();
    
    while(1)
    {
        //Gain can change mid drive, if wanted.
        kdy = Calculate_Gain();

        //Just started, drive forward until on incline
        if (!post_start)
        {
            BeginDrive();
        }
        //Exited the function: we finished the flat starting area.
        //Signal this so we can stop when we next hit a flat area.
        post_start = 1;

        Print_Data();
		Read_Accel();

        //drive backwards up slope
        Set_Neutral(); //check for stop servo/motor
        Set_Motion();
        Buzzer_Sound();

        //store slope readings if higher than max
        //max = (accel > max) ? accel : max;

        if (post_start)
        {
            StopDrive();
        }
    }
}

//----------------------------------------------------------------------------
// Begin Drive
//----------------------------------------------------------------------------
// Starts driving, slowly increasing speed until it leaves "flat" ground.
void BeginDrive( void )
{
    //Assume noisy from motor: 5% tolerance (max accel is 10k, 600/10k = 6% ~ 5%).
    while ( yaccel > -300 && yaccel < 300
            && xaccel > -300 && xaccel < 300 )
    {
        Print_Data();
        Read_Accel();
        //Slowly go faster - lessens the noise from the motor on the accelerometer.
        Motor_PW = (Motor_PW+20 > MOTOR_REVERSE_PW) ? MOTOR_REVERSE_PW : Motor_PW+20;
        PCA0CP2 = 0xFFFF - Motor_PW;
    }
}

//----------------------------------------------------------------------------
// Stop Drive
//----------------------------------------------------------------------------
// Stops driving, turn off buzzer; we are on "flat" ground.
void StopDrive( void )
{
    //stop while accel readings indicate flat ground
    //wheels add their own noise too
    while ( yaccel > -300 && yaccel < 300
            && xaccel > -300 && xaccel < 300 )
    {
        //Turn everything off/neutral and sit.
        BUZZ = 1;
        Servo_PW = SERVO_CENTER_PW;
        Motor_PW = MOTOR_NEUTRAL_PW;
        PCA0CP0 = 0xFFFF - Servo_PW;
        PCA0CP2 = 0xFFFF - Motor_PW;
    }
}

//----------------------------------------------------------------------------
// Start_Parameters
//----------------------------------------------------------------------------
// Allows user to set three gains for the car; the front-to-back gain is set
// with the pot and the side-to-side gain and steerin gain are set with either
// the keypad or the keyboard.
void Start_Parameters(void)
{
    Servo_PW = SERVO_CENTER_PW;		//Initialize car to straight steering and no movement
    Motor_PW = MOTOR_NEUTRAL_PW;	//Set pulse to stop car
    PCA0CP0 = 0xFFFF - Servo_PW;	//tell hardware to use new servo pulse width
    PCA0CP2 = 0xFFFF - Motor_PW;	//tell hardware to use new motor pulse width

    Wait();         //Wait for 1 second
    lcd_clear();    //clear lcd screen
    lcd_print("Calibration:\nHello world!\n012_345_678:\nabc def ghij");	
	Wait();         //wait 1 second
	
	
	lcd_clear();
	lcd_print("Set gain with pot");		//tell user to set gain with potentiometer
	printf("\r\nTurn the potentiometer clockwise to increase the front-to-back gain from 0 to 50.\r\nPress # when you are finished.\r\n");
    while (parallel_input() != '#')     //Waits until user presses # to set the gain
    {
        kdy = Calculate_Gain(); //set gain from pot
        //temp = gain;    //Cast gain as an unsigned int
        printf("\rYour gain is currently: %u   ", kdy);  //Print selected gain; overwrites existing print statement
        lcd_clear();    //Clear lcd screen
        lcd_print("Gain is: %u", kdy); //Print selected gain to screen
    }
    printf("\r\nYou selected %u as your front-to-back gain.", kdy);   //Print to confirm final gain
    lcd_print("\nFinal value above");               //Print to confirm final gain
    Wait();
    
	
	do
	{
		lcd_clear(); //clear screen
		lcd_print("Press 5 keys."); //print instructions
		printf("\r\nSelect a side-to-side gain (0 to 50) by inputing 5 digits. Lead with 0's. Press # to confirm.\r\n");
		kdx = calibrate();	//take 5 digit input
		Wait(); //wait a second
	}
	while (kdx > 50); //wait until you get appropriate gain
	printf("\r\nYou selected %u as your side-to-side gain", kdx); //print side-to-side gain
    lcd_print("\nFinal value above");  
    Wait();
	
	do
	{
		lcd_clear(); //clear screen
		lcd_print("Press 5 keys."); //print instructions
		printf("\r\nSelect a steering gain (0 to 50) by inputing 5 digits. Lead with 0's. Press # to confirm.\r\n");
		ks = calibrate();	//take 5 digit input
		Wait(); //wait a second
	}
	while (ks > 50); //wait until you get appropriate gain
	printf("\r\nYou selected %u as your steering gain", ks); //print steering gain
    lcd_print("\nFinal value above");  
    Wait();
    
    do
	{
		lcd_clear(); //clear screen
		lcd_print("Press 5 keys."); //print instructions
		printf("\r\nSelect a integral gain (0 to 50) by inputing 5 digits. Lead with 0's. Press # to confirm.\r\n");
		ki = calibrate();	//take 5 digit input
		Wait(); //wait a second
	}
	while (ki > 50); //wait until you get appropriate gain
	printf("\r\nYou selected %u as your integral gain", ki); //print integral gain
    lcd_print("\nFinal value above");  
    Wait();
    
    printf("\r\nCalibrating accelerometer...");
    Calibrate_Accel();
    printf("\r\nYour xoffset: %ld\r\nYour yoffset: %ld", xoffset, yoffset);
}

//----------------------------------------------------------------------------
//Set_Motion
//----------------------------------------------------------------------------
void Set_Motion(void)
{
	//read sensors and set pulse widths

    Set_Servo_PWM(); //steering control
    Set_Motor_PWM(); //speed control
}

//----------------------------------------------------------------------------
//Set_Neutral
//----------------------------------------------------------------------------
void Set_Neutral(void)
{
	//set servo to center and stop motor
    if (SS1)
    {
		Servo_PW = SERVO_CENTER_PW;
        servo_stop = 1;
    }
    else
    {
        servo_stop = 0;
    }
    if (SS2)
    {
        Motor_PW = MOTOR_NEUTRAL_PW;
        motor_stop = 1;
    }
    else
    {
        motor_stop = 0;
    }
    PCA0CP0 = 0xFFFF - Servo_PW;
    PCA0CP2 = 0xFFFF - Motor_PW;
}

//----------------------------------------------------------------------------
//Print_Data
//----------------------------------------------------------------------------
void Print_Data(void)
{
    if(print_flag)
		//Only prints every ~400 ms
    {
        print_flag = 0;
        printf("\r\n%ld,%ld,%u,%u,%u,%u", xaccel, yaccel, kdx, kdy, ks, Motor_PW);
        lcd_clear();
        lcd_print("x-angle: %u\ny-angle: %u\nGains (x,y,s): %u, %u, %u\nMotor PW: %u", xaccel, yaccel, kdx, kdy, ks, Motor_PW);
    }
}

//LOW LEVEL FUNCTIONS

//----------------------------------------------------------------------------
//Set_Servo_PWM
//----------------------------------------------------------------------------
void Set_Servo_PWM(void)
{
    if (servo_stop)
    {
        return;
    }

	//Servo_PW set to value based on heading_error modified by gain set in Car_Parameters()
    //correction for xaccel should be  in opposite direction to the accel; want to balance, not increase, the tilt
    //did math, not sure????
	Servo_PW = SERVO_CENTER_PW + ks*xpercent;

    //Additional precaution: if Servo_PW somehow exceeds the limits set in Lab 3-1,
    //then Servo_PW is set to corresponding endpoint of PW range [SERVO_LEFT_PW, SERVO_RIGHT_PW]

    //Check for out-of-bounds:
    Servo_PW =  (Servo_PW < SERVO_LEFT_PW) ? SERVO_LEFT_PW :
                (Servo_PW > SERVO_RIGHT_PW) ? SERVO_RIGHT_PW :
                Servo_PW;

	//if (Servo_PW > SERVO_RIGHT_PW) Servo_PW = SERVO_RIGHT_PW;
	//if (Servo_PW < SERVO_LEFT_PW) Servo_PW = SERVO_LEFT_PW;
	PCA0CP0 = 0xFFFF - Servo_PW;
}

//----------------------------------------------------------------------------
//Set_Motor_PWM
//----------------------------------------------------------------------------
void Set_Motor_PWM(void)
{
    if (motor_stop)
    {
        return;
    }

	//Add correction for front-to-back tilt, forcing a forward movement to climb the slope.
	//Add correction for side-to-side tilt, forcing a forward movement to turn the car.
    //Integral term:
	error_sum += ypercent + abs(xpercent);

    // kdy is the y-axis drive feedback gain; kdx is the x-axis drive feedback gain; ki is the integral gain
	Motor_PW = MOTOR_NEUTRAL_PW + kdy*ypercent + kdx*abs(xpercent) + ki*error_sum;

	//Motor_PW = MOTOR_NEUTRAL_PW+kdy*yaccel; // kdy is the y-axis drive feedback gain
	//Motor_PW += kdx*abs(xaccel); //kdx is the x-axis drive feedback gain
	//Motor_PW += kdx * abs(xaccel) + ki * error_sum //ki is the integral gain

    //Check for out-of-bounds:
    Motor_PW =  (Motor_PW < MOTOR_REVERSE_PW) ? MOTOR_REVERSE_PW :
                (Motor_PW > MOTOR_FORWARD_PW) ? MOTOR_FORWARD_PW :
                Motor_PW;

	PCA0CP2 = 0xFFFF - Motor_PW;
}

//----------------------------------------------------------------------------
//Pause
//----------------------------------------------------------------------------
void Pause(void)
{   
	//Stop for 40 ms
	wait_count = 0;
	while (wait_count < 2){}
    
}

//----------------------------------------------------------------------------
//Wait
//----------------------------------------------------------------------------
void Wait(void)
{   
	//Stop for 1000 ms
	wait_count = 0;
	while (wait_count < 50){}
    
}

//----------------------------------------------------------------------------
//Pow
//----------------------------------------------------------------------------
//
// Stripped back version of math.h power function, used in calibrate(). 
// Raises a to the power of b.
//
unsigned int pow(unsigned int a, unsigned char b)
{
    unsigned char i;
    unsigned char base = a;

    if (b == 0) return 1;
    for(i = 1; i < b; i++)
        a = a*base;
    return a;
}

//----------------------------------------------------------------------------
//calibrate
//----------------------------------------------------------------------------
unsigned int calibrate(void)
{
	unsigned char isPress = 0;
	unsigned char pressCheck = 0;
	unsigned int value = 0;	//Final value to be returned
	
    for (;pressCheck < 5;pressCheck++)
        Data[pressCheck] = 0;
    pressCheck = 0;
	while(1)
	{
		keyboard = getchar_nw();	//This constantly sets keyboard to whatever char is in the terminal
		keypad = read_keypad();		//This constantly sets the keypad to whatever char is on the LCD
		Pause();					//Pause necessary to prevent overreading the keypad

		if (keyboard == '#' || keypad == '#') //# is a confirm key, so it will finish calibrate()
        {
            printf("\r\nThis is isPress: %u\r\n", isPress);
            for (pressCheck = 0; 0 < isPress; isPress--)
            {
                value += Data[pressCheck++]*pow(10,isPress - 1);
                printf("\r\nThis is value: %u\r\n", value);
            }
            if (value > 255)    //If the gain is set too high, set to saturation for the unsigned char gains
                return 255;
			return value;	
        }

		if (isPress > pressCheck && keypad == 0xFF && keyboard == 0xFF)	//Only increments pressCheck if held key is released
			pressCheck++;
		

		if (pressCheck == 6)	//If a 6th key is pressed, then released
		{
            for (pressCheck = 0;pressCheck < 5;pressCheck++)
                Data[pressCheck] = 0;
			isPress = pressCheck = 0;	//Reset the flags
			lcd_print("\b\b\b\b\b\b");	//Clear value displayed on LCD, needs an extra \b for some reason?
			printf("\r      \r");	//Clear value displayed on terminal
			
		}

		
		if (isPress == pressCheck)	//pressCheck must be equal to isPress, only occurs if no key is held down
		{
			if (keypad != 0xFF)		//When an actual key is held down
			{
				lcd_print("%c",keypad);	//Adds pressed key to LCD screen
				printf("%c", keypad);	//Adds pressed key to computer terminal
				Data[isPress] = ((unsigned int)(keypad - '0'));	
				isPress++;
			}
			if (keyboard != 0xFF)	//When an actual key is held down
			{
				lcd_print("%c",keyboard);	//Adds pressed key to LCD screen
				//printf("%c", keyboard); this line is not necessary as getchar_nw automatically executes a putchar()
				Data[isPress] = ((unsigned int)(keyboard - '0'));
				isPress++;	
			}
		}
	}
}
//----------------------------------------------------------------------------
//parallel_input
//----------------------------------------------------------------------------
//
// Function designed to take input from either the keyboard or keypad; must be
// called multiple times in a while loop until desired value is input.
//
unsigned char parallel_input(void)
{
    unsigned char keypad;
    unsigned char keyboard;

	keyboard = getchar_nw();	//This constantly sets keyboard to whatever char is in the terminal
	keypad = read_keypad();		//This constantly sets the keypad to whatever char is on the LCD
	Pause();					//Pause necessary to prevent overreading the keypad

	//Returns the value of the respective input that has a key pressed
	if (keyboard != 0xFF)
		return keyboard;
	if (keypad != 0xFF)
		return keypad;
	else
		return 0;	//Return 0 if no key is pressed
}
//----------------------------------------------------------------------------
//read_AD_input
//----------------------------------------------------------------------------
unsigned char read_AD_input(unsigned char pin_number)
{
    AMX1SL = pin_number;		//Sets multiplexer to convert correct pin
    ADC1CN &= ~0x20;			//Clears the A/D conversion complete bit
    ADC1CN |= 0x10;				//Starts A/D conversion
    while(!(ADC1CN & 0x20));	//Waits until conversion completes 
    return ADC1;				//returns converted input, 0-255 inclusive
}
//----------------------------------------------------------------------------
//ADC_Init
//----------------------------------------------------------------------------
void ADC_Init(void)
{
    REF0CN = 0x03;	//Sets V_ref as 2.4V
    ADC1CN = 0x80;	//Enables AD/C converter

    //Gives capacitors in A/D converter time to charge
    wait_count = 0;
    while(wait_count < 6);

    //Sets gain to 1
    ADC1CF |= 0x01;
    ADC1CF &= 0xFD;
}

//-----------------------------------------------------------------------------
//Port_Init
//-----------------------------------------------------------------------------
void Port_Init()
{   
	//Initailize POT
	P1MDOUT |= 0x05;	//Set output pin for CEX0 and CEX2 in push-pull mode
	P1MDOUT &= ~0x80;	//Set potentiometer pin (P1.7) to open drain
	P1 |= 0x80;			//Set impedance high on P1.7
	P1MDIN &= ~0x80;	//Set P1.7 to analog input
	
	P3MDOUT &= ~0xC0;   //Pin 3.7, 3.8 open drain
    P3MDOUT |= 0x20;    //Pin 3.5 push/pull
	P3 |= 0xC0;         //Pin 3.7,3.8 high impedance
    
}

//-----------------------------------------------------------------------------
// Interrupt_Init
//-----------------------------------------------------------------------------
//
// Set up ports for input and output
//
void Interrupt_Init(void)
{
    // IE and EIE1
    EA = 1;
    EIE1 |= 0x08;
}

//-----------------------------------------------------------------------------
// XBR0_Init
//-----------------------------------------------------------------------------
//
// Set up the crossbar
//
void XBR0_Init(void)
{
    XBR0 = 0x27;	//configure crossbar with UART, SPI, SMBus, and CEX channels as
					//in worksheet
}

//-----------------------------------------------------------------------------
// PCA_Init
//-----------------------------------------------------------------------------
//
// Set up Programmable Counter Array
//
void PCA_Init(void)
{
    // reference to the sample code in Example 4.5 - Pulse Width Modulation implemented using
    // Use a 16 bit counter with SYSCLK/12.
    PCA0MD = 0x81;
    PCA0CPM0 = PCA0CPM2 = 0xC2;		//Sets both CCM0 and CCM2 in 16-bit compare mode, enables PWM
    PCA0CN = 0x40; //Enable PCA counter
}

//-----------------------------------------------------------------------------
// PCA_ISR
//-----------------------------------------------------------------------------
//
// Interrupt Service Routine for Programmable Counter Array Overflow Interrupt
//
void PCA_ISR ( void ) __interrupt 9
{   
    if(CF)
    {
        CF=0;           //clear flag
        PCA0 = 28672;   //determine period to 20 ms
        accel_count++;
		print_count++;
        wait_count++;

        if (accel_count >= 1)   //Accelerometer won't read unless 20 ms has passed
        {
            accel_flag = 1;
            accel_count = 0;
        }
        if (print_count >= 20)  //Prints will only occur every 400 ms
        {
            print_flag = 1;
            print_count = 0;
        }
        
    }
    PCA0CN &= 0x40; //Handle other interupt sources
    // reference to the sample code in Example 4.5 -Pulse Width Modulation implemented using
}

//-----------------------------------------------------------------------------
// SMB_Init
//-----------------------------------------------------------------------------
//
// Set up the I2C Bus
//
void SMB_Init()
{
    SMB0CR = 0x93;	//Sets SCL to 100 kHz (actually ~94594 Hz)
    ENSMB = 1;		//Enables SMB
}

//-----------------------------------------------------------------------------
//
// Read Accelerometer
//
void Read_Accel()
{
	char i=0; //counter variable
	xaccel = 0; //reset x reading
	yaccel = 0; //reset y reading

	for(i=0;i<8;i++) //loop through 8 iterations
	{
		while(!accel_flag);
		accel_flag=0;
		i2c_read_data(ACCEL_ADDR, 0x27, Data, 1); //read status registers
		if((Data[0]&0x03)==0x03) //are the 
		{
			while(!accel_flag);
			accel_flag=0;
			i2c_read_data(ACCEL_ADDR, 0x28, Data, 4); //read angle registers
			xaccel +=Data[1]<<8 | Data[0]>>4; //set and total x values
			yaccel +=Data[3]<<8 | Data[2]>>4; //set and total y values
		}
	}
	xaccel = (xaccel>>3)-xoffset; //average by dividing by 8 and subtract offset
    xpercent = (float) xaccel; //gravity is 9800 mm/s^2, so about 10k for error would be max.
    xpercent /= 10000; //gravity is 9800 mm/s^2, so about 10k for error would be max.

	yaccel = (yaccel>>3)-yoffset; //average by dividing by 8 and subtract offset
    ypercent = (float) yaccel; //gravity is 9800 mm/s^2, so about 10k for error would be may.
    ypercent /= 10000; //gravity is 9800 mm/s^2, so about 10k for error would be may.
    //ypercent = (float) yaccel/10000; //gravity is 9800 mm/s^2, so about 10k for error would be may.
}
//-----------------------------------------------------------------------------
//Calibrate Accelerometer
//-----------------------------------------------------------------------------
void Calibrate_Accel()
{
	char i=0; //counter variable
	xoffset=0; //reset x reading
	yoffset=0; //reset y reading


	for(i=0;i<64;i++) //loop through 8 iterations
	{
		while(!accel_flag);
		accel_flag=0;
		i2c_read_data(ACCEL_ADDR, 0x27, Data, 1); //read status registers
		if((Data[0]&0x03)==0x03) //are the 
		{
			while(!accel_flag);
			accel_flag=0;
			i2c_read_data(ACCEL_ADDR, 0x28, Data, 4); //read angle registers
			xoffset +=Data[1]<<8 | Data[0]>>4; //set and total x values
			yoffset +=Data[3]<<8 | Data[2]>>4; //set and total y values
		}
	}
	xoffset = xoffset>>6; //average by dividing by 64
	yoffset = yoffset>>6; //average by dividing by 64
}
//-----------------------------------------------------------------------------
// Buzzer_Sound
//-----------------------------------------------------------------------------
// Sets the buzzer to turn on for .5 seconds, then off for 1.0 seconds during
// Normal operating conditions (i.e., when the car is in motion even if Servo
// is set off)
void Buzzer_Sound(void)
{
    //Checks if the Drive motor has been set to neutral
    if (motor_stop)
    {
        BUZZ = 1;   //Turns buzzer off
        return;
    }
    
    if (wait_count > 75)    //75 pules of 20 ms makes up 1.5 seconds
    {
        wait_count = 0;     //Ensures that the timer is capped at 1.5 seconds
        BUZZ = 0;           //Sets buzzer on
    }
    
    if (wait_count > 25)    //After buzzer is on for .5 seconds...
    {
        BUZZ = 1;           //Turn off for 1 second
    }
}
//-----------------------------------------------------------------------------
// Calculate_Gain
//-----------------------------------------------------------------------------
//
//
//
unsigned char Calculate_Gain(void)
{
    return ((float)read_AD_input(7) / 255) * 50;
}

//-----------------------------------------------------------------------------
