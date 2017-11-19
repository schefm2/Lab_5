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

//Left and right pulse widths set so that Servo isn't strained
#define SERVO_LEFT_PW 2425
#define SERVO_CENTER_PW 2895
#define SERVO_RIGHT_PW 3245
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
void Start_Parameters(void);
void Print_Data(void);

//Low Level Functions
void Read_Accel(void);
void Set_Servo_PWM(void);
void Set_Motor_PWM(void);
void Pause(void);
void Wait(void);
unsigned int pow(unsigned int a, unsigned char b);
unsigned int calibrate(void);
unsigned char parallel_input(void);
unsigned char read_AD_input(unsigned char pin_number);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
unsigned char Data[5];  //Data array used to read and write to I2C Bus slaves
unsigned int desired_heading = 0, xaccel, yaccel, xoffset=0, yoffset=0;
unsigned int initial_speed = MOTOR_NEUTRAL_PW;
unsigned int PCA_overflows, current_heading, range, Servo_PW, Motor_PW;
unsigned char keyboard, keypad, accel_count, print_count, accel_flag, print_flag, answer, first_obstacle;
signed int heading_error;
float gain, time; //Time is in tenths of a second

//sbits
__sbit __at 0xB6 SS1;   //P3.6 (pin 31 on EVB connector); slideswitch run/stop for Servo
__sbit __at 0xB7 SS2;   //P3.7 (pin 32 on EVB connector); slideswitch run/stop for Drive Motor

//-----------------------------------------------------------------------------
// Main Function
//-----------------------------------------------------------------------------
void main(void)
{
    //Initialize board
    Accel_Init_C();
    Sys_Init();
    putchar(' '); //The quotes in this line may not format correctly
    Port_Init();
    XBR0_Init();
    Interrupt_Init();
    PCA_Init();
    SMB_Init();
    ADC_Init();	//Must come after PCA_Init to allow capacitors to charge
    
    printf("\r\nStart.\r\n");
    while(1)
    {

    }
}

//----------------------------------------------------------------------------
// Start_Parameters
//----------------------------------------------------------------------------
void Start_Parameters(void)
{
    /*
       Allow user to enter initial speed, desired heading, and other parameters.  This can be done by
       pressing keys on the keyboard or keypad. The system should allow the operator to enter a specific
       desired angle, or pick from a predefined list of 0°, 90°, 180°, 270°. If the user is to enter a
       desired angle, the SecureCRT or LCD screen should indicate so with a prompt. If the user is to 
       choose an angle from a predefined list, the screen should show the list along with the key press 
       needed to select that angle. The steering gain and drive gain (only for obstacle tracking, not used 
       here) must also be selectable. 
       Configure the A/D converter in the C8051 to read a potentiometer voltage. As mentioned previously, 
       the potentiometer is used to select the gain. This is set once with the other initializations. The
       final value must be displayed for the user to see, and allowing the user to make adjustments until
       a desired value is set is a nice feature.
     */
	unsigned int temp;	//Used to print gain to the LCD
    Servo_PW = SERVO_CENTER_PW;		//Initialize car to straight steering and no movement
    Motor_PW = MOTOR_NEUTRAL_PW;	//Set pulse to stop car
    PCA0CP0 = 0xFFFF - Servo_PW;	//tell hardware to use new servo pulse width
    PCA0CP2 = 0xFFFF - Motor_PW;	//tell hardware to use new motor pulse width


    printf("\nStart");		//print start

    Wait();					//Wait for 1 second
    lcd_clear();			//clear lcd screen
    lcd_print("Calibration:\nHello world!\n012_345_678:\nabc def ghij");	
	Wait();		//wait 1 second
	
	
	lcd_clear();
	lcd_print("Set gain with pot");		//tell user to set gain with potentiometer
	printf("\r\nTurn the potentiometer clockwise to increase the steering gain from 0 to 10.2.\r\nPress # when you are finished.");
	calibrate();		//take 5 digit input
	gain = ((float)read_AD_input(7) / 255) * 10.2; //set gain from pot
	printf_fast_f("\r\nYour gain is %3.1f", gain); //print gain
	temp = (unsigned char)(gain/10.2*100);
	lcd_print("\nGain is %u of 100",temp);
	Wait(); //wait a second
	
	do
	{
		lcd_clear(); //clear screen
		lcd_print("Press 5 keys.\n"); //print instructions
		printf("\r\nSelect a desired heading (0 to 3599) by inputing 5 digits. Lead with a 0. Press # to confirm.\r\n");
		desired_heading = calibrate();	//take 5 digit input
		Wait(); //wait a second
	}
	while (desired_heading > 3599); //wait until you get appropriate heading
	printf("\r\nYou selected %u as your heading", desired_heading); //print heading
	
	do
	{
		lcd_clear();
		lcd_print("Press 5 keys.\n");	//print instructions
		printf("\r\nSelect an initial speed (2765 to 3502) by inputing 5 digits. Lead with a 0. Press # to confirm.\r\n");
		initial_speed = calibrate(); //take 5 digits 
		Wait(); //wait a second
	}
	while (initial_speed < 2765 || initial_speed > 3502);  //wait for appropriate speed
	printf("\r\nYou selected %u as your speed", initial_speed);  //print speed
	Wait();  //wait a second
	Motor_PW = initial_speed;
	PCA0CP2 = 0xFFFF - Motor_PW; //activate motor
}

//----------------------------------------------------------------------------
//Set_Motion
//----------------------------------------------------------------------------
void Set_Motion(void)
{
	//read sensors and set pulse widths

    Set_Servo_PWM();
    Set_Motor_PWM();
}

//----------------------------------------------------------------------------
//Set_Neutral
//----------------------------------------------------------------------------
void Set_Neutral(void)
{
	//set servo to center and stop motor
    if (SS1)
    {
		PCA0CP0 = 0xFFFF - SERVO_CENTER_PW;
		PCA0CP2 = 0xFFFF - MOTOR_NEUTRAL_PW;

        while(SS1) {}	//wait until slideswitch is turned OFF
    }
}

//----------------------------------------------------------------------------
//Read_Print
//----------------------------------------------------------------------------
//
// Used to allow the car to continue making readings to print those readings
// while it is in non-normal operating conditions (i.e. encountering obstacle).
//
void Read_Print(void)
{
    //Read_Compass();
    //Read_Ranger();
    Print_Data();
}


//----------------------------------------------------------------------------
//Print_Data
//----------------------------------------------------------------------------
void Print_Data(void)
{
    if(print_count > 20)
		//Only prints ever ~400 ms
    {
		time += print_count/5;	//Ensures accurate time readings
        print_count = 0;
        printf("\r\n%u,%d,%u,%u", (int)time, heading_error, Servo_PW, range);
        lcd_clear();
        lcd_print("Heading is: %u\nRange is: %u\nServo Cycle: %u\nMotor Cycle: %u", current_heading, range, (int)(((float)Servo_PW/28672)*100), (int)(((float)Motor_PW/28672)*100));
    }
}

//LOW LEVEL FUNCTIONS

//----------------------------------------------------------------------------
//Set_Servo_PWM
//----------------------------------------------------------------------------
void Set_Servo_PWM(void)
{
	//Servo_PW set to value based on heading_error modified by gain set in Car_Parameters()
	Servo_PW = gain*(heading_error) + SERVO_CENTER_PW;

    //Additional precaution: if Servo_PW somehow exceeds the limits set in Lab 3-1,
    //then Servo_PW is set to corresponding endpoint of PW range [SERVO_LEFT_PW, SERVO_RIGHT_PW]
	if (Servo_PW > SERVO_RIGHT_PW) Servo_PW = SERVO_RIGHT_PW;
	if (Servo_PW < SERVO_LEFT_PW) Servo_PW = SERVO_LEFT_PW;
	PCA0CP0 = 0xFFFF - Servo_PW;
}

//----------------------------------------------------------------------------
//Set_Motor_PWM
//----------------------------------------------------------------------------
void Set_Motor_PWM(void)
{
	//Motor_PW = MOTOR_NEUTRAL_PW + kdy * y; // kdy is the y-axis drive feedback gain
	//Add correction for side-to-side tilt, forcing a forward movement to turn the car.
	//Motor_PW += kdx * abs(x); //kdx is the x-axis drive feedback gain
}

//----------------------------------------------------------------------------
//Pause
//----------------------------------------------------------------------------
void Pause(void)
{   /*
	//Stop for 40 ms
	r_count = 0;
	while (r_count < 2){}
    */
}

//----------------------------------------------------------------------------
//Wait
//----------------------------------------------------------------------------
void Wait(void)
{   /*
	//Stop for 1000 ms
	r_count = 0;
	while (r_count < 50){}
    */
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
	unsigned char keypad;
	unsigned char keyboard;
	unsigned char isPress = 0;
	unsigned char pressCheck = 0;
	unsigned int value = 0;	//Final value to be returned
	
	while(1)
	{
		keyboard = getchar_nw();	//This constantly sets keyboard to whatever char is in the terminal
		keypad = read_keypad();		//This constantly sets the keypad to whatever char is on the LCD
		Pause();					//Pause necessary to prevent overreading the keypad

		if (keyboard == '#' || keypad == '#') //# is a confirm key, so it will finish calibrate()
			return value;	

		if (isPress > pressCheck && keypad == 0xFF && keyboard == 0xFF)	//Only increments pressCheck if held key is released
			pressCheck++;
		

		if (pressCheck == 6)	//If a 6th key is pressed, then released
		{
			isPress = pressCheck = 0;	//Reset the flags
			value = 0;	//Reset return value
			lcd_print("\b\b\b\b\b\b");	//Clear value displayed on LCD, needs an extra \b for some reason?
			printf("\r      \r");	//Clear value displayed on terminal
			
		}

		
		if (isPress == pressCheck)	//pressCheck must be equal to isPress, only occurs if no key is held down
		{
			if (keypad != 0xFF)		//When an actual key is held down
			{
				lcd_print("%c",keypad);	//Adds pressed key to LCD screen
				printf("%c", keypad);	//Adds pressed key to computer terminal
				value = value + ((unsigned int)(keypad - '0')) * pow(10,4 - isPress);	//Essentially takes each pressed key and multiples by some power of 10
				isPress++;
			}
			if (keyboard != 0xFF)	//When an actual key is held down
			{
				lcd_print("%c",keyboard);	//Adds pressed key to LCD screen
				//printf("%c", keyboard); this line is not necessary as getchar_nw automatically executes a putchar()
				value = value + ((unsigned int)(keyboard - '0')) * pow(10,4 - isPress);	//Essentially takes each pressed key and multiples by some power of 10
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
    print_count = 0;
    while(print_count < 6);

    //Sets gain to 1
    ADC1CF |= 0x01;
    ADC1CF &= 0xFD;
}

//-----------------------------------------------------------------------------
//Port_Init
//-----------------------------------------------------------------------------
void Port_Init()
{   /*
	//Initailize POT
	P1MDOUT |= 0x05;	//Set output pin for CEX0 and CEX2 in push-pull mode
	P1MDOUT &= ~0x80;	//Set potentiometer pin (P1.7) to open drain
	P1 |= 0x80;			//Set impedance high on P1.7
	P1MDIN &= ~0x80;	//Set P1.7 to analog input
	
	P3MDOUT &= ~0x80; //Pin 3.7 open drain
	P3 |= 0x80; //Pin 3.7 high impedance
    */
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
        if (accel_count >= 1)   //Accelerometer won't read unless 40 ms has passed
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
	int i=0; //counter variable
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
			yaccel +=Data[4]<<8 | Data[3]>>4; //set and total y values
		}
	}
	xaccel = xaccel>>3-xoffset; //average by dividing by 8 and subtract offset
	yaccel = yaccel>>3-yoffset; //average by dividing by 8 and subtract offset
}
//-----------------------------------------------------------------------------
//
// Calibrate Accelerometer
//
void Calibrate_Accel()
{
	int i=0; //counter variable
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
			yoffset +=Data[4]<<8 | Data[3]>>4; //set and total y values
		}
	}
	xoffset = xoffset>>4; //average by dividing by 64
	yoffset = yoffset>>4; //average by dividing by 64
}
//-----------------------------------------------------------------------------


