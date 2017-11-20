# Tasks

## Matt
- [x] Refactor PCA_ISR logic for reading flags
- [x] Refactor Car_Parameters into Starting_Parameters
- [x] Recheck ADC function to ensure it is working properly
- [x] HW 11
- [x] Refactor Pause & Wait functions to not reset count variables
- [ ] Clean existing wiring 
- [ ] Beginning implementation of main logic
- [ ] Create a function to handle buzzer requirement

## Sydney
- [x] Refactor Print_Data
- [x] Write Read_Accel
- [x] Write Accel_Calibrate
- [x] HW 11
- [ ] Modify Set_Servo_PWM
- [ ] Wire accelerometer, buzzer, and additional slide switch.
- [ ] Determine if global keypad and keyboard are necessary

## Tom
- [x] Determine main loop logic
- [x] Change flags to use __bit type
- [x] Modified Set_Neutral
- [x] HW 11
- [ ] Modify Set_Motor_PWM
- [ ] Pseudocode
- [ ] Test & Correct Read_Accel

# Lab_5
LITEC Lab 5

## Project Requirements

### Hardware
1. Wire the accelerometer to the I2C bus (x arrow pointing to EVB ribbon); keep the existing ranger, compass, and keypad. Keep wires for RF transmission.
2. Reinstall a buzzer in series with a resistor and run it through the buffer chip.
3. Potentially add battery monitoring capabilities (OPTIONAL).
4. An additional slide switch; the two slide switches will each individually control run/stop of the two control algorithms (Drive and Servo).

### Software
1. Call the Accel_Init_C function from i2c.h file and write a Read_Accel function.
2. Need to modify Set_Servo_PWM that is based off the accelerometer reading.
3. Steering should be set such that the car will turn in the direction of the upward slope.
4. Motor_PW is set using both side-to-side (+x to the left) and front-to-back (+y to the front) tilt.
5. The accelerometer is noisy, so 4 to 8 readings of the accelerometer should be averaged.
6. Initial parameters and output should still be displayed on both the LCD display and the SecureCRT terminal.
7. Three different gains will be set by the user; each should be able to be set with both the keypad and keyboard.
8. Potentially add battery monitoring software (OPTIONAL).
9. Data should be printed as one combined printf() including time (possibly unnecessary?), x & y accelerations, Motor_PW and Servo_PW, current gains, and possibly battery voltage (OPTIONAL) IN THAT ORDER.
10. Switching the drive motor to neutral should operate in such a way that "adjustments can be made with the program running" (ask staff).
11. For the drive motor, front-to-back gain is set via pot and varies from 1 to 50 (pure integer is fine) which can be adjusted while the car is in motion. Side-to-side gain is set by key press on either the keypad or keyboard.
12. It should not be difficult to incorporate software for starting parameters that prints the gain from the pot and asks if you wish to try a different gain (Matt's recommendation).
13. Steering gain should be set by key press from keypad or keyboard.
14. The main logic for setting Servo_PW and Motor_PW is listed below:
```C
Servo_PW = SERVO_CENTER_PW - ks * xaccel
Motor_PW = MOTOR_NEUTRAL_PW + kdy * yaccel
Motor_PW += kdx * abs(xaccel)
```
Where ks is the Servo gain, and kdy and kdx are the drive motor gains.

For an additional integral function for the Motor_PW:
```C
drive_pw += kdx * abs(gx) + ki * error_sum
error_sum += gy + abs(gx)
```
Where ki is the integral gain.

### Car Behavior
1. Car will drive in the direction of maximum slope until it levels out at the top or bottom of the slope.
2. The car must drive up the ramp in reverse! While driving in reverse, it must turn the buzzer on and off for .5 s and 1.0 s, respectively. When it reaches the top, the buzzer should turn off. (**Ask staff if it needs to buzz while the motor or servo is in neutral.**)
3. Car strictly stops when it reaches level ground, and points and is parallel to the direction of maximum slope when it stops.
4. While driving, the program should find the maximum slope of the ramp and display this value in degrees when it stops at level ground.


## Other Considerations
1. The accelerometer updates every 20 ms. Avoid duplicate readings!
2. Keypad should not be queried for a read faster than every few ms.
3. It is necessary to try several gain values for optimal performance. Noise with the accelerometer is unpleasant and may cause jerky steering.
4. Be sure to maintain cases that prevent PW's from going outside their ranges.
5. I2C address of the accelerometer is 0x3A (ACCEL_ADDR). Point registers 0x28 and 0x29 contain the 12-bit x-axis acceleration and registers 0x2A and 0x2B the y-axis acceleration after 2 LS bits of 0x27 go high. Low byte is in lower register number. Discard low byte because of noise. Sign-extend the high byte for a 16-bit signed int; equivalent 12-bit value is in highest bits. Shift into 12 lowest bits.
6. The drive motor displays asymmetrical strength in terms of forward vs. backward travel; it may be wise to have separate gains for both.

```C
drive_pw += kdx * abs(gx) + ki * error_sum //ki is the integral gain
error_sum += gy + abs(gx)
```
Above is one solution to this issue that uses an integral term which will increase drive motor PW over time until it is high enough to actually move the car.

7. Accelerometer must be calculated for a flat zero point every time you run your program! It is suggested that 64 readings of both x and y acceleration be averaged and these averages used as offsets.
8. Ask about the Update_Value function's purpose on the Lab 5 pdf. Is it something that would be called while the car is actually driving up the slope?
9. The sample code of Lab 5 is encouraging declaring and setting of flags for reading the accelerometer and keypad in the PCA_ISR.
10. See last two pages of the pdf for sample code.