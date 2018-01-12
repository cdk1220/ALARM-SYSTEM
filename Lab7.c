/*******************************
 * Name: Don Kuruppu
 * Student ID#: 1001 101 220
 * Lab Day:
 * CSE 3442/5442 - Embedded Systems 1
 * Lab 7 (ABET): Building a PIC18F4520 Standalone Alarm System with EUSART Communication 
 ********************************/

#include <pic18F4520.h>
#include <xc.h>
#include <stdio.h>
#include <string.h>

#define _XTAL_FREQ 20000000

/******************************************
 * ROM addresses and sizes for system data
 *****************************************/
#define password_address 0
#define password_size 4

#define method_of_input_status_address 4
#define method_of_input_status_size 1

#define temperature_sensor_status_address 5
#define temperature_sensor_status_size 1

#define threshold_temperature_address 6
#define threshold_temperature_size 2

#define motion_sensor_status_address 8
#define motion_sensor_status_size 1

#define soft_reset_address 9
#define soft_reset_size 1

#define max_buffer_size 4


// CONFIG1H
#pragma config OSC = HS         // Oscillator Selection bits (HS oscillator)

// Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (RE3 input pin enabled; MCLR disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)


/**********************************
 * Operational and State Variables
 **********************************/
unsigned char password[password_size];

unsigned char methodOfInput;                         //This is 0 if the input method is the keyboard, 1 if it is the matrix pad, and 2 if it is both
unsigned char temperatureSensor;                     //This is 0 if the system doesn't have a previous or the system starting for the first time and becomes 1 when activated
double currentTemp;                                  //Holds the temperature readings from ADC
int tempThreshold;                                   //Temperature sensor interrupt trigger value set by user
unsigned char pirSensor;                             //This is 0 if the system doesn't have a previous or the system starting for the first time and becomes 1 when activated
unsigned char softReset;

/*******************************
 * Function prototypes
 *******************************/
void initTRISSettings();
void initUSARTSettings();
void initADCSettings();
void initLEDs();
void initTimer();
void interruptSettingsAndTurningOnTheComponents();

void keyboardRead(int numberOfDigitsExpecting, unsigned char show, unsigned char * userInputBuffer);
void matrixPadRead(int numberOfUserInputs, int aPressed, unsigned char show, unsigned char * userInputBuffer);
void showKeyPress(unsigned char character, unsigned char aPassword);
void enterTheCurrentPassword(unsigned char * userInputBuffer);
void enterChoice(int sizeOfChoice, int show, unsigned char * userInputBuffer);

void putch(unsigned char data);
void eepromRead(char numberOfBytesToRead, char startingAddress, unsigned char * eepromBuffer);
void eepromWrite(char numberOfBytesToWrite, char startingAddress, unsigned char * eepromBuffer);

void interrupt My_ISR_High(void);
void interrupt low_priority My_ISR_Low(void);
double celciusToFarenheit(double result);

void systemStartUp();
void assigningAState(char unsigned * userInputBuffer, char unsigned * eepromBuffer);

void showHeaderAndStateOfTheAlarm();
void showMenuAndGetUserInput(unsigned char * userInputBuffer);
void identifyingTheKeyPressed(unsigned char * userInputBuffer, unsigned char * eepromBuffer);

void userPressedOne(unsigned char * userInputBuffer, unsigned char * eepromBuffer);
void userPressedTwo(unsigned char * userInputBuffer, unsigned char * eepromBuffer);
void userPressedThree(unsigned char * userInputBuffer, unsigned char * eepromBuffer);
void userPressedFour(unsigned char * userInputBuffer, unsigned char * eepromBuffer);

void main(void) {
    unsigned char userInputBuffer[max_buffer_size];
    unsigned char eepromBuffer[max_buffer_size];
    
    systemStartUp();
    assigningAState(userInputBuffer, eepromBuffer);
    
    while(1){
        showHeaderAndStateOfTheAlarm();
        showMenuAndGetUserInput(userInputBuffer);
        identifyingTheKeyPressed(userInputBuffer, eepromBuffer);
    }
}


/****************************************************************************
 * initTRISSettings
 * 
 * This function does the TRIS settings for all the input and output pins of
 * the PIC being used. This is called when the alarm starts.
 ****************************************************************************/
void initTRISSettings(){
    //Serial Communication
    TRISCbits.RC6 = 0;         //Transmit pin should be an output
    TRISCbits.RC7 = 1;         //Receive pin should be an input
    
    //Temperature Sensor (ADC)
    TRISAbits.RA0 = 1;         //RA0 which is AN0 should be an input 
    
    //Motion sensor
    TRISBbits.RB0 = 1;         //RB0 should be an input to read from the motion sensor
    
    //LEDs
    TRISDbits.RD1 = 0;         //Output pin for the green LED
    TRISDbits.RD0 = 0;         //Output pin for the yellow LED
    TRISCbits.RC3 = 0;         //Output pin for the red LED
    TRISCbits.RC2 = 0;         //Output pin for the blue LED
    
    //Matrix keypad outputs
    TRISDbits.RD4 = 0;         //Output 0 from PIC
    TRISDbits.RD5 = 0;         //Output 1 from PIC
    TRISDbits.RD6 = 0;         //Output 2 from PIC
    TRISDbits.RD7 = 0;         //Output 3 from PIC
    
    //Matrix keypad inputs
    TRISBbits.RB4 = 1;         //Input 4 to PIC
    TRISBbits.RB5 = 1;         //Input 5 to PIC
    TRISBbits.RB6 = 1;         //Input 6 to PIC
    TRISBbits.RB7 = 1;         //Input 7 to PIC
}


/********************************************************************************
 * initUSARTSettings
 * 
 * This function sets up the USART communication between the PIC and the terminal.
 * This is called when the alarm starts.
 ********************************************************************************/
void initUSARTSettings(){
    SPBRG = 129;               //High speed 9600 BAUD rate
   
    //Transmission settings
    TXSTAbits.TX9 = 0;         //8-bit transmission
    TXSTAbits.SYNC = 0;        //Choosing asynchronous mode
    TXSTAbits.BRGH = 1;        //Choosing high speed
    TXSTAbits.TXEN = 1;        //Enabling transmission
    
    //Reception settings
    RCSTAbits.RX9 = 0;         //8-bit reception
    RCSTAbits.CREN = 0;        //Receiver disabled initially since the method of input is unknown
    
    RCSTAbits.SPEN = 1;        //Serial Port enabled
}


/********************************************************************************
 * initADCSettings
 * 
 * This functions sets up the ADC module of the PIC to be used by the temperature
 * sensor. This is called when the alarm starts.
 ********************************************************************************/
void initADCSettings(){
    //When the system starts up, don't know if it had a previous state of not having temperature sensor interrupt
    ADCON0bits.ADON = 0;
    
    //Analog channel bits for AN0
    ADCON0bits.CHS3 = 0;
    ADCON0bits.CHS2 = 0;
    ADCON0bits.CHS1 = 0;
    ADCON0bits.CHS0 = 0;
    
    ADCON1bits.VCFG1 = 0;       //Vref positive is Vdd
    ADCON1bits.VCFG0 = 0;       //Vref negative is Vss
    
    //AD port configuration bits for AN0 making only RA0 analog
    ADCON1bits.PCFG3 = 1;
    ADCON1bits.PCFG2 = 1;   
    ADCON1bits.PCFG1 = 1;
    ADCON1bits.PCFG0 = 0;
    
    ADCON2bits.ADFM = 1;        //Right justified
    
    //Acquisition time select bits making Tacq 4 Tads
    ADCON2bits.ACQT2 = 0;
    ADCON2bits.ACQT1 = 1;
    ADCON2bits.ACQT0 = 0;
    
    //Conversion clock select bits for FOSC/16
    ADCON2bits.ADCS2 = 1;
    ADCON2bits.ADCS1 = 0;
    ADCON2bits.ADCS0 = 1;
}


/************************************************************************
 * initLEDs
 * 
 * This function sets up the state of the LEDs for when the alarm starts.
 ************************************************************************/
void initLEDs(){
    //Only the green LED should be on indicating the system power is on
    PORTDbits.RD1 = 1;         
    PORTDbits.RD0 = 0;         
    PORTCbits.RC3 = 0;         
    PORTCbits.RC2 = 0;         
}


/*******************************************************************************
 * initTimer
 * 
 * This function sets up timer 0 to implement the sample delay for the ADC. This
 * is called when the alarm starts.
 *******************************************************************************/
void initTimer(){
    //Initial configuration
    T0CONbits.TMR0ON = 0; //Timer is off
    T0CONbits.T08BIT = 0; //Timer is 16bit
    T0CONbits.T0CS = 0;   //Internal instruction clock cycle
    T0CONbits.T0SE = 0;   //Increment on low to high transition
    T0CONbits.PSA = 0;    //A pre-scaler is assigned
    
    //Choosing 1 : 256 pre-scaler
    T0CONbits.T0PS2 = 1;
    T0CONbits.T0PS1 = 1;
    T0CONbits.T0PS0 = 1;
    
    //Pre-loading the registers and making sure the high byte is loaded first
    TMR0H = 0x67;
    TMR0L = 0x69; 
}


/*********************************************************************************
 * interruptSettingsAndTurningOnTheComponents
 * 
 * This function handles the interrupts settings of the ADC, timer 0, and INT0.
 * It also checks for the previous state of the alarm and decides which interrupts
 * should be enabled and what LEDs should be on.
 *********************************************************************************/
void interruptSettingsAndTurningOnTheComponents(){
    //Main interrupt settings
    RCONbits.IPEN = 1;                 //Enabling priorities on interrupts
    INTCONbits.PEIE = 1;               //Enabling peripheral interrupts
    INTCONbits.GIE = 1;                //Enabling global interrupts
    
    //Temperature Sensor (ADC & Timer 0)
    IPR1bits.ADIP = 0;                 //ADC is low priority
    PIR1bits.ADIF = 0;                 //Clearing ADC flag
    INTCONbits.TMR0IF = 0;             //Clearing the timer 0 flag
    INTCON2bits.TMR0IP = 0;            //Timer 0 is low priority
    
    //From previous state temperature sensor has been on
    if(temperatureSensor){
        PIE1bits.ADIE = 1;             //Enabling ADC interrupt
        PORTDbits.RD0 = 1;             //Lighting up yellow LED to show that temperature sensor is on
        ADCON0bits.ADON = 1;           //Turning on ADC module
        
        PORTDbits.RD0 = 1;             //yellow LED on as ADC conversion begins
        ADCON0bits.GO = 1;             //Starting the ADC conversion
        
        
        INTCONbits.TMR0IE = 1;         //Enabling timer 0 interrupt
        T0CONbits.TMR0ON = 1;          //Timer is on
    }
    
   
    //Motion Sensor (INT0)
    INTCON2bits.INTEDG0 = 0;           //PIR sensor is falling edge
    INTCONbits.INT0IF = 0;              //Clearing INT0 flag
    
    //From previous state motion sensor has been on
    if(pirSensor){
        INTCONbits.INT0IE = 1;          //Enabling INT0
    }
   
    //From previous state matrix pad has been the input method
    if(methodOfInput){
        PORTCbits.RC2 = 1;             //Lighting up blue LED to show that matrix pad is active
    }
}


/*******************************************************************************************
 * keyboardRead
 * 
 * This function handles the keyboard input from the user. It accepts input from 
 * all the keys and then filters out the keys that the alarm supports and shows them
 * on the terminal. Also waits for the user to press "enter" at the end of each input.
 * 
 * Input Arguments
 * int numberOfDigitsExpecting      - This function accepts only this many characters from
 *                                    the user.
 * unsigned char show               - Decides whether to show the actual characters or 
 *                                    asterisks when the user types (Passed down to 
 *                                    showKeyPressFunction).
 * unsigned char * userInputBuffer  - Address to where the user input gets stored
 *******************************************************************************************/
void keyboardRead(int numberOfDigitsExpecting, unsigned char show, unsigned char * userInputBuffer){
    int i;
    int aPassword = show;          //Indicates if it is a password or a regular input being typed
    
    RCSTAbits.CREN = 1;            //Enabling USART receive to get input from keyboard
       
    for(i = 0; i < numberOfDigitsExpecting; i++){
        while(PIR1bits.RCIF == 0);

        //If the entered digit is between 0 & 9, consider that as a valid input
        if((RCREG <= 57) && (RCREG >= 48)){
            userInputBuffer[i] = RCREG;
            showKeyPress(userInputBuffer[i], aPassword);
        }

        //Whenever the user enters an off-range digit, disregard it
        else{
            i--;                   //Don't increment the character counter
        }
    }
        
    //Entered supported digits, so break off 
    if(i == numberOfDigitsExpecting){
        //Wait for an enter key press by the user
        while(1){
            while(PIR1bits.RCIF == 0);
            if(RCREG == 13)
                break;
        }
    }
    
    RCSTAbits.CREN = 0;            //Disabling USART receive
}


/*********************************************************************************************
 * matrixPadRead
 * 
 * This function handles the matrix pad input from the user. It accepts input from 
 * all the keys and then filters out the keys that the alarm supports and shows them
 * on the terminal. This function is called to obtain user confirmation as well.
 * 
 * Input Arguments
 * int numberOfUserInputs           - This function accepts only this many characters from
 *                                    the user.
 * int aPressed                     - Decides whether the function is used to grab user input
 *                                    get user confirmation about something. 
 * unsigned char show               - Decides whether to show the actual characters or 
 *                                    asterisks when the user types (Passed down to 
 *                                    showKeyPressFunction).
 * unsigned char * userInputBuffer  - Address to where the user input gets stored
 *********************************************************************************************/
void matrixPadRead(int numberOfUserInputs, int aPressed, unsigned char show, unsigned char * userInputBuffer){
    int i;
    int aPassword = show;
    //Driving all outputs of the matrix pad from the PIC low to make sure not more than one output is high during individual row checks
    PORTDbits.RD4 = 0;
    PORTDbits.RD5 = 0;
    PORTDbits.RD6 = 0;
    PORTDbits.RD7 = 0;
    
    //This is to get the actual user input 
    for(i = 0; i < numberOfUserInputs; i++){
        while(1){
            //Checking the first row on the matrix pad
            PORTDbits.RD4 = 1;
            //Checking if the first button was pressed
            if((PORTBbits.RB4) && (!PORTBbits.RB5) && (!PORTBbits.RB6) && (!PORTBbits.RB7)){
                userInputBuffer[i] = '1';
                showKeyPress(userInputBuffer[i], aPassword);
                break; 
            }
            //Checking if the second button was pressed
            else if((!PORTBbits.RB4) && (PORTBbits.RB5) && (!PORTBbits.RB6) && (!PORTBbits.RB7)){
                userInputBuffer[i] = '2';
                showKeyPress(userInputBuffer[i], aPassword);
                break;
            }
            //Checking if the second button was pressed
            else if((!PORTBbits.RB4) && (!PORTBbits.RB5) && (PORTBbits.RB6) && (!PORTBbits.RB7)){
                userInputBuffer[i] = '3';
                showKeyPress(userInputBuffer[i], aPassword);
                break;
            }
            //Checking if the third button was pressed
            else if((!PORTBbits.RB4) && (!PORTBbits.RB5) && (!PORTBbits.RB6) && (PORTBbits.RB7)){
                userInputBuffer[i] = 'A';
                showKeyPress(userInputBuffer[i], aPassword);
                break;
            }
            PORTDbits.RD4 = 0;


            //Checking the second row of the matrix pad
            PORTDbits.RD5 = 1;
            //Checking if the first button was pressed
            if((PORTBbits.RB4) && (!PORTBbits.RB5) && (!PORTBbits.RB6) && (!PORTBbits.RB7)){            
                userInputBuffer[i] = '4';
                showKeyPress(userInputBuffer[i], aPassword);
                break;
            }
            //Checking if the second button was pressed
            else if((!PORTBbits.RB4) && (PORTBbits.RB5) && (!PORTBbits.RB6) && (!PORTBbits.RB7)){            
                userInputBuffer[i] = '5';
                showKeyPress(userInputBuffer[i], aPassword);
                break;
            }
            //Checking if the third button was pressed
            else if((!PORTBbits.RB4) && (!PORTBbits.RB5) && (PORTBbits.RB6) && (!PORTBbits.RB7)){           
                userInputBuffer[i] = '6';
                showKeyPress(userInputBuffer[i], aPassword);
                break;
            }
            //Checking if the fourth button was pressed
            else if((!PORTBbits.RB4) && (!PORTBbits.RB5) && (!PORTBbits.RB6) && (PORTBbits.RB7)){ 
                i--;    //Doesn't support the key. Decrement the counter.
                break;
            }
            PORTDbits.RD5 = 0;


            //Checking the third row of the matrix pad
            PORTDbits.RD6 = 1;
            //Checking if the first button was pressed
            if((PORTBbits.RB4) && (!PORTBbits.RB5) && (!PORTBbits.RB6) && (!PORTBbits.RB7)){            
                userInputBuffer[i] = '7';
                showKeyPress(userInputBuffer[i], aPassword);
                break;
            }
            //Checking if the second button was pressed
            else if((!PORTBbits.RB4) && (PORTBbits.RB5) && (!PORTBbits.RB6) && (!PORTBbits.RB7)){           
                userInputBuffer[i] = '8';
                showKeyPress(userInputBuffer[i], aPassword);
                break;
            }
            //Checking if the third button was pressed
            else if((!PORTBbits.RB4) && (!PORTBbits.RB5) && (PORTBbits.RB6) && (!PORTBbits.RB7)){
                userInputBuffer[i] = '9';
                showKeyPress(userInputBuffer[i], aPassword);
                break;
            }
            //Checking if the fourth button was pressed
            else if((!PORTBbits.RB4) && (!PORTBbits.RB5) && (!PORTBbits.RB6) && (PORTBbits.RB7)){
                i--;    //Doesn't support the key. Decrement the counter.
                break;
            }
            PORTDbits.RD6 = 0;


            //Checking the fourth row of the matrix pad
            PORTDbits.RD7 = 1;
            //Checking if the first button was pressed
            if((PORTBbits.RB4) && (!PORTBbits.RB5) && (!PORTBbits.RB6) && (!PORTBbits.RB7)){
                i--;    //Doesn't support the key. Decrement the counter.
                break;
            }
            //Checking if the second button was pressed
            else if((!PORTBbits.RB4) && (PORTBbits.RB5) && (!PORTBbits.RB6) && (!PORTBbits.RB7)){
                userInputBuffer[i] = '0';
                showKeyPress(userInputBuffer[i], aPassword);
                break;
            }
            //Checking if the third button was pressed
            else if((!PORTBbits.RB4) && (!PORTBbits.RB5) && (PORTBbits.RB6) && (!PORTBbits.RB7)){
                i--;    //Doesn't support the key. Decrement the counter.
                break;
            }
            //Checking if the fourth button was pressed
            else if((!PORTBbits.RB4) && (!PORTBbits.RB5) && (!PORTBbits.RB6) && (PORTBbits.RB7)){
                i--;    //Doesn't support the key. Decrement the counter.
                break;
            }
            PORTDbits.RD7 = 0;
        }
        __delay_ms(250); //Capturing only one instance of the pressed key
    }
    
    //'A' is assumed as the enter key and the lines below keep checking if the user pressed 'A'
    if(aPressed){
        while(1){
            PORTDbits.RD4 = 1;
            //Checking if 'A'
            if((!PORTBbits.RB4) && (!PORTBbits.RB5) && (!PORTBbits.RB6) && (PORTBbits.RB7)){
                break;
            }
            PORTDbits.RD4 = 0;
        }
        __delay_ms(250); //Capturing only one instance of the pressed key
    }
}


/****************************************************************************************
 * showKeyPress
 * 
 * This function decides what to show on the terminal when the alarm gets user input. It
 * is called in keyboardRead and matrixPadRead.
 * 
 * Input Arguments
 * unsigned char character  - The character that has to be printed on the terminal
 * unsigned char aPassword  - Decides whether to show the character itself or an asterisk  
 ****************************************************************************************/
void showKeyPress(unsigned char character, unsigned char aPassword){
    //If it is a password, don't show the character
    if(!aPassword){
        printf("*");
    }
    else{
        printf("%c", character);
    }
}


/**********************************************************************************
 * enterTheCurrentPassword
 * 
 * This function is called whenever the user has to input the password to clear the
 * motion sensor or the temperature sensor when they get triggered. This is also 
 * used to get the password from the user whenever the alarm is restarted. This 
 * function accepts the password from the current form of input and compares it to 
 * the current password and prompts the user the password till the user gets it
 * right.
 * 
 * Input Arguments
 * unsigned char * userInputBuffer  - Address to where the user input gets stored
 **********************************************************************************/
void enterTheCurrentPassword(unsigned char * userInputBuffer){
    //User gets unlimited number of times to enter the old password and get it right
    while(1){
        //Matrix is the current method of input
        if(methodOfInput == 1){
            //Waiting for the password input
            matrixPadRead(password_size, 0, 0, userInputBuffer);
            //Waiting for the user to confirm choice
            matrixPadRead(0, 1, 0, userInputBuffer);               
        }
        //Keyboard is the current method of input
        else{
            keyboardRead(password_size, 0, userInputBuffer);
        }
        
        //If the entered password matches the current password break off the loop
        if(!strncmp(userInputBuffer, password, 4)){
            break;
        }
        else{
            printf("\n\nPassword incorrect. Enter the password again: ");
        }
    }
}


/**********************************************************************************
 * enterChoice
 * 
 * This function is called whenever the user has to select an option from the alarm
 * menu or when the user has to decide to turn off or leave something on.
 * 
 * Input Arguments
 * int sizeOfChoice                 - Size of the suer input
 * int show                         - Usually 1 for this function indicating user 
 *                                    input is not a password and that actual 
 *                                    characters have to be printed on the terminal
 * unsigned char * userInputBuffer  - Address to where the user input gets stored
 **********************************************************************************/
void enterChoice(int sizeOfChoice, int show, unsigned char * userInputBuffer){
    //Have to take input from the method of input that is active
    if(methodOfInput == 1){
        //Waiting for user input of one key stroke and the key stroke will be displayed on the terminal
        matrixPadRead(sizeOfChoice, 0, show, userInputBuffer);
        //Waiting for the user to confirm choice
        matrixPadRead(0, 1, 0, userInputBuffer);
    }
    else{
        //Waiting for user input of one key stroke and then for the confirmation. The key stroke will be displayed on the terminal
        keyboardRead(sizeOfChoice, show, userInputBuffer);
    }
}


/************************************************************************************
 * putch
 * 
 * This function is used by printf enabling the usage of printf to send whole strings
 * instead of having to send a character at a time to be printed on the terminal.
 ************************************************************************************/
void putch(unsigned char data){
    while(!PIR1bits.TXIF)      //Wait until the transmitter is ready
        continue;
    TXREG = data;              //Send one character
}


/***************************************************************************************
 * eepromRead
 * 
 * This function handles reading from the EERPOM. Used to read data of previous state of 
 * the alarm if there is any.
 * 
 * Input Arguments
 * char numberOfBytesToRead     -   How many bytes to read from the EEPROM
 * char startingAddress         -   Starting place where to read from
 * unsigned char * eepromBuffer -   Address of the buffer where the reading gets 
 *                                  stored
 ***************************************************************************************/
void eepromRead(char numberOfBytesToRead, char startingAddress, unsigned char * eepromBuffer){
    int i;
    EECON1bits.EEPGD = 0; 
    EECON1bits.CFGS = 0;
    
    for(i = 0; i < numberOfBytesToRead; i++){
        EEADR = startingAddress + i; 
        EECON1bits.RD = 1; 
        while(EECON1bits.RD); 
        eepromBuffer[i] = EEDATA;
    }
}


/***************************************************************************************
 * eepromWrite
 * 
 * This function handles writing ti the EERPOM. Used to write updates about the state 
 * variables of the alarm modules.
 * 
 * Input Arguments
 * char numberOfBytesToWrite    -   How many bytes to read from the EEPROM
 * char startingAddress         -   Starting place where to write ti
 * unsigned char * eepromBuffer -   Address of the buffer where what to write is stored
 ***************************************************************************************/
void eepromWrite(char numberOfBytesToWrite, char startingAddress, unsigned char * eepromBuffer){
    int i;
   
    EECON1bits.EEPGD = 0; 
    EECON1bits.CFGS = 0; 
    INTCONbits.GIE = 0; 
    EECON1bits.WREN = 1;
    
    for(i = 0; i < numberOfBytesToWrite; i++){
        EEADR = startingAddress + i; 
        EEDATA = eepromBuffer[i]; 
        EECON2 = 0x55; 
        EECON2 = 0xAA; 
        EECON1bits.WR = 1; 
        while(EECON1bits.WR); 
        PIR2bits.EEIF = 0;  
    }
    
    EECON1bits.WREN = 0; 
    INTCONbits.GIE= 1;
}


/***************************************************************************
 * My_ISR_High
 * 
 * Handles the interrupts from the PIR motion sensor. Also, does a software
 * reset at the end to avoid any unexpected behavior of the system.
 ***************************************************************************/
void interrupt My_ISR_High(void){  
    //Motion sensor (INT0) interrupt handle if flag is tripped and INT0 interrupt was on
    if((INTCONbits.INT0IE == 1) && (INTCONbits.INT0IF == 1)){
        unsigned char temp[4];
        
        PORTCbits.RC3 = 1;         //Turning red LED on
        printf("\n\n\n*************************************************************************************************************************\n\n");
        printf("********************************************     MOTION DETECTED!!!!!!!!     ********************************************\n\n");    
        printf("*************************************************************************************************************************\n\n");
       
        //User has to input the password to clear the alarm
        printf("\n\nEnter the password: ");
        
        //Expecting password to clear the alarm
        enterTheCurrentPassword(temp);
       
        PORTCbits.RC3 = 0;         //Turning red LED off 
        
        printf("\n\n\nI.   Press one to leave the motion sensor on\n\n");
        printf("II.  Press two to turn off the motion sensor\n\n");
        
        //Expecting the user choice
        enterChoice(1, 1, temp);
        
        //If user pressed one, leave the alarm on
        if(temp[0] == '1'){
            printf("\n\nMotion sensor is left on\n\n\n");
            
 
        }
        //User pressed 2, turn it off
        else if(temp[0] == '2'){ 
            pirSensor = 0;         //Assigning off to the motion sensor
            printf("\n\nMotion sensor is off now\n\n\n");
            INTCONbits.INT0IE = 0; //Disabling interrupts for the motion sensor
            
            //Writing the change of state to EEPROM
            temp[0] = pirSensor;
            eepromWrite(motion_sensor_status_size, motion_sensor_status_address, temp);
        }
        else{
            printf("\n\nEither the entered number was not a choice. Motion sensor was left as is\n\n");
        }
        
        INTCONbits.INT0IF = 0;     //Clearing the flag for the interrupt
        
        //Do a software reset and not go back to main as going back to main makes program behave weird
        softReset = 1;
        temp[0] = softReset;
        eepromWrite(soft_reset_size, soft_reset_address, temp);
        Reset();
    }
    
}


/***************************************************************************************
 * My_ISR_Low
 * 
 * Handles the interrupts from the temperature sensor and timer 0. Also, does a software
 * reset once it prints the temperature sensor trigger to avoid any unexpected behavior 
 * of the system.
 ***************************************************************************************/
void interrupt low_priority My_ISR_Low(void){
    //ADC interrupt handle if flag is tripped and ADC interrupt was on
    if((PIR1bits.ADIF == 1) && (PIE1bits.ADIE == 1)){
        unsigned char temp[4];
        int adLow;
        int adHigh;
        
        //Acquiring the ADC result from ADC registers
        adLow = ADRESL;
        adHigh = ADRESH; 
         
        currentTemp = adHigh << 8;                        //Making the 2 bits of adHigh the 2 most significant bits of the 10 bit ten bit value
        
        currentTemp = currentTemp +  adLow;               //Populating the 8 least significant bits of the 10 bit value
        currentTemp = (currentTemp * 5) / 1023.0;         //Calculating the voltage value 
        
        
        currentTemp = currentTemp - 0.5;                  //Calculating the output voltage difference between 0 Celcius and current temperature
        currentTemp = currentTemp * 100;                  //Calculating how many 0.1mV are there in the voltage difference to get the temperature difference
        
        currentTemp = celciusToFarenheit(currentTemp);    //Converting the temperature to Farenheit from Celcius 
   
        //Let the user know that temperature has exceeded threshold
        if(currentTemp >= (double)tempThreshold){
            PORTDbits.RD0 = 1;                  //Turning on the yellow LED
            printf("\n\n\n*************************************************************************************************************************\n\n");
            printf("**************************************     Temperature Sensor Triggered!!!!!!!     **************************************\n\n");    
            printf("*************************************************************************************************************************\n\n");
            
            //User has to input the password to clear the temperature sensor
            printf("\n\nEnter the password: ");
            enterTheCurrentPassword(temp);
            
            PORTDbits.RD0 = 0;                  //Turning off the yellow LED
            
            printf("\n\n\nI.   Press one to leave the temperature sensor on\n\n");
            printf("II.  Press two to turn off the temperature sensor off\n\n");
            
            //Expecting a choice from the user
            enterChoice(1, 1, temp);
            
            //User pressed one, leave the sensor on
            if(temp[0] == '1'){
                printf("\n\nTemperature sensor is left on\n\n\n");
 
            }
            //User pressed two, switch it off
            else if(temp[0] == '2'){ 
                temperatureSensor = 0;         //Assigning off to the temperature sensor
                PIE1bits.ADIE = 0;             //Disabling interrupts for the temperature sensor
                ADCON0bits.ADON = 0;           //Turning off the ADC module

                INTCONbits.TMR0IE = 0;         //Disabling timer interrupts
                T0CONbits.TMR0ON = 0;          //Turning off the timer
                
                //Clearing temperature variables
                currentTemp = 0;
                tempThreshold = 0;
                
                
                //Writing the threshold temperature to the EEPROM
                temp[0] = tempThreshold;
                temp[1] = tempThreshold;
                eepromWrite(threshold_temperature_size, threshold_temperature_address, temp);
                

                //Writing the change of state to EEPROM
                temp[0] = temperatureSensor;
                eepromWrite(temperature_sensor_status_size, temperature_sensor_status_address, temp);
                
                printf("\n\nTemperature sensor is off now\n\n\n");
            }
            else{
                printf("\n\nEither the entered number was not a choice. Motion sensor was left as is\n\n");
            }
            
            PIR1bits.ADIF = 0;                   //Clearing the tripped ADC flag
            
            //Do a software reset and not go back to main as going back to main makes program behave weird
            softReset = 1;
            temp[0] = softReset;
            eepromWrite(soft_reset_size, soft_reset_address, temp);
            Reset();
            
        }
        PIR1bits.ADIF = 0;                      //Clearing the tripped ADC flag
        
      
    }
    
    //Checking if timer 0 can interrupt and its flag is actually tripped
    if((INTCONbits.TMR0IE == 1) && (INTCONbits.TMR0IF == 1)){
        //Pre-loading the registers and making sure the high byte is loaded first
        TMR0H = 0x67;
        TMR0L = 0x69;
        
        
        PORTDbits.RD0 ^= 1;                        //Toggling the LED as an ADC conversion begins
        ADCON0bits.GO = 1;                         //Starting the ADC conversion again, since two seconds have elapsed
       
        INTCONbits.TMR0IF = 0;                     //Resetting the flag   
    }
}


/***************************************************************************************
 * celciusToFarenheit
 * 
 * Converts the calculated ADC Celcius readings to Fareheit
 * 
 * Input Arguments
 * double result  -  The Celcius reading that needs to be converted to Farenheit
 ***************************************************************************************/
double celciusToFarenheit(double result){
    double temp = result;
    
    temp = temp * 9;
    temp = temp / 5;
    temp = temp + 32;
    
    return temp;
}


/*********************************************************************
 * systemStartUp
 * 
 * This functions initializes the TRIS, output, ADC, USART, Timer, 
 * and LED state settings.
 *********************************************************************/
void systemStartUp(){
    //Disable all interrupts since we don't want any of them before logging in
    INTCONbits.GIE = 0;     
    
    //Settings for the system
    initTRISSettings();
    initLEDs();
    initADCSettings();
    initUSARTSettings();
    initTimer();
}


/************************************************************************************************
 * assigningAState
 * 
 * This functions starts the actual operation of th alarm. It prompts the user for the password 
 * if it when the alarm starts. If there is any previous state tot he alarm it restores it.
 * 
 * Input Arguments
 * char unsigned * userInputBuffer  -   Address to where the user input gets stored when the user 
 *                                      types the password
 * char unsigned * eepromBuffer     -   Address to the buffer used by the EEPROM functions
 ************************************************************************************************/
void assigningAState(char unsigned * userInputBuffer, char unsigned * eepromBuffer){  
    //Checking if there is a saved password
    eepromRead(password_size, password_address, eepromBuffer);
    
    //First character read is 255 means no password has been saved
    if(eepromBuffer[0] == 255){
        printf("\n\n");                                     //Garbage may have been printed when burning the hex file so start from a new line
        
        printf("**************************************************** NO PASSWORD SET ****************************************************\n\n");
        printf("Please enter a 4-character password: ");
        keyboardRead(password_size, 0, userInputBuffer);
        strncpy(password, userInputBuffer, 4);
        
        printf("\n\nRe-enter the password: ");
        
        //User gets unlimited number of times to re-enter the password and get it right
        while(1){
            keyboardRead(password_size, 0, userInputBuffer);
            if(!strncmp(userInputBuffer, password, 4)){
                printf("\n\n****** Login Successful! ******\n\n\n\n");
                break;
            }
            else{
                printf("\n\nPassword incorrect. \nEnter the password again: ");
            }
        }
                
        //Defining a state for the machine since this its first time running                        
        methodOfInput = 0;              //Method of input is keyboard
        temperatureSensor = 0;          //Temperature sensor doesn't cause interrupts           
        currentTemp = 0;                //Since ADC has never been turned on, temperature hasn't been read                                
        tempThreshold = 0;              //User has set no temperature threshold                             
        pirSensor = 0;                  //Motion sensor doesn't cause any interrupts
        
        //Storing password in EEPROM
        strncpy(eepromBuffer, password, 4);
        eepromWrite(password_size, password_address, eepromBuffer);
        
        //Storing method of input status in EEPROM
        eepromBuffer[0] = methodOfInput;
        eepromWrite(method_of_input_status_size, method_of_input_status_address, eepromBuffer);
        
        //Storing temperature sensor status in EEPROM
        eepromBuffer[0] = temperatureSensor;
        eepromWrite(temperature_sensor_status_size, temperature_sensor_status_address, eepromBuffer);
        
        //Storing temperature threshold value in EEPROM
        eepromBuffer[0] = tempThreshold;
        eepromBuffer[1] = tempThreshold;
        eepromWrite(threshold_temperature_size, threshold_temperature_address, eepromBuffer);
        
        //Storing motion sensor status in EEPROM
        eepromBuffer[0] = pirSensor;
        eepromWrite(motion_sensor_status_size, motion_sensor_status_address, eepromBuffer);
        
        //Setting up interrupts and turning on the components
        interruptSettingsAndTurningOnTheComponents();
    }
    else{
        //Populating state and operational variables before letting user input the password because have to figure out previous state's input method and other information
        
        //Retrieving soft reset status because if it is a soft reset, no need to ask for the user input again
        eepromRead(soft_reset_size, soft_reset_address, eepromBuffer);
        softReset = eepromBuffer[0];
        
        //Retrieving password
        eepromRead(password_size, password_address, eepromBuffer);
        strncpy(password, eepromBuffer, 4);
        
        //Retrieving method of input status
        eepromRead(method_of_input_status_size, method_of_input_status_address, eepromBuffer);
        methodOfInput = eepromBuffer[0];
        
        //From previous state matrix pad has been the input method
        if(methodOfInput){
            PORTCbits.RC2 = 1;             //Lighting up blue LED to show that matrix pad is active
        }
        
        //Retrieving temperature sensor status 
        eepromRead(temperature_sensor_status_size, temperature_sensor_status_address, eepromBuffer);
        temperatureSensor = eepromBuffer[0];
        
        //Retrieving temperature threshold value
        eepromRead(threshold_temperature_size, threshold_temperature_address, eepromBuffer);
        tempThreshold = eepromBuffer[0] * 10;       //Getting the digit in the tenth place
        tempThreshold += eepromBuffer[1];           //Getting the digit in the unit place
        
        //Retrieving  motion sensor status
        eepromRead(motion_sensor_status_size, motion_sensor_status_address, eepromBuffer);
        pirSensor = eepromBuffer[0];
        
        //If this is running because of a software reset, don't ask for the password
        if(!softReset){
            
            printf("\n\nEnter the password: ");
            enterTheCurrentPassword(userInputBuffer);
            printf("\n\n****** Login Successful! ******\n\n\n\n");
        }
        else{
            //Clear software reset status and update EEPROM
            softReset = 0;
            eepromBuffer[0] = softReset;
            eepromWrite(soft_reset_size, soft_reset_address, eepromBuffer);
        }
        
        //Setting up interrupts and turning on the components
        interruptSettingsAndTurningOnTheComponents();
    }
}


/*****************************************************************************
 * showHeaderAndStateOfTheAlarm
 * 
 * This function prints the header and the system variables when it is called.
 *****************************************************************************/
void showHeaderAndStateOfTheAlarm(){
    int temp;
    unsigned char digit_1;
    unsigned char digit_2;
    unsigned char digit_3;
    
    //Converting current temp to an int so that it can be printed with %d. It is multiplied by 10 to get the first decimal place of the double
    currentTemp = currentTemp * 10;
    temp = currentTemp;
    
    digit_1 = temp / 100;       //Getting the digit in the tenth place
    temp = temp % 100;          
    digit_2 = temp / 10;        //Getting the digit in the unit place       
    digit_3 = temp % 10;        //Getting the digit in the first decimal place
    
    
    //Header
    printf("\n\n\n*************************************************************************************************************************\n\n");
    printf("                              PIC18F4520 Standalone Alarm System with EUSART Communication                               \n");
    printf("                                                       Don Kuruppu                                                       \n");
    printf("                                                    1001 - 101 - 220                                                     \n");
    printf("                                           CSE 3442/5442 - Embedded Systems I                                            \n\n");
    printf("*************************************************************************************************************************\n\n\n");
    
    printf("-------------------------------------------------- State of the Alarm ---------------------------------------------------\n\n\n");
    //Displaying the state of the function
    if(methodOfInput == 1){
        printf("Method Of Input        -   Matrix Pad\n\n");
    }
    else{
        printf("Method Of Input        -   Keyboard\n\n");
    }
    
    if(pirSensor){
        printf("Motion Sensor          -   ON\n\n");
    }
    else{
        printf("Motion Sensor          -   OFF\n\n");
    }
    
    if(temperatureSensor){
        printf("Temperature Sensor     -   ON\n");
    }
    else{
        printf("Temperature Sensor     -   OFF\n");
    }
    printf("Current Temperature    -   %d%d.%dF\n", digit_1, digit_2, digit_3);
    printf("Threshold Temperature  -   %dF\n\n\n", tempThreshold);
}


/*****************************************************************************
 * showMenuAndGetUserInput
 * 
 * This function displays the options the alarm system supports and prompts the
 * user to choose one.
 * 
 * Input Arguments  
 * unsigned char * userInputBuffer  -   Address to where the user input is 
 *                                      stored
 *****************************************************************************/
void showMenuAndGetUserInput(unsigned char * userInputBuffer){
    //Displaying menu options
    printf("------------------------------------------------------- Main Menu -------------------------------------------------------\n\n\n");
    printf("I.   Press one to change the password\n\n");
    printf("II.  Press two to change motion sensor settings\n\n");
    printf("III. Press three to change temperature sensor settings\n\n");
    printf("IV.  Press four to change the method of input\n\n");
    printf("V.   Press zero to refresh\n\n\n");
    
    //Expecting user input
    enterChoice(1, 1, userInputBuffer);
}


/*****************************************************************************
 * identifyingTheKeyPressed
 * 
 * This function identifies which menu option the user selected and calls the 
 * right function to handle the operation.
 * 
 * Input Arguments  
 * unsigned char * userInputBuffer  -   Address to where the user input is 
 *                                      stored
 * unsigned char * eepromBuffer     -   Needed by the functions called inside
 *                                      this function to read and write to the
 *                                      EEPROM
 *****************************************************************************/
void identifyingTheKeyPressed(unsigned char * userInputBuffer, unsigned char * eepromBuffer){
    //Call the right function considering the key that the user pressed
    if(userInputBuffer[0] == '1'){
         userPressedOne(userInputBuffer, eepromBuffer);
    }
    else if(userInputBuffer[0] == '2'){
         userPressedTwo(userInputBuffer, eepromBuffer);
    }
    else if(userInputBuffer[0] == '3'){
         userPressedThree(userInputBuffer, eepromBuffer);
    }
    else if(userInputBuffer[0] == '4'){
         userPressedFour(userInputBuffer, eepromBuffer);
    }
    else if(userInputBuffer[0] == '0'){
        return;
    }
    else{
        printf("\nEntered number doesn't execute any action. Please enter another number.\n\n\n");
    }
       
}


/***********************************************************************************
 * userPressedOne
 * 
 * This function handles the case of when a user wants to change the alarm password
 * 
 * Input Arguments  
 * unsigned char * userInputBuffer  -   Address to where the user input will be
 *                                      stored
 * unsigned char * eepromBuffer     -   Buffer for the EEPROM communication
 ***********************************************************************************/
void userPressedOne(unsigned char * userInputBuffer, unsigned char * eepromBuffer){
    //Retrieving password
    eepromRead(password_size, password_address, eepromBuffer);
    strncpy(password, eepromBuffer, 4);
    
    printf("\n\n\nEnter the current password: ");
    enterTheCurrentPassword(userInputBuffer);
    
    
    printf("\n\nEnter the new password: ");
    enterChoice(password_size, 0, userInputBuffer);
    
    //Updating the password buffer to show the changed password
    strncpy(password, userInputBuffer, 4);
    
    //Copying the new password from the corresponding buffer to EEPROM to write to EEPROM
    strncpy(eepromBuffer, userInputBuffer, 4);
    eepromWrite(password_size, password_address, userInputBuffer);
    
    printf("\n\n****** Password Change Successful! ******\n\n\n\n");
}


/*******************************************************************************
 * userPressedTwo
 * 
 * This function gives the user the ability to turn on/off the motion sensor
 * 
 * Input Arguments  
 * unsigned char * userInputBuffer  -   Address to where the user input will be
 *                                      stored
 * unsigned char * eepromBuffer     -   Buffer for the EEPROM communication
 *******************************************************************************/
void userPressedTwo(unsigned char * userInputBuffer, unsigned char * eepromBuffer){
    printf("\n\n\nI.   Press one to turn on the motion sensor\n\n");
    printf("II.  Press two to turn off the motion sensor\n\n");
    
    enterChoice(1, 1, userInputBuffer);
    
    if((userInputBuffer[0] == '1') && (pirSensor == 0)){
        pirSensor = 1;         //Assigning on to the motion sensor
        
        //Writing the change of state to EEPROM. Important to do update the state change in the EEPROM before the system could go to the interrupt section and do the software reset
        eepromBuffer[0] = pirSensor;
        eepromWrite(motion_sensor_status_size, motion_sensor_status_address, eepromBuffer);
        
        printf("\n\nMotion sensor is on now\n\n\n");
        INTCONbits.INT0IE = 1; //Enabling interrupts for the motion sensor 
    }
    else if((userInputBuffer[0] == '2') && (pirSensor == 1)){ 
        pirSensor = 0;         //Assigning off to the motion sensor
        
        //Writing the change of state to EEPROM
        eepromBuffer[0] = pirSensor;
        eepromWrite(motion_sensor_status_size, motion_sensor_status_address, eepromBuffer);
        
        printf("\n\nMotion sensor is off now\n\n\n");
        INTCONbits.INT0IE = 0; //Disabling interrupts for the motion sensor
    }
    else{
        printf("\n\nEither the entered number was not a choice or motion sensor was already in the state that it was required to changed to\n\n\n");
    }
    
}


/********************************************************************************
 * userPressedThree
 * 
 * This function gives the user the ability to turn on/off the temperature sensor
 * and all the ability to set a threshold temperature value
 * 
 * Input Arguments  
 * unsigned char * userInputBuffer  -   Address to where the user input will be
 *                                      stored
 * unsigned char * eepromBuffer     -   Buffer for the EEPROM communication
 ********************************************************************************/
void userPressedThree(unsigned char * userInputBuffer, unsigned char * eepromBuffer){
    printf("\n\n\nI.   Press one to turn on the temperature sensor\n\n");
    printf("II.  Press two to turn off the temperature sensor sensor\n\n");
    
    enterChoice(1, 1, userInputBuffer);
    
    if((userInputBuffer[0] == '1') && (temperatureSensor == 0)){
        
        temperatureSensor = 1;         //Assigning on to the temperature sensor
        
        //Writing the change of state to EEPROM. Important to do update the state change in the EEPROM before the system could go to the interrupt section and do the software reset
        eepromBuffer[0] = temperatureSensor;
        eepromWrite(temperature_sensor_status_size, temperature_sensor_status_address, eepromBuffer);
        
        printf("\n\nEnter the desired threshold temperature: ");
        
        //Will keep asking till the user enters only numbers for the temperature value
        while(1){
            enterChoice(2, 1, userInputBuffer);
            if(((userInputBuffer[0] <= '9') && (userInputBuffer[0] >= '0')) && ((userInputBuffer[1] <= '9') && (userInputBuffer[1] >= '0'))){
                
                //Converting to ASCII representation of the numbers to numbers themselves 
                tempThreshold = (userInputBuffer[0] - 48) * 10;                 //Getting the digit at the tenth place
                tempThreshold = tempThreshold + (userInputBuffer[1] - 48);      //Getting the digit at the unit place
                
                //Writing the threshold temperature to the EEPROM
                eepromBuffer[0] = userInputBuffer[0] - 48;
                eepromBuffer[1] = userInputBuffer[1] - 48;
                
                eepromWrite(threshold_temperature_size, threshold_temperature_address, eepromBuffer);
                break;
            }
            else{
                printf("\n\nEnter digits only.\n");
            }
        }
        
        printf("\n\nTemperature sensor is on now\n\n\n");
         
        //Pre-loading the timer 0 registers and making sure the high byte is loaded first, this is because timer 0 has to start when ADC starts
        TMR0H = 0x67;
        TMR0L = 0x69;
        
        INTCONbits.TMR0IF = 0;         //Clearing the timer flag
        INTCONbits.TMR0IE = 1;         //Enabling timer interrupts
        T0CONbits.TMR0ON = 1;          //Turning on the timer
        
        PIR1bits.ADIF = 0;             //Clearing ADC flag
        PIE1bits.ADIE = 1;             //Enabling interrupts for the temperature sensor
        PORTDbits.RD0 = 1;             //Turning on the yellow LED since an ADC conversion is about to start
        ADCON0bits.ADON = 1;           //Turning on the ADC module
        ADCON0bits.GO = 1;             //Starting the ADC conversion
       
    }
    else if((userInputBuffer[0] == '2') && (temperatureSensor == 1)){ 
        
        temperatureSensor = 0;         //Assigning off to the temperature sensor
        
        //Writing the change of state to EEPROM
        eepromBuffer[0] = temperatureSensor;
        eepromWrite(temperature_sensor_status_size, temperature_sensor_status_address, eepromBuffer);
        
        PIE1bits.ADIE = 0;             //Disabling interrupts for the temperature sensor
        ADCON0bits.ADON = 0;           //Turning off the ADC module
        
        INTCONbits.TMR0IE = 0;         //Disabling timer interrupts
        T0CONbits.TMR0ON = 0;          //Turning off the timer
        
        PORTDbits.RD0 = 0;             //Turning off the yellow LED if it is still on
        
        //Clearing temperature variables
        currentTemp = 0;
        tempThreshold = 0;
        
         //Writing the threshold temperature to the EEPROM
        eepromBuffer[0] = tempThreshold;
        eepromBuffer[1] = tempThreshold;
        eepromWrite(threshold_temperature_size, threshold_temperature_address, eepromBuffer);
        
        printf("\n\nTemperature sensor is off now\n\n\n");
    }
    else{
        printf("\n\nEither the entered number was not a choice or temperature sensor was already in the state that it was required to changed to\n\n\n");
    }
    
    
}


/*******************************************************************************
 * userPressedFour
 * 
 * TThis function gives the user the ability to choose the method of input
 * 
 * Input Arguments  
 * unsigned char * userInputBuffer  -   Address to where the user input will be
 *                                      stored
 * unsigned char * eepromBuffer     -   Buffer for the EEPROM communication
 *******************************************************************************/
void userPressedFour(unsigned char * userInputBuffer, unsigned char * eepromBuffer){
    printf("\n\n\nI.   Press one to change the method of input to matrix pad\n\n");
    printf("II.  Press two to change method of input to keyboard\n\n");
    
    enterChoice(1, 1, userInputBuffer);
    
    if((userInputBuffer[0] == '1') && (methodOfInput == 0)){
        methodOfInput = 1;       //Assigning matrix as the method of input
        PORTCbits.RC2 = 1;       //Turning on the blue LED
        printf("\n\nMethod of input is the matrix pad now\n\n\n");
    }
    else if((userInputBuffer[0] == '2') && (methodOfInput == 1)){ 
        methodOfInput = 0;       //Assigning keyboard as the method of input
        PORTCbits.RC2 = 0;       //Turning off the blue LED
        printf("\n\nMethod of input is the keyboard now\n\n\n");    
    }
    else{
        printf("\n\nEither the entered number was not a choice or the selected input method was the current input method\n\n\n");
        return;                  //No need to proceed further to write data to EEPROM as no change was made
    }
 
    //Writing the change of state to EEPROM
    eepromBuffer[0] = methodOfInput;
    eepromWrite(method_of_input_status_size, method_of_input_status_address, eepromBuffer);
}
