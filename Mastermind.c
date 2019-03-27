// Button-controlled LED (in C), now truly standalone, controlling LED and button
// Same as tinkerHaWo35.c but using different pins: pin 23 for LED, pin 24 for button

// Compile: gcc  -o  t tut_button.c
// Run:     sudo ./t

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <poll.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>

// original code based in wiringPi library by Gordon Henderson
// #include "wiringPi.h"

// =======================================================
// Tunables
// PINs (based on BCM numbering)
#define LEDYELLOW 13
#define LEDRED 5
#define BUTTON 19

#define STRB_PIN 24
#define RS_PIN   25
#define DATA0_PIN 23
// #define DATA1_PIN 17
#define DATA1_PIN 10
#define DATA2_PIN 27
#define DATA3_PIN 22
// delay for loop iterations (mainly), in ms
#define DELAY 200
// =======================================================

#ifndef	TRUE
#define	TRUE	(1==1)
#define	FALSE	(1==2)
#endif

#define	PAGE_SIZE		(4*1024)
#define	BLOCK_SIZE		(4*1024)

#define	INPUT			 0
#define	OUTPUT			 1

#define	LOW			 0
#define	HIGH			 1

static unsigned char newChar [8] =
{
    0b11111,
    0b10001,
    0b10001,
    0b10101,
    0b11111,
    0b10001,
    0b10001,
    0b11111,
} ;

/* bit pattern to feed into lcdCharDef to define a new character */

static unsigned char hawoNewChar [8] =
{
    0b11111,
    0b10001,
    0b10001,
    0b10001,
    0b10001,
    0b10001,
    0b10001,
    0b11111,
} ;

// data structure holding data on the representation of the LCD
struct lcdDataStruct
{
    int bits, rows, cols ;
    int rsPin, strbPin ;
    int dataPins [8] ;
    int cx, cy ;
} ;

static int lcdControl ;

/* ***************************************************************************** */
/* INLINED fcts from wiringPi/devLib/lcd.c: */
// HD44780U Commands (see Fig 11, p28 of the Hitachi HD44780U datasheet)

#define	LCD_CLEAR	0x01
#define	LCD_HOME	0x02
#define	LCD_ENTRY	0x04
#define	LCD_CTRL	0x08
#define	LCD_CDSHIFT	0x10
#define	LCD_FUNC	0x20
#define	LCD_CGRAM	0x40
#define	LCD_DGRAM	0x80

// Bits in the entry register

#define	LCD_ENTRY_SH		0x01
#define	LCD_ENTRY_ID		0x02

// Bits in the control register

#define	LCD_BLINK_CTRL		0x01
#define	LCD_CURSOR_CTRL		0x02
#define	LCD_DISPLAY_CTRL	0x04

// Bits in the function register

#define	LCD_FUNC_F	0x04
#define	LCD_FUNC_N	0x08
#define	LCD_FUNC_DL	0x10

#define	LCD_CDSHIFT_RL	0x04

static volatile unsigned int gpiobase ;
static volatile uint32_t *gpio ;


// Mask for the bottom 64 pins which belong to the Raspberry Pi
//	The others are available for the other devices

#define	PI_GPIO_MASK	(0xFFFFFFC0)

/* -------------------------------------------------/------ */
// protos
int failure (int fatal, const char *message, ...);
void waitForEnter (void);

/* ------------------------------------------------------- */
/* low-level interface to the hardware */

/* digitalWrite is now in a separate file: lcdBinary.c */
//void digitalWrite (uint32_t *gpio, int pin, int value);

/* ----------------------*/


void digitalWrite(int PIN, int val) {				//Digital write function, takes in the PIN and value as arguments
    int off;
    int res;
    if (val == HIGH) {						//if the value is high then off is equal to 7, which is set0
        off = 7;
    }
    else if(val == LOW) {					//if the value is low then off is equal to 10, which is clr0
        off = 10;
    }

    asm volatile(/* inline assembler version of setting/clearing LED to
      ouput" */
        "\tLDR R1, %[gpio]\n"					//loads gpio into register 1
        "\tADD R0, R1, %[off]\n" /* R0 = GPSET/GPCLR register */
        "\tMOV R2, #1\n"
        "\tMOV R1, %[pin]\n" /* NB: this works only for pin 0-31 */
        "\tAND R1, #31\n"
        "\tLSL R2, R1\n" /* R2 = bitmask set/clear reg %[act] */
        "\tSTR R2, [R0, #0]\n" /* write bitmask */
        "\tMOV %[result], R2\n"
        : [result] "=r" (res)
        : [pin] "r" (PIN)
        , [gpio] "m" (gpio)
        , [off] "r" (off*4)
        : "r0", "r1", "r2", "cc");

}


void pinMode(int PIN,int mode) {				//function for setting pin and pin mode
    int res;
    int fSel = PIN/10;						//gets the register by dividing the pin number by 10
    int shift = (PIN%10)*3;					//multiplies the value of pin%10 by 3 to get the shift, since 3 bits per pin
    asm(/* inline assembler version of setting LED to ouput" */
        "\tLDR R1, %[gpio]\n"
        "\tADD R0, R1, %[fSel]\n"  /* R0 = GPFSEL register to write to */
        "\tLDR R1, [R0, #0]\n"     /* read current value of the register */
        "\tMOV R2, #0b111\n"
        "\tLSL R2, %[shift]\n"
        "\tBIC R1, R1, R2\n"
        "\tMOV R2, %[mode]\n"
        "\tLSL R2, %[shift]\n"
        "\tORR R1, R2\n"
        "\tSTR R1, [R0, #0]\n"
        "\tMOV %[result], R1\n"
        : [result] "=r" (res)
        : [act] "r" (PIN)
        , [gpio] "m" (gpio)
        , [fSel] "r" (fSel*4)  /* offset in bytes! */
        , [shift] "r" (shift)
        , [mode] "r" (mode)
        : "r0", "r1", "r2", "cc");
}

int assemblyInput(int PIN) {						//reads input from a button
    int res=0;
    int lev;
    int reg = PIN/32;
    if(reg > 0){
        lev = 14*4;
    }
    else{
        lev = 13*4;
    }
    
    asm volatile(
        "\tLDR R0, [%[gpio], %[lev]]\n"          //LEV13, reading the value
        "\tMOV R1, %[pin]\n"
        "\tAND R1, #31\n"
        "\tMOV R2, #1\n"
        "\tLSL R2, R1\n"
        "\tAND R0, R2\n"
        "\tMOV %[result], R0\n"
        :[result] "=r" (res)
        :[pin] "r" (PIN)
        ,[gpio] "r" (gpio)
        ,[lev] "r" (lev)
        : "r0", "r1", "r2");
    return res;
}


int failure (int fatal, const char *message, ...)
{
    va_list argp ;
    char buffer [1024] ;

    if (!fatal) //  && wiringPiReturnCodes)
        return -1 ;

    va_start (argp, message) ;
    vsnprintf (buffer, 1023, message, argp) ;
    va_end (argp) ;

    fprintf (stderr, "%s", buffer) ;
    exit (EXIT_FAILURE) ;

    return 0 ;
}

void waitForEnter (void)
{
    printf ("Press ENTER to continue: ") ;
    (void)fgetc (stdin) ;
}

void delay (unsigned int howLong)
{
    struct timespec sleeper, dummy ;

    sleeper.tv_sec  = (time_t)(howLong / 1000) ;
    sleeper.tv_nsec = (long)(howLong % 1000) * 1000000 ;

    nanosleep (&sleeper, &dummy) ;
}

void delayMicroseconds (unsigned int howLong)
{
    struct timespec sleeper ;
    unsigned int uSecs = howLong % 1000000 ;
    unsigned int wSecs = howLong / 1000000 ;

    /**/ if (howLong ==   0)
        return ;
#if 0
    else if (howLong  < 100)
        delayMicrosecondsHard (howLong) ;
#endif
    else
    {
        sleeper.tv_sec  = wSecs ;
        sleeper.tv_nsec = (long)(uSecs * 1000L) ;
        nanosleep (&sleeper, NULL) ;
    }
}

void strobe (const struct lcdDataStruct *lcd)
{

    // Note timing changes for new version of delayMicroseconds ()
    digitalWrite (lcd->strbPin, 1) ;
    delayMicroseconds (50) ;
    digitalWrite (lcd->strbPin, 0) ;
    delayMicroseconds (50) ;
}

void sendDataCmd (const struct lcdDataStruct *lcd, unsigned char data)
{
    register unsigned char myData = data ;
    unsigned char          i, d4 ;

    if (lcd->bits == 4)
    {
        d4 = (myData >> 4) & 0x0F;
        for (i = 0 ; i < 4 ; ++i)
        {
            digitalWrite (lcd->dataPins [i], (d4 & 1)) ;
            d4 >>= 1 ;
        }
        strobe (lcd) ;

        d4 = myData & 0x0F ;
        for (i = 0 ; i < 4 ; ++i)
        {
            digitalWrite (lcd->dataPins [i], (d4 & 1)) ;
            d4 >>= 1 ;
        }
    }
    else
    {
        for (i = 0 ; i < 8 ; ++i)
        {
            digitalWrite (lcd->dataPins [i], (myData & 1)) ;
            myData >>= 1 ;
        }
    }
    strobe (lcd) ;
}

void lcdPutCommand (const struct lcdDataStruct *lcd, unsigned char command)
{
#ifdef DEBUG
    fprintf(stderr, "lcdPutCommand: digitalWrite(%d,%d) and sendDataCmd(%d,%d)\n", lcd->rsPin,   0, lcd, command);
#endif
    digitalWrite (lcd->rsPin,   0) ;
    sendDataCmd  (lcd, command) ;
    delay (2) ;
}

void lcdPut4Command (const struct lcdDataStruct *lcd, unsigned char command)
{
    register unsigned char myCommand = command ;
    register unsigned char i ;

    digitalWrite (lcd->rsPin,   0) ;

    for (i = 0 ; i < 4 ; ++i)
    {
        digitalWrite (lcd->dataPins [i], (myCommand & 1)) ;
        myCommand >>= 1 ;
    }
    strobe (lcd) ;
}

void lcdHome (struct lcdDataStruct *lcd)
{
#ifdef DEBUG
    fprintf(stderr, "lcdHome: lcdPutCommand(%d,%d)\n", lcd, LCD_HOME);
#endif
    lcdPutCommand (lcd, LCD_HOME) ;
    lcd->cx = lcd->cy = 0 ;
    delay (5) ;
}

void lcdClear (struct lcdDataStruct *lcd)
{
#ifdef DEBUG
    fprintf(stderr, "lcdClear: lcdPutCommand(%d,%d) and lcdPutCommand(%d,%d)\n", lcd, LCD_CLEAR, lcd, LCD_HOME);
#endif
    lcdPutCommand (lcd, LCD_CLEAR) ;
    lcdPutCommand (lcd, LCD_HOME) ;
    lcd->cx = lcd->cy = 0 ;
    delay (5) ;
}

void lcdPosition (struct lcdDataStruct *lcd, int x, int y)
{
    // struct lcdDataStruct *lcd = lcds [fd] ;

    if ((x > lcd->cols) || (x < 0))
        return ;
    if ((y > lcd->rows) || (y < 0))
        return ;

    lcdPutCommand (lcd, x + (LCD_DGRAM | (y>0 ? 0x40 : 0x00)  /* rowOff [y] */  )) ;

    lcd->cx = x ;
    lcd->cy = y ;
}

void lcdDisplay (struct lcdDataStruct *lcd, int state)
{
    if (state)
        lcdControl |=  LCD_DISPLAY_CTRL ;
    else
        lcdControl &= ~LCD_DISPLAY_CTRL ;

    lcdPutCommand (lcd, LCD_CTRL | lcdControl) ;
}

void lcdCursor (struct lcdDataStruct *lcd, int state)
{
    if (state)
        lcdControl |=  LCD_CURSOR_CTRL ;
    else
        lcdControl &= ~LCD_CURSOR_CTRL ;

    lcdPutCommand (lcd, LCD_CTRL | lcdControl) ;
}

void lcdCursorBlink (struct lcdDataStruct *lcd, int state)
{
    if (state)
        lcdControl |=  LCD_BLINK_CTRL ;
    else
        lcdControl &= ~LCD_BLINK_CTRL ;

    lcdPutCommand (lcd, LCD_CTRL | lcdControl) ;
}

void lcdPutchar (struct lcdDataStruct *lcd, unsigned char data)
{
    digitalWrite (lcd->rsPin, 1) ;
    sendDataCmd  (lcd, data) ;

    if (++lcd->cx == lcd->cols)
    {
        lcd->cx = 0 ;
        if (++lcd->cy == lcd->rows)
            lcd->cy = 0 ;

        // TODO: inline computation of address and eliminate rowOff
        lcdPutCommand (lcd, lcd->cx + (LCD_DGRAM | (lcd->cy>0 ? 0x40 : 0x00)   /* rowOff [lcd->cy] */  )) ;
    }
}

void lcdPuts (struct lcdDataStruct *lcd, const char *string)
{
    while (*string)
        lcdPutchar (lcd, *string++) ;
}

void blinkRed(int n) {						//function to blink the red LED
    int i;
    for(i = 0; i < n; i++) {
        int theValue = ((i  % 2) == 0) ? LOW : HIGH;		//if the value%2 is 0 then it sets the value to LOW,else it's HIGH
        int off = (theValue == HIGH) ? 10 : 7;			//if the value is high then off is clr0, else set0
        *(gpio + off) = 1 << (LEDRED & 31);
        delay(1000);
    }
}

void blinkYellow(int n) {					//function to blink the yellow LED
    int i;
    for(i = 0; i < n; i++) {
        int theValue = ((i  % 2) == 0) ? LOW : HIGH;		//if the value%2 is 0 then it sets the value to LOW,else it's HIGH
        int off = (theValue == HIGH) ? 10 : 7;			//if the value is high then off is clr0, else set0
        *(gpio + off) = 1 << (LEDYELLOW & 31);
        delay(500);
    }
}

void blinkRedAssembly(int n) {					//blink red LED in assembly
    int theValue;
    for(int i = 0; i < n; i++) {
        theValue = ((i% 2) == 0) ? HIGH : LOW;
        int off = (theValue == LOW) ? 10 : 7; 
        uint32_t res;


        asm volatile(/* inline assembler version of setting/clearing LED to ouput" */
            "\tLDR R1, %[gpio]\n"
            "\tADD R0, R1, %[off]\n"  /* R0 = GPSET/GPCLR register to write to */
            "\tMOV R2, #1\n"
            "\tMOV R1, %[act]\n"
            "\tAND R1, #31\n"
            "\tLSL R2, R1\n"          /* R2 = bitmask setting/clearing register %[act] */
            "\tSTR R2, [R0, #0]\n"    /* write bitmask */
            "\tMOV %[result], R2\n"
            : [result] "=r" (res)
            : [act] "r" (LEDRED)
            , [gpio] "m" (gpio)
            , [off] "r" (off*4)
            : "r0", "r1", "r2", "cc");


        delay(1000);
    }
}
//This function blinks the yellow LED in asembly
void blinkYellowAssembly(int n) {			//blink yellow LED in assembly
    
    //The loop that controls the value of the off variable, which holds the register to be used
    int theValue;
    for(int i = 0; i < n; i++) {
        theValue = ((i % 2) == 0) ? HIGH : LOW;
        int off = (theValue == LOW) ? 10 : 7; 
        uint32_t res;


        asm volatile(/* inline assembler version of setting/clearing LED to ouput" */

            "\tLDR R1, %[gpio]\n"
            "\tADD R0, R1, %[off]\n"  /* R0 = GPSET/GPCLR register to write to */
            "\tMOV R2, #1\n"
            "\tMOV R1, %[act]\n"
            "\tAND R1, #31\n"
            "\tLSL R2, R1\n"          /* R2 = bitmask setting/clearing register %[act] */
            "\tSTR R2, [R0, #0]\n"    /* write bitmask */
            "\tMOV %[result], R2\n"
            : [result] "=r" (res)
            : [act] "r" (LEDYELLOW)
            , [gpio] "m" (gpio)
            , [off] "r" (off*4)
            : "r0", "r1", "r2", "cc");


        delay(500);
    }
}

//This function allows the user to enter the secret for someone else to guess
int *colorInput(int loopNum, int numColors) {


    //Mallocs the array that will hold the secret
    int *secret = (int*)malloc(sizeof(int) * loopNum);
    
    int theValue;
    int counter = 0;
    int timer = 0;
    int colorNum;
    int currentvalue = HIGH;
    
    //This loops runs as many times as the chosen length of the secret
    fprintf(stderr, "-----------------------------\nStart entering the secret\n-----------------------------\n\n");
    for(colorNum = 0; colorNum <loopNum; colorNum++) {
        fprintf(stderr, "  -----------------\n\n  Enter color number %d\n\n", colorNum+1);
        while(counter < numColors) {
            if((BUTTON & 0xFFFFFFC0) == 0) {
                theValue = LOW;

                //Check the button value, if it's not 0, then it's being pressed, and also check if it was being pressed in the pervious
                //loop, this is to avoid one press registering as multiple presses
                if (((*(gpio + 13) & (1 << (BUTTON & 31))) != 0) && (currentvalue == HIGH)) {
                    theValue = LOW;
                    currentvalue = LOW;
                    counter++;
                    timer = 0;
                    printf("    Button Pressed\n");
                }
                else if (((*(gpio + 13) & (1 << (BUTTON & 31))) == 0) && (currentvalue == LOW)) {
                    theValue = HIGH;
                    currentvalue = HIGH;
                }
            }
            else {
                fprintf(stderr, "only supporting on-board pins\n");
            }
            delay(50);
            timer++;
            
            //Stop recording the current color if the last button press was 50 cycles ago (50ms each)
            if(timer == 50) {
                break;
            }
        }
        secret[colorNum] = counter;
        printf("\n  End of guess %d\n", colorNum+1);
        printf("  You pressed the button %d time(s)\n\n", counter);
        blinkRed(2);
        blinkYellow((counter*2));
        counter = 0;
        timer = 0;
    }
    printf("-----------------------------\nEnd of entering the secret\n-----------------------------\n\n");
    
    blinkRedAssembly(4);

    return secret;

}

//Sets the pins for lcd, and returns a struct to access the lcd
struct lcdDataStruct *setlcd () {
    struct lcdDataStruct *lcd;
    int bits, rows, cols ;
    unsigned char func ;

    bits = 4;
    cols = 16;
    rows = 2;

    struct tm *t ;
    time_t tim ;

    int   fd ;

    char buf [32] ;

    lcd = (struct lcdDataStruct *)malloc (sizeof (struct lcdDataStruct)) ;
    if (lcd == NULL)
        exit(1);



    // hard-wired GPIO pins
    lcd->rsPin   = RS_PIN ;
    lcd->strbPin = STRB_PIN ;
    lcd->bits    = 4 ;
    lcd->rows    = rows ;  // # of rows on the display
    lcd->cols    = cols ;  // # of cols on the display
    lcd->cx      = 0 ;     // x-pos of cursor
    lcd->cy      = 0 ;     // y-pos of curosr

    lcd->dataPins [0] = DATA0_PIN ;
    lcd->dataPins [1] = DATA1_PIN ;
    lcd->dataPins [2] = DATA2_PIN ;
    lcd->dataPins [3] = DATA3_PIN ;
    // lcd->dataPins [4] = d4 ;
    // lcd->dataPins [5] = d5 ;
    // lcd->dataPins [6] = d6 ;
    // lcd->dataPins [7] = d7 ;

    // lcds [lcdFd] = lcd ;

    digitalWrite (lcd->rsPin,   0) ;
    pinMode (lcd->rsPin,   OUTPUT) ;
    digitalWrite (lcd->strbPin, 0) ;
    pinMode ( lcd->strbPin, OUTPUT) ;



    for (int i = 0 ; i < bits ; ++i)
    {
        digitalWrite (lcd->dataPins [i], 0) ;
        pinMode      (lcd->dataPins [i], OUTPUT) ;
    }
    delay (35) ; // mS

    if (bits == 4)
    {
        func = LCD_FUNC | LCD_FUNC_DL ;			// Set 8-bit mode 3 times
        lcdPut4Command (lcd, func >> 4) ;
        delay (35) ;
        lcdPut4Command (lcd, func >> 4) ;
        delay (35) ;
        lcdPut4Command (lcd, func >> 4) ;
        delay (35) ;
        func = LCD_FUNC ;					// 4th set: 4-bit mode
        lcdPut4Command (lcd, func >> 4) ;
        delay (35) ;
        lcd->bits = 4 ;
    }
    else
    {
        failure(TRUE, "setup: only 4-bit connection supported\n");
        func = LCD_FUNC | LCD_FUNC_DL ;
        lcdPutCommand  (lcd, func     ) ;
        delay (35) ;
        lcdPutCommand  (lcd, func     ) ;
        delay (35) ;
        lcdPutCommand  (lcd, func     ) ;
        delay (35) ;
    }

    if (lcd->rows > 1)
    {
        func |= LCD_FUNC_N ;
        lcdPutCommand (lcd, func) ;
        delay (35) ;
    }

    // Rest of the initialisation sequence
    lcdDisplay     (lcd, TRUE) ;
    lcdCursor      (lcd, FALSE) ;
    lcdCursorBlink (lcd, FALSE) ;
    lcdClear       (lcd) ;

    lcdPutCommand (lcd, LCD_ENTRY   | LCD_ENTRY_ID) ;    // set entry mode to increment address counter after write
    lcdPutCommand (lcd, LCD_CDSHIFT | LCD_CDSHIFT_RL) ;  // set display shift to right-to-left
    // ------

    return lcd;

}



/* Main ----------------------------------------------------------------------------- */

int main (int argc, char **argv)
{
    if(argc != 0 ) {
        if(argv[argc-1][0] == 'd') {				// To enter debug mode, where the secret will be displayed to the user at the start
            printf("Welcome to debug mode (YOU CHEATER)\n\n", argc);
        }
        else if(argv[argc-1][0] == '.') {			// Normal game mode
            printf("Welcome to Mastermind :)\n\n");
        }
        else {
            printf("This is not a supported mode(try 'd' or nothing)\n\n");
        }

    }

    int   fd ;

    //printf ("Raspberry Pi button controlled LED (button in %d, led out %d)\n", BUTTON, LEDYELLOW) ;

    if (geteuid () != 0)
        fprintf (stderr, "setup: Must be root. (Did you forget sudo?)\n") ;

    // -----------------------------------------------------------------------------
    // constants for RPi2
    gpiobase = 0x3F200000 ;

    // -----------------------------------------------------------------------------
    // memory mapping
    // Open the master /dev/memory device

    if ((fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)		//enables file read and write
        return failure (FALSE, "setup: Unable to open /dev/mem: %s\n", strerror (errno)) ;

    // GPIO:
    gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, gpiobase) ;
    if ((int32_t)gpio == -1)
        return failure (FALSE, "setup: mmap (GPIO) failed: %s\n", strerror (errno)) ;

    // -----------------------------------------------------------------------------
    // setting the mode

    // controlling YELLOW LED pin 13
    pinMode(LEDYELLOW, OUTPUT);			//calls the inline function for setting the pin and mode, in this case it's output

    // controller RED LED pin 5
    pinMode(LEDRED, OUTPUT);			//calls the inline function for setting the pin and mode, in this case it's output

    // controlling button pin 
    pinMode(BUTTON, INPUT);			//calls the inline function for setting the pin and mode, here it's input because we're using a button

    // -----------------------------------------------------------------------------
    struct lcdDataStruct *lcd = setlcd();

    int mode;
    int length;
    int colors;
    printf("1-Single Player(Randomly Generated)\n2-Two Player\nplease select an option: ");		//providing the user with an option, which will be entered through the terminal
    scanf("%d",&mode);

    printf("Enter the length of the secret: ");
    scanf("%d",&length);

    printf("Enter number of colours available: ");
    scanf("%d",&colors);
    printf("\n\n");

    if (mode==1) {

        int secret[length];
        srand(time(NULL));				//randomly generates the secret for the user, that will be used in the game
        for (int i=0; i<length; i++) {
            secret[i]=(rand() % colors)+1 ;		//stores each random value in an array, the random value starts from 1 and goes up tp the number of colours available 
        }
        if(argv[argc-1][0] == 'd') {			//debug mode
            printf("The secret is\n");
            for(int i=0; i<length; i++) {
                fprintf(stderr,"%d   ", secret[i]);		//displays secret for the user
            }
        }
        delay(3000);
        fprintf(stderr, "\n\n-----------------\nStarting Round 1\n-----------------\n\n");
        game(secret, length, colors, lcd, 0);			//takes in the arguments for the game function
    }
    else if (mode==2) {
        delay(3000);
        int *secret = colorInput(length, colors);		//used a pointer to point to the values passed into the colorInput function

        if(argv[argc-1][0] == 'd') {
            printf("The secret is\n");
            for(int i=0; i<length; i++) {
                fprintf(stderr,"%d   ", secret[i]);
            }
        }
        delay(3000);
        fprintf(stderr, "\n\n-----------------\nStarting Round 1\n-----------------\n\n");
        game(secret, length, colors, lcd, 0);

    }
    else{
        printf("This game mode is not supported\n\n------------------\n\n");
        main(argc, argv);
    }

}


void game (int *mainSecret, int sequenceLength, int maxColors, struct lcdDataStruct *lcd, int roundNum) //roundNum variable created for the number of attempts
{
    int secret[sequenceLength];
    for(int i = 0; i<sequenceLength; i++) {
        secret[i] = mainSecret[i];
    }

    if (roundNum!=3)		//checks if roundNum does not equal to 3, because the max number of attempts is 3, so the user can keep trying until roundNum=3
    {
        int theValue;



        // now, start a loop, listening to pinButton and if set pressed, set pinLED
        int res = 0;
        int counter = 0;					//stores the number of times the button was pressed
        int timer = 0;
        int colors[sequenceLength];				//array to store the input from the user
        int currentvalue = HIGH;
        int colorNum;
        for(colorNum = 0; colorNum <sequenceLength; colorNum++) {
            fprintf(stderr, "  -----------------\n\n  Starting guess %d\n\n", colorNum+1);
            while(counter < maxColors) {			//loops until the counter is less than the max number of colours available
                res = assemblyInput(BUTTON);
                if((BUTTON & 0xFFFFFFC0) == 0) {
                    theValue = LOW;				//sets the value to LOW

                    if ((res != 0) && (currentvalue == HIGH)) {
                        theValue = LOW;
                        currentvalue = LOW;
                        counter++;				//only increments if the value has changed from high to low; even if the button has been pressed for 2 seconds the value will only increment by 1
                        timer = 0;
                        printf("    Button Pressed\n");
                    }
                    else if ((res== 0) && (currentvalue == LOW)) {
                        theValue = HIGH;
                        currentvalue = HIGH;
                    }
                }
                else {
                    fprintf(stderr, "only supporting on-board pins\n");
                }
                delay
                (50);
                timer++;

                if(timer == 50) {
                    break;
                }
            }
            printf("\n  End of guess %d\n", colorNum+1);
            printf("  You pressed the button %d time(s)\n\n", counter);
            colors[colorNum] = counter;			//adds the counter value to the array
            blinkRedAssembly(2);			//blinks the red LED once to show that the input has been accepted
            blinkYellowAssembly((counter*2));		//blinks the Yellow LED the number of times the button was pressed, to echo the input
            counter = 0;				//counter equals to 0 again to get the next value inputted
            timer = 0;
        }
        printf("  -----------------\n\n-----------------\nEnd of Round %d\n-----------------\n\n", roundNum+1);
        blinkRedAssembly(4);				//Red LED blinks twice at the end of the users guess

        int exact = 0;
        int color = 0;
        for(int colorI = 0; colorI<sequenceLength; colorI++) {		//loops in both arrays and compares values at the same index
            if(colors[colorI] == secret[colorI]) {			//checks if the values are the same
                exact++;						//increments exact counter
                secret[colorI] = maxColors+1;				//changes the secret value to a number that will not be used in the secret so that the comparison doesn't get affected; the results are correct
                colors[colorI] = maxColors+2;				//changes the color value in the array
            }
        }

        for(int colorI = 0; colorI < sequenceLength; colorI++) {				//loops through color array
            for(int secretI = 0; secretI < sequenceLength; secretI++) {			//loops through secret array
                if(colors[colorI] == secret[secretI]) {					//checks if the values are the same
                    color++;								//increments the color counter if the values are the same
                    secret[secretI] = maxColors+1;					//changes the value to avoid incorrect results
                    break;
                }							//once it has been through the first loop then the color index is incremented and the process repeats
            }
        }
        printf("Exact Matches: %d\n", exact);			//prints exact matches on the terminal
        printf("Color Matches: %d\n", color);			//prints colour matches on the terminal

        lcdClear(lcd) ;						//clears the LCD to allow data to be displayed

        char message1[16];					//array to hold the integer as characters
        char message2[16];
        sprintf(message1, "Exact: %d", exact);			//returns the formatted string
        sprintf(message2, "Color: %d", color);

        lcdPosition (lcd, 0, 0) ;
        lcdPuts (lcd, message1) ;		//Displays the string on the LCD, at the positions specified
        lcdPosition (lcd, 0, 1) ;
        lcdPuts (lcd, message2) ;

        blinkYellowAssembly(exact*2);		//Yellow LED blinks the number of exact matches
        blinkRedAssembly(2);			//Red LED blinks once
        blinkYellowAssembly(color*2);		//Yellow LED blinks the number of colour matches



        roundNum++;				//roundNum values gets incremented
        if(exact != sequenceLength) {		//checks if the exact matches equals the length of the secret, because if it does not then the guess was incorrect
            blinkRedAssembly(6);		//blink Red LED 3 times to show end of round
            printf("\n-----------------\nStarting Round %d\n-----------------\n\n", roundNum+1);
            game(mainSecret, sequenceLength, maxColors, lcd, roundNum);		//starts the next round
        }
        else {					//if the exact matches is equal to the length then the guess is correct and game ends
            blinkRedAssembly(1);			//turns the LED on
            blinkYellowAssembly(6);			//blinks the Yellow LED 3 times
            blinkRedAssembly(2);			//Red LED blinks once to signal end of game
            printf("YOU WIN\n");
            lcdClear    (lcd) ;			//clears LCD for next game

            char attempts[16];
            sprintf(attempts, "Attempts: %d", roundNum);
            lcdPosition (lcd, 0, 0) ;
            lcdPuts (lcd, "SUCCESS") ;			//displays SUCCESS on the LED on the top row
            lcdPosition (lcd, 0, 1) ;
            lcdPuts (lcd, attempts) ;			//displays the number of attempts on the LCD on the second row
        }

    }
    else {
        printf("You're out of attempts\nGame Over\n");
    }


}
