/* 
 * File:   timer.c
 * Author: STECH.DEV.PC.06
 *
 * Created on April 17, 2024, 5:34 PM
 */

// PIC24FJ48GA002 Configuration Bit Settings

// 'C' source line config statements

// CONFIG2
#pragma config POSCMOD = HS             // Primary Oscillator Select (HS Oscillator mode selected)
#pragma config I2C1SEL = PRI            // I2C1 Pin Location Select (Use default SCL1/SDA1 pins)
#pragma config IOL1WAY = ON             // IOLOCK Protection (Once IOLOCK is set, cannot be changed)
#pragma config OSCIOFNC = OFF           // Primary Oscillator Output Function (OSC2/CLKO/RC15 functions as CLKO (FOSC/2))
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Clock switching and Fail-Safe Clock Monitor are disabled)
#pragma config FNOSC = FRCDIV           // Oscillator Select (Fast RC Oscillator with Postscaler (FRCDIV))
#pragma config SOSCSEL = SOSC           // Sec Oscillator Select (Default Secondary Oscillator (SOSC))
#pragma config WUTSEL = LEG             // Wake-up timer Select (Legacy Wake-up Timer)
#pragma config IESO = ON                // Internal External Switch Over Mode (IESO mode (Two-Speed Start-up) enabled)

// CONFIG1
#pragma config WDTPS = PS32768          // Watchdog Timer Postscaler (1:32,768)
#pragma config FWPSA = PR128            // WDT Prescaler (Prescaler ratio of 1:128)
#pragma config WINDIS = ON              // Watchdog Timer Window (Standard Watchdog Timer enabled,(Windowed-mode is disabled))
#pragma config FWDTEN = ON              // Watchdog Timer Enable (Watchdog Timer is enabled)
#pragma config ICS = PGx1               // Comm Channel Select (Emulator EMUC1/EMUD1 pins are shared with PGC1/PGD1)
#pragma config GWRP = OFF               // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF                // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = ON              // JTAG Port Enable (JTAG port is enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <p24Fxxxx.h>
#include <libpic30.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define _XTAL_FREQ 32000000 // Oscillator frequency set to 8MHz
#define FCY (_XTAL_FREQ/2)  // CPU Clock Cycle Frequency (Fcy = Fosc/2)
#define TCY (1.0/FCY)  // Th?i gian cho m?t chu k? l?nh

/*
 * 
 */
#define PIN_TRANFER_1           LATBbits.LATB1
#define PIN_TRANFER_2           LATBbits.LATB3
#define PIN_TRANFER_3           LATBbits.LATB4
#define PIN_TRANFER_4           LATBbits.LATB10

#define PIN_READ_1              PORTBbits.RB14
#define PIN_READ_2              PORTBbits.RB15
#define PIN_READ_3              PORTBbits.RB8
#define PIN_READ_4              PORTBbits.RB9
/*
 * 
 */
#define LED_PIN0 LATBbits.LATB0 //  LED pin 0
#define LED_PIN4 LATAbits.LATA4 //  LED pin 4
/*
 * 
 */
#define START_CONDITION     0b011111 // 2.5 Tbit:1, 0.5 Tbit = 0
//uint8_t byteToSend_pin1 = 0b11110101;//0b11111001;//0b11100101; // 3 bit ??u ???c hi?u là bit start(luôn có giá tr? là 1)
//uint8_t byteToSend_pin2 = 0b11110101;//0b11100101; // 3 bit ??u ???c hi?u là bit start(luôn có giá tr? là 1)
//uint8_t byteToSend_pin3 = 0b11100111;//0b11100101; // 3 bit ??u ???c hi?u là bit start(luôn có giá tr? là 1)
//uint8_t byteToSend_pin4 = 0b11110101;//0b11100101; // 3 bit ??u ???c hi?u là bit start(luôn có giá tr? là 1)

#define DELAY_BETWEEN_BITS 1 // Thay ??i giá tr? này ?? t?ng ho?c gi?m th?i gian ch?

#define T_BIT_US_FIX        (4000)  // Set this to the desired time in micro-seconds
#define TCKPS_VALUE         (64)   // Prescaler value
//#define T_BIT_US_FIX        (1000)//(455)
#define UNIT_COUNT_US       (160) //Each timeDelay is 5us
#define T_BIT_US           (T_BIT_US_FIX/UNIT_COUNT_US)
#define T_BIT_US_FOR_READ   (T_BIT_US/2)    //??c gi?a xung

#define DATA_LENGTH         4
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/
// Use for data receive
static uint8_t receivedByte = 0;
static uint8_t count = 0;        // ??m s? bit nh?n ???c
static uint8_t isHandling = 0;

enum{
    BIT_START_1,
    BIT_START_2,
    BIT_START_3,
    BIT_DATA,
};
//static uint8_t Step = BIT_START_1;

// Use for data transfer
uint32_t Timer_flag = 0;
uint8_t byteToSend = 0b11100101;//0b11100101; // 3 bit ??u ???c hi?u là bit start(luôn có giá tr? là 1)
//0b11100101 0b11111111
uint8_t stringToSend[DATA_LENGTH] = {0b11100101, 0b11000001, 0b11100101, 0b11110101}; // in debug theo thu tu 4 bit 1 tu phai sang trai: 0b11000001 = 0b1100 0001 ->0011 + 1000 (truyen ))
                                                                                      // in debug theo thu tu 4 bit 1 tu phai sang trai: 0b11000001 = 1000000011
uint8_t highNibble[DATA_LENGTH], lowNibble[DATA_LENGTH]; // Chuoi tach data
uint16_t dataLow[DATA_LENGTH];
uint16_t dataHight[DATA_LENGTH];
int bounce = 0;
int bitPos = 3; // B?t ??u t? bit cao nh?t
uint8_t tick_Byte = 0; // ghep 4 byte dau voi 4 byte sau

uint16_t ByteToSend = 0b11001100011111;    //2 bit t??ng ?ng 1 bit truy?n ?i(m?i bit là Tbits/2(us))

volatile int timeDelay = 0; // Bi?n ??m th?i gian ch?

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/**************************** Configuration timer *****************************/

/**
 * @func   TimerInit
 * @brief  Initialize Timer1 for send data
 * @param  None
 * @retval None
 */
void TimerInit(void);

/**
 * @func   TimerInit
 * @brief  Initialize Timer2 for create a delay each time retrieve data
 * @param  None
 * @retval None
 */
void Timer2Init(void);

/********************** Configuration UART for debug **************************/

/**
 * @func   UART1_Init
 * @brief  Initialize UART 1 for debug
 * @param  None
 * @retval None
 */
void UART1_Init(void);

/**
 * @func   UART1_Write
 * @brief  UART send data
 * @param  data
 * @retval None
 */
void UART1_Write(char data);

/**
 * @func   UART1_SendByte
 * @brief  UART send 1 byte data
 * @param  data
 * @retval None
 */
void UART1_SendByte(uint8_t data);

/**
 * @func   UART1_SendString
 * @brief  UART send string
 * @param  str
 * @retval None
 */
void UART1_SendString(const char *str);

/******************************** Send data ***********************************/

/**
 * @func   SendBit
 * @brief  Send bit on data pin
 * @param  bit: 0 / 1
 *         pin: Pin send data
 * @retval None
 */
void SendBit(uint8_t bit, uint32_t pin);

/**
 * @func   SendBitOnPin
 * @brief  Send bit in byte data on data pin
 * @param  byteToSend: byte data
 *         position: Position of bits in data bytes
 * @retval None
 */
void SendBitOnPin(uint8_t byteToSend, int position, uint32_t pinNumber);

/******************************** Blink led ***********************************/

/**
 * @func   BlinkLed
 * @brief  Blink led in pin RB0
 * @param  None
 * @retval None
 */
void BlinkLed(void);

/**
 * @func   BlinkLed4
 * @brief  Blink led in pin RA4
 * @param  None
 * @retval None
 */
void BlinkLed4(void);

/*************************** Spit data to send ********************************/

/**
 * @func   splitByte
 * @brief  Divide the data byte into 2 parts to send
 * @param  byte: Byte data
 *         highNibble : Bit 7 -> Bit 4
 *         lowNibble  : Bit 3 -> Bit 0
 * @retval None
 */
void splitByte(uint8_t byte, uint8_t *highNibble, uint8_t *lowNibble);

/**
 * @func   doubleBitsAndReverse
 * @brief  Converts 4 bits of data into sent data including the start byte (Pin1 send first)
 * @param  x : Data to convert
 * @retval Data convert
 */
uint16_t doubleBitsAndReverse(uint8_t x);

/**
 * @func   doubleBits
 * @brief  Converts 4 bits of data into sent data including the start byte (Pin4 send first)
 * @param  x : Data to convert
 * @retval Data convert
 */
uint16_t doubleBits(uint8_t x);

/******************************** Delay ***************************************/

/**
 * @func   delay_ms
 * @brief  Delay ms
 * @param  ms
 * @retval None
 */
void delay_ms(uint32_t ms);

/******************************* Receive data *********************************/

/**
 * @func   receiveBit
 * @brief  Receive data on pin
 * @param  pinId : Pin receive data
 * @retval value of bit (0 / 1)
 */
uint8_t receiveBit(uint8_t pinId);

/**
 * @func   pollReceiveBit
 * @brief  Data receiving process
 * @param  None
 * @retval None
 */
void pollReceiveBit(void);

/******************************************************************************/
/*                                 MAIN FUNCTION                              */
/******************************************************************************/

int main(void)
{
    // C?u hình chân ??u vào RB8
    TRISBbits.TRISB14 = 1;
    TRISBbits.TRISB15 = 1;
    TRISBbits.TRISB8 = 1;
    TRISBbits.TRISB9 = 1;
    
    CNPU2bits.CN22PUE = 1;
    CNPU2bits.CN21PUE = 1;
    
    CNPU1bits.CN12PUE = 1;
    CNPU1bits.CN11PUE = 1;
    
    AD1PCFGbits.PCFG10 = 1;
    AD1PCFGbits.PCFG9 = 1;
    
    // C?u hình chân ??u ra RB1
    TRISBbits.TRISB1 = 0;
    TRISBbits.TRISB3 = 0;
    TRISBbits.TRISB4 = 0;
    TRISBbits.TRISB10 = 0;

    // C?u hình chân LED RB0 và RA4 là ??u ra ?? debug
    TRISBbits.TRISB0 = 0;
    TRISAbits.TRISA4 = 0;
    
    UART1_Init();
    UART1_SendString("Hello, UART !\r");

//    delay_ms(500);
//    UART1_SendString("Hello, UART! 1\n");
    SendBit(0, 1);
    SendBit(0, 2);
    SendBit(0, 3);
    SendBit(0, 4);
    delay_ms(500);
    UART1_SendString("Hello, UART 2!\r"); 
    
    // Goi ham tach byte
    int i= 0;
    for (i=0; i< DATA_LENGTH; i++)
    {
        splitByte(stringToSend[i], &highNibble[i], &lowNibble[i]); 
        dataHight[i] = doubleBitsAndReverse(highNibble[i]);
        dataLow[i] = doubleBitsAndReverse(lowNibble[i]);
//        dataHight[i] = doubleBits(highNibble[i]);
//        dataLow[i] = doubleBits(lowNibble[i]);
    }

    
//    UART1_SendString("\n");
//   uint16_t numberToSend = 0xABCD;      // Ví d? m?t s? 16-bit c?n g?i
//   UART_SendUint16(numberToSend);       // G?i hàm ?? g?i giá tr? s? này
//    UART1_SendByte(highNibble);
//    UART1_SendString((const uint8_t *)"1");
    
    // Initialization timer
    TimerInit();
    Timer2Init();
    while (1)
    {
//            ByteToSend = doubleBitsAndReverse(highNibble);

        pollReceiveBit(); // ?
    }

    return 0;
}

/**
 * @func   TimerInit
 * @brief  Initialize Timer1 for send data (Tbit = 455us)
 * @param  None
 * @retval None
 */
void TimerInit(void)
{
    // Configuration Timer1
    T1CONbits.TON = 0;      // Turn off Timer1
    T1CONbits.TCS = 0;      // Use internal pulse source
    T1CONbits.TCKPS = 0b10; // Prescaler: 64
    PR1 = (T_BIT_US_FIX/32)-1;//(T_BIT_US_FIX/16)-1;//27;               // Set timer counter
//    PR1 = (T_BIT_US_FIX/16)-1;
    T1CONbits.TON = 1;      // Turn on Timer1

    // Configuration  Timer1 interrupt 
    IFS0bits.T1IF = 0;    // Clear Timer1 interrupt
    IEC0bits.T1IE = 1;    // Set Timer1 interrupt
    IPC0bits.T1IP = 1;    // Set Timer1 interrupt priority
}

/**
 * @func   TimerInit
 * @brief  Initialize Timer2 for create a delay each time retrieve data 20 us
 * @param  None
 * @retval None
 */
void Timer2Init(void)
{
    T2CONbits.TON = 0;      // T?t Timer2 trong quá trình c?u hình
    T2CONbits.TCS = 0;      // Use internal pulse source
    T2CONbits.TCKPS = 0b01; // Prescaler: 8
    PR2 = (UNIT_COUNT_US/2)-1;//9;         // tao delay
//    PR2 = (UNIT_COUNT_US/1)-1;

    T2CONbits.TON = 1;      // B?t Timer2    

    // Configuration  Timer1 interrupt 
    IFS0bits.T2IF = 0;    // Clear Timer2 interrupt
    IEC0bits.T2IE = 1;    // Set Timer2 interrupt
    IPC1bits.T2IP = 1;    // Set Timer2 interrupt priority
}
/**
 * @func   UART1_Init
 * @brief  Initialize UART 1 for debug
 * @param  None
 * @retval None
 */
void UART1_Init(void) 
{
    /**************************************************************************/
    /*                     Configuration pins use for UART1                   */
    /**************************************************************************/
    
    // Configure pin RB13 as output for TX
    TRISBbits.TRISB13 = 0;

    // Configure pin RB12 as input for RX
    TRISBbits.TRISB12 = 1;

    // Map U1TX function to pin RB13 (assuming RB13 is RP13) (Data sheet 109)
    RPOR6bits.RP13R = 3; // '3' is the remappable peripheral pin select opcode for U1TX

    // Map U1RX function to pin RB12 (assuming RB12 is RP12)
    RPINR18bits.U1RXR = 12; // '12' is the RP number where U1RX is mapped to
    
    /**************************************************************************/
    /*                          Configuration UART1                           */
    /**************************************************************************/
    
    // Assuming BRGH=0 (Standard Speed mode), calculate U1BRG
    // Set Baud Rate Generator based on the Fcy (Remember to replace 'x' with your value)
//    U1BRG = ((_XTAL_FREQ / 2) / (16 * 9600)) - 1; // Example calculation for baud rate 9600
    U1BRG = 12;//25;
    
    // Set UART1 TX interrupt priority to level 1
//    IPC3bits.U1TXIP = 1;  
    // Set UART1 RX interrupt priority to level 1
    IPC2bits.U1RXIP = 1;  
    
    // Clear UART1 Status and Control register
    U1STA = 0;
    
    // Enable UART1 module, 8-bit data, no parity, 1 stop bit
    U1MODE = 0x8000;     

    // Enable UART1 TX (transmit)
    U1STAbits.UTXEN = 1;
    U1STAbits.URXISEL = 0;   // Enable Receive

    // Disable UART1 TX interrupt
    IEC0bits.U1TXIE = 0; 
    // Enable UART1 RX interrupt
    IEC0bits.U1RXIE = 1; 
}

/**
 * @func   UART1_Write
 * @brief  UART send data
 * @param  data
 * @retval None
 */
void UART1_Write(char data)
{
    // Wait for the transmit buffer to be empty before sending data.
    while(!U1STAbits.TRMT);  // TRMT is Transmit Shift Register Empty bit.
    
    U1TXREG = data;          // Load the transmission register with the data to be sent.
}

/**
 * @func   UART1_SendByte
 * @brief  UART send 1 byte data
 * @param  data
 * @retval None
 */
void UART1_SendByte(uint8_t data) {
    // Ch? cho ??n khi thanh ghi truy?n s?n sàng
    while (!U1STAbits.TRMT);

    // G?i byte d? li?u
    U1TXREG = data;
}

/**
 * @func   UART1_SendString
 * @brief  UART send string
 * @param  str
 * @retval None
 */
void UART1_SendString(const char *str) {
    while(*str != '\0') {  // Loop through the string and send each character
        UART1_Write(*str); // Send current character from string via UART2
        str++;             // Increment pointer to the next character
    }
    UART1_Write('\0');
//    UART1_Write('\r'); // Send carriage return
//    UART1_Write('\n'); // Send newline character to move to the
}

/**
 * @func   SendBit
 * @brief  Send bit on data pin
 * @param  bit: 0 / 1
 *         pin: Pin send data
 * @retval None
 */
void SendBit(uint8_t bit, uint32_t pin)
{
    switch (pin) {
        case 1:
            PIN_TRANFER_1 = (bit == 0) ?  0: 1;
            break;
        case 2:
            PIN_TRANFER_2 = (bit == 0) ?  0: 1;
            break;
        case 3:
            PIN_TRANFER_3 = (bit == 0) ?  0: 1;
            break;
        case 4:
            PIN_TRANFER_4 = (bit == 0) ?  0: 1;
            break;
        default:
            break;
    }
}

/**
 * @func   SendBitOnPin
 * @brief  Send bit in byte data on data pin
 * @param  byteToSend: byte data
 *         position: Position of bits in data bytes
 * @retval None
 */
void SendBitOnPin(uint8_t byteToSend, int position, uint32_t pinNumber) {
    if(byteToSend & (1 << position)) 
    {
        SendBit(1, pinNumber);
    } 
    else 
    {
        SendBit(0, pinNumber);
    }
}

/**
 * @func   BlinkLed
 * @brief  Blink led in pin RB0
 * @param  None
 * @retval None
 */
void BlinkLed(void)
{
    LED_PIN0 = !LED_PIN0;
    delay_ms(100);
    LED_PIN0 = !LED_PIN0;
    delay_ms(100);
}

/**
 * @func   BlinkLed4
 * @brief  Blink led in pin RA4
 * @param  None
 * @retval None
 */
void BlinkLed4(void)
{
    LED_PIN4 = !LED_PIN4;
    delay_ms(100);
    LED_PIN4 = !LED_PIN4;
    delay_ms(100);
}

/**
 * @func   doubleBitsAndReverse
 * @brief  Converts 4 bits of data into sent data including the start byte
 * @param  x : Data to convert
 * @retval Data convert
 */
uint16_t doubleBitsAndReverse(uint8_t x) {
    uint16_t ketQua = 0;
    int doDaiBit = sizeof(uint32_t); 
    int i = 0;
    for ( i = 0; i < doDaiBit; i++) {
        // Lay bit hien tai (phai nhat) bang cach & voi 1
        uint16_t bitHienTai = (x >> i) & 1;
        // Nhân ?ôi bit và d?ch sang trái t?i v? trí ng??c l?i
        // Nhan doi bit va dich sang trai tai vi tri nguoc lai
        // V? trí ng??c ???c tính b?ng cách l?y ?? dài bit g?p ?ôi tr? ?i 2*i và tr? thêm 1 ho?c 2 tùy thu?c vào v? trí bit c?n nhân ?ôi      
        ketQua |= bitHienTai << ((doDaiBit - i - 1) * 2);
        ketQua |= bitHienTai << ((doDaiBit - i - 1) * 2 + 1);
    }
    ketQua = (ketQua << 6) | START_CONDITION;
    return ketQua;
}

/**
 * @func   doubleBits
 * @brief  Converts 4 bits of data into sent data including the start byte (Pin4 send first)
 * @param  x : Data to convert
 * @retval Data convert
 */
uint16_t doubleBits(uint8_t x) {
    uint16_t result = 0;
    int bitLength = 8; // Vì x là uint8_t nên có 8 bit

    int i = 0;
    for ( i = 0; i < bitLength; i++) {
        // L?y bit hi?n t?i (ph?i nh?t)
        uint16_t currentBit = (x >> i) & 1;
        // Nhân ?ôi bit và d?ch ??n v? trí thích h?p
        result |= currentBit << (i * 2);
        result |= currentBit << (i * 2 + 1);
    }
    // Chèn START_CONDITION vào nh?ng bit ??u tiên
    result = (result) | (START_CONDITION << 8);

    return result;
}

/**
 * @func   splitByte
 * @brief  Divide the data byte into 2 parts to send
 * @param  byte: Byte data
 *         highNibble : Bit 7 -> Bit 4
 *         lowNibble  : Bit 3 -> Bit 0
 * @retval None
 */
void splitByte(uint8_t byte, uint8_t *highNibble, uint8_t *lowNibble) {
    *highNibble = (byte & 0xF0) >> 4; // Lay 4 bit cao
    *lowNibble = byte & 0x0F;        // Lay 4 bit thap
}

/**
 * @func   delay_ms
 * @brief  Delay ms
 * @param  ms
 * @retval None
 */
void delay_ms(uint32_t ms)
{
    uint32_t i, j;
    
    for (i=0; i<=ms; i++)
    {
        for (j=0; j<=1000; j++);
    }
}

/**
 * @func   receiveBit
 * @brief  Receive data on pin
 * @param  pinId : Pin receive data
 * @retval value of bit (0 / 1)
 */
uint8_t receiveBit(uint8_t pinId)
{
    switch(pinId) 
    {
        case 1: return PIN_READ_1 & 0x01;
        case 2: return PIN_READ_2 & 0x01;
        case 3: return PIN_READ_3 & 0x01;
        case 4: return PIN_READ_4 & 0x01;
        default: return 0;
    }
}

/**
 * @func   pollReceiveBit
 * @brief  Data receiving process
 * @param  None
 * @retval None
 */
void pollReceiveBit(void) 
{
    uint8_t receivedBit = 0; // G?i hàm ?? ??c bit t? chân nh?n bit
    if(isHandling == 0)
    {
        if (receiveBit(4) && receiveBit(3) && receiveBit(2) && receiveBit(1))  //Ch? khi chân 1 start thì ?o
        {
            timeDelay = 0;
            isHandling = 1;
            receivedByte = 0;
        }
    }
    else if (isHandling == 1)
    {
        if (timeDelay == T_BIT_US_FOR_READ)
        {
            // Bit first
            if (!(receiveBit(4) && receiveBit(3) && receiveBit(2) && receiveBit(1))){
                //N?u b?t c? kênh nào != 1 thì l?i
                count = 0;
                isHandling = 0;
                UART1_SendString("a");
            } else {
                count = 1;
            }
        }
        else if (count >= 1)
        {
            if( count == 1 ){
                if (timeDelay == T_BIT_US_FOR_READ + count * T_BIT_US){
                    if (!(receiveBit(4) && receiveBit(3) && receiveBit(2) && receiveBit(1))){
                        //N?u b?t c? kênh nào != 1 thì l?i
                        count = 0;
                        isHandling = 0;
                        UART1_SendString("y");
                    } else {
                        count = 2;  //Next bit start
                    }
                }
            }
            
            if( count == 2 ){
                //??c n?a bit ??u start giá tr? 4 kênh ??u là 1
                if (timeDelay == T_BIT_US_FOR_READ/3 + count * T_BIT_US){
                    if (!(receiveBit(4) && receiveBit(3) && receiveBit(2) && receiveBit(1))){
                        //N?u b?t c? kênh nào != 1 thì l?i
                        count = 0;
                        isHandling = 0;
//                        UART1_SendString("x");
                    }
                }
                //N?a bit sau start giá tr? là 0
                if (timeDelay == (T_BIT_US_FOR_READ*4)/3 + count * T_BIT_US){
                    if (!(receiveBit(4) == 0 && receiveBit(3) == 0 && receiveBit(2) == 0 && receiveBit(1) == 0)){
                        //N?u b?t c? kênh nào != 0 thì l?i
                        count = 0;
                        isHandling = 0;
                        UART1_SendString("k");
                    } else {
                        receivedByte = 0;
                        count = 3;
                    }
                }
            }
            if( count >= 3 ){
//                receivedBit = receiveBit(count-2);
                if (count == 3) receivedBit = receiveBit(4);
                if (count == 4) receivedBit = receiveBit(3);
                if (count == 5) receivedBit = receiveBit(2);
                if (count == 6) receivedBit = receiveBit(1);
                
                if (timeDelay == T_BIT_US_FOR_READ + count * T_BIT_US){
                    receivedByte= (receivedByte << 1) | receivedBit;
                    count ++;
//                    UART1_SendString("a");
                }                    
            }
        }
        
        if (count > 6)
        {
//            UART1_SendString("c");
            count = 0;
            tick_Byte ++;
            isHandling = 0;
//            if (receivedByte == )
            int i = 0;
            for(i = 0; i < 4; i++){
                if( receivedByte & (0x01<<i) ){
                    UART1_SendString("1");
                } else {
                    UART1_SendString("0");
                }
            }

            if (tick_Byte == 2)
            {
//                UART1_SendString("\r");
                tick_Byte = 0;

            }

        }
    }
}

/**
 * @func   _T2Interrupt
 * @brief  Interrupt for create a delay each time retrieve data
 * @param  None
 * @retval None
 */
void __attribute__((interrupt, auto_psv)) _T2Interrupt(void)
{
    timeDelay ++;
    
    // Clear Timer interrupt flag
    IFS0bits.T2IF = 0;
}

int16_t bitCount = 0;
//int bounce = 0;
//uint16_t ByteToSend = 0;    //2 bit t??ng ?ng 1 bit truy?n ?i(m?i bit là Tbits/2(us))

//uint16_t ByteToSend = 0b11001100011111;    //2 bit t??ng ?ng 1 bit truy?n ?i(m?i bit là Tbits/2(us))

/**
 * @func   _T1Interrupt
 * @brief  Interrupt for Send data
 * @param  None
 * @retval None
 */
void __attribute__((interrupt, auto_psv)) _T1Interrupt(void)
{    
    // Send n byte
    if (!Timer_flag && bounce %2 == 0)
    {
        ByteToSend = dataHight[(bounce) / 2];
    }
    else
    {
        ByteToSend = dataLow[ (bounce-1) / 2];
    }
    
    // Send 1 byte 
//    if (!Timer_flag == 0)
//    {
//        ByteToSend = dataHight[0];
//    }
//    else
//    {
//        ByteToSend = dataLow[ 0];
//    }
   
    bitCount--;
    if( bitCount > 0 && bitCount <= 14 ){
        int8_t bitRealPos = (14-bitCount);
        int8_t bitToSend = (ByteToSend >> bitRealPos) & 0x01;
        if( bitCount <= 14 && bitCount >=9 ){   //6 bit ??u(9->14) là dành cho start bit
            //Start bit
            SendBit(bitToSend, 1); // Send to pin 1
            SendBit(bitToSend, 2);
            SendBit(bitToSend, 3);
            SendBit(bitToSend, 4);
        } else {
            //8 bit ti?p theo(1->8) là cho d? li?u
            if( bitRealPos % 2 == 0 ){
                // Clear all pins sau m?i chu k? Tbit
                SendBit(0, 1);
                SendBit(0, 2);
                SendBit(0, 3);
                SendBit(0, 4);
            }
//            uint8_t pin = 4 - ((bitCount-1)/2); //Tính toán chân can xuat du lieu (lan luot tung chan)
            uint8_t pin = ((bitCount+1)/2); //Tính toán chân c?n xu?t d? li?u(l?n l??t t?ng chân)
            
//            if (pin == 1)
//            {
//                UART1_SendString("1");
//            }
//            else if (pin == 2)
//            {
//                UART1_SendString("2");                
//            }
//            else if (pin == 3)
//            {
//                UART1_SendString("3");                
//            }
//            else if (pin == 4)
//            {
//                UART1_SendString("4");                
//            }
            SendBit(bitToSend, pin); // Send to pin 1
        }
//        if (bitToSend)
//        {
//            UART1_SendString("1");
//        }
//        else
//        {
//            UART1_SendString("0");
//        }
    }
    
    if (bitCount == 0){
//        UART1_SendString("\r");
        Timer_flag = ~ Timer_flag;
        
        bounce ++;
    }
    if(bounce > 7) bounce = 0;
    
    if(bitCount <= 0) {
        bitCount = 500; // Create a delay 
        // Clear all pins
        SendBit(0, 1);
        SendBit(0, 2);
        SendBit(0, 3);
        SendBit(0, 4);
    }
    
    // Clear Timer interrupt flag
    IFS0bits.T1IF = 0;
}

/* END FILE */