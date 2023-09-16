#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include <esp32/rom/crc.h>
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "esp_console.h"

//PORTS
//SIG100 CONNECTIONS
#define HDC                 33  //Wire: white, OUTPUT: Data/Command select input. When pulled down, the SIG100 enters command mode. SIG100 has internal PU resistor.
#define HDI                 17//32  //Wire: grey, OUTPUT: PLC Data Input from the host ECU. 17 = Tx pin. SIG100 has internal PU resistor.
#define HDO                 16//35  //Wire: black, INPUT:  PLC Data output to the host ECU. 16 = Rx pin
#define INH                 34  //Wire: black, INPUT:  From PLC, High = PLC normal mode, Low = PLC sleep mode. Used to signal to the ESP32 what the status of the SIG100 is.
#define NLOOPBACK           25  //Wire: blue OUTPUT: Enable loopback of HDI to HDO pin, ECU controlled. SIG100 has internal PU resistor.
#define NSLEEP              26  //Wire: purple OUTPUT: Port to put PLC into sleep mode. High = PLC normal mode, Low = PLC sleep/standby mode. Upon transition from low to high, WUM is transmitted over the powerline.
#define FREQ_SEL_1          14  //Wire: red, OUTPUT:
#define FREQ_SEL_0          12  //Wire: red, OUTPUT:
#define NAUTO_SLEEP         27  //Wire: green, OUTPUT:
#define N_AUTO_FREQ_CHANGE  13  //Wire: orange, OUTPUT:
#define NRESET              23  //Wire: yellow, OUTPUT: hard reset switch for SIG100

//LEDS 
#define LED_NLOOPBACK  5    //NLOOPBACK, orange
#define LED_NSLEEP  19      //NSLEEP, green
#define LED_INH  15         //INH, yellow

//64-bit GPIO PORT DECLARATION STRUCT CONSTANTS
#define GPIO_OUTPUT_PIN_SEL     ((1ULL<<HDC) | (1ULL<<NLOOPBACK) | (1ULL<<NSLEEP) | (1ULL<<NRESET) |(1ULL<<NAUTO_SLEEP) |(1ULL<<N_AUTO_FREQ_CHANGE) | (1ULL<<LED_NLOOPBACK) | (1ULL<<LED_NSLEEP) | (1ULL<<LED_INH))
#define GPIO_INPUT_PIN_SEL      (1ULL<<INH)
//#define GPIO_INPUT_PU_PIN_SEL   ((1ULL<<BUTTON1) | (1ULL<<BUTTON2) | (1ULL<<BUTTON3) | (1ULL<<BUTTON4) | (1ULL<<BUTTON5) | (1ULL<<BUTTON6))

//BAUDRATES
//SIG100 has 6 available baudrates to choose from
//During command mode SIG100 uses bitrate of host regardless
#define BAUD_9600   9600    //minimum speed
#define BAUD_10417  10417
#define BAUD_19200  19200   //default speed
#define BAUD_38400  38400
#define BAUD_57600  57600
#define BAUD_115200 115200  //max speed

#define BUFFERSIZE 1024     //uart rx buffer

//FIRST BYTE TO SEND TO SIG100 FOR REGISTER ACCESS
#define WRITE_REG_START_BYTE 0xF5   //First byte to send over uart to SIG100 to switch to register write mode
#define READ_REG_START_BYTE  0xFD   //First byte to send over uart to SIG100 to switch to register read mode

//STATE NAMES
#define NORMAL  0
#define SLEEP   1
//#define STANDBY 2

//VTASK DELAY VALUES
#define T10ms   10 / portTICK_PERIOD_MS
#define T50ms   50 / portTICK_PERIOD_MS
#define T100ms  100 / portTICK_PERIOD_MS
#define T250ms  250 / portTICK_PERIOD_MS
#define T500ms  500 / portTICK_PERIOD_MS
#define T1000ms 1000 / portTICK_PERIOD_MS

//REGISTER NAMES
#define DVC_CTRL_0  0x00        //Reg_0, default R0101010, R = READ ONLY
#define DVC_CTRL_1  0x01        //Reg_1, default 00011111
#define FREQ_MAIN   0x02        //Reg_2, default 01010000
#define FREQ_ALT_1  0x03        //Reg_3, default 00000000
#define FREQ_ALT_2  0x04        //Reg_4, default 10101010
#define ACTIVE_FREQ 0x05        //Reg_5, default RRRRRRRR
#define SLP_CTRL    0x06        //Reg_6, default 01111100

//REGISTER VALUE VARIABLES
uint8_t DvcCtrl0_Val = 0;       //Reg_0, default R0101010, R = READ ONLY
uint8_t DvcCtrl1_Val = 0;       //Reg_1, default 00011111
uint8_t FreqMain_Val = 0;       //Reg_2, default 01010000
uint8_t FreqAlt1_Val = 0;       //Reg_3, default 00000000
uint8_t FreqAlt2_Val = 0;       //Reg_4, default 10101010
uint8_t ActiveFreq_Val = 0;     //Reg_5, default RRRRRRRR
uint8_t SlpCtrl_Val = 0;        //Reg_6, default 01111100

//Tags for ESP32 printf-line function "ESP_LOGx()"
static const char *ESP = "ESP32";
                      

//Other Console Stuff
#define PROMPT_STR CONFIG_IDF_TARGET
#define CONFIG_CONSOLE_MAX_COMMAND_LINE_LENGTH 0

//build test message
char DATA[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ\
abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ\
abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ\
abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ\
abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ\
!";

    char HEADER[3] = {(sizeof(DATA) - 1)/100 + '0', ((sizeof(DATA) - 1)/10)%10 + '0', (sizeof(DATA) - 1)%10 + '0'}; //converts the 3-digit length of TEST_MSG to a char array, only works for 3 digits or less
    uint8_t TEST_MSG[sizeof(HEADER) + sizeof(DATA) - 1];    //Full message frame (header + data), dont include null char in DATA
    
    
    char *CRCrecvptr;                           //CRC functions expect a pointer
//



//Function declarations
void ToggleLoopBack(void);

void ChangeSleepMode(uint8_t Mode);

void SleepViaNSleep(void);

void SleepViaReg(void);

void WakeUpViaNSleep(void);

void WakeUpViaHDC(void);

void AlternateFreq(void);

uint8_t ReadReg(uint8_t Reg);

void WriteReg(uint8_t Reg, uint8_t Data);

void Delay100ns(void);

uint32_t CountOnes(uint32_t num);

void ConfigGPIO(uint64_t pin_bit_mask, gpio_mode_t mode,  gpio_pullup_t pull_up_en,  gpio_pulldown_t pull_down_en,  gpio_int_type_t intr_type);

void ConfigUART(uint32_t baudrate);

void Setup(void);

void register_send_message(void);

void register_toggleloopback(void);

void register_sleepviansleep(void);

void register_readreg(void);

void register_AltFreq(void);

void register_Reset(void);


void register_button(void)
{
    register_send_message();
    register_toggleloopback();
    register_sleepviansleep();
    register_readreg();
    register_AltFreq();
    register_Reset();
}


//*********Function definitions*********



//--------------ConfigUART--------------
// Configures the UART between the ESP and the SIG100
// Uses the first argument as baudrate
// 8-N-1 encoding required by the SIG100
// This function is run by Setup()
// Input: Baudrate
// Output: none
void ConfigUART(uint32_t baudrate)
{   //todo: add functionality to take baudrate as argument
    uart_config_t SIG100_UART_CONFIG = 
    {                                           //All inputs other than .baud_rate should be predefined macros 
        .baud_rate  = baudrate,                 //baudrate
        .data_bits  = UART_DATA_8_BITS,         //word length: 8bits 
        .parity     = UART_PARITY_DISABLE,      //Disable UART parity 
        .stop_bits  = UART_STOP_BITS_1,         //stop bit: 1bit 
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE, //disable hardware flow control 
        .source_clk = UART_SCLK_APB,            //UART source clock is APB CLK = 80mhz in all cases
    };
    //int intr_alloc_flags = ESP_INTR_FLAG_INTRDISABLED;  //addhere
    int intr_alloc_flags = ESP_INTR_FLAG_INTRDISABLED;  //addhere

#if CONFIG_UART_ISR_IN_IRAM                             
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;              //addhere
#endif
    ESP_LOGI(ESP, "intr_alloc_flags: %u", intr_alloc_flags);

                                                            //Install driver on UART2
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2,         //Using Uart 2 of the ESP32 for the SIG100 connection. Arg: unsigned int
                                        BUFFERSIZE*2,       //Rx buffer size, buffer set to double max frame size = 2048. Arg: int
                                                            //Rx_buffer_size should be greater than UART_FIFO_LEN
                                        0,                  //Tx buffer size, set to 0. Arg: int
                                                            //Tx_buffer_size should be either zero or greater than UART_FIFO_LEN.
                                        0,                  //UART event queue size/depth. Arg: int
                                        NULL,               //UART event queue handle (out param). 
                                                            //On success, a new queue handle is written here to provide access to UART events. 
                                                            //If set to NULL, driver will not use an event queue. 
                                        0)); //Flags used to allocate the interrupt. 
                                                            //One or multiple (ORred) ESP_INTR_FLAG_* values. 
                                                            //See esp_intr_alloc.h for more info. Do not set ESP_INTR_FLAG_IRAM here (the driver’s ISR handler is not located in IRAM)

                                                            //Configure UART2
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2,           //Using UART 2 of the ESP32 for the SIG100 connection. Arg: unsigned int
                                    &SIG100_UART_CONFIG));  //Pass the uart_config_t struct from the beginning of the ConfigUART

                                                        //Set pins for UART2
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2,            //Using UART 2 of the ESP32 for the SIG100 connection. Arg: unsigned int
                                HDI,                    //UART TX pin GPIO number for ESP32
                                HDO,                    //UART RX pin GPIO number for ESP32
                                UART_PIN_NO_CHANGE,     //UART RTS pin GPIO number for ESP32
                                UART_PIN_NO_CHANGE));   //UART CTS pin GPIO number for ESP32
                                                        //UART_PIN_NO_CHANGE holds the value (-1), useful for reconfiguring UART
                                                        //In this case used to NOT set RTS and CTS, as they are NOT used for SIG100 UART
}

//--------------ConfigGPIO--------------
// Configure IO pins of the ESP32
// Pins can be set in groups if they have the same configuration
// All inputs other than pin_bit_mask should be predefined macros 
// which are found in the gpio header
// Input1: 64 bit unsigned int with bit mask of pins to be configured
// Input2: IO mode
// Input3: Internal pullup resistor behavior
// Input4: Internal pulldown resistor behavior
// Input5: Interrupt type for the pins
// Output: none
void ConfigGPIO(uint64_t pin_bit_mask, gpio_mode_t mode,  gpio_pullup_t pull_up_en,  gpio_pulldown_t pull_down_en,  gpio_int_type_t intr_type)
{
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = pin_bit_mask;
    //set IO mode
    io_conf.mode = mode;
    //Set pull-down mode
    io_conf.pull_down_en = pull_down_en;
    //Set pull-up mode
    io_conf.pull_up_en = pull_up_en;
    //Set interrupt
    io_conf.intr_type = intr_type;
    //configure GPIO with the given settings
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

//--------------CountOnes--------------
// Counts how many bits are '1' in a number
// Used to parse CRC result
// Input: 32-bits or smaller integer to be parsed
// Output: 32 bit integer with number of '1' bits
uint32_t CountOnes(uint32_t num)
{
    uint32_t count = 0;
    while(num)
    {
        count += num & 1;
        num>>=1;
    }
    return count;
}

//--------------Delay100ns--------------
// The SIG100 requires a minimum 100ns delay after several events
// ESP32 minimum timer delay is 50us, so this function is used instead
// Input: none
// Output: none
void Delay100ns(void) 
{                   //delay for 100ns @ 240mhz = 24 clock cycles
    asm (           //do 'nop' to waste clock cycles
    "nop \n" //1
    "nop \n" //2
    "nop \n" //3
    "nop \n" //4
    "nop \n" //5
    "nop \n" //6
    "nop \n" //7
    "nop \n" //8
    "nop \n" //9
    "nop \n" //10
    "nop \n" //11
    "nop \n" //12
    "nop \n" //13
    "nop \n" //14
    "nop \n" //15
    "nop \n" //16
    "nop \n" //17
    "nop \n" //18
    "nop \n" //19
    "nop \n" //20
    "nop \n" //21
    "nop \n" //22
    "nop \n" //23
    "nop \n" //24
    );
}

//---------------WriteReg---------------
// Write to a SIG100 internal register
// Registers are numbered 0 through 6
// This function assumes data value has been sanitized by first
// reading the register value then changing only the bits necessary
// Input 1: 8-bit unsigned int representing register number
// Input 2: 8-bit unsigned int with the new register value
// Output: none
void WriteReg(uint8_t RegNum, uint8_t Data)
{ 

    char RegStr [3] = {WRITE_REG_START_BYTE, RegNum, Data};
    gpio_set_level(HDC, 0);   //Lower HDC pin
    Delay100ns();
    uart_write_bytes(UART_NUM_2, (const char*)RegStr, sizeof(RegStr));                  //!!!Tx data to SIG100!!!
    Delay100ns();
    //more delay
    gpio_set_level(HDC, 1);   //Raise HDC pin
    if(RegNum == 0x02)        //Wait 1ms minimum after 'Main Frequency Select' register configuration, 
    {                         //Current frequency is auto set to main frequency
        vTaskDelay(T10ms);        //minimum wait for vTaskDelay is 10ms
    }

    ESP_LOGI(ESP, "RegNum:%X Data:%X ", RegNum, Data);
}

//---------------ReadReg----------------
// Read the value of a SIG100 internal register
// Registers are numbered 0 through 6
// Input: 8-bit unsigned int representing register number
// Output: 8-bit unsigned int with the register value
uint8_t ReadReg(uint8_t RegNum)
{
    char RegStr[2] = {READ_REG_START_BYTE, RegNum}; //convert int to char array for uart_write_bytes
    int rxLength = 0;
    uint8_t Data[1] = {0};    //holds rx data

    gpio_set_level(HDC, 0);   //lower HDC pin
    Delay100ns();
    uart_write_bytes(UART_NUM_2, (const char*)RegStr, sizeof(RegStr));                  //!!!Tx data to SIG100!!!
    do                                                                                  //wait for rx data 
    {
        vTaskDelay(T10ms);
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, (size_t*)&rxLength));    //Check for data in UART rx buffer
    }
    while(!rxLength);                                                                                                                     
    rxLength = uart_read_bytes(UART_NUM_2, Data, 1, 20 / portTICK_PERIOD_MS);           //read rx data
    Delay100ns();
    gpio_set_level(HDC, 1);                                                             //raise HDC pin
    ESP_LOGI(ESP, "Reg #%d value read: 0x%02X", RegNum, Data[0]);                           //print rx data
    //put function here to print data as binary digits to make it more readable

    return Data[0];                                                                     //return rx data
}

//-------------AlternateFreq------------
// Signal the SIG100 to change the PLC communication frequency
// by changing the values of the FREQ_SEL[1:0] pins.
// This function will cycle through the main frequency and two alternate frequencies
// Designed for a button interface
// Input: none
// Output: none
void AlternateFreq(void)
{
    static uint8_t counter = 0;

    switch(counter)
    {
        case 0:
        {   //freq_sel[1:0] = 01 = 1st alternate frequency
            gpio_set_level(FREQ_SEL_1, 0);
            gpio_set_level(FREQ_SEL_0, 1);          
            ESP_LOGI(ESP, "Freq alt 1 selected");
            counter++;
            break;
        }
        case 1:
        {   //freq_sel[1:0] = 10 = 2nd alternate frequency
            gpio_set_level(FREQ_SEL_1, 1);  
            gpio_set_level(FREQ_SEL_0, 0);          
            ESP_LOGI(ESP, "Freq alt 2 selected");
            counter++;
            break;
        }
        case 2:
        {   //freq_sel[1:0] = 00 or 11 = main frequency
            gpio_set_level(FREQ_SEL_1, 0);  
            gpio_set_level(FREQ_SEL_0, 0);          
            ESP_LOGI(ESP, "Freq main selected");
            counter = 0;
            break;
        }
    }
}

//-------------WakeUpViaHDC-------------
// Signal the SIG100 to sleep by doing a 
// HIGH-LOW-HIGH sequence with the HDC pin
// Use this when the nsleep pin is not connected
// Does not broadcast a WUM
// Input: none
// Output: none
void WakeUpViaHDC(void)         //need to test
{                               
    uint8_t INH_Check;             
                                //HDC will be high normally, only need to set LOW then HIGH
    gpio_set_level(HDC, 0);     //do HDC HIGH-LOW-HIGH transition with 100ns delay
    Delay100ns();               
    gpio_set_level(HDC, 1); 
    Delay100ns();

    do  //wait for INH pin to go high before returning
    {
        INH_Check = gpio_get_level(INH);
    }
    while(!INH_Check);  
    gpio_set_level(LED_NSLEEP, 0);
    ESP_LOGI(ESP, "SIG100 woken up via HDC.");
}

//------------WakeUpViaNSleep-----------
// Signal the SIG100 to wake up by setting the nsleep pin high
// If used while Sig100 in sleep mode, the SIG100 will broadcast
// a Wake Up Message on the PLC line to wake up all connected PLC devices.
// In standby mode the WUM will not broadcast 
// Input: none
// Output: none
void WakeUpViaNSleep(void)
{
    uint8_t Pin_Check;

    gpio_set_level(NSLEEP, 1);  // Set the nsleep pin high, this will cause the INH pin to go high
                                
    do  //wait for INH to go high before returning
    {
        Pin_Check = gpio_get_level(INH);                                
    }
    while(!Pin_Check);          

    do  //additionally wait for WUM to finish broadcasting before returning (wait for HDO pin high)
    {
        Pin_Check = gpio_get_level(HDO);
    }
    while(!Pin_Check);          
    gpio_set_level(LED_NSLEEP, 0);
    ESP_LOGI(ESP, "SIG100 woken up via NSleep.");
}

//-------------SleepViaReg-------------
// Signal the SIG100 to sleep by writing to the sleep control register
// Use this when nsleep pin is not connected
// Input: none
// Output: none
void SleepViaReg(void)                          //need to test
{                                               
    uint8_t INH_Check;

    SlpCtrl_Val = ReadReg(SLP_CTRL);            //get updated reg value   
    WriteReg(SLP_CTRL, (SlpCtrl_Val | 0x80));   //set bit 7 to '1' to send SIG100 sleep, bit 7 auto resets to 0 within Sig100, this will cause INH pin to go low
    do                                          //wait for INH to go low before returning
    {
        INH_Check = gpio_get_level(INH);
    }
    while(INH_Check);                           
    gpio_set_level(LED_NSLEEP, 1);
    ESP_LOGI(ESP, "SIG100 sent to sleep via Sleep Control register");
}

//------------SleepViaNSleep------------
// Signal the SIG100 to sleep by setting the nsleep pin low
// Input: none
// Output: none
void SleepViaNSleep(void)
{
    uint8_t INH_Check;

    gpio_set_level(NSLEEP, 0);      //sleep via nsleep pin, this will cause INH pin to go low when SIG100 sleeps
    do
    {
        INH_Check = gpio_get_level(INH);
    }
    while(INH_Check);               //wait for INH pin to go low before returning
    gpio_set_level(LED_NSLEEP, 1);        
    ESP_LOGI(ESP, "SIG100 sent to sleep via NSLEEP.");
}

//------------ChangeSleepMode-----------
// Signal the SIG100 to change its sleep mode
// The SIG100 has four different sleed modes
// The default is 'Enhanced Sleep Mode'
// SIG100 manual claims this mode is 'Best detection in a noisy environment'
// Input: 8-bit unsigned int with value 0, 1, 2 or 3, each representing a mode. 
// Output: none
void ChangeSleepMode(uint8_t Mode)  //haven't tested, test last
{
    SlpCtrl_Val = ReadReg(SLP_CTRL);//get current register value
    SlpCtrl_Val &= 0b11111100;      //clear last 2 bits
    SlpCtrl_Val |= Mode;            //update last 2 bits with Mode value 

    switch(Mode)                    //There are 4 sleep modes
    {
        case(0):                    //Enhanced Sleep mode
        {                           //Wake-up detection with-in 64mSec. Best detection in a noisy environment.
            WriteReg(SLP_CTRL, SlpCtrl_Val);
            ESP_LOGI(ESP, "SIG100 Sleep mode changed to Enhanced Sleep");
            break;
        }
        case(1):                    //Fast wake-up mode
        {                           //Fast wake-up detection with-in 250uSec.
            WriteReg(SLP_CTRL, SlpCtrl_Val);
            ESP_LOGI(ESP, "SIG100 Sleep mode changed to Fast wake-up");
            break;
        }
        case(2):                    //Low-power mode
        {                           //Wake-up Detection with-in 64mSec.
            WriteReg(SLP_CTRL, SlpCtrl_Val);
            ESP_LOGI(ESP, "SIG100 Sleep mode changed to Low-power");
            break;
        }
        case(3):                    //Deep sleep mode
        {                           //No bus wake-up detection.
            WriteReg(SLP_CTRL, SlpCtrl_Val);
            ESP_LOGI(ESP, "SIG100 Sleep mode changed to Deep Sleep");
            break;
        }
        default: ESP_LOGW(ESP, "Invalid sleep mode.");
    }
}

//------------ToggleLoopBack------------
// Signal the SIG100 to toggle loopback by changing the value of the nloopback pin
// The SIG100 can loop back the data received on the HDI pin to the HDO pin
// When enabled the ESP32 will receive a copy of the data it sends over UART to the SIG100
// This function will toggle between enabled and disabled
// Designed for a button interface
// Input: none
// Output: none
void ToggleLoopBack(void)   //Loopback between HDI and HDO pins on the Sig100, device transmits to self, working
{//future: check loopback register value before toggle, add in toggle via reg
    static uint8_t toggle = 0;          //counter variable

    if(toggle)                          //if toggle is 1, disable nloopback
    {
        gpio_set_level(NLOOPBACK, 1);   //High = nloopback disabled
        ESP_LOGI(ESP, "NLoopback Disabled.");
        gpio_set_level(LED_NLOOPBACK, 0);
        toggle = 0;
    }
    else                                //enable nloopback
    {
        gpio_set_level(NLOOPBACK, 0);   //Low = nloopback enabled
        ESP_LOGI(ESP, "NLoopback Enabled.");
        gpio_set_level(LED_NLOOPBACK, 1);
        toggle++;
    }
}

//*********Command definitions*********

static int Send_Message(int argc, char**argv)
{
    //CRC functions expect a pointer
    char *CRCsendptr;     
    CRCsendptr = &TEST_MSG; //reset/attach pointer, must be reset after CRC function
    //CRC-16/CCITT, poly = 0x1021, init = 0x0000, refin = true, refout = true, xorout = 0x0000
    uint16_t CRCresult = ~crc16_le((uint16_t)~0x0000, (const uint8_t*)CRCsendptr, sizeof(TEST_MSG)); //aka KERMIT
    uint8_t (*CRCbytes)[2] = (void *) &CRCresult;   //convert to bytes to add to message array
    ESP_LOGI(ESP, "Sending msg. CRC-KERMIT check: 0x%02X%02X", (*CRCbytes)[1], (*CRCbytes)[0]);

    uint8_t FINAL_STR[sizeof(TEST_MSG) + sizeof(uint16_t)];    //Full message frame (header + data + crc)
    ESP_LOGI(ESP, "final size with crc: %u, without crc: %u", sizeof(FINAL_STR), sizeof(TEST_MSG));

    uint32_t index = 0;
    for(int i = 0; i < sizeof(TEST_MSG); i++) //build test msg data
    {
        FINAL_STR[index++] = TEST_MSG[i];
    }

    FINAL_STR[index++] = (*CRCbytes)[0];//attach CRC1 IN ENDIAN MODE OF CRC (KERMIT = LE)
    FINAL_STR[index++] = (*CRCbytes)[1];//FINAL_STR does not get null character

    uart_write_bytes(UART_NUM_2, (const char*)FINAL_STR, sizeof(FINAL_STR));  //!!!Send tx data to SIG100!!!
    ESP_LOGI(ESP, "Sent msg.");
    //uart_write_bytes(UART_NUM_2, (const char*)TEST_MSG, sizeof(TEST_MSG));  //!!!Send tx data to SIG100!!!

    vTaskDelay(T500ms);     //500ms
    return 0;
}

void register_send_message(void)
{
    const esp_console_cmd_t button1_cmd = {
        .command = "send_msg",
        .help = "send data tx over UART",
        .hint = NULL,
        .func = &Send_Message,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&button1_cmd) );
}

static int ToggleLoopBackCMD(int argc, char ** argv)
{
    ToggleLoopBack();
    vTaskDelay(T500ms);     //500ms
    return 0;
}

void register_toggleloopback(void)
{
    const esp_console_cmd_t button2_cmd = {
        .command = "nloopback",
        .help = "change nloopback",
        .hint = NULL,
        .func = &ToggleLoopBackCMD,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&button2_cmd) );
}

static int SleepViaNSleepCMD(int argc, char ** argv)
{
    SleepViaNSleep();
    return 0;
}

void register_sleepviansleep(void)
{
    const esp_console_cmd_t button3_cmd = {
        .command = "nsleep",
        .help = "sleep via nsleep",
        .hint = NULL,
        .func = &SleepViaNSleepCMD,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&button3_cmd) );
}

static int ReadReg_(int argc, char ** argv)
{
    DvcCtrl0_Val = ReadReg(DVC_CTRL_0);       //Reg_0, default R0101010 = 19200 bitrate, nloopback disabled
                //ESP_LOGI(ESP, "Reg0 Val = 0x%X", DvcCtrl0_Val);
                vTaskDelay(T10ms);
                
                DvcCtrl1_Val = ReadReg(DVC_CTRL_1);       //Reg_1, default 00011111
                //ESP_LOGI(ESP, "Reg1 Val = 0x%X", DvcCtrl1_Val);
                vTaskDelay(T10ms);
                
                FreqMain_Val = ReadReg(FREQ_MAIN);       //Reg_2, default 01010000 = 13mhz
                //ESP_LOGI(ESP, "Reg2 Val = 0x%X", FreqMain_Val);
                vTaskDelay(T10ms);
                
                FreqAlt1_Val = ReadReg(FREQ_ALT_1);       //Reg_3, default 00000000 = 5mhz
                //ESP_LOGI(ESP, "Reg3 Val = 0x%X", FreqAlt1_Val);
                vTaskDelay(T10ms);
                
                FreqAlt2_Val = ReadReg(FREQ_ALT_2);       //Reg_4, default 10101010 = 22mhz
                //ESP_LOGI(ESP, "Reg4 Val = 0x%X", FreqAlt2_Val);
                vTaskDelay(T10ms);
                
                ActiveFreq_Val = ReadReg(ACTIVE_FREQ);   //Reg_5, will be the value of reg 2, 3 or 4
                //ESP_LOGI(ESP, "Reg5 Val = 0x%X", ActiveFreq_Val);
                vTaskDelay(T10ms);
                
                SlpCtrl_Val = ReadReg(SLP_CTRL);         //Reg_6, default 01111100, enhanced sleep mode, 6 sec autosleep
                //ESP_LOGI(ESP, "Reg6 Val = 0x%X", SlpCtrl_Val);
                vTaskDelay(T100ms);
                return 0;
}

void register_readreg(void)
{
    const esp_console_cmd_t button4_cmd = {
        .command = "readreg",
        .help = "read updated reg values, print to usb",
        .hint = NULL,
        .func = &ReadReg_,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&button4_cmd) );
}

static int AltFreqCMD(int argc, char ** argv)
{
    AlternateFreq();
    vTaskDelay(T500ms);
    return 0;
}

void register_AltFreq(void)
{
    const esp_console_cmd_t button5_cmd = {
        .command = "altfreq",
        .help = "change between main/alt1/alt2 freq",
        .hint = NULL,
        .func = &AltFreqCMD,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&button5_cmd) );
}

static int ResetCMD(int argc, char ** argv)
{
    ESP_LOGI(ESP, "Resetting SIG100");
                gpio_set_level(NRESET, 0);
                vTaskDelay(T100ms);
                gpio_set_level(NRESET, 1);
                vTaskDelay(T100ms);
    return 0;
}

void register_Reset(void)
{
    const esp_console_cmd_t button6_cmd = {
        .command = "reset",
        .help = "RESET SIG100",
        .hint = NULL,
        .func = &ResetCMD,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&button6_cmd) );
}