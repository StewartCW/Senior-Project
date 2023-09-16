#include "main.c"
//----------------Setup-----------------
// Initializes the ESP32
// Configures GPIO, sets output pin default levels,
// Runs ConfigUart()
// Input: none
// Output: none
void Setup(void)
{   
    vTaskDelay(T10ms); //some bootstrap pins need delay after bootup to be used as gpio

    //Initialize output pin states
    gpio_set_level(HDC, 1);                 //High = normal mode, Low =  Cmd mode.
    gpio_set_level(NLOOPBACK, 1);           //High = nloopback disabled, Low = enabled
    gpio_set_level(NSLEEP, 1);              //High = PLC normal mode, Low = PLC sleep/standby mode.
    gpio_set_level(FREQ_SEL_0, 0);          //freq_sel: "00" or "11" = main, "01" = alt1, "10" = alt2
    gpio_set_level(FREQ_SEL_1, 0);          
    gpio_set_level(NAUTO_SLEEP, 1);         //auto sleep disabled
    gpio_set_level(N_AUTO_FREQ_CHANGE, 1);  //auto freq change disabled
    gpio_set_level(NRESET, 1);              //active low, resets SIG100
    gpio_set_level(LED_NLOOPBACK, 0);       //led
    gpio_set_level(LED_NSLEEP, 0);          //led
    gpio_set_level(LED_INH, 0);             //led
    

    //Initialize Pin IO
    //Configure output pins
    ConfigGPIO(GPIO_OUTPUT_PIN_SEL,     //Pin bit mask
               GPIO_MODE_OUTPUT,       //Configure as output
               GPIO_PULLUP_DISABLE,    //Disable internal pullup
               GPIO_PULLDOWN_DISABLE,  //Disable internal pulldown (external ones are used where needed)
              GPIO_INTR_DISABLE);     //Disable interrupts

   //Configure input pins
    ConfigGPIO(GPIO_INPUT_PIN_SEL,      //Pin bit mask
               GPIO_MODE_INPUT,        //Configure as input
               GPIO_PULLUP_DISABLE,    //Disable internal pullup
               GPIO_PULLDOWN_DISABLE,  //Disable internal pulldown (external ones are used where needed)
               GPIO_INTR_DISABLE);     //Disable interrupts

    //Configure input pullup pins
    //ConfigGPIO(GPIO_INPUT_PU_PIN_SEL,   //Pin bit mask
    //            GPIO_MODE_INPUT,        //Configure as input
    //            GPIO_PULLUP_ENABLE,     //Enable internal pullup
    //            GPIO_PULLDOWN_DISABLE,  //Disable internal pulldown (external ones are used where needed)
    //            GPIO_INTR_DISABLE);     //Disable interrupts

    //UART connection to SIG100
    ConfigUART(BAUD_19200);
    
    vTaskDelay(T10ms);      //n*10ms ticks

    //SIG100 boots up with these register values as default.
    DvcCtrl0_Val = 0b00101010;      //Reg_0, default R0101010 = 19200 bitrate, nloopback disabled, R = READ ONLY
    DvcCtrl1_Val = 0b00011111;      //Reg_1, default 00011111
    FreqMain_Val = 0b01010000;      //Reg_2, default 01010000 = 13mhz
    FreqAlt1_Val = 0b00000000;      //Reg_3, default 00000000 = 5mhz
    FreqAlt2_Val = 0b10101010;      //Reg_4, default 10101010 = 22mhz
    ActiveFreq_Val = 0;           //Reg_5, default RRRRRRRR, reg 5 is read only 
    SlpCtrl_Val = 0b01111100;       //Reg_6, default 01111100, enhanced sleep mode, 6 sec autosleep

    //Reading from UART this early is unreliable
    //These will be the SIG100 default boot values, R = READ ONLY
    // DvcCtrl0_Val = ReadReg(DVC_CTRL_0);     //Reg_0, default R0101010 = 19200 bitrate, nloopback disabled
    // DvcCtrl1_Val = ReadReg(DVC_CTRL_1);     //Reg_1, default 00011111
    // FreqMain_Val = ReadReg(FREQ_MAIN);      //Reg_2, default 01010000 = 13mhz
    // FreqAlt1_Val = ReadReg(FREQ_ALT_1);     //Reg_3, default 00000000 = 5mhz
    // FreqAlt2_Val = ReadReg(FREQ_ALT_2);     //Reg_4, default 10101010 = 22mhz
    // ActiveFreq_Val = ReadReg(ACTIVE_FREQ);  //Reg_5, default RRRRRRRR, will be the value of reg 2, 3 or 4
    // SlpCtrl_Val = ReadReg(SLP_CTRL);        //Reg_6, default 01111100, enhanced sleep mode, 6 sec autosleep

    /*Console Initialization*/
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();

    repl_config.prompt = PROMPT_STR ">";
    repl_config.max_cmdline_length = CONFIG_CONSOLE_MAX_COMMAND_LINE_LENGTH;

    esp_console_register_help_command();
    register_button();

    #if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
        esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));

    #else
    #error Unsupported console type
    #endif

        ESP_ERROR_CHECK(esp_console_start_repl(repl));

    ESP_LOGI(ESP, "Boot up sequence completed.\n\n");
    ESP_LOGI(ESP, "NLoopback disabled, NAuto_Sleep disabled, N_Auto_Freq_Change disabled. Enhanced Sleep mode.");
}