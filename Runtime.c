//#include "main.c"
#include "Setup.c"

//Tags for ESP32 printf-line function "ESP_LOGx()"
static const char *PLC = "PLC";

void app_main(void)
{
    Setup();

//variable init
//For Sending & Receiving
    uint32_t Miss = 0;
    uint32_t TotalMiss = 0;
    uint32_t TotalRecv = 0;
    uint32_t rxLength = 0;

    for(int i = 0; i < sizeof(HEADER); i++)     //build test msg data, use re-use rxLength counter for this
    {
        TEST_MSG[rxLength++] = HEADER[i];
    }
    for(int i = 0; i < sizeof(DATA) - 1; i++)   //dont include null char in DATA
    {
        TEST_MSG[rxLength++] = DATA[i];
    }                                           //final mesage should look like "header+DATA" with no null char at the end
    rxLength = 0;   //reset counter

    while(1)    //Main loop
    {
        //Check pin states to toggle LEDs
        gpio_get_level(INH)       > 0 ? gpio_set_level(LED_INH, 1) : gpio_set_level(LED_INH, 0);
        
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, (size_t*)&rxLength));  //Check for data in UART rx buffer
        //ESP_LOGI(PLC, "%d", rxLength);
        if (rxLength)            //read rx data from SIG100 PLC over UART
        {
            uint8_t incomingChar [BUFFERSIZE] = {0};    //for incoming serial char
            ESP_LOGI(PLC, "Msg incoming.");
            rxLength = uart_read_bytes(UART_NUM_2, incomingChar, (BUFFERSIZE - 1), T250ms);    //read multiple bytes at once 

            CRCrecvptr = &incomingChar;
            uint16_t CRCresult = ~crc16_le((uint16_t)~0x0000, (const uint8_t*)CRCrecvptr, rxLength);    //CRC-KERMIT
            ESP_LOGI(PLC, "CRC check: %04X", (unsigned int)CRCresult);
            incomingChar[rxLength - 2] = 0;                     // Attach null char to form string
            
            //incomingChar[rxLength] = '\0';                      // Attach null char to form string
            
            ESP_LOGI(PLC, "%s", (char *)incomingChar);          //!!!Print rx data from SIG100!!!

            TotalRecv += rxLength;
            TotalMiss += CountOnes(CRCresult);
            Miss = CountOnes(CRCresult);
            ESP_LOGI(PLC, "Received %u total bytes, %u total misses, %u misses this message", TotalRecv, TotalMiss, Miss);

            rxLength = 0;                                       // Reset rxLength
            // for (size_t i = 0; i < sizeof(incomingChar); i++)   // Reset incomingChar array to all NULL values
            // {
            //     incomingChar[i] = '\0';
            // }
        }
        else
        {
            vTaskDelay(T250ms);  //wait 250ms
            //else do nothing
        }
    }
}