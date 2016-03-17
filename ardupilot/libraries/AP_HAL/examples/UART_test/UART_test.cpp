/*
  simple test of UART interfaces
 */
 
#include <AP_HAL/AP_HAL.h>

#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif

#define START_FLAG (uint8_t) 0xA5
#define START_FLAG2 0x5A
#define STOP 0x25
#define RESET 0x40
#define SCAN 0x20
#define FORCE_SCAN 0x21
#define GET_INFO 0x50
#define GET_HEALTH 0x51

#define SCAN_PACKET 0x5A
#define SCAN_PACKET_LENGTH 5

#define GET_INFO_PACKET 0x5
#define GET_INFO_PACKET_LENGTH 20
uint8_t uartReadBuffer[10];

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_HAL::UARTDriver* uarts[] = {
    hal.uartA, // console
};
#define NUM_UARTS (sizeof(uarts)/sizeof(uarts[0]))


/*
  setup one UART at 57600
 */
static void setup_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == NULL) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->begin(115200);
}

static void send_uart(AP_HAL::UARTDriver *uart, uint8_t *data, uint16_t numberOfBytes){
    if (uart == NULL){
        return;
    }
    for(int i = 0; i < numberOfBytes; i++){
        uart->write(*(data + i));  
    }
}

void setup(void) 
{
    /*
      start all UARTs at 57600 with default buffer sizes
     */
    setup_uart(hal.uartA, "uartA"); // console
    setup_uart(hal.uartB, "uartB"); // 1st GPS
    setup_uart(hal.uartC, "uartC"); // telemetry 1
    setup_uart(hal.uartD, "uartD"); // telemetry 2
    setup_uart(hal.uartE, "uartE"); // 2nd GPS
    hal.uartA->printf("BEGIN\n");
    uint8_t message[2] = {START_FLAG,SCAN}; //message to start scan
    uint8_t *data;
    data = &message[0];
    
    hal.scheduler->delay(5000);
    send_uart(hal.uartE, data, 2);
}

static void test_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == NULL) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->printf("Hello on UART %s at %.3f seconds\n",
                 name, AP_HAL::millis()*0.001f);
}



bool read_uart(AP_HAL::UARTDriver *uart, uint8_t *buffer){
    if(uart->available() >= 7){
        //hal.uartA->printf("available:%d\n",uart->available());
        buffer[0] = uart->read();
        buffer[1] = uart->read();
        //uart->write(buffer[0]);
        //uart->write(buffer[1]);
        if(buffer[1] == SCAN_PACKET){
            for(int i = 2; i < SCAN_PACKET_LENGTH + 2; i++){
                buffer[i] = uart->read();
                //hal.uartA->printf("Num: %d\tValue:0x%02hhx\t",i,buffer[i]);
                //uart->write(buffer[i]);
            }
        }
        return true;
        //uart->printf("\n");
    }
    else{
      
    }   
}

void read_uart2(AP_HAL::UARTDriver *uart, uint8_t *buffer){
    while(uart->available()){
        while(uart->read() != SCAN_PACKET)
        uartReadBuffer[2] = uart->read();
        uartReadBuffer[3] = uart->read();
        uartReadBuffer[4] = uart->read();
        uartReadBuffer[5] = uart->read();
        uartReadBuffer[6] = uart->read();
        
        uint8_t quality;
        uint16_t angle;
        uint16_t distance;
        bool newScanFlag;
        
        newScanFlag = uartReadBuffer[2] & 0x01; //get last bit of 1st data byte
        quality = uartReadBuffer[2] >> 2;       //get 6 most significant bits of 1st data byte
        angle = ((uartReadBuffer[3] & 0xFE) >> 1) | (uartReadBuffer[4] << 7);
        distance = uartReadBuffer[5] | (uartReadBuffer[6] << 8); 
        hal.uartA->printf("Quality:%d\tStart:%d\tAngle:%d\tDistance:%d\n",quality,newScanFlag,angle,distance);    
       
    }
}

void loop(void) 
{	
    read_uart2(hal.uartE,uartReadBuffer);
    /*while(read_uart(hal.uartE,uartReadBuffer)){
        if(uartReadBuffer[1] == SCAN_PACKET){
            uint8_t quality;
            uint16_t angle;
            uint16_t distance;
            bool newScanFlag;
            newScanFlag = uartReadBuffer[2] & 0x01; //get last bit of 1st data byte
            quality = uartReadBuffer[2] >> 2;       //get 6 most significant bits of 1st data byte
            angle = ((uartReadBuffer[3] & 0xFE) >> 1) | (uartReadBuffer[4] << 7);
            distance = uartReadBuffer[5] | (uartReadBuffer[6] << 8); 
            hal.uartA->printf("Quality:%d\tStart:%d\tAngle:%d\tDistance:%d\n",quality,newScanFlag,angle,distance);
            uartReadBuffer[1] = 0x00;
        }
        else{
        hal.uartA->printf("Unrecoginized Command: 0x%02hhx\n",uartReadBuffer[1]);
        }    
    }*/
    
    
    
    //test_uart(hal.uartA, "uartA"); 
    // also do a raw printf() on some platforms, which prints to the
    // debug console
#if HAL_OS_POSIX_IO
    ::printf("Hello on debug console at %.3f seconds\n", AP_HAL::millis()*0.001f);
#endif

    //hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
