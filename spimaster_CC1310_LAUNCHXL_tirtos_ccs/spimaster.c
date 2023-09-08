/*
 * Copyright (c) 2018-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== spimaster.c ========
 */

#include <inttypes.h>


/* Example/Board Header files */
#include "Board.h"


#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* POSIX Header files */
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>

/* Driver Header files */
#include <ti/drivers/PIN.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/display/Display.h>
#include <ti/drivers/PWM.h>

#include <ti/drivers/rf/RF.h>
#include <ti/drivers/Power.h>
#include <ti/devices/cc13x0/driverlib/gpio.h>
#include <ti/devices/cc13x0/driverlib/aon_batmon.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/cpu.h)

/* IMU */
#include "LSM6DSOX.h"
#include "Parameter_setting.h"

/* Example/Board Header files */
#include "smartrf_settings/smartrf_settings.h" // generated from smartrf application: 500 GB/s -> 915 MHz -> infinite 3512 code export; save; in the smartrf folder
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)
/* Packet TX Configuration */
//#define PAYLOAD_LENGTH      20  //remember to update for multi packets! lengh of tx buffer, larger than data sending; one packet

void send_databuffer(const void* buffer,int buffer_size);

int init_SPI_IMU(void);
int IMU_Configure(void);


int32_t platform_read(void *handle, uint16_t reg, uint8_t *bufp);
int32_t platform_write(void *handle, uint16_t reg, uint16_t data);

int Data_update_check(void *handle, uint16_t check_type);

uint8_t* Acceleration_raw_get(void *handle);
uint8_t* Angular_Rate_raw_get(void *handle);


int Activity_Detection(void *handle);

int Voltage_Temp_read(void);
int RF_transmission(uint8_t* XL_data_read, uint8_t* G_data_read);
void activityDetectionFxn(uint_least8_t index);
void intCallBackFxn(PIN_Handle handle, PIN_Id pinId);

#define THREADSTACKSIZE (1024)

#define SPI_MSG_LENGTH  (30)
//#define MASTER_MSG      ("Hello from master, msg#: ")

#define MAX_LOOP        (10)
#define SEND_DEBUG_INFO (0)
#define SEND_DATA (1)

static int16_t raw_temp; // need to set to be int16_t, otherwise does not make sense
static int16_t raw_accel[3];
static int8_t accel_8bit[6];

static int16_t raw_angular[3];
static int8_t angular_8bit[6];

static float accel_g[3];
static float angular_mdps[3];


uint8_t activity_detection = 0;

/* Pin driver handles */
static PIN_Handle intPinHandle;

/* Global memory storage for a PIN_Config table */
static PIN_State intPinState;


static Semaphore_Struct intSemaphore;
static Semaphore_Handle intSemaphoreHandle;


/***** Variable declarations for RF*****/
RF_Object rfObject;
RF_Handle rfHandle;
uint8_t packet[PAYLOAD_LENGTH+2];

static Display_Handle display;
static uint16_t seqNumber;

int16_t masterRxBuffer[SPI_MSG_LENGTH];
int16_t masterTxBuffer[SPI_MSG_LENGTH];
SPI_Transaction transaction;
SPI_Handle      masterSpi;
SPI_Params      spiParams;
uint8_t         data_ready = 0;

/* Semaphore to block master until slave is ready for transfer */
sem_t masterSem;

/* Led pin table */
PIN_Config LedPinTable[] =
{
    Board_PIN_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* LED initially off */
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* LED initially off */
    PIN_TERMINATE                                                                      /* Terminate list */
};

PIN_Config intPinTable[] = {
    IOID16 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

/*
 *  ======== slaveReadyFxn ========
 *  Callback function for the GPIO interrupt on Board_SPI_SLAVE_READY.
 */
void slaveReadyFxn(uint_least8_t index)
{
    data_ready = 1;
    //sem_post(&masterSem);
}

void intCallBackFxn(PIN_Handle handle, PIN_Id pinId)
{
    printf("It was interrupted!");
    activity_detection = 1;
    //sem_post(&masterSem);
}

/*
 * @brief  Send data through RF transmission
 *
 * @param  buffer            Array of 8-bit packet to be sent
 * @param  buffer_size       Size of the array
 *
 */

void send_databuffer(const void* buffer, int buffer_size) // buffer: size of the data element is undefined
{

    RF_cmdPropTx.pktLen = buffer_size+2;

    /* Set the frequency */
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);
    //uint8_t j;
    //for (j = 0; j < buffer_size/PAYLOAD_LENGTH; ++j)
    //{
        /* Create packet with incrementing sequence number and random payload */
        uint8_t *a=(((uint8_t*)buffer)/*+(PAYLOAD_LENGTH)*j*/); //rf always work in 8-bit packet, buffer[0] -> a[0] & a[1], 16 - into half
        packet[0] = (uint8_t)(seqNumber >> 8); //shift 8 bits, upper 8 bits
        packet[1] = (uint8_t)(seqNumber++); // lower 8 bits

        uint8_t i;
        for (i = 2; i < buffer_size+2 /*PAYLOAD_LENGTH+2*/; i++) // extra 2 bits for the length
        {
            packet[i]=a[i-2]; // load the whole array
//            packet[i] = ((uint8_t*)buffer)+(PAYLOAD_LENGTH)*j+i /* check pointer math */
        }

        /* Send packet */ // 01 23 - 23 01// data flipped
        RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx,
                                                   RF_PriorityNormal, NULL, 0);

        switch(terminationReason)
        {
            case RF_EventLastCmdDone:
                // A stand-alone radio operation command or the last radio
                // operation command in a chain finished.
                break;
            case RF_EventCmdCancelled:
                // Command cancelled before it was started; it can be caused
            // by RF_cancelCmd() or RF_flushCmd().
                break;
            case RF_EventCmdAborted:
                // Abrupt command termination caused by RF_cancelCmd() or
                // RF_flushCmd().
                break;
            case RF_EventCmdStopped:
                // Graceful command termination caused by RF_cancelCmd() or
                // RF_flushCmd().
                break;
            default:
                // Uncaught error event
                while(1);
        }

        uint32_t cmdStatus = ((volatile RF_Op*)&RF_cmdPropTx)->status;
        switch(cmdStatus)
        {
            case PROP_DONE_OK:
                // Packet transmitted successfully
                break;
            case PROP_DONE_STOPPED:
                // received CMD_STOP while transmitting packet and finished
                // transmitting packet
                break;
            case PROP_DONE_ABORT:
                // Received CMD_ABORT while transmitting packet
                break;
            case PROP_ERROR_PAR:
                // Observed illegal parameter
                break;
            case PROP_ERROR_NO_SETUP:
                // Command sent without setting up the radio in a supported
                // mode using CMD_PROP_RADIO_SETUP or CMD_RADIO_SETUP
                break;
            case PROP_ERROR_NO_FS:
                // Command sent without the synthesizer being programmed
                break;
            case PROP_ERROR_TXUNF:
                // TX underflow observed during operation
                break;
            default:
                // Uncaught error event - these could come from the
                // pool of states defined in rf_mailbox.h
                while(1);
        }

        /* Power down the radio */
        RF_yield(rfHandle);
}


/*
 * @brief  Read register of IMU
 *
 * @param  handle    masterSpi
 * @param  reg       Register to read from IMU
 * @param  bufp      Pointer to buffer that store the data read
 * @param  len       Number of consecutive register to read (always set to 1 - consider deleting this)
 *
 * @return  Value of the register read from IMU
 */
//e.g. platform_read(masterSpi, LSM6DSOX_WHOAMI, &dummy, 1);
int32_t platform_read(void *handle, uint16_t reg, uint8_t *bufp) //potential power saving: buffer
{
    int32_t ret;
    reg |= READ_BIT;

    uint8_t* tx_Address = &reg; /*Address of the register to write to IMU */

    transaction.count = 1;
    transaction.txBuf = tx_Address;/*!< void * to a buffer with data to be transmitted */
    transaction.rxBuf = bufp;/*< void * to a buffer to receive data */

    bool spitransferOK;

    spitransferOK = SPI_transfer(handle, &transaction);

    if (spitransferOK) {
      //  printf("SPI transfer successful\n");
      //  printf("Transmitted: 0x%04X\n", reg);

    }
    else {
        printf("SPI transfer failed\n");
        printf("Transmitted: 0x%04X\n", reg);
        while(1); //send message through RF and exit to the main loop
    }

    ret = *bufp; //return a known message so know what error it is
 //   printf("Received: 0x%04X\n", ret); // for debugging: check if match smartrf

    return ret;
}

/*
 * @brief  Write value to register of IMU
 *
 * @param  handle    masterSpi
 * @param  reg       Register to write to IMU
 * @param  data      Data to write to reg (here is 0xXXXX)
 * @param  len       Number of consecutive register to write
 *
 * @return  The data written to the register of IMU
 */
//e.g. platform_write(masterSpi, LSM6DSOX_CTRL1_XL, data_to_write, 1);
int32_t platform_write(void *handle, uint16_t reg, uint16_t data)
{
    int32_t ret;
    bool spitransferOK_message;

    uint16_t tx_Data = reg|data;
    uint16_t* tx_Address = &tx_Data; /*Address of the combined data of register & data to be transferred */

    transaction.count = 1;
    transaction.txBuf = tx_Address;/*!< void * to a buffer with data to be transmitted */

    spitransferOK_message = SPI_transfer(handle, &transaction);

    if (spitransferOK_message) {
     //   printf("SPI transfer successful\n");
     //   printf("Transmitted: 0x%04X\n", tx_Data);
    }
    else {
        printf("SPI transfer failed\n");
        printf("Transmitted: 0x%04X\n", reg);
        while(1);
    }

    ret = tx_Data;

    return ret;
}


/*
 * @brief  Check the status register of IMU to see if there is new acceleration or angular velocity available
 *
 * @param  handle          masterSpi
 * @param  reg             Register to check
 * @param  check_type      XL_BIT to check acceleration; G_BIT to check angular velocity
 *
 * @return  1-new data; 0-no new data
 */
//e.g. Data_update_check(masterSpi, LSM6DSOX_STATUS_REG, XL_BIT);
int Data_update_check(void *handle, uint16_t check_type){ //check_type: XL_BIT or G_BIT

 //   uint8_t dummy_buf;
    int16_t status_reg;

    status_reg = platform_read(handle, LSM6DSOX_STATUS_REG, &dummy_address);
    bool check_aval = ((status_reg & check_type) == check_type); // if true, the data is updated

    return check_aval;
}


/**
  * @brief  Acceleration data output
  *         L and H registers together express a 16-bit word in two's
  *         complement.
  * @param  handle      masterSpi
  * @param  reg         Register of IMU to check data ready status
  * @param  len         Number of consecutive register to write
  *
  * @return Array of 6 8-bit hex number (x, y, z axis low and high value)
  */
//e.g. Acceleration_raw_get(masterSpi, LSM6DSOX_STATUS_REG, 1)
uint8_t* Acceleration_raw_get(void *handle) {

    uint16_t data_XL;
    uint16_t data_XH;
    uint16_t data_YL;
    uint16_t data_YH;
    uint16_t data_ZL;
    uint16_t data_ZH;

    int check_XL_aval = Data_update_check(handle, XL_BIT);


    if(check_XL_aval) {

        data_XL = platform_read(masterSpi, LSM6DSOX_OUTX_L_A, buffer_XL);
        data_XH = platform_read(masterSpi, LSM6DSOX_OUTX_H_A, buffer_XL);
        data_YL = platform_read(masterSpi, LSM6DSOX_OUTY_L_A, buffer_XL);
        data_YH = platform_read(masterSpi, LSM6DSOX_OUTY_H_A, buffer_XL);
        data_ZL = platform_read(masterSpi, LSM6DSOX_OUTZ_L_A, buffer_XL);
        data_ZH = platform_read(masterSpi, LSM6DSOX_OUTZ_H_A, buffer_XL);

        accel_8bit[0] = (uint8_t)data_XH;
        accel_8bit[1] = (uint8_t)data_XL;
        accel_8bit[2] = (uint8_t)data_YH;
        accel_8bit[3] = (uint8_t)data_YL;
        accel_8bit[4] = (uint8_t)data_ZH;
        accel_8bit[5] = (uint8_t)data_ZL;


        data_XH <<= 8;
        data_YH <<= 8;
        data_ZH <<= 8;

    //    printf("data XH: 0x%04X\n", data_XL);
    //    printf("data XL: 0x%04X\n", data_XH);

        raw_accel[0] = data_XL | data_XH;
        raw_accel[1] = data_YL | data_YH;
        raw_accel[2] = data_ZL | data_ZH;

    //    printf("data X: 0x%04X\n", raw_accel[0]);

        accel_g[0] = ((float_t)raw_accel[0]) * XL_SCALE_RANGE_2_G/1000; // Refer Adafruit_LSM6DS.cpp
        accel_g[1] = ((float_t)raw_accel[1]) * XL_SCALE_RANGE_2_G/1000;
        accel_g[2] = ((float_t)raw_accel[2]) * XL_SCALE_RANGE_2_G/1000;

        printf("Acceleration [g]:%4.2f\t%4.2f\t%4.2f\r\n", accel_g[0], accel_g[1], accel_g[2]);


   }
    return accel_8bit;
}

/**
  * @brief  Gyroscope data output
  *         L and H registers together express a 16-bit word in two's
  *         complement.
  * @param  handle      masterSpi
  * @param  reg         Register of IMU to check data ready status
  * @param  len         Number of consecutive register to write - always set to 1
  *
  * @return Array of 6 8-bit hex number (x, y, z axis low and high value)
  */
//e.g. Angular_raw_get(masterSpi, LSM6DSOX_STATUS_REG, 1)
uint8_t* Angular_Rate_raw_get(void *handle) {


    uint16_t data_XL;
    uint16_t data_XH;
    uint16_t data_YL;
    uint16_t data_YH;
    uint16_t data_ZL;
    uint16_t data_ZH;

  //  uint8_t G_buff[6];

    int check_G_aval = Data_update_check(handle, G_BIT);

    if(check_G_aval) {

        data_XL = platform_read(masterSpi, LSM6DSOX_OUTX_L_G, buffer_G);
        data_XH = platform_read(masterSpi, LSM6DSOX_OUTX_H_G, buffer_G);
        data_YL = platform_read(masterSpi, LSM6DSOX_OUTY_L_G, buffer_G);
        data_YH = platform_read(masterSpi, LSM6DSOX_OUTY_H_G, buffer_G);
        data_ZL = platform_read(masterSpi, LSM6DSOX_OUTZ_L_G, buffer_G);
        data_ZH = platform_read(masterSpi, LSM6DSOX_OUTZ_H_G, buffer_G);

        angular_8bit[0] = (uint8_t)data_XH;
        angular_8bit[1] = (uint8_t)data_XL;
        angular_8bit[2] = (uint8_t)data_YH;
        angular_8bit[3] = (uint8_t)data_YL;
        angular_8bit[4] = (uint8_t)data_ZH;
        angular_8bit[5] = (uint8_t)data_ZL;


        // can be removed
        data_XH <<= 8;
        data_YH <<= 8;
        data_ZH <<= 8;

        raw_angular[0] = data_XL | data_XH;
        raw_angular[1] = data_YL | data_YH;
        raw_angular[2] = data_ZL | data_ZH;

        angular_mdps[0] = ((float_t)raw_angular[0]) * G_SCALE_RANGE_1000_DPS/1000;
        angular_mdps[1] = ((float_t)raw_angular[1]) * G_SCALE_RANGE_1000_DPS/1000;
        angular_mdps[2] = ((float_t)raw_angular[2]) * G_SCALE_RANGE_1000_DPS/1000;

        printf("Angular rate [dps]:%4.2f\t%4.2f\t%4.2f\r\n", angular_mdps[0], angular_mdps[1], angular_mdps[2]);

   }
    return angular_8bit;
}

/**
  * @brief  Check if there is activity detected in IMU
  *
  * @param  handle      masterSpi
  * @param  reg         Wake up register of IMU - LSM6DSOX_WAKE_UP_SRC
  * @param  len         Number of consecutive register to write
  *
  */

int Activity_Detection(void *handle) {

    int32_t ret;

    int data_status;

  //  uint16_t dummy_act;

    ret = platform_read(handle, LSM6DSOX_WAKE_UP_SRC, &dummy_address);

    bool check_activity = ((ret&ACTIVITY_BIT) == ACTIVITY_BIT); // if true, there's change in activity status
  //  printf("pin value is: %d\n", activity_detection);
  //  printf("GPIO 12 is: 0x%02X\n", GPIO_read(12));

    if(check_activity) {
    //    printf("zzzzzzzzzZZZZZZZZZZZZZZZZZ\n");
        data_status = 0;
    }
    else{
     //   printf("!!!!!!!!!!!!!!!!!!!!! Hello! !!!!!!!!!!!!!!!!!!!\n");
        data_status = 1;
    }
    return data_status;

}



/**
  * @brief  Initialize SPI communication and GPIO interrupts
  */

int init_SPI_IMU(void) {

/* Initialize SPI parameters */

    SPI_Params_init(&spiParams);            //spiParams is a global (TODO change eventually)
    spiParams.frameFormat = SPI_POL0_PHA0; // Mode 1
    spiParams.bitRate = 1000000;
    //spiParams.mode = SPI_MASTER;
    spiParams.dataSize = 16;
    spiParams.transferMode = SPI_MODE_BLOCKING;
        //GPIO_write(Board_SPI_MASTER_READY, 0); //AG CHECK TODO check

        /* Open SPI */
    masterSpi = SPI_open(Board_SPI_MASTER, &spiParams);    //masterSPI is a global (TODO change eventually)
    if (masterSpi == NULL) {
        //Display_printf(display, 0, 0, "Error initializing master SPI\n");
        printf("Error initializing master SPI\n");
        while (1);
    }
    else {
        printf("Master SPI initialized\n");
        send_databuffer(test_buffer_SPI,sizeof(test_buffer_SPI));

    }

    return 0;

}


/**
  * @brief  Assign value to IMU registers // can change the name of the function
  *
  */

int IMU_Configure(void) {

    int32_t new_data_XL = platform_write(masterSpi, LSM6DSOX_CTRL1_XL, CTRL1_XL_VALUE);      // Turn on the accelerometer by setting ODR_XL and FS_XL
    int32_t new_data_G = platform_write(masterSpi, LSM6DSOX_CTRL2_G, CTRL2_G_VALUE);         // Turn on the gyroscope by setting ODR_G and FS_G
    int32_t WakeUpDur = platform_write(masterSpi, LSM6DSOX_WAKE_UP_DUR,  WAKE_UP_DUR);       // Set duration for inactivity detection
                                                                                                // Select activity/inactivity threshold resolution and duration
    int32_t WakeUpTHS = platform_write(masterSpi, LSM6DSOX_WAKE_UP_THS, WAKE_UP_THS);        // Set activity/inactivity threshold

    int32_t Tap_Enable = platform_write(masterSpi, LSM6DSOX_TAP_CFG0, TAP_CFG0_VALUE);       // Select sleep-change notification
                                                                                                // Select slope filter
    int32_t InterruptEnable = platform_write(masterSpi, LSM6DSOX_TAP_CFG2, TAP_CFG2_VALUE);  // Enable interrupt
                                                                                                // Inacitvity configuration: accelerometer to 12.5 Hz (LP mode)
                                                                                                // Gyroscope to Power-Down mode
    int32_t INT1_Routing = platform_write(masterSpi, LSM6DSOX_MD1_CFG, MD1_CFG_VALUE);       // Activity/Inactivity interrupt driven to INT1 pin
    int32_t INT2_Routing = platform_write(masterSpi, LSM6DSOX_MD2_CFG, MD2_CFG_VALUE);

    int32_t INT_dataReadt = platform_write(masterSpi, LSM6DSOX_INT1_CTRL, INT1_CTRL_VALUE);

    printf("SPI initialized successfully and IMU has been waken up\n");
    send_databuffer(test_buffer_configure,sizeof(test_buffer_configure));

    return 0;

}


/**
  * @brief  Read voltage and temperature on MCU with TI functions and send directly through RF
  */

int Voltage_Temp_read(void){
    //Get battery voltage (this will return battery voltage in decimal form you need to convert)
    uint32_t BATstatus = AONBatMonBatteryVoltageGet();
    //Get current temp as a signed value in Deg Celsius
    uint32_t TEMPstatus = AONBatMonTemperatureGetDegC();

    // convert in Milli volts
    BATstatus = (BATstatus * 125) >> 5;

    //convert in floating point value
    float BATvoltage = (float)BATstatus / 1000;

  //  float TEMPdegc = (float)TEMPstatus;

    uint8_t Bat_bytes[2] =
    {
      ((uint32_t)BATstatus >> 8) & 0xFF,  // shift by 0 not needed, of course, just stylistic
      ((uint32_t)BATstatus >> 0) & 0xFF,
    };

    uint8_t Temp_bytes[2] =
    {
      ((uint32_t)TEMPstatus >> 8) & 0xFF,
      ((uint32_t)TEMPstatus >> 0) & 0xff,
    };

    float TEMPdegc = (float)TEMPstatus;

    uint8_t mdata_buffer[4];

    int i;

    for(i = 0; i < 2; i++){
        mdata_buffer[i] = Bat_bytes[i];
        mdata_buffer[i + 2] = Temp_bytes[i];
    }

    printf("Voltage is %f\n", BATvoltage);
    printf("Temp is %f\n", TEMPdegc);


    send_databuffer(mdata_buffer,sizeof(mdata_buffer));

    return 1;

}


/**
  * @brief  Send acceleration and angular velocity (6x 8-bit packets each) through RF transmission
  */

int RF_transmission(uint8_t* XL_data_read, uint8_t* G_data_read){

    uint8_t mdata_buffer[PACKET_SIZE] = {0x00}; //the buffer where the data is saved to

    int i;

    for (i = 0; i < PACKET_SIZE/2; i++){

        mdata_buffer[i] = XL_data_read[i];
        mdata_buffer[i + PACKET_SIZE/2] = G_data_read[i];

    }

    send_databuffer(mdata_buffer,sizeof(mdata_buffer));

    return 1;
}

void activityDetectionFxn(uint_least8_t index)
{
    activity_detection = 2;

}


/*
 *  ======== masterThread ========
 *  Master SPI sends a message to slave while simultaneously receiving a
 *  message from the slave.
 */
void *masterThread(void *arg0)
{
    int32_t         status;
    PIN_State       pinState;
    PIN_Handle      hPin;
    PIN_Handle      intPinHandle;
    PIN_State       intPinState;
    uint32_t        currentOutputVal;
  //  uint32_t        standbyDuration = 3;


    /* Allocate LED pins */
 //   hPin = PIN_open(&pinState, LedPinTable);

    /* Initialize button semaphore */
    Semaphore_construct(&intSemaphore, 0, NULL);
    intSemaphoreHandle = Semaphore_handle(&intSemaphore);

    intPinHandle = PIN_open(&intPinState, intPinTable);

    /* Setup callback for button pins */
    PIN_registerIntCb(intPinHandle, &intCallBackFxn);



    /*
     * Board_SPI_MASTER_READY & Board_SPI_SLAVE_READY are GPIO pins connected
     * between the master & slave.  These pins are used to synchronize
     * the master & slave applications via a small 'handshake'.  The pins
     * are later used to synchronize transfers & ensure the master will not
     * start a transfer until the slave is ready.  These pins behave
     * differently between spimaster & spislave examples:
     *
     * spimaster example:
     *     * Board_SPI_MASTER_READY is configured as an output pin.  During the
     *       'handshake' this pin is changed from low to high output.  This
     *       notifies the slave the master is ready to run the application.
     *       Afterwards, the pin is used by the master to notify the slave it
     *       has opened Board_SPI_MASTER.  When Board_SPI_MASTER is opened, this
     *       pin will be pulled low.
     *
     *     * Board_SPI_SLAVE_READY is configured as an input pin. During the
     *       'handshake' this pin is read & a high value will indicate the slave
     *       ready to run the application.  Afterwards, a falling edge interrupt
     *       will be configured on this pin.  When the slave is ready to perform
     *       a transfer, it will pull this pin low.
     *
     * Below we set Board_SPI_MASTER_READY & Board_SPI_SLAVE_READY initial
     * conditions for the 'handshake'.
     */
    /* INIT */
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    RF_cmdPropTx.pktLen = PAYLOAD_LENGTH+2; // 2: index sent every time
    RF_cmdPropTx.pPkt = packet; //global variable
    RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW; //send immediately
    /* Request access to the radio */
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams); //global defined, call in other places

    send_databuffer(test_buffer,sizeof(test_buffer));
 //   send_databuffer(test_startup_buffer2,sizeof(test_startup_buffer2));//confirm Tx OK; send the data wirelessly; test_star..; size of the buffer as well

    Power_enablePolicy(); //how to sleep


    GPIO_setConfig(IOID12, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setCallback(IOID12, activityDetectionFxn);
    GPIO_enableInt(IOID12);  /* INT1 */

    GPIO_setConfig(IOID16, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setCallback(IOID16, intCallBackFxn);
    GPIO_enableInt(IOID16);  /* INT2 */

    /* Communicate with IMU */

    init_SPI_IMU(); //send message if it's successful

    IMU_Configure();

    // enable battery monitor enable
    AONBatMonEnable();

    /* Check IMU ID */
 //   uint8_t dummy_read_XL;
 //   int32_t rx_Data_XL = platform_read(masterSpi, LSM6DSOX_WHOAMI, &dummy_read_XL, 1);

// if using GPIO_read(PIN_INDEX), TAP_CFG0_VALUE needs to be set as 0x0020
    while (1) { // add PinInterrupt - minimize the number of communications

     //   printf("value of activity detection flag is: %d\n", activity_detection);

    //    int check_status = Activity_Detection(masterSpi); //
     //   Semaphore_pend(intSemaphoreHandle, 300000);

        while(GPIO_read(12)==GPIO_INT_ACTIVE){
    //    if(check_status == 1){

            int check_G_aval = Data_update_check(masterSpi, G_BIT); //

            if(check_G_aval){
                printf("value of activity detection flag is: %d\n", activity_detection);

                uint8_t* XL_data = Acceleration_raw_get(masterSpi);
                uint8_t* G_data = Angular_Rate_raw_get(masterSpi);
                RF_transmission(XL_data, G_data);
            }

        }
      //  else{

            //send_databuffer(test_buffer,sizeof(test_buffer));
        Voltage_Temp_read();


        sleep(STANDBY_DURATION_SECOND); //add seconds

     //       Semaphore_pend(intSemaphoreHandle, 1000);

         //   /* Read current output value for all pins */
         //   currentOutputVal =  PIN_getPortOutputValue(hPin);

         //    /* Toggle the LEDs, configuring all LEDs at once */
         //   PIN_setPortOutputValue(hPin, ~currentOutputVal);
       // }

    }

    return (NULL);
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    pthread_t           thread0;
    pthread_attr_t      attrs;
    struct sched_param  priParam;
    int                 retc;
    int                 detachState;


    /* Call driver init functions. */
    Display_init();
    Board_init();
    GPIO_init();
    SPI_init();
 //   Power_init();
 //   Power_enablePolicy();


    /* Open the display for output */
    display = Display_open(Display_Type_UART, NULL);
    if (display == NULL) {
        /* Failed to open display driver */
        while (1);
    }

    /* Turn on user LED */
    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);

    Display_printf(display, 0, 0, "Starting the SPI master example");
    Display_printf(display, 0, 0, "This example requires external wires to be "
        "connected to the header pins. Please see the Board.html for details.\n");

    /* Create application threads */
    pthread_attr_init(&attrs);

    detachState = PTHREAD_CREATE_DETACHED;
    /* Set priority and stack size attributes */
    retc = pthread_attr_setdetachstate(&attrs, detachState);
    if (retc != 0) {
        /* pthread_attr_setdetachstate() failed */
        while (1);
    }

    retc |= pthread_attr_setstacksize(&attrs, THREADSTACKSIZE);
    if (retc != 0) {
        /* pthread_attr_setstacksize() failed */
        while (1);
    }

    /* Create master thread */
    priParam.sched_priority = 1;
    pthread_attr_setschedparam(&attrs, &priParam);

    retc = pthread_create(&thread0, &attrs, masterThread, NULL); // call master thread here
    if (retc != 0) {
        /* pthread_create() failed */
        while (1);
    }

    return (NULL);
}
