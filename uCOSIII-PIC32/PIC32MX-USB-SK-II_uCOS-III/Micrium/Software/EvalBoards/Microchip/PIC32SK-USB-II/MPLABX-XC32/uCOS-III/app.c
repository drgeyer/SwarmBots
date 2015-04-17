/*
*********************************************************************************************************
*                                               uC/OS-III
*                                         The Real-Time Kernel
*
*                             (c) Copyright 1998-2010, Micrium, Weston, FL
*                                          All Rights Reserved
*
*
*                                            MIPS Sample code
*
* File : APP.C
*********************************************************************************************************
*/

#include <includes.h>

/*
*********************************************************************************************************
*                                          CONFIGURATION BITS
*********************************************************************************************************
*/

#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (PIC32 Starter Kit: use divide by 2 only)
#pragma config FPLLMUL  = MUL_20        // PLL Multiplier
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = XT            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select
#pragma config DEBUG    = OFF           // Debugger Disabled for Starter Kit

/*
*********************************************************************************************************
*                                                Defines
*********************************************************************************************************
*/
#define TASK_STK_SIZE                           256


#define APP_CFG_TASK_ENCODER_STK_SIZE           TASK_STK_SIZE
#define APP_CFG_TASK_ENCODER_PRIO               5u      // 0 = lowest prio, 10 = highest
#define APP_CFG_TASK_ENCODER_STK_SIZE_LIMIT     TASK_STK_SIZE - 32

#define APP_CFG_TASK_IMU_STK_SIZE               TASK_STK_SIZE
#define APP_CFG_TASK_IMU_PRIO                   4u
#define APP_CFG_TASK_IMU_STK_SIZE_LIMIT         TASK_STK_SIZE - 32

#define APP_CFG_TASK_XBEE_STK_SIZE              TASK_STK_SIZE
#define APP_CFG_TASK_XBEE_PRIO                  3u
#define APP_CFG_TASK_XBEE_STK_SIZE_LIMIT        TASK_STK_SIZE - 32

//GY-80 Defines
enum ADXL345_MAP
{
    ADXL345_DEVID          = 0x00,
    ADXL345_THRESH_TAP     = 0x1D,
    ADXL345_OFSX           = 0x1E,
    ADXL345_OFSY           = 0x1F,
    ADXL345_OFSZ           = 0x20,
    ADXL345_DUR            = 0x21,
    ADXL345_Latent         = 0x22,
    ADXL345_Window         = 0x23,
    ADXL345_THRESH_ACT     = 0x24,
    ADXL345_THRESH_INACT   = 0x25,
    ADXL345_TIME_INACT     = 0x26,
    ADXL345_ACT_INACT_CTL  = 0x27,
    ADXL345_THRESH_FF      = 0x28,
    ADXL345_TIME_FF        = 0x29,
    ADXL345_TAP_AXES       = 0x2A,
    ADXL345_ACT_TAP_STATUS = 0x2B,
    ADXL345_BW_RATE        = 0x2C,
    ADXL345_POWER_CTL      = 0x2D,
    ADXL345_INT_ENABLE     = 0x2E,
    ADXL345_INT_MAP        = 0x2F,
    ADXL345_INT_SOURCE     = 0x30,
    ADXL345_DATA_FORMAT    = 0x31,
    ADXL345_DATAX0         = 0x32,
    ADXL345_DATAX1         = 0x33,
    ADXL345_DATAY0         = 0x34,
    ADXL345_DATAY1         = 0x35,
    ADXL345_DATAZ0         = 0x36,
    ADXL345_DATAZ1         = 0x37,
    ADXL345_FIFO_CTL       = 0x38,
    ADXL345_FIFO_STATUS    = 0x39,
    ADXL345_ADDRESS        = 0xA7
};

enum HMC5883_MAP
{
    HMC5883_CRA_REG_M             = 0x00,
    HMC5883_CRB_REG_M             = 0x01,
    HMC5883_MR_REG_M              = 0x02,
    HMC5883_OUT_X_H_M             = 0x03,
    HMC5883_OUT_X_L_M             = 0x04,
    HMC5883_OUT_Z_H_M             = 0x05,
    HMC5883_OUT_Z_L_M             = 0x06,
    HMC5883_OUT_Y_H_M             = 0x07,
    HMC5883_OUT_Y_L_M             = 0x08,
    HMC5883_SR_REG_Mg             = 0x09,
    HMC5883_IRA_REG_M             = 0x0A,
    HMC5883_IRB_REG_M             = 0x0B,
    HMC5883_IRC_REG_M             = 0x0C,
    HMC5883_TEMP_OUT_H_M          = 0x31,
    HMC5883_TEMP_OUT_L_M          = 0x32,
    HMC5883_ADDRESS               = 0x3D,
    //config options
    HMC5883_CONTINUOUS      = 0x00,
    HMC5883_SINGLEREAD      = 0x01,
    HMC5883_IDLE            = 0x03
};

struct ADXL345
{
    unsigned char DATAX0;
    unsigned char DATAX1;
    unsigned char DATAY0;
    unsigned char DATAY1;
    unsigned char DATAZ0;
    unsigned char DATAZ1;
    CPU_INT16S DATAX;
    CPU_INT16S DATAY;
    CPU_INT16S DATAZ;
};

struct HMC5883
{
    unsigned char DATAX0;
    unsigned char DATAX1;
    unsigned char DATAY0;
    unsigned char DATAY1;
    unsigned char DATAZ0;
    unsigned char DATAZ1;
    CPU_INT16S DATAX;
    CPU_INT16S DATAY;
    CPU_INT16S DATAZ;
    CPU_INT16S HEADING;
};
/*
*********************************************************************************************************
*                                                VARIABLES
*********************************************************************************************************
*/

static  OS_TCB    App_TaskStartTCB; 
static  CPU_STK   App_TaskStartStk[APP_CFG_TASK_START_STK_SIZE];

static  OS_TCB    App_TaskEncoderTCB;
static  CPU_STK   App_TaskEncoderStk[APP_CFG_TASK_ENCODER_STK_SIZE];

static  OS_TCB    App_TaskIMUTCB;
static  CPU_STK   App_TaskIMUStk[APP_CFG_TASK_IMU_STK_SIZE];

static  OS_TCB    App_TaskXBeeTCB;
static  CPU_STK   App_TaskXBeeStk[APP_CFG_TASK_IMU_STK_SIZE];



CPU_INT16U LeftEncoder_State;
CPU_INT16U RightEncoder_State;
CPU_INT16U LeftEncoder_Ticks;
CPU_INT16U RightEncoder_Ticks;
static CPU_INT16S LeftWheelPercent;
static CPU_INT16S RightWheelPercent;
static CPU_INT16U IdealLeftWheelSpeed;
static CPU_INT16U IdealRightWheelSpeed;
unsigned char rx_Buffer[265];
CPU_INT16U rx_Buffer_index = 0;

struct ADXL345 Accelerometer;
struct HMC5883 Compass;
//for a Mag Gain of 1.3
CPU_INT16S Gauss_LSB_XY = 1100;
CPU_INT16S Gauss_LSB_Z  = 980;


/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  App_TaskCreate  (void);
static  void  App_ObjCreate   (void);

static  void  App_TaskStart   (void  *p_arg);
static  void  App_TaskEncoder(void * data);
static  void  App_TaskIMUUpdate(void * imudata);
static void App_TaskXBeeUpdate(void * xbeedata);

void CN_Int_Init(void);
void PWM_Module_Init(void);
void Init_GY_80(void);
struct ADXL345 ReadADXL345(void);
struct HMC5883 ReadHMC5883(void);
void AdjustPWM(void);
void U2WriteByte(unsigned char byte);
void U2XBeeTransmit(void);

//I2C Prototypes
unsigned char mReadByteI2C1(unsigned char ack);
unsigned char mWriteByteI2C1(unsigned char DOUT);
void mStopI2C1(void);
void mNackI2C1(void);
void mAckI2C1(void);
void mRestartI2C1(void);
void mStartI2C1(void);
void mIdleI2C1(void);


/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C code.
*
* Arguments   : none
*********************************************************************************************************
*/

int  main (void)
{
    OS_ERR   os_err;

    CPU_Init();                                                           /* Initialize the uC/CPU services                           */
    BSP_IntDisAll();

    OSInit(&os_err);                                                      /* Init uC/OS-III.                                          */

    OSTaskCreate((OS_TCB      *)&App_TaskStartTCB,                        /* Create the start task                                    */
                 (CPU_CHAR    *)"Start",
                 (OS_TASK_PTR  )App_TaskStart, 
                 (void        *)0,
                 (OS_PRIO      )APP_CFG_TASK_START_PRIO,
                 (CPU_STK     *)&App_TaskStartStk[0],
                 (CPU_STK_SIZE )APP_CFG_TASK_START_STK_SIZE_LIMIT,
                 (CPU_STK_SIZE )APP_CFG_TASK_START_STK_SIZE,
                 (OS_MSG_QTY   )0u,
                 (OS_TICK      )0u,
                 (void        *)0,
                 (OS_OPT       )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR      *)&os_err);
    
    OSStart(&os_err);                                                     /* Start multitasking (i.e. give control to uC/OS-III).     */
    
    (void)&os_err;
    
    return (0);                                                       
}

/*$PAGE*/
/*
*********************************************************************************************************
*                                          STARTUP TASK
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
* Arguments   : p_arg   is the argument passed to 'AppStartTask()' by 'OSTaskCreate()'.
*********************************************************************************************************
*/

static  void  App_TaskStart (void *p_arg)
{
    OS_ERR  err;
        

    (void)p_arg;

    BSP_InitIO();                                               /* Initialize BSP functions                             */
    CN_Int_Init();
    PWM_Module_Init();
    Mem_Init();                                                 /* Initialize memory managment module                   */
    Math_Init();                                                /* Initialize mathematical module                       */

#if (OS_CFG_STAT_TASK_EN > 0u)
    OSStatTaskCPUUsageInit(&err);                               /* Determine CPU capacity                               */
#endif    
    
#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();
#endif
                              
    App_TaskCreate();                                           /* Create Application tasks                             */
    
    while (DEF_TRUE) {                                          /* Task body, always written as an infinite loop.       */
        LED_Toggle(1u);
        OSTimeDlyHMSM(0u, 0u, 0, 100u, 
                      OS_OPT_TIME_HMSM_STRICT, 
                      &err);
        LED_Toggle(2u);
        OSTimeDlyHMSM(0u, 0u, 0, 100u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
        LED_Toggle(3u);
        OSTimeDlyHMSM(0u, 0u, 0, 100u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
    }
}

/*
*********************************************************************************************************
*                                          AppTaskCreate()
*
* Description : Create application tasks.
*
* Argument(s) : none
*
* Return(s)   : none
*
* Caller(s)   : AppTaskStart()
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  App_TaskCreate (void)
{
    OS_ERR os_err;
    
    //create EncoderClear task
    OSTaskCreate((OS_TCB      *)&App_TaskEncoderTCB,                        /* Create the start task                                    */
             (CPU_CHAR    *)"ENCODER",
             (OS_TASK_PTR  )App_TaskEncoder,
             (void        *)0,
             (OS_PRIO      )APP_CFG_TASK_ENCODER_PRIO,
             (CPU_STK     *)&App_TaskEncoderStk[0],
             (CPU_STK_SIZE )APP_CFG_TASK_ENCODER_STK_SIZE_LIMIT,
             (CPU_STK_SIZE )APP_CFG_TASK_ENCODER_STK_SIZE,
             (OS_MSG_QTY   )0u,
             (OS_TICK      )0u,
             (void        *)0,
             (OS_OPT       )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
             (OS_ERR      *)&os_err);

    if(os_err != OS_ERR_NONE)
    {
        //oops there was an error starting this task
    }

    //create RF Communication task
    OSTaskCreate((OS_TCB      *)&App_TaskIMUTCB,                        /* Create the start task                                    */
     (CPU_CHAR    *)"IMU",
     (OS_TASK_PTR  )App_TaskIMUUpdate,
     (void        *)0,
     (OS_PRIO      )APP_CFG_TASK_IMU_PRIO,
     (CPU_STK     *)&App_TaskIMUStk[0],
     (CPU_STK_SIZE )APP_CFG_TASK_IMU_STK_SIZE_LIMIT,
     (CPU_STK_SIZE )APP_CFG_TASK_IMU_STK_SIZE,
     (OS_MSG_QTY   )0u,
     (OS_TICK      )0u,
     (void        *)0,
     (OS_OPT       )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
     (OS_ERR      *)&os_err);

    if(os_err != OS_ERR_NONE)
    {
        //oops there was an error starting this task
    }
    //create IMU Update task
    OSTaskCreate((OS_TCB      *)&App_TaskXBeeTCB,                        /* Create the start task                                    */
         (CPU_CHAR    *)"XBee",
         (OS_TASK_PTR  )App_TaskXBeeUpdate,
         (void        *)0,
         (OS_PRIO      )APP_CFG_TASK_XBEE_PRIO,
         (CPU_STK     *)&App_TaskXBeeStk[0],
         (CPU_STK_SIZE )APP_CFG_TASK_XBEE_STK_SIZE_LIMIT,
         (CPU_STK_SIZE )APP_CFG_TASK_XBEE_STK_SIZE,
         (OS_MSG_QTY   )0u,
         (OS_TICK      )0u,
         (void        *)0,
         (OS_OPT       )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
         (OS_ERR      *)&os_err);

    if(os_err != OS_ERR_NONE)
    {
        //oops there was an error starting this task
    }
    //create Ultrasonic Read task

    
}

void CN_Int_Init()
{
    CPU_INT16U config;
    CPU_INT32U dummy;
    
    TRISCbits.TRISC14 = 1; //change notice 0 as input
    TRISDbits.TRISD4 = 1;  //change notice 13 as input

    config = CHANGE_INT_ON | CHANGE_INT_PRI_3;

    CNCON = 0x8000;  //Enable the Change Notice module

    EnableCN0();
    EnableCN13();

    //Enable a weak pull-up corresponding to the CN pin
    ConfigCNPullups(CN0_PULLUP_ENABLE | CN13_PULLUP_ENABLE);

    //dummy read the ports
    dummy = PORTC;
    dummy = PORTD;

    ConfigIntCN(config);
    
    mCNClearIntFlag(); //clear the interrupt flag in case it was set

    LeftEncoder_State = PORTCbits.RC14;
    RightEncoder_State = PORTDbits.RD4;
}

void PWM_Module_Init()
{
   // Test of PWM mode

    OC1CON = 0x8000 | 0x06;  // ON, Timer 2 source, PWM fault disabled
    OC2CON = 0x8000 | 0x06;  // ON, Timer 2 source, PWM fault disabled

    T2CON = 0x8000 | 0x50;  //ON, 1:32 prescale @72 MHz FPB = 2.25 MHz, T=444.444ns

    PR2 =    2250;   // Set PWM period: 225*444.444ns = 100 us, T = 10 KHz
                               // PR2 = 2250 is 1000 Hz, T = 1 ms
                               // PR2 = 22,500 is 100 Hz, T = 10 ms

    int percent1 = 0;
    int percent2 = 0;
    OC1RS = (PR2 * percent1)/100 ;  // set PWM on time out of PR2 period
    OC2RS = (PR2 * percent2)/100;  // set pwm on time

}

void Init_GY_80(void)
{
    //Initialize I2C module 1
    I2C1BRG = 388; //set baud rate generator so we get 100KHz
    I2C1CONbits.DISSLW = 1; // disable slew rate
    I2C1CONbits.ON = 1; //enable I2C module, configure pins for serial op

    //now that the I2C Module is open we can set up the ADXL345
    mIdleI2C1();
    mStartI2C1();
    mWriteByteI2C1(ADXL345_ADDRESS & 0xFE);  //send a ADDR + W
    mIdleI2C1();
    mWriteByteI2C1(ADXL345_POWER_CTL);
    mIdleI2C1();
    mWriteByteI2C1(0x08);
    mIdleI2C1();
    mStopI2C1();

    //now set up the HMC5883 to read continuously
    mIdleI2C1();
    mStartI2C1();
    mWriteByteI2C1(HMC5883_ADDRESS & 0xFE);
    mIdleI2C1();
    mWriteByteI2C1(HMC5883_MR_REG_M);
    mIdleI2C1();
    mWriteByteI2C1(0x00);
    mIdleI2C1();
    mStopI2C1();

    //now set the gain to a known value
    mIdleI2C1();
    mStartI2C1();
    mWriteByteI2C1(HMC5883_ADDRESS & 0xFE);
    mIdleI2C1();
    mWriteByteI2C1(HMC5883_CRB_REG_M);
    mIdleI2C1();
    mWriteByteI2C1(0x20);   //Set MAGGAIN to +/- 1.3 Ga
    mIdleI2C1();
    mStopI2C1();
}

void initUART(void)
{
    TRISFbits.TRISF4 = 1;
    TRISFbits.TRISF5 = 0;
    U2MODE = 0x8008;
    U2STA = 0x1601;
    U2BRG = FPB / (7 *  9600) - 1;

    //clear UART interrupt flags
    IFS1bits.U2RXIF = 0;
    IFS1bits.U2EIF = 0;
    //set priority for interrupt vector
    IPC8bits.U2IP = 1;
    IPC8bits.U2IS = 1;
    //now enable the UART interrupts
    IEC1bits.U2RXIE = 1;
    IEC1bits.U2EIE = 1;

}

static void App_TaskEncoder(void * data)
{
    OS_ERR err;
    IdealLeftWheelSpeed = 6;
    IdealRightWheelSpeed = 6;
    while(1)
    {
        //compare the LeftEncoder_Ticks to RightEncoder_Ticks
        //which ever wheel is moving faster will have more ticks
        AdjustPWM();

        //Reset our ticks so the next time we get to this function we will
        //have valid data in these global tick counters
        LeftEncoder_Ticks = 0;
        RightEncoder_Ticks = 0;
        OSTimeDlyHMSM(0, 0, 0, 600,OS_OPT_TIME_HMSM_STRICT,&err);
        if(err != 0)
            err = 0;
    }
}

//employ simple PID controls
void AdjustPWM(void)
{
    static double prevLeftWheelSpeed, currLeftWheelSpeed;
    static double prevRightWheelSpeed, currRightWheelSpeed;
    static double LeftIntegral = 0, LeftDerivative = 0, LeftProportional;
    static double RightIntegral = 0, RightDerivative = 0, RightProportional;
    static const double P = 4, I = 2, D = 4;

    prevLeftWheelSpeed = currLeftWheelSpeed;
    prevRightWheelSpeed = currRightWheelSpeed;

    currLeftWheelSpeed = LeftEncoder_Ticks * (0.3427) / 0.6;
    currRightWheelSpeed = RightEncoder_Ticks * (0.3427) / 0.6;

    LeftProportional = P * (IdealLeftWheelSpeed - currLeftWheelSpeed);
    RightProportional = P * (IdealRightWheelSpeed - currRightWheelSpeed);

    LeftDerivative = D * (currLeftWheelSpeed - prevLeftWheelSpeed);
    RightDerivative = D *  (currRightWheelSpeed - prevRightWheelSpeed);

    LeftIntegral += I * (IdealLeftWheelSpeed - currLeftWheelSpeed);
    RightIntegral += I * (IdealRightWheelSpeed - currRightWheelSpeed);

    if(LeftIntegral > 100)
        LeftIntegral = 100;
    else if (LeftIntegral < -100)
        LeftIntegral = -100;

    if(RightIntegral > 100)
        RightIntegral = 100;
    else if (RightIntegral < -100)
        RightIntegral = -100;

    LeftWheelPercent += (LeftProportional - LeftDerivative + LeftIntegral);
    RightWheelPercent += (RightProportional - RightDerivative + RightIntegral);
    if(LeftWheelPercent > 100)
        LeftWheelPercent = 100;
    else if (LeftWheelPercent < 0)
        LeftWheelPercent = 0;
    
    if(RightWheelPercent > 100)
        RightWheelPercent = 100;
    else if (RightWheelPercent < 0)
        RightWheelPercent = 0;
    
    OC1RS = (PR2 * RightWheelPercent)/100 ;  // set PWM on time out of PR2 period
    OC2RS = (PR2 * LeftWheelPercent)/100;  // set pwm on time
}

static void App_TaskIMUUpdate(void * imudata)
{
    OS_ERR err;
    Init_GY_80();
    while(1)
    {
        //Read the ADXL345
        Accelerometer = ReadADXL345();
        //Read the HMC5883
        Compass = ReadHMC5883();
        OSTimeDlyHMSM(0, 0, 0, 200,OS_OPT_TIME_HMSM_STRICT,&err);
        if (err != 0)
            err = 0;
    }
}
void mIdleI2C1(void)
{
    //Wait for Acken, Rcen, Pen, Rsen and Sen to clear
    while((I2C1CON&0x001F)!=0){}

}

void mStartI2C1(void)
{
    I2C1CONbits.SEN=1;
    while(I2C1CONbits.SEN){}
}

void mRestartI2C1(void)
{
    I2C1CONbits.RSEN=1;
    while(I2C1CONbits.RSEN){}
}

void mAckI2C1(void)
{
    I2C1CONbits.ACKDT=0;
    I2C1CONbits.ACKEN=1;
    while(I2C1CONbits.ACKEN){}
}

void mNackI2C1(void)
{
    I2C1CONbits.ACKDT=1;
    I2C1CONbits.ACKEN=1;
    while(I2C1CONbits.ACKEN){}
}

void mStopI2C1(void)
{
    I2C1CONbits.PEN=1;
    while(I2C1CONbits.PEN){}
}

unsigned char mWriteByteI2C1(unsigned char DOUT)
{
    //Load data into transmit register
    I2C1TRN=DOUT;
    while(I2C1STATbits.TRSTAT){};
    //Recover Ack/Nack
    if(I2C1STATbits.ACKSTAT==1)
        return(1);
    else
        return(0);
    //returne 1 for acked, 0 for nacked
}

unsigned char mReadByteI2C1(unsigned char ack)
{
    unsigned char temp;
    I2C1CONbits.RCEN=1;
    while(I2C1CONbits.RCEN){}
    temp = I2C1RCV;
    //Reception is started, send ack/nack after read

    if(ack==0)
        mNackI2C1();
    else
        mAckI2C1();
    //Reception should be complete - pull out data
    return temp;
}

struct ADXL345 ReadADXL345(void)
{
    struct ADXL345 temp;
    //now that the I2C Module is open we can set up the ADXL345
    mIdleI2C1();
    mStartI2C1();
    mWriteByteI2C1(ADXL345_ADDRESS & 0xFE);  //send a ADDR + W
    mIdleI2C1();
    mWriteByteI2C1(ADXL345_DATAX0);
    mIdleI2C1();
    mStopI2C1();
    mIdleI2C1();

    mStartI2C1();
    mWriteByteI2C1(ADXL345_ADDRESS);    //send a ADDR + R
    mIdleI2C1();
    temp.DATAX0 = mReadByteI2C1(1); //ack after receipt
    mIdleI2C1();
    temp.DATAX1 = mReadByteI2C1(1);
    mIdleI2C1();
    temp.DATAY0 = mReadByteI2C1(1);
    mIdleI2C1();
    temp.DATAY1 = mReadByteI2C1(1);
    mIdleI2C1();
    temp.DATAZ0 = mReadByteI2C1(1);
    mIdleI2C1();
    temp.DATAZ1 = mReadByteI2C1(0);
    mStopI2C1();

    temp.DATAX = (temp.DATAX1 << 8) | temp.DATAX0;
    temp.DATAY = (temp.DATAY1 << 8) | temp.DATAY0;
    temp.DATAZ = (temp.DATAZ1 << 8) | temp.DATAZ0;

    return temp;
}

struct HMC5883 ReadHMC5883(void)
{
    struct HMC5883 temp;
    //now that the I2C Module is open we can set up the ADXL345
    mIdleI2C1();
    mStartI2C1();
    mWriteByteI2C1(HMC5883_ADDRESS & 0xFE);  //send a ADDR + W
    mIdleI2C1();
    mWriteByteI2C1(HMC5883_OUT_X_H_M);
    mIdleI2C1();
    mStopI2C1();
    mIdleI2C1();

    mStartI2C1();
    mWriteByteI2C1(HMC5883_ADDRESS);    //send a ADDR + R
    mIdleI2C1();
    temp.DATAX0 = mReadByteI2C1(1); //ack after receipt
    mIdleI2C1();
    temp.DATAX1 = mReadByteI2C1(1);
    mIdleI2C1();
    temp.DATAY0 = mReadByteI2C1(1);
    mIdleI2C1();
    temp.DATAY1 = mReadByteI2C1(1);
    mIdleI2C1();
    temp.DATAZ0 = mReadByteI2C1(1);
    mIdleI2C1();
    temp.DATAZ1 = mReadByteI2C1(0);
    mStopI2C1();

    temp.DATAX = (temp.DATAX0 << 8) | temp.DATAX1;
    temp.DATAY = (temp.DATAY0 << 8) | temp.DATAY1;
    temp.DATAZ = (temp.DATAZ0 << 8) | temp.DATAZ1;

    //now calculate the heading from these values
    float heading = atan(temp.DATAY / (float)temp.DATAX);
    float declination_angle = 0.13;
    heading += declination_angle;

    if(heading < 0)
        heading += (2*3.141592);

    if(heading > 2*3.141592)
        heading -= 2 * (3.141592);

    //convert radians to degrees for readability
    temp.HEADING = (heading * 180.0 / 3.141592);
    
    return temp;
}

static void App_TaskXBeeUpdate(void * xbeedata)
{
    OS_ERR err;

    initUART();
    while(1)
    {
        //ok send our heading, speed out over UART to XBee
        U2XBeeTransmit();
        //
        OSTimeDlyHMSM(0, 0, 0, 20,OS_OPT_TIME_HMSM_STRICT,&err);
        if (err != 0)
            err = 0;
    }
}

void U2WriteByte(unsigned char byte)
{
    while(U2STAbits.UTXBF == 1){};
    U2TXREG = byte;
}

void U2XBeeTransmit()
{
    OS_ERR err;
    OS_OPT opt;
    OSSchedLock(&err);

    //write the Compass Data and Speed Data
    U2WriteByte(Compass.HEADING & 0xFF);
    U2WriteByte((Compass.HEADING & 0xFF00) >> 8);
    U2WriteByte(((IdealLeftWheelSpeed + IdealRightWheelSpeed)/2) & 0xFF);
    U2WriteByte('+');
    U2WriteByte('+');
    U2WriteByte('+');
    //Write the send sequence
//    U2WriteByte(0x03);
//    U2WriteByte(0xE8); //GT
//    OSTimeDly(1, opt, &err);
//    U2WriteByte(0x2B); //CC
//    OSTimeDly(1, opt, &err);
//    U2WriteByte(0x03);
//    U2WriteByte(0xE8); //GT
    OSSchedUnlock(&err);
}