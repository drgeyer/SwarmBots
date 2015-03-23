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

#define APP_CFG_TASK_ENCODER_STK_SIZE           256u
#define APP_CFG_TASK_ENCODER_PRIO               5u      // 0 = lowest prio, 10 = highest
#define APP_CFG_TASK_ENCODER_STK_SIZE_LIMIT     256u
/*
*********************************************************************************************************
*                                                VARIABLES
*********************************************************************************************************
*/

static  OS_TCB    App_TaskStartTCB; 
static  CPU_STK   App_TaskStartStk[APP_CFG_TASK_START_STK_SIZE];

static  OS_TCB    App_TaskEncoderTCB;
static  CPU_STK   App_TaskEncoderStk[APP_CFG_TASK_ENCODER_STK_SIZE];


/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  App_TaskCreate  (void);
static  void  App_ObjCreate   (void);

static  void  App_TaskStart   (void  *p_arg);
static  void  App_TaskEncoder(void * data);

void CN_Int_Init();
void PWM_Module_Init();
void AdjustPWM();

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
    CN_Int_Init();
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
    //create IMU Update task
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
    OC3CON = 0x8000 | 0x06;  // ON, Timer 2 source, PWM fault disabled
    OC4CON = 0x8000 | 0x06;  // ON, Timer 2 source, PWM fault disabled

    T2CON = 0x8000 | 0x00;  //ON, 1:1 prescalar

    PR2 =    225;   // Set PWM period: 225*444.444ns = 100 us, f = 10 KHz

    OC1RS = (PR2 * LeftWheelPercent)/100;  // set PWM on time out of PR2 period
    OC2RS = (PR2 * RightWheelPercent)/100;  // set pwm on time
}

static void App_TaskEncoder(void * data)
{
    OS_ERR err;
    PWM_Module_Init();
    while(1)
    {
        //compare the LeftEncoder_Ticks to RightEncoder_Ticks
        //which ever wheel is moving faster will have more ticks
        AdjustPWM();

        //Reset our ticks so the next time we get to this function we will
        //have valid data in these global tick counters
        LeftEncoder_Ticks = 0;
        RightEncoder_Ticks = 0;
        OSTimeDlyHMSM(0, 0, 0, 200,OS_OPT_TIME_HMSM_STRICT,&err);
    }
}

//employ simple PID controls
void AdjustPWM()
{
    static double prevLeftWheelSpeed, currLeftWheelSpeed;
    static double prevRightWheelSpeed, currRightWheelSpeed;
    static double LeftIntegral = 0, LeftDerivative = 0, LeftProportional;
    static double RightIntegral = 0, RightDerivative = 0, RightProportional;
    static const double P = 0.1, I = 0.2, D = 1.0;

    prevLeftWheelSpeed = currLeftWheelSpeed;
    prevRightWheelSpeed = currRightWheelSpeed;

    currLeftWheelSpeed = LeftEncoder_Ticks * (0.3427) / 0.2;
    currRightWheelSpeed = RightEncoder_Ticks * (0.3427) / 0.2;

    LeftProportional = P * (IdealLeftWheelSpeed - currLeftWheelSpeed);
    RightProportional = P * (IdealRightWheelSpeed - currRightWheelSpeed);

    LeftDerivative = D * (currLeftWheelSpeed - prevLeftWheelSpeed);
    RightDerivative = D *  (currRightWheelSpeed - prevRightWheelSpeed);

    LeftIntegral += I * (IdealLeftWheelSpeed - currLeftWheelSpeed);
    RightIntegral += I * (IdealRightWheelSpeed - currRightWheelSpeed);

    if(LeftIntegral > 50)
        LeftIntegral = 50;
    else if (LeftIntegral < -50)
        LeftIntegral = -50;

    if(RightIntegral > 20)
        RightIntegral = 20;
    else if (RightIntegral < -20)
        RightIntegral = -20;

    LeftWheelPercent += (LeftProportional - LeftDerivative + LeftIntegral);
    RightWheelPercent += (RightProportional - RightDerivative + RightIntegral);
}