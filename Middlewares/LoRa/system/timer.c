 /*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Generic lora driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Wael Guibene
*/
/******************************************************************************
  * @file    timer.c
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    01-June-2017
  * @brief   Time server infrastructure
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sx126x-board.h"
#include "timer.h"
#include "rtc-board.h"

#define END_OF_FEBRUARY_LEAP                         60 //31+29
#define END_OF_JULY_LEAP                            213 //31+29+...

#define END_OF_FEBRUARY_NORM                         59 //31+28
#define END_OF_JULY_NORM                            212 //31+28+...

#define UNIX_YEAR                                    68 //1968 is leap year

//UNIX time 0 = start at 01:00:00, 01/01/1970
#define UNIX_HOUR_OFFSET                            ( ( TM_DAYS_IN_LEAP_YEAR + TM_DAYS_IN_YEAR ) * TM_SECONDS_IN_1DAY )

/*!
 * \brief Correction factors
 */
#define  DAYS_IN_MONTH_CORRECTION_NORM              ( (uint32_t )0x99AAA0 )
#define  DAYS_IN_MONTH_CORRECTION_LEAP              ( (uint32_t )0x445550 )


/* 365.25 = (366 + 365 + 365 + 365)/4 */
#define DIV_365_25( X )                               ( ( ( X ) * 91867 + 22750 ) >> 25 )

#define DIV_APPROX_86400( X )                       ( ( ( X ) >> 18 ) + ( ( X ) >> 17 ) )

#define DIV_APPROX_1000( X )                        ( ( ( X ) >> 10 ) +( ( X ) >> 16 ) + ( ( X ) >> 17 ) )

#define DIV_APPROX_60( X )                          ( ( ( X ) * 17476 ) >> 20 )

#define DIV_APPROX_61( X )                          ( ( ( X ) * 68759 ) >> 22 )

#define MODULO_7( X )                               ( ( X ) -( ( ( ( ( X ) + 1 ) * 299593 ) >> 21 ) * 7 ) )

/*!
 * \brief Calculates ceiling( X / N )
 */
#define DIVC( X, N )                                ( ( ( X ) + ( N ) -1 ) / ( N ) )

#define DIVC_BY_4( X )                              ( ( ( X ) + 3 ) >>2 )

#define DIVC_BY_2( X )                              ( ( ( X ) + 1 ) >> 1 )

#define	leapyear(year)		((year) % 4 == 0)
#define	days_in_year(a) 	(leapyear(a) ? 366 : 365)
#define	days_in_month(a) 	(month_days[(a) - 1])

static int month_days[12] = {	31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
//static uint32_t CalendarGetMonth( uint32_t days, uint32_t year );
static void CalendarDiv86400( uint32_t in, uint32_t* out, uint32_t* remainder );
//static uint32_t CalendarDiv61( uint32_t in );
static void CalendarDiv60( uint32_t in, uint32_t* out, uint32_t* remainder );

const char *WeekDayString[]={ "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };

/*!
 * safely execute call back
 */
#define exec_cb( _callback_ )   \
do                              \
{                               \
    if( _callback_ == NULL )    \
    {                           \
        while(1);               \
    }                           \
    else                        \
    {                           \
        _callback_( );          \
    }                           \
} while(0);                   


volatile uint8_t HasLoopedThroughMain = 0;
static TimerTime_t g_systime_ref = 0;

/*!
 * Timers list head pointer
 */
static TimerEvent_t *TimerListHead = NULL;

/*!
 * \brief Adds or replace the head timer of the list.
 *
 * \remark The list is automatically sorted. The list head always contains the
 *     next timer to expire.
 *
 * \param [IN]  obj Timer object to be become the new head
 * \param [IN]  remainingTime Remaining time of the previous head to be replaced
 */
static void TimerInsertNewHeadTimer( TimerEvent_t *obj );

/*!
 * \brief Adds a timer to the list.
 *
 * \remark The list is automatically sorted. The list head always contains the
 *     next timer to expire.
 *
 * \param [IN]  obj Timer object to be added to the list
 * \param [IN]  remainingTime Remaining time of the running head after which the object may be added
 */
static void TimerInsertTimer( TimerEvent_t *obj );

/*!
 * \brief Sets a timeout with the duration "timestamp"
 * 
 * \param [IN] timestamp Delay duration
 */
static void TimerSetTimeout( TimerEvent_t *obj );

/*!
 * \brief Check if the Object to be added is not already in the list
 * 
 * \param [IN] timestamp Delay duration
 * \retval true (the object is already in the list) or false  
 */
static bool TimerExists( TimerEvent_t *obj );

SysTime_t SysTimeAdd( SysTime_t a, SysTime_t b )
{
    SysTime_t c =  { .Seconds = 0, .SubSeconds = 0 };

    c.Seconds = a.Seconds + b.Seconds;
    c.SubSeconds = a.SubSeconds + b.SubSeconds;
    if( c.SubSeconds >= 1000 )
    {
        c.Seconds++;
        c.SubSeconds -= 1000;
    }
    return c;
}

SysTime_t SysTimeSub( SysTime_t a, SysTime_t b )
{
    SysTime_t c = { .Seconds = 0, .SubSeconds = 0 };

    c.Seconds = a.Seconds - b.Seconds;
    c.SubSeconds = a.SubSeconds - b.SubSeconds;
    if( c.SubSeconds < 0 )
    {
        c.Seconds--;
        c.SubSeconds += 1000;
    }
    return c;
}

void SysTimeSet( SysTime_t sysTime )
{
//    TimerTime_t cur_time = RtcGetTimerValue( );
//    TimerTime_t set_time = (TimerTime_t)sysTime.Seconds*1000 + sysTime.SubSeconds;
//    
//    g_systime_ref = set_time - cur_time;
	
	  SysTime_t DeltaTime;
	
	  SysTime_t calendarTime = { .Seconds = 0, .SubSeconds = 0 };
		TimerTime_t cur_time = RtcGetTimerValue( );
		
		calendarTime.Seconds=(uint32_t)cur_time/1000;
		calendarTime.SubSeconds=(uint16_t)cur_time%1000;
		
		// sysTime is epoch
    DeltaTime = SysTimeSub( sysTime, calendarTime );
	
		*((unsigned int *)(0x2000F000)) =(DeltaTime.Seconds>>24)&0xFF;
		*((unsigned int *)(0x2000F001)) =(DeltaTime.Seconds>>16)&0xFF;
		*((unsigned int *)(0x2000F002)) =(DeltaTime.Seconds>>8)&0xFF;
		*((unsigned int *)(0x2000F003)) =(DeltaTime.Seconds)&0xFF;

		*((unsigned int *)(0x2000F004)) =(DeltaTime.SubSeconds>>8)&0xFF;
		*((unsigned int *)(0x2000F005)) =(DeltaTime.SubSeconds)&0xFF;
}

SysTime_t SysTimeGet( void )
{
//    SysTime_t sysTime = { 0 };
//    TimerTime_t curTime = TimerGetCurrentTime();

//    sysTime.Seconds = (uint32_t)(curTime/1000);
//    sysTime.SubSeconds = (uint16_t)(curTime%1000);

	  SysTime_t calendarTime = { .Seconds = 0, .SubSeconds = 0 };
    SysTime_t sysTime = { .Seconds = 0, .SubSeconds = 0 };
    SysTime_t DeltaTime;
		
		TimerTime_t cur_time = RtcGetTimerValue( );
		
		calendarTime.Seconds=(uint32_t)cur_time/1000;
		calendarTime.SubSeconds=(uint16_t)cur_time%1000;
		
		DeltaTime.Seconds=(*((uint8_t *)(0x2000F000)))<<24|(*((uint8_t *)(0x2000F001)))<<16|(*((uint8_t *)(0x2000F002)))<<8|(*((uint8_t *)(0x2000F003)));
		DeltaTime.SubSeconds=(*((uint8_t *)(0x2000F004)))<<8|(*((uint8_t *)(0x2000F005)));
		
		sysTime = SysTimeAdd( DeltaTime, calendarTime );
		
    return sysTime;
}

SysTime_t SysTimeGetMcuTime( void )
{
    SysTime_t mcuTime = { 0 };
    TimerTime_t curTime = RtcGetTimerValue();

    mcuTime.Seconds = (uint32_t)(curTime/1000);
    mcuTime.SubSeconds = (uint16_t)(curTime%1000);

    return mcuTime;
}

uint32_t SysTimeToMs( SysTime_t sysTime )
{
    SysTime_t deltaTime = { 0 };

		deltaTime.Seconds = (uint32_t)(g_systime_ref/1000);
		deltaTime.SubSeconds = (uint16_t)(g_systime_ref%1000);
		
    SysTime_t calendarTime = SysTimeSub( sysTime, deltaTime );

    return calendarTime.Seconds * 1000 + calendarTime.SubSeconds;
}

SysTime_t SysTimeFromMs( uint32_t timeMs )
{
    uint32_t seconds = timeMs / 1000;
    uint32_t subSeconds = timeMs - seconds * 1000;
    SysTime_t sysTime = { .Seconds = seconds, .SubSeconds = ( int16_t )subSeconds };

		SysTime_t DeltaTime = { 0 };
		DeltaTime.Seconds=(*((uint8_t *)(0x2000F000)))<<24|(*((uint8_t *)(0x2000F001)))<<16|(*((uint8_t *)(0x2000F002)))<<8|(*((uint8_t *)(0x2000F003)));
		DeltaTime.SubSeconds=(*((uint8_t *)(0x2000F004)))<<8|(*((uint8_t *)(0x2000F005)));
		   
    return SysTimeAdd( sysTime, DeltaTime );
}

void SysTimeLocalTime( const uint32_t timestamp, struct tm *localtime )
{
    uint32_t seconds;
    uint32_t minutes;
    uint32_t days;
    uint32_t divOut;
    uint32_t divReminder;
    uint16_t i;
		
    CalendarDiv86400( timestamp , &days, &seconds );

    // Calculates seconds
    CalendarDiv60( seconds, &minutes, &divReminder );
    localtime->tm_sec = ( uint8_t )divReminder;

    // Calculates minutes and hours
    CalendarDiv60( minutes, &divOut, &divReminder);
    localtime->tm_min = ( uint8_t )divReminder;
    localtime->tm_hour = ( uint8_t )divOut;

		for (i = 1970; days >= days_in_year(i); i++) {
			days -= days_in_year(i);
		}
		localtime->tm_year = i-1900;

		if (leapyear(localtime->tm_year)) {
			days_in_month(2) = 29;
		}
		
		if(localtime->tm_year==200)
		{
			days_in_month(2) = 28;
		}
		
		for (i = 1; days >= days_in_month(i); i++) {
			days -= days_in_month(i);
		}
		days_in_month(2) = 28;
		localtime->tm_mon = i;

		localtime->tm_mday = days + 1;
		
		localtime->tm_isdst = -1;
}

static void CalendarDiv86400( uint32_t in, uint32_t* out, uint32_t* remainder )
{
#if 0
    *remainder = in % SECONDS_IN_1DAY;
    *out       = in / SECONDS_IN_1DAY;
#else
    uint32_t outTemp = 0;
    uint32_t divResult = DIV_APPROX_86400( in );

    while( divResult >=1 )
    {
        outTemp += divResult;
        in -= divResult * 86400;
        divResult= DIV_APPROX_86400( in );
    }
    if( in >= 86400 )
    {
        outTemp += 1;
        in -= 86400;
    }

    *remainder = in;
    *out = outTemp;
#endif
}

//static uint32_t CalendarDiv61( uint32_t in )
//{
//#if 0
//    return( in / 61 );
//#else
//    uint32_t outTemp = 0;
//    uint32_t divResult = DIV_APPROX_61( in );
//    while( divResult >=1 )
//    {
//        outTemp += divResult;
//        in -= divResult * 61;
//        divResult = DIV_APPROX_61( in );
//    }
//    if( in >= 61 )
//    {
//        outTemp += 1;
//        in -= 61;
//    }
//    return outTemp;
//#endif
//}

static void CalendarDiv60( uint32_t in, uint32_t* out, uint32_t* remainder )
{
#if 0
    *remainder = in % 60;
    *out       = in / 60;
#else
    uint32_t outTemp = 0;
    uint32_t divResult = DIV_APPROX_60( in );

    while( divResult >=1 )
    {
        outTemp += divResult;
        in -= divResult * 60;
        divResult = DIV_APPROX_60( in );
    }
    if( in >= 60 )
    {
        outTemp += 1;
        in -= 60;
    }
    *remainder = in;
    *out = outTemp;
#endif
}

static void TimeStampsUpdate()
{    
    TimerTime_t old =  RtcGetTimerContext(); 
    TimerTime_t now =  RtcSetTimerContext(); 
    uint32_t DeltaContext = (uint32_t)(now - old);
    
    TimerEvent_t* cur = TimerListHead;
    while(cur) {
        if (cur->Timestamp > DeltaContext)
            cur->Timestamp -= DeltaContext;
        else
            cur->Timestamp = 0 ;
        cur = cur->Next;
    }
}

void TimerInit( TimerEvent_t *obj, void ( *callback )( void ) )
{
  obj->Timestamp = 0;
  obj->ReloadValue = 0;
  obj->IsRunning = false;
  obj->Callback = callback;
  obj->Next = NULL;
}

void TimerStart( TimerEvent_t *obj )
{
    uint32_t elapsedTime = 0;
    BoardDisableIrq();

    if( ( obj == NULL ) || ( TimerExists( obj ) == true ) )
    {
        BoardEnableIrq();
        return;
    }

    obj->Timestamp = obj->ReloadValue;
    obj->IsRunning = false;

    if( TimerListHead == NULL )
    {
        RtcSetTimerContext();
        TimerInsertNewHeadTimer( obj ); // insert a timeout at now+obj->Timestamp
    }
    else 
    {
        elapsedTime = RtcGetElapsedTime();
        obj->Timestamp += elapsedTime;
        if( obj->Timestamp < TimerListHead->Timestamp )
        {
            TimeStampsUpdate();
            obj->Timestamp -= elapsedTime;
            TimerInsertNewHeadTimer( obj);
        }
        else
        {
            TimerInsertTimer( obj);
        }
    }
    BoardEnableIrq();
}

static void TimerInsertTimer( TimerEvent_t *obj)
{
  TimerEvent_t* cur = TimerListHead;
  TimerEvent_t* next = TimerListHead->Next;

  while (cur->Next != NULL )
  {  
    if( obj->Timestamp  > next->Timestamp )
    {
        cur = next;
        next = next->Next;
    }
    else
    {
        cur->Next = obj;
        obj->Next = next;
        return;

    }
  }
  cur->Next = obj;
  obj->Next = NULL;
}

static void TimerInsertNewHeadTimer( TimerEvent_t *obj )
{
  TimerEvent_t* cur = TimerListHead;

  if( cur != NULL )
  {
    cur->IsRunning = false;
  }

  obj->Next = cur;
  TimerListHead = obj;
  TimerSetTimeout( TimerListHead );
}

void TimerIrqHandler( void )
{
    TimerEvent_t* cur;

    //update timer context for callbacks
    TimeStampsUpdate();
    /* execute imediately the alarm callback */
    if ( TimerListHead != NULL ) {
        cur = TimerListHead;
        TimerListHead = TimerListHead->Next;
        exec_cb( cur->Callback );
    }

    // remove all the expired object from the list
    while( ( TimerListHead != NULL ) && ( (TimerListHead->Timestamp < RtcGetElapsedTime(  )) || TimerListHead->Timestamp==0  ))
    {
        cur = TimerListHead;
        TimerListHead = TimerListHead->Next;
        exec_cb( cur->Callback );
    }
    
    //update timestamps after callbacks
    TimeStampsUpdate();
   
    /* start the next TimerListHead if it exists AND NOT running */
    if(( TimerListHead != NULL ) && (TimerListHead->IsRunning == false)) {
        TimerSetTimeout( TimerListHead );
    }
}

void TimerStop( TimerEvent_t *obj ) 
{
    BoardDisableIrq();

    TimerEvent_t* prev = TimerListHead;
    TimerEvent_t* cur = TimerListHead;

    // List is empty or the Obj to stop does not exist 
    if( ( TimerListHead == NULL ) || ( obj == NULL ) )
    {
        BoardEnableIrq();
        return;
    }

    if( TimerListHead == obj ) // Stop the Head                  
    {
        if( TimerListHead->IsRunning == true ) // The head is already running 
        {    
            if( TimerListHead->Next != NULL )
            {
                TimerListHead->IsRunning = false;
                TimerListHead = TimerListHead->Next;

                //update timestamps after stopping timer
                TimeStampsUpdate();
                TimerSetTimeout( TimerListHead );
            }
            else
            {
                RtcStopTimeout( );
                TimerListHead = NULL;
            }
        }
        else // Stop the head before it is started
        {   
            if( TimerListHead->Next != NULL )   
            {
                TimerListHead = TimerListHead->Next;
            }
            else
            {
                TimerListHead = NULL;
            }
        }
    }
    else // Stop an object within the list
    {      
        while( cur != NULL )
        {
            if( cur == obj )
            {
                if( cur->Next != NULL )
                {
                    cur = cur->Next;
                    prev->Next = cur;
                }
                else
                {
                    cur = NULL;
                    prev->Next = cur;
                }
                break;
            }
            else
            {
                prev = cur;
                cur = cur->Next;
            }
        }   
    }

    obj->IsRunning = false;
    BoardEnableIrq();
}  
  
static bool TimerExists( TimerEvent_t *obj )
{
    TimerEvent_t* cur = TimerListHead;

    while( cur != NULL )
    {
        if( cur == obj )
        {
            return true;
        }
        cur = cur->Next;
    }
    return false;  
}

void TimerReset( TimerEvent_t *obj )
{
  TimerStop( obj );
  TimerStart( obj );
}

void TimerSetValue( TimerEvent_t *obj, uint32_t value )
{
    TimerStop( obj );
    
    obj->Timestamp = value;
    obj->ReloadValue = value;
}

TimerTime_t TimerGetCurrentTime( void )
{
    return RtcGetTimerValue( ) + g_systime_ref;
}

TimerTime_t TimerGetElapsedTime( TimerTime_t savedTime )
{
    return (TimerGetCurrentTime() - savedTime);
}


static void TimerSetTimeout( TimerEvent_t *obj )
{
    obj->IsRunning = true;
    RtcSetTimeout(obj->Timestamp);
}

TimerTime_t TimerTempCompensation( TimerTime_t period, float temperature )
{
    (void)temperature;
    
    return period;
}

void TimerLowPowerHandler( void )
{
    //if( ( TimerListHead != NULL ) && ( TimerListHead->IsRunning == true ) )
//    {
        if( HasLoopedThroughMain < 5 )
        {
            HasLoopedThroughMain++;
        }
        else
        {
            HasLoopedThroughMain = 0;
            RtcEnterLowPowerStopMode( );
        }
//    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
