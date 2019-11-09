/****************************************************************
**	OrangeBot Project
*****************************************************************
**        /
**       /
**      /
** ______ \
**         \
**          \
*****************************************************************
**	Project
*****************************************************************
**  Compiler Flage:
**	-fkeep-inline-functions
****************************************************************/

/****************************************************************
**	DESCRIPTION
****************************************************************
**
****************************************************************/

/****************************************************************
**	HISTORY VERSION
****************************************************************
**
****************************************************************/

/****************************************************************
**	KNOWN BUGS
****************************************************************
**
****************************************************************/

/****************************************************************
**	TODO
****************************************************************
**
****************************************************************/

/****************************************************************
**	INCLUDES
****************************************************************/

//Standard C Libraries
#include <cstdio>
#include <cstdlib>

//Standard C++ libraries
#include <iostream>
//#include <array>
//#include <vector>
//#include <queue>
//#include <string>
//#include <fstream>
//#include <chrono>
//#include <thread>

//OS Libraries
//#define _WIN32_WINNT 0x0500	//Enable GetConsoleWindow
//#include <windows.h>

//User Libraries
//Include user log trace
#define DEBUG_FILE
#include "at_utils.h"

#include "Pid_s16.h"


/****************************************************************
**	NAMESPACES
****************************************************************/

//Never use a whole namespace. Use only what you need from it.
using std::cout;
using std::endl;

/****************************************************************
**	DEFINES
****************************************************************/

//Number of steps to simulate
#define NUM_STEPS				1024

	//!TESTS - Activate only one

//Test the natural response of the system under test
//#define TEST_SYSTEM_RESPONSE

//Use input as reference and use a PID to drive the system under test
//#define TEST_PID

//Limit the output command of the PID
#define TEST_PID_SATCMD

//Test PID unlock error callback
//#define TEST_PID_SATCMD_ERR

/****************************************************************
**	MACROS
****************************************************************/

/****************************************************************
**	PROTOTYPES
****************************************************************/

extern bool pid_s16_test_bench( void );

//System under test is a simple low-pass filter
extern int16_t test_system( int16_t input, int16_t max_delta, int16_t min_delta );

//Return true when output does not change for a given threshold of samples
extern bool detect_stability( int16_t in, int stability_th );

//Error handler automatically called by the PID in case of error
extern void my_error_handler( void );

/****************************************************************
**	GLOBAL VARIABILES
****************************************************************/

DEBUG_VARS();
//User::Dummy my_class;

bool g_flag = false;

/****************************************************************
**	FUNCTIONS
****************************************************************/

/****************************************************************************
**	Function
**	main |
****************************************************************************/
//! @return bool |
//! @brief dummy method to copy the code
//! @details verbose description
/***************************************************************************/

int main()
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Start Debugging. Show function nesting level 0 and above
	DSTART( 0 );
	//Trace Enter main
	DENTER();

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	cout << "OrangeBot Projects\n";
	//print in the 'debug.log' file. works just like a fully featured printf
	DPRINT("OrangeBot Projects\n");

	pid_s16_test_bench();

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	//Trace Return from main
	DRETURN();
	//Stop Debugging
	DSTOP();

    return 0;
}	//end function: main

/****************************************************************************
**	Function
**	pid_s16_test_bench | bool
****************************************************************************/
//! @param f bool
//! @return bool |
//! @brief dummy method to copy the code
//! @details verbose description
/***************************************************************************/

bool pid_s16_test_bench( void )
{
	//Trace Enter with arguments
	DENTER_ARG("in: %d\n", 0);

	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	int t;
	int16_t in;
	int16_t out;

	int stability_th = 100;
	bool f_stability;


	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------
	//! @details algorithm:



		//----------------------------------------------------------------
		//	TEST_SYSTEM_RESPONSE
		//----------------------------------------------------------------
		//Test the natural response of the system under test

	#ifdef TEST_SYSTEM_RESPONSE

	in = 256;
	DPRINT("Test natural response on a constant input: %d", in);

	t = 0;
	f_stability = false;

	while ((t < NUM_STEPS) && (f_stability == false))
	{
		out = test_system( in, +100, -100 );
		f_stability = detect_stability( out, stability_th );
		t++;
	}
	if (f_stability == true)
	{
		DPRINT("stability reached after %d steps\n", t -stability_th);
	}


	#endif // TEST_SYSTEM_RESPONSE


		//----------------------------------------------------------------
		//	TEST_PID
		//----------------------------------------------------------------
		//Test the natural response of the system under test

	#ifdef TEST_PID

	//Construct a PID instance
	OrangeBot::Pid_s16 my_pid = OrangeBot::Pid_s16();

	int16_t reference = 256;
	int16_t cmd;

	my_pid.gain_kp() = 1024;
	my_pid.gain_kd() = -32;
	my_pid.gain_ki() = 64;

	DPRINT("Test natural response on a constant input: %d", reference);

	out = test_system( 0, +100, -100 );;
	t = 0;
	f_stability = false;

	while ((t < NUM_STEPS) && (f_stability == false))
	{
		cmd = my_pid.exe( reference, out );

		out = test_system( cmd, +100, -100 );
		f_stability = detect_stability( out, stability_th );

		DPRINT(" step: %5d | reference: %5d | command: %5d | output: %5d\n", t, reference, cmd, out );
		t++;
	}
	if (f_stability == true)
	{
		DPRINT("stability reached after %d steps\n", t -stability_th);
	}


	#endif // TEST_PID

		//----------------------------------------------------------------
		//	TEST_PID_SATCMD
		//----------------------------------------------------------------
		//Test the natural response of the system under test

	#ifdef TEST_PID_SATCMD

	//Construct a PID instance
	OrangeBot::Pid_s16 my_pid = OrangeBot::Pid_s16();

	int16_t reference = 128;
	int16_t cmd;

	my_pid.gain_kp() = 1024;
	my_pid.gain_kd() = -32;
	my_pid.gain_ki() = 64;
	my_pid.limit_cmd_max() = +256;
	my_pid.limit_cmd_min() = -256;


	DPRINT("Test natural response on a constant input: %d\n", reference);

	out = test_system( 0, +100, -100 );;
	t = 0;
	f_stability = false;

	while ((t < NUM_STEPS) && (f_stability == false))
	{
		cmd = my_pid.exe( reference, out );

		out = test_system( cmd, +100, -100 );
		f_stability = detect_stability( out, stability_th );

		DPRINT(" step: %5d | reference: %5d | command: %5d | output: %5d\n", t, reference, cmd, out );
		t++;
	}
	if (f_stability == true)
	{
		DPRINT("stability reached after %d steps\n", t -stability_th);
	}


	#endif // TEST_PID_SATCMD

		//----------------------------------------------------------------
		//	TEST_PID_SATCMD_ERR
		//----------------------------------------------------------------
		//Test the natural response of the system under test

	#ifdef TEST_PID_SATCMD_ERR

	//Construct a PID instance
	OrangeBot::Pid_s16 my_pid = OrangeBot::Pid_s16();

	int16_t reference = 256;
	int16_t cmd;

	my_pid.gain_kp() = -1024;
	my_pid.gain_kd() = 0;
	my_pid.gain_ki() = 0;
	my_pid.limit_cmd_max() = +256;
	my_pid.limit_cmd_min() = -256;

	my_pid.register_error_handler( 16, (void *)&my_error_handler );

	DPRINT("Test natural response on a constant input: %d\n", reference);

	out = test_system( 0, +100, -100 );;
	t = 0;
	f_stability = false;

	while ((t < NUM_STEPS) && (f_stability == false) && (g_flag == false))
	{
		cmd = my_pid.exe( reference, out );

		out = test_system( cmd, +100, -100 );
		f_stability = detect_stability( out, stability_th );

		DPRINT(" step: %5d | reference: %5d | command: %5d | output: %5d\n", t, reference, cmd, out );
		t++;
	}
	if (f_stability == true)
	{
		DPRINT("stability reached after %d steps\n", t -stability_th);
	}


	#endif // TEST_PID_SATCMD_ERR

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	//Trace Return vith return value
	DRETURN_ARG("out: %d\n", 0);

	return true; //OK
}	//end function: pid_s16_test_bench | bool

/****************************************************************************
**	Function
**	test_system |
****************************************************************************/
//! @param f bool
//! @return bool |
//! @brief System under test is a simple low-pass filter
//! @details
/***************************************************************************/

int16_t test_system( int16_t input, int16_t max_delta, int16_t min_delta )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//internal accumulator
	static int16_t acc = 0;

	int16_t delta;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Trace Enter with arguments
	DENTER_ARG("in: %d\n", input);

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------
	//! @details algorithm:

	//two samples derivative
	delta = input -acc;
	//Time constant
	delta = AT_DIVIDE_RTO( delta, 4);

	if (delta > max_delta)
	{
		delta = max_delta;

	}
	else if (delta < min_delta)
	{
		delta = min_delta;
	}

	acc += delta;

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	//Trace Return vith return value
	DRETURN_ARG("out: %d\n", acc);

	return acc; //OK
}	//end function: Dummy | test_system


/****************************************************************************
**	Function
**	detect_stability | int16_t, int
****************************************************************************/
//! @param f bool
//! @return bool |
//! @brief
//! @details Return true when output does not change for a given threshold of samples
/***************************************************************************/

//
bool detect_stability( int16_t in, int stability_th )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	static int16_t old = in;

	static int cnt = 0;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Trace Enter with arguments
	DENTER_ARG("in: %d\n", in);

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------
	//! @details algorithm:

	if (in == old)
	{
		cnt++;
		if (cnt >= stability_th)
		{
			DRETURN_ARG("Stability Detected!\n");
			//System is stable
			return true;
		}

	}
	else
	{
		cnt=0;
	}

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	old = in;

	//Trace Return vith return value
	DRETURN();

	return false; //OK
}	//end function: detect_stability | int16_t, int

/****************************************************************************
**	Function
**	Dummy | bool
****************************************************************************/
//! @param f bool
//! @return bool |
//! @brief Error handler automatically called by the PID in case of error
//! @details verbose description
/***************************************************************************/

void my_error_handler( void )
{
	//Trace Enter with arguments
	DENTER();

	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------



	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------
	//! @details algorithm:

	DPRINT("PID Unlock error detected!!! Manually stop PID\n");

	g_flag = true;

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	//Trace Return vith return value
	DRETURN();

	return; //OK
}	//end function: Dummy | bool

/****************************************************************************
**	Function
**	Dummy | bool
****************************************************************************/
//! @param f bool
//! @return bool |
//! @brief dummy method to copy the code
//! @details verbose description
/***************************************************************************/

bool f( bool f )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Trace Enter with arguments
	DENTER_ARG("in: %d\n", 0);

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------
	//! @details algorithm:

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	//Trace Return vith return value
	DRETURN_ARG("out: %d\n", 0);

	return false; //OK
}	//end function: Dummy | bool
