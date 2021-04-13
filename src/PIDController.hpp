/***************************************************
PIDController - Version 2.2.0
Copyright (c) 2021 Luis Llamas (www.luisllamas.es)

This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

This library is licensed under a GPLv3 License
You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses>
****************************************************/

// BASED ON:
// (discontinued, thanks for your great work Brett):
/**********************************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/


#ifndef __PID_LIBRARY_h__
#define __PID_LIBRARY_h__

#define __PID_LIBRARY_VERSION__	2.2.0

#include "Arduino.h"

namespace PID
{
	//Parameter types for some of the functions below
	enum MODE { MANUAL, AUTOMATIC };
	enum DIRECTION { DIRECT, REVERSE };
	enum PROPORTIONAL_ON { MEASURE, ERROR };
	enum RESOLUTION { MILLIS, MICROS };

	template <typename T>
	class PIDParameters
	{
	public:
		PIDParameters() : Kp(T(0)), Ki(T(0)), Kd(T(0))
		{
		}

		PIDParameters(T kp, T ki, T kd) : Kp(kp), Ki(ki), Kd(kd)
		{
		}

		T Kp;
		T Ki;
		T Kd;

		bool HasNegatives()
		{
			return (Kp < 0 || Ki < 0 || Kd < 0);
		}

		void Set(PIDParameters<T> parameters)
		{
			Set(parameters.Kp, parameters.Ki, parameters.Kd);
		}

		void Set(T kp, T ki, T kd)
		{
			Kp = kp;
			Ki = ki;
			Kd = kd;
		}

		void Invert()
		{
			Kp = (0 - Kp);
			Ki = (0 - Ki);
			Kd = (0 - Kd);
		}

		static PIDParameters Linear(PIDParameters from, PIDParameters to, float t)
		{
			return
			{
				from.Kp + (to.Kp - from.Kp) * t,
				from.Ki + (to.Ki - from.Ki) * t,
				from.Kd + (to.Kd - from.Kd) * t,
			};
		}
	};


	template <typename T>
	class PIDParametersAdaptative
	{
	public:
		PIDParametersAdaptative(T near_distance, PIDParameters<T> near_parameter, T far_distance, PIDParameters<T> far_parameter)
			: NearDistance(near_distance), NearParameter(near_parameter), FarDistance(far_distance), FarParameter(far_parameter)
		{
		}

		PIDParameters<T> NearParameter;
		T NearDistance;

		PIDParameters<T> FarParameter;
		T FarDistance;

		PIDParameters<T> GetAt(T distance)
		{
			distance = distance >= 0 ? distance : -distance;
			if (distance < NearDistance) return NearParameter;
			if (distance > FarDistance) return FarParameter;

			auto relative_distance = (distance - NearDistance) / (FarDistance - NearDistance);
			return PIDParameters<T>::Linear(NearParameter, FarParameter, relative_distance);
		}
	};


	template <typename T>
	class PIDController
	{
	public:
		PIDController(PIDParameters<T> pid_parameters, DIRECTION direction = DIRECTION::DIRECT)
		{
			SetDirection(direction);
			SetTunings(pid_parameters);

			Last_time = GetTime() - Sample_time;
		}

		T Input;
		T Output;
		T Setpoint;


		// **********************************************************************************
		// * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
		// * when the transition from manual to auto occurs, the controller is
		// * automatically initialized
		// **********************************************************************************
		void SetMode(const MODE mode)
		{
			if (Mode == mode) return;
			Mode = mode;

			if (IsTurnedOn())
				Initialize();
		}

		void SetProportionalOn(const PROPORTIONAL_ON proportional_on)
		{
			Proportional_On = proportional_on;
		}

		// **********************************************************************************
		// *  This function will be used far more often than SetInputLimits.  while
		// *  the input to the controller will generally be in the 0-1023 range (which is
		// *  the default already,)  the output will be a little different.  maybe they'll
		// *  be doing a time window and will need 0-8000 or something.  or maybe they'll
		// *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
		// *  here.
		// **********************************************************************************
		void SetOutputLimits(const T min_limit, const T max_limit)
		{
			if (min_limit >= max_limit) return;
			Output_min = min_limit;
			Output_max = max_limit;

			if (IsTurnedOn())
			{
				Output = Clamp(Output, Output_min, Output_max);
				Output_sum = Clamp(Output_sum, Output_min, Output_max);
			}
		}


		// **********************************************************************************
		// * This function allows the controller's dynamic performance to be adjusted.
		// * it's called automatically from the constructor, but tunings can also
		// * be adjusted on the fly during normal operation
		// **********************************************************************************
		void SetTunings(PIDParameters<T> pid_parameters, const PROPORTIONAL_ON proportional_on = PROPORTIONAL_ON::MEASURE)
		{
			if (pid_parameters.HasNegatives()) return;

			Proportional_On = proportional_on;

			Parameters_original.Set(pid_parameters);

			T sample_time_in_sec = static_cast<T>(Sample_time) / (Resolution == RESOLUTION::MILLIS ? 1000 : 1000000);
			pid_parameters.Ki *= sample_time_in_sec;
			pid_parameters.Kd /= sample_time_in_sec;
			Parameters_computed.Set(pid_parameters);

			if (Direction == REVERSE)
				Parameters_computed.Invert();
		}


		// **********************************************************************************
		// * The PIDController will either be connected to a DIRECT acting process (+Output leads
		// * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
		// * know which one, because otherwise we may increase the output when we should
		// * be decreasing.  This is called from the constructor.
		// **********************************************************************************
		void SetDirection(const DIRECTION direction)
		{
			if (IsTurnedOn() && Direction != direction)
				Parameters_computed.Invert();

			Direction = direction;
		}


		// **********************************************************************************	
		// * sets the period, in Milliseconds, at which the calculation is performed
		// **********************************************************************************
		void SetSampleTime(unsigned long sample_time)
		{
			if (sample_time > 0)
			{
				T ratio = static_cast<T>(sample_time) / static_cast<T>(sample_time);
				Parameters_computed.Ki *= ratio;
				Parameters_computed.Kd /= ratio;
				sample_time = sample_time;
			}
		}

		void TurnOn() { SetMode(MODE::AUTOMATIC); }

		void TurnOff() { SetMode(MODE::MANUAL); }

		void Toggle()
		{
			if (IsTurnedOn()) TurnOff();
			else TurnOn();
		}


		// **********************************************************************************
		// * Just because you set the parameters.Kp=-1 doesn't mean it actually happened.  these
		// * functions query the internal state of the PIDController.  they're here for parameters_user.lay
		// * purposes.  this are the functions the PIDController Front-end uses for example
		// **********************************************************************************
		T GetError() { return Setpoint - Input; }

		T GetKp() { return Parameters_original.Kp; }

		T GetKi() { return Parameters_original.Ki; }

		T GetKd() { return Parameters_original.Kd; }

		T GetCorrectedKp() { return Parameters_computed.Kp; }

		T GetCorrectedKi() { return Parameters_computed.Ki; }

		T GetCorrectedKd() { return Parameters_computed.Kd; }

		MODE GetMode() const { return  Mode; }

		DIRECTION GetDirection() const { return Direction; }

		PROPORTIONAL_ON GetProportionalOn() const { return Proportional_On; }

		bool IsTurnedOn() const { return Mode == MODE::AUTOMATIC; }


		// **********************************************************************************
		// *   This, as they say, is where the magic happens.  this function should be called
		// *   every time "void loop()" executes.  the function will decide for itself whether a new
		// *   pid Output needs to be computed.  returns true when the output is computed,
		// *   false when nothing has been done.
		// **********************************************************************************
		bool Update()
		{
			if (IsTurnedOn() == false) return false;

			const unsigned long now = GetTime();
			const unsigned long time_change = (now - Last_time);
			if (time_change >= Sample_time)
			{
				UpdatePID();

				Last_time = now;
				return true;
			}
			else return false;
		}

		bool Update(T input)
		{
			Input = input;
			return Update();
		}

		void ForceUpdate()
		{
			if (IsTurnedOn() == false) return;

			UpdatePID();

			Last_time = GetTime();
		}

		void ForceUpdate(T input)
		{
			Input = input;
			return ForceUpdate();
		}


	private:
		// **********************************************************************************
		// *  does all the things that need to happen to ensure a bumpless transfer
		// *  from manual to automatic mode.
		// **********************************************************************************
		void Initialize()
		{
			Last_input = Input;
			Output_sum = Output;

			Output_sum = Clamp(Output_sum, Output_min, Output_max);
		}

		void UpdatePID()
		{
			T error = GetError();
			T diff_input = (Input - Last_input);
			Output_sum += (Parameters_computed.Ki * error);

			/*Add Proportional on Measurement, if P_ON_M is specified*/
			if (Proportional_On == PROPORTIONAL_ON::MEASURE) Output_sum -= Parameters_computed.Kp * diff_input;
			Output_sum = Clamp(Output_sum, Output_min, Output_max);

			/*Add Proportional on Error, if P_ON_E is specified*/
			if (Proportional_On == PROPORTIONAL_ON::ERROR) Output = Parameters_computed.Kp * error;
			else Output = 0;

			/*Compute Rest of PIDController Output*/
			Output += Output_sum - Parameters_computed.Kd * diff_input;
			Output = Clamp(Output, Output_min, Output_max);

			Last_input = Input;
		}

		unsigned long GetTime() const
		{
			return Resolution == MILLIS ? millis() : micros();
		}

		static constexpr T Clamp(T x, T min, T max)
		{
			return (x < min) ? min : ((x > max) ? max : x);
		}

		PIDParameters<T> Parameters_original;
		PIDParameters<T> Parameters_computed;

		T Output_min = 0;
		T Output_max = 255.0;
		T Output_sum;

		T Last_input;
		unsigned long Last_time;
		unsigned long Sample_time = 100;

		MODE Mode = MODE::MANUAL;
		DIRECTION Direction = DIRECTION::DIRECT;
		PROPORTIONAL_ON Proportional_On = PROPORTIONAL_ON::MEASURE;
		RESOLUTION Resolution = RESOLUTION::MILLIS;
	};
}
#endif