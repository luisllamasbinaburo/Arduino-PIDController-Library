/***************************************************
PIDController - Version 2.0.0
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

#include <PIDController.hpp>

#define PIN_INPUT 0

PID::PIDParameters<double> conservative(1.0, 0.05, 0.25);
PID::PIDParameters<double> aggresive(4.0, 0.2, 1);
PID::PIDParametersAdaptative<double> adaptative(10, conservative, 100, aggresive);
PID::PIDController<double> pidController(aggresive);

void setup()
{
	pidController.Input = analogRead(PIN_INPUT);
	pidController.Setpoint = 100;

	pidController.TurnOn();
}

void loop()
{
	pidController.Input = 0;

	const double gap = abs(pidController.GetError());
	auto parameter = adaptative.Get(gap);
	pidController.SetTunings(parameter);

	pidController.Update();
}