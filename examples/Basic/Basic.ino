
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
#define PIN_OUTPUT 3

PID::PIDParameters<double> parameters(4.0, 0.2, 1);
PID::PIDController<double> pidController(parameters);

void setup()
{
  pidController.Input = analogRead(PIN_INPUT);
  pidController.Setpoint = 100;

  pidController.TurnOn();
}

void loop()
{
  pidController.Input = analogRead(PIN_INPUT);
  pidController.Update();

  analogWrite(PIN_OUTPUT, pidController.Output);
}