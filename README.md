# Arduino-PIDController-Library
Arduino PID Library is an great work from Brett Beauregard. But it have not been updated since 2017. So this library is continues with the work in this Library, and adapt it to modern times and processors.

## Usage
This is how Basic example works with PIDController Library

```c++
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
```

## Changes from Arduino-PID-Library
- PID class it's now named PIDController class
- Added PID namespace to avoid name collisions
- PIDController uses templating, so both `PIDController<float>` or `PIDController<double>` are now posible
- Input, Output and SetPoint are not longer pointers, but internal PIDControllers variables
- `Calculate()` funtion is now name `Update()`
- Added `Update(T input)` that simultaneous set Input and run `Update()` method
- Added `ForzeUpdate()` to update even when Interval have not been complete (not recommended, but required for some users)
- Kp, Ki and Kd factors are now wrapped into `PIDParameters` class
- #defines for PID options are now enums
- "Resolution" option added for working with `millis()` or `micros()`
- Major refactoring and code cleaning


------

### Original README (Arduino PID)

```
***************************************************************
* Arduino PID Library - Version 1.2.1
* by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
*
* This Library is licensed under the MIT License
***************************************************************
```

 - For an ultra-detailed explanation of why the code is the way it is, please visit:
   http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

 - For function documentation see:  http://playground.arduino.cc/Code/PIDLibrary

------
