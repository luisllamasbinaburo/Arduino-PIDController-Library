# Arduino-PIDController-Library
Arduino PID Library is an great work from Brett Beauregard. But it have not been updated since 2017. So this library is continues with the work in this Library, and adapt it to modern times and processors.

## Changes from Arduino-PID-Library
- PID class it's now call PIDController class
- Added PID namespace to avoid name collisions
- PIDController use templating, so both PIDController<float> or PIDController<double> are now posible
- Input, Output and SetPoint are not longer pointers, but internal PIDControllers variables
- `Calculate()` funtion is now name `Update()`
- Added `ForzeUpdate()` to update even when Interval have not been complete (not recommended, but required for some users)
- Kp, Ki and Kd factors are now wrapped into PIDParameters object
- #defines for PID options are now enums
- "Resolution" option added for working with `millis()` or `micros()`
- Major refactoring and code cleaning

### Change Log
#### Version 2.0.0

- Initial Version with modifications as listed in [Changes from Arduino-PID-Library.]

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
