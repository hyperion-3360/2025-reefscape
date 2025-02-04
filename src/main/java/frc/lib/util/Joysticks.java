package frc.lib.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import java.util.function.DoubleSupplier;

public class Joysticks {

  /***
   * conditionJoystick
   * Condition a joystick axis value given a slewrate limiter and deadband
   * @param axis axis to condition
   * @param limiter slewrate limiter (to smooth the rate of changed
   * @see https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/slew-rate-limiter.html)
   * @param deadband deadband to suppress noise around the 0 of a joystick axis
   * @see https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/MathUtil.html#applyDeadband(double,double)
   * @param flipAxis whether to flip the value read (example, positive y is down, but we usually want up)
   * @return the conditioned value
   */
  public static double conditionJoystick(
      DoubleSupplier axis, SlewRateLimiter limiter, double deadband, boolean flipAxis) {
    double valueToCondition = axis.getAsDouble();
    double conditionedVal =
        Math.signum(valueToCondition)
            * Math.pow(limiter.calculate(MathUtil.applyDeadband(valueToCondition, deadband)), 2);
    return flipAxis ? -conditionedVal : conditionedVal;
  }
}
