package frc.lib.util;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class SwerveElevatorSlowDownFunc {

  /**
   * uses a function to slow down the swerves in function of the elevator height
   *
   * @return a percentage of the slow down clamped at a minimum of 20 percent
   */
  public static double calculate(DoubleSupplier elevatorPos) {
    // speed slow down so that the elevator does not break when going at high speed
    return MathUtil.clamp(
        (Math.pow(-0.47 * elevatorPos.getAsDouble(), 5) / Constants.Swerve.maxSpeed) + 1, 0.4, 1);
  }
}
