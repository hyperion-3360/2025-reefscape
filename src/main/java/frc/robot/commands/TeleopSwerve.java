package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private DoubleSupplier elevatorHeight;

  /**
   * Swerve drive command for tele operation
   *
   * @param s_Swerve swerve submodule instance
   * @param translationSup translation forward or backward
   * @param strafeSup strafe moving laterally in field oriented or changing orientation in robot
   *     centric
   * @param rotationSup rotation robot turning on itself
   * @param robotCentricSup robot centric (true) field oriented (false)
   */
  public TeleopSwerve(
      Swerve s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup,
      DoubleSupplier elevatorHeight) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.elevatorHeight = elevatorHeight;
  }

  @Override
  public void execute() {
    /* Get Values, Deadband*/
    double translationVal =
        MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
    double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
    double elevatorHeightVal = elevatorHeight.getAsDouble();

    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal)
            .times(Constants.Swerve.maxSpeed)
            .times(SwerveSpeedSlow(elevatorHeightVal)),
        (rotationVal * Constants.Swerve.maxAngularVelocity) * SwerveSpeedSlow(elevatorHeightVal),
        !robotCentricSup.getAsBoolean(),
        true);
  }

  public double SwerveSpeedSlow(double elevatorHeight) {
    var functionVal = (Math.pow(-0.8 * elevatorHeight, 3) / Constants.Swerve.maxSpeed) + 1;
    return MathUtil.clamp(functionVal, 0.2, 1);
  }
}
