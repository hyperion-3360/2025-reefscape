package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
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
  private double allianceinverter = 1;

  /**
   * Swerve drive command for teleoperation
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

    if (DriverStation.getAlliance().get() == Alliance.Red) {
      allianceinverter = -1;
    }
    /* Get Values, Deadband*/
    double translationVal =
        MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband)
            * allianceinverter;
    double strafeVal =
        MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband) * allianceinverter;
    double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
    double elevatorHeightVal = elevatorHeight.getAsDouble();

    translationVal *= SwerveSpeedSlow(elevatorHeightVal);
    strafeVal *= SwerveSpeedSlow(elevatorHeightVal);

    if (Climber.climberActivated()) {
      translationVal *= 0.1;
      strafeVal *= 0.1;
      rotationVal *= 0.1;
    }

    SmartDashboard.putNumber("translation v", translationVal);
    SmartDashboard.putNumber("strafe v", strafeVal);
    SmartDashboard.putNumber("function value", SwerveSpeedSlow(elevatorHeightVal));
    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        (rotationVal * Constants.Swerve.maxAngularVelocity),
        !robotCentricSup.getAsBoolean(),
        true);
  }

  public double SwerveSpeedSlow(double elevatorHeight) {
    var functionVal = (Math.pow(-1.5 * elevatorHeight, 3) / Constants.Swerve.maxSpeed) + 1;
    return MathUtil.clamp(functionVal, 0.2, 1);
  }
}
