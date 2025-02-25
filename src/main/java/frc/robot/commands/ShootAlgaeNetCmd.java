package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.Pose2dSupplier;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaeIntake.elevation;
import frc.robot.subsystems.AlgaeIntake.shooting;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;
import frc.robot.subsystems.swerve.Swerve;

public class ShootAlgaeNetCmd extends SequentialCommandGroup {
  /**
   * @brief Creates a new ShootAlgaeNetCmd.
   *     <p>This command will auto drive to a given position, then set the elevator to the correct
   *     height, then set the algae arm to the correct angle, then shoot the algae ball into the
   *     net.
   * @param driveTrain swerve drive train subsystem
   * @param algaeIntake algae intake subsystem
   * @param elevator elevator subsystem
   * @param leds leds subsystem
   * @param target Pose2dSupplier target position to drive to
   */
  public ShootAlgaeNetCmd(
      Swerve driveTrain,
      AlgaeIntake algaeIntake,
      Elevator elevator,
      LEDs leds,
      Pose2dSupplier target) {
    addRequirements(algaeIntake);
    addRequirements(leds);
    addRequirements(elevator);
    addRequirements(driveTrain);

    addCommands(
        //        Commands.runOnce(() -> leds.SetPattern(Pattern.
        Commands.runOnce(() -> algaeIntake.setShootingSpeed(shooting.STORING)),
        Commands.runOnce(() -> driveTrain.drivetoTarget(target.getAsPose2d())),
        Commands.runOnce(() -> elevator.SetHeight(desiredHeight.NET)),
        Commands.runOnce(() -> leds.SetPattern(Pattern.SHOOTER)),
        Commands.runOnce(() -> algaeIntake.setShootingAngle(elevation.NET)),
        new WaitCommand(0.3),
        Commands.runOnce(() -> algaeIntake.setShootingSpeed(shooting.NET)),
        new WaitCommand(0.3),
        Commands.runOnce(() -> algaeIntake.setShootingSpeed(shooting.STORED)),
        Commands.runOnce(() -> leds.SetPattern(Pattern.IDLE)));
  }
}
