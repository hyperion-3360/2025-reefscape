package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.vision.Selection;

public class DriveAndIntakeCmd extends SequentialCommandGroup {

  /**
   * constructor to construct a algae intake command for the reef
   *
   * @param m_algaeIntake
   * @param m_leds
   * @param m_elevator
   * @param height
   */
  public DriveAndIntakeCmd(
      AlgaeIntake m_algaeIntake,
      LEDs m_leds,
      Elevator m_elevator,
      desiredHeight height,
      Swerve m_swerve,
      Selection m_selector) {
    addRequirements(m_algaeIntake);
    addRequirements(m_leds);
    addRequirements(m_elevator);
    addCommands(
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
        Commands.runOnce(() -> m_elevator.SetHeight(height)),
        Commands.runOnce(() -> m_algaeIntake.setShootingAngle(AlgaeIntake.elevation.FLOOR)),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.INTAKE)),
        Commands.runOnce(
            () -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.INTAKE), m_algaeIntake),
        new WaitUntilCommand(() -> m_swerve.targetReached()),
        new InstantCommand(() -> m_swerve.disableDriveToTarget()),
        Commands.runOnce(() -> m_swerve.drivetoTarget(m_selector.getDesiredCloseUpPoseAlgae())),
        new WaitUntilCommand(() -> m_algaeIntake.sensorTriggered()),
        new InstantCommand(() -> m_swerve.disableDriveToTarget()),
        new InstantCommand(() -> m_swerve.drivetoTarget(m_selector.getDesiredposeAlgae())),
        new WaitUntilCommand(() -> m_swerve.targetReached()),
        new InstantCommand(() -> m_swerve.disableDriveToTarget()),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.READY)));
  }

  public Command NoAlgaeCmd(Elevator m_elevator, AlgaeIntake m_algaeIntake, LEDs m_leds) {
    addRequirements(m_elevator);
    addRequirements(m_algaeIntake);
    addRequirements(m_leds);
    return Commands.sequence(
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.LOW)),
        Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.STORING)),
        Commands.runOnce(() -> m_algaeIntake.setShootingAngle(AlgaeIntake.elevation.STORED)),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.IDLE)));
  }
}
