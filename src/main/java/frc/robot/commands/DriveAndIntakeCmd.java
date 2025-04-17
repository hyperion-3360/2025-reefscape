package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.MinuteMoveCmd.OffsetDir;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.vision.Vision;
import java.util.function.BooleanSupplier;

public class DriveAndIntakeCmd extends SequentialCommandGroup {
  private boolean bManualMode = false;
  private BooleanSupplier isManualMode = () -> bManualMode;
  private BooleanSupplier isAlgaeL2 = () -> true;

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
      Vision m_vision) {
    MinuteMoveCmd backTrack = new MinuteMoveCmd(m_swerve, 0.5, 0.8, OffsetDir.BACK);
    addRequirements(m_algaeIntake);
    addRequirements(m_leds);
    addRequirements(m_elevator);
    addCommands(
        new ConditionalCommand(
            new ProxyCommand(
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
                        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.ALGAEL2)),
                        Commands.runOnce(
                            () -> m_algaeIntake.setShootingAngle(AlgaeIntake.elevation.FLOOR)),
                        Commands.runOnce(() -> m_leds.SetPattern(Pattern.INTAKE)),
                        Commands.runOnce(
                            () -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.INTAKE),
                            m_algaeIntake),
                        new WaitCommand(1.0),
                        new WaitUntilCommand(() -> m_algaeIntake.sensorTriggered()),
                        Commands.runOnce(() -> m_leds.SetPattern(Pattern.READY))),
                    new SequentialCommandGroup(
                        Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
                        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.ALGAEL3)),
                        Commands.runOnce(
                            () -> m_algaeIntake.setShootingAngle(AlgaeIntake.elevation.FLOOR)),
                        Commands.runOnce(() -> m_leds.SetPattern(Pattern.INTAKE)),
                        Commands.runOnce(
                            () -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.INTAKE),
                            m_algaeIntake),
                        new WaitCommand(1.0),
                        new WaitUntilCommand(() -> m_algaeIntake.sensorTriggered()),
                        Commands.runOnce(() -> m_leds.SetPattern(Pattern.READY))),
                    isAlgaeL2)),
            new SequentialCommandGroup(
                Commands.runOnce(
                    () -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.STORING)),
                new InstantCommand(
                    () ->
                        m_swerve.drivetoTarget(
                            m_vision.getDesiredPoseAlgae(() -> m_swerve.getPose()))),
                new WaitUntilCommand(() -> m_swerve.targetReached()),
                Commands.runOnce(() -> m_elevator.SetHeight(m_vision.getAlgaeHeight())),
                Commands.runOnce(() -> m_algaeIntake.setShootingAngle(AlgaeIntake.elevation.FLOOR)),
                Commands.runOnce(() -> m_leds.SetPattern(Pattern.INTAKE)),
                Commands.runOnce(
                    () -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.INTAKE),
                    m_algaeIntake),
                new InstantCommand(() -> m_swerve.disableDriveToTarget()),
                new WaitCommand(0.3)
                    .unless(() -> m_vision.getAlgaeHeight().equals(desiredHeight.ALGAEL2)),
                new InstantCommand(
                    () -> m_swerve.drivetoTarget(m_vision.getDesiredCloseUpPoseAlgae())),
                new WaitUntilCommand(() -> m_algaeIntake.sensorTriggered()),
                new InstantCommand(() -> m_swerve.disableDriveToTarget()),
                backTrack,
                Commands.runOnce(() -> m_leds.SetPattern(Pattern.READY)),
                Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.LOW)),
                new WaitCommand(0.4),
                Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.STORED)),
                Commands.runOnce(
                    () -> m_algaeIntake.setShootingAngle(AlgaeIntake.elevation.STORED)),
                new WaitCommand(0.2),
                Commands.runOnce(
                    () -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.STORING))),
            isManualMode));
  }

  public Command NoAlgaeCmd(
      Elevator m_elevator, AlgaeIntake m_algaeIntake, LEDs m_leds, Swerve m_swerve) {
    addRequirements(m_elevator);
    addRequirements(m_algaeIntake);
    addRequirements(m_leds);
    return Commands.sequence(
        new InstantCommand(() -> m_swerve.disableDriveToTarget()),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.LOW)),
        Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.STORED))
            .unless(() -> m_algaeIntake.sensorTriggered()),
        Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.STORING))
            .unless(() -> !m_algaeIntake.sensorTriggered()),
        Commands.runOnce(() -> m_algaeIntake.setShootingAngle(AlgaeIntake.elevation.STORED)),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.IDLE)));
  }

  public void toggleManualMode() {

    bManualMode = !bManualMode;
    SmartDashboard.putBoolean("intakeAlgaeReef", isManualMode.getAsBoolean());
  }

  public void toggleL2() {
    if (isAlgaeL2.getAsBoolean() == true) {
      isAlgaeL2 = () -> false;
    } else {
      isAlgaeL2 = () -> true;
    }
  }
}
