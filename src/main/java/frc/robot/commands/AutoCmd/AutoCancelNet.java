package frc.robot.commands.AutoCmd;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.PathfindingV2;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaeIntake.elevation;
import frc.robot.subsystems.AlgaeIntake.shooting;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;
import frc.robot.subsystems.swerve.Swerve;

public class AutoCancelNet extends SequentialCommandGroup {
  public AutoCancelNet(
      AlgaeIntake m_algaeIntake,
      LEDs m_leds,
      Elevator m_elevator,
      Swerve m_swerve,
      PathfindingV2 m_pathfinding) {
    addRequirements(m_algaeIntake);
    addRequirements(m_leds);
    addRequirements(m_elevator);
    addRequirements(m_swerve);
    addCommands(
        new InstantCommand(() -> m_swerve.disableDriveToTarget()),
        new InstantCommand(() -> m_algaeIntake.setShootingAngle(elevation.STORED)),
        new InstantCommand(() -> m_algaeIntake.setShootingSpeed(shooting.STORED))
            .unless(() -> m_algaeIntake.sensorTriggered()),
        new InstantCommand(() -> m_algaeIntake.setShootingSpeed(shooting.STORING))
            .unless(() -> !m_algaeIntake.sensorTriggered()),
        new InstantCommand(() -> m_elevator.SetHeight(desiredHeight.LOW)),
        new InstantCommand(() -> m_leds.SetPattern(Pattern.IDLE))
            .unless(() -> m_algaeIntake.sensorTriggered()),
        new InstantCommand(() -> m_leds.SetPattern(Pattern.READY))
            .unless(() -> !m_algaeIntake.sensorTriggered()));
  }
}
