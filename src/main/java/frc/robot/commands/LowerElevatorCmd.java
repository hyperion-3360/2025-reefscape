package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaeIntake.elevation;
import frc.robot.subsystems.AlgaeIntake.shooting;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.shootSpeed;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;
import frc.robot.subsystems.swerve.Swerve;

public class LowerElevatorCmd extends SequentialCommandGroup {
  public LowerElevatorCmd(
      Elevator m_elevator,
      LEDs m_leds,
      Shooter m_shooter,
      AlgaeIntake m_algaeIntake,
      Swerve m_swerve) {
    addRequirements(m_elevator);
    addRequirements(m_leds);
    addRequirements(m_shooter);
    addRequirements(m_algaeIntake);
    addCommands(
        new InstantCommand(() -> m_swerve.disableDriveToTarget()),
        Commands.runOnce(() -> m_shooter.setShoot(shootSpeed.STOP)),
        new ConditionalCommand(
            Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(shooting.STORING)),
            Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(shooting.STORED)),
            () -> m_algaeIntake.sensorTriggered()),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
        Commands.runOnce(() -> m_shooter.closeBlocker()),
        Commands.runOnce(() -> m_shooter.stop()),
        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.LOW)),
        Commands.runOnce(() -> m_algaeIntake.setShootingAngle(elevation.NET)),
        new WaitCommand(1.5),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.IDLE)));
  }
}
