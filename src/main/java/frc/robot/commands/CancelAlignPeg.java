package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.shootSpeed;
import frc.robot.subsystems.swerve.Swerve;

public class CancelAlignPeg extends SequentialCommandGroup {
  public CancelAlignPeg(Swerve m_swerve, Shooter m_shooter, Elevator m_elevator) {
    addRequirements(m_swerve);
    addRequirements(m_shooter);
    addRequirements(m_elevator);

    addCommands(
        new InstantCommand(() -> m_swerve.disableDriveToTarget()),
        new InstantCommand(() -> m_shooter.closeBlocker()),
        new InstantCommand(() -> m_shooter.setShoot(shootSpeed.STOP)),
        new InstantCommand(() -> m_elevator.SetHeight(desiredHeight.LOW)));
  }
}
