package frc.robot.commands.AutoCmd;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Auto.Pathfinding;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.shootSpeed;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;
import frc.robot.subsystems.swerve.Swerve;

public class AutoShootBranch extends SequentialCommandGroup {
  public AutoShootBranch(
      Shooter m_shooter, Elevator m_elevator, LEDs m_leds, Swerve m_swerve, Pose2d branch) {
    addRequirements(m_elevator, m_leds, m_swerve, m_shooter);
    addCommands(
        Pathfinding.goThere(branch)
            .alongWith(
                Commands.sequence(
                    Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.LOW)),
                    Commands.runOnce(() -> m_leds.SetPattern(Pattern.IDLE)),
                    Commands.runOnce(() -> m_shooter.openBlocker()),
                    new WaitUntilCommand(() -> Pathfinding.isCloseToPOI(branch)),
                    Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.L4)),
                    new WaitUntilCommand(() -> m_elevator.isElevatorAtSetPoint()),
                    Commands.runOnce(() -> m_shooter.setShoot(shootSpeed.L4)),
                    new WaitUntilCommand(() -> !m_shooter.isCoralIn()),
                    new WaitCommand(0.2))),
        Commands.runOnce(() -> m_shooter.setShoot(shootSpeed.STOP)),
        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.LOW))
            .finallyDo(
                () ->
                    Commands.sequence(
                        Commands.runOnce(() -> m_swerve.disableDriveToTarget()),
                        Commands.runOnce(() -> m_shooter.setShoot(shootSpeed.STOP)),
                        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.LOW)))));
  }
}
