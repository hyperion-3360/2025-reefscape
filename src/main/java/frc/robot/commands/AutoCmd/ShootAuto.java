package frc.robot.commands.AutoCmd;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Auto.Pathfinding;
import frc.robot.Auto.Pathfinding.POI;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.shootSpeed;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;
import frc.robot.subsystems.swerve.Swerve;

public class ShootAuto extends SequentialCommandGroup {
  public ShootAuto(Shooter m_shooter, Elevator m_elevator, LEDs m_leds, Swerve m_swerve) {
    addRequirements(m_elevator, m_leds, m_swerve, m_shooter);
    addCommands(
        Pathfinding.goThere(new Pose2d(5.293, 2.626, Rotation2d.fromDegrees(120)))
            .alongWith(
                Commands.sequence(
                    Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.LOW)),
                    Commands.runOnce(() -> m_leds.SetPattern(Pattern.IDLE)))),
        Commands.runOnce(
                () -> m_swerve.drivetoTarget(RobotContainer.m_selector.getDesiredposeLeft()))
            .alongWith(
                Commands.runOnce(() -> m_shooter.openBlocker()),
                Commands.sequence(
                    new WaitUntilCommand(() -> Pathfinding.isCloseToPOI(POI.BRANCHES)),
                    Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.L4)),
                    new WaitUntilCommand(() -> m_elevator.isElevatorAtSetPoint()),
                    Commands.runOnce(() -> m_shooter.setShoot(shootSpeed.L4)),
                    new WaitUntilCommand(() -> !m_shooter.isCoralIn()),
                    new WaitCommand(0.2)))
            .finallyDo(
                () ->
                    Commands.sequence(
                        Commands.runOnce(() -> m_swerve.disableDriveToTarget()),
                        Commands.runOnce(() -> m_shooter.setShoot(shootSpeed.STOP)),
                        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.LOW)))));
  }
}
