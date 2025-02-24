package frc.robot.commands.AutoCmd;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Auto.Pathfinding;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;
import frc.robot.subsystems.swerve.Swerve;

public class ShootAuto extends SequentialCommandGroup {
  public ShootAuto(Shooter m_shooter, Elevator m_elevator, LEDs m_leds, Swerve m_swerve) {
    addCommands(
        Pathfinding.goThere(new Pose2d(3.648, 2.420, Rotation2d.fromDegrees(60)))
            .alongWith(
                Commands.sequence(
                    Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
                    Commands.runOnce(() -> m_shooter.openBlocker()),
                    Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.L2)),
                    new WaitCommand(1.5),
                    Commands.runOnce(() -> m_leds.SetPattern(Pattern.IDLE)))),
        Commands.runOnce(
                () -> m_swerve.drivetoTarget(RobotContainer.m_selector.getDesiredposeRight()))
            .alongWith(
                Commands.sequence(Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.L4)))));
  }
}
