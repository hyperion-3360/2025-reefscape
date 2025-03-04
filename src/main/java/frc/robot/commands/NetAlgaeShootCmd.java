package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaeIntake.elevation;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;
import frc.robot.subsystems.swerve.Swerve;

public class NetAlgaeShootCmd extends SequentialCommandGroup {

  public NetAlgaeShootCmd(
      AlgaeIntake m_algaeIntake, LEDs m_leds, Elevator m_elevator, Swerve m_swerve) {
    addRequirements(m_algaeIntake);
    addRequirements(m_leds);
    addRequirements(m_elevator);
    addCommands(
        // new DeferredCommand(
        //         () ->
        //             Pathfinding.goThere(
        //                 () -> new Pose2d(7.5, m_swerve.getPose().getY(), Rotation2d.kZero)),
        //         Set.of())
        //     .alongWith(
        //         new WaitUntilCommand(
        //                 () ->
        //                     Pathfinding.isCloseTo(
        //                         new Pose2d(7.5, m_swerve.getPose().getY(), Rotation2d.kZero),
        // 1.3))
        //             .andThen(
        //                 Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
        //                 Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.NET)))),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.NET)),
        Commands.runOnce(() -> m_algaeIntake.setShootingAngle(elevation.NET)),
        new WaitCommand(0.9),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.SHOOTER)),
        Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.INTAKE)),
        new WaitCommand(0.3),
        Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.NET)),
        new WaitUntilCommand(() -> !m_algaeIntake.sensorTriggered()),
        new WaitCommand(0.3),
        Commands.runOnce(() -> m_algaeIntake.setShootingAngle(AlgaeIntake.elevation.STORED)),
        Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.STORED)),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.LOW)),
        new WaitCommand(0.5),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.IDLE)));
  }
}
