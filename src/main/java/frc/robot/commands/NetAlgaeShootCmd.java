package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Auto.PathfindingV2;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaeIntake.elevation;
import frc.robot.subsystems.AlgaeIntake.shooting;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;
import frc.robot.subsystems.swerve.Swerve;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class NetAlgaeShootCmd extends SequentialCommandGroup {
  private double targetX;
  private Rotation2d targetRotation;
  private BooleanSupplier forbidenZone;
  private DoubleSupplier targetY;

  public NetAlgaeShootCmd(
      AlgaeIntake m_algaeIntake,
      LEDs m_leds,
      Elevator m_elevator,
      Swerve m_swerve,
      PathfindingV2 m_pathfinding) {
    addRequirements(m_algaeIntake);
    addRequirements(m_leds);
    addRequirements(m_elevator);
    addRequirements(m_swerve);

    // if we don't have any alliance assume blue alliance
    try {
      targetX = DriverStation.getAlliance().get().equals(Alliance.Blue) ? 7.3 : 10.3;
      targetRotation =
          DriverStation.getAlliance().get().equals(Alliance.Blue)
              ? Rotation2d.kZero
              : Rotation2d.k180deg;
      forbidenZone =
          DriverStation.getAlliance().get().equals(Alliance.Blue)
              ? () -> 4.62 < m_swerve.getPose().getY()
              : () -> 3.43 > m_swerve.getPose().getY();
      targetY =
          () ->
              forbidenZone.getAsBoolean()
                  ? m_swerve.getPose().getY()
                  : DriverStation.getAlliance().get().equals(Alliance.Blue) ? 4.7 : 3.2;
    } catch (Exception e) {
      targetX = 7.3;
      targetRotation = Rotation2d.kZero;
      forbidenZone = () -> 4.62 > m_swerve.getPose().getY();
    }

    addCommands(
        new ParallelCommandGroup(
            new DeferredCommand(
                () ->
                    m_pathfinding.goThere(
                        () -> new Pose2d(targetX, targetY.getAsDouble(), targetRotation)),
                Set.of(m_swerve)),
            new WaitUntilCommand(
                    () ->
                        m_pathfinding.isCloseTo(
                            new Pose2d(targetX, targetY.getAsDouble(), targetRotation), 1))
                .andThen(
                    Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
                    Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.NET)),
                    Commands.runOnce(() -> m_algaeIntake.setShootingAngle(elevation.NET)))),
        new WaitCommand(1.3),
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

  public Command cancelNet(
      AlgaeIntake m_algaeIntake,
      LEDs m_leds,
      Elevator m_elevator,
      Swerve m_swerve,
      PathfindingV2 m_pathfinding) {
    addRequirements(m_algaeIntake);
    addRequirements(m_leds);
    addRequirements(m_elevator);
    addRequirements(m_swerve);
    return new SequentialCommandGroup(
        new InstantCommand(() -> m_algaeIntake.setShootingAngle(elevation.STORED)),
        new InstantCommand(() -> m_algaeIntake.setShootingSpeed(shooting.STORED))
            .unless(() -> m_algaeIntake.sensorTriggered()),
        new InstantCommand(() -> m_algaeIntake.setShootingSpeed(shooting.STORING))
            .unless(() -> !m_algaeIntake.sensorTriggered()),
        new InstantCommand(() -> m_elevator.SetHeight(desiredHeight.LOW)),
        new InstantCommand(() -> m_swerve.stopSwerve()),
        new InstantCommand(() -> m_leds.SetPattern(Pattern.IDLE))
            .unless(() -> m_algaeIntake.sensorTriggered()),
        new InstantCommand(() -> m_leds.SetPattern(Pattern.READY))
            .unless(() -> !m_algaeIntake.sensorTriggered()));
  }
}
