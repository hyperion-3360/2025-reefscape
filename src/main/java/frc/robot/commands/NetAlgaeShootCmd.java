package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Auto.PathfindingV2;
import frc.robot.commands.MinuteMoveCmd.OffsetDir;
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
  private double forbiddenZoneRed = 3.43;
  private double forbiddenZoneBlue = 4.62;
  private double closeZoneRed = 3.08;
  private double closeZoneBlue = 4.90;
  private BooleanSupplier isManualMode = () -> false;

  private MinuteMoveCmd deadStop;

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
    deadStop = new MinuteMoveCmd(m_swerve, 0.06, 0.3, OffsetDir.RIGHT);
    // if we don't have any alliance assume blue alliance
    try {
      targetX = DriverStation.getAlliance().get().equals(Alliance.Blue) ? 7.2 : 10.4;
      targetRotation =
          DriverStation.getAlliance().get().equals(Alliance.Blue)
              ? Rotation2d.kZero
              : Rotation2d.k180deg;
      forbidenZone =
          DriverStation.getAlliance().get().equals(Alliance.Blue)
              ? () -> forbiddenZoneBlue < m_swerve.getPose().getY()
              : () -> forbiddenZoneRed > m_swerve.getPose().getY();
    } catch (Exception e) {
      targetX = 7.3;
      targetRotation = Rotation2d.kZero;
      forbidenZone = () -> 4.62 > m_swerve.getPose().getY();
    }

    addCommands(
        new ConditionalCommand(
            new SequentialCommandGroup(
                new InstantCommand(() -> m_elevator.SetHeight(desiredHeight.NET)),
                new WaitCommand(1.3),
                Commands.runOnce(() -> m_leds.SetPattern(Pattern.SHOOTER)),
                Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.INTAKE)),
                new WaitCommand(0.2),
                Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.NET)),
                new WaitUntilCommand(() -> !m_algaeIntake.sensorTriggered()),
                new WaitCommand(0.1),
                Commands.runOnce(
                    () -> m_algaeIntake.setShootingAngle(AlgaeIntake.elevation.STORED)),
                Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.STORED)),
                Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
                Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.LOW)),
                new WaitCommand(0.5),
                Commands.runOnce(() -> m_leds.SetPattern(Pattern.IDLE))),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    // we need to put the ternary operators inside the deffered command so that the
                    // position
                    // actually changes
                    new DeferredCommand(
                        () ->
                            m_pathfinding.goThere(
                                () ->
                                    new Pose2d(
                                        targetX,
                                        forbidenZone.getAsBoolean()
                                            ? m_swerve.getPose().getY()
                                            : DriverStation.getAlliance()
                                                    .get()
                                                    .equals(Alliance.Blue)
                                                ? closeZoneBlue
                                                : closeZoneRed,
                                        targetRotation)),
                        Set.of(m_swerve)),
                    // same thing for the wait until
                    new WaitUntilCommand(
                            () ->
                                m_pathfinding.isCloseTo(
                                    new Pose2d(
                                        targetX,
                                        forbidenZone.getAsBoolean()
                                            ? m_swerve.getPose().getY()
                                            : DriverStation.getAlliance()
                                                    .get()
                                                    .equals(Alliance.Blue)
                                                ? closeZoneBlue
                                                : closeZoneRed,
                                        targetRotation),
                                    1.4))
                        .andThen(
                            Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
                            Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.NET)),
                            Commands.runOnce(() -> m_algaeIntake.setShootingAngle(elevation.NET)),
                            Commands.runOnce(
                                () -> m_algaeIntake.setShootingSpeed(shooting.SUPERSTORE)))),
                deadStop,
                new WaitCommand(0.08),
                new InstantCommand(
                    () ->
                        m_swerve.drivetoTarget(
                            new Pose2d(
                                targetX,
                                forbidenZone.getAsBoolean()
                                    ? m_swerve.getPose().getY()
                                    : DriverStation.getAlliance().get().equals(Alliance.Blue)
                                        ? closeZoneBlue
                                        : closeZoneRed,
                                targetRotation))),
                new WaitCommand(0.6),
                new InstantCommand(() -> m_swerve.disableDriveToTarget()),
                Commands.runOnce(() -> m_leds.SetPattern(Pattern.SHOOTER)),
                Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.INTAKE)),
                new WaitCommand(0.2),
                Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.NET)),
                new WaitUntilCommand(() -> !m_algaeIntake.sensorTriggered()),
                new WaitCommand(0.1),
                Commands.runOnce(
                    () -> m_algaeIntake.setShootingAngle(AlgaeIntake.elevation.STORED)),
                Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.STORED)),
                Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
                Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.LOW)),
                new WaitCommand(0.5),
                Commands.runOnce(() -> m_leds.SetPattern(Pattern.IDLE))),
            isManualMode));
  }

  public void toggleManualMode() {
    if (isManualMode.getAsBoolean() == true) {
      isManualMode = () -> false;
    } else {
      isManualMode = () -> true;
    }
    SmartDashboard.putBoolean("netManualMode", isManualMode.getAsBoolean());
  }
}
