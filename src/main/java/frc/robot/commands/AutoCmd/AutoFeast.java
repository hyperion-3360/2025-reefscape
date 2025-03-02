package frc.robot.commands.AutoCmd;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Auto.AutoWaypoints;
import frc.robot.Auto.PathfindingV2;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;
import frc.robot.subsystems.swerve.Swerve;
import java.util.ArrayList;
import java.util.List;

public class AutoFeast extends SequentialCommandGroup {
  List<Pose2d> feederWaypoints = new ArrayList<>();

  public AutoFeast(
      Swerve m_swerve,
      Elevator m_elevator,
      Shooter m_shooter,
      LEDs m_leds,
      PathfindingV2 m_pathfinding) {
    addRequirements(m_swerve);
    addRequirements(m_elevator);
    addRequirements(m_shooter);
    addRequirements(m_leds);
    feederWaypoints.add(AutoWaypoints.BlueAlliance.RightSide.feederWaypoints.feederLeft);
    feederWaypoints.add(AutoWaypoints.BlueAlliance.RightSide.feederWaypoints.feederRight);
    feederWaypoints.add(AutoWaypoints.BlueAlliance.LeftSide.feederWaypoints.feederLeft);
    feederWaypoints.add(AutoWaypoints.BlueAlliance.LeftSide.feederWaypoints.feederRight);
    addCommands(
        new DeferredCommand(
                () -> m_pathfinding.goThere(() -> m_swerve.getPose().nearest(feederWaypoints)),
                getRequirements())
            .alongWith(
                Commands.sequence(
                    Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
                    Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.FEEDER)),
                    Commands.runOnce(() -> m_shooter.closeBlocker()),
                    Commands.runOnce(() -> m_leds.SetPattern(Pattern.INTAKE)),
                    Commands.run(() -> m_shooter.setIntake()).until(() -> m_shooter.isCoralIn()),
                    Commands.runOnce(() -> m_leds.SetPattern(Pattern.READY)),
                    new WaitCommand(1.2),
                    Commands.runOnce(() -> m_shooter.stop()),
                    Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.LOW)))));
  }
}
