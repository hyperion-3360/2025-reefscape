package frc.robot.commands.AutoCmd;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.AutoWaypoints;
import frc.robot.Auto.PathfindingV2;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.swerve.Swerve;
import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.function.BooleanSupplier;

public class AutoFeast extends SequentialCommandGroup {
  private List<Pose2d> feederWaypoints = new ArrayList<>();
  private Alliance currentAlliance = Alliance.Blue;
  private BooleanSupplier isManualMode = () -> false;

  public AutoFeast(
      Swerve m_swerve,
      Elevator m_elevator,
      Shooter m_shooter,
      LEDs m_leds,
      PathfindingV2 m_pathfinding) {
    addRequirements(m_swerve);
    // addRequirements(m_elevator);
    // addRequirements(m_shooter);
    try {
      if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
        currentAlliance = Alliance.Blue;
      } else if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
        currentAlliance = Alliance.Red;
      }
    } catch (NoSuchElementException e) {
      currentAlliance = Alliance.Blue;
    }

    if (currentAlliance.equals(Alliance.Blue)) {
      feederWaypoints.add(AutoWaypoints.BlueAlliance.RightSide.feederWaypoints.feederLeft);
      feederWaypoints.add(AutoWaypoints.BlueAlliance.RightSide.feederWaypoints.feederRight);
      feederWaypoints.add(AutoWaypoints.BlueAlliance.LeftSide.feederWaypoints.feederLeft);
      feederWaypoints.add(AutoWaypoints.BlueAlliance.LeftSide.feederWaypoints.feederRight);
    } else if (currentAlliance.equals(Alliance.Red)) {
      feederWaypoints.add(AutoWaypoints.RedAlliance.RightSide.feederWaypoints.feederLeft);
      feederWaypoints.add(AutoWaypoints.RedAlliance.RightSide.feederWaypoints.feederRight);
      feederWaypoints.add(AutoWaypoints.RedAlliance.LeftSide.feederWaypoints.feederLeft);
      feederWaypoints.add(AutoWaypoints.RedAlliance.LeftSide.feederWaypoints.feederRight);
    }
    addCommands(
        new ConditionalCommand(
            Commands.none(),
            new DeferredCommand(
                () -> m_pathfinding.goThere(() -> m_swerve.getPose().nearest(feederWaypoints)),
                getRequirements()),
            isManualMode));
    // .alongWith(
    //     Commands.sequence(
    //         Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
    //         Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.FEEDER)),
    //         Commands.runOnce(() -> m_shooter.closeBlocker()),
    //         Commands.runOnce(() -> m_leds.SetPattern(Pattern.INTAKE)),
    //         Commands.run(() -> m_shooter.setIntake()).until(() -> m_shooter.isCoralIn()),
    //         Commands.runOnce(() -> m_leds.SetPattern(Pattern.READY)),
    //         new WaitCommand(1.2),
    //         Commands.runOnce(() -> m_shooter.stop()),
    //         Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.LOW)))));
  }

  public void toggleManualMode() {
    if (isManualMode.getAsBoolean() == true) {
      isManualMode = () -> false;
    }
    isManualMode = () -> true;
  }
}
