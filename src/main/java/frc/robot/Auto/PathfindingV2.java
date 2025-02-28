package frc.robot.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.swerve.Swerve;

public class PathfindingV2 extends Command {
  public PathfindingV2(Shooter m_shooter, Elevator m_elevator, LEDs m_leds, Swerve m_swerve) {
    addRequirements(m_shooter, m_elevator, m_leds, m_swerve);
  }

  private static Pose2d POICoordinatesOptimisation(Pose2d poiToPathfind) {

    double robotLengthPlusBuffer = (Constants.Swerve.robotLength / 2) * 1.0;
    double robotWidthPlusBuffer = (Constants.Swerve.robotWidth / 2) * 1.0;
    Rotation2d rotation = poiToPathfind.getRotation();

    // calculates the coordinates to displace the robot actual wanted position relative to the POI
    Translation2d widthToBacktrack =
        new Translation2d(
            poiToPathfind.getX() + robotLengthPlusBuffer * rotation.getCos(),
            poiToPathfind.getY() + robotWidthPlusBuffer * rotation.getSin());

    return new Pose2d(widthToBacktrack, poiToPathfind.getRotation());
  }

  public Command auto() {
    SequentialCommandGroup pathfindingSequence = new SequentialCommandGroup(Commands.none());
    pathfindingSequence.addCommands(AutoBuilder.pathfindToPose(null, null));
    return pathfindingSequence;
  }
}
