package frc.robot.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
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
  PathConstraints constraints = new PathConstraints(1.0, 1.0, 1, 2);

  public PathfindingV2(Shooter m_shooter, Elevator m_elevator, LEDs m_leds, Swerve m_swerve) {
    addRequirements(m_shooter, m_elevator, m_leds, m_swerve);
  }

  public Command goThere(Pose2d pose) {
    return goThere(pose, 0);
  }
  
  public Command goThere(Pose2d pose, double goalEndVelocity) {
    return AutoBuilder.pathfindToPose(pose, constraints, goalEndVelocity);
  }

  public Command auto() {
    SequentialCommandGroup pathfindingSequence = new SequentialCommandGroup(Commands.none());
    pathfindingSequence.addCommands(
        AutoBuilder.pathfindToPose(new Pose2d(7.3, 3, Rotation2d.k180deg), constraints));
    return pathfindingSequence;
  }
}
