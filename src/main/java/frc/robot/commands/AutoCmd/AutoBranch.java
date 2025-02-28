package frc.robot.commands.AutoCmd;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.PathfindingV2;
import frc.robot.subsystems.swerve.Swerve;

public class AutoBranch extends SequentialCommandGroup{
   public AutoBranch(Pose2d branch, boolean isLeftBranch, PathfindingV2 m_pathfinder, Swerve m_swerve) {
    addCommands(m_pathfinder.goThere(branch));
   }
}
