package frc.robot.commands.AutoCmd;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Auto.Pathfinding;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.shootSpeed;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;
import frc.robot.subsystems.swerve.Swerve;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class AutoBranchShooting extends SequentialCommandGroup {
  private List<Pose2d> branchList = new ArrayList<>();
  private Supplier<Pose2d> currentPose = () -> Constants.Pegs.kPegs[3];
  private desiredHeight height = desiredHeight.L4;

  public AutoBranchShooting(Swerve m_swerve, Elevator m_elevator, Shooter m_shooter, LEDs m_leds) {

    for (int i = 0; i <= 6; i++) {
      branchList.add(Constants.Pegs.kPegs[i]);
    }
    List<Pose2d> L4branchList = branchList;
    List<Pose2d> L3branchList = branchList;
    List<Pose2d> L2branchList = branchList;

    addRequirements(m_swerve, m_elevator, m_shooter, m_leds);
    addCommands(
        // I need to change the height and target dynamically so it is called into the command
        Commands.runOnce(
                () -> {
                  height = desiredHeight.L4;
                  currentPose = () -> m_swerve.getPose().nearest(L4branchList);
                  L4branchList.remove(currentPose.get());
                })
            .onlyIf(() -> !L4branchList.isEmpty()),
        Commands.runOnce(
                () -> {
                  height = desiredHeight.L3;
                  currentPose = () -> m_swerve.getPose().nearest(L3branchList);
                  L3branchList.remove(currentPose.get());
                })
            .onlyIf(() -> !L3branchList.isEmpty() && L4branchList.isEmpty()),
        Commands.runOnce(
                () -> {
                  height = desiredHeight.L2;
                  currentPose = () -> m_swerve.getPose().nearest(L2branchList);
                  L2branchList.remove(currentPose.get());
                })
            .onlyIf(() -> !L2branchList.isEmpty() && L3branchList.isEmpty()),
        Pathfinding.goThere(currentPose.get())
            .alongWith(
                Commands.runOnce(() -> m_leds.SetPattern(Pattern.SHOOTER)),
                new WaitUntilCommand(
                        () ->
                            m_swerve
                                    .getPose()
                                    .getTranslation()
                                    .getDistance(currentPose.get().getTranslation())
                                <= 0.4)
                    .andThen(
                        Commands.sequence(
                            Commands.runOnce(() -> m_shooter.openBlocker()),
                            Commands.runOnce(() -> m_elevator.SetHeight(height)),
                            new WaitCommand(1.5),
                            Commands.runOnce(() -> m_leds.SetPattern(Pattern.SHOOTER)),
                            Commands.runOnce(() -> m_shooter.setShoot(shootSpeed.L4))
                                .until(() -> !m_shooter.isCoralIn()),
                            new WaitCommand(0.8),
                            Commands.runOnce(() -> m_shooter.stop()),
                            Commands.runOnce(() -> m_shooter.closeBlocker()),
                            Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.LOW)),
                            Commands.runOnce(() -> m_leds.SetPattern(Pattern.IDLE))))));
  }
}
