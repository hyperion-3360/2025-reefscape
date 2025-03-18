package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.MinuteMoveCmd.OffsetDir;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.shootSpeed;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;
import frc.robot.subsystems.swerve.Swerve;

public class DriveAndShootCoralCmd extends SequentialCommandGroup {

  /**
   * constructor to construct a algae intake command for the reef
   *
   * @param m_shooter
   * @param m_leds
   * @param m_elevator
   * @param height
   */
  public DriveAndShootCoralCmd(
      Shooter m_shooter,
      LEDs m_leds,
      Elevator m_elevator,
      Swerve m_swerve,
      AlgaeIntake m_algaeIntake,
      Pose2d desiredPose,
      OffsetDir direction) {
        MinuteMoveCmd backTrack = new MinuteMoveCmd(m_swerve, 0.5, 0.8, OffsetDir.BACK);
        MinuteMoveCmd shuffleRight = new MinuteMoveCmd(m_swerve, OffsetDir.RIGHT, m_algaeIntake);
        MinuteMoveCmd shuffleLeft = new MinuteMoveCmd(m_swerve, OffsetDir.LEFT, m_algaeIntake);
    addRequirements(m_shooter);
    addRequirements(m_leds);
    addRequirements(m_elevator);
    addRequirements(m_swerve);
    addCommands(
        Commands.runOnce(() -> m_elevator.ElevateAutomatic()),
        Commands.runOnce(() -> m_swerve.lessenedConstraints()).unless(() -> m_elevator.getTargetHeight() != desiredHeight.L4),
        new InstantCommand(() -> m_swerve.drivetoTarget(desiredPose)),
        new WaitUntilCommand(() -> m_swerve.targetReached()),
        shuffleRight.unless(() -> direction.equals(OffsetDir.LEFT)),
        shuffleLeft.unless(() -> direction.equals(OffsetDir.RIGHT)),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.SHOOTER)),
        Commands.runOnce(() -> m_shooter.openBlocker()),
        new WaitCommand(0.3),
        Commands.runOnce(() -> m_shooter.setShoot(getShootingSpeed(m_elevator.getTargetHeight()))),
        new WaitUntilCommand(() -> !m_shooter.isCoralIn()),
        new WaitCommand(0.3),
        Commands.runOnce(() -> m_shooter.stop()),
        Commands.runOnce(() -> m_shooter.closeBlocker()),
        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.LOW)),
        backTrack,
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.IDLE))
        );
  }

  private shootSpeed getShootingSpeed(desiredHeight height) {
    shootSpeed speed;
    switch (height) {
      case L1:
        speed = shootSpeed.L1;
        break;
      case L2:
        speed = shootSpeed.L2;
        break;
      case L3:
        speed = shootSpeed.L3;
        break;
      case L4:
        speed = shootSpeed.L4TELEOP;
        break;
      default:
        speed = shootSpeed.STOP;
        break;
    }

    return speed;
  }
}
