package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.AlgaeIntake;

public class ShootAlgaeCmd extends SequentialCommandGroup {
  AlgaeIntake m_algaeIntake;

  /**
   * @brief Creates a new ShootAlgaeCmd.
   *     <p>This command will shoot the algae ball at the correct elevation. The first part of the
   *     command will pull the algae ball into the robot, then the robot will pivot the algae arm to
   *     the correct angle
   * @param m_algaeIntake The subsystem used by this command.
   * @param shootingAngle The angle at which the algae ball will be shot
   */
  public ShootAlgaeCmd(AlgaeIntake m_algaeIntake, AlgaeIntake.elevation shootingAngle) {
    addCommands(
        Commands.run(
            () -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.INTAKE), m_algaeIntake),
        new WaitUntilCommand(0.1),
        m_algaeIntake.pivotAlgae(shootingAngle),
        Commands.run(
            () ->
                m_algaeIntake.setShootingSpeed(
                    shootingAngle == AlgaeIntake.elevation.NET
                        ? AlgaeIntake.shooting.NET
                        : AlgaeIntake.shooting.PROCESSOR),
            m_algaeIntake),
        new WaitUntilCommand(1),
        Commands.run(
            () -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.STORED), m_algaeIntake));
  }
}
