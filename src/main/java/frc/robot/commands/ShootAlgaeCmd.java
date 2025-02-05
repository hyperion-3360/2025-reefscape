package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.leds.LEDs;

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
  public ShootAlgaeCmd(
      AlgaeIntake m_algaeIntake, AlgaeIntake.elevation shootingAngle, LEDs m_leds) {
    addRequirements(m_algaeIntake);
    addRequirements(m_leds);
    addCommands(
        Commands.runOnce(() -> m_leds.shootColor()),
        Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.INTAKE)),
        new WaitCommand(0.3),
        Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.PROCESSOR)),
        new WaitUntilCommand(() -> !m_algaeIntake.sensorTriggered()),
        Commands.runOnce(() -> m_algaeIntake.setShootingAngle(AlgaeIntake.elevation.STORED)),
        Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.STORED)),
        m_leds.idleColor());
  }
}
