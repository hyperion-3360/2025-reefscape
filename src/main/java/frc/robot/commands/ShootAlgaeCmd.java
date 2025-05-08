package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaeIntake.elevation;
import frc.robot.subsystems.AlgaeIntake.shooting;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;

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
      AlgaeIntake m_algaeIntake, Elevator m_elevator, LEDs m_leds, desiredHeight height) {
    addRequirements(m_algaeIntake);
    addRequirements(m_leds);
    addRequirements(m_elevator);
    addCommands(
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
        Commands.runOnce(() -> m_elevator.SetHeight(height)),
        Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(shooting.STORING)),
        Commands.runOnce(() -> m_algaeIntake.setShootingAngle(elevation.FLOOR)),
        new WaitCommand(0.3),
        Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(shooting.STORING)),
        new WaitCommand(0.2),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.SHOOTER)),
        Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.INTAKE)),
        new WaitCommand(0.2),
        Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.PROCESSOR)),
        new WaitUntilCommand(() -> !m_algaeIntake.sensorTriggered()),
        new WaitCommand(0.1),
        Commands.runOnce(() -> m_algaeIntake.setShootingAngle(AlgaeIntake.elevation.STORED)),
        Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.STORED)),
        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.LOW)),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.IDLE)));
  }
}
