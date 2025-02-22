package frc.robot.commands.AutoCmd;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Auto.Pathfinding;
import frc.robot.Auto.Pathfinding.POI;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaeIntake.elevation;
import frc.robot.subsystems.AlgaeIntake.shooting;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;

public class AutoProcessor extends SequentialCommandGroup {

  public AutoProcessor(Elevator m_elevator, AlgaeIntake m_algaeIntake, LEDs m_leds) {
    addCommands(
        Pathfinding.goThere(POI.PROCESSOR)
            .alongWith(
                Commands.sequence(
                    Commands.runOnce(() -> m_leds.SetPattern(Pattern.ELEVATOR)),
                    Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.PROCESSOR)),
                    Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(shooting.STORED)),
                    Commands.runOnce(() -> m_algaeIntake.setShootingAngle(elevation.FLOOR)),
                    new WaitCommand(0.5),
                    Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(shooting.STORING)),
                    new WaitCommand(0.5),
                    Commands.runOnce(() -> m_leds.SetPattern(Pattern.SHOOTER)))),
        Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.INTAKE)),
        new WaitCommand(0.25),
        Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.PROCESSOR)),
        new WaitUntilCommand(() -> !m_algaeIntake.sensorTriggered()),
        Commands.runOnce(() -> m_algaeIntake.setShootingAngle(AlgaeIntake.elevation.STORED)),
        Commands.runOnce(() -> m_algaeIntake.setShootingSpeed(AlgaeIntake.shooting.STORED)),
        Commands.runOnce(() -> m_elevator.SetHeight(desiredHeight.LOW)),
        Commands.runOnce(() -> m_leds.SetPattern(Pattern.IDLE)));
  }
}
