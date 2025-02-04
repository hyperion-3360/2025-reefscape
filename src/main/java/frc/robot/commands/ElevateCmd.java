package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.desiredHeight;

public class ElevateCmd extends SequentialCommandGroup {

  public ElevateCmd(Elevator m_elevator, desiredHeight height) {
    // TODO Auto-generated constructor stub
    addRequirements(m_elevator);
    Commands.runOnce(() -> m_elevator.SetHeight(height));
  }
}
