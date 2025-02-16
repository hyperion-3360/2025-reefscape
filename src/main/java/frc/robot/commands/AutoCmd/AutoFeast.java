package frc.robot.commands.AutoCmd;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Pathfinding;
import frc.robot.Auto.Pathfinding.POI;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.leds.LEDs;

public class AutoFeast extends SequentialCommandGroup {
  Elevator s_elevator;
  Shooter s_shooter;
  LEDs s_Leds;

  public AutoFeast(Elevator m_elevator, Shooter m_shooter, LEDs m_leds) {

    s_elevator = m_elevator;
    s_shooter = m_shooter;
    s_Leds = m_leds;

    addRequirements(m_elevator);
    addRequirements(m_shooter);
    addRequirements(m_leds);
    addCommands(Pathfinding.goThere(POI.FEEDERS));
  }
}
