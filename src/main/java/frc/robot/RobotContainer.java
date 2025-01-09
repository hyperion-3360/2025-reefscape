// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auto.Pathfinding;
import frc.robot.subsystems.swerve.CTREConfigs;
import frc.robot.subsystems.swerve.Swerve;

public class RobotContainer {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();
  public static final Swerve m_swerve = new Swerve();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Pathfinding.doPathfinding();
  }
}
