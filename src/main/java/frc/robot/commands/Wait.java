// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.BooleanSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Wait {
  /**
   * Waits a certain number of seconds
   *
   * @param seconds the number of seconds to wait (crazy)
   */
  public static Command waitSecs(double seconds) {
    return new WaitCommand(seconds);
  }

  /**
   * Waits until the condition returns true
   *
   * @param condition the condition to wait for
   */
  public static Command waitUntil(BooleanSupplier condition) {
    return new WaitUntilCommand(condition);
  }
}
