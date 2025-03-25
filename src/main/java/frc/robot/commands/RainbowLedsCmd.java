// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.Pattern;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RainbowLedsCmd extends Command {
  LEDs leds;

  public RainbowLedsCmd(LEDs leds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.leds = leds;
    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    leds.SetPattern(Pattern.RAINBOW);
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      leds.SetPattern(Pattern.IDLE);
    }
  }
}
