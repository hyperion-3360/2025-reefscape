// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

/** All the patterns for the LEDs (currently untested) */
public class Patterns {
  public class solids {
    LEDPattern orange = LEDPattern.solid(Color.kOrangeRed);
    LEDPattern white = LEDPattern.solid(Color.kWhite);
    LEDPattern blue = LEDPattern.solid(Color.kBlue);
    LEDPattern green = LEDPattern.solid(Color.kLime);
    LEDPattern yellow = LEDPattern.solid(Color.kYellow);
    LEDPattern purple = LEDPattern.solid(Color.kPurple);
  }
}
