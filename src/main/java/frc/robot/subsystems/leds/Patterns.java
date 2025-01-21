// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Map;

/** All the patterns for the LEDs (some are currently untested) */
public class Patterns {
  /** Solid colors */
  public class solids {
    // TODO: Check which colors drive team wants
    // TODO: Remove unused colors
    LEDPattern orange = LEDPattern.solid(Color.kOrangeRed);
    LEDPattern white = LEDPattern.solid(Color.kWhite);
    LEDPattern blue = LEDPattern.solid(Color.kBlue);
    LEDPattern green = LEDPattern.solid(Color.kLime);
    LEDPattern yellow = LEDPattern.solid(Color.kYellow);
    LEDPattern purple = LEDPattern.solid(Color.kPurple);
    LEDPattern red = LEDPattern.solid(Color.kRed);
    LEDPattern cyan = LEDPattern.solid(Color.kCyan);
    LEDPattern off = LEDPattern.kOff;
  }

  public class reveal {
    // might not use this (red to yellow gradient, yellow in the middle, red on the ends) (or the
    // inverse i forgot)
    LEDPattern cGradient = LEDPattern.gradient(GradientType.kContinuous, Color.kRed, Color.kOrange);
    // same as last one but red on one end and yellow on the other
    LEDPattern dGradient =
        LEDPattern.gradient(GradientType.kDiscontinuous, Color.kRed, Color.kOrange);
    // moving mask
    Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, 0.5, Color.kBlack);
    LEDPattern mask =
        LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(0.25));
    // scrolling gradient
    LEDPattern sGradient =
        LEDPattern.gradient(GradientType.kContinuous, Color.kRed, Color.kOrange)
            .scrollAtRelativeSpeed(Percent.per(Second).of(25));
    
  }
}
