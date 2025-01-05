// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

/** Add your docs here. */
public final class Constants {

  public static final double stickDeadband = 0.1;

  public static final class Swerve {
    public static final int pigeonID = 1;

    public static final COTSTalonFXSwerveConstants chosenModule =
        COTSTalonFXSwerveConstants.WCP.SwerveXFlipped.Falcon500(
            COTSTalonFXSwerveConstants.WCP.SwerveXFlipped.driveRatios.X1_10);

    // coming from datasheet using typical values
    // https://store.ctr-electronics.com/content/user-manual/Magnetic%20Encoder%20User%27s%20Guide.pdf
    public static final double kPwmPeriod = 1.0 / 244.0;
    public static final double dutyCycleMin = 1e-6 / kPwmPeriod;
    public static final double dutyCycleMax = 4096e-6 / kPwmPeriod;

    public static final int calibrationFreqSamples = 30;

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(26);
    public static final double wheelBase = Units.inchesToMeters(24.25);
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /* Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0), // front left
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), // front right
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), // back left
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)); // back right

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
    public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

    /* Swerve Current Limiting */
    public static final int angleCurrentLimit = 25;
    public static final int angleCurrentThreshold = 40;
    public static final double angleCurrentThresholdTime = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveCurrentLimit = 35;
    public static final int driveCurrentThreshold = 60;
    public static final double driveCurrentThresholdTime = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.12;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values From SYSID */
    public static final double driveKS = 0.32;
    public static final double driveKV = 1.51;
    public static final double driveKA = 0.27;

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.5;

    /** Radians per Second */
    public static final double maxAngularVelocity = 10.0;

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    /* Module Specific Constants */

    /* Front Left Module - Module 0 */
    // good
    public static final class Mod0 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int magEncoderID = 3; // 2
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(104.94); // -75.06
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, magEncoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int magEncoderID = 0; // 1
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(105.94); // -74.06
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, magEncoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    // good
    public static final class Mod2 {
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 4;
      public static final int magEncoderID = 2; // 3
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-93.51);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, magEncoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int magEncoderID = 1; // 0
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-86.31);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, magEncoderID, angleOffset);
    }
  }

  public static class AutoConstants {
    public static final HolonomicPathFollowerConfig kHolonomicPathFollowerConfig =
        new HolonomicPathFollowerConfig(
            new PIDConstants(10, 0, 0),
            new PIDConstants(10, 0, 0),
            Swerve.maxSpeed,
            Math.sqrt(
                (Swerve.trackWidth / 2.0 * Swerve.trackWidth / 2.0)
                    + (Swerve.wheelBase / 2.0 * Swerve.wheelBase / 2.0)),
            new ReplanningConfig());
  }
}
