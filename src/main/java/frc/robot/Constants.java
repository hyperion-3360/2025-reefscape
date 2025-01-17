// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

/** Add your docs here. */
public final class Constants {

  public static final class Swerve {

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
    // TODO: update with real motor information once robot is built

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
    // config the configs for the robot so that the robot may have configs.
    // I got forced to write this boilerplate... I'm sorry.
    private static RobotConfig configConfigs() {
      RobotConfig kConfig = null;
      try {
        kConfig = RobotConfig.fromGUISettings();
      } catch (Exception e) {
        e.printStackTrace();
      }
      return kConfig;
    }

    public static final RobotConfig kRobotConfig = configConfigs();

    public static final PathFollowingController kPathFollowController =
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            );
  }

  public static final class Conditions {
    // TODO put actual subsystems condition
    public static boolean hasAlgae() {
      boolean hasAlgae = false;
      if (hasAlgae == true) {
        return true;
      }
      return true;
    }

    public static boolean hasCoral() {
      boolean hasCoral = false;
      if (hasCoral == true) {
        return true;
      }
      return true;
    }
  }

  public static final class AlgaeCoralStand {
    // TODO add actual values not approximates using pathplanner
    public static final Pose2d[] kStands = {
      new Pose2d(1.225, 5.855, new Rotation2d(-180)),
      new Pose2d(1.225, 4.025, new Rotation2d(-180)),
      new Pose2d(1.225, 2.195, new Rotation2d(-180)),
    };
  }

  public static class Pegs {
    public static final Pose2d[] kPegs = {
      new Pose2d(3.9314751317, 3.2240855, new Rotation2d(240)),
      new Pose2d(3.657607315, 3.861435, new Rotation2d(240)),
      new Pose2d(4.6632495, 3.861435, new Rotation2d(180)),
      new Pose2d(4.6632495, 4.190365, new Rotation2d(180)),
      new Pose2d(4.2163368683, 4.6632495, new Rotation2d(300)),
      new Pose2d(3.9314751317, 4.8277145, new Rotation2d(300)),
      new Pose2d(5.0471708683, 4.8277145, new Rotation2d(60)),
      new Pose2d(4.7623091317, 4.6632495, new Rotation2d(60)),
      new Pose2d(5.321046, 4.190365, new Rotation2d(-180)),
      new Pose2d(5.321046, 3.861435, new Rotation2d(-180)),
      new Pose2d(4.7623, 3.4901, new Rotation2d(300)),
      new Pose2d(5.0471, 3.2240, new Rotation2d(300)),
    };
  }

  public static class Feeders {
    public static final Pose2d[] kFeeders = {
      new Pose2d(0.8382, 0.635, new Rotation2d(54)),
      new Pose2d(0.8382, 7.3914, new Rotation2d(306)),
    };
  }

  public static class Priorities {
    // TODO add actual priorities
    public static final int kShootCoralL4 = 7;
    public static final int kShootNet = 1;
    public static final int kIntakeCoral = 5;
    public static final int kShootingProcessor = 2;
  }

  public static class TimeToAction {
    // TODO add actual time
    public static final double kShootCoralL1 = 1;
    public static final double kShootCoralL4 = 3;
    public static final double kShootAlgaeProcessor = 4;
    public static final double kIntakeCoral = 2;
    public static final double kShootNet = 6;
  }

  public static class ElevatorHeight {
    // TODO: modify heights for elevator L = level
    // in inches
    public static final double kL1Height = 0.0;
    public static final double kL2Height = 0.0;
    public static final double kL3Height = 0.0;
    public static final double kL4Height = 0.0;

    public static final double kAlgaeLow = 0.0;
    public static final double kAlgaeHigh = 0.0;

    public static final double kHandoffHeight = 0.0;
    public static final double kBaseHeight = 0.0;
  }

  public static class CoralIntakeVariables {
    // TODO: modify angles and speeds

    // elbow angles in º
    public static final double kL1Angle = 0.0;
    public static final double kHandoffAngle = 0.0;
    public static final double kIntakeAngle = 0.0;

    // wrist angles in º
    public static final double kShootAngle = 0.0;
    public static final double kNormalAngle = 0.0;

    // wheel speeds
    public static final double kIntakeSpeed = 0.0;
    public static final double kShootSpeed = 0.0;
    public static final double kStopSpeed = 0.0;
  }

  public static class AlgaeIntakeVariables {
    // TODO: modify angles and wheel speeds

    // in º
    public static final double kStartingAngle = 0.0;
    public static final double kFloorIntakeAngle = 0.0;
    public static final double kProcessorAngle = 0.0;
    // auto intake bcuz of the little tower thing
    public static final double kAutoIntakeAngle = 0.0;

    // wheel speeds
    public static final double kIntakeSpeed = 0.0;
    public static final double kProcessorSpeed = 0.0;
    public static final double kStopSpeed = 0.0;
  }

  public static class CoralShooterSpeeds {
    // TODO: modify speeds
    // not sure if we'll need different speeds but :
    public static final double kIntakeSpeed = 0.0;
    public static final double kShootSpeed = 0.0;
  }

  public static class LEDConstants {
    // TODO: update port when robot is built
    public static final int kLEDPWMPort = 6;
    public static final int kLEDLength = 60;
  }

  public static class ClimberState {
    // Constant for climber possible positions
    public static final int kGrab = 0;
    public static final int kLift = 1;
  }

  public static class SubsystemInfo {
    // TODO: change ids to right ones and change names to more accurate, and add sensors and
    // encoders
    // motor ids
    public static final int kCoralIntakeMotorWristID = 0;
    public static final int kCoralIntakeMotorElbowID = 0;
    public static final int kAlgaeIntakeShooterMotorID = 0;
    public static final int kAlgaeIntakeArmMotorID = 0;
    public static final int kCoralShooterMotorID = 0;
    public static final int kElevatorMotorID = 0;
    public static final int kFlickerMotorID = 0;
    public static final int kClimberMotorID = 0;
    // sensor ids
    public static final int kCoralIntakeLimitSwitchID = 0;
    public static final int kCoralShooterBeambreakID = 0;
    public static final int kAlgaeIntakeLimitSwitchID = 0;

    public static final InvertedValue kElevatorMotorInversion = InvertedValue.Clockwise_Positive;
    // no clue if this is a good value but it should be fine to start
    public static final double kElevatorMotorCurrentLimit = 10.0;
  }
}
