// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

/** Add your docs here. */
public final class Constants {

  public static final double stickDeadband = 0.03;

  public static final AprilTagFieldLayout tagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public static final class Swerve {

    public static final double kWheelDiameter = Units.inchesToMeters(3.8774);
    public static final int kGyroCanId = 20;

    public static final double robotLength = Units.inchesToMeters(35.0); // with bumper: 32.5
    public static final double robotWidth = Units.inchesToMeters(35.0); // with bumper: 32.5

    public static final COTSTalonFXSwerveConstants chosenModule =
        COTSTalonFXSwerveConstants.WCP.SwerveXFlipped.Falcon500(
            COTSTalonFXSwerveConstants.WCP.SwerveXFlipped.driveRatios.X1_11);

    // coming from datasheet using typical values
    // https://store.ctr-electronics.com/content/user-manual/Magnetic%20Encoder%20User%27s%20Guide.pdf
    public static final double kPwmPeriod = 1.0 / 244.0;
    public static final double dutyCycleMin = 1e-6 / kPwmPeriod;
    public static final double dutyCycleMax = 4096e-6 / kPwmPeriod;

    public static final int calibrationFreqSamples = 30;

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(24.75);
    public static final double wheelBase = Units.inchesToMeters(24.75);
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
    public static final class Mod0 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int magEncoderID = 3; // g
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(89.20); // 89.20 // -4.3
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, magEncoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int magEncoderID = 8; // g
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(187.33); // 187.33// 0
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, magEncoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int magEncoderID = 2; // g
      public static final Rotation2d angleOffset =
          Rotation2d.fromDegrees(310.42); // 310.42// -24.39
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, magEncoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 4;
      public static final int magEncoderID = 5; // g
      public static final Rotation2d angleOffset =
          Rotation2d.fromDegrees(166.43); // 166.43// -83.38
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

    public enum Sequence {
      ThreeCoralLeft,
      ThreeCoralRight,
      OneCoralThenAlgae,
      Line
    }
  }

  public static final class AlgaeCoralStand {
    public static final Pose2d[] kStands = {
      new Pose2d(1.225, 5.855, Rotation2d.fromDegrees(180)),
      new Pose2d(1.225, 4.025, Rotation2d.fromDegrees(180)),
      new Pose2d(1.225, 2.195, Rotation2d.fromDegrees(180)),
    };
  }

  public static class Pegs {
    public static final Pose2d[] kPegs = {
      new Pose2d(3.9314751317, 3.2240855, Rotation2d.fromDegrees(60)),
      new Pose2d(3.657607315, 3.861435, Rotation2d.fromDegrees(60)),
      new Pose2d(4.6632495, 3.861435, Rotation2d.fromDegrees(0)),
      new Pose2d(4.6632495, 4.190365, Rotation2d.fromDegrees(0)),
      new Pose2d(4.2163368683, 4.6632495, Rotation2d.fromDegrees(300)),
      new Pose2d(3.9314751317, 4.8277145, Rotation2d.fromDegrees(300)),
      new Pose2d(5.0471708683, 4.8277145, Rotation2d.fromDegrees(240)),
      new Pose2d(4.7623091317, 4.6632495, Rotation2d.fromDegrees(240)),
      new Pose2d(5.321046, 4.190365, Rotation2d.fromDegrees(180)),
      new Pose2d(5.321046, 3.861435, Rotation2d.fromDegrees(180)),
      new Pose2d(4.7623, 3.4901, Rotation2d.fromDegrees(120)),
      new Pose2d(5.0471, 3.2240, Rotation2d.fromDegrees(120)),
    };
  }

  public static class Feeders {
    public static final Pose2d[] kFeeders = {
      new Pose2d(0.8382, 0.635, Rotation2d.fromDegrees(54)),
      new Pose2d(0.8382, 7.3914, Rotation2d.fromDegrees(126)),
    };
  }

  public static class Priorities {
    public static final int kShootCoralL4 = 7;
    public static final int kShootNet = 1;
    public static final int kIntakeCoral = 5;
    public static final int kShootingProcessor = 2;
  }

  public static class TimeToAction {
    public static final double kShootCoralL1 = 1;
    public static final double kShootCoralL4 = 3;
    public static final double kShootAlgaeProcessor = 4;
    public static final double kIntakeCoral = 2;
    public static final double kShootNet = 6;
  }

  public static class CoralIntakeVariables {

    // elbow angles in º
    public static final double kHandoffAngle = 0;
    public static final double kIntakeAngle = 26;

    // wrist angles in motor rotations
    public static final double Openposition = 1.5;
    public static final double Closedposition = 0;
  }

  public static class AlgaeIntakeVariables {

    // in º
    public static final double kStartingAngle = 0.1;
    public static final double kFloorIntakeAngle = -22.0;
    public static final double kNetAngle = -5.0;
    public static final double kProcessorAngle = -15;
    // auto intake bcuz of the little tower thing
    public static final double kAutoIntakeAngle = 0.0;
    public static final double kCurrentLimit = 15.0;

    // wheel speeds
    public static final double kIntakeSpeed = 0.5;
    public static final double kStoringSpeed = 0.08;
    public static final double kStoredSpeed = 0.0;
    public static final double kProcessorSpeed = -1.0;
    public static final double kStopSpeed = 0.0;
    public static final double kNetSpeed = -0.7;
  }

  public static class CoralShooterVariables {
    public static final double kShootNo = 0.0;
    public static final double kShootL1 = -0.4;
    public static final double kShootL2 = -0.80;
    public static final double kShootL3 = -0.85;
    public static final double kShootL4 = -0.6;
    // we need a different speed for teleop to compensate for angle
    public static final double kShootL4Teleop = -0.85;
    public static final double kIntakeSpeed = -0.6;

    public static final NeutralMode kCoralShooterNeutralMode = NeutralMode.Coast;
    public static final int kCoralShooterCurrentLimit = 30;
    public static final double kCoralShooterRamprate = 0.5;

    // angles in º
    public static final double kCoralShooterClosed = 10.0;
    public static final double kCoralShooterOpen = 113.0;
  }

  public static class LEDConstants {
    public static final int kLEDPWMPort = 5;
    public static final int kLEDLength = 30;
  }

  public static class SubsystemInfo {
    // encoders
    // motor ids
    public static final int kAlgaeGrabberLeftMotorID = 15;
    public static final int kAlgaeGrabberRightMotorID = 16;
    public static final int kAlgaeArmMotorID = 14;
    public static final int kCoralShooterTalonID = 17;
    public static final int kCoralShooterServoID = 7;
    public static final int kClimberShallowMotorID = 11;
    public static final int kClimberDeepMotorID = 12;
    public static final int kCoralDumperLeftServoID = 8;
    public static final int kCoralDumperRightServoID = 9;

    // sensor ids
    public static final int kCoralShooterBeambreakID = 7;
    public static final int kCoralAutoIntakeBeamBrake = 6;
    public static final int kClimberBeamBrakeID = 4;
    public static final int kClimberPenisID = 6;
    public static final int kClimberFingerID = 2;

    public static final InvertedValue kElevatorMotorInversion = InvertedValue.Clockwise_Positive;
    // no clue if this is a good value but it should be fine to start
    public static final double kElevatorMotorCurrentLimit = 10.0;
    public static final int kRightElevatorMotorID = 9;
    public static final int kLeftElevatorMotorID = 10;
  }

  public static class ClimberConstants {
    public static final double kRaisedAngle = 90;
  }

  public static class ElevatorConstants {

    public static final double kElevatorMotorCurrentLimit = 30.0;
    public static final InvertedValue kRightElevatorMotorNotInverted =
        InvertedValue.Clockwise_Positive;

    // elevator heights
    public static final double kElevatorDown = 0.0;
    public static final double kElevatorL1 = 0.82;
    public static final double kElevatorL2 = 1.19;
    public static final double kElevatorL3 = 1.82;
    public static final double kElevatorL4 = 2.80;
    public static final double kElevatorNet = 2.90;
    public static final double kElevatorProcessor = 0.36;
    public static final double kElevatorAlgaeLow = 0.05;
    public static final double kElevatorLollypop = 0.50;
    public static final double kElevatorFeeder = 0.247;
    public static final double kElevatorAlgaeL2 = 1.25;
    public static final double kElevatorAlgaeL3 = 1.87;
  }

  public static class AlgaeIntakeConstants {
    public static final double kAngleTolerance = 0.0;
  }

  public static class CameraInfo {
    public static final double kCamHeight = 0.0;
    public static final double kCamPitch = 0.0;
  }
}
