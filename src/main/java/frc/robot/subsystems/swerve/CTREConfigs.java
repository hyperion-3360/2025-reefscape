package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.Constants;

public final class CTREConfigs {
  public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
  public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
  public double dutyCycleRangeMin, dutyCycleRangeMax;

  public CTREConfigs() {
    /** Swerve Absolute Encoder Configurations */
    dutyCycleRangeMin = Constants.Swerve.dutyCycleMin;
    dutyCycleRangeMax = Constants.Swerve.dutyCycleMax;

    /** Swerve Angle Motor Configurations */
    /* Motor Inverts and Neutral Mode */
    swerveAngleFXConfig.MotorOutput.Inverted = Constants.Swerve.angleMotorInvert;
    swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.Swerve.angleNeutralMode;

    /* Gear Ratio and Wrapping Config */
    swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;
    swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

    /* Current Limiting */
    swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.Swerve.angleEnableCurrentLimit;
    swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleCurrentLimit;
    swerveAngleFXConfig
            .CurrentLimits
            .SupplyCurrentLowerLimit = // from SupplyCurrentThreshold to SupplyCurrentLowerLimit
        Constants.Swerve.angleCurrentThreshold;
    swerveAngleFXConfig
            .CurrentLimits
            .SupplyCurrentLowerTime = // from SupplyTimeThreshold to SupplyCurrentLowerTime
        Constants.Swerve.angleCurrentThresholdTime;

    /* PID Config */
    swerveAngleFXConfig.Slot0.kP = Constants.Swerve.angleKP;
    swerveAngleFXConfig.Slot0.kI = Constants.Swerve.angleKI;
    swerveAngleFXConfig.Slot0.kD = Constants.Swerve.angleKD;

    /** Swerve Drive Motor Configuration */
    /* Motor Inverts and Neutral Mode */
    swerveDriveFXConfig.MotorOutput.Inverted = Constants.Swerve.driveMotorInvert;
    swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.Swerve.driveNeutralMode;

    /* Gear Ratio Config */
    swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;

    /* Current Limiting */
    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.Swerve.driveEnableCurrentLimit;
    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveCurrentLimit;
    swerveDriveFXConfig
            .CurrentLimits
            .SupplyCurrentLowerLimit = // from SupplyCurrentThreshold to SupplyCurrentLowerLimit
        Constants.Swerve.driveCurrentThreshold;
    swerveDriveFXConfig
            .CurrentLimits
            .SupplyCurrentLowerTime = // from SupplyTimeThreshold to SupplyCurrentLowerTime
        Constants.Swerve.driveCurrentThresholdTime;

    /* PID Config */
    swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
    swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
    swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;

    /* Open and Closed Loop Ramping */
    swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
    swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;

    swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod =
        Constants.Swerve.closedLoopRamp;
    swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        Constants.Swerve.closedLoopRamp;
  }
}
