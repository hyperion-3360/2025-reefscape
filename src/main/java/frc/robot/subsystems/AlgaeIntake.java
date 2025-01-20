// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

/** 3 neos 550 (mini-neo) one to go up and down and two to intake* */
public class AlgaeIntake extends SubsystemBase {

  public enum elevation {
    NET,
    FLOOR,
    STORED
  }

  public enum shooting {
    INTAKE,
    PROCESSOR,
    NET,
    STORING, // this is the intake speed / 2
    STORED // this is resting speed or 0
  }

  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private PIDController m_pid = new PIDController(kP, kI, kD);

  private SparkMaxConfig m_intakeLeftConfig = new SparkMaxConfig();
  private SparkMaxConfig m_intakeRightConfig = new SparkMaxConfig();
  private SparkMaxConfig m_directionConfig = new SparkMaxConfig();

  private SparkMax m_pivotMotor =
      new SparkMax(Constants.SubsystemInfo.kAlgaeArmMotorID, MotorType.kBrushless);
  private SparkMax m_intakeLeft =
      new SparkMax(Constants.SubsystemInfo.kAlgaeGrabberLeftMotorID, MotorType.kBrushless);
  private SparkMax m_intakeRight =
      new SparkMax(Constants.SubsystemInfo.kAlgaeGrabberRightMotorID, MotorType.kBrushless);

  private AbsoluteEncoder m_directionEncoder = m_pivotMotor.getAbsoluteEncoder();

  private double m_AnglesTarget = Constants.AlgaeIntakeVariables.kStartingAngle;
  private double m_SpeedTarget = Constants.AlgaeIntakeVariables.kIntakeSpeed;

  public AlgaeIntake() {

    m_intakeLeftConfig.follow(m_intakeRight, true);

    m_intakeLeftConfig.smartCurrentLimit(15);
    m_intakeRightConfig.smartCurrentLimit(15);
    m_directionConfig.smartCurrentLimit(15);

    m_intakeLeft.configure(
        m_intakeLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_intakeRight.configure(
        m_intakeRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_pivotMotor.configure(
        m_directionConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      m_pid.reset();
    }

    SmartDashboard.putNumber("Current", m_intakeRight.getOutputCurrent());
    SmartDashboard.putNumber("AlgaeEncoder", m_pivotMotor.getEncoder().getPosition());
  }

  public void setShootingSpeed(shooting speed) {

    switch (speed) {
      case INTAKE:
        this.m_SpeedTarget = Constants.AlgaeIntakeVariables.kIntakeSpeed;
        break;

      case NET:
        this.m_SpeedTarget = Constants.AlgaeIntakeVariables.kNetSpeed;
        break;

      case PROCESSOR:
        this.m_SpeedTarget = Constants.AlgaeIntakeVariables.kProcessorSpeed;
        break;

      case STORING:
        this.m_SpeedTarget = Constants.AlgaeIntakeVariables.kIntakeSpeed / 2;
        break;

      case STORED:
        this.m_SpeedTarget = 0.0;
    }
  }

  public void setShootingAngle(elevation angle) {

    switch (angle) {
      case NET:
        this.m_AnglesTarget = Constants.AlgaeIntakeVariables.kNetAngle;
        break;

      case FLOOR:
        this.m_AnglesTarget = Constants.AlgaeIntakeVariables.kFloorIntakeAngle;
        break;

      case STORED:
        this.m_AnglesTarget = Constants.AlgaeIntakeVariables.kStartingAngle;
        break;
    }
  }

  public boolean isAlgaeIn() {
    return m_intakeRight.getOutputCurrent() >= Constants.AlgaeIntakeVariables.kCurrentLimit;
  }

  public boolean isAtAngle() {
    return Math.abs(m_AnglesTarget - m_directionEncoder.getPosition())
        <= Constants.AlgaeIntakeConstants.kAngleTolerance;
  }

  public Command setAngle(DoubleSupplier angle) {
    return run(
        () -> {
          m_pivotMotor.set(angle.getAsDouble());
        });
  }

  public Command setSpeed(DoubleSupplier speed) {
    return run(
        () -> {
          m_intakeRight.set(speed.getAsDouble());
        });
  }
}
