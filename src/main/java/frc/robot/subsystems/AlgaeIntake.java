// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.Joysticks;
import frc.lib.util.TestBindings;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

/** 3 neos 550 (mini-neo) one to go up and down and two to intake* */
public class AlgaeIntake extends SubsystemBase implements TestBindings {

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

  private static final double kP = 0.01;
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

  private double m_AnglesTarget = Constants.AlgaeIntakeVariables.kStartingAngle;
  private double m_SpeedTarget = Constants.AlgaeIntakeVariables.kStopSpeed;

  private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter translationLimiter = new SlewRateLimiter(3);
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int translationAxis = XboxController.Axis.kLeftY.value;

  public AlgaeIntake() {

    m_intakeLeftConfig.limitSwitch.forwardLimitSwitchEnabled(false);
    m_intakeLeftConfig.follow(m_intakeRight, true);

    // set current limit
    m_intakeLeftConfig.smartCurrentLimit(20);
    m_intakeRightConfig.smartCurrentLimit(20);
    m_directionConfig.smartCurrentLimit(20);

    // brake mode for both pivot and intake
    m_intakeLeftConfig.idleMode(IdleMode.kBrake);
    m_intakeRightConfig.idleMode(IdleMode.kBrake);
    m_directionConfig.idleMode(IdleMode.kBrake);

    m_pivotMotor.getEncoder().setPosition(0);

    m_intakeLeft.configure(
        m_intakeLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_intakeRight.configure(
        m_intakeRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_pivotMotor.configure(
        m_directionConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_pivotMotor.getEncoder().setPosition(0.0);
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      m_pid.reset();
      m_AnglesTarget = Constants.AlgaeIntakeVariables.kStartingAngle;
      m_SpeedTarget = 0.0;
    }
    m_pivotMotor.set(m_pid.calculate(m_pivotMotor.getEncoder().getPosition(), m_AnglesTarget));
    m_intakeRight.set(m_SpeedTarget);

    SmartDashboard.putNumber(
        "Output du pid", m_pid.calculate(m_pivotMotor.getEncoder().getPosition(), m_AnglesTarget));
    SmartDashboard.putNumber("Current", m_intakeRight.getOutputCurrent());
    SmartDashboard.putNumber("AlgaeEncoder", m_pivotMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("error", m_pid.getError());
    SmartDashboard.putNumber(
        "pid Calculation",
        m_pid.calculate(m_pivotMotor.getEncoder().getPosition(), m_AnglesTarget));
    SmartDashboard.putBoolean("sensor left", m_intakeLeft.getForwardLimitSwitch().isPressed());
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
        this.m_SpeedTarget = Constants.AlgaeIntakeVariables.kStoringSpeed;
        break;

      case STORED:
        this.m_SpeedTarget = 0.0;
        break;
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

  public boolean sensorTriggered() {
    return m_intakeLeft.getForwardLimitSwitch().isPressed();
  }

  public boolean isAtAngle() {
    return m_pid.atSetpoint();
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

  private double currentInterpolation(double current) {
    double startPoint = Constants.AlgaeIntakeVariables.kCurrentLimit;
    double linearInterpolate = -0.86 * (12.5 - RobotController.getBatteryVoltage()) + startPoint;
    return linearInterpolate;
  }

  public Command pivotAlgae(elevation angle) {
    return runOnce(
        () -> {
          setShootingAngle(angle);
        });
  }

  public Command shootAlgae(shooting speed) {
    return runOnce(
        () -> {
          setShootingSpeed(speed);
        });
  }

  @Override
  public void setupTestBindings(Trigger moduleTrigger, CommandXboxController controller) {

    moduleTrigger
        .and(controller.y())
        .whileTrue(
            this.setAngle(
                () ->
                    Joysticks.conditionJoystick(
                        () -> controller.getRawAxis(strafeAxis),
                        strafeLimiter,
                        Constants.stickDeadband,
                        true)))
        .onFalse(this.setAngle(() -> 0.0));

    moduleTrigger
        .and(controller.povDown())
        .whileTrue(
            this.setSpeed(
                () ->
                    Joysticks.conditionJoystick(
                        () -> controller.getRawAxis(translationAxis),
                        translationLimiter,
                        Constants.stickDeadband,
                        true)))
        .onFalse(this.setSpeed(() -> 0.0));
  }
}
