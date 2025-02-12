// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
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

public class Elevator extends SubsystemBase implements TestBindings {

  public enum desiredHeight {
    PROCESSOR,
    NET,
    FEEDER,
    ALGAELOW,
    LOW,
    L1,
    L2,
    L3,
    L4,
    ALGAEL2,
    ALGAEL3
  }

  private static double kP = 4.5;
  private static double kI = 0.0;
  private static double kD = 0.0;

  private static double kDt = 0.02;

  private static double kMaxVelocity = 3;
  private static double kMaxAcceleration = 1.5;

  //  private static double kG = 0.98; // not moving
  private static double kG = 0.70;
  private static double kA = 0.0;
  private static double kV = 3.3;
  private static double kS = 0.2;

  private static double pulleyDiam = 3;
  private static double pulleyCircumference = Math.PI * Units.inchesToMeters(pulleyDiam);
  private static double toRotations = 1 / (2 * Math.PI);

  // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.
  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);

  private TalonFXConfiguration m_rightMotorConfig = new TalonFXConfiguration();
  private TalonFXConfiguration m_leftMotorConfig = new TalonFXConfiguration();
  private Follower m_follower = new Follower(Constants.SubsystemInfo.kRightElevatorMotorID, false);
  private TalonFX m_rightElevatorMotor =
      new TalonFX(Constants.SubsystemInfo.kRightElevatorMotorID, "CANivore_3360");
  private TalonFX m_leftElevatorMotor =
      new TalonFX(Constants.SubsystemInfo.kLeftElevatorMotorID, "CANivore_3360");

  private DigitalInput m_sensor = new DigitalInput(9);

  //  private int test_heightIndex;

  private final int rightTriggerAxis = XboxController.Axis.kRightTrigger.value;
  private final SlewRateLimiter elevatorLimiter = new SlewRateLimiter(3);

  private double elevatorPos;

  private desiredHeight heightEnum = desiredHeight.LOW;

  public Elevator() {
    // motor configs
    m_rightMotorConfig.MotorOutput.Inverted =
        Constants.ElevatorConstants.kRightElevatorMotorNotInverted;

    m_rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    m_rightMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_rightMotorConfig.CurrentLimits.SupplyCurrentLimit =
        Constants.ElevatorConstants.kElevatorMotorCurrentLimit;
    m_leftMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_leftMotorConfig.CurrentLimits.SupplyCurrentLimit =
        Constants.ElevatorConstants.kElevatorMotorCurrentLimit;

    m_rightElevatorMotor.getConfigurator().apply(m_rightMotorConfig);
    m_leftElevatorMotor.getConfigurator().apply(m_leftMotorConfig);
    m_leftElevatorMotor.setControl(m_follower);
    m_rightElevatorMotor.setPosition(0.0, 0.15);
    m_leftElevatorMotor.setPosition(0.0, 0.15);

    // SmartDashboard.putData("Elevator PID", m_feedback);
    // SmartDashboard.putData("Tunable feedforward", m_feedforward);
    SendableRegistry.add(this, "TunableElevator", 0);
    SmartDashboard.putData("ElevatorTuning", this);
    m_controller.setGoal(0.01);
    // test_heightIndex = 0;
  }

  @Override
  public void periodic() {

    var setPointPosition = m_controller.getSetpoint().position;
    // elevatorPos is changed periodically
    elevatorPos =
        (m_rightElevatorMotor.getPosition().getValueAsDouble() * toRotations) * pulleyCircumference;
    SmartDashboard.putNumber("elevator position", elevatorPos);
    SmartDashboard.putNumber("setpoint position", setPointPosition);
    SmartDashboard.putBoolean("elevator sensor", m_sensor.get());

    if (DriverStation.isDisabled()) {
      m_controller.reset(0.01);
      m_controller.setGoal(0.01);
      return;
    }
    if (heightEnum == desiredHeight.LOW && m_controller.atSetpoint()) {
      System.out.println(heightEnum);
      m_controller.reset(0.01);
      m_controller.setGoal(0.01);
    }

    var elevatorVelocity = m_rightElevatorMotor.getMotorVoltage().getValueAsDouble();

    var feedback = m_controller.calculate(elevatorPos);
    var setPointVelocity = m_controller.getSetpoint().velocity;
    var feedforward = m_feedforward.calculate(setPointVelocity);
    var output = feedback + feedforward;

    SmartDashboard.putNumber("elevator velocity", elevatorVelocity);
    SmartDashboard.putNumber("setpoint velocity", setPointVelocity);
    SmartDashboard.putNumber("output voltage", output);
    SmartDashboard.putNumber("error", m_controller.getPositionError());

    // Run controller and update motor output
    m_rightElevatorMotor.setVoltage(output);
  }

  public void SetHeight(desiredHeight height) {
    // add things to move to desired height
    var heightTarget = 0.0;
    switch (height) {
      case LOW:
        heightTarget = Constants.ElevatorConstants.kElevatorDown;
        heightEnum = desiredHeight.LOW;
        break;

      case L1:
        heightTarget = Constants.ElevatorConstants.kElevatorL1;
        heightEnum = desiredHeight.L1;
        break;

      case L2:
        heightTarget = Constants.ElevatorConstants.kElevatorL2;
        heightEnum = desiredHeight.L2;
        break;

      case L3:
        heightTarget = Constants.ElevatorConstants.kElevatorL3;
        heightEnum = desiredHeight.L3;
        break;

      case L4:
        heightTarget = Constants.ElevatorConstants.kElevatorL4;
        heightEnum = desiredHeight.L4;
        break;

      case PROCESSOR:
        heightTarget = Constants.ElevatorConstants.kElevatorProcessor;
        heightEnum = desiredHeight.PROCESSOR;
        break;

      case NET:
        heightTarget = Constants.ElevatorConstants.kElevatorNet;
        heightEnum = desiredHeight.NET;
        break;

      case ALGAELOW:
        heightTarget = Constants.ElevatorConstants.kElevatorAlgaeLow;
        heightEnum = desiredHeight.ALGAELOW;
        break;

      case FEEDER:
        heightTarget = Constants.ElevatorConstants.kElevatorFeeder;
        heightEnum = desiredHeight.FEEDER;
        break;

      case ALGAEL2:
        heightTarget = Constants.ElevatorConstants.kElevatorAlgaeL2;
        heightEnum = desiredHeight.ALGAEL2;
        break;

      case ALGAEL3:
        heightTarget = Constants.ElevatorConstants.kElevatorAlgaeL3;
        heightEnum = desiredHeight.ALGAEL3;
        break;
    }
    m_controller.setGoal(heightTarget);
  }

  public Command manualTest(DoubleSupplier speed) {
    return run(
        () -> {
          System.out.println(String.format("Elevator speed: %f", speed.getAsDouble()));
          m_rightElevatorMotor.set(speed.getAsDouble());
        });
  }

  public Command Elevate(desiredHeight height) {

    return runOnce(() -> SetHeight(height));
    // desiredHeight[] heightArray = {
    //   desiredHeight.L1, desiredHeight.L2, desiredHeight.L3, desiredHeight.L4, desiredHeight.NET
    // };
    // return runOnce(
    //     () -> {
    //       System.out.println("Before: " + test_heightIndex);
    //       desiredHeight testHeight = heightArray[test_heightIndex];
    //       test_heightIndex++;
    //       test_heightIndex = test_heightIndex % heightArray.length;
    //       System.out.println("After: " + test_heightIndex);
    //       SetHeight(testHeight);
    //     });
  }

  public double getEncoderPos() {
    return elevatorPos;
  }

  @Override
  public void setupTestBindings(Trigger moduleTrigger, CommandXboxController controller) {

    moduleTrigger
        .and(controller.a())
        .whileTrue(
            this.manualTest(
                () ->
                    Joysticks.conditionJoystick(
                        () -> controller.getRawAxis(rightTriggerAxis),
                        elevatorLimiter,
                        0.0,
                        false)));

    moduleTrigger.and(controller.leftBumper()).onTrue(this.Elevate(Elevator.desiredHeight.L2));

    moduleTrigger.and(controller.x()).onTrue(this.Elevate(Elevator.desiredHeight.NET));

    moduleTrigger.and(controller.y()).onTrue(this.Elevate(Elevator.desiredHeight.L1));

    moduleTrigger.and(controller.b()).onTrue(this.Elevate(Elevator.desiredHeight.L2));
  }

  public desiredHeight getTargetHeight() {
    return heightEnum;
  }
}
