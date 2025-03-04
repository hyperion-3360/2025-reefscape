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
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
    ALGAEL3,
    LOLLYPOP
  }

  // private static double kP = 20.0;
  // private static double kI = 1.0;
  // private static double kD = 0.0;
  private static double kP = 40.5; // 4?
  private static double kI = 0;
  private static double kD = 0.0;

  private static double kDt = 0.02;

  private static double kMaxVelocity = 20;
  private static double kMaxAcceleration = 6.05;
  private static double kMidVelocity = 8;
  private static double kMidAcceleration = 4;
  private static double kMinVelocity = 8;
  private static double kMinAcceleration = 4;

  private static double kG = 0.41; // barely moves up
  private static double kA = 1.5;
  private static double kV = 4.3;
  private static double kS = 0.0;

  private static double pulleyDiam = 3;
  private static double pulleyCircumference = Math.PI * Units.inchesToMeters(pulleyDiam);
  private static double toRotations = 1 / (2 * Math.PI);
  private static double gearRatio = 9.0;
  private static double elevatorSlack = 0.0;

  // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.
  private final TrapezoidProfile.Constraints m_MaxConstraints =
      new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
  private final TrapezoidProfile.Constraints m_MidConstraints =
      new TrapezoidProfile.Constraints(kMidVelocity, kMidAcceleration);
  private final TrapezoidProfile.Constraints m_MinConstraints =
      new TrapezoidProfile.Constraints(kMinVelocity, kMinAcceleration);
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(kP, kI, kD, m_MaxConstraints, kDt);

  private TalonFXConfiguration m_rightMotorConfig = new TalonFXConfiguration();
  private TalonFXConfiguration m_leftMotorConfig = new TalonFXConfiguration();
  private Follower m_follower = new Follower(Constants.SubsystemInfo.kRightElevatorMotorID, false);
  private TalonFX m_rightElevatorMotor =
      new TalonFX(Constants.SubsystemInfo.kRightElevatorMotorID, "CANivore_3360");
  private TalonFX m_leftElevatorMotor =
      new TalonFX(Constants.SubsystemInfo.kLeftElevatorMotorID, "CANivore_3360");

  private AnalogPotentiometer m_sensor = new AnalogPotentiometer(0);

  private final SlewRateLimiter elevatorLimiter = new SlewRateLimiter(3);

  private double elevatorPos;
  private boolean chronoStarted = false;
  private double chronoStartTime = 0;
  private boolean lowElevatorHeight = true;
  private boolean midHeight = false;
  private double elevatorVelocity;
  private double elevatorSetpoint;
  private desiredHeight heightEnum = desiredHeight.LOW;

  public Elevator() {
    // motor configs
    m_rightMotorConfig.MotorOutput.Inverted =
        Constants.ElevatorConstants.kRightElevatorMotorNotInverted;

    m_rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    m_rightMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
    m_rightMotorConfig.CurrentLimits.SupplyCurrentLimit =
        Constants.ElevatorConstants.kElevatorMotorCurrentLimit;
    m_leftMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
    m_leftMotorConfig.CurrentLimits.SupplyCurrentLimit =
        Constants.ElevatorConstants.kElevatorMotorCurrentLimit;

    m_rightElevatorMotor.getConfigurator().apply(m_rightMotorConfig);
    m_leftElevatorMotor.getConfigurator().apply(m_leftMotorConfig);
    m_leftElevatorMotor.setControl(m_follower);
    m_rightElevatorMotor.setPosition(0.0);
    m_leftElevatorMotor.setPosition(0.0);

    m_controller.setGoal(0.0);
    // test_heightIndex = 0;
  }

  @Override
  public void periodic() {

    if (isElevatorAtBottom()) {
      elevatorSlack =
          (((m_leftElevatorMotor.getPosition().getValueAsDouble() * toRotations)
                      * pulleyCircumference)
                  / gearRatio)
              * 5;
    }
    elevatorSetpoint = m_controller.getSetpoint().position;
    // elevatorPos is changed periodically
    elevatorPos =
        ((((m_leftElevatorMotor.getPosition().getValueAsDouble() * toRotations)
                        * pulleyCircumference)
                    / gearRatio))
                * 10
            - elevatorSlack;
    if (DriverStation.isDisabled()) {
      SetHeight(desiredHeight.LOW);
      return;
    }

    if (isElevatorAtBottom() && heightEnum == desiredHeight.LOW) {
      m_controller.reset(0.0);
      m_rightElevatorMotor.setVoltage(0.0);
    } else {

      elevatorVelocity = m_rightElevatorMotor.getMotorVoltage().getValueAsDouble();
      var feedback = m_controller.calculate(elevatorPos);
      var setPointVelocity = m_controller.getSetpoint().velocity;
      var feedforward = m_feedforward.calculate(setPointVelocity);
      var output = feedback + feedforward;

      if (Climber.climberActivated()) {
        m_controller.reset(0.0);
        m_rightElevatorMotor.setVoltage(0.0);
      } else {
        // Run controller and update motor output
        m_rightElevatorMotor.setVoltage(output);
      }
    }

    if (chronoStarted && (Math.abs(elevatorPos - m_controller.getGoal().position) < 0.02)) {
      chronoStarted = false;
      System.out.println(
          "Elevator arrived in " + (Timer.getFPGATimestamp() - chronoStartTime) + " secs");
    }

    // System.out.println(elevatorPos);
  }

  public void SetHeight(desiredHeight height) {
    // add things to move to desired height
    var heightTarget = 0.0;
    switch (height) {
      case LOW:
        // slowDownWhenDescent(Constants.ElevatorConstants.kElevatorDown);
        heightTarget = Constants.ElevatorConstants.kElevatorDown;
        heightEnum = desiredHeight.LOW;
        lowElevatorHeight = true;
        midHeight = false;
        break;

      case L1:
        // slowDownWhenDescent(Constants.ElevatorConstants.kElevatorDown);
        heightTarget = Constants.ElevatorConstants.kElevatorL1;
        heightEnum = desiredHeight.L1;
        lowElevatorHeight = true;
        midHeight = false;
        break;

      case L2:
        // slowDownWhenDescent(Constants.ElevatorConstants.kElevatorDown);
        heightTarget = Constants.ElevatorConstants.kElevatorL2;
        heightEnum = desiredHeight.L2;
        lowElevatorHeight = false;
        midHeight = true;
        break;

      case L3:
        // slowDownWhenDescent(Constants.ElevatorConstants.kElevatorDown);
        heightTarget = Constants.ElevatorConstants.kElevatorL3;
        heightEnum = desiredHeight.L3;
        lowElevatorHeight = false;
        midHeight = true;
        break;

      case L4:
        // slowDownWhenDescent(Constants.ElevatorConstants.kElevatorDown);
        heightTarget = Constants.ElevatorConstants.kElevatorL4;
        heightEnum = desiredHeight.L4;
        lowElevatorHeight = false;
        midHeight = false;
        break;

      case PROCESSOR:
        // slowDownWhenDescent(Constants.ElevatorConstants.kElevatorDown);
        heightTarget = Constants.ElevatorConstants.kElevatorProcessor;
        heightEnum = desiredHeight.PROCESSOR;
        lowElevatorHeight = true;
        midHeight = false;
        break;

      case NET:
        // slowDownWhenDescent(Constants.ElevatorConstants.kElevatorDown);
        heightTarget = Constants.ElevatorConstants.kElevatorNet;
        heightEnum = desiredHeight.NET;
        lowElevatorHeight = false;
        midHeight = false;
        break;

      case ALGAELOW:
        // slowDownWhenDescent(Constants.ElevatorConstants.kElevatorDown);
        heightTarget = Constants.ElevatorConstants.kElevatorAlgaeLow;
        heightEnum = desiredHeight.ALGAELOW;
        lowElevatorHeight = true;
        midHeight = false;
        break;

      case FEEDER:
        // slowDownWhenDescent(Constants.ElevatorConstants.kElevatorDown);
        heightTarget = Constants.ElevatorConstants.kElevatorFeeder;
        heightEnum = desiredHeight.FEEDER;
        lowElevatorHeight = true;
        midHeight = false;
        break;

      case ALGAEL2:
        // slowDownWhenDescent(Constants.ElevatorConstants.kElevatorDown);
        heightTarget = Constants.ElevatorConstants.kElevatorAlgaeL2;
        heightEnum = desiredHeight.ALGAEL2;
        lowElevatorHeight = false;
        midHeight = true;
        break;

      case ALGAEL3:
        // slowDownWhenDescent(Constants.ElevatorConstants.kElevatorDown);
        heightTarget = Constants.ElevatorConstants.kElevatorAlgaeL3;
        heightEnum = desiredHeight.ALGAEL3;
        lowElevatorHeight = false;
        midHeight = false;
        break;

      case LOLLYPOP:
        // slowDownWhenDescent(Constants.ElevatorConstants.kElevatorDown);
        heightTarget = Constants.ElevatorConstants.kElevatorLollypop;
        heightEnum = desiredHeight.ALGAEL3;
        lowElevatorHeight = true;
        midHeight = false;
        break;
    }

    if (lowElevatorHeight) {
      m_controller.setConstraints(m_MinConstraints);
    } else if (midHeight) {
      m_controller.setConstraints(m_MidConstraints);
    } else {
      m_controller.setConstraints(m_MaxConstraints);
    }
    m_controller.setGoal(heightTarget);
    chronoStarted = true;
    chronoStartTime = Timer.getFPGATimestamp();
  }

  public double getElevatorPos() {
    return elevatorPos;
  }

  public double getElevatorSetpoint() {
    return elevatorSetpoint;
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

  public double elevatorVel() {
    return elevatorVelocity;
  }

  @Override
  public void setupTestBindings(Trigger moduleTrigger, CommandXboxController controller) {

    moduleTrigger
        .and(controller.a())
        .whileTrue(
            this.manualTest(
                () ->
                    Joysticks.conditionJoystick(
                        () -> controller.getLeftY(), elevatorLimiter, 0.0, true)));

    moduleTrigger.and(controller.leftBumper()).onTrue(this.Elevate(Elevator.desiredHeight.L2));

    moduleTrigger.and(controller.x()).onTrue(this.Elevate(Elevator.desiredHeight.NET));

    moduleTrigger.and(controller.y()).onTrue(this.Elevate(Elevator.desiredHeight.L1));

    moduleTrigger.and(controller.b()).onTrue(this.Elevate(Elevator.desiredHeight.L2));
  }

  public desiredHeight getTargetHeight() {
    return heightEnum;
  }

  public boolean isElevatorAtBottom() {
    return m_sensor.get() >= 0.3;
  }
}
