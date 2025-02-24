// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.Joysticks;
import frc.lib.util.TestBindings;
import frc.robot.Constants;
import frc.robot.Constants.SubsystemInfo;
import java.util.function.DoubleSupplier;

/** 1 falcon winch and unwinch* */
// La classe devrait avoir un option pour seulement s'activer a 30sec de la fin du jeu,
// pour eviter un mouvement par accident du pilot
public class Climber extends SubsystemBase implements TestBindings {

  private TalonFXConfiguration m_climberMotorConfig = new TalonFXConfiguration();
  private TalonFX m_shallowMotor =
      new TalonFX(SubsystemInfo.kClimberShallowMotorID, "CANivore_3360");
  // private TalonFX m_deepMotor = new TalonFX(SubsystemInfo.kClimberDeepMotorID);

  private Double m_direction = 0.0;

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final SlewRateLimiter climberSpeedLimiter = new SlewRateLimiter(3);

  public Climber() {
    // m_deepMotor.setNeutralMode(NeutralModeValue.Brake);
    m_shallowMotor.setNeutralMode(NeutralModeValue.Brake);
    // motor configs
    m_climberMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_climberMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_climberMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_climberMotorConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    m_shallowMotor.getConfigurator().apply(m_climberMotorConfig);
    m_shallowMotor.set(0);
    // m_deepMotor.getConfigurator().apply(m_climberMotorConfig);
    // m_deepMotor.set(0);

    SendableRegistry.add(this, "Climber", 0);
    SmartDashboard.putData("Climber", this);
  }

  // This is for test mode
  public Command shallowClimb(DoubleSupplier speed) {
    return this.run(
        () -> {
          double val = speed.getAsDouble();
          m_direction = Math.signum(val);
          m_shallowMotor.set(Math.pow(val, 2) * m_direction);
        });
  }

  public Command deepClimb(DoubleSupplier speed) {
    return this.run(
        () -> {
          double val = speed.getAsDouble();
          m_direction = Math.signum(val);
          // m_deepMotor.set(Math.pow(val, 2) * m_direction);
        });
  }

  @Override
  public void setupTestBindings(Trigger moduleTrigger, CommandXboxController controller) {

    moduleTrigger
        .and(controller.leftBumper())
        .whileTrue(
            this.shallowClimb(
                () ->
                    Joysticks.conditionJoystick(
                        () -> controller.getRawAxis(translationAxis),
                        climberSpeedLimiter,
                        Constants.stickDeadband,
                        true)));
    moduleTrigger
        .and(controller.rightBumper())
        .whileTrue(
            this.deepClimb(
                () ->
                    Joysticks.conditionJoystick(
                        () -> controller.getRawAxis(translationAxis),
                        climberSpeedLimiter,
                        Constants.stickDeadband,
                        true)));
  }
}
