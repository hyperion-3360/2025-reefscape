// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

/** Add your docs here. */
public class TunableElevatorFF implements Sendable {
  private static int instances;
  private double k_s = 0.0;
  private double k_g = 0.0;
  private double k_v = 0.0;
  private ElevatorFeedforward m_feedForward;

  public TunableElevatorFF(double s, double g, double v) {
    k_s = s;
    k_g = g;
    k_v = v;
    renewFF();
    SendableRegistry.addLW(this, "Tunable Elevator Feedforward", instances++);
  }

  public double getS() {
    return m_feedForward.getKs();
  }

  public void setS(double s) {
    k_s = s;
    renewFF();
  }

  public double getG() {
    return m_feedForward.getKg();
  }

  public void setG(double g) {
    k_g = g;
    renewFF();
  }

  public double getV() {
    return m_feedForward.getKv();
  }

  public void setV(double v) {
    k_v = v;
    renewFF();
  }

  private void renewFF() {
    m_feedForward = new ElevatorFeedforward(k_s, k_g, k_v, 0.0);
  }

  public double calculate(double velocity) {
    return m_feedForward.calculate(velocity);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Tunable Elevator Feedforward");
    builder.addDoubleProperty("Ks", this::getS, this::setS);
    builder.addDoubleProperty("Kg", this::getG, this::setG);
    builder.addDoubleProperty("Kv", this::getV, this::setV);
  }
}
