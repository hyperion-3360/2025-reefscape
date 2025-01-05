package frc.lib.util;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

/**
 * As described in https://github.com/wpilibsuite/allwpilib/issues/5284 the DutyCycleEncoder cannot
 * be used reliably with CTRE magnetic encoder, the jitter is well above 1 degree (when one should
 * expect the encoder to be predise to 0.08 degree (12 bits resolution) The following class enables
 * the user to perform a calibration of the frequency, this is not perfect as the "real" frequency
 * turns out to be a simple interpolation between the min and max frequencies recorded during the
 * calibration period. But in practice it turns out to be sufficient to get a 0.1 degree accuracy
 */
public class CTREMagEncoder extends DutyCycle {
  private int m_freqSum = 0;
  private int m_numFreq = 0;
  private double m_realFreq = 0.0;

  /**
   * Constructor
   *
   * @param channel just like the DutyCycleEncoder take the DIO channel number to read PWM from
   */
  public CTREMagEncoder(int channel) {
    super(new DigitalInput(channel));
  }

  /**
   * @return the interpolated frequency between max and min recorded during the calibration phase
   */
  public double getRealFrequency() {
    return m_realFreq;
  }

  /**
   * Perform a simple sampling of the current PWM frequency. It is suggested to call this routine in
   * a loop with delay between calls. A loop of 10 calls with 100ms between calls has proven to
   * yield good results in practice
   */
  public void calibrate() {
    m_freqSum += getFrequency();
    m_numFreq += 1;
  }

  /**
   * This function has to be called before one can reliably use the encoder values. Calling this
   * function should be done after the calibration loop has been performed
   */
  public void finishCalibration() {
    m_realFreq = (double) m_freqSum / (double) m_numFreq;
  }

  /**
   * Get the absolution encoder position
   *
   * @return a normalized value between 0 and 1.0
   */
  public double getAbsolutePosition() {
    var currentVal = (double) getHighTimeNanoseconds();
    var raw12bit = Math.abs((currentVal * m_realFreq * 1e-6 * 4.096) - 1);
    return raw12bit / Math.max(raw12bit, 4096.0);
  }

  /** Provides information about this encoder on the SmartDashboard */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Duty Cycle");
    builder.addDoubleProperty("Frequency", this::getRealFrequency, null);
    builder.addDoubleProperty("Output", this::getAbsolutePosition, null);
  }
}
;
