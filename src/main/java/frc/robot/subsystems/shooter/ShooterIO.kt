package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double lowerPositionDegrees = 0.0;
    public double lowerVelocityRpm = 0.0;
    public double lowerAppliedVolts = 0.0;
    public double lowerRpmSetpoint = 0.0;
    public double lowerSurfaceSpeed = 0.0;
    public double lowerOutput = 0.0;
    public double[] lowerCurrentAmps = new double[] {};

    public double upperPositionDegrees = 0.0;
    public double upperVelocityRpm = 0.0;
    public double upperAppliedVolts = 0.0;
    public double upperRpmSetpoint = 0.0;
    public double upperSurfaceSpeed = 0.0;
    public double upperOutput = 0.0;
    public double[] upperCurrentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void set(double lowerPercent, double upperPercent) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(
      double lowerRpm, double upperRpm) {}

  public default void setVoltage(double volts) {}


}
