package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  // Cerealizer (n.) - 1. A device that dispenses cereal. 2. A typo the robot lead made that stuck.
  // fr tho this is hilarious
  @AutoLog
  public static class IntakeIOInputs {
    public double intakePositionDegrees = 0.0;
    public double intakeVelocityRpm = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double[] intakeCurrentAmps = new double[] {};

    public double cerealizerPositionDegrees = 0.0;
    public double cerealizerVelocityRpm = 0.0;
    public double cerealizerAppliedVolts = 0.0;
    public double[] cerealizerCurrentAmps = new double[] {};

  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void set(double intakePercent, double cerealizerPercent) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(
      double intakeRpm, double cerealizerRpm) {}

  public default void setIntakeVelocity(double intakeRpm) {}
  public default void setCerealizerVelocity(double cerealizerRpm) {}

  public default void setCerealizer(double a) { }
  public default void setIntake(double a) {}
}
