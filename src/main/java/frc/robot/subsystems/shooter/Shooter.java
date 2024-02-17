package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  public static final double launchVelocity = Constants.ShooterConstants.MAX_RPM;
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final double feedforward = Constants.ShooterConstants.FF;

  public Shooter(ShooterIO io) {
    this.io = io;
    setDefaultCommand(
        run(
            () -> io.setVoltage(0.0, 0.0)));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public void setVoltage(double lowerVolts, double upperVolts) {
    io.setVoltage(lowerVolts, upperVolts);
  }

  public void setVelocity(double lowerRpm, double upperRpm, double lowerFF, double upperFF) {
    io.setVelocity(lowerRpm, upperRpm, lowerFF, upperFF);
  }

  /**
   * Returns a command that shoots a note.
   */
  public Command shootCommand() {
    return Commands.startEnd(
        () -> io.setVelocity(launchVelocity, launchVelocity, feedforward, feedforward),
        () -> io.setVoltage(0.0, 0.0));

  }
}
