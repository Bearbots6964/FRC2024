package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
  public static final double launchVelocity = Constants.ShooterConstants.MAX_RPM;
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final double feedforward = Constants.ShooterConstants.FF;

  public Shooter(ShooterIO io) {
    this.io = io;
    setDefaultCommand(
        run(
            () -> io.set(0.0, 0.0)));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public void set(double lowerPercent, double upperPercent) {
    io.set(lowerPercent, upperPercent);
  }

  public void setVelocity(double lowerRpm, double upperRpm) {
    io.setVelocity(lowerRpm, upperRpm);
  }

}
