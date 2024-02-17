package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.util.Constants;

public class ShooterIOSim implements ShooterIO {
  private DCMotorSim lowerMotorSim = new DCMotorSim(DCMotor.getNEO(1), 1, 0.0001);
  private DCMotorSim upperMotorSim = new DCMotorSim(DCMotor.getNEO(1), 1, 0.0001);
  private EncoderSim lowerEncoderSim = new EncoderSim(new Encoder(0, 1));
  private EncoderSim upperEncoderSim = new EncoderSim(new Encoder(2, 3));

  private PIDController lowerController = new PIDController(Constants.ShooterConstants.P, Constants.ShooterConstants.I, Constants.ShooterConstants.D);
  private PIDController upperController = new PIDController(Constants.ShooterConstants.P, Constants.ShooterConstants.I, Constants.ShooterConstants.D);

  private double lowerAppliedVolts = 0.0;
  private double upperAppliedVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    lowerMotorSim.update(0.02);
    upperMotorSim.update(0.02);

    inputs.lowerPositionDegrees = Units.radiansToDegrees(lowerMotorSim.getAngularPositionRad());
    inputs.lowerVelocityRpm = Units.radiansPerSecondToRotationsPerMinute(lowerMotorSim.getAngularVelocityRadPerSec());
    inputs.lowerAppliedVolts = lowerAppliedVolts;
    inputs.lowerCurrentAmps = new double[] {lowerMotorSim.getCurrentDrawAmps()};

    inputs.upperPositionDegrees = Units.radiansToDegrees(upperMotorSim.getAngularPositionRad());
    inputs.upperVelocityRpm = Units.radiansPerSecondToRotationsPerMinute(upperMotorSim.getAngularVelocityRadPerSec());
    inputs.upperAppliedVolts = upperAppliedVolts;
    inputs.upperCurrentAmps = new double[] {upperMotorSim.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double lowerVolts, double upperVolts) {
    lowerAppliedVolts = lowerVolts;
    upperAppliedVolts = upperVolts;
    lowerMotorSim.setInputVoltage(lowerVolts);
    upperMotorSim.setInputVoltage(upperVolts);
  }

  @Override
  public void setVelocity(double lowerRpm, double upperRpm, double lowerFF, double upperFF) {
    lowerController.setSetpoint(lowerRpm);
    upperController.setSetpoint(upperRpm);
    lowerMotorSim.setInputVoltage(lowerController.calculate(lowerMotorSim.getAngularVelocityRadPerSec(), lowerRpm));
    upperMotorSim.setInputVoltage(upperController.calculate(upperMotorSim.getAngularVelocityRadPerSec(), upperRpm));
  }

}
