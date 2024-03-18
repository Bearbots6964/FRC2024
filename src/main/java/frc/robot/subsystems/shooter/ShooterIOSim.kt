package frc.robot.subsystems.shooter

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import edu.wpi.first.wpilibj.simulation.EncoderSim
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs
import frc.robot.util.Constants.ShooterConstants

/**
 *
 */
class ShooterIOSim : ShooterIO {
    private val lowerMotorSim = DCMotorSim(DCMotor.getNEO(1), 1.0, 0.0001)
    private val upperMotorSim = DCMotorSim(DCMotor.getNEO(1), 1.0, 0.0001)
    private val lowerEncoderSim = EncoderSim(Encoder(0, 1))
    private val upperEncoderSim = EncoderSim(Encoder(2, 3))

    private val lowerController = PIDController(ShooterConstants.P, ShooterConstants.I, ShooterConstants.D)
    private val upperController = PIDController(ShooterConstants.P, ShooterConstants.I, ShooterConstants.D)

    private val lowerAppliedVolts = 0.0
    private val upperAppliedVolts = 0.0

    override fun updateInputs(inputs: ShooterIOInputs) {
        lowerMotorSim.update(0.02)
        upperMotorSim.update(0.02)

        inputs.lowerPositionDegrees = Units.radiansToDegrees(lowerMotorSim.angularPositionRad)
        inputs.lowerVelocityRpm = Units.radiansPerSecondToRotationsPerMinute(lowerMotorSim.angularVelocityRadPerSec)
        inputs.lowerAppliedVolts = lowerAppliedVolts
        inputs.lowerCurrentAmps = doubleArrayOf(lowerMotorSim.currentDrawAmps)

        inputs.upperPositionDegrees = Units.radiansToDegrees(upperMotorSim.angularPositionRad)
        inputs.upperVelocityRpm = Units.radiansPerSecondToRotationsPerMinute(upperMotorSim.angularVelocityRadPerSec)
        inputs.upperAppliedVolts = upperAppliedVolts
        inputs.upperCurrentAmps = doubleArrayOf(upperMotorSim.currentDrawAmps)
    }

    override fun set(lowerPercent: Double, upperPercent: Double) {
        lowerMotorSim.setInputVoltage(lowerPercent * 12)
        upperMotorSim.setInputVoltage(upperPercent * 12)
    }

    override fun setVelocity(lowerRpm: Double, upperRpm: Double) {
        lowerController.setpoint = lowerRpm
        upperController.setpoint = upperRpm
        lowerMotorSim.setInputVoltage(lowerController.calculate(lowerMotorSim.angularVelocityRadPerSec, lowerRpm))
        upperMotorSim.setInputVoltage(upperController.calculate(upperMotorSim.angularVelocityRadPerSec, upperRpm))
    }
}
