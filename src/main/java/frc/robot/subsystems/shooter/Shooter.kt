package frc.robot.subsystems.shooter

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.robot.util.Constants.ShooterConstants
import org.littletonrobotics.junction.Logger

class Shooter(private val io: ShooterIO) : SubsystemBase() {
    private val inputs = ShooterIOInputsAutoLogged()
    private val feedforward = ShooterConstants.FF

    init {
        defaultCommand = run { io.set(0.0, 0.0) }
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Shooter", inputs)
    }

    operator fun set(lowerPercent: Double, upperPercent: Double) {
        io[lowerPercent] = upperPercent
    }

    fun setVelocity(lowerRpm: Double, upperRpm: Double) {
        io.setVelocity(lowerRpm, upperRpm)
    }


    val sysIdRoutine: SysIdRoutine
        get() = SysIdRoutine(
            SysIdRoutine.Config(),
            Mechanism({ v: Measure<Voltage?> -> io.setVoltage(v.`in`(Units.Volt)) }, null, this)
        )

    fun generateSysIdCommand(
        sysIdRoutine: SysIdRoutine, delay: Double, quasiTimeout: Double,
        dynamicTimeout: Double
    ): Command {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(quasiTimeout)
            .andThen(Commands.waitSeconds(delay))
            .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(quasiTimeout))
            .andThen(Commands.waitSeconds(delay))
            .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(dynamicTimeout))
            .andThen(Commands.waitSeconds(delay))
            .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(dynamicTimeout))
    }

    companion object {
        const val launchVelocity: Double = ShooterConstants.MAX_RPM
    }
}
