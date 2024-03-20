package frc.robot.subsystems.intake

import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.robot.Robot
import frc.robot.RobotContainer
import frc.robot.subsystems.SwerveSubsystem
import frc.robot.util.Constants.IntakeConstants
import org.littletonrobotics.junction.*
import java.util.function.DoubleSupplier
import java.util.function.IntSupplier
import kotlin.math.abs

// Not doing the sim because I would like to retain what little sanity I have left
@Suppress("ReplaceGetOrSet")
class Intake(val io: IntakeIO) : SubsystemBase() {
    val inputs: IntakeIOInputsAutoLogged = IntakeIOInputsAutoLogged()
    private var runIntake = true

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)
    }

    /** Returns a command that intakes a note.  */


    /** Returns a command that intakes a note. Now with speed-agnostic output.  */
    fun intakeCommand(): Command {
        return runEnd({
            io.setVelocity(IntakeConstants.INTAKE_SPEED, IntakeConstants.CEREALIZER_SPEED)
        }, {
            io[0.0] = 0.0
        })
    }

    fun setVelocity(intakeRpm: Double, cerealizerRpm: Double) {
        io.setVelocity(intakeRpm, cerealizerRpm)
    }

    operator fun set(intakePercent: Double, cerealizerPercent: Double) {
        io[intakePercent] = cerealizerPercent
    }

    fun setIntakeVelocity(intakeRpm: Double) {
        io.setIntakeVelocity(intakeRpm)
    }

    fun setCerealizerVelocity(cerealizerRpm: Double) {
        io.setCerealizerVelocity(cerealizerRpm)
    }

    fun setCerealizer(cerealizer: Double) {
        io.setCerealizer(cerealizer)
    }

    fun setIntake(a: Double) {
        io.setIntake(a)
    }

    fun setRunIntake(runIntake: Boolean) {
        this.runIntake = runIntake
    }

    var intakeSysId: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            null, null, null
        ) { state: SysIdRoutineLog.State -> Logger.recordOutput("SysIdTestState", state.toString()) },
        Mechanism(
            { v: Measure<Voltage?> -> this.setIntakeVoltage(v.`in`(edu.wpi.first.units.Units.Volts)) },
            null,
            this
        )
    )
    var cerealizerSysId: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            null, null, null
        ) { state: SysIdRoutineLog.State -> Logger.recordOutput("SysIdTestState", state.toString()) },
        Mechanism(
            { v: Measure<Voltage?> -> this.setCerealizerVoltage(v.`in`(edu.wpi.first.units.Units.Volts)) },
            null,
            this
        )
    )

    init {
        defaultCommand = run { io[0.0] = 0.0 }
    }

    fun setIntakeVoltage(volts: Double) {
        io.setIntakeVoltage(volts)
    }

    fun setCerealizerVoltage(volts: Double) {
        io.setCerealizerVoltage(volts)
    }


    val colorSensorProximity: DoubleSupplier?
        get() = io.colorSensorProximity

    val colorSensorRed: DoubleSupplier?
        get() = io.colorSensorRed

    val colorSensorGreen: DoubleSupplier?
        get() = io.colorSensorGreen

    val colorSensorBlue: DoubleSupplier?
        get() = io.colorSensorBlue

    val colorSensorIR: DoubleSupplier?
        get() = io.colorSensorIR


    companion object {
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


    }

//    fun getColorSensorProximity() : DoubleSupplier {
//        return  io::colorSensorProximity.get()
//    }
}
