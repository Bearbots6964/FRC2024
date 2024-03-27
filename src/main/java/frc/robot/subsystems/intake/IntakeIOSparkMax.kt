package frc.robot.subsystems.intake

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.ColorSensorV3
import com.revrobotics.REVLibError
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.I2C
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs
import frc.robot.util.Constants.MotorConstants
import java.util.function.DoubleSupplier
import java.util.function.Supplier

// apparently staying up late coding leads to deranged comments so that's cool
class IntakeIOSparkMax : IntakeIO {
    private val intakeMotor = CANSparkMax(MotorConstants.INTAKE_MOTOR, CANSparkLowLevel.MotorType.kBrushless)
    private val leftRollerMotor = CANSparkMax(MotorConstants.LEFT_ROLLER_MOTOR, CANSparkLowLevel.MotorType.kBrushless)
    private val rightRollerMotor = CANSparkMax(MotorConstants.RIGHT_ROLLER_MOTOR, CANSparkLowLevel.MotorType.kBrushless)
    private val cerealizerMotor = CANSparkMax(MotorConstants.CEREALIZER_MOTOR, CANSparkLowLevel.MotorType.kBrushless)

    private var intakeError: REVLibError? = null
    private var leftRollerError: REVLibError? = null
    private var rightRollerError: REVLibError? = null
    private var cerealizerError: REVLibError? = null

    init {
        intakeMotor.setCANTimeout(250)
        leftRollerMotor.setCANTimeout(250)
        rightRollerMotor.setCANTimeout(250)
        cerealizerMotor.setCANTimeout(250)

        intakeMotor.setIdleMode(IdleMode.kBrake)
        leftRollerMotor.setIdleMode(IdleMode.kBrake)
        rightRollerMotor.setIdleMode(IdleMode.kBrake)
        cerealizerMotor.setIdleMode(IdleMode.kBrake)

        intakeMotor.inverted = true
        leftRollerMotor.inverted = true
        rightRollerMotor.inverted = false
        cerealizerMotor.inverted = false

        intakeMotor.pidController.setFeedbackDevice(intakeMotor.encoder)
        leftRollerMotor.pidController.setFeedbackDevice(leftRollerMotor.encoder)
        rightRollerMotor.pidController.setFeedbackDevice(rightRollerMotor.encoder)
        cerealizerMotor.pidController.setFeedbackDevice(cerealizerMotor.encoder)

        intakeMotor.setSmartCurrentLimit(40)
        leftRollerMotor.setSmartCurrentLimit(20)
        rightRollerMotor.setSmartCurrentLimit(20)
        cerealizerMotor.setSmartCurrentLimit(20)

        intakeMotor.encoder.setVelocityConversionFactor(1.0) // 15:1 gear ratio
        leftRollerMotor.encoder.setVelocityConversionFactor(1.0) // 20:1 gear ratio
        rightRollerMotor.encoder.setVelocityConversionFactor(1.0) // 20:1 gear ratio
        cerealizerMotor.encoder.setVelocityConversionFactor(1.0) // 20:1 gear ratio

        intakeMotor.pidController.setP(0.001)
        intakeMotor.pidController.setI(0.0)
        intakeMotor.pidController.setD(0.003)
        intakeMotor.pidController.setIZone(0.0)
        intakeMotor.pidController.setFF(0.001)
        intakeMotor.pidController.setOutputRange(-1.0, 1.0)
        intakeMotor.pidController.setSmartMotionMaxVelocity(1800.0, 0)
        intakeMotor.pidController.setSmartMotionMinOutputVelocity(0.0, 0)
        intakeMotor.pidController.setSmartMotionMaxAccel(1000.0, 0)


        cerealizerMotor.pidController.setP(0.0005)
        cerealizerMotor.pidController.setI(0.0)
        cerealizerMotor.pidController.setD(0.005)
        cerealizerMotor.pidController.setIZone(0.0)
        cerealizerMotor.pidController.setFF(0.0)
        cerealizerMotor.pidController.setOutputRange(-1.0, 1.0)
        cerealizerMotor.pidController.setSmartMotionMaxVelocity(5200.0, 0)
        cerealizerMotor.pidController.setSmartMotionMaxAccel(2000.0, 0)
        cerealizerMotor.pidController.setSmartMotionMinOutputVelocity(0.0, 0)

        intakeMotor.burnFlash()
        leftRollerMotor.burnFlash()
        rightRollerMotor.burnFlash()
        cerealizerMotor.burnFlash()
    }

    override fun updateInputs(inputs: IntakeIOInputs) {
        inputs.intakePositionDegrees = intakeMotor.encoder.position
        inputs.intakeVelocityRpm = intakeMotor.encoder.velocity
        inputs.intakeAppliedVolts = intakeMotor.appliedOutput * intakeMotor.busVoltage
        inputs.intakeCurrentAmps = doubleArrayOf(intakeMotor.outputCurrent)

        inputs.cerealizerPositionDegrees = cerealizerMotor.encoder.position
        inputs.cerealizerVelocityRpm = cerealizerMotor.encoder.velocity
        inputs.cerealizerAppliedVolts = cerealizerMotor.appliedOutput * cerealizerMotor.busVoltage
        inputs.cerealizerCurrentAmps = doubleArrayOf(cerealizerMotor.outputCurrent)

        SmartDashboard.putNumber("Color Sensor Proximity", Companion.colorSensor.proximity.toDouble())
        SmartDashboard.putBoolean("Has Note?", Companion.colorSensor.proximity.toDouble() < 150.0)
    }

    override fun set(intakePercent: Double, cerealizerPercent: Double) {
        intakeMotor.set(intakePercent)
        leftRollerMotor.set(intakePercent)
        rightRollerMotor.set(intakePercent)
        cerealizerMotor.set(cerealizerPercent)
    }

    override fun setVelocity(intakeRpm: Double, cerealizerRpm: Double) {
        // clamp the RPM to the max RPM
        var intakeRpm = intakeRpm
        var cerealizerRpm = cerealizerRpm
        intakeRpm = MathUtil.clamp(intakeRpm, 0.0, MAX_RPM)
        cerealizerRpm = MathUtil.clamp(cerealizerRpm, 0.0, MAX_RPM)
        // kid named cambrian explosion
        intakeMotor.pidController.setReference(intakeRpm, CANSparkBase.ControlType.kSmartVelocity, 0)
        cerealizerMotor.pidController.setReference(cerealizerRpm, CANSparkBase.ControlType.kSmartVelocity, 0)
    }



    override fun setIntakeVelocity(intakeRpm: Double) {
        intakeMotor.pidController.setReference(intakeRpm, CANSparkBase.ControlType.kSmartVelocity, 0)
    }

    override fun setCerealizerVelocity(cerealizerRpm: Double) {
        cerealizerMotor.pidController.setReference(cerealizerRpm, CANSparkBase.ControlType.kSmartVelocity, 0)
    }

    override fun setCerealizer(a: Double) {
        cerealizerMotor.set(a)
    }

    override fun setIntake(a: Double) {
        intakeMotor.set(a)
    }


    override fun setIntakeVoltage(volts: Double) {
        intakeMotor.setVoltage(volts)
    }

    override fun setCerealizerVoltage(volts: Double) {
        cerealizerMotor.setVoltage(volts)
    }


    override val colorSensorProximity: DoubleSupplier
        get() = DoubleSupplier { Companion.colorSensor.proximity.toDouble() }

    override val colorSensorRed: DoubleSupplier
        get() = DoubleSupplier { Companion.colorSensor.red.toDouble() }

    override val colorSensorGreen: DoubleSupplier
        get() = DoubleSupplier { Companion.colorSensor.green.toDouble() }

    override val colorSensorBlue: DoubleSupplier
        get() = DoubleSupplier { Companion.colorSensor.blue.toDouble() }

    override val colorSensorIR: DoubleSupplier
        get() = DoubleSupplier { Companion.colorSensor.ir.toDouble() }

    override val colorSensorColor: Supplier<Color>
        get() = Supplier { Companion.colorSensor.color }

    companion object {
        const val MAX_RPM: Double = 5700.0
        val colorSensor = ColorSensorV3(I2C.Port.kMXP)
    }
}
