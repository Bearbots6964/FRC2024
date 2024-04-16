package frc.robot.subsystems.intake

import edu.wpi.first.wpilibj.util.Color
import org.littletonrobotics.junction.AutoLog
import java.util.function.DoubleSupplier
import java.util.function.Supplier

/**
 *
 */
interface IntakeIO {
    /**
     *
     */// Cerealizer (n.) - 1. A device that dispenses cereal. 2. A typo the robot lead made that stuck.
    // fr tho this is hilarious
    @AutoLog
    open class IntakeIOInputs {
        /**
         *
         */
        @JvmField
        var intakePositionDegrees: Double = 0.0
        /**
         *
         */
        @JvmField
        var intakeVelocityRpm: Double = 0.0
        /**
         *
         */
        @JvmField
        var intakeAppliedVolts: Double = 0.0
        /**
         *
         */
        @JvmField
        var intakeCurrentAmps: DoubleArray = doubleArrayOf()

        /**
         *
         */
        @JvmField
        var cerealizerPositionDegrees: Double = 0.0
        /**
         *
         */
        @JvmField
        var cerealizerVelocityRpm: Double = 0.0
        /**
         *
         */
        @JvmField
        var cerealizerAppliedVolts: Double = 0.0

        /**
         *
         */
        @JvmField
        var cerealizerCurrentAmps: DoubleArray = doubleArrayOf()
    }

    /** Updates the set of loggable inputs.  */
    fun updateInputs(inputs: IntakeIOInputs) {}

    /** Run open loop at the specified voltage.  */
    operator fun set(intakePercent: Double, cerealizerPercent: Double) {}
    operator fun set(intakeMotorPercent: Double, intakePercent: Double, cerealizerPercent: Double) {}

    /** Run closed loop at the specified velocity.  */
    fun setVelocity(
        intakeRpm: Double, cerealizerRpm: Double
    ) {
    }

    /**
     *
     */
    fun setIntakeVelocity(intakeRpm: Double) {}
    /**
     *
     */
    fun setCerealizerVelocity(cerealizerRpm: Double) {}

    /**
     *
     */
    fun setCerealizer(a: Double) {}
    /**
     *
     */
    fun setIntake(a: Double) {}

    /**
     *
     */
    fun setIntakeVoltage(volts: Double) {}
    /**
     *
     */
    fun setCerealizerVoltage(volts: Double) {}

    /**
     *
     */
    val colorSensorProximity: DoubleSupplier

    /**
     *
     */
    val colorSensorRed: DoubleSupplier

    /**
     *
     */
    val colorSensorGreen: DoubleSupplier

    /**
     *
     */
    val colorSensorBlue: DoubleSupplier

    /**
     *
     */
    val colorSensorIR: DoubleSupplier

    /**
     *
     */
    val colorSensorColor: Supplier<Color>
}
