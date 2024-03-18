package frc.robot.subsystems.shooter

import org.littletonrobotics.junction.AutoLog

/**
 *
 */
interface ShooterIO {
    /**
     *
     */
    @AutoLog
    open class ShooterIOInputs {
        /**
         *
         */
        @JvmField
        var lowerPositionDegrees: Double = 0.0
        /**
         *
         */
        @JvmField
        var lowerVelocityRpm: Double = 0.0
        /**
         *
         */
        @JvmField
        var lowerAppliedVolts: Double = 0.0
        /**
         *
         */
        @JvmField
        var lowerRpmSetpoint: Double = 0.0
        /**
         *
         */
        @JvmField
        var lowerSurfaceSpeed: Double = 0.0
        /**
         *
         */
        @JvmField
        var lowerOutput: Double = 0.0
        /**
         *
         */
        @JvmField
        var lowerCurrentAmps: DoubleArray = doubleArrayOf()

        /**
         *
         */
        @JvmField
        var upperPositionDegrees: Double = 0.0
        /**
         *
         */
        @JvmField
        var upperVelocityRpm: Double = 0.0
        /**
         *
         */
        @JvmField
        var upperAppliedVolts: Double = 0.0
        /**
         *
         */
        @JvmField
        var upperRpmSetpoint: Double = 0.0
        /**
         *
         */
        @JvmField
        var upperSurfaceSpeed: Double = 0.0
        /**
         *
         */
        @JvmField
        var upperOutput: Double = 0.0
        /**
         *
         */
        @JvmField
        var upperCurrentAmps: DoubleArray = doubleArrayOf()
    }

    /** Updates the set of loggable inputs.  */
    fun updateInputs(inputs: ShooterIOInputs) {}

    /** Run open loop at the specified voltage.  */
    operator fun set(lowerPercent: Double, upperPercent: Double) {}

    /** Run closed loop at the specified velocity.  */
    fun setVelocity(
        lowerRpm: Double, upperRpm: Double
    ) {
    }

    /**
     *
     */
    fun setVoltage(volts: Double) {}
}
