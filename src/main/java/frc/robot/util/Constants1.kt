package frc.robot.util

import com.pathplanner.lib.util.PIDConstants
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units
import swervelib.math.Matter

/**
 *
 */
object Constants {
    /**
     *
     */
    const val ROBOT_MASS: Double = 39.97 // 32lbs * kg per pound
    /**
     *
     */
    @JvmField
    val CHASSIS: Matter = Matter(Translation3d(0.0, 0.0, Units.inchesToMeters(8.0)), ROBOT_MASS)
    /**
     *
     */
    const val LOOP_TIME: Double = 0.13 //s, 20ms + 110ms sprk max velocity lag

    /**
     *
     */
    const val TUNING_MODE: Boolean = false

    /**
     *
     */
    val CURRENT_MODE: Mode = Mode.REAL

    /**
     *
     */
    object ShooterConstants {
        /**
         *
         */
        const val P: Double = 5e-5
        /**
         *
         */
        const val I: Double = 1e-6
        /**
         *
         */
        const val D: Double = 0.0
        /**
         *
         */
        const val Iz: Double = 0.0
        /**
         *
         */
        const val FF: Double = 0.000156
        /**
         *
         */
        const val MAX_OUTPUT: Double = 1.0
        /**
         *
         */
        const val MIN_OUTPUT: Double = -1.0
        /**
         *
         */
        const val MAX_RPM: Double = 5700.0
        /**
         *
         */
        const val MAX_VELOCITY: Double = 2000.0
        /**
         *
         */
        const val MAX_ACCELERATION: Double = 1500.0


        /**
         *
         */
        const val WHEEL_DIAMETER: Double = 4.0 // inches
    }

    /**
     *
     */
    enum class Mode {
        /**
         * Running on a real robot.
         */
        REAL,

        /**
         * Running a physics simulator.
         */
        SIM,

        /**
         * Replaying from a log file.
         */
        REPLAY
    }

    /**
     *
     */
    object PositionConstants {
        /**
         *
         */
        val ORIGIN_TO_BACK_LIMELIGHT: Pose3d = Pose3d(
            Units.inchesToMeters(-0.25),
            Units.inchesToMeters(0.25),
            Units.inchesToMeters(18.1171875),
            Rotation3d(0.0, 0.0, Math.PI)
        )

        /**
         *
         */
        val ORIGIN_TO_NAVX: Translation3d =
            Translation3d(Units.inchesToMeters(-7.0), Units.inchesToMeters(0.0), Units.inchesToMeters(6.5))
    }

    /**
     *
     */
    object IntakeConstants {
        /**
         *
         */
        private const val HEIGHT_FROM_GROUND: Double = 4.0 // inches
        /**
         *
         */
        const val CIRCUMFERENCE: Double = HEIGHT_FROM_GROUND * 2 * Math.PI // inches
        /**
         *
         */
        const val P: Double = 5e-5
        /**
         *
         */
        const val I: Double = 1e-6
        /**
         *
         */
        const val D: Double = 0.0
        /**
         *
         */
        const val Iz: Double = 0.0
        /**
         *
         */
        const val FF: Double = 0.000156
        /**
         *
         */
        const val MAX_OUTPUT: Double = 1.0
        /**
         *
         */
        const val MIN_OUTPUT: Double = -1.0
        /**
         *
         */
        const val MAX_RPM: Double = 240.0 // 4 rotations per second is the max. Notes are expensive enough as it is.
        /**
         *
         */
        const val MAX_VELOCITY: Double = 240.0 // I don't know what these do but fuck it we ball
        /**
         *
         */
        const val MAX_ACCELERATION: Double = 120.0
        /**
         *
         */
        const val INTAKE_SPEED: Double = 5.0
        /**
         *
         */
        const val CEREALIZER_SPEED: Double = 5.0
    }

    /**
     *
     */
    object AutonomousConstants {
        /**
         *
         */
        val TRANSLATION_PID: PIDConstants = PIDConstants(0.7, 0.0, 0.0)
        /**
         *
         */
        val ANGLE_PID: PIDConstants = PIDConstants(0.04, 0.0, 0.075)
    }

    /**
     *
     */
    object DrivebaseConstants {
        /**
         *
         */// Hold time on motor brakes when disabled
        const val WHEEL_LOCK_TIME: Double = 10.0 // seconds
    }

    /**
     *
     */
    object OperatorConstants {
        /**
         *
         */// Joystick Deadband
        const val JOYSTICK_X_DEADBAND: Double = 0.1
        /**
         *
         */
        const val JOYSTICK_Y_DEADBAND: Double = 0.1
        /**
         *
         */
        const val JOYSTICK_TWIST_DEADBAND: Double = 0.1

        /**
         *
         */
        const val TURN_CONSTANT: Double = 6.0
        /**
         *
         */
        const val LEFT_Y_DEADBAND: Double = 0.1
        /**
         *
         */
        const val LEFT_X_DEADBAND: Double = 0.1
        /**
         *
         */
        const val RIGHT_X_DEADBAND: Double = 0.1
    }

    /**
     *
     */
    object MotorConstants {
        /**
         *
         */// Shooter Motor Constants
        const val LOWER_SHOOTER_MOTOR: Int = 23
        /**
         *
         */
        const val UPPER_SHOOTER_MOTOR: Int = 24
        /**
         *
         */
        const val SHOOTER_SPEED: Double = 0.70

        /**
         *
         */// Arm Motor Constants
        const val LEFT_ARM_MOTOR: Int = 21 // Currently the ids are arbitrary.
        /**
         *
         */
        const val RIGHT_ARM_MOTOR: Int = 20 // They will need to be adjusted to reflect the actual connected ports

        /**
         *
         */// Intake Constants
        const val INTAKE_MOTOR: Int = 12
        /**
         *
         */
        const val CEREALIZER_MOTOR: Int = 22
        /**
         *
         */
        const val INTAKE_SPEED: Double = 0.45
        /**
         *
         */
        const val LEFT_ROLLER_MOTOR: Int = 11
        /**
         *
         */
        const val RIGHT_ROLLER_MOTOR: Int = 10
    }
}