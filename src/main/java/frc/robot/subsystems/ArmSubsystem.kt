package frc.robot.subsystems

import com.revrobotics.*
import com.revrobotics.CANSparkBase.IdleMode
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.robot.FieldConstants.Speaker
import frc.robot.util.Constants.MotorConstants
import org.littletonrobotics.junction.*
import kotlin.math.atan2

class ArmSubsystem private constructor() : SubsystemBase() {
    // With eager singleton initialization, any static variables/fields used in the
    // constructor must appear before the "INSTANCE" variable so that they are initialized
    // before the constructor is called when the "INSTANCE" variable initializes.
    // TODO: Set the default command, if any, for this subsystem by calling
    // setDefaultCommand(command)
    //       in the constructor or in the robot coordination class, such as RobotContainer.
    //       Also, you can call addChild(name, sendableChild) to associate sendables with the
    // subsystem
    //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    private val leftMotor =
        CANSparkMax(MotorConstants.LEFT_ARM_MOTOR, CANSparkLowLevel.MotorType.kBrushless) // Lead motor
    private val rightMotor =
        CANSparkMax(MotorConstants.RIGHT_ARM_MOTOR, CANSparkLowLevel.MotorType.kBrushless) // Follow Motor
    private val forwardLimitSwitch: SparkLimitSwitch
    private val reverseLimitSwitch: SparkLimitSwitch

    private val pidController: SparkPIDController
    private val encoder: SparkAbsoluteEncoder = rightMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle)
    private var maxRange = 0.0 // amount of revolutions (or other unit) it takes to move the arm from min to max
    private val setpoint: Double


    // Raises the arm (the right arm is set to mirror the left arm)
    fun raiseArm() {
        leftMotor.set(0.5)
    }

    fun moveArmButBetter(speed: Double) {
        leftMotor.set(speed)
    }

    fun moveArmButWithVelocity(velocity: Double) {
        pidController.setReference(velocity, CANSparkBase.ControlType.kVelocity)
    }

    // Lowers the arm ()
    fun lowerArm() {
        leftMotor.set(-0.5)
    }

    // Control the arm via voltage
    fun setVoltage(voltage: Double) {
        leftMotor.setVoltage(voltage)
    }

    // This should set the current arm location to be zero on the encoder?
    // MAKE SURE THAT THE ZERO OFFSET IS SET WHEN THE ARM IS AT IT'S LOWER LIMIT
    // Set lower limit before upper limit.
    fun setZeroOffset() {
        encoder.setZeroOffset(encoder.position)
    }

    // Sets max range
    fun setMaxRange() {
        maxRange = encoder.position
    }

    private fun rotate(rotations: Double) {
        pidController.setReference(rotations, CANSparkBase.ControlType.kPosition)
    }

    // Takes a double 0 to 1 as a parameter, where 0 is the arm at its lower limit
    // and 1 at its upper limit.
    fun moveArm(p: Double) {
        val distanceFromMin = p * maxRange
        val distanceFromFinal = distanceFromMin - encoder.position // Final - inital = delta position.
        rotate(distanceFromFinal)
    }


    var sysId: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            null, null, null
        ) { state: SysIdRoutineLog.State -> Logger.recordOutput("SysIdTestState", state.toString()) },
        Mechanism({ v: Measure<Voltage?> -> this.setVoltage(v.`in`(Units.Volts)) }, null, this)
    )

    /**
     * Creates a new instance of this ArmSubsystem. This constructor is private since this class
     * is a Singleton. Code should use the [.getInstance] method to get the singleton
     * instance.
     */
    init {
        encoder.setPositionConversionFactor(360.0) // 1 rotation = 360 degrees

        // I don't want to have to deal with radians because of rounding errors

        // The right motor will mirror the leader, but it will be inverted
        rightMotor.follow(leftMotor, true)

        leftMotor.setIdleMode(IdleMode.kBrake)
        leftMotor.setSmartCurrentLimit(40)
        leftMotor.burnFlash()

        rightMotor.setIdleMode(IdleMode.kBrake)
        rightMotor.setSmartCurrentLimit(40)
        rightMotor.burnFlash()

        pidController = rightMotor.pidController


        forwardLimitSwitch = rightMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen)
        reverseLimitSwitch = rightMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen)

        forwardLimitSwitch.enableLimitSwitch(true)
        reverseLimitSwitch.enableLimitSwitch(true)

        pidController.setFeedbackDevice(encoder)
        pidController.setP(0.01)
        pidController.setI(0.0)
        pidController.setD(0.0)
        pidController.setPositionPIDWrappingMinInput(-0.75)
        pidController.setPositionPIDWrappingMaxInput(0.75)

        setpoint = encoder.position

        SmartDashboard.putNumber("Arm Setpoint", setpoint)


        encoder.setPositionConversionFactor(360.0) // 1 rotation = 360 degrees


        // I don't want to have to deal with radians because of rounding errors


        //
//    pidController.setP(kP);
//    pidController.setI(kI);
//    pidController.setD(kD);
//    pidController.setIZone(kIz);
//    pidController.setFF(kFF);
//    pidController.setOutputRange(kMinOutput, kMaxOutput);
        pidController.setFeedbackDevice(encoder)
    }

    fun moveArmToAngle(angle: Double) {
        pidController.setReference(angle, CANSparkBase.ControlType.kPosition)
    }


    override fun periodic() {
        // This method will be called once per scheduler run

        // 0 degrees is set to (r, 270) in polar coordinates, because that way it will never wrap around
        // If the forward limit switch is pressed, we're at 243 degrees

        if (forwardLimitSwitch.isPressed) {
            encoder.setZeroOffset(242.88780212402344)
        }
        // If the reverse limit switch is pressed, we're at 40 degrees
        if (reverseLimitSwitch.isPressed) {
            encoder.setZeroOffset(39.87791442871094)
        }

        //pidController.setReference(SmartDashboard.getNumber("Arm Setpoint", setpoint), CANSparkMax.ControlType.kPosition);
    }

    fun getArmAngleForShooting(robotPose: Pose3d): Double {
        val alliance = DriverStation.getAlliance().get()

        // add 15.5 inches to the height of the robot to get the height of the shooter
        val shooterHeight = robotPose.translation.z + 15.5

        val speaker = Speaker.CENTER_SPEAKER_OPENING

        // distance from the robot to the speaker
        val distance = speaker.getDistance(robotPose.translation)

        val difference = speaker.minus(robotPose.translation)

        // get that angle
        var angle = atan2(difference.z, difference.x)
        // that angle uses (1, 0) as the 0 degree mark, so we need to add 90 degrees to it, because the arm uses (0, -1) as the 0 degree mark
        angle += Math.PI / 2
        angle = edu.wpi.first.math.util.Units.radiansToDegrees(angle)
        return angle
    }

    val angle: Double
        get() = encoder.position

    companion object {
        private const val kP = 0.1
        private const val kI = 1e-4
        private const val kD = 1.0
        private const val kIz = 0.0
        private const val kFF = 0.0
        private const val kMaxOutput = 1.0
        private const val kMinOutput = -1.0 // We'll probably want to put these in the constants class later
        /**
         * Returns the Singleton instance of this ArmSubsystem. This static method should be used,
         * rather than the constructor, to get the single instance of this class. For example: `ArmSubsystem.getInstance();`
         */
        /**
         * The Singleton instance of this ArmSubsystem. Code should use the [.getInstance]
         * method to get the single instance (rather than trying to construct an instance of this class.)
         */
        val instance: ArmSubsystem = ArmSubsystem()

        fun generateSysIdCommand(
            sysIdRoutine: SysIdRoutine?, delay: Double, quasiTimeout: Double,
            dynamicTimeout: Double
        ): Command {
            return sysIdRoutine!!.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(quasiTimeout)
                .andThen(Commands.waitSeconds(delay))
                .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(quasiTimeout))
                .andThen(Commands.waitSeconds(delay))
                .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(dynamicTimeout))
                .andThen(Commands.waitSeconds(delay))
                .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(dynamicTimeout))
        }
    }
}
