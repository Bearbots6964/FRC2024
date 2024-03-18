package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.util.LimelightHelpers
import org.littletonrobotics.junction.*

class VisionSubsystem
/**
 * Creates a new instance of this VisionSubsystem. This constructor is private since this class is
 * a Singleton. Code should use the [.getInstance] method to get the singleton instance.
 */
private constructor() : SubsystemBase() {
    /**
     *
     */
    override fun periodic() {
        // This method will be called once per scheduler run


        Logger.recordOutput("Targets", LimelightHelpers.getTargetPose3d_RobotSpace("limelight-back"))
    }

    /**
     *
     */// simple proportional turning control with Limelight.
    // "proportional control" is a control algorithm in which the output is proportional to the error.
    // in this case, we are going to return an angular velocity that is proportional to the
    // "tx" value from the Limelight.
    fun limelightAimProportionalFront(): Double {
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
        // if it is too high, the robot will oscillate.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        val kP = .035

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of
        // your limelight 3 feed, tx should return roughly 31 degrees.
        var targetingAngularVelocity = LimelightHelpers.getTX("limelight-front") * kP

        // convert to radians per second for our drive method
        targetingAngularVelocity *= SwerveSubsystem.instance.swerveDrive.maximumAngularVelocity

        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0

        return targetingAngularVelocity
    }

    /**
     *
     */
    fun limelightAimProportionalBack(): Double {
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
        // if it is too high, the robot will oscillate.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        val kP = .035

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of
        // your limelight 3 feed, tx should return roughly 31 degrees.
        var targetingAngularVelocity = LimelightHelpers.getTX("limelight-back") * kP

        // convert to radians per second for our drive method
        targetingAngularVelocity *= SwerveSubsystem.instance.swerveDrive.maximumAngularVelocity

        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= 1.0

        return targetingAngularVelocity
    }

    /**
     *
     */// simple proportional ranging control with Limelight's "ty" value
    // this works best if your Limelight's mount height and target mount height are different.
    // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
    fun limelightRangeProportional(): Double {
        val kP = .1
        var targetingForwardSpeed = LimelightHelpers.getTY("limelight-front") * kP
        targetingForwardSpeed *= SwerveSubsystem.instance.swerveDrive.maximumVelocity
        targetingForwardSpeed *= -1.0
        return targetingForwardSpeed
    }

    companion object {
        // With eager singleton initialization, any static variables/fields used in the
        // constructor must appear before the "INSTANCE" variable so that they are initialized
        // before the constructor is called when the "INSTANCE" variable initializes.
        /**
         * Returns the Singleton instance of this VisionSubsystem. This static method should be used,
         * rather than the constructor, to get the single instance of this class. For example: `VisionSubsystem.getInstance();`
         */
        /**
         * The Singleton instance of this VisionSubsystem. Code should use the [.getInstance]
         * method to get the single instance (rather than trying to construct an instance of this class.)
         */
        val instance: VisionSubsystem = VisionSubsystem()
    }
}
