package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase

/**
 *
 */
object TelemetrySubsystem : SubsystemBase() {
    // With eager singleton initialization, any static variables/fields used in the
    // constructor must appear before the "INSTANCE" variable so that they are initialized
    // before the constructor is called when the "INSTANCE" variable initializes.
    /**
     * Returns the Singleton instance of this TelemetrySubsystem. This static method should be used,
     * rather than the constructor, to get the single instance of this class. For example: `TelemetrySubsystem.getInstance();`
     */
    /**
     * The Singleton instance of this TelemetrySubsystem. Code should use the [.getInstance]
     * method to get the single instance (rather than trying to construct an instance of this class.)
     */
    val instance: TelemetrySubsystem = TelemetrySubsystem
}
