package frc.robot.subsystems

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Subsystem
import java.util.function.BooleanSupplier

/**
 *
 */
class TelemetrySubsystem
private constructor() : Subsystem {
    var hasNote: BooleanSupplier? = null
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
    companion object { val instance: TelemetrySubsystem = TelemetrySubsystem() }

    init {
        Shuffleboard.getTab("Main")
            .addBoolean("Has Note", hasNote).withSize(58, 2).withPosition(1, 0).withProperties(mapOf(Pair("Colors/Color when true", "#FF6A23FF"), Pair("Colors/Color when false", "#000000FF")))
    }

    fun setNoteSupplier(supplier: BooleanSupplier) {
        hasNote = supplier
    }
}
