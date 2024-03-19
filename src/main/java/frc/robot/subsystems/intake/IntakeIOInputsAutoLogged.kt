//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//
package frc.robot.subsystems.intake

import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

/**
 *
 */
class IntakeIOInputsAutoLogged : IntakeIOInputs(), LoggableInputs, Cloneable {

    /**
     *
     */
    override fun toLog(table: LogTable) {
        table.put("IntakePositionDegrees", this.intakePositionDegrees)
        table.put("IntakeVelocityRpm", this.intakeVelocityRpm)
        table.put("IntakeAppliedVolts", this.intakeAppliedVolts)
        table.put("IntakeCurrentAmps", this.intakeCurrentAmps)
        table.put("CerealizerPositionDegrees", this.cerealizerPositionDegrees)
        table.put("CerealizerVelocityRpm", this.cerealizerVelocityRpm)
        table.put("CerealizerAppliedVolts", this.cerealizerAppliedVolts)
        table.put("CerealizerCurrentAmps", this.cerealizerCurrentAmps)
    }

    /**
     *
     */
    override fun fromLog(table: LogTable) {
        this.intakePositionDegrees = table["IntakePositionDegrees", intakePositionDegrees]
        this.intakeVelocityRpm = table["IntakeVelocityRpm", intakeVelocityRpm]
        this.intakeAppliedVolts = table["IntakeAppliedVolts", intakeAppliedVolts]
        this.intakeCurrentAmps = table["IntakeCurrentAmps", intakeCurrentAmps]
        this.cerealizerPositionDegrees = table["CerealizerPositionDegrees", cerealizerPositionDegrees]
        this.cerealizerVelocityRpm = table["CerealizerVelocityRpm", cerealizerVelocityRpm]
        this.cerealizerAppliedVolts = table["CerealizerAppliedVolts", cerealizerAppliedVolts]
        this.cerealizerCurrentAmps = table["CerealizerCurrentAmps", cerealizerCurrentAmps]
    }

    /**
     *
     */
    public override fun clone(): IntakeIOInputsAutoLogged {
        val copy = IntakeIOInputsAutoLogged()
        copy.intakePositionDegrees = this.intakePositionDegrees
        copy.intakeVelocityRpm = this.intakeVelocityRpm
        copy.intakeAppliedVolts = this.intakeAppliedVolts
        copy.intakeCurrentAmps = intakeCurrentAmps.clone()
        copy.cerealizerPositionDegrees = this.cerealizerPositionDegrees
        copy.cerealizerVelocityRpm = this.cerealizerVelocityRpm
        copy.cerealizerAppliedVolts = this.cerealizerAppliedVolts
        copy.cerealizerCurrentAmps = cerealizerCurrentAmps.clone()
        return copy
    }
}