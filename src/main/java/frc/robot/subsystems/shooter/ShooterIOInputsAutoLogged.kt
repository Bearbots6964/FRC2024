//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//
package frc.robot.subsystems.shooter

import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

/**
 *
 */
class ShooterIOInputsAutoLogged : ShooterIOInputs(), LoggableInputs, Cloneable {
    /**
     *
     */
    override fun toLog(table: LogTable) {
        table.put("LowerPositionDegrees", this.lowerPositionDegrees)
        table.put("LowerVelocityRpm", this.lowerVelocityRpm)
        table.put("LowerAppliedVolts", this.lowerAppliedVolts)
        table.put("LowerRpmSetpoint", this.lowerRpmSetpoint)
        table.put("LowerSurfaceSpeed", this.lowerSurfaceSpeed)
        table.put("LowerOutput", this.lowerOutput)
        table.put("LowerCurrentAmps", this.lowerCurrentAmps)
        table.put("UpperPositionDegrees", this.upperPositionDegrees)
        table.put("UpperVelocityRpm", this.upperVelocityRpm)
        table.put("UpperAppliedVolts", this.upperAppliedVolts)
        table.put("UpperRpmSetpoint", this.upperRpmSetpoint)
        table.put("UpperSurfaceSpeed", this.upperSurfaceSpeed)
        table.put("UpperOutput", this.upperOutput)
        table.put("UpperCurrentAmps", this.upperCurrentAmps)
    }

    /**
     *
     */
    override fun fromLog(table: LogTable) {
        this.lowerPositionDegrees = table["LowerPositionDegrees", lowerPositionDegrees]
        this.lowerVelocityRpm = table["LowerVelocityRpm", lowerVelocityRpm]
        this.lowerAppliedVolts = table["LowerAppliedVolts", lowerAppliedVolts]
        this.lowerRpmSetpoint = table["LowerRpmSetpoint", lowerRpmSetpoint]
        this.lowerSurfaceSpeed = table["LowerSurfaceSpeed", lowerSurfaceSpeed]
        this.lowerOutput = table["LowerOutput", lowerOutput]
        this.lowerCurrentAmps = table["LowerCurrentAmps", lowerCurrentAmps]
        this.upperPositionDegrees = table["UpperPositionDegrees", upperPositionDegrees]
        this.upperVelocityRpm = table["UpperVelocityRpm", upperVelocityRpm]
        this.upperAppliedVolts = table["UpperAppliedVolts", upperAppliedVolts]
        this.upperRpmSetpoint = table["UpperRpmSetpoint", upperRpmSetpoint]
        this.upperSurfaceSpeed = table["UpperSurfaceSpeed", upperSurfaceSpeed]
        this.upperOutput = table["UpperOutput", upperOutput]
        this.upperCurrentAmps = table["UpperCurrentAmps", upperCurrentAmps]
    }

    /**
     *
     */
    public override fun clone(): ShooterIOInputsAutoLogged {
        val copy = ShooterIOInputsAutoLogged()
        copy.lowerPositionDegrees = this.lowerPositionDegrees
        copy.lowerVelocityRpm = this.lowerVelocityRpm
        copy.lowerAppliedVolts = this.lowerAppliedVolts
        copy.lowerRpmSetpoint = this.lowerRpmSetpoint
        copy.lowerSurfaceSpeed = this.lowerSurfaceSpeed
        copy.lowerOutput = this.lowerOutput
        copy.lowerCurrentAmps = lowerCurrentAmps.clone()
        copy.upperPositionDegrees = this.upperPositionDegrees
        copy.upperVelocityRpm = this.upperVelocityRpm
        copy.upperAppliedVolts = this.upperAppliedVolts
        copy.upperRpmSetpoint = this.upperRpmSetpoint
        copy.upperSurfaceSpeed = this.upperSurfaceSpeed
        copy.upperOutput = this.upperOutput
        copy.upperCurrentAmps = upperCurrentAmps.clone()
        return copy
    }
}