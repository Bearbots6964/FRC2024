package frc.robot.util

import edu.wpi.first.wpilibj2.command.Command

class UtilityCommand(function: () -> Unit) : Command() {
    private val a = function
    override fun execute() {
        a
    }
}