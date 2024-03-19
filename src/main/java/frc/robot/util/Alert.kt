// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found below.
// MIT License
//
// Copyright (c) 2023 FRC 6328
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
package frc.robot.util

import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import java.util.function.Predicate

/** Class for managing persistent alerts to be sent over NetworkTables.  */
class Alert(group: String, text: String, type: AlertType?) {
    private val type: AlertType?
    private var active = false
    private var activeStartTime = 0.0
    private var text: String

    /**
     * Creates a new Alert in the default group - "Alerts". If this is the first to be instantiated,
     * the appropriate entries will be added to NetworkTables.
     *
     * @param text Text to be displayed when the alert is active.
     * @param type Alert level specifying urgency.
     */
    constructor(text: String, type: AlertType?) : this("Alerts", text, type)

    /**
     * Creates a new Alert. If this is the first to be instantiated in its group, the appropriate
     * entries will be added to NetworkTables.
     *
     * @param group Group identifier, also used as NetworkTables title
     * @param text Text to be displayed when the alert is active.
     * @param type Alert level specifying urgency.
     */
    init {
        if (!groups.containsKey(group)) {
            // java: groups.put(group, SendableAlerts())
            groups[group] = SendableAlerts()
            SmartDashboard.putData(group, groups[group])
        }

        this.text = text
        this.type = type
        groups[group]!!.alerts
    }

    /**
     * Sets whether the alert should currently be displayed. When activated, the alert text will also
     * be sent to the console.
     */
    fun set(active: Boolean) {
        if (active && !this.active) {
            activeStartTime = Timer.getFPGATimestamp()
            when (type) {
                AlertType.ERROR -> {
                    DriverStation.reportError(text, false)
                }
                AlertType.ERROR_TRACE -> {
                    DriverStation.reportError(text, true)
                }
                AlertType.WARNING -> {
                    DriverStation.reportWarning(text, false)
                }
                AlertType.WARNING_TRACE -> {
                    DriverStation.reportWarning(text, true)
                }
                AlertType.INFO -> {
                    println(text)
                }
                null -> {
                    TODO()
                }
                else -> {}
            }
        }
        this.active = active
    }

    /** Updates current alert text.  */
    fun setText(text: String) {
        if (active && text != this.text) {
            when (type) {
                AlertType.ERROR -> DriverStation.reportError(text, false)
                AlertType.ERROR_TRACE -> DriverStation.reportError(text, true)
                AlertType.WARNING -> DriverStation.reportWarning(text, false)
                AlertType.WARNING_TRACE -> DriverStation.reportWarning(text, true)
                AlertType.INFO -> println(text)
                null -> TODO()
                else -> {}
            }
        }
        this.text = text
    }

    class SendableAlerts : Sendable {
        val alerts: List<Alert> = ArrayList()

        fun getStrings(type: AlertType): Array<String> {
            val activeFilter = Predicate { x: Alert -> x.type == type && x.active }
            val timeSorter =
                java.util.Comparator { a1: Alert, a2: Alert -> (a2.activeStartTime - a1.activeStartTime).toInt() }
            return alerts.stream()
                .filter(activeFilter)
                .sorted(timeSorter)
                .map<String> { a: Alert -> a.text }
                .toArray { length: Int -> arrayOfNulls(length) }
        }

        override fun initSendable(builder: SendableBuilder) {
            builder.setSmartDashboardType("Alerts")
            builder.addStringArrayProperty("errors", { getStrings(AlertType.ERROR) }, null)
            builder.addStringArrayProperty("errors", { getStrings(AlertType.ERROR_TRACE) }, null)
            builder.addStringArrayProperty("warnings", { getStrings(AlertType.WARNING) }, null)
            builder.addStringArrayProperty("warnings", { getStrings(AlertType.WARNING_TRACE) }, null)
            builder.addStringArrayProperty("infos", { getStrings(AlertType.INFO) }, null)
        }
    }

    /** Represents an alert's level of urgency.  */
    enum class AlertType {
        /**
         * High priority alert - displayed first on the dashboard with a red "X" symbol. Use this type
         * for problems which will seriously affect the robot's functionality and thus require immediate
         * attention.
         */
        ERROR,

        /**
         * High priority alert - displayed first on the dashboard with a red "X" symbol. Use this type
         * for problems which will seriously affect the robot's functionality and thus require immediate
         * attention. Trace printed to driver station console.
         */
        ERROR_TRACE,

        /**
         * Medium priority alert - displayed second on the dashboard with a yellow "!" symbol. Use this
         * type for problems which could affect the robot's functionality but do not necessarily require
         * immediate attention.
         */
        WARNING,

        /**
         * Medium priority alert - displayed second on the dashboard with a yellow "!" symbol. Use this
         * type for problems which could affect the robot's functionality but do not necessarily require
         * immediate attention. Trace printed to driver station console.
         */
        WARNING_TRACE,

        /**
         * Low priority alert - displayed last on the dashboard with a green "i" symbol. Use this type
         * for problems which are unlikely to affect the robot's functionality, or any other alerts
         * which do not fall under "ERROR" or "WARNING".
         */
        INFO
    }

    companion object {
        private val groups: java.util.HashMap<String, SendableAlerts> = HashMap<String, SendableAlerts>()
    }
}
