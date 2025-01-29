package frc.robot.coralintake

import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import edu.wpi.first.math.geometry.Rotation2d


interface CoralIntakeIO {
    @AutoLog
    open class Inputs {
        var appliedVolts: Double = 0.0
        var velocityMetersPerSec: Double = 0.0
        var currentAmps: DoubleArray = doubleArrayOf()
    }

    class LoggedInputs : Inputs(), LoggableInputs {
        override fun toLog(table: LogTable) {
            table.put("appliedVolts", appliedVolts)
            table.put("velocityMetersPerSec", velocityMetersPerSec)
            table.put("currentAmps", currentAmps)
        }

        override fun fromLog(table: LogTable) {
            table.get("appliedVolts", appliedVolts)
            table.get("velocityMetersPerSec", velocityMetersPerSec)
            table.get("currentAmps", currentAmps)
        }
    }

    fun updateInputs(inputs: Inputs)

    fun setWheelVoltage(voltage: Double)

    fun setAngle(angle: Rotation2d)

    // Enum
    // fun setAngle();

    fun stop()
}