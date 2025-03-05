package frc.robot.algaeintake

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface AlgaeIntakeIO {
    @AutoLog
    open class Inputs {
        var jointAngle:Rotation2d = Rotation2d(0.0)
        var jointVelocityMetersPerSec: Double = 0.0

        var mainAppliedVolts: Double = 0.0
        var mainVelocityMetersPerSec: Double = 0.0
        var mainCurrentAmps: DoubleArray = doubleArrayOf()

        var revAppliedVolts: Double = 0.0
        var revVelocityMetersPerSec: Double = 0.0
        var revCurrentAmps: DoubleArray = doubleArrayOf()
    }

    val pidController: PIDController

    class LoggedInputs : Inputs(), LoggableInputs {
        override fun toLog(table: LogTable) {
            table.put("jointAngle", jointAngle)
            table.put("jointVelocityMetersPerSec", jointVelocityMetersPerSec)

            table.put("mainAppliedVolts", mainAppliedVolts)
            table.put("mainVelocityMetersPerSec", mainVelocityMetersPerSec)

            table.put("revAppliedVolts", revAppliedVolts)
            table.put("revVelocityMetersPerSec", revVelocityMetersPerSec)
        }

        override fun fromLog(table: LogTable) {
            table.get("jointAngle", jointAngle)
            table.get("jointVelocityMetersPerSec", jointVelocityMetersPerSec)

            table.get("mainAppliedVolts", mainAppliedVolts)
            table.get("mainVelocityMetersPerSec", mainVelocityMetersPerSec)

            table.get("revAppliedVolts", revAppliedVolts)
            table.get("revVelocityMetersPerSec", revVelocityMetersPerSec)
        }
    }

    fun updateInputs(inputs: Inputs)
    fun setJointVoltage(voltage: Double)
    fun setMainAndRevVoltage(voltage: Double)
    fun setRevVolate(voltage: Double)
    fun setMainVoltage(voltage: Double)
    fun stop()
    fun stopMain()
    fun stopRev()
    fun stopJoint()
}