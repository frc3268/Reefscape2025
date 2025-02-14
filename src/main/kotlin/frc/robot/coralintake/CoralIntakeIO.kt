package frc.robot.coralintake

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs


interface CoralIntakeIO {
    @AutoLog
    open class Inputs {
        var jointAngle = Rotation2d()
        var jointVelocityRPM = 0.0
        var jointAppliedVolts = 0.0
        var jointCurrentAmps = doubleArrayOf()

        var intakeVelocityRPM = 0.0
        var intakeAppliedVolts = 0.0
        var intakeCurrentAmps = doubleArrayOf()
    }

    val pidController: PIDController

    class LoggedInputs : Inputs(), LoggableInputs {
        override fun toLog(table: LogTable) {
            table.put("jointAngle", jointAngle)
            table.put("jointVelocityRPM", jointVelocityRPM)
            table.put("jointAppliedVolts", jointAppliedVolts)
            table.put("jointCurrentAmps", jointCurrentAmps)

            table.put("intakeVelocityRPM", intakeVelocityRPM)
            table.put("intakeAppliedVolts", intakeAppliedVolts)
            table.put("intakeCurrentAmps", intakeCurrentAmps)
        }

        override fun fromLog(table: LogTable) {
            table.get("jointAngle", jointAngle)
            table.get("jointVelocityRPM", jointVelocityRPM)
            table.get("jointAppliedVolts", jointAppliedVolts)
            table.get("jointCurrentAmps", jointCurrentAmps)

            table.get("intakeVelocityRPM", intakeVelocityRPM)
            table.get("intakeAppliedVolts", intakeAppliedVolts)
            table.get("intakeCurrentAmps", intakeCurrentAmps)
        }
    }

    fun updateInputs(inputs: Inputs)

    fun setIntakeVoltage(voltage: Double)

    fun setJointVoltage(volatge: Double)

    fun stopJoint()

    fun stopIntake()

    fun stop()

    // Close all motors and cleanup.
    // Warning: this should **never, ever, EVER, be a command or otherwise called unless it is a unittest! You have been warned!**
    fun close()
}