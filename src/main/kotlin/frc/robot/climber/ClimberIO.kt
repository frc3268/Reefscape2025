package frc.robot.climber

import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.algaeintake.AlgaeIntakeIO

interface ClimberIO {
    @AutoLog
    open class Inputs {

    }

    val pidController: PIDController


    class LoggedInputs : AlgaeIntakeIO.Inputs(), LoggableInputs {
        override fun toLog(table: LogTable) {

        }

        override fun fromLog(table: LogTable) {

        }
    }
    fun stop()
}

