package frc.robot.climber

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.abs
import frc.robot.Constants


class ClimberSubsystem(val io: ClimberIO) : SubsystemBase() {
    val inputs = ClimberIO.LoggedInputs()
    val kg = 0.0

    // Get some trouble - shooting information to the ShuffleBoard
    val troubleshootingtab = Shuffleboard.getTab(Constants.TROUBLESHOOTING_TAB)
    val rightMotorPositionDegrees = troubleshootingtab.add("Right Motor Position", 0.0).withPosition(4,0).entry
    val rightMotorAppliedVolts = troubleshootingtab.add("Right Motor Applied Volts", 0.0).withPosition(4, 1).entry
    val leftMotorPositionDegrees = troubleshootingtab.add("Left Motor Position", 0.0).withPosition(4, 2).entry
    val leftMotorAppliedVolts = troubleshootingtab.add("Left Motor Applied Volts", 0.0).withPosition(4, 3).entry
    val climberPositionDegrees = troubleshootingtab.add("Climber Position", 0.0).withPosition(4, 4).entry
    init {

    }

    override fun periodic() {
        io.updateInputs(inputs)

        rightMotorPositionDegrees.setDouble(inputs.rightMotorPositionDegrees)
        rightMotorAppliedVolts.setDouble(inputs.rightMotorAppliedVolts)
        leftMotorPositionDegrees.setDouble(inputs.leftMotorPositionDegrees)
        leftMotorAppliedVolts.setDouble(inputs.leftMotorAppliedVolts)
        climberPositionDegrees.setDouble(inputs.climberPositionDegrees)
    }

    fun setToPosition(setPointDegrees: Double): Command =
        run {
            io.setBothVolts(io.pidController.calculate(inputs.climberPositionDegrees, setPointDegrees) * 12.0)
        }
            .until { abs(inputs.climberPositionDegrees - setPointDegrees) < 10.0 }

    fun stop(): Command = runOnce { io.stop() }

    fun zero(): Command = runOnce { io.reset() }
}