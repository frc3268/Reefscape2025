package frc.robot.elevator

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.lib.swerve.ElevatorIO
import frc.lib.swerve.ElevatorIOInputsAutoLogged
import kotlin.math.abs

class ElevatorSubsystem(val io:ElevatorIO) : SubsystemBase() {
    val inputs = ElevatorIOInputsAutoLogged()
    init {

    }
    override fun periodic() {
        io.updateInputs(inputs)
    }

    fun setToPosition(setPointMeters:Double): Command = run{
        io.setBothVolts(io.pidController.calculate(inputs.elevatorPositionMeters, setPointMeters) * 12.0)
    }.until({abs(inputs.elevatorPositionMeters - setPointMeters) < 0.01}).andThen(stop())

    fun stop(): Command = runOnce{io.stop()}

    fun zero():Command = runOnce{io.reset()}
}