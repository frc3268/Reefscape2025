package frc.robot.elevator

import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.AngleUnit
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import frc.lib.swerve.ElevatorIO

class ElevatorIOKraken(override val pidController: PIDController) : ElevatorIO{
    val leftMotor = TalonFX(0, "rio")
    val rightMotor = TalonFX(0, "rio")


    init{
        //todo!
        //leftEncoder.positionConversionFactor = 0.0
        //rightEncoder.positionConversionFactor = 0.0

    }
    override fun updateInputs(inputs: ElevatorIO.ElevatorIOInputs) {
        //this formula may need to me changed to reflect the reality
        inputs.elevatorPositionMeters = (leftMotor.position.valueAsDouble + rightMotor.position.valueAsDouble) / 2
        inputs.rightMotorPositionMeters = rightMotor.position.valueAsDouble
        inputs.leftMotorPositionMeters = leftMotor.position.valueAsDouble
        inputs.rightMotorCurrentAmps = doubleArrayOf(rightMotor.statorCurrent.valueAsDouble)
        inputs.leftMotorCurrentAmps = doubleArrayOf(leftMotor.statorCurrent.valueAsDouble)
        inputs.rightMotorAppliedVolts = rightMotor.motorVoltage.valueAsDouble
        inputs.leftMotorAppliedVolts = leftMotor.motorVoltage.valueAsDouble
        inputs.rightMotorVelocityMetersPerSec = rightMotor.velocity.valueAsDouble
        inputs.leftMotorVelocityMetersPerSec = leftMotor.velocity.valueAsDouble
    }

    override fun setBothVolts(volts: Double) {
        rightMotor.setVoltage(volts)
        leftMotor.setVoltage(volts)
    }

    override fun reset() {
        //not totally sure if this works as intended
        //as intended means that it just changes the value reported by encoder
        rightMotor.setPosition(Angle.ofRelativeUnits(0.0, Units.Degree))
        leftMotor.setPosition(Angle.ofRelativeUnits(0.0, Units.Degree))
    }

    override fun stop() {
        rightMotor.stopMotor()
        leftMotor.stopMotor()
    }

    override fun setLeftVolts(volts: Double) {
        leftMotor.setVoltage(volts)
    }

    override fun setRightVolts(volts: Double) {
        rightMotor.setVoltage(volts)
    }
}