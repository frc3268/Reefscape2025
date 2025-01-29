package frc.robot.elevator

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.PIDController
import frc.lib.swerve.ElevatorIO

class ElevatorIOSparkMax(override val pidController: PIDController) :ElevatorIO {
    val leftMotor = SparkMax(0, SparkLowLevel.MotorType.kBrushless)
    val rightMotor = SparkMax(0, SparkLowLevel.MotorType.kBrushless)

    val leftEncoder =  leftMotor.encoder
    val rightEncoder = rightMotor.encoder

    val leftConfig = SparkMaxConfig()
    val rightConfig = SparkMaxConfig()

    init{
        leftConfig.encoder.positionConversionFactor(0.0)
        rightConfig.encoder.positionConversionFactor(0.0)

    }
    override fun updateInputs(inputs: ElevatorIO.ElevatorIOInputs) {
        //this forumla may need to me changed to reflect the reality
        inputs.elevatorPositionMeters = (leftEncoder.position + rightEncoder.position) / 2
        inputs.rightMotorPositionMeters = rightEncoder.position
        inputs.leftMotorPositionMeters = leftEncoder.position
        inputs.rightMotorCurrentAmps = doubleArrayOf(rightMotor.outputCurrent)
        inputs.leftMotorCurrentAmps = doubleArrayOf(leftMotor.outputCurrent)
        inputs.rightMotorAppliedVolts = rightMotor.busVoltage * rightMotor.appliedOutput
        inputs.leftMotorAppliedVolts = leftMotor.busVoltage * leftMotor.appliedOutput
        inputs.rightMotorVelocityMetersPerSec = rightEncoder.velocity
        inputs.leftMotorVelocityMetersPerSec = leftEncoder.velocity
    }

    override fun setBothVolts(volts: Double) {
        rightMotor.setVoltage(volts)
        leftMotor.setVoltage(volts)
    }

    override fun reset() {
        rightEncoder.position = 0.0
        leftEncoder.position = 0.0
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