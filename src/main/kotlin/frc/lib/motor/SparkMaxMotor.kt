package frc.lib.motor

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.PIDController

// We should use this class more fr fr
// Also not use as much PiD Controllers
class SparkMaxMotor(
    override val ID: Int,
    override var inverse: Boolean = false,
    override val positionPidController: PIDController = PIDController(0.0,0.0,0.0),
    override val velocityPidController: PIDController = PIDController(0.0,0.0,0.0),
) : Motor {

    val motor = SparkMax(ID, SparkLowLevel.MotorType.kBrushless)
    var motorConfig = SparkMaxConfig()

    init{
        motorConfig.inverted(inverse)
        motor.configure(motorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters)
    }

    override fun setVoltage(voltage: Double) {
        motor.setVoltage(voltage)
    }

    override fun setPosition(position: Double) {
        TODO("Not yet implemented")

        // yeah idk if this works
        // positionPidController.goal = TrapezoidProfile.State(position, 0.0)
        // controller.set(positionPidController.calculate(getPositonMeasurement()))
    }

    override fun setVelocity(velocity: Double) {
        TODO("Not yet implemented")
    }


    override fun getVelocityRPMMeasurement(): Double {
        return motor.getEncoder().velocity
    }

    override fun getAppliedVoltage(): Double {
        return motor.busVoltage
    }

    override fun getPositionDegreeMeasurement(): Double {
        TODO("Not yet implemented")
    }

    override fun getCurrentAmps(): DoubleArray {
        TODO("Not yet implemented")
    }

    override fun stop() {
        motor.stopMotor()
    }

    override fun close() {
        motor.close()
    }

    override fun reset() {
        motor.encoder.position = 0.0
    }
}
