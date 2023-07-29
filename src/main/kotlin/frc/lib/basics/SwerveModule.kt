package frc.lib.basics

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.RelativeEncoder
import com.revrobotics.SparkMaxPIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.AnalogEncoder
import frc.lib.constants.SwerveDriveConstants

/*
Props: drive motor, drive encoder, angle motor, angle encoder, absolute encoder
Get: Cancoder measurement, Module state(velocity) and position
Set: Module state
 */
class SwerveModule(val moduleConstants: SwerveDriveConstants.ModuleConstants) {
    private val driveMotor:CANSparkMax = CANSparkMax(moduleConstants.DRIVE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val angleMotor:CANSparkMax = CANSparkMax(moduleConstants.ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)

    private val driveEncoder:RelativeEncoder = driveMotor.encoder
    private val angleEncoder:RelativeEncoder = angleMotor.encoder

    private val absoluteEncoder:AnalogEncoder = AnalogEncoder(moduleConstants.ENCODER_ID)

    init {
        //TODO: Config
        SwerveDriveConstants.DrivetrainConsts.turnController.enableContinuousInput(
            -90.0,90.0
        )
    }

    fun resetToAbsolute(){
        driveEncoder.position = 0.0
        angleEncoder.position = getAbsoluteEncoderMeasurement().degrees
    }
    fun getAbsoluteEncoderMeasurement() : Rotation2d = Rotation2d.fromDegrees(absoluteEncoder.absolutePosition)
    fun getModuleState() : SwerveModuleState = SwerveModuleState(driveEncoder.velocity, Rotation2d.fromDegrees(angleEncoder.position))
    fun getModulePosition() : SwerveModulePosition = SwerveModulePosition(driveEncoder.position, Rotation2d.fromDegrees(angleEncoder.position))

    fun setDesiredState(desiredState:SwerveModuleState){
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001){
            stop()
            return
        }
        val optimizedState = SwerveModuleState.optimize(desiredState, getModuleState().angle)
        //TODO: 5.0 should be a const
        driveMotor.set(desiredState.speedMetersPerSecond / 5.0)
        angleMotor.set(SwerveDriveConstants.DrivetrainConsts.turnController.calculate(getModuleState().angle.degrees, optimizedState.angle.degrees))
    }
    fun stop(){
        driveMotor.set(0.0)
        angleMotor.set(0.0)
    }

}