package frc.lib.swerve

import com.revrobotics.RelativeEncoder
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.AnalogEncoder
import frc.lib.swerve.SwerveModuleIO.ModuleIOInputs
import frc.lib.rotation2dFromDeg
import kotlin.math.IEEErem


class SwerveModuleIOSparkMax(val moduleConstants: SwerveDriveConstants.ModuleConstants) : SwerveModuleIO {

    private val driveMotor = SparkMax(moduleConstants.DRIVE_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)
    private val angleMotor = SparkMax(moduleConstants.ANGLE_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)

    private val driveConfig = SparkMaxConfig()
    private val angleConfig = SparkMaxConfig()

    override val turnPIDController: PIDController = moduleConstants.PID_CONTROLLER

    private val driveEncoder: RelativeEncoder = driveMotor.encoder
    private val angleEncoder: RelativeEncoder = angleMotor.encoder

    private val absoluteEncoder = AnalogEncoder(moduleConstants.ENCODER_ID)

    // Gear ratios for SDS MK4i L2, adjust as necessary
    private val DRIVE_GEAR_RATIO: Double = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)
    private val TURN_GEAR_RATIO: Double = 150.0 / 7.0

    init {
<<<<<<< Updated upstream
                absoluteEncoder.distancePerRotation =
                    SwerveDriveConstants.Encoder.POSITION_CONVERSION_FACTOR_DEGREES_PER_ROTATION
                absoluteEncoder.positionOffset = moduleConstants.ANGLE_OFFSET.degrees
                driveEncoder.positionConversionFactor =
                    SwerveDriveConstants.DriveMotor.POSITION_CONVERSION_FACTOR_METERS_PER_ROTATION
                driveEncoder.velocityConversionFactor =
                    SwerveDriveConstants.DriveMotor.VELOCITY_CONVERSION_FACTOR_METERS_PER_SECOND
                angleEncoder.positionConversionFactor =
                    SwerveDriveConstants.AngleMotor.POSITION_CONVERSION_FACTOR_DEGREES_PER_ROTATION
=======
                driveConfig.encoder.positionConversionFactor(
                    SwerveDriveConstants.DriveMotor.POSITION_CONVERSION_FACTOR_METERS_PER_ROTATION)
                driveConfig.encoder.velocityConversionFactor(
                    SwerveDriveConstants.DriveMotor.VELOCITY_CONVERSION_FACTOR_METERS_PER_SECOND)
                angleConfig.encoder.positionConversionFactor(
                    SwerveDriveConstants.AngleMotor.POSITION_CONVERSION_FACTOR_DEGREES_PER_ROTATION)
>>>>>>> Stashed changes

                driveConfig.inverted(moduleConstants.DRIVE_MOTOR_REVERSED)
                angleConfig.inverted(moduleConstants.ANGLE_MOTOR_REVERSED)

                driveConfig.openLoopRampRate(SwerveDriveConstants.DrivetrainConsts.OPEN_LOOP_RAMP_RATE_SECONDS)
                angleConfig.openLoopRampRate(SwerveDriveConstants.DrivetrainConsts.OPEN_LOOP_RAMP_RATE_SECONDS)

                driveConfig.signals.primaryEncoderPositionPeriodMs(15)
                //todo: fix? below
                angleConfig.signals.primaryEncoderPositionPeriodMs(15)

                driveMotor.configure(driveConfig,  SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
                angleMotor.configure(angleConfig,  SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    override fun updateInputs(inputs: ModuleIOInputs) {
        inputs.drivePositionMeters =
            -driveEncoder.position
        inputs.driveVelocityMetersPerSec =
            -driveEncoder.velocity
        inputs.driveAppliedVolts = driveMotor.appliedOutput * driveMotor.busVoltage
        inputs.driveCurrentAmps = doubleArrayOf(driveMotor.outputCurrent)

        inputs.turnAbsolutePosition =
            ((absoluteEncoder.absolutePosition * 360.0) + moduleConstants.ANGLE_OFFSET.degrees).rotation2dFromDeg()
        inputs.turnPosition =
            ((-inputs.turnAbsolutePosition.degrees).IEEErem(360.0).rotation2dFromDeg())
        inputs.turnVelocityRadPerSec = (
                Units.rotationsPerMinuteToRadiansPerSecond(angleEncoder.velocity)
                        / TURN_GEAR_RATIO)
        inputs.turnAppliedVolts = angleMotor.appliedOutput * angleMotor.busVoltage
        inputs.turnCurrentAmps = doubleArrayOf(angleMotor.outputCurrent)
    }

    override fun setDriveVoltage(volts: Double) {
        driveMotor.setVoltage(volts)
    }

    override fun setTurnVoltage(volts: Double) {
        angleMotor.setVoltage(volts)
    }

    override fun setDriveBrakeMode(enable: Boolean) {
        driveConfig.idleMode(if (enable) SparkBaseConfig.IdleMode.kBrake else SparkBaseConfig.IdleMode.kCoast)
        driveMotor.configure(driveConfig,  SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    override fun setTurnBrakeMode(enable: Boolean) {
        angleConfig.idleMode(if (enable) SparkBaseConfig.IdleMode.kBrake else SparkBaseConfig.IdleMode.kCoast)
        angleMotor.configure(angleConfig,  SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    override fun reset() {
        driveEncoder.position = 0.0
        angleEncoder.position = ((absoluteEncoder.absolutePosition * 360.0) + moduleConstants.ANGLE_OFFSET.degrees)
    }
}