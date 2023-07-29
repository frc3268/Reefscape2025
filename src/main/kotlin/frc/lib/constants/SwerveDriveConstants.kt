package frc.lib.constants

import com.revrobotics.CANSparkMax.IdleMode
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import frc.lib.constants.SwerveDriveConstants.DrivetrainConsts.WHEEL_DIAMETER_METERS

class SwerveDriveConstants {
    data class ModuleConstants(
        val MODULE_NUMBER: Int,
        val ANGLE_OFFSET: Rotation2d,
        val DRIVE_MOTOR_ID: Int,
        val ANGLE_MOTOR_ID: Int,
        val ENCODER_ID: Int

    )

    object ModuleGains {
        /* Drive Motor Characterization Values */
        const val DRIVE_KS: Double = 0.0
        const val DRIVE_KV: Double = 0.0
        const val DRIVE_KA: Double = 0.0
    }

    object DriveMotorConsts {
        const val GEAR_RATIO: Double = 8.14 / 1.0
        const val CONTINUOUS_CURRENT_LIMIT: Int = 80
        const val POSITION_CONVERSION_FACTOR_METERS_PER_ROTATION: Double =
            (WHEEL_DIAMETER_METERS * Math.PI) / GEAR_RATIO
        const val VELOCITY_CONVERSION_FACTOR_METERS_PER_SECOND = POSITION_CONVERSION_FACTOR_METERS_PER_ROTATION / 60.0
        val NEUTRAL_MODE: IdleMode = IdleMode.kBrake
        const val INVERT: Boolean = false


        /* Drive Motor PID Values */
        const val KP: Double = 0.1
        const val KI: Double = 0.0
        const val KD: Double = 0.0
        const val KFF: Double = 0.0
    }

    object AngleMotorConsts {
        private const val GEAR_RATIO: Double = 12.8 / 1.0
        const val CONTINUOUS_CURRENT_LIMIT: Int = 20
        const val POSITION_CONVERSION_FACTOR_DEGREES_PER_ROTATION = 360 / GEAR_RATIO
        val NEUTRAL_MODE: IdleMode = IdleMode.kBrake
        const val INVERT: Boolean = false

        //will have to set these for real for real
        const val KP: Double = 0.01
        const val KI: Double = 0.0
        const val KD: Double = 0.0
        const val KFF: Double = 0.0

    }

    object EncoderConsts {
        const val INVERT:Boolean = false
        const val POSITION_CONVERSION_FACTOR_DEGREES_PER_ROTATION:Double = 360.0
    }

    object DrivetrainConsts {        /* Drivetrain Constants */
        val TRACK_WIDTH_METERS = 0.7112
        val WHEEL_BASE_METERS = 0.7112

        const val WHEEL_DIAMETER_METERS = 0.1016
        val WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI

        const val OPEN_LOOP_RAMP_RATE_SECONDS: Double = 0.25
        const val CLOSED_LOOP_RAMP_RATE_SECONDS: Double = 0.0

        /* Swerve Voltage Compensation */
        const val VOLTAGE_COMPENSATION: Double = 12.0

        /* Swerve Profiling Values */
        const val MAX_SPEED_METERS_PER_SECOND = 4.5
        const val MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND = 11.5

        var turnController:PIDController = PIDController(
            0.0,
            0.0,
            0.0
        )

    }
    object modules{
        val list:List<ModuleConstants> = listOf(
            ModuleConstants(1, Rotation2d.fromDegrees(-181.8), 1, 2, 0),
            ModuleConstants(2, Rotation2d.fromDegrees(-8.7), 3, 4, 1),
            ModuleConstants(3, Rotation2d.fromDegrees(-253.7), 5, 6, 2),
            ModuleConstants(4, Rotation2d.fromDegrees(-114.7), 7, 8, 3)
        )
    }

}