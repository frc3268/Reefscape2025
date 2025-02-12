package frc.lib.test

import edu.wpi.first.hal.HAL
import frc.lib.swerve.SwerveDriveConstants
import frc.lib.swerve.SwerveModule
import frc.lib.swerve.SwerveModuleIOSim
import frc.robot.Constants
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher


class SwerveModuleTest {

 private var modules: List<SwerveModule>? = null
 val DEVIATION: Double = 1e-2 // acceptable deviation range

 @BeforeEach
 fun setup() {
  Constants.mode = Constants.States.SIM

/*  assert(
   HAL.initialize(500, 0) // initialize the HAL, crash if failed
  )*/

  Logger.addDataReceiver(NT4Publisher())
  Logger.recordMetadata("ProjectName", "SwerveModuleUnitTest") // Set a metadata value
  Logger.start() // Start logging! No more data receivers, replay sources, or metadata values may be added.

  SwerveDriveConstants.modules.mapIndexed { _, swerveMod -> SwerveModule(SwerveModuleIOSim(swerveMod.MODULE_NUMBER),swerveMod.MODULE_NUMBER) }
 }

 @Test
 fun controlTest() {
  modules!![0]
 }

 @Test
  fun setTurnVoltageTest() {
   modules?.get(0)
  }
}