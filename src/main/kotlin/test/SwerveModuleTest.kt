package test

import frc.lib.swerve.SwerveDriveConstants
import frc.lib.swerve.SwerveModule
import frc.lib.swerve.SwerveModuleIOSim
import frc.robot.Constants
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

// To anyone trying to compile tests:
// Copy and paste all dlls from Reefscape2025\build\jni\release to C:\Users\<Name>\.jdks\corretto-18.0.2\bin
// I dont know why but its the only way to get it to compile

class SwerveModuleTest {

 val DEVIATION: Double = 1e-2 // acceptable deviation range

 @Test
 fun controlTest() {
  assert(true)
 }

 @Test
 fun stopTest() {

  Constants.mode = Constants.States.SIM

  val modules = SwerveDriveConstants.modules.mapIndexed { _, swerveMod ->
   SwerveModule(
    SwerveModuleIOSim(swerveMod.MODULE_NUMBER),
    swerveMod.MODULE_NUMBER
   )
  }
  for (x in modules) {
   x.stop()
   assertEquals(0.0, x.getState().speedMetersPerSecond, DEVIATION); // make sure that the value set to the motor is 0
  }
 }
}