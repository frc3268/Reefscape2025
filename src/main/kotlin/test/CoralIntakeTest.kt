package test

import frc.robot.coralintake.CoralIntakeIOSparkMaxSim
import frc.robot.coralintake.CoralIntakeSubsystem
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test


// To anyone trying to compile tests:
// Copy and paste all dlls from Reefscape2025\build\jni\release to C:\Users\<Name>\.jdks\corretto-18.0.2\bin
// I don't know why, but it's the only way to get it to compile

class CoralIntakeTest {

 val ALLOWED_DEVIATION: Double = 1e-2 // acceptable deviation range
 private var coralIntake: CoralIntakeSubsystem? = null


 @BeforeEach
 fun setup() {
  this.coralIntake = CoralIntakeSubsystem(CoralIntakeIOSparkMaxSim())
 }

 @AfterEach
 fun shutdown() {
  coralIntake!!.io.close() // destroy our intake object
 }

 // This test **should** always pass
 // If it doesn't and you tried the above suggestion, give up.
 @Test
 fun controlTest() {
  assert(true)
 }

 // I dont know why but running multiple tests at once causes a crash
 @Test
 fun intakeCloseTest() {
  coralIntake!!.raiseToIntake()
  // Write stuff
  TODO("Finish this")
 }
}