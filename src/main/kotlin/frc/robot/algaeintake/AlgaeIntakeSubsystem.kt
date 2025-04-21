package frc.robot.algaeintake

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import frc.lib.rotation2dFromDeg
import kotlin.math.abs


//algae has 3 motors - one for arm, two for wheels. main and rev are wheel motors, rev always goes reverse direction
class AlgaeIntakeSubsystem(val io: AlgaeIntakeIO) : SubsystemBase() {
    val inputs = AlgaeIntakeIO.LoggedInputs()

    val troubleshootingTab = Shuffleboard.getTab("Algae")
    val jointAngleEntry = troubleshootingTab.add("Joint Angle", 0.0).withPosition(2, 0).entry
    val jointVelocityMetersPerSecEntry = troubleshootingTab.add("Joint Velocity MPS", 0.0).withPosition(2, 1).entry
    val mainVelocityMetersPerSecEntry = troubleshootingTab.add("Main Velocity MPS", 0.0).withPosition(2, 2).entry
    val revVelocityMetersPerSecEntry = troubleshootingTab.add("Reverse Velocity MPS", 0.0).withPosition(2, 3).entry

    val cntrl = ProfiledPIDController(io.pidController.p, io.pidController.i, io.pidController.d, TrapezoidProfile.Constraints(100.0, 120.0))

    var setpoint = 0.0
    var stopped = false
    var initSpin = false
    var startedSpinning = false
    var holdIN = false

    init {

        io.resetJointEncoder()
        troubleshootingTab.add("up", raise()).withWidget(BuiltInWidgets.kCommand)
        troubleshootingTab.add("down", lower()).withWidget(BuiltInWidgets.kCommand)
        troubleshootingTab.add("half", half()).withWidget(BuiltInWidgets.kCommand)
        troubleshootingTab.add("stop", stopJoint()).withWidget(BuiltInWidgets.kCommand)
        troubleshootingTab.add("intake", intake()).withWidget(BuiltInWidgets.kCommand)
        troubleshootingTab.add("outtake", runOnce{holdIN = false}.andThen({io.setMainAndRevVoltage(0.6 * 12.0)})).withWidget(BuiltInWidgets.kCommand)
        troubleshootingTab.add("stopwheels", stopWheels()).withWidget(BuiltInWidgets.kCommand)
    }
    
    override fun periodic() {
        io.updateInputs(inputs)

        // Debug stuff I guess
        jointAngleEntry.setDouble(inputs.jointAngle.degrees)
        jointVelocityMetersPerSecEntry.setDouble(inputs.jointVelocityMetersPerSec.toDouble())
        mainVelocityMetersPerSecEntry.setDouble(inputs.mainVelocityMetersPerSec.toDouble())
        revVelocityMetersPerSecEntry.setDouble(inputs.revVelocityMetersPerSec.toDouble())

        if(!stopped){
            io.setJointVoltage(io.pidController.calculate(inputs.jointAngle.degrees, setpoint))
        }

        if(holdIN){
            io.setMainAndRevVoltage(-0.05 * 12.0)
        }

    }
    fun intake(): Command =
        runOnce { holdIN = false }.andThen(
        run{io.setMainAndRevVoltage(-0.3 * 12.0)}
        .withTimeout(1.0)
        .andThen(run{})
        .until { abs(inputs.mainVelocityMetersPerSec) > 3000 }
        .andThen(runOnce { initSpin = true })
        .andThen(run{})
        .until { (abs(inputs.mainVelocityMetersPerSec) + abs(inputs.revVelocityMetersPerSec)) / 2 < 2900  && initSpin == true}
        .andThen(runOnce { initSpin = false
                            startedSpinning = true})
        .andThen(run{io.setMainAndRevVoltage(-0.4 * 12.0)})
        .andThen(run{})
        .until { (abs(inputs.mainVelocityMetersPerSec)  < 3900 ||  abs(inputs.revVelocityMetersPerSec) < 3900) && startedSpinning }
        .andThen(stopWheels())
        .andThen(runOnce{startedSpinning = false})
        .andThen(runOnce { holdIN = true }))

//    fun intake(): Command = run{io.setMainAndRevVoltage(-0.4 * 12.0)}
//        .withTimeout(1.0)
//        .andThen(run{})
//        .until { abs(inputs.mainVelocityMetersPerSec) > 4400 }
//        .andThen(runOnce { initSpin = true })
//        .andThen(run{})
//        .until { (abs(inputs.mainVelocityMetersPerSec) + abs(inputs.revVelocityMetersPerSec)) / 2 < 4000  && initSpin == true}
//        .andThen(runOnce { initSpin = false
//            startedSpinning = true})
//        .andThen(run{io.setMainAndRevVoltage(-0.2 * 12.0)})
//        .andThen(run{})
//        .until { (abs(inputs.mainVelocityMetersPerSec)  +  abs(inputs.revVelocityMetersPerSec)) / 2 < 2500 && startedSpinning == true }
//        .andThen(stopWheels())
//        .andThen(runOnce{startedSpinning = false})



    fun outtake(): Command =
        runOnce { holdIN = false }.andThen(
        run{io.setMainAndRevVoltage(0.4 * 12.0)}
        .withTimeout(1.0)
        .andThen(run{})
        .until { abs(inputs.mainVelocityMetersPerSec) > 4400 }
        .andThen(stopWheels()))




    fun stopAll(): Command = runOnce{
        stopped = true
        io.stop()
    }
    fun stopJoint(): Command = runOnce{
        stopped = true
        io.stopJoint()
    }
    fun stopWheels(): Command = runOnce{ io.stopMain() }.andThen(runOnce { io.stopRev()})
    fun toggle(): Command = if (inputs.jointAngle.degrees > 0.0) {
        raise()
    } else if (inputs.jointAngle.degrees < 0.0) {
        lower()
    } else {
        runOnce{}
    }

    fun raise(): Command =
        runOnce{stopped = true}.andThen(
        run {
        io.setJointVoltage(cntrl.calculate(inputs.jointAngle.degrees, 5.0))
    }.until { (inputs.jointAngle - 5.0.rotation2dFromDeg()).degrees < 2.0 }.andThen(
        runOnce{
            cntrl.reset(inputs.jointAngle.degrees)
        }.andThen(
            runOnce {
                setpoint = 5.0
                stopped = false
                holdIN = false
            } ) ))

    fun lower(): Command =
        runOnce {
            stopped = true
        }
            .andThen(run {
        io.setJointVoltage(cntrl.calculate(inputs.jointAngle.degrees, -105.0))
    }.until { (inputs.jointAngle - (-105.0).rotation2dFromDeg()).degrees < 2.0 }.andThen(
        runOnce{
            cntrl.reset(inputs.jointAngle.degrees)
        }.andThen(
            runOnce {
                setpoint = -105.0
                stopped = false } ) ))


    fun half(): Command =
        runOnce {
            stopped = true
        }.andThen(
        run{
        io.setJointVoltage(cntrl.calculate(inputs.jointAngle.degrees, -75.0))
        }.until { (inputs.jointAngle - (-75.0).rotation2dFromDeg()).degrees < 2.0 }.andThen(
        runOnce{
            cntrl.reset(inputs.jointAngle.degrees)
        }.andThen(
            runOnce {
                setpoint = -75.0
                stopped = false } ) ))

            fun threeQuarters(): Command =
                runOnce {
                    stopped = true
                }.andThen(
                    run{
                        io.setJointVoltage(cntrl.calculate(inputs.jointAngle.degrees, -70.0))
                    }.until { (inputs.jointAngle - (-70.0).rotation2dFromDeg()).degrees < 2.0 }.andThen(
                        runOnce{
                            cntrl.reset(inputs.jointAngle.degrees)
                        }.andThen(
                            runOnce {
                                setpoint = -70.0
                                stopped = false } ) ))

    fun dropAlgae() : Command = runOnce {
        outtake()
        lower()
    }
}