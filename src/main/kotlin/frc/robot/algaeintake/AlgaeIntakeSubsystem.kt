package frc.robot.algaeintake

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase

import frc.robot.Constants
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.WidgetType


//algae has 3 motors - one for arm, two for wheels. main and rev are wheel motors, rev always goes reverse direction
class AlgaeIntakeSubsystem(val io: AlgaeIntakeIO) : SubsystemBase() {
    val inputs = AlgaeIntakeIO.LoggedInputs()

    val troubleshootingTab = Shuffleboard.getTab("Algae")
    val jointAngleEntry = troubleshootingTab.add("Joint Angle", 0.0).withPosition(2, 0).entry
    val jointVelocityMetersPerSecEntry = troubleshootingTab.add("Joint Velocity MPS", 0.0).withPosition(2, 1).entry
    val mainVelocityMetersPerSecEntry = troubleshootingTab.add("Main Velocity MPS", 0.0).withPosition(2, 2).entry
    val revVelocityMetersPerSecEntry = troubleshootingTab.add("Reverse Velocity MPS", 0.0).withPosition(2, 3).entry

    init {
        troubleshootingTab.add("up", {raise()}).withWidget(BuiltInWidgets.kCommand)
        troubleshootingTab.add("down", {lower()}).withWidget(BuiltInWidgets.kCommand)
        troubleshootingTab.add("stop", {stopJoint()}).withWidget(BuiltInWidgets.kCommand)
    }
    
    override fun periodic() {
        io.updateInputs(inputs)

        // Debug stuff I guess
        jointAngleEntry.setDouble(inputs.jointAngle.degrees)
        jointVelocityMetersPerSecEntry.setDouble(inputs.jointVelocityMetersPerSec.toDouble())
        mainVelocityMetersPerSecEntry.setDouble(inputs.mainVelocityMetersPerSec.toDouble())
        revVelocityMetersPerSecEntry.setDouble(inputs.revVelocityMetersPerSec.toDouble())

    }
    fun intake():Command = run { io.setMainAndRevVoltage(0.3 * 12.0) }.withTimeout(1.5)

    fun outtake():Command = run { io.setMainAndRevVoltage(-0.3 * 12.0) }.withTimeout(1.5)

    fun stopAll(): Command = runOnce{ io.stop() }
    fun stopJoint(): Command = runOnce{ io.stopJoint() }
    fun stopWheels(): Command = runOnce{ io.stopMain() }.alongWith(runOnce { io.stopRev()})
    fun toggle(): Command = if (inputs.jointAngle.degrees > 0.0) {
        raise()
    } else if (inputs.jointAngle.degrees < 0.0) {
        lower()
    } else {
        runOnce{}
    }

    fun raise(): Command = run {
        io.setJointVoltage(io.pidController.calculate(inputs.jointAngle.degrees, 60.0))
    }

    fun lower(): Command = run {
        io.setJointVoltage(io.pidController.calculate(inputs.jointAngle.degrees, 0.0))
    }

    fun dropAlgae() : Command = runOnce {
        outtake()
        lower()
    }
}