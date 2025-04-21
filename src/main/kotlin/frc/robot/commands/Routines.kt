package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.lib.FieldPositions
import frc.lib.swerve.SwerveDriveBase
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.algaeintake.AlgaeIntakeSubsystem
import frc.robot.climber.ClimberSubsystem
import frc.robot.coralintake.CoralIntakeSubsystem
import frc.robot.elevator.ElevatorSubsystem

/** Setup shuffleboard buttons
 *
 * TODO: Put controller stuff in here */

/** High level routines / commands consisting of lower level commands */
object Routines {

    //intakes new coral from the source
    //raises coral arm, intakes coral, does NOT lower
    fun takeCoral(coralIntake: CoralIntakeSubsystem, elevator: ElevatorSubsystem): Command = SequentialCommandGroup(
        coralIntake.raiseToIntake().alongWith(elevator.setToPosition(-8.0)).andThen(coralIntake.intake()).andThen(coralIntake.lower()).andThen(elevator.setToPosition(Constants.Levels.LEVEL0.lvl))
    )

    // scores coral on reef at level
    //raise elevator, raise coral arm, run coral outake, lower coral arm, lower elevator
    fun placeCoralAtLevel(level: Double, elevator: ElevatorSubsystem, coralIntake: CoralIntakeSubsystem): Command =
        SequentialCommandGroup(
            elevator.setToPosition(level).andThen(
                coralIntake.raiseToScore()
            )
//            .andThen(coralIntake.outtake() ).andThen(coralIntake.lower())
//            .andThen(elevator.setToPosition(Constants.Levels.LEVEL0.lvl)))
        )

    fun placeAlgae(elevator: ElevatorSubsystem, algaeIntake: AlgaeIntakeSubsystem): Command = SequentialCommandGroup(
        elevator.setToPosition(0.0),
        algaeIntake.half(),
        algaeIntake.outtake(),
        algaeIntake.raise(),
    )

    //takes algae from reef at level
    //raise elevator, moves coral arm out of way, raises algae arm, runs algae flywheels, lowers elevator
    fun reefPickup(level: Double, elevator: ElevatorSubsystem, algaeIntake: AlgaeIntakeSubsystem): Command = SequentialCommandGroup(
        elevator.setToPosition(level) ,
        algaeIntake.half(),
        algaeIntake.intake(),
        WaitCommand(1.0),
        elevator.setToPosition(Constants.Levels.LEVEL0.lvl)
    )

    fun groundPickup(elevator: ElevatorSubsystem, algaeIntake: AlgaeIntakeSubsystem): Command = SequentialCommandGroup(
        elevator.setToPosition(Constants.Levels.LEVEL0.lvl),
        algaeIntake.lower(),
        algaeIntake.intake()
    )


    //stops everything, doesn't lower
    fun stopAll(elevator: ElevatorSubsystem, algaeIntake: AlgaeIntakeSubsystem, climberSubsystem: ClimberSubsystem): Command = SequentialCommandGroup(
        algaeIntake.stopWheels(),
        algaeIntake.raise(),
        elevator.setToPosition(Constants.Levels.LEVEL0.lvl),
        climberSubsystem.stop()
    )

    fun actualEmergency(elevator: ElevatorSubsystem, algaeIntake: AlgaeIntakeSubsystem, climberSubsystem: ClimberSubsystem) = SequentialCommandGroup(
        algaeIntake.stopAll(),
        elevator.stop(),
        climberSubsystem.stop()
    )

    fun inchForward(drive: SwerveDriveBase, teleopCommand:Command) = SwerveJoystickDrive(drive, {0.2}, {0.0}, {0.0}, {false}).withTimeout(0.2).andThen(teleopCommand)

    fun inchBack(drive: SwerveDriveBase, teleopCommand:Command) = SwerveJoystickDrive(drive, {-0.2}, {0.0}, {0.0}, {false}).withTimeout(0.2).andThen(teleopCommand)

    fun inchLeft(drive: SwerveDriveBase, teleopCommand:Command) = SwerveJoystickDrive(drive, {0.0}, {-0.2}, {0.0}, {false}).withTimeout(0.2).andThen(teleopCommand)

    fun inchRight(drive: SwerveDriveBase, teleopCommand:Command) = SwerveJoystickDrive(drive, {0.0}, {0.2}, {0.0}, {false}).withTimeout(0.2).andThen(teleopCommand)


    fun goLeftCommand(elevatorSubsystem: ElevatorSubsystem, coralIntakeSubsystem: CoralIntakeSubsystem, robot: RobotContainer): SequentialCommandGroup? =
        robot.goto(FieldPositions.reefLeftFar).andThen(
            placeCoralAtLevel(Constants.Levels.LEVEL3.lvl, elevatorSubsystem, coralIntakeSubsystem)
        ).andThen(
            coralIntakeSubsystem.outtake().andThen(coralIntakeSubsystem.lower())
                .andThen(coralIntakeSubsystem.reset()).andThen(
                    elevatorSubsystem.setToPosition(
                        Constants.Levels.LEVEL0.lvl
                    )
                )
        ).andThen(
            robot.goto(FieldPositions.sourceLeft)
        ).andThen(
            takeCoral(coralIntakeSubsystem, elevatorSubsystem)
        ).andThen(
            robot.goto(FieldPositions.reefLeftClose)
        ).andThen(
            placeCoralAtLevel(Constants.Levels.LEVEL3.lvl, elevatorSubsystem, coralIntakeSubsystem)
        ).andThen(
            coralIntakeSubsystem.outtake().andThen(coralIntakeSubsystem.lower())
                .andThen(coralIntakeSubsystem.reset()).andThen(
                    elevatorSubsystem.setToPosition(
                        Constants.Levels.LEVEL0.lvl
                    )
                )
        ).andThen(
            robot.goto(FieldPositions.sourceLeft)
            ).andThen(
                takeCoral(coralIntakeSubsystem, elevatorSubsystem)
            ).andThen(
            robot.goto(FieldPositions.reefCenterClose)
            ).andThen(
                placeCoralAtLevel(Constants.Levels.LEVEL3.lvl, elevatorSubsystem, coralIntakeSubsystem)
            ).andThen(
                coralIntakeSubsystem.outtake().andThen(coralIntakeSubsystem.lower())
                    .andThen(coralIntakeSubsystem.reset()).andThen(
                        elevatorSubsystem.setToPosition(
                            Constants.Levels.LEVEL0.lvl
                        )
                    )
            )

    fun goRightCommand(elevatorSubsystem: ElevatorSubsystem, coralIntakeSubsystem: CoralIntakeSubsystem, robot: RobotContainer): SequentialCommandGroup? =
        robot.goto(FieldPositions.reefRightFar).andThen(
            Routines.placeCoralAtLevel(Constants.Levels.LEVEL3.lvl, elevatorSubsystem, coralIntakeSubsystem)
        ).andThen(
            coralIntakeSubsystem.outtake().andThen(coralIntakeSubsystem.lower())
                .andThen(coralIntakeSubsystem.reset()).andThen(
                    elevatorSubsystem.setToPosition(
                        Constants.Levels.LEVEL0.lvl
                    )
                )
        ).andThen(
            robot.goto(FieldPositions.sourceRight)
        ).andThen(
            Routines.takeCoral(coralIntakeSubsystem, elevatorSubsystem)
        ).andThen(
            robot.goto(FieldPositions.reefRightClose)
        ).andThen(
            Routines.placeCoralAtLevel(Constants.Levels.LEVEL3.lvl, elevatorSubsystem, coralIntakeSubsystem)
        ).andThen(
            coralIntakeSubsystem.outtake().andThen(coralIntakeSubsystem.lower())
                .andThen(coralIntakeSubsystem.reset()).andThen(
                    elevatorSubsystem.setToPosition(
                        Constants.Levels.LEVEL0.lvl
                    )
                )
        )
            .andThen(
                robot.goto(FieldPositions.sourceRight)
            ).andThen(
                Routines.takeCoral(coralIntakeSubsystem, elevatorSubsystem)
            ).andThen(
                robot.goto(FieldPositions.reefCenterClose)
            ).andThen(
                Routines.placeCoralAtLevel(Constants.Levels.LEVEL3.lvl, elevatorSubsystem, coralIntakeSubsystem)
            ).andThen(
                coralIntakeSubsystem.outtake().andThen(coralIntakeSubsystem.lower())
                    .andThen(coralIntakeSubsystem.reset()).andThen(
                        elevatorSubsystem.setToPosition(
                            Constants.Levels.LEVEL0.lvl
                        )
                    )
            )

}