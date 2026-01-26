package org.firstinspires.ftc.teamcode

import com.hamosad.lib.commands.Command
import com.hamosad.lib.opModes.CommandOpModeTeleop
import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.components.Controllers.HaCommandController
import com.hamosad.lib.math.AngularVelocity
import com.hamosad.lib.math.HaRotation2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.Ball
import org.firstinspires.ftc.teamcode.commands.aimAtGoalAndMoveWithVision
import org.firstinspires.ftc.teamcode.commands.angularVelocityDriveCommand
import org.firstinspires.ftc.teamcode.commands.collectSimpleCommand
import org.firstinspires.ftc.teamcode.commands.disableIntakeCommand
import org.firstinspires.ftc.teamcode.commands.maintainHoodAngleAndWheelSpeedCommand
import org.firstinspires.ftc.teamcode.commands.setServoVoltageCommand
import org.firstinspires.ftc.teamcode.commands.shootBallCommand
import org.firstinspires.ftc.teamcode.commands.shootClosestBallCommand
import org.firstinspires.ftc.teamcode.commands.shootClosestKnownBallCommand
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.loader.LoaderConstants
import org.firstinspires.ftc.teamcode.subsystems.loader.LoaderSubsystem
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumSubsystem
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterState
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem

@TeleOp
class SimpleOpMode: CommandOpModeTeleop() {
    val stationaryShootingState: ShooterState = ShooterState(HaRotation2d.fromDegrees(0.0), AngularVelocity.fromRPM(0.0))

    val controller1 = HaCommandController({ super.gamepad1 }, 0.03, 1)
    val controller2 = HaCommandController({ super.gamepad2 }, 0.03, 1)
    override var subsystemsToUse: List<Subsystem> = listOf(
        MecanumSubsystem, LoaderSubsystem,
        ShooterSubsystem, IntakeSubsystem
    )

    override fun configureDefaultCommands() {
        MecanumSubsystem.defaultCommand = MecanumSubsystem.angularVelocityDriveCommand(
            true,
            { controller1.getLeftX() },
            { controller1.getLeftY() },
            { controller1.getRightX() },
        )
        IntakeSubsystem.defaultCommand = IntakeSubsystem.disableIntakeCommand()
        LoaderSubsystem.defaultCommand = LoaderSubsystem.setServoVoltageCommand(LoaderConstants.ROULETTE_SPIN_VOLTAGE)
        ShooterSubsystem.defaultCommand = ShooterSubsystem.maintainHoodAngleAndWheelSpeedCommand(
            ShooterState(HaRotation2d.fromDegrees(0.0), AngularVelocity.fromRPM(0.0)))
    }

    override fun configureBindings() {
        controller1.cross().toggleOnTrue(collectSimpleCommand(1.0))

        controller1.r1().toggleOnTrue(MecanumSubsystem.aimAtGoalAndMoveWithVision(
            { controller1.getLeftX() }, { controller1.getLeftY() },))


        controller2.square().toggleOnTrue(shootBallCommand(Ball.BALL_1, stationaryShootingState, 4.0))
        controller2.triangle().toggleOnTrue(shootBallCommand(Ball.BALL_2, stationaryShootingState, 4.0))
        controller2.circle().toggleOnTrue(shootBallCommand(Ball.BALL_3, stationaryShootingState, 4.0))

        controller2.l1().toggleOnTrue(shootClosestBallCommand(stationaryShootingState, 4.0))
        controller2.r1().toggleOnTrue(shootClosestKnownBallCommand(stationaryShootingState, 4.0))
    }
}