package org.firstinspires.ftc.teamcode.testingOpModes

import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.components.Controllers.HaCommandController
import com.hamosad.lib.math.HaRotation2d
import com.hamosad.lib.opModes.CommandOpModeTeleop
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.Ball
import org.firstinspires.ftc.teamcode.commands.continuouslyLoadToShooterCommand
import org.firstinspires.ftc.teamcode.commands.holdRoulettePositionCommand
import org.firstinspires.ftc.teamcode.commands.loadToShooterCommand
import org.firstinspires.ftc.teamcode.commands.positionBallToIntakeCommand
import org.firstinspires.ftc.teamcode.commands.positionBallToShooterCommand
import org.firstinspires.ftc.teamcode.commands.setServoVoltageCommand
import org.firstinspires.ftc.teamcode.commands.stopLoadingToShooterCommand
import org.firstinspires.ftc.teamcode.subsystems.loader.LoaderSubsystem

@TeleOp
class LoaderTestOpMode: CommandOpModeTeleop() {
    override var subsystemsToUse: List<Subsystem> = listOf(LoaderSubsystem)
    val controller = HaCommandController({ super.gamepad1 }, 0.03, 1)

    override fun configureDefaultCommands() {
        LoaderSubsystem.defaultCommand = LoaderSubsystem.setServoVoltageCommand(0.0)
    }

    override fun configureBindings() {
        controller.l1().whileTrue(LoaderSubsystem.setServoVoltageCommand(6.0))
        controller.r1().whileTrue(LoaderSubsystem.setServoVoltageCommand(-6.0))

        controller.square().whileTrue(LoaderSubsystem.positionBallToShooterCommand(Ball.BALL_1))
        controller.triangle().whileTrue(LoaderSubsystem.positionBallToShooterCommand(Ball.BALL_2))
        controller.circle().whileTrue(LoaderSubsystem.positionBallToShooterCommand(Ball.BALL_3))

        controller.dpadUp().whileTrue(LoaderSubsystem.positionBallToIntakeCommand(Ball.BALL_2))
        controller.dpadLeft().whileTrue(LoaderSubsystem.positionBallToIntakeCommand(Ball.BALL_1))
        controller.dpadRight().whileTrue(LoaderSubsystem.positionBallToIntakeCommand(Ball.BALL_3))

        controller.l2Pressed().whileTrue(LoaderSubsystem.loadToShooterCommand())
        controller.r2Pressed().whileTrue(LoaderSubsystem.stopLoadingToShooterCommand())

        controller.dpadUp().whileTrue(LoaderSubsystem.continuouslyLoadToShooterCommand())
    }
}