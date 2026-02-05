package org.firstinspires.ftc.teamcode.testingOpModes

import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.components.Controllers.HaCommandController
import com.hamosad.lib.opModes.CommandOpModeTeleop
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.Ball
import org.firstinspires.ftc.teamcode.commands.loadToShooterCommand
import org.firstinspires.ftc.teamcode.commands.positionBallToIntakeCommand
import org.firstinspires.ftc.teamcode.commands.positionBallToShooterCommand
import org.firstinspires.ftc.teamcode.commands.positionColorToShooterCommand
import org.firstinspires.ftc.teamcode.commands.setServoVoltageCommand
import org.firstinspires.ftc.teamcode.commands.stopLoadingToShooterCommand
import org.firstinspires.ftc.teamcode.subsystems.loader.BallColor
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
        controller.r2Pressed().whileTrue(LoaderSubsystem.positionBallToIntakeCommand(LoaderSubsystem.closestBallToIntake))
        controller.l2Pressed().whileTrue(LoaderSubsystem.positionBallToShooterCommand(LoaderSubsystem.closestBallToShooter))



    }
}