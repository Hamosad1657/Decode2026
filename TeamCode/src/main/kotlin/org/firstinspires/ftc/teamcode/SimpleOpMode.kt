package org.firstinspires.ftc.teamcode

import com.hamosad.lib.opModes.CommandOpModeTeleop
import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.components.Controllers.HaCommandController
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.angularVelocityDriveCommand
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.loader.LoaderSubsystem
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumSubsystem
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem

@TeleOp
class SimpleOpMode: CommandOpModeTeleop() {
    override var subsystemsToUse: List<Subsystem> = listOf(
        MecanumSubsystem, LoaderSubsystem,
        ShooterSubsystem, IntakeSubsystem
    )
    val controller = HaCommandController({ super.gamepad1 }, 0.03, 1)

    override fun configureDefaultCommands() {
        MecanumSubsystem.defaultCommand = MecanumSubsystem.angularVelocityDriveCommand(
            true,
            { controller.getLeftX() },
            { controller.getLeftY() },
            { controller.getRightX() },
        )
    }

    override fun configureBindings() {
        controller.r2Pressed()
    }
}