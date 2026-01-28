package org.firstinspires.ftc.teamcode

import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.commands.runOnce
import com.hamosad.lib.components.Controllers.HaCommandController
import com.hamosad.lib.opModes.CommandOpModeTeleop
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.angularVelocityDriveCommand
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumSubsystem

@TeleOp
class DefenseOpMode: CommandOpModeTeleop() {
    override var subsystemsToUse: List<Subsystem> = listOf(MecanumSubsystem)
    val controller = HaCommandController({ super.gamepad1 }, 0.03, 1)

    override fun configureDefaultCommands() {
        MecanumSubsystem.defaultCommand = MecanumSubsystem.angularVelocityDriveCommand(
            true,
            { controller.getLeftX() },
            { controller.getLeftY() },
            { controller.getRightX() },
        )    }

    override fun configureBindings() {
        controller.options().onTrue(MecanumSubsystem.runOnce { MecanumSubsystem.resetGyro() })
    }
}