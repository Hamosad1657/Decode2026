package org.firstinspires.ftc.teamcode.testingOpModes

import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.components.Controllers.HaCommandController
import com.hamosad.lib.opModes.CommandOpModeTeleop
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.disableIntakeCommand
import org.firstinspires.ftc.teamcode.commands.runIntakeCommand
import org.firstinspires.ftc.teamcode.commands.runIntakeReverseCommand
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem

@TeleOp
class IntakeTest: CommandOpModeTeleop() {
    override var subsystemsToUse: List<Subsystem> = listOf(IntakeSubsystem)
    val controller = HaCommandController({ super.gamepad1 }, 0.05)

    override fun configureDefaultCommands() {
        IntakeSubsystem.defaultCommand = IntakeSubsystem.disableIntakeCommand()
    }

    override fun configureBindings() {
        controller.l1().onTrue(IntakeSubsystem.runIntakeCommand())
        controller.r1().onTrue(IntakeSubsystem.runIntakeReverseCommand())
    }
}