package org.firstinspires.ftc.teamcode.commands

import com.hamosad.lib.commands.Command
import com.hamosad.lib.commands.andThen
import com.hamosad.lib.commands.runCommand
import com.hamosad.lib.commands.runOnce
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem

fun IntakeSubsystem.disableIntakeCommand() = runCommand {
    stopIntake()
}

fun IntakeSubsystem.runIntakeCommand(): Command = runCommand {
    runIntake()
} andThen runOnce { stopIntake() }

fun IntakeSubsystem.runIntakeReverseCommand(): Command = runCommand {
    runIntakeReverse()
} andThen runOnce { stopIntake() }