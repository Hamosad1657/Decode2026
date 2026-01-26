package org.firstinspires.ftc.teamcode.autonomous

import com.hamosad.lib.commands.Command
import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.commands.withTimeout
import com.hamosad.lib.opModes.CommandOpModeAutonomous
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.angularVelocityDriveCommand
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumSubsystem

@TeleOp
class SuperBasicAutonomous: CommandOpModeAutonomous() {
    override var subsystemsToUse: List<Subsystem> = listOf(MecanumSubsystem)

    override fun configureDefaultCommands() {}

    override fun getAutonomousCommand(): Command {
        return MecanumSubsystem.angularVelocityDriveCommand(
            false,
            { 0.0 },
            { 1.0 },
            { 0.0 }
        ).withTimeout(2.0)
    }
}