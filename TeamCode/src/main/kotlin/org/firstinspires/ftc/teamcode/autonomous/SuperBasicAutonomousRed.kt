package org.firstinspires.ftc.teamcode.autonomous

import com.hamosad.lib.commands.Command
import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.commands.withTimeout
import com.hamosad.lib.opModes.CommandOpModeAutonomous
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.commands.angularVelocityDriveCommand
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumSubsystem

@Autonomous
class SuperBasicAutonomousRed: CommandOpModeAutonomous() {
    override var subsystemsToUse: List<Subsystem> = listOf(MecanumSubsystem)

    override fun configureDefaultCommands() {
        MecanumSubsystem.angularVelocityDriveCommand(
            true,
            { 0.0 },
            { 0.0 },
            { 0.0 }
        )
    }

    override fun getAutonomousCommand(): Command {
        return MecanumSubsystem.angularVelocityDriveCommand(
            true,
            { -1.0 },
            { 1.0 },
            { 0.0 }
        ).withTimeout(2.0)
    }
}