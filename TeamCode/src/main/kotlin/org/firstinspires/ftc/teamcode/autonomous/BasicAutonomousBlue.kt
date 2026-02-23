package org.firstinspires.ftc.teamcode.autonomous

import com.hamosad.lib.commands.Command
import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.commands.andThen
import com.hamosad.lib.commands.meanwhile
import com.hamosad.lib.commands.withTimeout
import com.hamosad.lib.opModes.CommandOpModeAutonomous
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.commands.angularVelocityDriveCommand
import org.firstinspires.ftc.teamcode.commands.setWheelMotorsVoltageCommand
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumSubsystem
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem

@Autonomous
class BasicAutonomousBlue: CommandOpModeAutonomous() {
    override var subsystemsToUse: List<Subsystem> = listOf(MecanumSubsystem, ShooterSubsystem)

    override fun configureDefaultCommands() {
        MecanumSubsystem.angularVelocityDriveCommand(
            true,
            { 0.0 },
            { 0.0 },
            { 0.0 }
        )
    }

    override fun getAutonomousCommand(): Command {
        return (MecanumSubsystem.angularVelocityDriveCommand(
            true,
            { 1.0 },
            { 0.0 },
            { 0.0 }
        ).withTimeout(1.0)) andThen
                ((MecanumSubsystem.angularVelocityDriveCommand(
                    true,
                    {0.0},
                    {0.0},
                    {0.0}) meanwhile
                        ShooterSubsystem.setWheelMotorsVoltageCommand(10.0)).withTimeout(8.0)) andThen
                ((MecanumSubsystem.angularVelocityDriveCommand(
                    true,
                    {-1.0},
                    {-1.0},
                    {0.0}
                ).withTimeout (1.0)) meanwhile ShooterSubsystem.setWheelMotorsVoltageCommand(0.0))
}}