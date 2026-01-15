package org.firstinspires.ftc.teamcode.commands

import com.hamosad.lib.commands.Command
import com.hamosad.lib.commands.andThen
import com.hamosad.lib.commands.raceWith
import com.hamosad.lib.commands.runCommand
import com.hamosad.lib.commands.runOnce
import com.hamosad.lib.math.AngularVelocity
import com.hamosad.lib.math.HaRotation2d
import com.hamosad.lib.math.Volts
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem

fun ShooterSubsystem.setHoodAngleCommand(angle: HaRotation2d): Command = ShooterSubsystem.runOnce { setHoodAngle(angle) }

fun ShooterSubsystem.setWheelSpeedCommand(speed: AngularVelocity): Command = ShooterSubsystem.runCommand { updateDesiredVelocity(speed) }

fun ShooterSubsystem.setHoodAngleAndWheelSpeedCommand(angle: HaRotation2d, speed: AngularVelocity) =
    setHoodAngleCommand(angle) raceWith setWheelSpeedCommand(speed)


fun ShooterSubsystem.setWheelMotorsVoltageCommand(voltage: Volts) = ShooterSubsystem.runCommand { setWheelMotorsVoltage(voltage) }