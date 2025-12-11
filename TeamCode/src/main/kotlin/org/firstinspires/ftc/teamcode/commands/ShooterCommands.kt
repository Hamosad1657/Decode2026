package org.firstinspires.ftc.teamcode.commands

import com.hamosad.lib.commands.Command
import com.hamosad.lib.commands.andThen
import com.hamosad.lib.commands.runCommand
import com.hamosad.lib.commands.runOnce
import com.hamosad.lib.math.AngularVelocity
import com.hamosad.lib.math.Rotation2d
import com.hamosad.lib.math.Volts
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem

fun ShooterSubsystem.setHoodAngleCommand(angle: Rotation2d): Command = ShooterSubsystem.runOnce { setHoodAngle(angle) }

fun ShooterSubsystem.maintainWheelSpeedCommand(speed: AngularVelocity): Command = ShooterSubsystem.runCommand { updateShooterVelocityControl(speed) }

fun ShooterSubsystem.setHoodAngleAndMaintainWheelSpeedCommand(angle: Rotation2d, speed: AngularVelocity) =
    setHoodAngleCommand(angle) andThen maintainWheelSpeedCommand(speed)


fun ShooterSubsystem.setWheelMotorsVoltageCommand(voltage: Volts) = ShooterSubsystem.runCommand { setWheelMotorsVoltage(voltage) }