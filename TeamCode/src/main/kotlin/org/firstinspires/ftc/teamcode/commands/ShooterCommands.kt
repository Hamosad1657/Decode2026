package org.firstinspires.ftc.teamcode.commands

import com.hamosad.lib.commands.Command
import com.hamosad.lib.commands.runCommand
import com.hamosad.lib.math.AngularVelocity
import com.hamosad.lib.math.Rotation2d
import com.hamosad.lib.math.Volts
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem

fun ShooterSubsystem.setHoodAngleCommand(angle: Rotation2d): Command = ShooterSubsystem.runCommand { setHoodAngle(angle) }

fun ShooterSubsystem.setWheelSpeedCommand(speed: AngularVelocity): Command = ShooterSubsystem.runCommand { updateShooterVelocityControl(speed) }

fun ShooterSubsystem.setHoodAngleAndWheelSpeedCommand(angle: Rotation2d, speed: AngularVelocity) =
    ShooterSubsystem.runCommand { setHoodAngle(angle); updateShooterVelocityControl(speed) }

fun ShooterSubsystem.setWheelMotorsVoltageCommand(voltage: Volts) = ShooterSubsystem.runCommand { setWheelMotorsVoltage(voltage) }