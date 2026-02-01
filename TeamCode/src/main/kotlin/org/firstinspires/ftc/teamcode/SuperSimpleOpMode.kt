package org.firstinspires.ftc.teamcode

import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.commands.runOnce
import com.hamosad.lib.components.Controllers.HaCommandController
import com.hamosad.lib.math.AngularVelocity
import com.hamosad.lib.math.HaRotation2d
import com.hamosad.lib.opModes.CommandOpModeTeleop
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.angularVelocityDriveCommand
import org.firstinspires.ftc.teamcode.commands.disableIntakeCommand
import org.firstinspires.ftc.teamcode.commands.manualControlCommand
import org.firstinspires.ftc.teamcode.commands.runIntakeCommand
import org.firstinspires.ftc.teamcode.commands.runIntakeReverseCommand
import org.firstinspires.ftc.teamcode.commands.setWheelMotorsVoltageCommand
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.loader.LoaderSubsystem
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumSubsystem
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterState
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem

@TeleOp
class SuperSimpleOpMode: CommandOpModeTeleop() {
    val stationaryShootingState: ShooterState = ShooterState(HaRotation2d.fromDegrees(0.0), AngularVelocity.fromRPM(0.0))

    val controller1 = HaCommandController({ super.gamepad1 }, 0.03, 1)
    val controller2 = HaCommandController({ super.gamepad2 }, 0.03, 1)
    override var subsystemsToUse: List<Subsystem> = listOf(
        MecanumSubsystem, LoaderSubsystem,
        ShooterSubsystem, IntakeSubsystem
    )

    override fun configureDefaultCommands() {
        MecanumSubsystem.defaultCommand = MecanumSubsystem.angularVelocityDriveCommand(
            true,
            { controller1.getLeftX() },
            { controller1.getLeftY() },
            { controller1.getRightX() },
        )
        IntakeSubsystem.defaultCommand = IntakeSubsystem.disableIntakeCommand()
        LoaderSubsystem.defaultCommand = LoaderSubsystem.manualControlCommand(
            { controller2.getLeftX() }, { controller2.controller()?.right_bumper ?: false }
        )
        ShooterSubsystem.defaultCommand = ShooterSubsystem.setWheelMotorsVoltageCommand(0.0)
    }

    override fun configureBindings() {
        controller1.options().onTrue(MecanumSubsystem.runOnce { MecanumSubsystem.resetGyro() })

        controller1.r2Pressed().whileTrue(IntakeSubsystem.runIntakeCommand())
        controller1.l2Pressed().whileTrue(IntakeSubsystem.runIntakeReverseCommand())

        controller2.l1().whileTrue(ShooterSubsystem.setWheelMotorsVoltageCommand(12.0))
    }
}