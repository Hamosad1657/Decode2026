package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.arcrobotics.ftclib.purepursuit.Path
import com.hamosad.lib.commands.Command
import com.hamosad.lib.commands.runCommand
import com.hamosad.lib.math.AngularVelocity
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumConstants as Constants
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumSubsystem

fun MecanumSubsystem.angularVelocityDriveCommand(
    fieldRelative: Boolean,
    leftJoyX: () -> Double,
    leftJoyY: () -> Double,
    rightJoyX: () -> Double
) = MecanumSubsystem.runCommand {
    drive(fieldRelative, ChassisSpeeds(
        leftJoyY() * Constants.MAX_CHASSIS_SPEED,
        -leftJoyX() * Constants.MAX_CHASSIS_SPEED,
        -AngularVelocity.fromRPS(rightJoyX() * Constants.MAX_CHASSIS_ANGULAR_VELOCITY.asRPS).asRadPS,
    ))
}

fun MecanumSubsystem.purePursuitFollowPath(path: Path): Command = MecanumSubsystem.runCommand {
    val chassisSpeeds = path.loop(robotPose.translation2d.x, robotPose.translation2d.y, robotPose.rotation2d.asDegrees)
    drive(true, ChassisSpeeds(chassisSpeeds[1], chassisSpeeds[2], chassisSpeeds[3]))
}

//fun MecanumSubsystem.testMotorsCommand(motor: Int): Command = MecanumSubsystem.runCommand {
//    testMotors(motor)
//}
//
//fun MecanumSubsystem.testPIDCommand(): Command = MecanumSubsystem.runCommand {
//    testPID()
//}

//fun MecanumSubsystem.followPathCommand(path: Path, hardwareMap: HardwareMap): Command {
//    val follower = PedroConstants.createFollower(hardwareMap)
//}