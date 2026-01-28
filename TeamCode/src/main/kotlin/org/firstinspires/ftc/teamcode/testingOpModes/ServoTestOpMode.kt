package org.firstinspires.ftc.teamcode.testingOpModes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.subsystems.loader.LoaderConstants

@TeleOp
class ServoTestOpMode: LinearOpMode() {

    override fun runOpMode() {
        waitForStart()
        val servo: Servo = super.hardwareMap.get(Servo::class.java, LoaderConstants.ARM_SERVO_NAME)
        servo.direction = Servo.Direction.REVERSE
        while (super.opModeIsActive()) {
            servo.position = if (super.gamepad1.right_bumper) 1.0 else 0.0
            super.telemetry.addData("Commanded postion", servo.position)
            super.telemetry.update()
        }
    }

}