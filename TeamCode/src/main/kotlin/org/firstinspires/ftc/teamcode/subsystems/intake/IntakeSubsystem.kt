package org.firstinspires.ftc.teamcode.subsystems.intake

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants as Constants
import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.components.motors.HaMotor
import com.hamosad.lib.components.motors.MotorType
import com.hamosad.lib.math.Volts
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

object IntakeSubsystem: Subsystem() {
    var motor: HaMotor? = null

    override fun init(newHardwareMap: HardwareMap) {
        super.init(newHardwareMap)
        motor = HaMotor(Constants.MOTOR_NAME, hardwareMap!!, MotorType.GO_BUILDA5202)
        motor?.setDirection(Constants.DIRECTION)
        motor?.stopMotor()
    }

    private fun setVoltage(voltage: Volts) {
        motor?.setVoltage(voltage)
    }

    fun runIntake() {
        setVoltage(Constants.INTAKING_VOLTAGE)
    }

    fun runIntakeReverse() {
        setVoltage(-Constants.INTAKING_VOLTAGE)
    }

    fun stopIntake() {
        motor?.stopMotor()
    }

    // periodic
    override fun periodic() {

    }

    // Telemetry
    override fun updateTelemetry(telemetry: Telemetry, dashboardPacket: TelemetryPacket) {
        dashboardPacket.put("Intake velocity RPM", motor?.currentVelocity?.asRPM ?: 0.0)
    }
}