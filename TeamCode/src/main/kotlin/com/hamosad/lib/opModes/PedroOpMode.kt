package com.hamosad.lib.opModes

import com.hamosad.lib.commands.CommandScheduler
import com.hamosad.lib.commands.Subsystem
import com.pedropathing.follower.Follower
import java.util.Timer
import org.firstinspires.ftc.teamcode.pedropathing.PedroConstants as Constants

abstract class PedroOpMode: TimedRobotOpMode() {
    abstract var subsystemsToUse: List<Subsystem>
    var useTelemetry: Boolean = true

    var follower: Follower? = null // Pedro Pathing follower instance
    var pathState = 0 // Current autonomous path state (state machine)
    val currentTime get() = System.currentTimeMillis()
    var startTime: Long = 0L

    //updates the current pathstate
    abstract fun autonomousPathUpdate()

    //runs commands in relation to current pathstate
    abstract fun runPathStateCommands()

    fun setPathState(state: Int){
        pathState = state
    }

    fun setStartTime() {
        startTime = currentTime
    }

    final override fun disabledInit() {
        follower = Constants.createFollower(hardwareMap)

        telemetry.addData("Status", "Initialized")
        telemetry.update()

        for (subsystem in subsystemsToUse) {
            if (!subsystem.isInitialized) {
                subsystem.init(super.hardwareMap)
            }
            CommandScheduler.registerSubsystem(subsystem)
        }
    }

    final override fun disabledPeriodic() {
        for (subsystem in subsystemsToUse) {
            subsystem.periodic()
            if (useTelemetry) subsystem.updateTelemetry(super.telemetry)
        }
        super.telemetry.update()
        dashboardManager?.update(super.telemetry)
    }

    final override fun startInit() {
        setStartTime()
        CommandScheduler.initialize()
    }

    /** Called repeatedly after start is pressed. */
    final override fun startPeriodic() {
        CommandScheduler.execute()
        if (useTelemetry) {
            for (subsystem in subsystemsToUse) {
                subsystem.updateTelemetry(super.telemetry)
            }
        }

        follower?.update() // Update Pedro Pathing
        autonomousPathUpdate() // Update autonomous state machine

        // Log values to Panels and Driver Station
        telemetry.addData("Path State", pathState)
        telemetry.addData("X", follower?.pose?.x ?: 0)//panelsTelemetry!!.debug("X", follower!!.getPose().getX())
        telemetry.addData("Y", follower?.pose?.y ?: 0)
        telemetry.addData("Heading", follower!!.pose.heading)
        telemetry.update()

        dashboardManager?.update(super.telemetry)
    }

    /** Called once after stop is pressed. */
    final override fun onEnd() {
        CommandScheduler.reset()
    }
}