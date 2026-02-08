package com.hamosad.lib.opModes

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.hamosad.lib.commands.CommandScheduler
import com.hamosad.lib.commands.Subsystem
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.pedropathing.PedroConstants as Constants

abstract class PedroOpMode: TimedRobotOpMode() {
    abstract var subsystemsToUse: List<Subsystem>
    var useTelemetry: Boolean = true

    var follower: Follower? = null // Pedro Pathing follower instance
    private var pathState = 0 // Current autonomous path state (state machine)

    /** Set your default commands here. */
    abstract fun configureDefaultCommands()

    abstract fun redefinePaths()


    final override fun disabledInit() {
        follower = Constants.createFollower(hardwareMap)
        follower!!.setStartingPose(Pose(72.0, 8.0, Math.toRadians(90.0)))

        redefinePaths()

        telemetry.addData("Status", "Initialized")
        telemetry.update()

        for (subsystem in subsystemsToUse) {
            if (!subsystem.isInitialized) {
                subsystem.init(super.hardwareMap)
            }
            CommandScheduler.registerSubsystem(subsystem)
        }

        configureDefaultCommands()
    }

    final override fun disabledPeriodic() {
        for (subsystem in subsystemsToUse) {
            subsystem.periodic()
            if (useTelemetry) subsystem.updateTelemetry(super.telemetry)
        }
        super.telemetry.update()
        dashboardManager.update(super.telemetry)
    }

    final override fun startInit() {
        CommandScheduler.initialize()
    }

    abstract fun runPathStateCommands()

    /** Called repeatedly after start is pressed. */
    final override fun startPeriodic() {
        CommandScheduler.execute()
        if (useTelemetry) {
            for (subsystem in subsystemsToUse) {
                subsystem.updateTelemetry(super.telemetry)
            }
        }

        follower?.update() // Update Pedro Pathing
        pathState = autonomousPathUpdate() // Update autonomous state machine

        // Log values to Panels and Driver Station
        telemetry.addData("Path State", pathState)
        telemetry.addData("X", follower?.pose?.x ?: 0)//panelsTelemetry!!.debug("X", follower!!.getPose().getX())
        telemetry.addData("Y", follower?.pose?.y ?: 0)
        telemetry.addData("Heading", follower!!.pose.heading)
        telemetry.update()

        dashboardManager.update(super.telemetry)
    }

    fun autonomousPathUpdate(): Int {
        // Event markers will automatically trigger at their positions
        // Make sure to register NamedCommands in your RobotContainer
        return pathState
    }

    /** Called once after stop is pressed. */
    final override fun onEnd() {
        CommandScheduler.reset()
    }
}