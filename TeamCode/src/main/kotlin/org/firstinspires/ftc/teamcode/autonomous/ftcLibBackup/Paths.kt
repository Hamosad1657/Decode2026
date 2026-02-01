package org.firstinspires.ftc.teamcode.autonomous.ftcLibBackup


import com.arcrobotics.ftclib.purepursuit.Path
import com.arcrobotics.ftclib.purepursuit.Waypoint
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint
import com.arcrobotics.ftclib.purepursuit.waypoints.PointTurnWaypoint
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint
import org.firstinspires.ftc.teamcode.autonomous.PathPoints
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumConstants as Constants

object Paths {
    // Close
    val startClose: Waypoint = StartWaypoint(PathPoints.START_CLOSE.pose.toFTCLibPose())
    val shootClose: Waypoint = EndWaypoint(
        PathPoints.SHOOT_CLOSE.pose.toFTCLibPose(),
        0.0,
        0.0,
        Constants.FOLLOW_RADIUS.asMeters,
        Constants.POSITION_BUFFER,
        Constants.ANGLE_BUFFER,
    )
    val closeEndPoint: Waypoint = EndWaypoint(
        PathPoints.CLOSE_ENDPOINT.pose.toFTCLibPose(),
        0.0,
        0.0,
        Constants.FOLLOW_RADIUS.asMeters,
        Constants.POSITION_BUFFER,
        Constants.ANGLE_BUFFER,
    )


    val goToCloseEndpointPath: Path = Path(
        shootClose,
        closeEndPoint
    )

    val startCollectingClose: Waypoint = PointTurnWaypoint(
        PathPoints.START_COLLECTING_CLOSE.pose.toFTCLibPose(),
        0.5,
        0.5,
        Constants.FOLLOW_RADIUS.asMeters,
        Constants.POSITION_BUFFER,
        Constants.ANGLE_BUFFER,
    )
    val finishCollectingClose: Waypoint = PointTurnWaypoint(
        PathPoints.FINISH_COLLECTING_CLOSE.pose.toFTCLibPose(),
        0.0,
        0.0,
        Constants.FOLLOW_RADIUS.asMeters,
        Constants.POSITION_BUFFER,
        Constants.ANGLE_BUFFER,
    )

    val shootClosePath: Path = Path(
        startClose,
        shootClose
    )

    // Far
    val startFar: Waypoint = StartWaypoint(PathPoints.START_FAR.pose.toFTCLibPose())
    val shootFar: Waypoint = PointTurnWaypoint(
        PathPoints.SHOOT_FAR.pose.toFTCLibPose(),
        0.0,
        0.0,
        Constants.FOLLOW_RADIUS.asMeters,
        Constants.POSITION_BUFFER,
        Constants.ANGLE_BUFFER
        )
    val farEndPoint: Waypoint = PointTurnWaypoint(
        PathPoints.FAR_ENDPOINT.pose.toFTCLibPose(),
        0.0,
        0.0,
        Constants.FOLLOW_RADIUS.asMeters,
        Constants.POSITION_BUFFER,
        Constants.ANGLE_BUFFER
    )

    val shootFarPath: Path = Path(
        startFar,
        shootFar
    )
    val goToFarEndpointPath: Path = Path(
        shootFar,
        farEndPoint
    )
}