package org.firstinspires.ftc.teamcode.localization

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.Twist2dDual

class FuseLocation(val otosLocalizer: OTOSLocalizer, val twoWheel: TwoDeadWheelLocalizer) :
    SettableLocalizer {
    override fun setCurrentPose(pose: Pose2d) {
        otosLocalizer.setCurrentPose(pose)
        otosPose = pose
        twoWheelPose = pose
    }

    var otosPose = Pose2d(0.0, 0.0, 0.0)
    var twoWheelPose = Pose2d(0.0, 0.0, 0.0)

    override fun update(): Twist2dDual<Time> {
        val otosTwist = otosLocalizer.update()
//        val twoWheelTwist = twoWheel.update()

        otosPose = otosPose.plus(otosTwist.value())
        twoWheelPose

        val location = otosTwist.line.plus(otosTwist.line).div(2.0)
        val rotation = otosTwist.angle.plus(otosTwist.angle).div(2.0)

        return Twist2dDual(
            location,
            rotation
        )
    }
}