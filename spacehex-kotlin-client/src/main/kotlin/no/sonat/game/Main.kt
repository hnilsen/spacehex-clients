package no.sonat.game

import no.sonat.game.geometry.LineSegment2D
import no.sonat.game.geometry.Vec2D
import org.slf4j.Logger
import org.slf4j.LoggerFactory

val logger: Logger = LoggerFactory.getLogger("Main")

fun main() {
    logger.info("Start client")
    val ag = AgentClient(
        wsUri = "ws://spacehex.norwayeast.cloudapp.azure.com:7070/test",
        //wsUri = "ws://spacehex.norwayeast.cloudapp.azure.com:7070/play",
        room = "j1ycg", //Not in use for test runs
        name = "Team kOtlin",
        strategy = ::calculateAcceleration,
        joinAction = {
            logger.info(it)
        }
    )
}

fun calculateAcceleration(env: Environment, lander: Lander): Acceleration {
    // If the lander has crashed or completed, no need to calculate acceleration
    if (lander.status != LanderStatus.FLYING) {
        return Acceleration(up = false, left = false, right = false)
    }

    // Calculate distance to goal
    val distanceToGoal = (lander.position - env.goal).length()

    // Calculate direction to goal
    val directionToGoal = (env.goal - lander.position).unit()

    // Calculate current speed
    val currentSpeed = lander.velocity.length()

    // Check if we're close to the goal
    val isCloseToGoal = distanceToGoal < 50.0
    val isVeryCloseToGoal = distanceToGoal < 10.0
    val isExtremelyCloseToGoal = distanceToGoal < 5.0  // Changed to 5.0 to match the requirement

    // Log current status for debugging
    logger.info("Distance to goal: $distanceToGoal, Speed: $currentSpeed, Position: ${lander.position}, Velocity: ${lander.velocity}")

    // Calculate desired velocity based on distance to goal
    // We want to slow down as we approach the goal but move faster when far away
    val maxSpeed = when {
        isExtremelyCloseToGoal -> 1.0 // Very slow when extremely close
        isVeryCloseToGoal -> 1.5 // Slower than the 2.0 requirement to give some margin
        isCloseToGoal -> 3.0 + distanceToGoal / 8.0 // Gradually slow down, but slightly faster
        distanceToGoal < 150.0 -> 15.0 + distanceToGoal / 10.0 // Increased medium speed
        else -> 40.0 // Increased maximum speed when far from goal
    }

    // Calculate desired velocity vector
    val desiredVelocity = directionToGoal * maxSpeed

    // Calculate velocity error (difference between current and desired velocity)
    val velocityError = desiredVelocity - lander.velocity

    // Check for potential collisions with ground segments
    // Adjust safety distance based on velocity - faster movement needs more safety margin
    val baseSafetyDistance = 30.0
    val velocityFactor = currentSpeed * 0.5 // Higher speed = larger safety distance
    val safetyDistance = baseSafetyDistance + velocityFactor

    var closestGroundDistance = Double.MAX_VALUE
    var closestGroundNormal = Vec2D(0.0, 1.0) // Default to upward direction
    var closestSegment: LineSegment2D? = null

    for (segment in env.segments) {
        val closestPoint = segment.closestPoint(lander.position)
        val distance = (lander.position - closestPoint).length()

        if (distance < closestGroundDistance) {
            closestGroundDistance = distance
            closestSegment = segment
            // Calculate normal vector pointing away from the segment
            val segmentVector = segment.vector()
            closestGroundNormal = segmentVector.normalVector().unit()
            // Ensure the normal points away from the lander
            if (closestGroundNormal.dot(lander.position - closestPoint) < 0) {
                closestGroundNormal = -closestGroundNormal
            }
        }
    }

    // Calculate relative velocity towards the ground
    val relativeVelocityTowardsGround = if (closestSegment != null) {
        // Get the closest point on the segment to the lander
        val closestPoint = closestSegment.closestPoint(lander.position)
        // Calculate direction from lander to ground
        val directionToGround = (closestPoint - lander.position)
        // Normalize to get unit vector
        val directionToGroundUnit = if (directionToGround.length() > 0) {
            directionToGround.div(directionToGround.length())
        } else {
            Vec2D(0.0, 0.0)
        }
        // Calculate component of velocity in direction of ground
        lander.velocity.dot(directionToGroundUnit)
    } else {
        0.0
    }

    // If we're moving towards the ground, increase avoidance factor
    val movingTowardsGround = relativeVelocityTowardsGround > 0

    // If we're too close to the ground, prioritize moving away from it
    val avoidGroundFactor = when {
        closestGroundDistance < safetyDistance * 0.5 -> 0.9 // Very close - strong avoidance
        closestGroundDistance < safetyDistance && movingTowardsGround -> 
            (safetyDistance - closestGroundDistance) / safetyDistance * 1.5 // Moving towards ground - enhanced avoidance
        closestGroundDistance < safetyDistance -> 
            (safetyDistance - closestGroundDistance) / safetyDistance // Standard avoidance
        else -> 0.0 // Far enough - no avoidance needed
    }

    // Combine goal-seeking and ground-avoidance
    val combinedDirection = if (avoidGroundFactor > 0.0) {
        (directionToGoal * (1.0 - avoidGroundFactor) + closestGroundNormal * avoidGroundFactor).unit()
    } else {
        directionToGoal
    }

    // Apply precise hover control when close to goal and not moving too fast
    val hoverMode = (isVeryCloseToGoal && currentSpeed < 4.0) || isExtremelyCloseToGoal

    // Calculate gravity compensation factor
    // Full compensation when hovering, partial when moving
    val gravityCompFactor = when {
        hoverMode -> 2.0 // Overcompensate when in hover mode to ensure stable hovering
        isExtremelyCloseToGoal -> 1.5 // Full compensation when extremely close
        isVeryCloseToGoal -> 1.0 // Almost full compensation when very close
        isCloseToGoal -> 1.0 // Increased partial compensation when close
        else -> 1.0 // Increased minimal compensation when far
    }

    // Gravity compensation threshold - apply upward thrust when falling faster than this
    val gravityThreshold = -env.constants.gravity * gravityCompFactor

    // Determine acceleration based on the combined direction and velocity error
    val needUpAcceleration = velocityError.y > 0.5 || // Need to go up faster
                            (lander.velocity.y < gravityThreshold) || // Falling too fast
                            (avoidGroundFactor > 0.3 && closestGroundNormal.y > 0.3) // Need to avoid ground

    val needLeftAcceleration = velocityError.x < -0.5 || // Need to go left faster
                              (lander.velocity.x > 0.5 && isCloseToGoal && directionToGoal.x < -0.2) // Moving right but need to go left

    val needRightAcceleration = velocityError.x > 0.5 || // Need to go right faster
                               (lander.velocity.x < -0.5 && isCloseToGoal && directionToGoal.x > 0.2) // Moving left but need to go right

    // In hover mode, apply more precise control
    val (up, left, right) = if (hoverMode) {
        // Precise hover control - more aggressive to maintain position
        // Apply upward thrust more aggressively to counteract gravity
        val preciseUp = lander.velocity.y < 1.0 || 
                        (isExtremelyCloseToGoal && lander.position.y < env.goal.y) ||
                        (lander.position.y < env.goal.y - 1.0)

        // More aggressive horizontal control
        val preciseLeft = lander.velocity.x > 0.1 || 
                         (lander.position.x > env.goal.x + 0.5) // More sensitive to position

        val preciseRight = lander.velocity.x < -0.1 || 
                          (lander.position.x < env.goal.x - 0.5) // More sensitive to position

        Triple(preciseUp, preciseLeft, preciseRight)
    } else {
        // Normal flight control
        Triple(needUpAcceleration, needLeftAcceleration, needRightAcceleration)
    }

    return Acceleration(
        up = up,
        left = left,
        right = right
    )
}
