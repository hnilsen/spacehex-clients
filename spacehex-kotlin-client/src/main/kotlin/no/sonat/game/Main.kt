package no.sonat.game

import no.sonat.game.geometry.LineSegment2D
import no.sonat.game.geometry.Vec2D
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import kotlin.math.abs

val logger: Logger = LoggerFactory.getLogger("Main")

// Tracking variables to maintain state between function calls
var previousAcceleration = Acceleration(up = false, left = false, right = false)
var previousVelocity = Vec2D.ZERO
var approachFromAbove = false
var accelerationHistory = mutableListOf<Acceleration>()
var velocityHistory = mutableListOf<Vec2D>()
const val HISTORY_SIZE = 5 // Number of previous states to track

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

/**
 * Calculates the acceleration to navigate the lander to the goal while avoiding obstacles.
 * 
 * @param env The environment containing the goal and obstacles
 * @param lander The current state of the lander
 * @return The acceleration to apply to the lander
 */
fun calculateAcceleration(env: Environment, lander: Lander): Acceleration {
    // If the lander has crashed or completed, don't do anything
    if (lander.status != LanderStatus.FLYING) {
        // Reset tracking variables
        previousAcceleration = Acceleration(up = false, left = false, right = false)
        previousVelocity = Vec2D.ZERO
        approachFromAbove = false
        accelerationHistory.clear()
        velocityHistory.clear()
        return Acceleration(up = false, left = false, right = false)
    }

    // Update tracking variables
    velocityHistory.add(lander.velocity)
    if (velocityHistory.size > HISTORY_SIZE) {
        velocityHistory.removeAt(0)
    }

    // Calculate vector to goal
    val toGoal = env.goal - lander.position
    val distanceToGoal = toGoal.length()

    // Calculate safe landing parameters
    val maxSafeLandingVelocity = 2.5 // Maximum safe landing velocity (2.5 GU/second)
    val approachDistance = 5.0 // Distance to start slowing down for landing (5 GU)
    val preApproachDistance = 20.0 // Distance to start positioning above the goal

    // Calculate average velocity to smooth control
    val averageVelocity = if (velocityHistory.isNotEmpty()) {
        velocityHistory.reduce { acc, vel -> acc + vel } / velocityHistory.size.toDouble()
    } else {
        lander.velocity
    }

    val currentSpeed = averageVelocity.length()

    // Determine if we should approach from above
    if (distanceToGoal < preApproachDistance && !approachFromAbove) {
        approachFromAbove = true
    }

    // Check for obstacles in the path
    val obstacleAvoidanceVector = calculateObstacleAvoidance(env, lander)

    // Determine desired direction based on approach strategy and obstacles
    val desiredDirection = if (obstacleAvoidanceVector != Vec2D.ZERO) {
        // If there's an obstacle, prioritize avoiding it with higher weight
        (toGoal.unit() + obstacleAvoidanceVector * 4.0).unit()
    } else if (approachFromAbove && lander.position.y < env.goal.y && distanceToGoal > approachDistance) {
        // If we're below the goal and not in final approach, aim above the goal
        val aboveGoal = Vec2D(env.goal.x, env.goal.y + 5.0)
        (aboveGoal - lander.position).unit()
    } else {
        // Otherwise head straight for the goal
        toGoal.unit()
    }

    // Enhanced landing logic for final approach
    val finalApproach = distanceToGoal < approachDistance
    val needToSlowDown = (distanceToGoal < preApproachDistance && currentSpeed > maxSafeLandingVelocity * 2) || 
                         (finalApproach && currentSpeed > maxSafeLandingVelocity)

    // Calculate acceleration based on desired direction and current velocity
    val up = if (finalApproach) {
        // When in final approach, use thrusters to carefully control speed
        val wasGoingUp = previousAcceleration.up
        val isGoingDown = averageVelocity.y < 0
        val isTooFast = currentSpeed > maxSafeLandingVelocity

        // Smooth control by considering previous acceleration
        if (wasGoingUp && !isGoingDown && !isTooFast) {
            // Continue going up if we were already going up and not going too fast
            true
        } else if (isGoingDown && isTooFast) {
            // Counter downward movement if going too fast
            true
        } else if (desiredDirection.y > 0 && averageVelocity.y < maxSafeLandingVelocity / 2) {
            // Gentle upward adjustment if needed
            true
        } else {
            // Otherwise, don't accelerate up
            false
        }
    } else {
        // Normal flight mode with smoother control
        (desiredDirection.y > 0 && averageVelocity.y < env.constants.landerAccelerationUp) || // Accelerate up if we need to go up
        (averageVelocity.y < -env.constants.gravity) || // Counter gravity if falling too fast
        (needToSlowDown && averageVelocity.y < 0) // Slow down for landing
    }

    // Horizontal control logic with more aggressive movement
    val left = if (finalApproach) {
        // When in final approach, be more conservative with horizontal movement
        val wasGoingLeft = previousAcceleration.left
        val isGoingRight = averageVelocity.x > 0
        val isTooFast = abs(averageVelocity.x) > maxSafeLandingVelocity / 2

        if (wasGoingLeft && !isGoingRight && !isTooFast) {
            // Continue going left if we were already going left and not going too fast
            true
        } else {
            // Otherwise, only go left if needed and not too fast
            desiredDirection.x < 0 && averageVelocity.x > -maxSafeLandingVelocity / 2
        }
    } else {
        // Normal flight mode - more aggressive horizontal movement
        // Allow higher horizontal velocity (2x the acceleration constant) for better obstacle avoidance
        desiredDirection.x < 0 && (
            averageVelocity.x > -env.constants.landerAccelerationLeft * 2 || // Allow higher speed for obstacle avoidance
            obstacleAvoidanceVector.x < -0.3 // Force left if obstacle avoidance strongly suggests it
        )
    }

    val right = if (finalApproach) {
        // When in final approach, be more conservative with horizontal movement
        val wasGoingRight = previousAcceleration.right
        val isGoingLeft = averageVelocity.x < 0
        val isTooFast = abs(averageVelocity.x) > maxSafeLandingVelocity / 2

        if (wasGoingRight && !isGoingLeft && !isTooFast) {
            // Continue going right if we were already going right and not going too fast
            true
        } else {
            // Otherwise, only go right if needed and not too fast
            desiredDirection.x > 0 && averageVelocity.x < maxSafeLandingVelocity / 2
        }
    } else {
        // Normal flight mode - more aggressive horizontal movement
        // Allow higher horizontal velocity (2x the acceleration constant) for better obstacle avoidance
        desiredDirection.x > 0 && (
            averageVelocity.x < env.constants.landerAccelerationRight * 2 || // Allow higher speed for obstacle avoidance
            obstacleAvoidanceVector.x > 0.3 // Force right if obstacle avoidance strongly suggests it
        )
    }

    // Create and store the new acceleration
    val acceleration = Acceleration(up = up, left = left, right = right)
    accelerationHistory.add(acceleration)
    if (accelerationHistory.size > HISTORY_SIZE) {
        accelerationHistory.removeAt(0)
    }

    // Update previous values for next call
    previousAcceleration = acceleration
    previousVelocity = lander.velocity

    return acceleration
}

/**
 * Calculates a vector to help avoid obstacles.
 * 
 * @param env The environment containing obstacles
 * @param lander The current state of the lander
 * @return A vector pointing away from nearby obstacles
 */
fun calculateObstacleAvoidance(env: Environment, lander: Lander): Vec2D {
    val position = lander.position
    val velocity = lander.velocity

    // Look ahead based on current velocity to predict future position
    // Use a minimum lookAheadDistance to ensure we detect obstacles even when moving slowly
    val minLookAheadDistance = 10.0
    val velocityBasedDistance = velocity.length() * 3.0
    val lookAheadDistance = maxOf(minLookAheadDistance, velocityBasedDistance)

    // Consider gravity in our prediction by adding a downward component
    val gravityAdjustedVelocity = velocity + Vec2D(0.0, -env.constants.gravity * 0.5)
    val predictedPath = LineSegment2D(
        position, 
        position + gravityAdjustedVelocity.unit() * lookAheadDistance
    )

    // Add a second prediction path directly below for ground detection
    val groundDetectionPath = LineSegment2D(
        position,
        position + Vec2D(0.0, -minLookAheadDistance)
    )

    // Check for potential collisions with ground segments
    var closestIntersection: Vec2D? = null
    var minDistance = Double.MAX_VALUE

    // Check both the velocity-based path and the ground detection path
    val pathsToCheck = listOf(predictedPath, groundDetectionPath)

    for (path in pathsToCheck) {
        for (segment in env.segments) {
            val intersection = path.intersects(segment)
            if (intersection != null) {
                val distance = (intersection - position).length()
                if (distance < minDistance) {
                    minDistance = distance
                    closestIntersection = intersection
                }
            }
        }
    }

    // If we found a potential collision, calculate avoidance vector
    if (closestIntersection != null) {
        // Find the closest segment to our position
        var closestSegment: LineSegment2D? = null
        minDistance = Double.MAX_VALUE

        for (segment in env.segments) {
            val closestPoint = segment.closestPoint(position)
            val distance = (closestPoint - position).length()
            if (distance < minDistance) {
                minDistance = distance
                closestSegment = segment
            }
        }

        // Calculate normal vector to the closest segment (pointing away from it)
        if (closestSegment != null) {
            val segmentDirection = closestSegment.direction()
            val normal = segmentDirection.normalVector()

            // Make sure the normal points away from the segment
            val toPosition = position - closestSegment.closestPoint(position)
            val dotProduct = normal.dot(toPosition)

            // Get the basic avoidance vector
            val basicAvoidance = if (dotProduct >= 0) normal else -normal

            // If the segment is below us (ground), add a strong upward component
            // to ensure we prioritize moving up to avoid ground collisions
            if (closestSegment.closestPoint(position).y < position.y) {
                return (basicAvoidance + Vec2D(0.0, 2.0)).unit()
            }

            return basicAvoidance
        }
    }

    return Vec2D.ZERO // No obstacles detected
}
