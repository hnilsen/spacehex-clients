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
    // Constants for our algorithm
    val distanceToSlowDown = 100.0 // Distance at which we start slowing down
    val hoverDistance = 5.0 // Distance to goal to be considered "at goal" (as per requirements)
    val maxSpeed = 2.0 // Maximum speed allowed at goal (as per requirements)
    val safetyDistance = 30.0 // Minimum distance to keep from ground

    // Calculate distance to goal
    val distanceToGoal = (env.goal - lander.position).length()

    // Calculate current speed
    val currentSpeed = lander.velocity.length()

    // Check if we're at the goal
    if (distanceToGoal <= hoverDistance && currentSpeed <= maxSpeed) {
        // We're at the goal and moving slowly enough, hover
        return calculateHoverAcceleration(env, lander)
    }

    // Check if we're approaching the goal and need to slow down
    if (distanceToGoal <= distanceToSlowDown) {
        // We're close to the goal, slow down
        return calculateSlowDownAcceleration(env, lander)
    }

    // We're far from the goal, navigate towards it while avoiding obstacles
    return calculateNavigationAcceleration(env, lander, safetyDistance)
}

/**
 * Calculate acceleration to hover at the goal
 */
fun calculateHoverAcceleration(env: Environment, lander: Lander): Acceleration {
    // To hover, we need to counteract gravity
    // If we're falling, accelerate up
    // If we're moving horizontally, counteract that movement

    val up = lander.velocity.y < 0 || lander.position.y < env.goal.y
    val left = lander.velocity.x > 0 || lander.position.x > env.goal.x
    val right = lander.velocity.x < 0 || lander.position.x < env.goal.x

    return Acceleration(up = up, left = left, right = right)
}

/**
 * Calculate acceleration to slow down when approaching the goal
 */
fun calculateSlowDownAcceleration(env: Environment, lander: Lander): Acceleration {
    // Calculate direction to goal
    val directionToGoal = (env.goal - lander.position).unit()

    // Calculate dot product to see if we're moving towards or away from the goal
    val dotProduct = directionToGoal.dot(lander.velocity.unit())

    // If dot product is positive, we're moving towards the goal
    // If dot product is negative, we're moving away from the goal

    // Calculate acceleration to slow down
    val up = lander.velocity.y < 0 || (lander.velocity.y > 2.0 && lander.position.y > env.goal.y)
    val left = lander.velocity.x > 2.0 || (dotProduct < 0 && lander.position.x > env.goal.x)
    val right = lander.velocity.x < -2.0 || (dotProduct < 0 && lander.position.x < env.goal.x)

    return Acceleration(up = up, left = left, right = right)
}

/**
 * Calculate acceleration for navigating towards the goal while avoiding obstacles
 */
fun calculateNavigationAcceleration(env: Environment, lander: Lander, safetyDistance: Double): Acceleration {
    // Calculate direction to goal
    val directionToGoal = (env.goal - lander.position).unit()

    // Check for potential collisions with ground segments
    val potentialCollision = checkForPotentialCollision(env, lander, safetyDistance)

    if (potentialCollision) {
        // If there's a potential collision, navigate away from the ground
        return calculateAvoidanceAcceleration(env, lander, safetyDistance)
    }

    // No potential collision, navigate towards the goal
    val up = directionToGoal.y > 0 || lander.velocity.y < 0
    val left = directionToGoal.x < 0 && lander.velocity.x > -5.0
    val right = directionToGoal.x > 0 && lander.velocity.x < 5.0

    return Acceleration(up = up, left = left, right = right)
}

/**
 * Check if there's a potential collision with the ground
 */
fun checkForPotentialCollision(env: Environment, lander: Lander, safetyDistance: Double): Boolean {
    // Project the lander's position forward based on its velocity
    val projectedPosition = lander.position + lander.velocity

    // Check distance to each ground segment
    for (segment in env.segments) {
        val closestPoint = segment.closestPoint(lander.position)
        val distance = (closestPoint - lander.position).length()

        if (distance < safetyDistance) {
            return true
        }

        // Also check the projected position
        val projectedClosestPoint = segment.closestPoint(projectedPosition)
        val projectedDistance = (projectedClosestPoint - projectedPosition).length()

        if (projectedDistance < safetyDistance) {
            return true
        }
    }

    return false
}

/**
 * Calculate acceleration to avoid collision with the ground
 */
fun calculateAvoidanceAcceleration(env: Environment, lander: Lander, safetyDistance: Double): Acceleration {
    // Find the closest ground segment
    var closestSegment: LineSegment2D? = null
    var minDistance = Double.MAX_VALUE

    for (segment in env.segments) {
        val closestPoint = segment.closestPoint(lander.position)
        val distance = (closestPoint - lander.position).length()

        if (distance < minDistance) {
            minDistance = distance
            closestSegment = segment
        }
    }

    // If we found a close segment, move away from it
    if (closestSegment != null) {
        val closestPoint = closestSegment.closestPoint(lander.position)
        val avoidanceDirection = (lander.position - closestPoint).unit()

        // Calculate acceleration to move away from the ground
        val up = avoidanceDirection.y > 0 || lander.velocity.y < 0
        val left = avoidanceDirection.x < 0 && lander.velocity.x > 0
        val right = avoidanceDirection.x > 0 && lander.velocity.x < 0

        return Acceleration(up = up, left = left, right = right)
    }

    // Default to moving up if we can't determine a better direction
    return Acceleration(up = true, left = false, right = false)
}
