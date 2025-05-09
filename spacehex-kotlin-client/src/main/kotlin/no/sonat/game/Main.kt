package no.sonat.game

import no.sonat.game.geometry.LineSegment2D
import no.sonat.game.geometry.Vec2D
import org.slf4j.Logger
import org.slf4j.LoggerFactory

val logger: Logger = LoggerFactory.getLogger("Main")

/**
 * Configuration class for the lander's behavior
 */
data class LanderConfig(
    // Goal-related thresholds
    val atGoalThreshold: Double = 5.0,
    val nearGoalThreshold: Double = 20.0,
    val approachingGoalThreshold: Double = 50.0,

    // Hovering parameters
    val hoverVerticalVelocityThreshold: Double = -1.0,
    val hoverSpeedThreshold: Double = 1.5,
    val hoverHorizontalVelocityThreshold: Double = 1.0,

    // Speed control parameters
    val maxSpeed: Double = 30.0,
    val slowdownDistance: Double = 150.0,
    val nearGoalSpeedFactor: Double = 0.3,
    val veryCloseToGoalThreshold: Double = 0.1,

    // Acceleration thresholds
    val verticalAccelerationThreshold: Double = 0.3,
    val fallingTooFastThreshold: Double = -1.5,
    val bottomScreenThreshold: Double = -350.0,
    val nearGoalHorizontalThreshold: Double = 0.2,
    val farGoalHorizontalThreshold: Double = 0.5,

    // Collision detection parameters
    val baseLookAheadTime: Double = 1.0,
    val speedFactor: Double = 0.05,
    val maxLookAheadTime: Double = 2.0,
    val baseSafeDistance: Double = 10.0,
    val safeDistanceSpeedFactor: Double = 0.5,
    val closeDistanceThreshold: Double = 50.0,
    val dotProductThreshold: Double = 0.7
)

fun main() {
    logger.info("Start client")

    // Create a configuration with default values
    // You can modify these values to tweak the lander's behavior
    val config = LanderConfig(
        // Example of overriding a default value:
        // maxSpeed = 35.0,
        // slowdownDistance = 180.0,
    )

    val ag = AgentClient(
        wsUri = "ws://spacehex.norwayeast.cloudapp.azure.com:7070/test",
        //wsUri = "ws://spacehex.norwayeast.cloudapp.azure.com:7070/play",
        room = "j1ycg", //Not in use for test runs
        name = "Team kOtlin",
        strategy = { env, lander -> calculateAcceleration(env, lander, config) },
        joinAction = {
            logger.info(it)
        }
    )
}

fun calculateAcceleration(env: Environment, lander: Lander, config: LanderConfig = LanderConfig()): Acceleration {
    // If we've already reached the goal or crashed, don't do anything
    if (lander.status != LanderStatus.FLYING) {
        return Acceleration(up = false, left = false, right = false)
    }

    // Log current position, velocity, and goal
    logger.info("Position: ${lander.position}, Velocity: ${lander.velocity}, Goal: ${env.goal}")

    // Calculate vector to goal
    val toGoal = env.goal - lander.position
    val distanceToGoal = toGoal.length()
    logger.info("Distance to goal: $distanceToGoal")

    // Check if we're very close to the goal (within atGoalThreshold GU as per requirements)
    val isAtGoal = distanceToGoal < config.atGoalThreshold
    // Check if we're approaching the goal and need to start slowing down
    val isNearGoal = distanceToGoal < config.nearGoalThreshold
    // Check if we're in the general vicinity of the goal
    val isApproachingGoal = distanceToGoal < config.approachingGoalThreshold

    // Calculate current speed
    val currentSpeed = lander.velocity.length()
    logger.info("Current speed: $currentSpeed")

    // If we're at the goal, focus on hovering with speed < 2 GU/s
    if (isAtGoal) {
        logger.info("AT GOAL - Hovering")
        // If we're moving too fast, apply opposite acceleration
        val needUp = lander.velocity.y < config.hoverVerticalVelocityThreshold || 
                    (lander.velocity.y < 0 && currentSpeed > config.hoverSpeedThreshold)
        val needLeft = lander.velocity.x > config.hoverHorizontalVelocityThreshold
        val needRight = lander.velocity.x < -config.hoverHorizontalVelocityThreshold

        return Acceleration(
            up = needUp,
            left = needLeft,
            right = needRight
        )
    }

    // Calculate the desired velocity based on distance to goal
    // We want to slow down as we approach the goal
    val maxSpeed = config.maxSpeed
    val slowdownDistance = config.slowdownDistance

    // Calculate desired speed - linear decrease from maxSpeed to 0 as we approach the goal
    val desiredSpeed = when {
        distanceToGoal > slowdownDistance -> maxSpeed
        isNearGoal -> min(2.0, distanceToGoal * config.nearGoalSpeedFactor) // Very slow when near goal
        else -> maxSpeed * (distanceToGoal / slowdownDistance)
    }

    logger.info("Desired speed: $desiredSpeed")

    // Calculate desired velocity vector (direction to goal * desired speed)
    val desiredVelocity = if (distanceToGoal > config.veryCloseToGoalThreshold) {
        toGoal.unit() * desiredSpeed
    } else {
        Vec2D(0.0, 0.0) // If we're very close to the goal, aim for zero velocity
    }

    // Calculate velocity difference (what we need to change)
    val velocityDiff = desiredVelocity - lander.velocity

    // Check if we're about to hit the ground
    val groundCollisionImminent = isGroundCollisionImminent(env, lander, config)
    if (groundCollisionImminent) {
        logger.info("COLLISION IMMINENT - Avoiding ground")
    }

    // Determine acceleration based on velocity difference and collision avoidance
    val needUp = (velocityDiff.y > config.verticalAccelerationThreshold) || // Need to go up
                 groundCollisionImminent || // About to hit ground
                 (isApproachingGoal && lander.velocity.y < config.fallingTooFastThreshold) || // Approaching goal and falling too fast
                 (lander.position.y < config.bottomScreenThreshold) // Too close to bottom of screen

    // Adjust horizontal acceleration thresholds based on distance to goal
    val horizontalThreshold = if (isNearGoal) config.nearGoalHorizontalThreshold else config.farGoalHorizontalThreshold

    val needLeft = velocityDiff.x < -horizontalThreshold || 
                  (isNearGoal && lander.velocity.x > config.hoverHorizontalVelocityThreshold) // Near goal and moving right too fast

    val needRight = velocityDiff.x > horizontalThreshold || 
                   (isNearGoal && lander.velocity.x < -config.hoverHorizontalVelocityThreshold) // Near goal and moving left too fast

    // Apply acceleration
    return Acceleration(
        up = needUp,
        left = needLeft,
        right = needRight
    )
}

// Helper function to get the minimum of two doubles
private fun min(a: Double, b: Double): Double = if (a < b) a else b

/**
 * Checks if the lander is about to collide with the ground
 */
fun isGroundCollisionImminent(env: Environment, lander: Lander, config: LanderConfig = LanderConfig()): Boolean {
    // Calculate current speed
    val currentSpeed = lander.velocity.length()

    // Adjust look-ahead time based on speed (faster speed = look further ahead)
    // But cap it to avoid looking too far ahead
    val baseLookAheadTime = config.baseLookAheadTime
    val speedFactor = config.speedFactor // How much to increase look-ahead time per unit of speed
    val futureTime = minOf(baseLookAheadTime + currentSpeed * speedFactor, config.maxLookAheadTime)

    val gravity = env.constants.gravity

    // Calculate future position with gravity
    val futurePosition = Vec2D(
        lander.position.x + lander.velocity.x * futureTime,
        lander.position.y + lander.velocity.y * futureTime - 0.5 * gravity * futureTime * futureTime
    )

    // Create a line segment from current position to future position
    val trajectory = LineSegment2D(lander.position, futurePosition)

    // Check for intersection with any ground segment
    for (segment in env.segments) {
        if (trajectory.intersects(segment) != null) {
            logger.info("Collision detected: trajectory intersects with ground segment")
            return true
        }
    }

    // Also check if the future position is very close to any ground segment
    // Adjust safe distance based on speed (faster speed = need more distance)
    val baseSafeDistance = config.baseSafeDistance
    val safeDistance = baseSafeDistance + currentSpeed * config.safeDistanceSpeedFactor

    var closestDistance = Double.MAX_VALUE
    var closestSegment: LineSegment2D? = null

    for (segment in env.segments) {
        val closestPoint = segment.closestPoint(futurePosition)
        val distance = (futurePosition - closestPoint).length()

        if (distance < closestDistance) {
            closestDistance = distance
            closestSegment = segment
        }

        if (distance < safeDistance) {
            logger.info("Collision detected: future position too close to ground segment (${distance} < ${safeDistance})")
            return true
        }
    }

    // Check if we're heading toward the ground
    if (closestSegment != null && closestDistance < config.closeDistanceThreshold) {
        val velocityDirection = lander.velocity.unit()
        val toClosestPoint = (closestSegment.closestPoint(lander.position) - lander.position).unit()

        // If the dot product is positive, we're heading toward the closest point
        val dotProduct = velocityDirection.dot(toClosestPoint)
        if (dotProduct > config.dotProductThreshold) { // Heading directly toward ground
            logger.info("Collision predicted: heading toward ground (dot product: $dotProduct)")
            return true
        }
    }

    return false
}

/**
 * Returns the minimum of two values
 */
private fun minOf(a: Double, b: Double): Double = if (a < b) a else b
