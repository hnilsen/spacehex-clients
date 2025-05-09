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

    // If we're at the goal, focus on hovering with speed < 2 GU/s
    if (isAtGoal) {
        logger.info("AT GOAL - Hovering")
        // If we're moving too fast, apply opposite acceleration
        val needUp = lander.velocity.y < config.hoverVerticalVelocityThreshold || 
                    (lander.velocity.y < 0 && lander.velocity.length() > config.hoverSpeedThreshold)
        val needLeft = lander.velocity.x > config.hoverHorizontalVelocityThreshold
        val needRight = lander.velocity.x < -config.hoverHorizontalVelocityThreshold

        return Acceleration(
            up = needUp,
            left = needLeft,
            right = needRight
        )
    }

    // Use the TerrainNavigator to find a safe path and calculate acceleration
    val terrainNavigator = TerrainNavigator(env, config)

    // Find a safe path to the goal
    val path = terrainNavigator.findPath(lander.position, lander.velocity)
    logger.info("Path to goal: $path")

    // Calculate acceleration to follow the path
    return terrainNavigator.calculatePathAcceleration(lander, path)
}

// Helper function to get the minimum of two doubles
private fun min(a: Double, b: Double): Double = if (a < b) a else b

// Note: The isGroundCollisionImminent function has been moved to TerrainNavigator class
