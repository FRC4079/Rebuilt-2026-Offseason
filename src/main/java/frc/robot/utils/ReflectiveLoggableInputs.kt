package frc.robot.utils

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.wpilib.math.geometry.Rotation2d
import kotlin.reflect.KMutableProperty1
import kotlin.reflect.full.memberProperties

abstract class ReflectiveLoggableInputs : LoggableInputs {
    // Reflection lookups are cached per-class, not re-done every call
    private val properties: List<KMutableProperty1<Any, Any?>> by lazy {
        @Suppress("UNCHECKED_CAST")
        this::class
            .memberProperties
            .filterIsInstance<KMutableProperty1<Any, Any?>>()
    }

    override fun toLog(table: LogTable) {
        for (prop in properties) {
            val key = prop.name.replaceFirstChar { it.uppercase() }
            when (val value = prop.get(this)) {
                is Boolean -> table.put(key, value)

                is Int -> table.put(key, value)

                is Long -> table.put(key, value)

                is Float -> table.put(key, value)

                is Double -> table.put(key, value)

                is String -> table.put(key, value)

                is Rotation2d -> table.put(key, Rotation2d.struct, value)

                // add more types here as you need them
                else -> Unit // unsupported type: silently skipped
            }
        }
    }

    override fun fromLog(table: LogTable) {
        for (prop in properties) {
            val key = prop.name.replaceFirstChar { it.uppercase() }
            val current = prop.get(this)
            val updated: Any? =
                when (current) {
                    is Boolean -> table.get(key, current)
                    is Int -> table.get(key, current)
                    is Long -> table.get(key, current)
                    is Float -> table.get(key, current)
                    is Double -> table.get(key, current)
                    is String -> table.get(key, current)
                    is Rotation2d -> table.get(key, Rotation2d.struct, current)
                    else -> current
                }
            prop.set(this, updated)
        }
    }
}
