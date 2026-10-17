package frc.robot.utils.enums

enum class State {
    IDLE,
    INTAKE,
    SHOOT,
    CLIMB,
}

data object GlobalState {
    var state: State = State.IDLE
}
