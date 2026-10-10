package frc.robot.utils.enums

enum class State {
    IDLE,
    STOW,
    INTAKE,
    OUTTAKE,
    FEED,
    SPINUP,
    INDEX,
    SHOOT,
}

data object GlobalState {
    var state: State = State.IDLE
}
