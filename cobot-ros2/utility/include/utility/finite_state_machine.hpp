#ifndef FINITE_STATE_MACHINE_H_
#define FINITE_STATE_MACHINE_H_

#include <iostream>
#include <functional>
#include <unordered_map>

enum class State
{
    UNINITIALIZED,
    INITIALIZING,
    POWERED_OFF,
    POWERING_ON,
    IDLE,
    RUNNING,
    STOPPING,
    COMMUNICATION_LOSS,
    ERROR,
    ERROR_RECOVERY,
    ESTOP,
    SHUTDOWN
};

enum class Event
{
    INIT,
    INIT_DONE,
    POWER_ON,
    POWER_OFF,
    START,
    STOP,
    FAULT,
    RECOVER,
    RECOVERY_DONE,
    COMM_LOSS,
    COMM_RESTORED,
    ESTOP_TRIGGERED,
    ESTOP_RELEASED,
    SHUTDOWN
};

class RobotFSM
{
public:
    using Key = std::pair<State, Event>;

    struct KeyHash
    {
        std::size_t operator()(const Key &k) const
        {
            return std::hash<int>()(static_cast<int>(k.first)) ^
                   std::hash<int>()(static_cast<int>(k.second));
        }
    };

    struct Transition
    {
        State next_state;
    };

    RobotFSM()
    {
        current_state_ = State::UNINITIALIZED;
        setupTransitions();
    }

    State getState() const { return current_state_; }

    void handleEvent(Event event)
    {
        Key key{current_state_, event};

        auto it = transitions_.find(key);
        if (it == transitions_.end())
            return;

        const Transition &t = it->second;


        transitionTo(t.next_state);
    }

private:
    State current_state_;

    std::unordered_map<Key, Transition, KeyHash> transitions_;

    // ---------- Transition Setup ----------
    void setupTransitions()
    {
        add(State::UNINITIALIZED, Event::INIT, State::INITIALIZING);

        add(State::INITIALIZING, Event::INIT_DONE, State::POWERED_OFF);
        add(State::INITIALIZING, Event::FAULT, State::ERROR);

        add(State::POWERED_OFF, Event::POWER_ON, State::POWERING_ON);

        add(State::POWERING_ON, Event::START, State::IDLE);
        add(State::POWERING_ON, Event::FAULT, State::ERROR);

        add(State::IDLE, Event::START, State::RUNNING);
        add(State::IDLE, Event::POWER_OFF, State::POWERED_OFF);

        add(State::RUNNING, Event::STOP, State::STOPPING);
        add(State::RUNNING, Event::FAULT, State::ERROR);
        add(State::RUNNING, Event::COMM_LOSS, State::COMMUNICATION_LOSS);

        add(State::STOPPING, Event::STOP, State::IDLE);

        add(State::COMMUNICATION_LOSS, Event::COMM_RESTORED, State::IDLE);
        add(State::COMMUNICATION_LOSS, Event::FAULT, State::ERROR);

        add(State::ERROR, Event::RECOVER, State::ERROR_RECOVERY);

        add(State::ERROR_RECOVERY, Event::RECOVERY_DONE, State::IDLE);
        add(State::ERROR_RECOVERY, Event::FAULT, State::ERROR);

        add(State::IDLE, Event::ESTOP_TRIGGERED, State::ESTOP);
        add(State::RUNNING, Event::ESTOP_TRIGGERED, State::ESTOP);

        add(State::ESTOP, Event::ESTOP_RELEASED, State::IDLE);

        add(State::IDLE, Event::SHUTDOWN, State::SHUTDOWN);
    }

    // ---------- Add helper ----------
    void add(State from, Event event, State to)
    {
        transitions_[{from, event}] = {to};
    }

    // ---------- Transition ----------
    void transitionTo(State new_state)
    {
        if (current_state_ == new_state)
            return;

        onExit(current_state_);

        current_state_ = new_state;

        onEnter(current_state_);
    }

    // ---------- Hooks ----------
    void onEnter(State s)
    {
        // Example:
        // log, enable motors, reset flags, etc.
    }

    void onExit(State s)
    {
        // cleanup if needed
    }
};

#endif