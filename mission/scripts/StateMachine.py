from abc import ABC, abstractmethod

class State(ABC):
    IDLE = 0
    RUNNING = 1
    FINISHED = 2

    def __init__(self):
        self._status = State.IDLE

    def start(self):
        self._status = State.RUNNING

    @abstractmethod
    def run(self):
        pass

    def end(self):
        self._status = State.FINISHED
    
    def is_idle(self):
        return self._status == State.IDLE

    def is_running(self):
        return self._status == State.RUNNING
    
    def is_finished(self):
        return self._status == State.FINISHED

class SequentialStateGroup(State):
    def __init__(self, states):
        self._states = states
        self._current_state_idx = 0
        self._current_state = states[0]

    def start(self):
        super().start()
        self._current_state.start()

    def run(self):
        if self._current_state.is_idle():
            self._current_state.start()
        elif self._current_state.is_running():
            self._current_state.run()
        elif self._current_state.is_stopped():
            self._current_state_idx += 1
            self._current_state = self._states[self._current_state_idx]
            if self._current_state_idx == len(self._states) - 1:
                self.end()

    def end(self):
        super().end()

class ParallelStateGroup(State):
    def __init__(self, states):
        self._states = states

    def start(self):
        super().start()
        for state in self._states:
            state.start()

    def run(self):
        finished = True
        for state in self._states:
            if state.is_running():
                finished = False
                state.run()
        if finished:
            self.end()

    def end(self):
        super().end()
