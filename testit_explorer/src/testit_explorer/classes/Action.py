class Action:
    def __init__(self, index, step):
        self.index = index
        self.step = step

    def to_state(self, state):
        new_state = list(state)
        new_state[self.index] += self.step
        return tuple(new_state)

    @staticmethod
    def get_actions(steps):
        actions = []
        for index, step in enumerate(steps):
            actions.append(Action(index, step))
            if step != -step:
                actions.append(Action(index, -step))
        return actions
