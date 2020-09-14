class BaseGenerator():
    def __init__(self, _libraries, _config, _states, _globalNamespace):
        self.libraries = _libraries
        self.config = _config
        self.states = _states
        self.globalNamespace = _globalNamespace

    def getAllStates(self):
        addedStates = {}
        allStates = []
        for state in self.states:
            if state.id not in addedStates:
                addedStates[state.id] = state
                allStates.append(state)

            for childState in state.getChildren():
                if childState.id not in addedStates:
                    addedStates[childState.id] = childState
                    allStates.append(childState)

        return allStates

    def getAllTransitions(self):
        addedTransitions = {}
        transitions = []
        for state in self.states:
            for tran in state.getOriginTransitions():
                if tran.id not in addedTransitions:
                    addedTransitions[tran.id] = tran
                    transitions.append(tran)
            for childState in state.getChildren():
                for tran in childState.getOriginTransitions():
                    if tran.id not in addedTransitions:
                        addedTransitions[tran.id] = tran
                        transitions.append(tran)

        return transitions

    def getStateById(self, id):
        for state in self.states:
            if state.id == id:
                return state
        return None

    def getRootState(self):
        for state in self.states:
            if state.parent is None:
                return state

    # def getAllNamespaces(self):
    #     allNamespaces = []
    #     for state in self.getAllStates():
    #         allNamespaces.append(state.getNamespace())
    #     return allNamespaces
