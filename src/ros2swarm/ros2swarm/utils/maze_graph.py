from ros2swarm.utils.state import State


class MazeNode:
    def __init__(self):
        self.parent: MazeNode = None
        self.left: MazeNode = None
        self.right: MazeNode = None
        self.isLeaf: bool = False
        self.visited: int = 0
        self.isGoal: bool = False
        self.isRoot: bool = False
        self.level: int = -1
        self.node_id: int = -1


class MazeGraph:
    def __init__(self):

        self.rootNode = MazeNode()
        self.rootNode.isRoot = True
        self.rootNode.level = 0
        self.rootNode.node_id = 0

        self.central_crossing = MazeNode()
        self.central_crossing.parent = self.rootNode
        self.central_crossing.level = 1
        self.central_crossing.node_id = 1

        self.lastLocation = self.rootNode
        self.nextLocation = self.central_crossing

        self.build_tree(remaining_levels=3, parent=self.central_crossing, level=2, parent_node_id=1)

    def calc_tube_last_visited(self):
        """
        Returns the  (State.CROSSING_LEFT, State.CROSSING_RIGHT) if the left tube was more recently visited
        and (State.CROSSING_RIGHT, State.CROSSING_LEFT) otherwise.

        :raises AssertionError: if parent, left or right of the next location is not defined
        """

        assert self.nextLocation.parent is not None
        assert self.nextLocation.left is not None
        assert self.nextLocation.right is not None

        if self.lastLocation.level < self.nextLocation.level:
            # going down the tree
            left_node = self.nextLocation.left
            right_node = self.nextLocation.right
        else:
            # up the tree
            left_node = self.nextLocation.parent
            right_node = self.nextLocation.left

        if left_node.visited >= right_node.visited:
            last_visited = (State.CROSSING_LEFT, State.CROSSING_RIGHT)
        else:
            last_visited = (State.CROSSING_RIGHT, State.CROSSING_LEFT)

        return last_visited

    def calc_tube_less_explored_recursive(self, last_node: MazeNode, next_node: MazeNode):
        """
        Counts the number of unexplored endings recursively.
        """

        if next_node.isLeaf:
            num_endings = 1
        elif next_node.isRoot:
            num_endings = 0
        else:
            # TODO find up or down and respectively up/down
            if last_node.level < next_node.level:
                # going down the tree
                left = next_node.left
                right = next_node.right
            else:
                # up the tree
                # TODO this only works for right endings if next_node
                if last_node == next_node.left:
                    # coming from left tunnel
                    left = next_node.right
                    right = next_node.parent
                else:
                    # coming from right tunnel
                    left = next_node.parent
                    right = next_node.left
                # TODO does this work with root?
            num_endings = self.calc_tube_less_explored_recursive(next_node, left) \
                          + self.calc_tube_less_explored_recursive(next_node, right)
        return num_endings

    def calc_tube_less_explored(self):
        """
        Calculate the number of unvisited endpoints in the left / right tube returning
        (State.CROSSING_LEFT, State.CROSSING_RIGHT) if the left tube has more unvisited endpoints and
        (State.CROSSING_RIGHT, State.CROSSING_LEFT) otherwise.
        """
        if self.lastLocation.level < self.nextLocation.level:
            # going down the tree
            left_node = self.nextLocation.left
            right_node = self.nextLocation.right
        else:
            # up the tree
            if self.lastLocation == self.nextLocation.left:
                # coming from left tunnel
                # next_node = self.nextLocation.right
                left_node = self.nextLocation.right
                right_node = self.nextLocation.parent
            else:
                # coming from right tunnel
                # next_node = self.nextLocation.parent
                left_node = self.nextLocation.parent
                right_node = self.nextLocation.left

        # calc all unvisited endpoints in that direction
        sum_left = self.calc_tube_less_explored_recursive(self.lastLocation, left_node)
        sum_right = self.calc_tube_less_explored_recursive(self.lastLocation, right_node)

        if sum_left > sum_right:
            unexplored = (State.CROSSING_LEFT, State.CROSSING_RIGHT)
        else:
            unexplored = (State.CROSSING_RIGHT, State.CROSSING_LEFT)

        return unexplored

    def calc_next_node(self, state: State):
        """
        Returns the next node depending on the current location in the graph
        and the direction indicated by the given state.
        State.CROSSING_LEFT -> move left
        else -> move right
        """
        if self.lastLocation.level < self.nextLocation.level:
            # TODO does this work add root level? and central crossing?
            # going down the tree
            if state == State.CROSSING_LEFT:
                next_node = self.nextLocation.left
            else:
                next_node = self.nextLocation.right
        else:
            # going up the tree
            # TODO write test to ensure this calculation works as expected for all directions

            if state == State.CROSSING_LEFT:
                if self.lastLocation == self.nextLocation.left:
                    # coming from left tunnel
                    next_node = self.nextLocation.right
                else:
                    # coming from right tunnel
                    next_node = self.nextLocation.parent
            else:
                # turn right
                if self.lastLocation == self.nextLocation.left:
                    # coming from left tunnel
                    next_node = self.nextLocation.parent
                else:
                    # coming from right tunnel
                    next_node = self.nextLocation.left

        return next_node

    def update(self, state: State, visit: int):
        """
        Update location in the graph depending on the state by setting the next location

        State -> Next Location
        ------
        State.CROSSING_RIGHT -> right node
        State.CROSSING_RIGHT -> left node
        State.ENDING -> last location
        State.START_CHAMBER  -> central crossing
        """

        self.nextLocation.visited = visit

        next_loc = None
        if state == State.CROSSING_RIGHT or state == State.CROSSING_LEFT:
            next_loc = self.calc_next_node(state)
        elif state == State.ENDING:
            next_loc = self.lastLocation
        elif state == State.START_CHAMBER:
            next_loc = self.central_crossing

        if next_loc is not None:
            self.lastLocation = self.nextLocation
            self.nextLocation = next_loc

    def is_next_location_leaf(self):
        """
        Returns True if the next location is a leaf.
        """
        return self.nextLocation.isLeaf

    def mark_as_goal(self):
        """
        Mark the node stored in next location as goal.
        """
        self.nextLocation.isGoal = True

    def __str__(self):
        """
        Returns a formatted string of the graphs tree structure.
        """

        last_node = '>>LAST<<' if self.rootNode == self.lastLocation else ''
        next_node = '>>NEXT<<' if self.rootNode == self.nextLocation else ''

        tree = '\ntree: [root, visited:' + str(self.rootNode.visited) + ']' + last_node + next_node + '\n'
        tree = tree + self.get_graph_representation(0, self.central_crossing)
        return tree

    def get_graph_representation(self, depth: int, node: MazeNode):
        """
        Recursively builds a string representation of the graph
        """
        last_node = '>>LAST<<' if node == self.lastLocation else ''
        next_node = '>>NEXT<<' if node == self.nextLocation else ''

        s = '>'
        sep = ''
        for i in range(0, depth):
            sep = sep + ' '

        info = sep + s + '[level:' + str(node.level) + ' visited:' + str(node.visited) + ' leaf:' + str(
            node.isLeaf) + ' goal: ' + str(node.isGoal) + ' node_id: ' + str(
            node.node_id) + ']' + next_node + last_node + '\n'

        if node.isLeaf:
            lines = info
        else:
            lines = info + self.get_graph_representation(depth + 1, node.left) \
                    + self.get_graph_representation(depth + 1, node.right)
        return lines

    def build_tree(self, remaining_levels: int, parent: MazeNode, level: int, parent_node_id: int):
        """
        Build recursively the default tree
        param: remaining_levels full node levels before adding leaf's to all paths
        """
        parent.left = MazeNode()
        parent.left.parent = parent
        parent.left.level = level

        parent.right = MazeNode()
        parent.right.parent = parent
        parent.right.level = level

        node_id = parent_node_id + 1
        if remaining_levels == 0:
            parent.left.isLeaf = True
            parent.right.isLeaf = True

            parent.left.node_id = node_id
            node_id += 1
            parent.right.node_id = node_id

        else:
            parent.left.node_id = node_id
            node_id = self.build_tree(remaining_levels - 1, parent.left, level + 1, node_id)

            node_id += 1
            parent.right.node_id = node_id
            node_id = self.build_tree(remaining_levels - 1, parent.right, level + 1, node_id)

        return node_id
