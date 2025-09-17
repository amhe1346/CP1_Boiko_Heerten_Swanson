import numpy as np
import random
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Define the grid size
GRID_SIZE = 5

# Robot types
ROBOT_TYPES = ['quadrotor', 'humanoid', 'differential_drive']

class Robot:
    def __init__(self, robot_id, robot_type, start, goal):
        self.id = robot_id   # Unique identifier for the robot
        self.type = robot_type
        self.position = start
        self.goal = goal

    def move(self, grid):
        if self.position == self.goal:
            return  # Already at the goal

        # Calculate the shortest path (greedy move)
        dx = self.goal[0] - self.position[0]
        dy = self.goal[1] - self.position[1]

        if abs(dx) > abs(dy):
            primary_move = (self.position[0] + np.sign(dx), self.position[1])
            secondary_move = (self.position[0], self.position[1] + np.sign(dy))
        else:
            primary_move = (self.position[0], self.position[1] + np.sign(dy))
            secondary_move = (self.position[0] + np.sign(dx), self.position[1])

        # Attempt primary move first, then fallback to secondary move
        if grid.can_move(self, primary_move):
            new_position = primary_move
        elif grid.can_move(self, secondary_move):
            new_position = secondary_move
        else:
            new_position = self.position  # No valid move

        # Check movement rules
        if grid.can_move(self, new_position):
            self.position = new_position

class Grid:
    def __init__(self, size):
        self.size = size
        self.robots = []
        self.reached_goals = []  # has the population of the black dots

    def add_robot(self, robot):   # Adds the robot class to the grid
        self.robots.append(robot)

    def can_move(self, robot, position):
        if not (0 <= position[0] < self.size and 0 <= position[1] < self.size):
            return False  # Out of bounds

        # Check if the position is already occupied
        for other_robot in self.robots:
            if other_robot.position == position:
                # Handle rules for occupying the same cell
                if robot.type == 'quadrotor':
                    if other_robot.type == 'quadrotor':
                        return False
                elif robot.type == 'humanoid':
                    if other_robot.type in ['humanoid', 'differential_drive']:
                        return False
                elif robot.type == 'differential_drive':
                    if other_robot.type in ['differential_drive', 'humanoid']:
                        return False
        return True

    def resolve_conflicts(self):
        # Sort robots by distance to goal (descending)
        self.robots.sort(key=lambda r: abs(r.goal[0] - r.position[0]) + abs(r.goal[1] - r.position[1]), reverse=True)

        # Track positions that are being moved to
        target_positions = {}

        for robot in self.robots:
            if robot.position == robot.goal:
                continue  # Skip robots that have reached their goal
                # Remove robots that have reached their goal
                self.robots = [robot for robot in self.robots if robot.position != robot.goal]
            # Calculate the next position
            dx = robot.goal[0] - robot.position[0]
            dy = robot.goal[1] - robot.position[1]
            if abs(dx) > abs(dy):
                new_position = (robot.position[0] + np.sign(dx), robot.position[1])
            else:
                new_position = (robot.position[0], robot.position[1] + np.sign(dy))

            # Check if the move is valid
            if self.can_move(robot, new_position):
                if new_position not in target_positions:
                    target_positions[new_position] = [robot]
                else:
                    target_positions[new_position].append(robot)
                    

        # Resolve conflicts for target positions
        for position, robots in target_positions.items():
            if len(robots) == 1:
                # Only one robot wants to move to this position
                robots[0].position = position
            else:

                # Multiple robots want to move to this position
                robots.sort(key=lambda r: abs(r.goal[0] - r.position[0]) + abs(r.goal[1] - r.position[1]), reverse=True)
                chosen_robot = robots[0]
                chosen_robot.position = position
                for other_robot in robots[1:]:
                    self.attempt_switch()
                    print("called attempt switch")
                      # Attempt to switch positions with adjacent robots
                    pass

    def attempt_switch(self):
        for robot in self.robots:
            if robot.position == robot.goal:
                continue  # Skip robots that have reached their goal

            for other_robot in self.robots:
                if other_robot == robot or other_robot.position == other_robot.goal:
                    continue  # Skip itself or robots already at their goal

                # Check if they are adjacent
                if abs(robot.position[0] - other_robot.position[0]) + abs(robot.position[1] - other_robot.position[1]) == 1:
                    # Check if robots can occupy each other's positions
                    original_rob_1_pos1 = robot.position
                    original_rob_2_pos2 = other_robot.position
                   
                    # check distance from robots to goals after a switch
                    first_robot_to_others_goal = abs(other_robot.goal[0] - robot.position[0]) + abs(other_robot.goal[1] - robot.position[1])
                    other_robots_to_firsts_goal = abs(robot.goal[0] - other_robot.position[0]) + abs(robot.goal[1] - other_robot.position[1])
                    current_distance = first_robot_to_others_goal + other_robots_to_firsts_goal
                   
                    # check potential to self positions distance
                    first_robot_to_firsts_goal = abs(robot.goal[0] - robot.position[0]) + abs(robot.goal[1] - robot.position[1])
                    other_robots_to_others_goal = abs(other_robot.goal[0] - other_robot.position[0]) + abs(other_robot.goal[1] - other_robot.position[1])
                    self_distance = first_robot_to_firsts_goal + other_robots_to_others_goal

                    #check to see if its valid to switch by the rules of the bots
                    if self.can_move(robot, original_rob_2_pos2) and self.can_move(other_robot, original_rob_1_pos1):
                        if self_distance > current_distance:
                            # Revert positions if not beneficial
                            robot.position = original_rob_2_pos2
                            other_robot.position = original_rob_1_pos1
                            print("Switched")

                  

                   

    def update(self):
        # Sort robots by distance to goal (descending)
        self.robots.sort(key=lambda r: abs(r.goal[0] - r.position[0]) + abs(r.goal[1] - r.position[1]), reverse=True)
        for robot in self.robots[:]:  # Iterate over a copy of the list
            robot.move(self)
            if robot.position == robot.goal:
                self.reached_goals.append(robot)  # Add to reached goals list
                self.robots.remove(robot)
    def all_reached_goals(self):
        return len(self.robots) == 0
    
def visualize(grid, timestep):
    plt.figure(figsize=(9, 6))
    ax = plt.gca()
    ax.set_xlim(-0.5, GRID_SIZE - 0.5)
    ax.set_ylim(-0.5, GRID_SIZE - 0.5)
    ax.set_xticks(range(GRID_SIZE))
    ax.set_yticks(range(GRID_SIZE))
    ax.grid(True)
    plt.title(f"Timestep: {timestep}")

    # Draw robots that have reached their goals as black dots
    for robot in grid.reached_goals:
        plt.plot(robot.goal[1], robot.goal[0], 'o', color='black', label='_nolegend_')

    for robot in grid.robots:
        # Draw the goal
        goal_marker = patches.Rectangle((robot.goal[1] - 0.5, robot.goal[0] - 0.5), 1, 1, color='gray', alpha=0.3)
        ax.add_patch(goal_marker)

        # Draw the robot
        color = {'quadrotor': 'blue', 'humanoid': 'green', 'differential_drive': 'red'}[robot.type]
        plt.plot(robot.position[1], robot.position[0], 'o', color=color, label=robot.type)

        # Draw a line with an arrow to the goal
        plt.annotate(
            '',  # No text
            xy=(robot.goal[1], robot.goal[0]),  # Goal position
            xytext=(robot.position[1], robot.position[0]),  # Robot position
            arrowprops=dict(arrowstyle='->', color=color, lw=1.5, alpha=0.7)
        )

    # Add the legend outside the grid
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.tight_layout()  # Adjust layout to prevent clipping
    plt.pause(2)
    #plt.clf()

def main():
    grid = Grid(GRID_SIZE)

    # Initialize robots with random positions and goals
    positions = random.sample([(i, j) for i in range(GRID_SIZE) for j in range(GRID_SIZE)], 20)
    for i in range(10):
        start = positions[i]
        goal = positions[i + 10]
        robot_type = random.choice(ROBOT_TYPES)
        robot = Robot(i, robot_type, start, goal)
        grid.add_robot(robot)

    timestep = 0
    while not grid.all_reached_goals():
        visualize(grid, timestep)
        grid.update()
        timestep += 1

    visualize(grid, timestep)  # Final state
    plt.show()  # Keep the final figure open
    return timestep  # Return the timestep value

if __name__ == "__main__":
    timestep = main() 

    print("All robots reached their goals in", timestep, "timesteps.")# Print the final timestep value


