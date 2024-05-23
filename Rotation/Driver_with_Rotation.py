import pygame
import sys
import numpy as np
import math
from Prioritized_with_Rotation import *

# Initialize Pygame
pygame.init()

# Define constants
CELL_SIZE = 50
GRID_WIDTH = 10
GRID_HEIGHT = 10
SCREEN_WIDTH = GRID_WIDTH * CELL_SIZE
SCREEN_HEIGHT = GRID_HEIGHT * CELL_SIZE
map_width = 20
map_height = 20
SCREEN = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Grid Movement Simulation")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
TURQUOISE = (48, 213, 200)
RED = (255, 0, 0)
ORANGE = (255, 165, 0)
PINK = (255, 192, 203)
BROWN = (165, 42, 42)
GREY = (128, 128, 128)
CYAN = (0, 255, 255)
MAGENTA = (255, 0, 255)
LIME = (0, 255, 0)
NAVY = (0, 0, 128)
OLIVE = (128, 128, 0)
MAROON = (128, 0, 0)
SILVER = (192, 192, 192)
GOLD = (255, 215, 0)
VIOLET = (238, 130, 238)
INDIGO = (75, 0, 130)
TEAL = (0, 128, 128)
SALMON = (250, 128, 114)
CORAL = (255, 127, 80)
PLUM = (221, 160, 221)
KHAKI = (240, 230, 140)
YELLOW = (255, 255, 80)

colors = [PURPLE, TURQUOISE, BLACK, RED, ORANGE, PINK, BROWN, GREY, CYAN, MAGENTA, LIME, NAVY,
          OLIVE, MAROON, SILVER, GOLD, VIOLET, INDIGO, TEAL, SALMON, CORAL, PLUM, KHAKI, YELLOW]

# Map and paths
map_array = np.zeros((map_height, map_width), dtype=int)
starts = []
goals = []

def draw_grid():
    SCREEN.fill(WHITE)
    for x in range(0, SCREEN_WIDTH, CELL_SIZE):
        pygame.draw.line(SCREEN, BLACK, (x, 0), (x, SCREEN_HEIGHT))
    for y in range(0, SCREEN_HEIGHT, CELL_SIZE):
        pygame.draw.line(SCREEN, BLACK, (0, y), (SCREEN_WIDTH, y))

def draw_paths(paths, timestep):
    for agent_index, path in enumerate(paths):
        color = colors[agent_index % len(colors)]
        for i in range(len(path) - 1):
            if path[i][1] <= timestep:
                start_pos = (path[i][0][0] * CELL_SIZE + CELL_SIZE // 2, path[i][0][1] * CELL_SIZE + CELL_SIZE // 2)
                end_pos = (path[i+1][0][0] * CELL_SIZE + CELL_SIZE // 2, path[i+1][0][1] * CELL_SIZE + CELL_SIZE // 2)
                if path[i+1][1] <= timestep:
                    pygame.draw.line(SCREEN, color, start_pos, end_pos, 5)
                else:
                    break

def draw_agents(paths, timestep):
    for agent_index, path in enumerate(paths):
        color = colors[agent_index % len(colors)]
        for i in range(len(path) - 1):
            if path[i][1] <= timestep < path[i+1][1]:
                pos = (path[i][0][0] * CELL_SIZE + CELL_SIZE // 2, path[i][0][1] * CELL_SIZE + CELL_SIZE // 2)
                next_pos = (path[i+1][0][0] * CELL_SIZE + CELL_SIZE // 2, path[i+1][0][1] * CELL_SIZE + CELL_SIZE // 2)
                draw_arrow(color, pos, next_pos, path[i][1], path[i+1][1], timestep)
                break
            elif timestep >= path[-1][1]:
                pos = (path[-1][0][0] * CELL_SIZE + CELL_SIZE // 2, path[-1][0][1] * CELL_SIZE + CELL_SIZE // 2)
                draw_arrow(color, pos, pos, path[-1][1], path[-1][1], timestep)

    
def draw_arrow(color, pos, next_pos, start_time, end_time, timestep):
    angle = math.atan2(next_pos[1] - pos[1], next_pos[0] - pos[0])
    if end_time - start_time > 1:
        rotation_step = (timestep - start_time) % (end_time - start_time)
        angle_change = math.radians(60 * rotation_step)
        if angle_change != 0:  # Check if angle change is not zero
            angle += angle_change
    body_length = CELL_SIZE // 3
    body_width = CELL_SIZE // 12
    head_size = CELL_SIZE // 6
    pos1 = pos[0]
    pos2 = pos[1]
    body_rect = [
        (pos1 + body_length * math.cos(angle) - body_width * math.sin(angle), pos2 + body_length * math.sin(angle) + body_width * math.cos(angle)),
        (pos1 - body_length * math.cos(angle) - body_width * math.sin(angle), pos2 - body_length * math.sin(angle) + body_width * math.cos(angle)),
        (pos1 - body_length * math.cos(angle) + body_width * math.sin(angle), pos2 - body_length * math.sin(angle) - body_width * math.cos(angle)),
        (pos1 + body_length * math.cos(angle) + body_width * math.sin(angle), pos2 + body_length * math.sin(angle) - body_width * math.cos(angle))
    ]
    head = [
        (pos1 + body_length * math.cos(angle), pos[1] + body_length * math.sin(angle)),
        (pos1 + (body_length + head_size) * math.cos(angle + 2 * math.pi / 3), pos2 + (body_length + head_size) * math.sin(angle + 2 * math.pi / 3)),
        (pos1 + (body_length + head_size) * math.cos(angle - 2 * math.pi / 3), pos2 + (body_length + head_size) * math.sin(angle - 2 * math.pi / 3))
    ]
    pygame.draw.polygon(SCREEN, color, body_rect)
    pygame.draw.polygon(SCREEN, color, head)


def draw_start_and_goal_nodes():
    for startInd, start_node in enumerate(starts):
        pygame.draw.circle(SCREEN, colors[startInd % len(colors)], (start_node[0] * CELL_SIZE + CELL_SIZE // 2, start_node[1] * CELL_SIZE + CELL_SIZE // 2), CELL_SIZE // 2, 3)
    for goalInd, goal_node in enumerate(goals):
        pygame.draw.rect(SCREEN, colors[goalInd % len(colors)], (goal_node[0] * CELL_SIZE, goal_node[1] * CELL_SIZE, CELL_SIZE, CELL_SIZE))

def get_grid_coordinates(mouse_pos):
    x = mouse_pos[0] // CELL_SIZE
    y = mouse_pos[1] // CELL_SIZE
    return x, y

print("Map:")
print(map_array)

startInd = -1
goalInd = -1

running = True
draw_grid()
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            pygame.quit()
            sys.exit()

        if event.type == pygame.MOUSEBUTTONDOWN:
            mouse_pos = pygame.mouse.get_pos()
            grid_x, grid_y = get_grid_coordinates(mouse_pos)

            if event.button == 1:
                start_node = (grid_x, grid_y)
                startInd += 1
                starts.append(start_node)

            elif event.button == 3:
                goal_node = (grid_x, grid_y)
                goalInd += 1
                goals.append(goal_node)
        
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                print("Starts:", starts)
                print("Goals:", goals)
                PPS = PrioritizedPlanningSolver(map_array, starts, goals)
                paths = PPS.find_solution()
                print("Paths in driver:", paths)
                print("Time Taken to compute the paths: ",PPS.CPU_time)

                clock = pygame.time.Clock()
                timestep = 0
                max_timestep = max(max(t[1] for t in path) for path in paths)

                while timestep <= max_timestep:
                    for event in pygame.event.get():
                        if event.type == pygame.QUIT:
                            pygame.quit()
                            sys.exit()

                    draw_grid()  # Clear the screen and redraw the grid
                    draw_start_and_goal_nodes()  # Redraw start and goal nodes
                    draw_paths(paths, timestep)
                    draw_agents(paths, timestep)

                    pygame.display.flip()
                    pygame.time.wait(1000)  # Delay between each timestep

                    timestep += 1

    draw_start_and_goal_nodes()  # Draw start and goal nodes initially
    pygame.display.flip()