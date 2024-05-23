import os
import time as timer
import random
import numpy as np
import pygame
import threading
from CBS import CBSSolver
from Prioritized import *
from pathlib import Path

CELL_SIZE = 100
GRID_WIDTH = 5
GRID_HEIGHT = 5
SCREEN_WIDTH = GRID_WIDTH * CELL_SIZE
SCREEN_HEIGHT = GRID_HEIGHT * CELL_SIZE

# Pygame setup
pygame.init()
SCREEN = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Grid Movement Simulation")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
colors = [
    (128, 0, 128), (48, 213, 200), BLACK, (255, 165, 0), (255, 192, 203), (165, 42, 42),
    (128, 128, 128), (0, 255, 255), (255, 0, 255), (0, 255, 0), (0, 0, 128), (128, 128, 0),
    (128, 0, 0), (192, 192, 192), (255, 215, 0), (238, 130, 238), (75, 0, 130), (0, 128, 128),
    (250, 128, 114), (255, 127, 80), (221, 160, 221), (240, 230, 140), (255, 255, 80)
]

# Player properties
PLAYER_SIZE = 20

# Initialize map and positions
map_array = np.zeros((GRID_HEIGHT, GRID_WIDTH), dtype=int)
starts = []
goals = []

# Draw functions
def draw_grid():
    SCREEN.fill(WHITE)
    for x in range(0, SCREEN_WIDTH, CELL_SIZE):
        pygame.draw.line(SCREEN, BLACK, (x, 0), (x, SCREEN_HEIGHT))
    for y in range(0, SCREEN_HEIGHT, CELL_SIZE):
        pygame.draw.line(SCREEN, BLACK, (0, y), (SCREEN_WIDTH, y))


def get_grid_coordinates(mouse_pos):
    return mouse_pos[0] // CELL_SIZE, mouse_pos[1] // CELL_SIZE


def draw_path(path, color):
    for i in range(len(path) - 1):
        pygame.draw.line(
            SCREEN, color,
            (path[i][0] * CELL_SIZE + CELL_SIZE // 2, path[i][1] * CELL_SIZE + CELL_SIZE // 2),
            (path[i + 1][0] * CELL_SIZE + CELL_SIZE // 2, path[i + 1][1] * CELL_SIZE + CELL_SIZE // 2),
            4
        )
    pygame.display.update()
    pygame.time.delay(1000)


def draw_paths(paths):
    for i, path in enumerate(paths):
        threading.Thread(target=draw_path, args=(path, colors[i % len(colors)])).start()


def main():
    running = True
    startInd = -1
    goalInd = -1

    draw_grid()
    pygame.display.update()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.MOUSEBUTTONDOWN:
                mouse_pos = pygame.mouse.get_pos()
                grid_x, grid_y = get_grid_coordinates(mouse_pos)

                if event.button == 1:  # Left mouse button
                    start_node = (grid_x, grid_y)
                    startInd += 1
                    pygame.draw.circle(
                        SCREEN, colors[startInd % len(colors)],
                        (start_node[0] * CELL_SIZE + CELL_SIZE // 2, start_node[1] * CELL_SIZE + CELL_SIZE // 2),
                        CELL_SIZE // 2, 3
                    )
                    starts.append(start_node)

                elif event.button == 3:  # Right mouse button
                    goal_node = (grid_x, grid_y)
                    goalInd += 1
                    pygame.draw.rect(
                        SCREEN, colors[goalInd % len(colors)],
                        (goal_node[0] * CELL_SIZE, goal_node[1] * CELL_SIZE, CELL_SIZE, CELL_SIZE)
                    )
                    goals.append(goal_node)

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    print("Starts:", starts)
                    print("Goals:", goals)
                    # cbs = CBSSolver(map_array, starts, goals)
                    # paths = cbs.findSolution(disjoint=True)
                    PPS = PrioritizedPlanningSolver(map_array,starts,goals)
                    paths = PPS.find_solution()
                    draw_paths(paths)

        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()
