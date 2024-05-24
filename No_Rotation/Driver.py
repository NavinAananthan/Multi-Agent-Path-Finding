#from cbsNew import CBSSolver
import os
import time as timer
import random
from pathlib import Path
import numpy as np
import pygame
import threading
import math
from Prioritized import *

CELL_SIZE = 50
GRID_WIDTH = 10
GRID_HEIGHT = 10
SCREEN_WIDTH = GRID_WIDTH * CELL_SIZE
SCREEN_HEIGHT = GRID_HEIGHT * CELL_SIZE
map_width = 20
map_height = 20
SCREEN = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Grid Movement Simulation")

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0,0,255)
RED = (255, 0, 0)
PURPLE = (128, 0, 128)
TURQUOISE = (48, 213, 200)
GREY = (128, 128, 128)
ORANGE = (255, 165, 0)
YELLOW = (255, 255, 80)
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

# Player properties
PLAYER_SIZE = 20
player_pos = [0, 0]

def draw_grid():
    SCREEN.fill(WHITE)
    for x in range(0, SCREEN_WIDTH, CELL_SIZE):
        pygame.draw.line(SCREEN, BLACK, (x, 0), (x, SCREEN_HEIGHT))
    for y in range(0, SCREEN_HEIGHT, CELL_SIZE):
        pygame.draw.line(SCREEN, BLACK, (0, y), (SCREEN_WIDTH, y))


def get_grid_coordinates(mouse_pos):
    x = mouse_pos[0] // CELL_SIZE
    y = mouse_pos[1] // CELL_SIZE
    return x, y  


def draw_path(path, color):
    print(path)
    for i in range(len(path) - 1):
        print(path[i])
        pygame.draw.lines(SCREEN, color,False, [(path[i][0] * CELL_SIZE + CELL_SIZE // 2, path[i][1] * CELL_SIZE + CELL_SIZE // 2),
                         (path[i + 1][0] * CELL_SIZE + CELL_SIZE // 2, path[i + 1][1] * CELL_SIZE + CELL_SIZE // 2)], 4)
        pygame.display.update()
        pygame.time.delay(500)
        
       
def drawThread(paths,i):
    colors = [PURPLE, TURQUOISE, BLACK, ORANGE, PINK, BROWN, GREY, CYAN, MAGENTA, LIME, NAVY, 
              OLIVE, MAROON, SILVER, GOLD, VIOLET, INDIGO, TEAL, SALMON, CORAL, PLUM, KHAKI, YELLOW]
    draw_path(paths, colors[i%len(paths)])   


map_array = np.zeros((map_height, map_width), dtype=int)

print("Map:")
print(map_array)

startInd=-1
goalInd=-1

starts = []
goals = [] 

colors = [PURPLE, TURQUOISE, BLACK, ORANGE, PINK, BROWN, GREY, CYAN, MAGENTA, LIME, NAVY, 
              OLIVE, MAROON, SILVER, GOLD, VIOLET, INDIGO, TEAL, SALMON, CORAL, PLUM, KHAKI, YELLOW]

max_time = 2*60

running = True

draw_grid()
pygame.display.update()

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            pygame.quit()

        if event.type == pygame.MOUSEBUTTONDOWN:
            mouse_pos = pygame.mouse.get_pos()
            grid_x, grid_y = get_grid_coordinates(mouse_pos)

            # If left mouse button is clicked, mark start node
            if event.button == 1:
                #SCREEN.fill(colors[startInd%len(colors)])
                start_node = (grid_x, grid_y)
                startInd+=1
                pygame.draw.circle(SCREEN, colors[startInd%len(colors)], (start_node[0] * CELL_SIZE+CELL_SIZE//2, start_node[1] * CELL_SIZE+CELL_SIZE//2),CELL_SIZE//2,3)
                starts.append(start_node)

            # If right mouse button is clicked, mark goal node
            elif event.button == 3:
                goal_node = (grid_x, grid_y)
                goalInd+=1
                pygame.draw.rect(SCREEN, colors[goalInd%len(colors)], (goal_node[0] * CELL_SIZE, goal_node[1] * CELL_SIZE, CELL_SIZE, CELL_SIZE))
                goals.append(goal_node)
                
        elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    print("starts:", starts)
                    print('Goals: ', goals)
                    PPS =   PrioritizedPlanningSolver(map_array, starts, goals)
                    paths = PPS.find_solution()
                    print(paths)
                    threads = [threading.Thread(target = drawThread, args=(paths[i],i)) for i in range(len(paths))]

                    for t in threads:
                        t.start()                                

    pygame.display.flip()