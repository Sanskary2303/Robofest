#! /usr/bin/env python3

import pygame, rospy, math
from pygame.locals import *
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Polygon

background_color = (0, 0, 0)
line_color = (255, 255, 255)
minor_line_color = (100, 100, 100)
major_line_color = (200, 200, 200)
font_color = (250, 250, 250)
point_index_font_color = (10, 10, 10)
point_index_background_color = (190, 190, 190)
line_width = 3
minor_line_width = 1
major_line_width = 2
WIDTH, HEIGHT = 600, 600
major_x, minor_x, major_y, minor_y = 100, 20, 100, 20
font_size = 20
point_index_font_size = 10
frequency = 1
points = []

polygon_publisher = rospy.Publisher("/polygon", Polygon, queue_size=10)
publish = False

def distance(point_1, point_2):
    return math.sqrt((point_1[0]-point_2[0])**2+(point_1[1]-point_2[1])**2)

def drawGrid(screen, font):
    width, height = screen.get_size()
    for x in range(0, width, minor_x):
        pygame.draw.line(screen, minor_line_color, (x, 0), (x, height), minor_line_width)
    for x in range(0, width, major_x):
        pygame.draw.line(screen, major_line_color, (x, 0), (x, height), major_line_width)
    for y in range(0, height, minor_y):
        pygame.draw.line(screen, minor_line_color, (0, y), (width, y), minor_line_width)
    for y in range(0, height, major_y):
        pygame.draw.line(screen, major_line_color, (0, y), (width, y), major_line_width)
    for x in range(0, width, major_x):
        text = font.render(f"{x//major_x}", True, font_color, background_color)
        textPos = text.get_rect()
        textPos.center = (x + font_size//2, font_size // 2 + major_line_width)
        screen.blit(text, textPos)
    for y in range(0, height, major_y):
        text = font.render(f"{y//major_y}", True, font_color, background_color)
        textPos = text.get_rect()
        textPos.center = (font_size//2, y + font_size // 2 + major_line_width)
        screen.blit(text, textPos)

if __name__ == "__main__":
    rospy.init_node("polygon")
    rate = rospy.Rate(frequency)
    pygame.init()
    logo = pygame.image.load("assets/images/ariitk.jpg")
    screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.RESIZABLE)
    pygame.display.set_caption("Polygon")
    pygame.display.set_icon(logo)
    font = pygame.font.Font('freesansbold.ttf', font_size)
    print("Left Click to Add a Point")
    print("Right Click to Delete the Nearest Point")
    print("Press Backspace on Keyboard to delete the Lastest Added Point")
    print("Press 'C' on Keyboard to delete all the points")
    print("Press 'P' on Keyboard to toggle Publishing")
    print("Points will be Published on /polygon with message type as geometry_msgs/Polygon")
    running = True
    while running and not rospy.is_shutdown():
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                break
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_BACKSPACE:
                    if len(points) > 0:
                        points.pop()
                if event.key == pygame.K_c:
                    points.clear()
                if event.key == pygame.K_p:
                    if publish == False:
                        publish = True
                        print("Started Publishing on /polygon with message type as geometry_msgs/Polygon")
                    else:
                        publish = False
                        print("Stopped Publishing on /polygon with message type as geometry_msgs/Polygon")
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    points.append(pygame.mouse.get_pos())
                if event.button == 3:
                    mouse_position = pygame.mouse.get_pos()
                    point_index = -1
                    width, height = screen.get_size()
                    min_distance = width+height
                    for index, point in enumerate(points):
                        current_distance = distance(point, mouse_position)
                        if current_distance < min_distance:
                            min_distance = current_distance
                            point_index = index
                    if point_index != -1:
                        points.pop(point_index)
        screen.fill(background_color)
        drawGrid(screen, font)
        ros_publisher_points = Polygon()
        for draw_point_index, point in enumerate(points):
            ros_publisher_point = Point32(point[0], point[1], draw_point_index)
            ros_publisher_points.points.append(ros_publisher_point)
            pygame.draw.line(screen, line_color, point, points[(draw_point_index+1)%len(points)], line_width)
            pygame.draw.circle(screen, point_index_background_color, point, int(point_index_font_size*1.2))
        for draw_point_index, point in enumerate(points):
            text = font.render(f"{draw_point_index+1}", True, point_index_font_color, point_index_background_color)
            textPos = text.get_rect()
            textPos.center = point
            screen.blit(text, textPos)
        if publish == True:
            polygon_publisher.publish(ros_publisher_points)
        pygame.display.update()