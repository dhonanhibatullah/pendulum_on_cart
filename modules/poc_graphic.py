import pygame as pg
import numpy as np



class PocGraphic:



    def __init__(self) -> None:
        
        # Constants
        self.WINDOW_TITLE       = 'Inverted Pendulum on Cart'
        self.SCREEN_WIDTH       = 1600
        self.SCREEN_HEIGHT      = 450
        self.SCREEN_CENTER_X    = self.SCREEN_WIDTH//2
        self.SCREEN_CENTER_Y    = self.SCREEN_HEIGHT//2
        self.SCREEN_SCALE       = 34
        self.BG_COLOR           = (0, 0, 0)
        self.FPS                = 60
        self.CART_WIDTH         = 70
        self.CART_HEIGHT        = 32
        self.CART_OFFSET_Y      = self.CART_HEIGHT//2
        self.CART_COLOR         = (255, 165, 117)
        self.POLE_LENGTH        = 160
        self.POLE_RADIUS        = 5
        self.POLE_COLOR         = (0, 255, 0)
        self.JOINT_COLOR        = (120, 0, 153)
        self.WHEEL_RADIUS       = 10
        self.WHEEL_COLOR        = (117, 117, 117)
        self.GROUND_RADIUS      = 5
        self.GROUND_COLOR       = (57, 72, 84)


        # Initiate pygame
        pg.init()
        pg.display.set_caption(self.WINDOW_TITLE)
        self.__screen__ = pg.display.set_mode((self.SCREEN_WIDTH, self.SCREEN_HEIGHT))
        self.__clock__  = pg.time.Clock()


    
    def stepRender(self, cart_x:float, pole_theta:float) -> None:
        
        # Fill screen
        self.__screen__.fill(self.BG_COLOR)


        # Terminate process if window is closed
        for event in pg.event.get():
            if event.type == pg.QUIT:
                pg.quit()
                exit()


        # Get values from arguments
        cart_pos    = int(cart_x*self.SCREEN_SCALE) + self.SCREEN_CENTER_X
        pole_pos    = [cart_pos, self.SCREEN_CENTER_Y + self.CART_OFFSET_Y - self.CART_HEIGHT//2]
        pole_tip    = [pole_pos[0] + int(self.POLE_LENGTH*np.sin(pole_theta)), pole_pos[1] + int(self.POLE_LENGTH*np.cos(pole_theta))]
        cart_rect   = [
            [cart_pos - self.CART_WIDTH//2, self.SCREEN_CENTER_Y + self.CART_OFFSET_Y],
            [cart_pos + self.CART_WIDTH//2, self.SCREEN_CENTER_Y + self.CART_OFFSET_Y],
            [cart_pos + self.CART_WIDTH//2, self.SCREEN_CENTER_Y + self.CART_OFFSET_Y - self.CART_HEIGHT],
            [cart_pos - self.CART_WIDTH//2, self.SCREEN_CENTER_Y + self.CART_OFFSET_Y - self.CART_HEIGHT],
        ]
        pole_rect   = [
            [pole_pos[0] - int(self.POLE_RADIUS*np.cos(pole_theta)), pole_pos[1] + int(self.POLE_RADIUS*np.sin(pole_theta))],
            [pole_pos[0] + int(self.POLE_RADIUS*np.cos(pole_theta)), pole_pos[1] - int(self.POLE_RADIUS*np.sin(pole_theta))],
            [pole_tip[0] + int(self.POLE_RADIUS*np.cos(pole_theta)), pole_tip[1] - int(self.POLE_RADIUS*np.sin(pole_theta))],
            [pole_tip[0] - int(self.POLE_RADIUS*np.cos(pole_theta)), pole_tip[1] + int(self.POLE_RADIUS*np.sin(pole_theta))]
        ]
        cartwheel1_pos  = [cart_pos + self.CART_WIDTH//2 - self.WHEEL_RADIUS, cart_rect[0][1] + self.WHEEL_RADIUS]
        cartwheel2_pos  = [cart_pos - self.CART_WIDTH//2 + self.WHEEL_RADIUS, cart_rect[0][1] + self.WHEEL_RADIUS]
        ground          = [
            [0, cartwheel1_pos[1] + self.WHEEL_RADIUS],
            [0, cartwheel1_pos[1] + self.WHEEL_RADIUS + self.GROUND_RADIUS],
            [self.SCREEN_WIDTH, cartwheel1_pos[1] + self.WHEEL_RADIUS + self.GROUND_RADIUS],
            [self.SCREEN_WIDTH, cartwheel1_pos[1] + self.WHEEL_RADIUS]
        ]


        # Draw the polygons
        pg.draw.polygon(self.__screen__, self.GROUND_COLOR, ground, 0)
        pg.draw.polygon(self.__screen__, self.CART_COLOR, cart_rect, 0)
        pg.draw.circle(self.__screen__, self.WHEEL_COLOR, cartwheel1_pos, self.WHEEL_RADIUS, 0)
        pg.draw.circle(self.__screen__, self.WHEEL_COLOR, cartwheel2_pos, self.WHEEL_RADIUS, 0)

        pg.draw.polygon(self.__screen__, self.POLE_COLOR, pole_rect, 0)
        pg.draw.circle(self.__screen__, self.JOINT_COLOR, pole_pos, self.POLE_RADIUS + 5, 0)
        pg.draw.circle(self.__screen__, self.JOINT_COLOR, pole_tip, self.POLE_RADIUS + 5, 0)
        

        # Update display
        pg.display.update()
        self.__clock__.tick(self.FPS)