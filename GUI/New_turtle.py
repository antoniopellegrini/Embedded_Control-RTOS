import turtle
import time

class Plot:

    def __init__(self):

        self.elapsed_time = 0
        self.Initial_time_angle = 0.0
        self.direction_angle = 0.0
        self.degrees_per_second = 6
        WIDTH, HEIGHT = 0, 400
        self.window = turtle.Screen()
        self.window.tracer(0)
        self.rocket_B1 = turtle.Turtle()
        self.rocket_B2 = turtle.Turtle()
        self.rocket_B1_line = turtle.Turtle()
        self.rocket_B2_line = turtle.Turtle()
        self.line = turtle.Turtle()
        self.timer_text = turtle.Turtle()
        self.timer_text.setposition(WIDTH, HEIGHT)
        self.timer_text.hideturtle()
        self.timer_text.clear()
        #t = turtle.getscreen()
        line_shape = ((0, -1000), (0, 1000))
        rocket_shape = ((-15,15),(-10,15),(-10,20),(-2.5,20),(-2.5,30),(2.5,30),(2.5,20),(10,20),(10,15),(15,15),(15,0),(5,0)
                        ,(5,-5),(15,-5),(15,-10),(20,-10),(20,-20),(15,-20),(15,-15),(10,-15),(10,-10),(5,-10),(5,-15)
                        ,(-5,-15),(-5,-10),(-10,-10),(-10,-15),(-15,-15),(-15,-20),(-20,-20),(-20,-10),(-15,-10),(-15,-5),(-5,-5)
                        ,(-5,0),(-15,0))

        turtle.register_shape('rocket', rocket_shape)
        turtle.register_shape('line', line_shape)
        self.rocket_B1.shape('rocket')
        self.rocket_B2.shape('rocket')
        self.rocket_B1_line.shape('line')
        self.rocket_B2_line.shape('line')
        self.rocket_B1.color('green')
        self.rocket_B1_line.color('green')
        self.rocket_B2.color('black')
        self.rocket_B2_line.color('black')
        self.line.shape('line')
        self.line.color('red')
        # rocket.showturtle()

    def quit_window(self):
        self.window.bye()

    def set_angle(self):
        self.Initial_time_angle = time.time()
        #self.direction_angle = self.Initial_time_angle

    def startDirection(self):

        if self.elapsed_time <= 30:
            self.elapsed_time = abs(self.Initial_time_angle - time.time())
            #print(self.elapsed_time)
            if self.elapsed_time <= 30.0:
                self.direction_angle = self.elapsed_time * self.degrees_per_second




    def update(self, rocket_angle_1, rocket_angle_2, line_angle, time, is_started):

        time = int(time)
        self.rocket_B1.tiltangle(rocket_angle_1)
        self.rocket_B2.tiltangle(rocket_angle_2)
        self.rocket_B1_line.tiltangle(rocket_angle_1)
        self.rocket_B2_line.tiltangle(rocket_angle_2)

        if is_started:

            if time == 0:
                self.set_angle()
                self.timer_text.clear()
                self.timer_text.color('Yellow')
                #self.timer_text.write(f'Mission starts in:\n T: {time}', font=('Courier', 30, "normal"))
                self.timer_text.write(f' T: {time}', font=('Courier', 30, "normal"))

            elif 0 < time <= 30:
                self.startDirection()
                self.timer_text.clear()
                #self.timer_text.write(f'Mission timer:\n T: {time}', font=('Courier', 30, "normal"))
                self.timer_text.write(f' T: {time}', font=('Courier', 30, "normal"))
                self.timer_text.color('Green')

            elif time < 0:
                #self.timer_text.clear()
                self.timer_text.clear()
                self.timer_text.write(f'Mission starts in:\n T: {time}', font=('Courier', 30, "normal"))
                #self.timer_text.write(f' T: {time}', font=('Courier', 30, "normal"))
                self.timer_text.color('Red')

            elif time > 30:
                self.timer_text.clear()
                self.timer_text.write(f'Mission END\n T: {time}', font=('Courier', 30, "normal"))
                # self.timer_text.write(f' T: {time}', font=('Courier', 30, "normal"))
                self.timer_text.color('Green')




        self.line.tiltangle(self.direction_angle)
        #self.timer_text.clear()
        #self.timer_text.write(f'T: {time}', font=('Courier', 30, "normal"))

        if self.elapsed_time < 30.0 and self.elapsed_time > 0:
            #self.timer_text.write(f'T: {self.elapsed_time}', font=('Courier', 30, "normal"))
            pass
        elif self.elapsed_time >= 30.0:
            pass
            #self.timer_text.write(f'T: {self.elapsed_time} \n Mission END', font=('Courier', 30, "normal"))

        self.timer_text.color('Red')
        self.window.update()



