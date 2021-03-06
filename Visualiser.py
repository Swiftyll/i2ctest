from Tkinter import *
from course import *
import math

### Use the Visualiser class to create a window that shows a graphical
### representation of what the chariot ought to be doing. 
class Visualiser:

    ### (Constructor) Initialises the visualiser. Creates a gui window and starts the loop.
    def __init__(self,c,a):
        self.xoffset = 20
        self.yoffset = 10
        self.chariot = c
        self.course = a
        self.canvas_width = self.course.width
        self.canvas_height = self.course.height
        self.root = Tk()
        self.root.wm_title("Rampyge Visualiser")
        self.canvas = Canvas(self.root, width=a.width, height=a.height)
        self.canvas.pack()
        self.root.after_idle(self.paint)
        mainloop()

    ### Redraw the contents of the window.
    def paint(self):
        self.canvas.delete("all")
        self.canvas.create_rectangle(0,0,self.canvas_width,self.canvas_height,\
                                     fill="#DDDDDD")
        points = []
        for w in self.course.walls:    #Points for background polygon
            points += (w.x1+self.xoffset),(w.y1+self.yoffset)

        self.canvas.create_polygon(points, fill="#408040",outline='black') #Background

        for w in self.course.waypoints:
            tl = w[0]-2 , self.canvas_height-w[1]-2
            br = w[0]+2, self.canvas_height-w[1]+2
            self.canvas.create_oval(tl,br,fill='red')

        for o in self.course.obstacles:
            if type(o) is Pole:
                self.canvas.create_oval(o.x-3 + self.xoffset, self.canvas_height-(o.y-3) + self.yoffset, \
                                        o.x+3 + self.xoffset, self.canvas_height-(o.y+3) + self.yoffset, fill='white')


        #Rotate and paint the chariot
        self.canvas.create_polygon(self.getChariotPoints(), fill="#FFFF00", outline="black")

        #Place and paint the rangefinder(s)
        self.canvas.create_oval(self.getRangefinderPoints(),fill="red")

        self.root.update_idletasks()
        self.root.after(100,self.paint)


    ### Finds the points required to draw a distance sensor
    def getRangefinderPoints(self):
        rangecx = self.chariot.x + self.chariot.s.offsetx
        rangecy = self.chariot.y + self.chariot.s.offsety
        cx = self.chariot.x
        cy = self.chariot.y
        x0 = rangecx - cx
        y0 = rangecy - cy
        angle = -math.radians(self.chariot.bearing)
        tx= math.cos(angle)*x0-math.sin(angle)*y0
        ty= math.sin(angle)*x0+math.cos(angle)*y0
        tx = int(tx+cx)+self.xoffset
        ty = self.canvas_height-(int(ty+cy)+self.yoffset)
        points = [(tx-2,ty-2),(tx+2,ty+2)]
        return points
        

    ### Function for working points for chariot rectangle based on rotation.
    def getChariotPoints(self):
        v = [self.chariot.x - self.chariot.width/2, self.chariot.x+self.chariot.width/2,\
                  self.chariot.y - self.chariot.length/2, self.chariot.y+self.chariot.length/2]
        points = [(v[0],v[2]),(v[1],v[2]),(v[1],v[3]),(v[0],v[3])]
        angle = -math.radians(self.chariot.bearing)
        #Find center points
        cx = self.chariot.x
        cy = self.chariot.y
        for i in range (0,4): #rotational transform on each vertex
            x0 = points[i][0] - cx
            y0 = points[i][1] - cy
            tx= math.cos(angle)*x0-math.sin(angle)*y0
            ty= math.sin(angle)*x0+math.cos(angle)*y0
            points[i] = (int(tx + cx)+self.xoffset,\
                         self.canvas_height - (int(ty+cy)+self.yoffset))
        #print(points)
        return points

        
    
