# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a test for plotting MatLib with Tkinter 
Using a canvas plotting a graph for voltage data display
"""

import tkinter

#something to get stuff connected to tkinter interface
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg,Na)
from matplotlib.backends.backend_bases import key_press_handler
from matplotlib.figure import Figure 

import numpy as np

root = tkinter.Tk()
root.wm_title("Monkey Brains with tkinter embedding")

fig = Figure(figsize = (5 , 4), dpi = 100)
t = np.arange(0, 3, 0.01)
# certain operations to perform some plot
fig.add_subplot(111).plot(t, 2 * np.sin(2 * np.pi * t))

# show your graph, use a canvas
canvas = FigureCanvasTkAgg(fig, master = root) # A tk.DrawingArea.
canvas.draw()
# positioning canvas
canvas.get_tk_widget().pack(side = tkinter.TOP, fill = tkinter.BOTH, expand = 1)

toolbar = NavagationToolbar2Tk(canvas, root)
toolbar.update()
canvas.get_tk_widget().pack(side = tkinter.TOP, fill = tkinter.BOTH, expand = 1)

def on_key(event):
    print("Voltage control 3.3 V".format(event.key))
    # binding the canvas together with the toolbar
    key_press_handler(event, canvas, toolbar)

canvas.mpl_connect("Display 3.3 Volt Waveform", on_key)

# quitting the window 
def quit():
    root.quit()
    root.destroy()
    
# binding the button
button = tkinter.Button(master = root, text = "Quit", command = quit)
button.pack(side = tkinter.BOTTOM)

tkinter.mainloop()
    

