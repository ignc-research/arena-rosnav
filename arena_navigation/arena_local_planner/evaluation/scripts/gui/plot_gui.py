#!/usr/bin/env python
from matplotlib.ticker import NullFormatter  # useful for `logit` scale
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import PySimpleGUI as sg
import matplotlib
matplotlib.use('TkAgg')

# ------------------------------- PASTE YOUR MATPLOTLIB CODE HERE -------------------------------
#
# # Goal is to have your plot contained in the variable  "fig"
#
# # Fixing random state for reproducibility
# np.random.seed(19680801)
#
# # make up some data in the interval ]0, 1[
# y = np.random.normal(loc=0.5, scale=0.4, size=1000)
# y = y[(y > 0) & (y < 1)]
# y.sort()
# x = np.arange(len(y))
#
# # plot with various axes scales
# plt.figure(1)
#
# # linear
# plt.subplot(221)
# plt.plot(x, y)
# plt.yscale('linear')
# plt.title('linear')
# plt.grid(True)
#
# # log
# plt.subplot(222)
# plt.plot(x, y)
# plt.yscale('log')
# plt.title('log')
# plt.grid(True)
#
# # symmetric log
# plt.subplot(223)
# plt.plot(x, y - y.mean())
# plt.yscale('symlog', linthreshy=0.01)
# plt.title('symlog')
# plt.grid(True)
#
# # logit
# plt.subplot(224)
# plt.plot(x, y)
# plt.yscale('logit')
# plt.title('logit')
# plt.grid(True)
# plt.gca().yaxis.set_minor_formatter(NullFormatter())
# plt.subplots_adjust(top=0.92, bottom=0.08, left=0.10, right=0.95, hspace=0.25,
#                     wspace=0.35)
# fig = plt.gcf()
#


fig = matplotlib.figure.Figure(figsize=(5, 7), dpi=100)

t = np.arange(0, 6, .01)
b = -1.85
c = 2.416
d = -0.25

trj = d*t**3 + c*t**2 + b*t 
fig.add_subplot(111).plot(t, trj)

# ------------------------------- END OF YOUR MATPLOTLIB CODE -------------------------------

# ------------------------------- Beginning of Matplotlib helper code -----------------------

def draw_figure(canvas, figure):
    figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
    figure_canvas_agg.draw()
    figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)
    return figure_canvas_agg

# ------------------------------- Beginning of GUI CODE -------------------------------

# define the window layout
layout = [[sg.Text('Plot test')],
          [sg.Canvas(key='-CANVAS-')],
          [sg.Button('Ok')]]

# create the form and show it without the plot
window = sg.Window('Demo Application - Embedding Matplotlib In PySimpleGUI', layout, finalize=True, element_justification='center', font='Helvetica 18')

# add the plot to the window
fig_canvas_agg = draw_figure(window['-CANVAS-'].TKCanvas, fig)

event, values = window.read()

window.close()