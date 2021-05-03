import PySimpleGUI as sg 

# --------------------------- add stuff dynamically
# num_buttons = 2
# layout = [[sg.Text('Your typed chars appear here:'), sg.Text('', key='_OUTPUT_')],
#             [sg.Input(do_not_clear=True, key='_IN_')],
#             *[[sg.Button('Button'),] for i in range(num_buttons)],
#             [sg.Button('Add Rows'), sg.Button('Delete Rows' ), sg.Button('Exit')]]

# location = (600,600)
# window = sg.Window('Window Title', location=location).Layout(layout)


# num_buttons = 2
# while True:             # Event Loop
#     event, values = window.Read()
#     print(event, values)
#     if event is None or event == 'Exit':
#         break
#     if event == 'Add Rows' or event == 'Delete Rows':
#         num_buttons +=  -2 if event == 'Delete Rows' else 2

#         layout = [[sg.Text('Your typed chars appear here:'), sg.Text('', key='_OUTPUT_')],
#                     [sg.Input(do_not_clear=True, key='_IN_')],
#                     *[[sg.Button('Button'),] for i in range(num_buttons)],
#                   [sg.Button('Add Rows'), sg.Button('Delete Rows'), sg.Button('Exit')]]
#         window1 = sg.Window('Window Title', location=location).Layout(layout)
#         window.Close()
#         window = window1

# window.Close()

# --------------------------- toggle sections
#!/usr/bin/env python
from matplotlib.ticker import NullFormatter  # useful for `logit` scale
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib
matplotlib.use('TkAgg')

def append_layout(i):

    kid = "_" + str(i)
    default_cfg_layout = [[sg.Text('Decide settings for all figures', key = 'txt'       + kid)], 
        [sg.Button('bags',                                            key = "bags"      + kid)],
        [sg.Checkbox('trajectory ',  default=True, size=(9,1),        key = "cb_trj"    + kid), 
         sg.Checkbox('global plan',  default=True,                    key = "cb_gp"     + kid), 
         sg.Checkbox('obstacles',    default=True, size=(9,1),        key = "cb_obst"   + kid), 
         sg.Checkbox('collisions',   default=True, size=(9,1),        key = "cb_cols"   + kid)],
        [sg.Checkbox('static map ',  default=True, size=(9,1),        key = "cb_sm"     + kid), 
         sg.Checkbox('zones',        default=True, size=(9,1),        key = "cb_zones"  + kid), 
         sg.Checkbox('subgoal',      default=True, size=(9,1),        key = "cb_sg"     + kid)]]

    section = [[sg.TabGroup([[sg.Tab('Plot Cfg', default_cfg_layout,  key = "cfg"       + kid)]], key = kid)],
               [sg.Canvas(                                            key = 'CANVAS'    + kid)]]

    print(type(section))
    
    toggled = [Collapsible(section, "section" + kid,  'Fig' + kid, collapsed=True)]
    return section

def draw_plot(i):
    fig_canvas_agg = draw_figure(window['CANVAS_' + str(i)].TKCanvas, fig)



def draw_figure(canvas, figure):
    figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
    figure_canvas_agg.draw()
    figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)
    return figure_canvas_agg

fig = matplotlib.figure.Figure(figsize=(5, 7), dpi=100)

t = np.arange(0, 6, .01)
b = -1.85
c = 2.416
d = -0.25

trj = d*t**3 + c*t**2 + b*t 
fig.add_subplot(111).plot(t, trj)

def Collapsible(layout, key, title='', arrows=(sg.SYMBOL_DOWN, sg.SYMBOL_UP), collapsed=False):
    """
    User Defined Element
    A "collapsable section" element. Like a container element that can be collapsed and brought back
    :param layout:Tuple[List[sg.Element]]: The layout for the section
    :param key:Any: Key used to make this section visible / invisible
    :param title:str: Title to show next to arrow
    :param arrows:Tuple[str, str]: The strings to use to show the section is (Open, Closed).
    :param collapsed:bool: If True, then the section begins in a collapsed state
    :return:sg.Column: Column including the arrows, title and the layout that is pinned
    """
    print(key)
    return sg.Column([[sg.T((arrows[1] if collapsed else arrows[0]),enable_events=True, k=key+'-BUTTON-'),
                    #    sg.T(title, enable_events=True, key=key+'-TITLE-')]])
                       sg.T(title, enable_events=True, key=key+'-TITLE-')],
                      [sg.pin(layout)]])


SEC1_KEY = '-SECTION1-'

default_cfg_layout = [[sg.Text('Decide settings for all figures')], 
    [sg.Button('bags')],
    [sg.Checkbox('trajectory ', default=True,size=(9,1)), sg.Checkbox('global plan', default=True), 
    sg.Checkbox('obstacles', default=True,size=(9,1)), sg.Checkbox('collisions', default=True,size=(9,1))],
    [sg.Checkbox('static map ', default=True,size=(9,1)), sg.Checkbox('zones', default=True,size=(9,1)), 
    sg.Checkbox('subgoal', default=True,size=(9,1))]]

#pad=((2,0),3)
section1 = [
            [sg.TabGroup([[sg.Tab('Plot Cfg', default_cfg_layout)]])],
            [sg.Canvas(key='CANVAS_0')]]

layout =  [] 

layout =   [
            #### Section 1 part ####
            [Collapsible(section1, SEC1_KEY,  'Fig 1', collapsed=True)],
            #### Section 2 part ####
            [sg.Button('Button1'),sg.Button('add Fig'), sg.Button('Exit')]]

# column_layout = [
#     [sg.Button('add',enable_events=True, key="addf")]
# ]

# layout = [
#     [sg.Column(column_layout, key='-Column-')],
# ]

window = sg.Window('Visible / Invisible Element Demo', layout, size=(430,500), resizable=True)
# draw_plot(0)

i = 0

while True:             # Event Loop
    event, values = window.read()
    print(event, values)
    # draw_plot(0)

    if event == sg.WIN_CLOSED or event == 'Exit':
        break

    elif event.startswith(SEC1_KEY):
        window[SEC1_KEY].update(visible=not window[SEC1_KEY].visible)
        window[SEC1_KEY+'-BUTTON-'].update(window[SEC1_KEY].metadata[0] if window[SEC1_KEY].visible else window[SEC1_KEY].metadata[1])

    elif event == 'addf':
        window.extend_layout(window['-Column-'], append_layout(i))
        i += 1
        print(i)
        # draw_plot(i)
        
        

event, values = window.read()
window.close()