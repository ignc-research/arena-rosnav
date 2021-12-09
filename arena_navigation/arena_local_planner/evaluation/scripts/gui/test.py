import PySimpleGUI as sg

def new_layout(i):
    return [[sg.T("Question: "), sg.InputText(key=("-q-", i)), sg.T("Answer"), sg.InputText(key=("-ans-", i))]]


category_list = ("General", "Networking", "Socket 1", "Socket 2", "Socket 3", "Socket 4")

column_layout = [
    [sg.Button(enable_events=True, key="addf")]
]

layout = [
    # [sg.T("Category: "), sg.Combo(values=category_list, size=(15, len(category_list)), default_value="General", key="-cat-", enable_events=True),sg.T("Tag: "), sg.InputText(size=(10,1), key="-t-")],
    [sg.Column(column_layout, key='-Column-')],
    # [sg.Submit(button_text="Update/Insert"), sg.Cancel(button_text="Cancel")],
]

window = sg.Window('Question Taker', layout)
i = 1
while True:
    event, values = window.read()
    if event in (sg.WIN_CLOSED, 'Exit', 'Cancel'):
        break
    if event == 'addf':
        if i<5:
            window.extend_layout(window['-Column-'], new_layout(i))
            i += 1
            print(i)
    print(event, values)

event, values = window.read()
window.close()