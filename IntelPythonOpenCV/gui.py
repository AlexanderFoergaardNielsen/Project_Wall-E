import PySimpleGUI as sg
#GUI setup
relevantpath = False

sg.theme('DarkBlack')

selectMission = (
    'Follow path', 'Sweep area')

width = max(map(len, selectMission))+1

font = ("Arial", 18)


layout = [
    [sg.Text("Select mission", font=font)], [sg.Combo(selectMission, size=(width, 5), font=font, enable_events=True, key='SBCC')],
    [sg.Button('Confirm', key='SI', font=font, visible=True),
     sg.Cancel('Exit', font=font, key='EXIT')]
]

window = sg.Window("Litter Robot Controller", layout, margins=(50, 20), finalize=True)

#Main loop for GUI

while True:
    event, values = window.read()
    #print(event, values)

    if event == sg.WINDOW_CLOSED:
        break

    if event == 'EXIT':
        break

    if event == 'SI':
        if values['SBCC'] == "":
            sg.popup_error(f'Choose a mission please!', font=font)
        else:
            print('Initialising Programme...')
            #launch init script
            print('Initialisation done!')
            if values['SBCC'] == "Follow path":
                choose_mission = "F"
            elif values['SBCC'] == "Sweep area":
                choose_mission = "S"
            break
print(choose_mission)
window.close()