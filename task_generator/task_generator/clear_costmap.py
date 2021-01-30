import subprocess

def clear_costmaps():
    bashCommand = "rosservice call /move_base/clear_costmaps"
    import subprocess
    process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
    output, error = process.communicate()
    return output, error
