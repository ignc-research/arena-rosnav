import subprocess

def clear_costmaps():
    bash_command = "rosservice call /move_base/clear_costmaps"
    #import subprocess
    #print(f"executing command {bash_command}")
    #process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
    print("clearing costmap...")
    subprocess.run(bash_command.split())
    print("done...")
    #print(f"bashCommand.split(): {bash_command.split()}")
    #print("returned")
    #output, error = process.communicate()
    #print(f"output {output}, error: {error}")

if __name__ == "__main__":
    clear_costmaps()