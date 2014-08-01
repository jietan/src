import shlex, subprocess
import sys, os, time
from os import listdir
from os.path import isfile, join

def main():
    MODEL_DIR = 'C:/Users/JieTan/Documents/MyProjects/AdobeIntern/PrincetonModelNet/chair/'
    OUT_DIR = 'C:/Users/JieTan/Documents/MyProjects/AdobeIntern/PrincetonModelNet/chair/'
    folderNames = [ f for f in listdir(MODEL_DIR)]
    #fnames = [ f[:len(f)-4] for f in listdir(MODEL_DIR) if isfile(join(MODEL_DIR,f)) and (f.endswith(".dae"))]
    for sceneName in folderNames:
        fname = MODEL_DIR+sceneName+'/models/untitled.dae'
        print(fname)
        if isfile(fname):
            command_line = 'C:/Users/JieTan/Documents/MyProjects/AdobeIntern/Mitsuba 0.5.0/mtsimport ' + fname + ' ' + OUT_DIR + sceneName + '/' + sceneName + '.xml'
            args = shlex.split(command_line)
            print(args)
            p = subprocess.Popen(args) # Success!
            time.sleep(0.1)

if __name__ == '__main__':
    main()
