"""
Synchronize homework files (either save files to google drive or copy files from github/google drive) 
"""
from os import path
from shutil import copyfile
import urllib.request
from pathlib import Path

def sync_hw2():
  if 'google.colab' in str(get_ipython()):
    # mount google drive to Colab
    from google.colab import drive
    drive.mount('/content/drive')

    dir = "drive/My Drive/MEAM517_colab/hw2/"
    git_url = "https://raw.githubusercontent.com/mposa/MEAM517/hw2/hw2/"
    files = ["quadrotor.py", "quad_sim.py", "trajectories.py"]

    # 1. Copy files from Colab to Google drive
    if path.exists(files[0]):
      if not path.exists("drive/My Drive/MEAM517_colab/hw2"):
        # create hw folder in google drive
        Path(dir).mkdir(parents=True, exist_ok=True)
      [copyfile(file, dir + file) for file in files]
    # 2. Copy files into Colab
    else:
      # If the hw folder exists in google drive, copy the existing homework files from google drive to colab
      if path.exists("drive/My Drive/MEAM517_colab/hw2"):
        [copyfile(dir + file, file) for file in files]
      # Otherwise, create the folder and create new homework scripts
      else: 
        # create hw folder in google drive
        Path(dir).mkdir(parents=True, exist_ok=True)
        # download homework scirpts to colab 
        [urllib.request.urlretrieve(git_url + file, file) for file in files]
        # copy the new scripts to hw folder in google drive
        [copyfile(file, dir + file) for file in files]
