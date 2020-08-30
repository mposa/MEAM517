"""
Synchronize homework files (either save files to google drive or copy files from github/google drive) 
"""
from os import path
from shutil import copyfile
import urllib.request
from pathlib import Path

def sync_hw2():
  if 'google.colab' in str(get_ipython()):
    from google.colab import drive
    drive.mount('/content/drive')

    dir = "drive/My Drive/MEAM517_colab/hw2/"
    git_url = "https://raw.githubusercontent.com/yminchen/MEAM517_Colab/test/test/"

    files = ["quadrotor.py", "quadrotor_generator.py", "quad_sim.py", "trajectories.py"]

    # Save files from Colab to Google drive
    if path.exists("quadrotor.py"):
      if not path.exists("drive/My Drive/MEAM517_colab/hw2"):
        # create hw folder in google drive
        Path(dir).mkdir(parents=True, exist_ok=True)
      [copyfile(file, dir + file) for file in files]
    # Read files to Colab
    else:
      # If the hw folder doesn't exist in google drive, create the folder and create new homework scripts
      if not path.exists("drive/My Drive/MEAM517_colab/hw2"):
        # create hw folder in google drive
        Path(dir).mkdir(parents=True, exist_ok=True)
        # download homework scirpts to colab 
        [urllib.request.urlretrieve(git_url + file, file) for file in files]
        # copy the new scripts to hw folder in google drive
        [copyfile(file, dir + file) for file in files]
      # Otherwise, copy the existing homework scirpts from google drive to colab
      else: 
        [copyfile(dir + file, file) for file in files]
