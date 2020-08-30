"""
Synchronize homework files (either save files to google drive or copy files from github/google drive) 
"""
import os
from os import path
from shutil import copyfile
import urllib.request

def sync_hw2():
  if 'google.colab' in str(get_ipython()):
    from google.colab import drive
    drive.mount('/content/drive')

    dir = "drive/My Drive/MEAM517_colab/hw2/"
    git_url = "https://raw.githubusercontent.com/yminchen/MEAM517_Colab/"

    # Save files from Colab to Google drive
    if path.exists("quadrotor.py"):
      if not path.exists("drive/My Drive/MEAM517_colab/hw2"):
        # create hw folder in google drive
        # !mkdir drive/My\ Drive/MEAM517_colab/hw2
        os.mkdir('drive/My Drive/MEAM517_colab/hw2')
      copyfile("quadrotor.py",  dir + "quadrotor.py")
      copyfile("quadrotor_generator.py", dir + "quadrotor_generator.py")
      copyfile("quad_sim.py", dir + "quad_sim.py")
      copyfile("trajectories.py", dir + "trajectories.py")
    # Read files to Colab
    else:
      # If the hw folder doesn't exist in google drive, create the folder and create new homework scripts
      if not path.exists("drive/My Drive/MEAM517_colab/hw2"):
        # create hw folder in google drive
        os.mkdir('drive/My Drive/MEAM517_colab/hw2')
        # download homework scirpts to colab 
        urllib.request.urlretrieve(git_url + "test/test/quadrotor.py", "quadrotor.py")
        urllib.request.urlretrieve(git_url + "test/test/quadrotor_generator.py", "quadrotor_generator.py")
        urllib.request.urlretrieve(git_url + "test/test/quad_sim.py", "quad_sim.py")
        urllib.request.urlretrieve(git_url + "test/test/trajectories.py", "trajectories.py")
        # copy the new scripts to hw folder in google drive
        copyfile("quadrotor.py",  dir + "quadrotor.py")
        copyfile("quadrotor_generator.py", dir + "quadrotor_generator.py")
        copyfile("quad_sim.py", dir + "quad_sim.py")
        copyfile("trajectories.py", dir + "trajectories.py")
      # Otherwise, copy the existing homework scirpts from google drive to colab
      else: 
        copyfile(  dir + "quadrotor.py", "quadrotor.py")
        copyfile( dir + "quadrotor_generator.py", "quadrotor_generator.py")
        copyfile( dir + "quad_sim.py", "quad_sim.py")
        copyfile( dir + "trajectories.py", "trajectories.py")