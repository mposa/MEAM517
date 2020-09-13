# This file is added to reduced the size of setup code in Colab jupyter notebook

curl -s https://raw.githubusercontent.com/mposa/MEAM517/hw2/colab_drake_setup.py > drake_setup.py
python3 drake_setup.py
rm -rf sample_data
rm drake.tar.gz
rm drake_setup.py