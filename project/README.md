# CSC_5RO11_TA, AI for Robotics

## Description
The project wants to create an interactive guessing game with [Pepper Robot](https://us.softbankrobotics.com/pepper) to evaluate user response when tricked.

## Installation

### Requirements

Project was developed in WSL 2 with Ubuntu under VSCode IDLE. Find below a brief requirements explaination:
- Windows:
  - [VSCode](https://code.visualstudio.com/download), Visual Studio Code: programming IDLE;
  - [WSL](https://learn.microsoft.com/en-us/windows/wsl/install), Windows Subsystem for Linux: allow Ubuntu usage inside Windows;
    - [Git](https://git-scm.com/downloads): distributed version control system;
    - [Miniconda](https://docs.anaconda.com/miniconda/miniconda-install/): package management system;
    - [NAOqi](https://www.aldebaran.com/en/support/nao-6/downloads-softwares): Aldebaran's Python API for Pepper;

### Git

As the code was developed in six hands we used [Git](https://git-scm.com/) as our distributed version control system and the project was stored in [GitHub/tr0fin0/ensta_CSC_5RO11_TA](https://github.com/tr0fin0/ensta_CSC_5RO11_TA).

Open VSCode, connect to WSL and open a `bash` terminal. Then run the following to install Git and clone the project repository:

```bash
sudo apt-get install git-all

git clone https://github.com/tr0fin0/ensta_CSC_5RO11_TA CSC_5RO11_TA_project
```

### NAOqi

Download the `SDKs 2.8.6 - Python 2.7 SDK` file as `.tar.gz` from [Aldebaran's Download Softwares](https://www.aldebaran.com/en/support/nao-6/downloads-softwares) site and place it in the `src` folder of the project.

In the `src` folder of the project, run the following command to extract Aldebaran's Python SDK package files:

```bash
tar -xvzf pynaoqi-python2.7-2.8.7.4-linux64-20210819_141148.tar.gz
```

Then run the following command to add the SDK package to Python's environment path:

```bash
export PYTHONPATH=${PYTHONPATH}:/path/to/pynaoqi-python2.7-2.8.7.4-linux64-20210819_141148/lib/python2.7/site-packages
```



### Miniconda

Since Aldebaran's Python SDK package uses `Python 2.7`, Miniconda was used to create an suitable environment.

Then run the following commands to install Miniconda and create a suitable environment:

```bash
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm ~/miniconda3/miniconda.sh

conda create -n CSC_5RO11_TA_project python=2.7
```

#### Activate

Run the following to activate the Miniconda environment:
```bash
conda activate CSC_5RO11_TA_project
```

#### Deactivate

Run the following to deactivate the Miniconda environment:
```bash
conda deactivate
```

## Usage




## Roadmap
This project had incremental goals:
1. k

## Authors and Acknowledment
- [Guilherme TROFINO](mailto:guilherme.trofino@ensta-paris.fr):
  - [![Linkedin](https://i.stack.imgur.com/gVE0j.png) LinkedIn](https://www.linkedin.com/in/guilherme-trofino/)
  - [![GitHub](https://i.stack.imgur.com/tskMh.png) GitHub](https://github.com/tr0fin0)
- [Gianluca BAGHINO GOMEZ](mailto:gianluca.baghino@ensta-paris.fr):
  - [![Linkedin](https://i.stack.imgur.com/gVE0j.png) LinkedIn]()
  - [![GitHub](https://i.stack.imgur.com/tskMh.png) GitHub](https://github.com/gianlucabaghino)
- [Natalia GALLEGO](mailto:natalia.gallego@ensta-paris.fr):
  - [![Linkedin](https://i.stack.imgur.com/gVE0j.png) LinkedIn](https://www.linkedin.com/in/natalia-gallego-castrillon-825a4b208/)
  - [![GitHub](https://i.stack.imgur.com/tskMh.png) GitHub](https://github.com/NataliaGCR)

We greatly appreciate our CSC_5RO11_TA teachers at [ENSTA](https://www.ensta-paris.fr/):
- [Adriana TAPUS](mailto:adriana.tapus@ensta-paris.fr)
- [Adnan SAOOD](mailto:adnan.saood@ensta-paris.fr)
- [Juan Jose GARCIA CARDENAS](mailto:juan-jose.garcia@ensta-paris.fr)
