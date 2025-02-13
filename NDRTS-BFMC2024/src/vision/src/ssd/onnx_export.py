#!/usr/bin/env python3
# Acest script convertește un model salvat PyTorch în format ONNX
import os
import sys
import argparse

import torch.onnx

# Importă funcțiile pentru crearea diferitelor arhitecturi de rețele SSD
from vision.ssd.vgg_ssd import create_vgg_ssd
from vision.ssd.mobilenetv1_ssd import create_mobilenetv1_ssd
from vision.ssd.mobilenetv1_ssd_lite import create_mobilenetv1_ssd_lite
from vision.ssd.squeezenet_ssd_lite import create_squeezenet_ssd_lite
from vision.ssd.mobilenet_v2_ssd_lite import create_mobilenetv2_ssd_lite

from vision.ssd.config import mobilenetv1_ssd_config


# Parsează argumentele din linia de comandă
parser = argparse.ArgumentParser()
parser.add_argument('--net', default="ssd-mobilenet", help="Arhitectura rețelei, poate fi mb1-ssd (aka ssd-mobilenet), mb1-ssd-lite, mb2-ssd-lite sau vgg16-ssd.")
parser.add_argument('--input', type=str, default='', help="Calea către modelul PyTorch de intrare (.pth checkpoint)")
parser.add_argument('--output', type=str, default='', help="Calea dorită pentru modelul ONNX convertit (implicit: <NET>.onnx)")
parser.add_argument('--labels', type=str, default='labels.txt', help="Numele fișierului cu etichetele claselor")
parser.add_argument('--resolution', type=int, default=300, help="Rezoluția NxN a modelului (poate fi schimbată doar pentru mb1-ssd)")
parser.add_argument('--batch-size', type=int, default=1, help="Dimensiunea batch-ului pentru modelul care va fi exportat (implicit=1)")
parser.add_argument('--model-dir', type=str, default='', help="Directorul în care se caută modelul PyTorch de intrare și unde se exportă modelul ONNX (dacă --output nu specifică un director)")

# Parsează argumentele
args = parser.parse_args() 
print(args)

# Setează dispozitivul (GPU sau CPU)
device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
print(f"=> rulând pe dispozitivul {device}")

# Formatează căile pentru modelele de intrare
if args.model_dir:
    args.model_dir = os.path.expanduser(args.model_dir)
    
    # Găsește checkpoint-ul cu cea mai mică pierdere
    if not args.input:
        best_loss = 10000
        for index, file in enumerate(os.listdir(args.model_dir)):
            if not file.endswith(".pth"):
                continue
            try:
                # Extrage pierderea din numele fișierului
                loss = float(file[file.rfind("-")+1:len(file)-4])
                if loss < best_loss:
                    best_loss = loss
                    args.input = os.path.join(args.model_dir, file)
            except ValueError:
                args.input = os.path.join(args.model_dir, file)
                continue    

        if not args.input:
            raise IOError(f"nu s-a găsit un checkpoint .pth valid sub '{args.model_dir}'")
            
        print(f"=> găsit checkpoint-ul cu cea mai mică pierdere {best_loss} ({args.input})")
        
    # Adaugă directorul modelului la calea fișierului (dacă este necesar)
    if not os.path.isfile(args.input):
        args.input = os.path.join(args.model_dir, args.input)
    if not os.path.isfile(args.labels):
        args.labels = os.path.join(args.model_dir, args.labels)

# Determină numărul de clase
class_names = [name.strip() for name in open(args.labels).readlines()]
num_classes = len(class_names)

# Construcția arhitecturii rețelei
print(f"=> crearea rețelei:  {args.net}")
print(f"=> număr clase:       {num_classes}")
print(f"=> rezoluție:        {args.resolution}x{args.resolution}")

# Creează rețeaua bazată pe arhitectura specificată
if args.net == 'vgg16-ssd':
    net = create_vgg_ssd(len(class_names), is_test=True)
elif args.net == 'mb1-ssd' or args.net == 'ssd-mobilenet':
    mobilenetv1_ssd_config.set_image_size(args.resolution)
    net = create_mobilenetv1_ssd(len(class_names), is_test=True)
elif args.net == 'mb1-ssd-lite':
    net = create_mobilenetv1_ssd_lite(len(class_names), is_test=True)
elif args.net == 'mb2-ssd-lite':
    net = create_mobilenetv2_ssd_lite(len(class_names), is_test=True)
elif args.net == 'sq-ssd-lite':
    net = create_squeezenet_ssd_lite(len(class_names), is_test=True)
else:
    print("Tipul de rețea este greșit. Ar trebui să fie unul dintre vgg16-ssd, mb1-ssd sau mb1-ssd-lite.")
    sys.exit(1)
    
print(f"=> încărcarea checkpoint-ului:  {args.input}")

# Încarcă modelul din checkpoint
net.load(args.input)
net.to(device)
net.eval()
# Creează un input fictiv pentru export
dummy_input = torch.randn(args.batch-size, 3, args.resolution, args.resolution).cuda()
# Setează calea fișierului de ieșire dacă nu este specificată
if not args.output:
    args.output = args.net + '.onnx'
# Adaugă directorul modelului la calea fișierului de ieșire (dacă este necesar)
if args.model_dir and args.output.find('/') == -1 and args.output.find('\\') == -1:
    args.output = os.path.join(args.model_dir, args.output)
# Nume pentru input și output
input_names = ['input_0']
output_names = ['scores', 'boxes']
print("=> exportarea modelului la ONNX...")
# Exportă modelul în format ONNX
torch.onnx.export(net, dummy_input, args.output, verbose=True, input_names=input_names, output_names=output_names)
print(f"model exportat la:  {args.output}")
