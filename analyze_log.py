#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script de análise para encontrar linhas problemáticas no arquivo de log
"""

import re
import os

def analyze_log_file(input_path):
    """
    Analisa o arquivo para encontrar padrões anormais
    """
    
    print(f"Analisando arquivo: {input_path}\n")
    
    with open(input_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    
    print(f"Total de linhas: {len(lines)}\n")
    
    # Procura por linhas com padrões suspeitos
    # Um registro normal tem formato: int,float,float,int,int,int
    
    # Padrão normal
    normal_pattern = r'^\d+,[\d.]+,[\d.]+,\d+,\d+,\d+$'
    
    # Padrão anormal: números muito grandes ou estrutura quebrada
    suspicious_lines = []
    
    for i, line in enumerate(lines):
        line = line.rstrip('\n')
        
        if not line.strip():
            continue
        
        # Verifica se é um padrão normal
        if not re.match(normal_pattern, line):
            # É uma linha suspeita
            suspicious_lines.append((i+1, line))
            
            if len(suspicious_lines) <= 20:  # Mostra apenas 20 primeiras
                print(f"Linha {i+1} SUSPEITA: {line[:100]}")
                if len(line) > 100:
                    print(f"                     ... (restante: {line[100:]})")
    
    print(f"\nTotal de linhas não-padrão encontradas: {len(suspicious_lines)}")
    
    # Analisa as linhas suspeitas para encontrar o padrão
    print("\n--- Análise de padrões anormais ---\n")
    
    for line_num, line in suspicious_lines[:10]:
        fields = line.split(',')
        print(f"Linha {line_num}: {len(fields)} campos")
        print(f"  Conteúdo: {line}")
        
        # Tenta verificar se é concatenação de dois registros
        # Se temos 12 campos, pode ser 2 registros (6 campos cada)
        if len(fields) == 12:
            print(f"  -> Parece ser 2 registros: {','.join(fields[0:6])} | {','.join(fields[6:12])}")
        elif len(fields) == 11:
            # Pode ser um campo "mordido"
            print(f"  -> Pode ser um campo incompleto ou concatenação parcial")
            print(f"  Campos: {fields}")
        else:
            print(f"  -> Estrutura anormal: {len(fields)} campos em vez de 6")
        print()

if __name__ == '__main__':
    log_file = r'.\Logs\COM5_2026_02_06_11_26_50_755.txt'
    
    if not os.path.exists(log_file):
        print(f"Erro: Arquivo não encontrado: {log_file}")
        exit(1)
    
    analyze_log_file(log_file)
