#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script para corrigir registros concatenados
Detecta linhas com exatamente 11 campos (2 registros concatenados)
Tenta todas as combinações possíveis de divisão do campo 5
"""

import os

def fix_concatenated_v4(input_path, output_path=None):
    """
    Corrige linhas com exatamente 11 campos
    Mais flexível na busca pela melhor divisão
    """
    
    if output_path is None:
        output_path = input_path.replace('.txt', '_final.txt')
    
    print(f"Lendo arquivo: {input_path}")
    
    with open(input_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    
    print(f"Total de linhas: {len(lines)}\n")
    
    fixed_lines = []
    fixed_count = 0
    
    for i, line in enumerate(lines):
        line = line.rstrip('\n')
        original_line = line
        
        if not line.strip():
            fixed_lines.append(line)
            continue
        
        fields = line.split(',')
        
        # Verifica se tem exatamente 11 campos
        if len(fields) == 11:
            suspicious_field = fields[5]
            
            # Tenta todas as divisões possíveis
            best_split = None
            
            for split_idx in range(1, len(suspicious_field)):
                part1 = suspicious_field[:split_idx]
                part2 = suspicious_field[split_idx:]
                
                if not part1.isdigit() or not part2.isdigit():
                    continue
                
                part1_val = int(part1)
                part2_val = int(part2)
                
                # Heurística básica:
                # - part1 (último campo do primeiro registro) deve ser pequeno: 0-10
                # - part2 (primeira coluna do segundo registro) deve ser razoável: 1-600
                # Preferir divisões onde part1 é menor que part2 (ordem crescente)
                
                if 0 <= part1_val <= 10 and 1 <= part2_val <= 600:
                    # Se já tem candidato, preferir o que tem part1 menor
                    if best_split is None or int(best_split[0]) > part1_val:
                        best_split = (part1, part2)
                    # Ou preferir onde part2 está mais próximo de part1 (menos extremo)
                    elif int(best_split[1]) > part2_val:
                        best_split = (part1, part2)
            
            if best_split:
                part1, part2 = best_split
                
                # Reconstrói os dois registros
                first_record = ','.join(fields[0:5]) + ',' + part1
                second_record = part2 + ',' + ','.join(fields[6:11])
                
                fixed_lines.append(first_record)
                fixed_lines.append(second_record)
                fixed_count += 1
                
                if fixed_count <= 10:
                    print(f"Linha {i+1} CORRIGIDA:")
                    print(f"  Antes: {original_line}")
                    print(f"  Depois:")
                    print(f"    {first_record}")
                    print(f"    {second_record}")
                    print()
                
                continue
        
        # Se não foi corrigida, mantém como está
        fixed_lines.append(line)
    
    print(f"Escrevendo arquivo corrigido: {output_path}")
    with open(output_path, 'w', encoding='utf-8') as f:
        for line in fixed_lines:
            f.write(line + '\n')
    
    print(f"\nResumo:")
    print(f"  Total de linhas lidas: {len(lines)}")
    print(f"  Total de linhas escrevidas: {len(fixed_lines)}")
    print(f"  Linhas corrigidas: {fixed_count}")

if __name__ == '__main__':
    log_file = r'.\Logs\COM5_2026_02_06_11_26_50_755_fixed.txt'
    
    if not os.path.exists(log_file):
        print(f"Erro: Arquivo não encontrado: {log_file}")
        exit(1)
    
    fix_concatenated_v4(log_file)
    print("\nScript concluído!")
