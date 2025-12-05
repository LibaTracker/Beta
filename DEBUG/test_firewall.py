#!/usr/bin/env python3
"""
test_firewall.py
Testa se a porta UDP está acessível e se o firewall está bloqueando.

Uso:
  python test_firewall.py
"""
import socket
import sys
import json
from pathlib import Path

def load_config():
    """Carregar configuração"""
    config_path = Path("network_config.json")
    if config_path.exists():
        with open(config_path, 'r') as f:
            return json.load(f)
    return {"udp_port": 5005, "listen_address": "0.0.0.0"}

def test_udp_bind(port, address="0.0.0.0"):
    """Testar se consegue fazer bind na porta UDP"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((address, port))
        sock.close()
        return True, "OK"
    except PermissionError:
        return False, "Permissão negada - execute como administrador"
    except OSError as e:
        if "address already in use" in str(e).lower():
            return False, "Porta já está em uso - outro programa está usando"
        return False, f"Erro: {e}"
    except Exception as e:
        return False, f"Erro desconhecido: {e}"

def test_send_receive(port, timeout=2):
    """Testar se consegue enviar/receber na porta"""
    try:
        # Socket de envio
        send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Socket de recepção
        recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        recv_sock.bind(("127.0.0.1", port))
        recv_sock.settimeout(timeout)
        
        # Enviar mensagem de teste
        test_message = b"VRTRACKER_TEST"
        send_sock.sendto(test_message, ("127.0.0.1", port))
        
        # Tentar receber
        try:
            data, addr = recv_sock.recvfrom(1024)
            send_sock.close()
            recv_sock.close()
            return True, "Envio e recepção funcionando"
        except socket.timeout:
            send_sock.close()
            recv_sock.close()
            return False, "Timeout - firewall pode estar bloqueando"
            
    except Exception as e:
        return False, f"Erro no teste: {e}"

def check_windows_firewall():
    """Verificar status do Windows Firewall (Windows apenas)"""
    try:
        import subprocess
        result = subprocess.run(
            ["netsh", "advfirewall", "show", "allprofiles", "state"],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        if "ON" in result.stdout or "Ativado" in result.stdout:
            return True, "Firewall do Windows está ATIVO"
        return False, "Firewall do Windows está DESATIVADO"
    except:
        return None, "Não foi possível verificar (não é Windows ou sem permissões)"

def main():
    print("=" * 60)
    print("  VR TRACKER - TESTE DE FIREWALL E PORTAS")
    print("=" * 60)
    print()
    
    # Carregar configuração
    config = load_config()
    port = config.get("udp_port", 5005)
    address = config.get("listen_address", "0.0.0.0")
    
    print(f"[CONFIG] Porta UDP configurada: {port}")
    print(f"[CONFIG] Endereço de escuta: {address}")
    print()
    
    # Teste 1: Bind na porta
    print("[TESTE 1] Testando bind na porta UDP...")
    success, message = test_udp_bind(port, address)
    
    if success:
        print(f"  ✅ {message}")
    else:
        print(f"  ❌ {message}")
        print()
        print("  SOLUÇÃO:")
        if "já está em uso" in message:
            print("    1. Feche outros programas que possam estar usando a porta")
            print("    2. Execute PARAR_TRACKER.bat")
            print("    3. Tente novamente")
        elif "permissão" in message.lower():
            print("    1. Clique direito neste script")
            print("    2. Executar como administrador")
        print()
    
    # Teste 2: Enviar/Receber
    print("[TESTE 2] Testando envio e recepção na porta...")
    success2, message2 = test_send_receive(port)
    
    if success2:
        print(f"  ✅ {message2}")
    else:
        print(f"  ❌ {message2}")
        if "firewall" in message2.lower():
            print()
            print("  ⚠️ FIREWALL PODE ESTAR BLOQUEANDO!")
            print()
    
    # Teste 3: Windows Firewall
    print()
    print("[TESTE 3] Verificando Windows Firewall...")
    fw_active, fw_msg = check_windows_firewall()
    
    if fw_active is None:
        print(f"  ℹ️  {fw_msg}")
    elif fw_active:
        print(f"  ⚠️  {fw_msg}")
        print()
        print("  SOLUÇÃO para liberar a porta:")
        print("    Execute este comando como ADMINISTRADOR:")
        print()
        print(f'    netsh advfirewall firewall add rule name="VR Tracker UDP" ^')
        print(f'    dir=in action=allow protocol=UDP localport={port}')
        print()
    else:
        print(f"  ℹ️  {fw_msg}")
    
    # Resumo final
    print()
    print("=" * 60)
    print("  RESUMO")
    print("=" * 60)
    
    if success and success2:
        print()
        print("  ✅ TUDO FUNCIONANDO!")
        print("  A porta está acessível e pronta para uso.")
        print()
        print("  Próximos passos:")
        print("    1. Execute INICIAR_TRACKER.vbs")
        print("    2. Acesse http://localhost:8000")
        print()
        return 0
    else:
        print()
        print("  ❌ PROBLEMAS DETECTADOS")
        print()
        if not success:
            print("  • Não foi possível fazer bind na porta")
        if not success2:
            print("  • Firewall pode estar bloqueando")
        print()
        print("  Siga as soluções acima para corrigir.")
        print()
        return 1

if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\nInterrompido pelo usuário.")
        sys.exit(1)