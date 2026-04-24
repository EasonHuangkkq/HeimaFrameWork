#!/usr/bin/env python3
import socket
import sys

def print_usage():
    print("Usage: python3 udp_cmd.py <alias|all> <position_in_rad>")
    print("Example (Move Alias 5 to 0.5 rad): python3 udp_cmd.py 5 0.5")
    print("Example (Move ALL motors to 0):    python3 udp_cmd.py all 0.0")

def main():
    if len(sys.argv) != 3:
        print_usage()
        sys.exit(1)

    alias_str = sys.argv[1]
    pos_str = sys.argv[2]
    
    # 简单的格式验证
    if alias_str.lower() != "all" and not alias_str.isdigit():
        print("Error: <alias> must be an integer, or 'all'.")
        print_usage()
        sys.exit(1)

    try:
        float(pos_str)
    except ValueError:
        print("Error: <position_in_rad> must be a valid float.")
        print_usage()
        sys.exit(1)

    msg = f"{alias_str.lower()}:{pos_str}"

    try:
        # Create UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Send string message to 127.0.0.1:8888
        sock.sendto(msg.encode('utf-8'), ("127.0.0.1", 8888))
        print(f"成功发送控制指令: '{msg}' 给本机 127.0.0.1:8888")
    except Exception as e:
        print(f"UDP发送失败: {e}")

if __name__ == "__main__":
    main()
