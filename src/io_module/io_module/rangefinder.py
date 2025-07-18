import socket
import tkinter as tk
from tkinter import messagebox

def tcp_client():
    server_address = '192.168.1.100'
    port = 64000
    message = 'M0\r\n'

    try:
        # 建立 socket 並連線
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((server_address, port))

            # 發送資料
            s.sendall(message.encode('ascii'))
            messagebox.showinfo("TCP Client", f"Sent to server: {message.strip()}")

            # 接收資料
            received_data = s.recv(256)
            received_message = received_data.decode('ascii')
            messagebox.showinfo("TCP Client", f"Received from server: {received_message.strip()}")

    except Exception as e:
        messagebox.showerror("TCP Client", f"Connection Error: {e}")

def main():
    root = tk.Tk()
    root.withdraw()  # 不顯示主視窗，只彈出 messagebox
    tcp_client()

# 測試主程式
if __name__ == '__main__':
    main()