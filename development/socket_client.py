import socket
import sys
import threading

def send_message(sock):
    while True:
        try:
            message = input("Enter your message: ")
            if message.lower() == 'exit':
                print("Exiting...")
                sock.close()
                break
            sock.sendall(message.encode())
        except Exception as e:
            print("Error:", e)
            break

def main():
    #host = '192.168.1.134'  # Change this to your server's IP address or hostname
    host = '35.3.226.65'
    port = 9999         # Change this to your server's port
    
    try:
        # Create a TCP/IP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # Connect the socket to the server
        sock.connect((host, port))
        
        print("Connected to server.")
        
        # Start a thread to handle sending messages
        # send_thread = threading.Thread(target=send_message, args=(sock,))
        # send_thread.start()
        
        # Main thread will handle receiving messages
        while True:
            data = sock.recv(1024)
            if not data:
                print("Disconnected from server.")
                break
            print("\nReceived:", data.decode())
    
    except Exception as e:
        print("Error:", e)
    finally:
        sock.close()

if __name__ == "__main__":
    main()
