import asyncio
import socket

# Global variable to store the message received through socket
message_content = None

async def check_message():
    global message_content
    # Create a socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('35.3.226.65', 9999))  # Bind to localhost and port 9999 (adjust as needed)
    sock.listen(1)
    client_socket,addr = sock.accept()

    #sock.setblocking(False)
    while True:
        try:
            # print("before data")
            data = client_socket.recv(1024)  # Receive data from the socket
            message_content = data.decode("utf-8")
        except socket.error as e:
            print(f"Error receiving message: {e}")
        
async def print_continuous():
    global message_content
    while True:
        if message_content:
            print(message_content)  # Print the content of the message
        else:
            print("here")
        await asyncio.sleep(0.5)  # Print every 0.5 seconds

async def main():
    task1 = asyncio.create_task(print_continuous())
    task2 = asyncio.create_task(check_message())
    await asyncio.gather(task1, task2)

asyncio.run(main())
