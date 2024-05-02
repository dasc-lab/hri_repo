import ast

# String received from the socket
received_string = "(100, 100)"

# Convert the string to a tuple
tuple_data = ast.literal_eval(received_string)

print(tuple_data)  # Output: (100, 100)
print(tuple_data[0])
