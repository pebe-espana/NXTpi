#!/usr/bin/env/python
config_filename = '../config/keyboard.txt'

print("---- test of input methods ---")
print("via keyboard: testing input() command")
name = raw_input("what text?")
print(name)

# Get a number
age = int(raw_input("Positive/negative number? "))

# Display the age.
print("number=:", age)


print('now via file input')

# open the sample file used
file = open(config_filename)

# read the content of the file opened
content = file.readlines()
lastLine = len(content)
file.close()
# read 10th line from the file
print("tenth line")
print(content[9])

# print first 3 lines of file
print("first three lines")
print(content[0:3])
print("result:")
print(content[lastLine-1])

line = content[lastLine-1]
letter = line[0:4]
number = int(line[4:7])
print(letter)
print(number)
number2 = int(line[7:10])
print(number2)

print("hope that was successful!!  bye bye")
