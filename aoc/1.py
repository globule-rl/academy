import sys

inp0 = '''L68
L30
R48
L5
R60
L55
L1
L99
R14
L82'''
s, sum = 50, 0

# lines = inp0.splitlines()
with open("1.txt", "r") as f:
    lines = f.read().splitlines()
    for line in lines:
        line = ''.join(line.split())
        instruction, step = line[0], int(line[1:])

        if s > 99: s = s % 100
        if s < 0: s += 100
        if s == 0: sum += 1
        if step > 99: step = step % 100
        if instruction == 'L': s-=step
        else: s+=step
print(sum)

