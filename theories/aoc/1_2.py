
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

dial, sum = 50, 0

# lines = inp0.splitlines()
with open("1.txt", "r") as f:
    lines = f.read().splitlines()
    for line in lines:
        line = ''.join(line.split())
        dir, step = line[0], int(line[1:])
        for _ in range(step):
            if dir == "L": dial -= 1
            else: dial += 1
            if dial % 100 == 0: sum += 1

print(sum)

