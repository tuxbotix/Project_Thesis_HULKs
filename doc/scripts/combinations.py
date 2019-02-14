def factorial(n):
    if n == 0:
        return 1
    else:
        return n * factorial(n-1)

def range_factorial(m, n):
    if n == 0:
        return 1
    elif n==m:
        return m
    else:
        return n * range_factorial(m, n-1)

print(range_factorial(4,8))

print(factorial(8)/ factorial(8-5))


def unique_unordered_combos(choiceCount, length):
    return range_factorial(choiceCount-length + 1, choiceCount)/factorial(length)

print(unique_unordered_combos(26, 4))
print(unique_unordered_combos(100, 20))