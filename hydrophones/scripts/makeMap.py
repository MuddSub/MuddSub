s = ""
while(True):
    name = input("Register Name (or \'done\'): ")
    if name == "done":
        f = open("map.txt", 'w')
        f.write(s)
        print(s)
        break
    address = input("Register Address: ")
    st = "#define {} ({})".format(name, address)
    approve = input("Correct? (y/n): {}".format(st))
    if approve == "y":
        s += st + "\n"
